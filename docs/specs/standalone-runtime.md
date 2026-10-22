# Standalone scene runtime

`RigExecStandaloneSystem` evaluates a rig from an owned `RigExecSceneDb` of
resolved values and scene descriptors. It retains no `UsdStage` and does not
run OpenExec. Preparation captures scene descriptors from the database and
translates them with `RigExecTranslateSceneProgram` into a `RigExecSceneProgram`,
which uses the shared operation-graph compiler, readiness rules and kernels
(`libs/rigExecStandalone/sceneRuntime.cpp:4-18`,
`libs/rigExecGraph/sceneProgramTranslation.cpp:648`). The library still links
the OpenUSD libraries and `rigExec`; this is stage independence, not a
USD-free binary dependency tree.

`libs/rigExecStandalone/adapter.{h,cpp}` implements OpenExec's Esf stage,
prim, property, relationship and attribute-query interfaces over
`RigExecSceneDb`. Nothing calls it. See
[OpenExec reference checks](../concepts/baked-vs-dynamic.md#openexec-reference-checks).

Translation covers providers, solvers and their joint bindings, constraints
(the IK/FK match included: its values are public values and its presented
control frames are `RigExecSceneProgram::presentedPoses`), space switches,
auto-clavicles, weights, pose interpolators, and geometry and property movers.
An active
prim of an unknown RigExec type fails preparation
(`libs/rigExecGraph/sceneProgramTranslation.cpp:71-72`). Native USD attributes
are source values. The runtime and [pack exporter/loader](standalone-pack.md)
validate capabilities by preparing the same scene program. A tap phase must
be `base` or `final`. An unknown requested computation fails instead of
producing a successful empty aggregate.

```cpp
#include "rigExecStandalone/pack.h"
#include "rigExecStandalone/system.h"
#include <stdexcept>

std::string error;
auto database = rigExec::RigExecLoadRigPack("character.rigpack", &error);
if (!database) throw std::runtime_error(error);
rigExec::RigExecStandaloneSystem runtime(*database);
int frames = runtime.AddTap(rigExec::RigExecValueAddress::Prim(
    pxr::SdfPath("/Rig/FK"), pxr::TfToken("computePointFrameArray")));
if (!runtime.Prepare(&error)) throw std::runtime_error(error);
auto snapshot = runtime.Evaluate(pxr::UsdTimeCode::Default());
if (snapshot.valid) {
    auto value = snapshot.Get<rigExec::RigExecPointFrameArray>(frames);
    // Inspect the typed frame validity/degeneracy flags as required by the host.
}
```

`RigExecStandaloneResult::valid` means that every requested value was extracted
without an execution error. It has the completeness meaning of the existing
`RigExecSnapshot`; it does not override a typed packet's semantic failure flags.
An empty tap list produces a valid, empty generation. Results own their copied
`VtValue` payloads and survive later evaluation, dirty edits and runtime teardown.

Direct taps on raw attributes, including RigExec avars and custom attributes,
report an explicit no-value state before execution. The check follows a valid
single source connection; an inactive connection target uses the destination's
own value. Explicit `computeResolvedValue` taps read the raw sample and also
report missing raw states. Only the five matrix-space properties on Control and
Joint skip this source check: `default:space`, `avars:defaultSpace`,
`posed:defaultSpace`, `parent:space` and `parent:defaultSpace`. They resolve
through their connection or computed fallback, as described in
[default spaces](xformable-default-spaces.md).

Of the RigExec applied APIs, `RigExecMoverAPI` and `RigExecControlAPI` are
lowered; any other RigExec API fails preparation
(`libs/rigExecScene/sceneProgramTranslation.cpp`). Applied APIs from outside
RigExec, such as physics, pipeline or material schemas, are ignored, as the
native evaluator ignores them.

Only explicitly exported Default, exact numeric and PreTime identities are
accepted. There is no fallback to a nearby row and no custom interpolation or
spline sampler. `EvaluateResolved` can temporarily supply a complete resolved
value/block set for every retained attribute at another identity. It validates
all slots before changing state and restores the prior rows after evaluation.

The host serializes all calls on a runtime. `SetValue` replaces one exported
row with a value of the attribute's exact native type; unchanged values do
nothing. A changed numeric value keeps the compiled
program and counts the requested outputs it reaches through the compiled
graph's readers. A changed structural value, `SetConnections`, `SetTargets`
and `SetPrimActive` recompile on the next preparation. Connection updates
require retained source attributes; RigExec scalar attributes permit at most
one source. Durable edits, new objects and new schema descriptors belong in
the USD authoring scene and a new export. The public database is copied at
construction.

`testRigExecStandalone` compares the standalone runtime with `RigExecTapSet`
OpenExec requests on the source stage at Default, numeric and PreTime
identities, including native geometry arrays and blend channels. It also
exercises production domains (property and geometry movers, read phases),
repeated rest edits, unrelated-source dirty isolation, relationship and
connection rewiring, type/cardinality validation, activity removal/revival,
complete transient states, unsupported requests and retained snapshots after
source-stage destruction. `testRigExecPack` covers the producer/loader
boundary.
