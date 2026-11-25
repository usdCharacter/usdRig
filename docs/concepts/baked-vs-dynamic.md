---
title: Evaluation and independent checks
summary: One production operation graph for native, frozen and runtime evaluation, with optional independent judges.
order: 20
---

RigExec compiles a rig into one operation graph. Native evaluation reads scene
inputs at the owning-thread sampling boundary, then executes operations over
typed values. Frozen evaluation samples those inputs first and runs detached
state through the same graph. A `.rigexec` binary supplies compiled declarations
and values to the runtime. There is no evaluator mode selector or fallback walk.

## Compilation and execution

A **compiled rig** is the operation graph and bindings produced by rig
compilation. Control changes and animation frames reuse that compiled rig.
Structural edits, such as reconnecting a mover or changing mesh topology,
require recompilation and produce a new **compiled rig version**. Topology
and dependency bindings remain fixed for each compiled rig version.

The **compiled rig signature** identifies its structural bindings for cache
and validity checks. A worker snapshot holds the compiled rig version it was
created from, so later structural edits cannot change the program it runs.
C++ callers read the structural signature with `GetCompiledRigSignature()`;
Python callers use `Rig.compiled_rig_signature()`.

Each operation declares its input and output values. The shared compiler binds
reads to exact producers, computes a canonical dependency order, and translates
clusters without introducing extra causal dependencies. Descriptor emission
order does not require an input producer to be emitted before its consumer.
Changed input values activate their downstream operations; unchanged outputs
remain available to later generations. Serial and parallel execution use the
same graph and readiness rules.

Structural scene edits rebuild the compiled declarations. Cyclic operations
are reported by canonical loop keys and set aside; their outputs are cleared so
a later or held generation cannot reuse stale cyclic results. Unsupported
source declarations or failed compilation produce an invalid pose with a
reason, rather than selecting a second evaluator.

Step execution itself is shared. `rigExecGraph` schedules one step at a time;
`rigExecEvalCore` then executes that one already-scheduled step through
`RunGeometryStep` for the revision chain and `RunValueStep` for the pose,
weight, space and property families. The baked program and the `.rigexec`
runtime are front-end adapters over that core: each translates its own step
record into a `StepRef`, binds its captured or decoded inputs and typed
stores through a compile-time host, runs the one body, and publishes through
its own generation fence. Neither adapter keeps a second CPU step switch or
starts a traversal of its own, so native, frozen and playback evaluation
reach the same arithmetic decision for the same facts; only input binding,
diagnostics wording and publication differ. The parity inventory in
[Evaluation core parity](../specs/eval-core-parity.md) records, per
operation, which body is canonical and which differences stay with a named
front end.

## Exact floating point

Every translation unit that instantiates the kernels compiles with the
settings named by `cmake/rigexec_flags.cmake`: `/fp:precise` on MSVC,
`-fno-fast-math -ffp-contract=off` elsewhere. Without them the compiler fuses
`a*b+c` into an FMA and the baked, frozen and `.rigexec` results drift an ulp
from each other and from the goldens. That fragment installs under
`lib/cmake/rigExec`, and the same flags install as `exactFpFlags.txt` beside
the USD-free source and include manifests in the SDK's data directory
(`share/rigExec` by default), the manifests with paths relative to the
checkout. One directory supplies the whole set -- the four manifests and
`exactFpFlags.txt` -- so a host building those sources with its own tool
reads the flag list and the file list from one installed layout instead of
copying them.

## The installed boundary

The installed SDK ships `rigExec/api/` and the internal headers that
`tools/rigexec_installed_internal.txt` lists because installed headers still
include them; nothing else under `rigExec/`. A host reads a compiled rig
through `RigExecProgramView` (`api/programView.h`), which lends the standing
program read-only and answers its queries (cluster count, last evaluated time).
The builder state (`compiledProgram.h`, `bakedProgramImpl.h`,
`bakedSchedule.h`, `bakedOpGraph.h`, `frozenProgram.h`,
`frozenContextInternal.h`) and the op core's program and compile records stay
out of the install; internal access to them goes through
`RigExecBakedProgramAccess`. Detached replay freezes a copy of that program for
a worker, and the snapshot boundary drops the scratch those records carry
before the job runs. `tests/testRigExecInstalledSeal.cpp` builds an SDK-only
consumer from the install: it fails if the step graph or an impl type becomes
reachable, if the installed header set differs from `api/` plus that list, or
if an op-core API header does not compile on its own.

## Optional independent judges

`Rig.cpu_reference` enables the independent scalar point reference. Its original
arithmetic consumes captured source facts and explicit current-generation
phase values. The pose reports `reference_agreements` and
`reference_mismatches`; the C++ pose also holds the reference points in
`movedPropertiesCpu`. This check does not select a production engine.

`rigExecPose --cpu-reference` checks that reference coverage matches the
published point chains. The [exec-row goldens](#exec-row-goldens) pin the
values the operation rows store. Exact golden
verification compares published values, status and ordered semantic
diagnostics against independent original captures;
work counters are outside that value contract. CPU point agreement retains its
existing numerical comparison policy and is separate from exact golden checks.

The golden tool supports `--golden-backend native`, `frozen`, or `runtime`.
Runtime visits also require `--golden-program file.rigexec`. Each backend uses
the same explicit first, held, forward, reverse, cold and action visit protocol.
An unsupported checker/backend or interactive/runtime combination fails
explicitly.

## Exec-row goldens

RigExec links no OpenExec library. Native, frozen, frame-cache and
`.rigexec` runtime evaluation all compile and execute the shared operation
graph.

For every compiled step, `RigExecBakedExecCheckRows::Build`
(`libs/rigExecOracle/bakedExecCrossCheckRows.cpp`) records rows: before the
operation body runs, a row captures the values the operation read, and after
it, the value the operation stored. `RigExecBakedProgramTesting::EnableExecRowCapture`
attaches the rows to a native program; the frozen clone omits them. These step
kinds store comparable values:

| Step kind | Row value |
|---|---|
| ComposeSubtree | `computePointFrame`, one row per slot |
| RestCompose | `computeRestFrame`, one row per slot |
| LadderCompose | `computedDefaultSpace`, one row per slot |
| ProviderMatrix, FrameMatrix | `computeMatrix` |
| Solve | `computePointFrameArray` |
| WeightPacket | `computeWeightPacket` |
| VolumePlacements | `computePointFrame`, converted to a volume placement |

Other step kinds are skip-only `NoEquivalent` rows. `testRigExecExecGoldens`
compares the rows of 55 rigs, at three frames each, with the sets checked in
under `tests/fixtures/execRows`; OpenExec judged those sets bit for bit when
they were first recorded. A deliberate change re-records them from the
production evaluator (see [fixture checks](../../tests/fixtures/README.md)).

Aim, rotation and parent constraints in supported configurations are also
judged by an independent C++ witness
(`libs/rigExec/internal/independentConstraintCheck.h`) through
`RigExecBakedProgramTesting::EnableExecCrossCheck`;
`testRigExecExecCrossCheck` runs it over `tests/fixtures/independentConstraints`.
Rows without such a witness are counted as `NoEquivalent` there.

`RigExecValueAddress` and `RigExecValueOverride` are declared in
`libs/rigExec/internal/valueAddress.h` as plain data types. Production uses them
for interactive overrides, falloff tables and input replay.

## Scene and binary playback

`rigExec:asset` selects a `.rigexec` asset in hosts that support binary playback.
Without that asset selection, the host evaluates the stage's rig. This is a
choice of input representation and playback source, not an evaluator policy
mode. Rendering the scene requires no exported bake:

```sh
USD/bin/usdrecord --renderer GL --camera /IkAsset/MainCam \
  --frames 1001:1012 docs/examples/two_bone_ik.usda frame.###.png
```

Pass an explicit camera that covers the evaluated motion.

## Exporting: a stage that needs no plugin

`rigexec.export_baked(stage, rig_paths, times, path)` is a different thing
again. It evaluates the rig at the sample times you name and writes a
**standalone standard USD file**: ordinary animated `points`, and joint,
control and revised-provider frames written as plain local
`xformOp:transform:baked` matrices. Geometry topology, materials, primvars and
metadata survive the flattened copy; joint and control prims become `Xform`,
other execution prims become inert `Scope`s, and the runtime schemas, their
properties and connections into them are removed. The result renders anywhere,
with no RigExec plugin.

```python
import rigexec

baked = rigexec.export_baked(
    stage, ["/Character/Rig"], range(1001, 1050), "character-baked.usdc")
```

You must name **every** active `RigExecRoot` in the stage, so stripping runtime
schemas cannot silently leave another rig unevaluated. The source stage is
never authored, a destination naming a source layer is rejected, and the file is
published by atomic replacement only after every sample succeeds. Treat the
result as a sampled geometry/transform cache: it does not create a new
`UsdSkel` skinning rig, and USD's interpolation between your requested samples
is not a substitute for evaluating nonlinear rig motion at more times.

## Implementation

See [architecture](../specs/spec.md), [frame warming](frame-cache-warming.md),
[bake APIs](../specs/python-bake-inverse.md), and
`tests/testRigExecBakedSchedule.cpp`.

[Godot and baked rigs](tutorial-godot-baked-rig.md) describes a self-contained
`.rigexec` asset with embedded visuals and exposed controllers.
