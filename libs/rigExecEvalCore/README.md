# EvalCore: one step, two front ends

EvalCore runs a single already-scheduled step of a compiled rig. The op
scheduler in `rigExecGraph` still owns edges, readiness, invalidation and
invocation; the baked program and the `.rigexec` runtime still bind their own
inputs and publish their own outputs. What EvalCore owns is the step identity
and the arithmetic decision, so both front ends make the same choice for the
same facts.

| File | Responsibility |
| --- | --- |
| `step.h` | the step reference and the normalized outcome |
| `kinds.h` / `kinds.cpp` | the step-kind ordinals, reused from the wire schema |
| `host.h` | the compile-time host binding (`Math`, `Inputs`, `State`) and the adapter mapping each front end keeps |
| `envelope.h` | blend-envelope, matrix-kernel and sparse-walk orchestration over the shared memo |
| `skin.h` | skin acceptance, dispatch, blend composition and layout build |
| `chain.h` | `RunGeometryStep` and the revision chain (`ChainInputs` through `Derived`, plus the paired skip switch) |
| `gate.h` | the one group-gate predicate over explicit enabled, proof and packet facts |
| `closure.h` | the step run framing and admission policy both dispatchers run around their routing switch |
| `pose.h` | the pose family's `RunValueStep` bodies (ladder, compose, solvers, constraints, interpolation, harmonics, IK/FK) |
| `weights.h` | weight packet, placement and field-resolution bodies behind `RunValueStep` |
| `spaces.h` | space-expression and provider-refresh bodies behind `RunValueStep` |
| `properties.h` | property-revision bodies behind `RunValueStep` |

`RunGeometryStep` (in `chain.h`) and `RunValueStep` (in `pose.h`,
`weights.h`, `spaces.h` and `properties.h`) are templates instantiated
against a host that binds storage and math at compile time — nested `Math`,
`Inputs` and `State`, with `inputs()`, `state()` and `ranges()`. They are
not a vtable. Both front ends now execute them: the baked program hands CPU
work to `RunGeometryStep` from `RigExecBakedRunGeometryStep` and to
`RunValueStep` from its pose, weight, space and property adapters, and
RigExecRuntime does the same from `RrRunGeometryStep` and its value-step
adapters. Neither wrapper keeps a second CPU step switch after translating
its record into a `StepRef`, and no engine-identity flag or serialized
record selects between paths.

Kind ordinals come from `rigExecBinary/rigexec.fbs`, which declares its
`StepKind` as `RigExecBakedStepKind` in the same order; `SnapshotFinals` (8)
stays reserved. `StepKindName` reads the name from the generated schema, so a
kind is spelled once.

EvalCore links only `rigExecGraph`. It uses the USD-free `rigExecMath` headers
without linking the `rigExecMath` archive, which publicly carries `arch tf gf
vt`; `bin/check_runtime_usd_free.sh` compiles every source listed in
`build/evalCoreSources.txt` with no OpenUSD include path and fails if any of
them needs one.
