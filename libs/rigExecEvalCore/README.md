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

`RunGeometryStep` and `RunValueStep` are templates instantiated against a host
that binds storage and math at compile time — nested `Math`, `Inputs` and
`State`, with `inputs()`, `state()` and `ranges()`. They are not a vtable, and
they live in the family and chain headers that instantiate them, not here.
Nothing calls them yet: this archive carries the contract, not an executor, and
no engine-identity flag or serialized record selects between paths.

Kind ordinals come from `rigExecBinary/rigexec.fbs`, which declares its
`StepKind` as `RigExecBakedStepKind` in the same order; `SnapshotFinals` (8)
stays reserved. `StepKindName` reads the name from the generated schema, so a
kind is spelled once.

EvalCore links only `rigExecGraph`. It uses the USD-free `rigExecMath` headers
without linking the `rigExecMath` archive, which publicly carries `arch tf gf
vt`; `bin/check_runtime_usd_free.sh` compiles every source listed in
`build/evalCoreSources.txt` with no OpenUSD include path and fails if any of
them needs one.
