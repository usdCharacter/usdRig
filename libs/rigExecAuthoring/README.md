# rigExecAuthoring

`rigExecAuthoring` is the C++ API that writes RigExec rigs onto a USD stage.
It defines the typed prims, attributes, and relationships the evaluator
discovers -- controls, joints, solvers, constraints, weight objects, poses,
and mover chains -- and nothing else. Compile, evaluation, imaging, bake, and
playback never author; this library, the `usdview` editors, and explicit
tools working on their own stages are the code that writes rigs.

The library builds as a shared library (`rigExecAuthoring.dll` on Windows,
`librigExecAuthoring.so` on Linux). It links `sdf`, `tf`, `gf`, `vt`, and
`usd` publicly and `rigExecMath` and `rigExecScene` privately. Only the
skin-weight generator uses `rigExecScene`, and the `rigExec` it brings, to
compose rests and measure weight fields: authoring a rig neither compiles nor
evaluates it. Installed headers live
under `include/rigExecAuthoring/`. The root [README](../../README.md) is the
build entry point; [CMakeLists.txt](../../CMakeLists.txt) defines the target.

## What it does

| Header | API | Use |
| --- | --- | --- |
| [rigBuilder.h](rigBuilder.h) | `RigExecRigBuilder`, `RigExecMoverChain`, and typed handles such as `RigExecControlHandle`, `RigExecJointHandle`, the solver, constraint, weight, pose, and mover handles | Builds a rig quickly. `RigExecRigBuilder::Create(stage, rigRoot)` defines or reuses a `RigExecRoot`; `Add*` calls define prims and wire their relationships and connections. Every setter authors into the stage's current edit target at once. |
| [generateSkinWeights.h](generateSkinWeights.h) | `RigExecGenerateSkinWeights` | Authors a skin mover's `rigExec:jointIndices`, `rigExec:jointWeights`, and `rigExec:elementSize` from the weight volumes under its influences, measured at their rest placement. |
| [schemaAuthoring.h](schemaAuthoring.h) | `RigExecSchemaPrim` | Strict, schema-backed edits. `Define` or `Get` a prim of one exact registered schema type, then `SetAttribute`, `SetRelationship`, `ClearAttribute`, `ApplyAPI`, or `SetReadPhase`. Property types and variability come from the registered prim definition; an undeclared property or a value of the wrong type throws before anything is authored. |

Both throw `std::invalid_argument` for an invalid request and
`std::runtime_error` when a schema is unavailable or authoring fails.
`RigExecSchemaPrim` needs the generated RigExec schema registered, so run
with the schema resources on `PXR_PLUGINPATH_NAME` (`bin/_env.sh` sets it).

The builder places prims under `Controls`, `Joints`, `Solvers`, `Weights`,
and `Movers` scopes. Those names are authoring conventions only: the
compiler finds a prim by its schema and wiring wherever it sits under
`RigExecRoot`, so a rig authored another way evaluates the same.

## What uses it

| Consumer | Purpose |
| --- | --- |
| Python `_rigexec` module | `rigexec.Builder`, `rigexec.MoverChain`, `rigexec.SchemaPrim`, `rigexec.binding.generate_skin_weights`, and `rigexec.schema.<Type>.define` in [python/rigexec](../../python/rigexec/__init__.py) |
| [examples/python](../../examples/python) | Sample rigs generated with the Python builder |
| `testRigExecRigBuilder`, `testRigExecSchemaAuthoring` | Tests of this library |
| Other C++ tests | Building their test stages |

For worked examples, read [testRigExecRigBuilder.cpp](../../tests/testRigExecRigBuilder.cpp)
and [make_fk_arm.py](../../examples/python/make_fk_arm.py).
