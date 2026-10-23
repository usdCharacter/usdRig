# rigExecScene

`rigExecScene` translates authored USD rig data into typed operations and
value bindings for RigExec's shared operation graph. It resolves solvers,
movers, connections, and read phases into explicit inputs, outputs, and
producers.
Evaluation leaves the source stage unchanged.

The library builds as a separate shared library (`rigExecScene.dll` on
Windows, `librigExecScene.so` on Linux). Installed headers live under
`include/rigExecScene/`. The root [README](../../README.md) is the build
entry point; [CMakeLists.txt](../../CMakeLists.txt) defines source ownership
and linking.

## What it does

1. Reads composed scene facts through `RigExecSceneAccess`; the USD
   implementation is `RigExecUsdSceneAccess`.
2. Captures detached `RigExecSceneDescriptors`, including schema roles,
   attributes, relationships, mover applications, and sampled inputs.
   Source access ends when capture returns.
3. Translates those facts into domain records and binds reads to exact typed
   value versions, including the requested `rigExecReadPhase`.
4. Assembles operations for the shared producer compiler and readiness
   executor in `rigExecGraph`.

`RigExecSceneProgram` holds a typed operation table and compiled graph.
`RigExecSceneProgramRuntime` retains values and operation workspaces and
executes that graph. It uses the common scheduler; operation bodies consume
captured values without querying the source scene.

| Files | Responsibility |
| --- | --- |
| [sceneAccess.h](sceneAccess.h), [usdSceneAccess.h](usdSceneAccess.h) | Scene access contract and USD adapter |
| [sceneDescriptors.h](sceneDescriptors.h) | Detached compiler input |
| `*SceneTranslation.cpp` | Domain-specific translation of scene facts |
| `*GraphBinding.cpp` | Typed operation inputs, outputs, and producers |
| [sceneVersionRoutes.h](sceneVersionRoutes.h), [sceneGraphTypedRead.h](sceneGraphTypedRead.h) | Value-version routing and typed reads |
| [sceneProgramTranslation.cpp](sceneProgramTranslation.cpp), [sceneProgramLayout.h](sceneProgramLayout.h) | Program assembly and value layout |
| [sceneProgram.h](sceneProgram.h) | Compiled program and retained execution workspace |
| [providerRecordExport.cpp](providerRecordExport.cpp) | Provider records used by binary export |
| [registerScene.cpp](registerScene.cpp), [sceneLinkAnchor.cpp](sceneLinkAnchor.cpp) | Load-time registration and linker retention |

## What uses it

| Consumer | Purpose |
| --- | --- |
| `rigExecImaging` | Compiling live rigs for Hydra and `usdview` evaluation |
| `rigExecBake` | Scene compilation and record export for `.rigexec` files |
| `rigExecStandalone` | Experimental scene adapter and rigpack path |
| Python `_rigexec` module | Stage-based rig compilation and evaluation |
| `rigExecPose`, `rigExecBakeTool`, selected tests and benchmarks | Direct stage compilation |

Imaging, bake, and the standalone adapter link `rigExecScene` privately.
The Python module and executables that need it directly name it on their
link lines. Stage-compiling tests are listed near the end of the root
`CMakeLists.txt`.

The evaluator in `rigExec` calls scene functions through
[sceneDispatch.h](../rigExec/sceneDispatch.h). `RigExecRegisterScene()` in
[registerScene.h](registerScene.h) publishes an immutable function table.
The build and installed CMake targets supply `sceneLinkAnchor.cpp` to
consumers so the linker retains the library and registers it when loaded.
Manual linker users call `RigExecRegisterScene()` before compiling or
evaluating, and keep the library loaded for the process lifetime once
registered. Missing scene support returns false from `Compile` or an
invalid pose from `Evaluate`, with a diagnostic in the existing error channel.

`rigExecScene` links `rigExec` publicly; `rigExec` does not link back to
`rigExecScene`. Its other public dependencies are `tf`, `gf`, `vt`, `sdf`,
`ts`, `usd`, and `usdGeom`; `rigExecGraph` and `rigExecMath` are private
dependencies.

## Boundaries with neighboring libraries

| Library | Responsibility |
| --- | --- |
| `rigExec` | Evaluator lifecycle, compiled native/frozen execution, invalidation, and caches |
| `rigExecGraph` | USD-free dependency compiler, scheduling, and readiness rules |
| `rigExecMath` | Numerical kernels independent of stage access |
| `rigExecSampler` | Copying live USD values into binary-runtime inputs |
| `rigExecRuntime` | Executing compiled `.rigexec` programs without OpenUSD or `rigExecScene` |
| `rigExecOracle` | Optional independent reference checks and golden capture |

The directory and library boundaries differ. The following sources in this
directory compile into **`rigExec`**, rather than `rigExecScene`:

- `constraintProgram.cpp`, `geometryProgram.cpp`, `poseProgram.cpp`,
  `propertyProgram.cpp`, `providerProgram.cpp`, `solverProgram.cpp`, and
  `weightProgram.cpp`.
- `providerContextBinding.cpp`, `sceneCompileInputs.cpp`, and `typedValues.cpp`.

Check the CMake source lists before selecting build targets or tests.

## Implementation contracts

Discover roles from schemas and wiring under `RigExecRoot`; scope names
such as `Movers` and `Solvers` are authoring conventions. Bind dependencies
and value versions explicitly, with undeclared reads using `base`.
Capture topology when compiling the rig and reuse it until recompilation.

Native, frozen, and binary evaluation share the operation graph, dependency
rules, and kernels. Extend that graph when adding behavior. Capture live
USD inputs on their owning thread before dispatch; operation bodies and
worker arenas must remain free of stage access and locking.

See [evaluation and checks](../../docs/concepts/baked-vs-dynamic.md) for
execution semantics and [AGENTS.md](../../AGENTS.md) for ownership, build,
and verification requirements.
