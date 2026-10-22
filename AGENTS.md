# Working in RigExec

## Project

RigExec evaluates USD character rigs without changing their source stages.
`RigExecRoot` marks a rig. Controls and solvers produce transforms; movers
revise transforms, properties, or geometry in a defined dependency order.
Hydra scene indices publish evaluated data to `usdview` and other consumers.
The root README is the build entry point; `docs/index.md` is the node catalog.

## Repository skills

Skills live in `skills/<skill-name>/SKILL.md`. Read the relevant skill before
starting its workflow, and keep this list current when adding or changing skills.

- [compose-character](skills/compose-character/SKILL.md): Use when composing,
  fitting, or rebuilding a character from `asset_lib/components`, or adapting
  an existing rig to that library. Covers component selection, character fit
  layers, bindings, deformation attachments, assembly, and validation.

## Code map

- `libs/rigExecMath`: numerical kernels. Keep math independent of stage access.
  The archive links `arch`, `tf`, `gf`, and `vt`; it does not read a stage.
  `rigExec`, the scene library, rigging, and the oracle library link it
  privately. The header-only `*Kernel.h` files here also compile into the
  USD-free runtime.
- `libs/rigExecGraph`: the USD-free op scheduler. The archive is
  `opGraph.cpp` only; the other headers here (typed op values, pose and
  provider arithmetic, window history) are header-only and also compile
  into the runtime. Runtime links the archive publicly. `rigExec` and the
  scene library link it privately, so an edit here does not select every
  evaluator test. Do not add a stage or `pxr/` include here.
- `libs/rigExecScene`: USD scene translation, built as `librigExecScene.so`.
  It links `tf`, `gf`, `vt`, `sdf`, `ts`, `usd`, `usdGeom`, and `rigExec`
  publicly; graph and math are private. `librigExec.so` does not link it.
  Imaging, bake, and the standalone adapter link it privately, so playback,
  runtime, and imaging tests load the library without being selected as
  scene tests. The evaluator calls it through the hooks in `sceneDispatch`.
  Installed headers live under `include/rigExecScene/`. Not every file in
  this directory is part of this library: `constraintProgram`,
  `geometryProgram`, `poseProgram`, `propertyProgram`, `providerProgram`,
  `solverProgram`, `weightProgram`, `providerContextBinding`,
  `sceneCompileInputs`, and `typedValues` compile into `librigExec`. Check
  the source lists in `CMakeLists.txt` to see which library an edit changes.
- `libs/rigExecOracle`: test oracles and golden capture, built as
  `librigExecOracle.so`. `librigExec.so` does not link it. The Python
  module links it, and so does each test that runs a cpu-reference,
  golden, or exec cross-check. Imaging, playback, and runtime tests do
  not. The install ships this library because the installed Python module
  loads it.
- `libs/rigExecGpu`: the OpenGL 4.5 compute path for per-vertex deformation
  and cloth, run on a worker thread that owns a hidden context, plus the
  GLSL that the Storm ExtComputation shares. It links `rigExec` publicly and
  `garch` privately; imaging and the Python module link it privately. Its
  tests run the CPU path alone when no GPU device is available.
- `libs/rigExecSampler`: copies stage values into runtime inputs. This is
  the USD boundary in front of playback.
- `libs/rigExecSchema/schema.usda`: source schema. Resources under
  `plugin/rigExecSchema/resources` are generated and checked in; regenerate
  with `bin/gen_schema.sh` or `bin/gen_schema.bat` after schema edits and
  review the diff.
- `libs/rigExec`: evaluator, mover graph, baked program, scheduling,
  invalidation, and frame caching. Preserve dynamic/baked parity.
  `Evaluate` runs that baked program, or returns an invalid pose and the
  `IsBakeable` reasons. It does not evaluate through OpenExec. The tap set
  remains for cross-check and tests.
- `libs/rigExecRigging` and `python`: authoring APIs and Python bindings.
- `libs/rigExecImaging`: scene indices, activation, and display state.
- `libs/rigExecBake`, `libs/rigExecBinary`, `libs/rigExecRuntime`: export,
  binary layout, and playback. Check version compatibility when changing records.
  `rigExecRuntime` links `rigExecBinary` and `rigExecGraph` and no OpenUSD
  library. Do not include a `pxr/` header from a runtime translation unit or
  from a header-only kernel the runtime instantiates (the `rigExecGraph`
  headers and the `rigExecMath` kernel headers it includes).
- `libs/rigExecStandalone`: experimental scene adapter and rigpack path;
  distinct from `.rigexec` binary playback.
- `plugin/rigExecUsdview`: editor UI plus Qt-free interaction models
  (`*Model.py`). `plugin/touchPose` and `plugin/shapeEditor` are further
  `usdview` tools with the same split.
- `plugin/usdNoodles`: bundled third-party node editor; preserve its license.
- `tools`: the `rigExecPose`, `rigExecBake`, and `rigExecReplayInputs`
  command-line programs, and the test selector behind `bin/test_changed.sh`.
- `examples`: sample stages and procedural generators. The biped's eye
  shader keeps its own EULA; see `THIRD_PARTY_NOTICES.md`.
- `tests`: C++ suites, Python model tests, and graphical viewer tests.

## Unified evaluation loop

Native, frozen, and binary runtime evaluation must use the shared operation
graph, dependency compiler, readiness rules, and kernels. Add behavior through
declared inputs, outputs, and producers in that graph. Do not introduce a
second evaluator, fallback walk, private-stage evaluation path, or separate
pre/post pass that computes rig results outside the unified loop. Independent
reference checks remain optional judges, never production fallbacks.

Reuse compiled dependency edges and cluster closures for invalidation and
cache queries; do not rebuild a parallel dependency model or expand all
transitive dependencies on each edit. Preserve exact producer/version binding,
cycle handling, and native/frozen/runtime parity. See
[evaluation and checks](docs/concepts/baked-vs-dynamic.md).

## Rig semantics

- A prim's role comes from its schema and wiring, wherever it sits under
  `RigExecRoot`; a mover is any prim with `rigExec:moves`. `Movers`,
  `Solvers`, `Controls`, `Joints`, and `Weights` scopes are authoring
  conventions. Never key discovery, validation, UI, or tooling on a scope
  name.
- Dependencies decide order. Do not let evaluator structure choose an order
  or which version of a value a consumer reads; a consumer that needs another
  version declares it with `rigExecReadPhase`. An undeclared connection or
  relationship reads `base`.
- Every operation runs in native, frozen, and `.rigexec` evaluation without
  per-evaluator special cases. When one evaluator lacks support, add it there;
  refusing the bake so another path runs is not a fix. Refuse only genuine
  cycles, and refuse them in every evaluator.
- Topology is fixed per compiled rig. Point counts, indices, skin layouts, and chunk
  partitions are compiled once; reconnecting a mover to new geometry
  recompiles. Do not sample, hash, or validate topology per frame.
- A `.rigexec` holds a static graph, its static data, and inputs with
  defaults. Never add per-time records; the runtime computes results rather
  than replaying them.
- Core code never names an external mover. Plugin movers go through the
  generic contract in [external movers](docs/concepts/external-movers.md).
- Never write back to the stage. Compile, evaluation, activation, caching,
  playback and imaging author nothing: no session-layer or edit-target
  opinions, no derived relationship targets or defaults, no cached results.
  A value derived from the scene is resolved at compile into the compiled rig
  as a declared input, the same way in every evaluator; display-only results
  go to Hydra as primvars. Only the authoring APIs (`rigExecRigging`, the
  `usdview` editors, explicit tools on their own stages) write to a stage.

## No-mutex zones and thread ownership

Operation bodies, numerical kernels, and the frozen worker's private arena
are no-mutex zones. Do not add mutexes, lock-taking helpers, shared mutable
caches, or blocking synchronization there. This includes indirect locks from
profiling and logging: collect per-step data locally and publish it at the
existing boundary. Respect the contracts in
[bodyPurity.h](libs/rigExec/bodyPurity.h),
[bakedSchedule.h](libs/rigExec/bakedSchedule.h), and
[frozenContext.h](libs/rigExec/frozenContext.h).

Sample live USD inputs on the owning thread before dispatch. Computation
bodies consume captured facts and declared typed values; workers own their
mutable arenas and must not access the live stage, evaluator, or imaging
registry. Solve sharing through immutable snapshots and explicit ownership,
not by wrapping a live evaluator or stage in a new mutex.

Existing scheduler, cache-publication, and registry locks belong to their
documented coordination boundaries. Preserve their lock order and generation
fences; do not extend their scope into computation or add locks to hide an
ownership violation. Keep authoring and UI callbacks on their owner thread.
A queued UI callback still blocks that thread while it runs: offload only
detached work through the existing worker path, and measure both dispatch and
completion latency when fixing interaction stalls.

### Hidden locks

Compile, digest, baked and frozen steps, and runtime execution run on
parallel lanes. Many ordinary calls take a lock and serialize those lanes:

- Building a `TfToken` from text (`TfToken("name")`, `TfToken(str)`,
  `TfToken::Find`) locks the global token registry. Make tokens
  namespace-scope constants or `TF_DEFINE_PRIVATE_TOKENS`, or build them once
  before parallel work and pass them in. Copying, comparing, and hashing an
  existing token is free.
- `SdfPath` locks a node-table shard when it parses text, creates a node
  (`AppendChild`, `AppendProperty`, `ReplacePrefix`, and similar), or drops
  the last reference to one. `GetString()`, `GetText()`, and `GetAsToken()`
  intern the text under a lock; spell paths with `RigExecPathText` or
  `RigExecSpellPathText` from [pathText.h](libs/rigExec/pathText.h).
- A function-local `static` runs a thread-safe initialization guard. Library
  calls hide some: `TfToken::GetString()` on an empty token reads one, and
  `SdfPath::IsPrimPath()` may.
- `RIGEXEC_PROFILE_SCOPE` takes three mutexes even when recording is off.
  Steps store their own clock reads and the epilogue replays them.
- Posting a Tf diagnostic (`TF_WARN`, `TF_CODING_ERROR`, a failing
  `TF_VERIFY`), `TF_DEBUG` output, and `printf` all lock. On MSVC,
  constructing any iostream, `std::ostringstream` included, takes the global
  locale lock. Collect messages per step and publish them at the boundary.
- `getenv` takes the C runtime's environment lock on Windows, and `TfType`
  lookups take the type registry lock. Read settings once at Build or Open.

Keep per-slot results rather than shared containers on parallel paths.
Before reporting a change, search every added line for these calls and for
`mutex`, `lock`, `call_once`, and `static`. Audit every commit of the task
and anything merged or ported into it, not only the last diff.

## Build and verify

Use an unchanged OpenUSD 26.08 install with OpenExec, CMake 3.26+, Ninja,
C++17, and a compatible Python interpreter. `USD` and `PY` configure helper
scripts; their defaults look for a sibling dependency install.

1. Read relevant code and current changes before editing. Preserve user work.
2. Make the whole set of related changes, then build once. Do not alternate
   one edit, a rebuild and a test run.
3. Before testing, and again before committing, review the whole change
   against this file's rules: unified loop, rig semantics, no-mutex zones,
   hidden locks, USD-free runtime, and documentation hygiene. Fix every
   violation first; do not test or commit code that breaks a rule here.
4. Rebuild only the targets you need, e.g.
   `cmake --build build --target testRigExecOpValues rigExecPose`.
   `bin/build_rigexec.sh` / `bin\build_rigexec.bat` configure, build
   everything, and run the whole ctest suite unless given `--no-test`; use
   them for a fresh checkout only.
5. Run the smallest set of tests that covers the change, capped at 4 minutes
   of wall clock. `bin/test_changed.sh <paths>` prints that set.
   `bin/test_changed.sh --run <paths>` builds the direct executables and
   runs them. The direct set is the inner loop; run the transitive set
   before pushing. Both follow public and interface links, so a private
   dependency stays inside the library that links it. The ctest line uses
   labels: `usd-free` when every selected executable links no OpenUSD
   library, otherwise the project libraries named on those executables'
   own link lines. `-R` keeps the run to the selected tests.
   For a header edit, read the `--explain` output: the selector follows one
   level of `#include`, so a header reached only through another header
   selects nothing. Pass the `.cpp` files that include it as well.
   `ctest --test-dir build -L fast` is the short tier. `ctest -L usd-free`
   runs without a stage. `tests/fixtures/minimal.rigexec` is the checked-in
   program `testRigExecRuntimeFixture` opens.
   `bin/build_rigexec.sh` writes `build/compile_commands.json` and uses
   ccache when it is on `PATH`. `.clangd` reads that database.
   `bin/check_runtime_usd_free.sh` compiles the runtime with no `pxr`
   include path; it calls `gcc` and `g++`, so it does not run under MSVC.
6. Use `bin/test/run_python_tests.sh` or `.bat` only for Qt-free interaction
   changes, and the relevant `bin/test/run_testusdview*` helper only for
   viewport changes. The `execstack` and `ikfk_opacity` helpers need ignored
   `biped_*.usda` stages and exit with "stage not found" in a fresh checkout.
7. Report checks actually run and any unavailable dependency or graphics test.

CI runs on Linux only: a Release build, `ctest -R '^testRigExec' -E
'Cones_|ExampleParity'`, the USD-free runtime check, and an install check.
It does not build on Windows or macOS, and it does not run the Python
bindings or the `usdview` helpers. Run those yourself when a change
touches them.

Run ctest, and any script that imports `pxr` or `rigexec`, from a shell that
sourced `bin/_env.sh` (or called `bin\_env.bat`). It puts this build's USD
and Python first and points `PXR_PLUGINPATH_NAME` at the generated schema and
imaging resources; otherwise another `pxr` or an inherited plugin path can
load instead. A script that builds its own subprocess environment sets
`PXR_PLUGINPATH_NAME` explicitly rather than inheriting it.

Linking rules:

- A new test, bench, or probe that compiles a stage through `rigExec` alone
  joins the `rigExecScene` list near the end of `CMakeLists.txt`. Without
  it, compile prints "RigExec scene library is not linked" and the process
  exits. A test that runs a cpu-reference, golden, or exec cross-check joins
  the `rigExecOracle` list beside it, or exits with "RigExec oracle library
  is not linked".
- A Windows DLL exports only its own objects, so a caller of an archive that
  `rigExec` links privately has nothing to resolve against.
  `cmake/WindowsArchiveLinks.cmake` re-exposes those archives on Windows.
  Keep Windows-only links in that file: the selector reads the root
  `CMakeLists.txt` as text, ignores `if()`, and would apply them to every
  platform. After changing link visibility or splitting a library, build
  every target on Windows (`cmake --build build -- -k 0`); CI does not.

### Windows and Git Bash

- Git Bash and PowerShell have no MSVC environment, so `cmake --build` fails
  as soon as anything compiles. `bin/test_changed.sh --run` and the
  `usdview` helpers rebuild first and then report "build failed". Build with
  `cmd //c 'call bin\_vcvars.bat && cmake --build build --target <targets>'`
  from Git Bash (`cmd /c "..."` from PowerShell), then run the ctest line.
- Git Bash rewrites an argument or environment value that starts with `/`
  into a Windows path: `/Rig/Geo` reaches Python as
  `C:/Program Files/Git/Rig/Geo`, and the prim lookup silently fails. Pass
  prim paths without the leading slash, or put them in a script file. Do not
  set `MSYS_NO_PATHCONV=1` with `_env.sh`; it also stops the conversion of
  the paths `_env.sh` exports.
- Under Git Bash, write `PXR_PLUGINPATH_NAME` entries as `/c/...`, not
  `C:/...`: the drive colon splits the list and the plugin silently fails to
  load.

Do not change tolerances just to make failures disappear. Evaluation must
remain non-authoring; scene edits belong in the authoring layer, and
display-only results go to Hydra as primvars, never back to the stage. Keep
USD row-vector conventions explicit at math boundaries and avoid stage access
inside pure computation callbacks.

## Documentation and release hygiene

Edit node prose in `docs/operator_notes.py` and schema docs, then regenerate
with `python docs/build_pages.py`. It rewrites `docs/nodes/*.md` and
`docs/index.md`; do not edit those by hand. Handwritten guides live in
`docs/concepts` and `docs/specs`. `docs/build_html.py` produces ignored
`docs/site` output. MkDocs publishes `docs/` through `mkdocs.yml` and
`.github/workflows/docs.yml` with `--strict`; its `site/` output is ignored.
Keep Markdown links relative and verify their targets after moving files.

Use short functional comments for contracts, numerical conventions, ownership,
and non-obvious constraints. Omit change diaries, issue transcripts, product
comparisons, and comments that repeat the code. Preserve copyright, license,
source attribution, and tooling directives.

Describe behavior in project terms. Cite public algorithm sources in
`docs/references.md` and link them from affected kernels or guides. Never
replace missing provenance with a claim of originality. Root MIT terms apply
to original project work; third-party material retains its existing terms.

Do not commit credentials, local paths, build trees, research-paper copies,
assistant state, generated HTML, or `docs/sizzle` content. Keep useful current
docs; do not reintroduce archived plans or performance diaries as specifications.

Keep one-off investigation scripts, image prompts, capture manifests, and trial
implementations outside this checkout. A new utility belongs here only if it
has a supported purpose, documented inputs, and dependencies available to a
fresh checkout. Retain real test fixtures and example generators. Check the
Git candidate file list and packaged archives, not only `.gitignore`: ignore
rules do not remove already tracked files or archive members.
