# Fixture checks

`cmake/FixtureChecks.cmake` registers the skeleton2d, layerBind and volumeSkin
fixture checks. Every platform runs each fixture with `RIGEXEC_ENABLE_SIMD=0`
and `1`. A native run first checks the independent CPU point reference and
writes a build-local capture. Fresh native, frozen and binary-runtime runs
then compare every encoded bit against it over first, held, forward, reverse
and cold visits. These are backend-parity checks, not independent goldens.
CTest fixtures order capture and bake before their consumers and use separate
files for each fixture and arithmetic mode, including parallel CTest runs.

The checked-in `.golden` files are fixed scalar regression captures for Linux
x86-64, GCC 13 and the Ubuntu 24.04 math library, with unchanged OpenUSD 26.08.
CTest checks them on Linux x86-64 GCC 13 builds with SIMD explicitly disabled.
Other hosts run the same numerical-reference suites and backend-parity checks;
they do not compare their math library's last bits against a foreign capture.
The SIMD skinning path intentionally accumulates floats, while scalar skinning
accumulates doubles; an exact capture cannot be shared between those modes.

The nested-chain capture was aligned to that reference configuration when the
cross-platform checks were added. All float points, keys, validity, diagnostics
and visit metadata stayed unchanged; the largest double-value change was
1.45e-15. The other four captures were already bit-identical with scalar
skinning. No comparison tolerance was changed.

To review a new reference capture in the reference configuration, source
`bin/_env.sh`, set `RIGEXEC_ENABLE_SIMD=0`, and run:

```sh
build/rigExecPose tests/fixtures/skeleton2d/nested_chain.usda \
  --rig /Char/Rig --frames 0,1,2,3,4 --cpu-reference \
  --golden-out /tmp/nested_chain.golden
```

Review the full diff and run the numerical-reference and backend-parity tests
before replacing a checked-in capture. A new native capture alone does not
establish semantic correctness. The independent numerical expectations live in
`testRigExecSkeleton2dKernel`, `testRigExecSkeleton2d`, `testRigExecLayerBind`
and `testRigExecVolumeWeights`; CPU/SIMD comparisons also run in
`testRigExecParallelKernels`.

## Exec-row goldens

`execRows/<reference>/<fixture>.golden` hold, per frame, the rows the baked
program's step bodies store: the row key, a digest of the inputs the step
body read, and the exact value it stored. The sets were first recorded while
OpenExec judged every row bit for bit; OpenExec has since left the engine, and
these goldens stand in for that judge. A frame with more than 512 rows (the
biped stages) is stored as one line per step kind instead: the kind, its row
count and one digest over those rows' keys, input digests and values.
`testRigExecExecGoldens` captures the same rows from the production evaluator
and compares them. `<reference>` names the platform, architecture and
compiler that recorded the file (`RigExecExecRowsReference`). That
configuration compares every bit; any other compares which rows were stored
and the type and length of each value.

The authoritative reference is the Linux configuration above
(`linux-x86_64-gcc13`). Two more exact sets have the same rows and keys as the
Linux set; only the last bits of doubles differ, from the math library:

- `windows-x86_64-msvc1936`, recorded on a Windows development machine: 111
  of the 2286 row lines differ, at most 5.3e-15 absolute.
- `macos-arm64-clang17`, recorded by the macOS CI job: 92 of the 2286 row
  lines differ, at most 1.6e-14 absolute.

Every rig the `example_graph_*` tests run, plus `computed_ik_space` and the
six `independentConstraints` stages, has a golden: `rigexec_add_exec_rows_test`
in `CMakeLists.txt` registers `testRigExecExecGoldens_<name>` with the first,
middle and last of the rig's frames.

A change that alters rows on purpose re-records them from the production
evaluator on each reference configuration, and the commit carries the
reviewed diff. From a shell that sourced `bin/_env.sh`:

```sh
RIGEXEC_RECORD_EXEC_ROWS=1 ctest --test-dir build -R '^testRigExecExecGoldens_<name>$'
git diff tests/fixtures/execRows
```

The test writes `tests/fixtures/execRows/<reference>/<name>.golden` for the
configuration it runs on (omit `<name>` to record every rig). A new rig is
recorded the same way after adding its `rigexec_add_exec_rows_test` entry.
Nothing but the change under review judges the new values, so read the diff
row by row.

## Rendered-image baselines

`render/twistRibbon_1001.png` and `render/twistRibbon_1024.png` are 128x128
Storm color captures of `examples/05_TwistRibbonSpine.usda` (guides shown,
lighting off) at frames 1001 and 1024. `testRigExecImagingRender` renders the
stage headlessly through `UsdImagingGLEngine` on a hidden OpenGL context and
compares each frame with its baseline. A pixel differs when any channel moves
by more than 8; the test fails when more than 40 pixels differ and writes
`actual_<name>.png` to its working directory. It exits with ctest's skip code
where no context or Storm is available (a context is only created on macOS).

`render/reference.txt` records the GPU renderer, graphics API, OS release and
architecture of the capture. On any other configuration the test still renders
and checks that both frames are non-blank and differ from each other, but skips
the pixel comparison with a note and passes. After a deliberate rendering change, review the new
images and rewrite them:

```sh
source bin/_env.sh
RIGEXEC_RENDER_UPDATE_BASELINES=1 build/testRigExecImagingRender \
  examples tests/fixtures/render
```

## Baked programs

`minimal.rigexec` is the smallest valid program at the current format
version; `testRigExecRuntimeFixture` rebuilds it and compares bytes. After a
format bump, run that test with `RIGEXEC_UPDATE_FIXTURES=1` to rewrite it.
Open reads the current format only: an older file is refused and is baked
again from its stage. `registeredMover.rigexec` is a rig with a registered
external mover at the current format; `testRigExecExternalMovers` writes the
same program to the build tree, so after a format bump copy that file over it.
