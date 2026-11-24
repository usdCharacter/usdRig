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

## Exec cross-check rows

`execRows/<reference>/<fixture>.golden` hold, per frame, every row the exec
cross-check judged when the file was recorded: the row key, a digest of the
inputs the baked step body read, and the exact value it stored, which OpenExec
reproduced bit for bit at that time. `testRigExecExecGoldens` captures the same
rows from the production evaluator without OpenExec and compares them.
`<reference>` names the platform, architecture and compiler that recorded the
file (`RigExecExecRowsReference`). That configuration compares every bit; any
other compares which rows were judged and the type and length of each value.

The authoritative reference is the Linux configuration above
(`linux-x86_64-gcc13`); the Linux CI job records it on every run and
uploads it as the `exec-rows-linux` artifact. The files under
`windows-x86_64-msvc1936` were recorded on a Windows development machine with
the same rows and keys; 15 of the 870 rows differ from the Linux set in the
last bits of a double (at most 20 ULP, from the math library), so that set is
exact only for that compiler. Replace both sets together. To record, from a
shell that sourced `bin/_env.sh`, with `RIGEXEC_ENABLE_SIMD=0`:

```sh
build/testRigExecExecCrossCheckRows examples/01_FkChainTail.usda 1001 \
  --golden-out tests/fixtures/execRows/<reference>/01_FkChainTail.golden \
  1001 1024 1048
```

The examples use frames 1001, 1024 and 1048; `computed_ik_space` uses 1, 5
and 10. Recording fails unless every judged row matches OpenExec.

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
`minimal_format34.rigexec` is the same program at format 34, before op
nodes; keep it at 34.
`minimal_format32.rigexec` and `registeredMover.rigexec` were baked at format
32, before the manifest. Keep them at 32: they are what proves older files
still open, through the format-32 manifest, and play.
