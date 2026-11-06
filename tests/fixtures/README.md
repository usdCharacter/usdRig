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
