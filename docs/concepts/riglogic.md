---
title: RigLogic behaviours
summary: RigLogic's radial-basis solvers, ML morph deformer and animated maps in RigExec, where each one comes from, and the examples that show them.
order: 38
---

Three behaviours come from Epic Games'
[OpenRigLogic](https://github.com/EpicGames/OpenRigLogic), the runtime behind
MetaHuman faces:

| Behaviour | Node | Kernel | Example |
|---|---|---|---|
| Radial-basis solvers | [Pose Interpolator](../nodes/pose_interpolator.md) with `rigExec:solver` `additive` or `interpolative` | `libs/rigExecMath/rbfMethodKernel.h` | [pose_interpolator_riglogic.usda](../examples/pose_interpolator_riglogic.usda) |
| Machine-learned morphs | [ML Mover](../nodes/ml_mover.md), [ML Output](../nodes/ml_output.md) | `libs/rigExecMath/mlpKernel.h` | [ml_mover.usda](../examples/ml_mover.usda) |
| Animated maps | [Animated Maps](../nodes/animated_maps.md), [Animated Map](../nodes/animated_map.md) | `libs/rigExecMath/conditionalTableKernel.h` | [animated_maps.usda](../examples/animated_maps.usda) |

All three are steps of the pose-interpolator pipeline in the shared
operation graph. The native evaluator, the frame-cache worker and the
`.rigexec` runtime run the same header-only kernels on the same declared
inputs, so the three agree bit for bit. No behaviour has a pass of its own,
and none takes a lock. Each one publishes floats that any consumer can
connect to, like a pose weight. Nothing is written to the stage.

## Radial-basis solvers

A pose interpolator keeps its own solve by default (`rigExec:solver = "pose"`),
and old files evaluate exactly as before. `additive` and `interpolative`
select RigLogic's solvers:

- **additive** sums each pose's kernel value.
- **interpolative** weights the poses through the inverse of the
  pose-to-pose kernel matrix, inverted once at compile with RigLogic's Crout
  LU, and clamps each weight to [0, 1].

Both then apply `rigExec:normalizeMethod`: `onlyAboveOne` divides by the sum
when it exceeds one, `always` divides every time. Next they multiply by each
pose's `rigExec:scale`, then publish zero for any pose at or below
`rigExec:weightThreshold`.

The input row is each frame driver's local rotation from rest, as
(x, y, z, w) on the w ≥ 0 hemisphere, in `rigExec:driver` order. With
numeric drivers it is instead the values of up to three
`rigExec:driverAttributes`. A RigLogic solve may name several frame drivers.
Each pose then authors one rotation per driver in `rigExec:rotations`.

`rigExec:distanceMethod` chooses the metric:

- **euclidean** is the distance over the raw row, and is the only one
  numeric drivers may use.
- **quaternion** is the arc length between rotations.
- **swingAngle** is the angle between the swings about `rigExec:twistAxis`.
- **twistAngle** is the difference of the unwound twist angles.

`rigExec:kernel` chooses the weight function of distance `d` and radius `r`.
The formulas are RigLogic's:

| Kernel | Weight |
|---|---|
| `gaussian` | exp(−d / r²) |
| `exponential` | exp(−2d / r) |
| `linear` | max(0, 1 − d/r) |
| `cubic` | max(0, 1 − (d/r)³) |
| `quintic` | max(0, 1 − (d/r)⁵) |

RigLogic's gaussian depends on the units of the radius. The pose solver's own
gaussian stays exp(−(d/r)²), and it also accepts `exponential`, `cubic` and
`quintic`. `rigExec:automaticRadius` replaces `rigExec:radius` with the mean
distance between every pair of poses.

Compile warns when two poses are at the same place under the distance
method, because an interpolative solve is singular there. A pure twist
about the twist axis, for example, has the neutral pose's swing.

## ML mover

`RigExecMlMover` is a learned corrective, and one prim has two roles:

- **The network.** A multilayer perceptron reads the input row: each
  `rigExec:driver` contributes its local rotation from rest (four values,
  measured as a pose interpolator measures its driver), and each
  `rigExec:driverAttributes` value (float or double, any number) follows.
  The network writes one weight per `RigExecMlOutput` child, in namespace
  order.
- **The mover.** It is a blend-shape revision of `rigExec:moves`. Its
  `rigExec:blendInputs` connect to those outputs and name UsdSkelBlendShape
  samples, so the morphs are sparse offsets under the common mover envelope.

The network is RigLogic's:

- Dense layers, each with a row-major weight matrix (one row per output
  unit) and a bias.
- Activation codes 0 linear, 1 relu, 2 leakyRelu, 3 tanh and 4 sigmoid.
- `rigExec:weights` holds every layer's weights, then `rigExec:biases` holds
  every layer's biases.
- `inputs:mask` is RigLogic's region mask, a per-frame float input. Exactly
  zero publishes `rigExec:outputDefaults` without running the network; any
  other value multiplies every output.

`rigExec:inputMean` and `rigExec:inputScale` are a RigExec addition: an
optional `(x − mean) · scale` before the first layer, the form deformer
trainers export.

RigExec ships no trainer. Any MLP whose layers can be written out in this
layout runs as authored. The network's arrays, normalization and output count
are epoch identity, so an edit recompiles. A network whose output count
differs from its `RigExecMlOutput` children is set aside at compile, with a
diagnostic.

## Animated maps

`RigExecAnimatedMaps` turns control values into [0, 1] mask intensities. A
face shader uses them to blend wrinkle maps in. Each `RigExecAnimatedMap`
child is a RigLogic conditional table:

- Each row reads one input `x` and adds `slope · x + cut` when `x` lies in
  `[from, to]`. A reversed interval is swapped.
- Among consecutive rows on the same input, only the first matching interval
  counts, so neighbouring segments that share an endpoint count it once.
- The sum is clamped to [0, 1] and published on `outputs:value`.

Each value is also published as a constant float primvar,
`primvars:<rigExec:primvarName>`, on every `rigExec:meshes` target.
Imaging hands these primvars to Hydra. A material reads one through
`UsdPrimvarReader_float`, and a glslfx shader through its attributes block.
A changed value dirties only the mesh's primvars. A map appearing or going
away is a structural change. Playing a `.rigexec` file in usdview does not
publish these primvars, which is also true of the surface projector's
matrices; live evaluation does.

## Authoring

The C++ builder (`libs/rigExecRigging/rigBuilder.h`) and the Python module
expose the same calls:

```python
import rigexec

builder = rigexec.Builder.create(stage, "/Asset/Rig")

# A RigLogic solve on a pose interpolator.
arm = builder.add_pose_interpolator("ArmFold", "/Asset/Rig/Joints/Upper")
arm.set_drivers(["/Asset/Rig/Joints/Upper", "/Asset/Rig/Joints/Upper/Fore"])
arm.set_solver("interpolative")
arm.set_distance_method("quaternion")
arm.set_kernel("gaussian")
arm.set_radius(1.2)
arm.set_weight_threshold(0.05)
arm.add_pose("fold").set_rotations([(1, 0, 0, 0), (0.7071, 0, 0, -0.7071)])

# An ML mover: one call adds an output, its blend input and its sample.
ml = builder.new_mover_chain("Arm", "/Asset/Geom/Limb.points").add_ml_mover("Muscles")
ml.set_drivers(["/Asset/Rig/Joints/Upper/Fore"])
ml.set_driver_attributes(["/Asset/Rig/Controls/Flex.avars:tx"])
ml.set_network([3, 2], ["tanh", "sigmoid"], [], weights, biases)
ml.add_morph("bicep", "/Asset/Targets/Bicep")   # one per network output
ml.add_morph("crease", "/Asset/Targets/Crease")

# Animated maps on a face mesh.
maps = builder.add_animated_maps("Wrinkles")
maps.set_inputs(["/Asset/Rig/Controls/BrowL.avars:ty"])
maps.set_meshes(["/Asset/Geom/Face"])
maps.add_map("foreheadL", [0], [0.0], [1.0], [1.0], [0.0],
             primvar_name="wrinkleForeheadL")
```

`AddMlMover`, `AddMorph`, `AddAnimatedMaps`, `AddMap`, `SetSolver`,
`SetDistanceMethod`, `SetNormalizeMethod`, `SetRadius`, `SetWeightThreshold`,
`SetRotations` and `SetScale` are the C++ names.

## Examples

Each example is a docs stage with a node page, and each plays live in
usdview. In the node pages' GIFs the cyan wireframe is the rest pose.

- [ml_mover.usda](../examples/ml_mover.usda): a skinned two-joint arm and a
  Flex slider.
  - The network reads the forearm's rotation and the slider and drives a
    bicep bulge and an elbow crease.
  - Flexing the straight arm lifts the bicep to 0.44. Closing the elbow to
    80 degrees lifts it to 0.73 and fires the crease late, at 0.98. Flexing
    while bent takes the bicep to 0.94.
- [animated_maps.usda](../examples/animated_maps.usda): two brow controls
  feed three maps.
  - Each map drives a brow morph through its `outputs:value`.
  - Each map is also published as a primvar, which the wrinkle-line
    materials read into opacity.
- [pose_interpolator_riglogic.usda](../examples/pose_interpolator_riglogic.usda):
  a shoulder and elbow sweep a skinned arm. Three interpolators read the
  motion, and each pose weight lifts one bar of a meter.
  - An additive swing solve.
  - An interpolative quaternion solve over two drivers, with a half-scale
    pose and a weight threshold.
  - An additive euclidean solve over the controls' own avars, with an
    automatic radius.

```bat
bin\launch_usdview.bat docs\examples\ml_mover.usda
```

Each example has two tests:

- `example_graph_docs_<name>` compares the evaluators, the CPU reference and
  the exec cross-check at frames between the keys.
- `verify_binary_docs_<name>` bakes the stage and holds the `.rigexec`
  runtime to the native evaluator, under animation and under drags of a
  control and of an operator input.

## In a .rigexec file

Format 24 carries the three solves on the `PoseInterpolator` record:

- `RbfMethod`: the solved constants (the stored targets, the radius with
  any automatic radius applied, and for an interpolative solve the inverted
  kernel matrix), with `extra_driver_slots` and `extra_parent_slots`.
- `MlNetwork`, and the mask as a per-frame `Input`.
- `AnimatedMaps`, with `primvar_slots` and `primvar_sources`.

A file never has to re-derive a solve. A reader built before format 24
refuses these files, and a current reader still opens older ones.

## Verification

- `testRigExecRigLogicKernels` holds the RBF and conditional-table ports to
  values RigLogic itself produced (`tests/fixtures/rigLogicReference.inc`).
  - RigLogic computes in float and these ports in double. Additive solves
    agree within 2e-6.
  - Interpolative solves agree within 1e-3: RigLogic inverts the matrix in
    float, and the worst case measured is 4.0e-4. The double inverse is
    pinned exactly by separate checks.
  - The network kernel is checked against a forward pass written out term
    by term, not against RigLogic output.
- `testRigExecRigLogic` evaluates every solve, the ML mover and the maps
  live. It holds them to the kernels, then holds the frame cache and the
  runtime to live evaluation, bit for bit.

## Provenance

The method source is Epic Games
[OpenRigLogic](https://github.com/EpicGames/OpenRigLogic), MIT licensed, at
RigLogic 13.5.0, commit
[`9ca65ac08b64a1ea5f0d79466e64706b3ac524ba`](https://github.com/EpicGames/OpenRigLogic/commit/9ca65ac08b64a1ea5f0d79466e64706b3ac524ba).
Its [whitepaper](https://cdn2.unrealengine.com/rig-logic-whitepaper-v2-5c9f23f7e210.pdf)
describes the design. The license text is in
[THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md), and the citation in
[algorithm references](../references.md).

**Ported:** the arithmetic, rewritten for RigExec's kernels rather than
copied.

| RigExec kernel | OpenRigLogic source |
|---|---|
| `rbfMethodKernel.h` | `src/riglogic/rbf/cpu/RBFSolver.cpp` (distances, swing and twist, weight functions, normalization, automatic radius), `AdditiveRBFSolver.cpp`, `InterpolativeRBFSolver.cpp` (Crout LU inverse) |
| `mlpKernel.h` | `src/riglogic/ml/cpu/Operation.h` (forward pass, mask, defaults), `src/riglogic/ml/cpu/layers/*LayerEvaluator.h` (activations) |
| `conditionalTableKernel.h` | `src/riglogic/conditionaltable/ConditionalTable.cpp` (`calculateForward`), `src/riglogic/animatedmaps/AnimatedMapsFactory.cpp` |

**RigExec's own:**

- **Schema and nodes:** the schema, prims and node pages.
- **Graph integration:** the pose-pipeline steps, the format records and
  the imaging primvar path.
- **Arithmetic choices:**
  - double accumulation where RigLogic uses float;
  - the ML input normalization;
  - RigLogic's kernels added to the pose solver;
  - a zero radius treated as an exact match, where RigLogic divides by zero.
- **Examples and data:** the three examples and their hand-set weights,
  poses and shapes.

**Not used:** no RigLogic source file, DNA file or MetaHuman asset is in
this repository. RigLogic's DNA reader, LODs, joint and blend-shape
behaviours, and its SIMD and GPU evaluators were not ported.

**Reference data:** `tests/fixtures/rigLogicReference.inc` is RigLogic's own
output.
- A harness kept outside this checkout compiled OpenRigLogic's
  `RBFSolver`, `AdditiveRBFSolver`, `InterpolativeRBFSolver`,
  `ConditionalTable` and memory-resource sources at that commit.
- It ran `RBFSolver::create(...)->solve(...)` and
  `ConditionalTable::calculateForward` on the fixture's cases.
- It printed RigLogic's float results with `%.9g`.

The fixture header records the same procedure.
