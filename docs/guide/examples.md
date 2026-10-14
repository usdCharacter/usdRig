---
title: Examples
---

# Examples

Every example is a self-contained, animated stage with one `RigExecRoot`.
Open any of them with `bin/launch_usdview.bat` (Windows) or `bin/usdview.sh`.

## The biped

The full character: skeleton, rig, face, skinned mesh, correctives, picker and
TouchPose regions, composed as a tree of layers. It needs nothing else to
open.

```bat
bin\launch_usdview.bat examples\biped\Biped_stack.usda
```

<div class="grid cards" markdown>

-   **Body and limbs**

    ---

    FK and IK arms and legs with matched IK/FK switching, pole-vector
    pinning, stretch, auto clavicles, bendy limbs and pose-space
    correctives.

    [:octicons-arrow-right-24: Limbs](../tools/limbs.md)

-   **Face**

    ---

    Brows, cheeks, eyes, lids, mouth and jaw built from clusters, curve
    wires and blend shapes, with a projected eye shader.

    [:octicons-arrow-right-24: The biped rig](../specs/biped-rig.md)

-   **Picker and TouchPose**

    ---

    A two-panel picker (Body and Face) and painted touch regions on the
    skin, both composed into the stage as data.

    [:octicons-arrow-right-24: Control picker](../tools/picker.md)

</div>

The layers and how they compose are listed in the biped's
[README](../../examples/biped/README.md).

## Smaller rigs

[Build a rolling ball in usdview](../concepts/tutorial-rolling-ball.md):
follow a 3-minute video with a screenshot for every step, starting from an
unrigged ball and creating the control and movers by hand in Noodles.

| Stage | Shows |
|---|---|
| [`rigexec_flat.usda`](../../examples/rigexec_flat.usda) | The smallest complete rig. Start here to read one end to end. |
| [`ArmRig.usda`](../../examples/ArmRig.usda) | An arm asset: FK, IK and blend, twist distribution, ribbon, layered skinning, blend shape, volume correction, and a complexity variant set. |
| [`ArmShotAnim.usda`](../../examples/ArmShotAnim.usda) | A shot referencing the arm: IK animation, an IK/FK switch spline and a driven weight. |
| [`2d/bust_dd_b`](../../examples/2d/bust_dd_b/README.md) | An articulated 2D character with expressions, a picker and whole-character TouchPose. |

## The numbered series

One focus area per file, numbered roughly from simplest to most involved.

| Stage | Focus |
|---|---|
| [`01_FkChainTail.usda`](../../examples/01_FkChainTail.usda) | An FK chain |
| [`02_TwoBoneIkLeg.usda`](../../examples/02_TwoBoneIkLeg.usda) | Two-bone IK with a pole vector |
| [`03_IkFkBlendClamp.usda`](../../examples/03_IkFkBlendClamp.usda) | Blending IK and FK |
| [`04_BlendShapeFace.usda`](../../examples/04_BlendShapeFace.usda) | Blend shapes |
| [`05_TwistRibbonSpine.usda`](../../examples/05_TwistRibbonSpine.usda) | Twist distribution and a ribbon spine |
| [`06_LatticeBulge.usda`](../../examples/06_LatticeBulge.usda) | A lattice deformer |
| [`07_SurfaceDrape.usda`](../../examples/07_SurfaceDrape.usda) | Draping over a surface |
| [`08_AimEyes.usda`](../../examples/08_AimEyes.usda) | Aim constraints |
| [`09_PropertyMathMovers.usda`](../../examples/09_PropertyMathMovers.usda) | Property math |
| [`10_AimXformTurret.usda`](../../examples/10_AimXformTurret.usda) | Aiming a plain transform |
| [`11_VolumeWeights.usda`](../../examples/11_VolumeWeights.usda) | Volume weight fields |
| [`13_ReadPhases.usda`](../../examples/13_ReadPhases.usda) | Read phases |
| [`14_VolumeConstrainedSweep.usda`](../../examples/14_VolumeConstrainedSweep.usda) | A volume-constrained sweep |
| [`15_TransformMatrixMover.usda`](../../examples/15_TransformMatrixMover.usda) | A transform matrix mover |
| [`16_ConnectionReadPhases.usda`](../../examples/16_ConnectionReadPhases.usda) | Read phases through connections |

The full list, including the composition-arc and constraint test stages, is
in the [example catalog](../../examples/README.md).

## Node examples

Every page in the [node reference](../index.md) has its own minimal animated
stage under `docs/examples/`, the one its GIF was rendered from.
