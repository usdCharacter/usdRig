---
title: Limbs
---

# Limbs

The biped's arms and legs each have an FK chain and a two-bone IK, blended by
the limb control's `ikfk` channel (0 is FK, 1 is IK). The limb controls are
`L_Arm`, `R_Arm`, `L_Leg` and `R_Leg`; their channels are on the picker's
switch buttons and in the Avar Editor.

## The other half follows the limb

The half a limb is not using stays on it. In FK, the IK hand or foot and the
pole are drawn where the arm or leg actually is; in IK, the three FK controls
are. The rig does this itself, with an
[IK/FK match constraint](../nodes/ikfk_match_constraint.md) on each limb, and
nothing else in the rig reads those drawn positions, so the limb evaluates
exactly as before.

## Switching IK and FK without a pop

Switch a limb with its button in the [picker](picker.md), with
**Switch to IK / FK** in the [marking menu](marking-menu.md), or from
usdview's right-click menu. The controls taking over are already where the
limb is, so the switch only writes their values, which the rig has already
computed, including the stretch carried in `stretchTop` and
`stretchBottom`. Nothing is solved when you click, and the values and the
switch land as one edit and one undo. In animation mode they are keyed at the
current frame.

Switching to FK resets `pvPin` and `twist`, which only IK has.

## Channels

| Channel | Range | Does |
|---|---|---|
| `ikfk` | 0 to 1 | Blend between FK (0) and IK (1). |
| `pvPin` | 0 to 1 | Pin the elbow or knee to the pole-vector control in IK. |
| `stretch` | 0 to 1 | Let the IK limb stretch past full reach to follow the hand or foot. |
| `softStretch` | ≥ 0 | How gently the IK limb eases into full reach instead of popping straight. |
| `stretchTop` | ≥ 0 | Lengthen the upper bone, in IK and FK. In FK the lower controls follow. |
| `stretchBottom` | ≥ 0 | Lengthen the lower bone, in IK and FK. |
| `twist` | degrees | Roll the IK bend plane about the limb. |
| `autoClav` (on `L_Shldr`, `R_Shldr`) | 0 to 1 | How far the shoulder follows the arm ([Auto Clavicle](../nodes/auto_clavicle.md)); 0 turns it off. |

Stretched bones scale the skin along the bone, so the mesh lengthens with the
limb.

!!! note "At rest, IK reaches slightly further"
    With the default soft stretch, an IK limb at rest is a fraction of a
    percent longer than FK. That is intended and matches the delivered rig:
    the soft ease starts just short of full reach.

## The solver

The limb behaviour is the [Two-Bone IK](../nodes/two_bone_ik.md) node's
`softDistance` stretch policy, with the [FK Chain](../nodes/fk_chain.md)'s
`segmentScale`. Both are documented with every parameter in the node
reference.
