# ![IK/FK Match Constraint](../../icons/concept.png) IK/FK Match Constraint

*Keeps a switchable limb's inactive half on its joints and publishes the values a switch authors.*

| | |
|---|---|
| **Node type** | `RigExecIkFkMatchConstraint` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A limb that switches between an FK chain and a two-bone IK
has two sets of controls, and only one of them drives the joints. This
constraint keeps the other set on the limb: while the switch is on FK, the IK
control and the pole are drawn where the joints are; while it is on IK, the
three FK controls are. Every frame it also publishes `outputs:ikValues` and
`outputs:fkValues`, the channel values each half needs to drive the joints
exactly where they are. A switch authors one of those sets together with the
switch value, so the limb does not move and nothing is solved at the click.

Keeps the half of a switchable limb that is not driving its
joints on those joints, and publishes the channel values that would make
it drive them unchanged.

A limb here is three joints a RigExecBlendPointFrames writes from a
RigExecFkChain and a RigExecTwoBoneIk, selected by the switch. While the
switch sits exactly on its FK value the IK control and the pole are
revised onto where the joints are; on its IK value the three FK
controls are. Between the two nothing is revised. Every control under a
revised one rides with it, as under any constraint, so a foot's roll
controls follow the IK control. The revision is a later version of each
control: the solvers below this constraint read the controls as their
own channels put them, so the limb itself does not move.

Each frame it also publishes, whichever half drives:

outputs:ikValues  = [ikControl tx ty tz rx ry rz,
                 poleControl tx ty tz,
                 upperScale, lowerScale]
outputs:fkValues  = [fkControls[0] tx ty tz rx ry rz,
                 fkControls[1] tx ty tz rx ry rz,
                 fkControls[2] tx ty tz rx ry rz,
                 upperScale, lowerScale]

the values that half's channels need for the joints to stay where they
are once it drives them. A switch authors one of those sets and the
switch value at the same time; nothing is solved at the switch. Rotations
are raw channel values (before avars:rotationSign), nearest the
channels' current values. Scale and spin are never written.

How each value is found, in row-vector convention, a control's frame
being A * P with A its channels and P what they compose onto:

IK control  the solve ends on its effector and copies the effector's
        orientation onto the end joint, so the effector's target
        is the end joint's frame, and the IK control's follows by
        the effector's current offset from it.
pole        on the bend plane, turned back by inputs:twist, past the
        middle joint by its rest distance from it; on the middle
        joint when inputs:pin is above zero. Its P is the one its
        own space gives once the IK control has moved.
FK controls each control's rest offset from its joint, carried by the
        joint. The P of controls 1 and 2 rides the control above
        as the match moves it, its bone offset lengthened in
        proportion to the new upper or lower scale.
scales      softDistance: the IK pair is solved (Newton's method on
        the limb lengths of RigExecTwoBoneIk) so the bones come
        out the joints' lengths; the FK pair is each bone's
        length over its rest length in rigExec:space.

The switch's other channels -- the pin and the twist -- are the IK
solve's own; a switch to FK leaves them for the tool to set.

## How it works

The constraint reads the limb's joints where it stands in the
pose stack, the IK solve's root and effector, its five controls and their
channels, and the IK solve's stretch, pin and twist channels. From them a
shared kernel (`ikFkMatchKernel.h`) computes both halves' values in one pass:
the IK control from the end joint and the effector's offset from the control,
the pole on the bend plane (turned back by the twist; on the middle joint when
the solve pins), the stretch scales by Newton's method on the limb lengths, and
the FK controls from their joints with each parent carried by the control above
it. The pole's parent after the switch is its own space switch re-run with the
IK control moved. The revision of the inactive controls is presentation only:
it is applied where control frames are published, after the walk, and every
control under a presented one rides with it. No solver, constraint or mover
reads it, so the rig evaluates exactly as it would without the constraint.
Between the two switch values neither half is presented. Native, frozen and
binary playback run the same kernel.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:ikControl`, `rigExec:poleControl` | The IK control and the IK solve's pole. | yes |
| `rigExec:fkControls` | The FK chain's three controls, root to end. | yes |
| `rigExec:limbJoints` | The three joints the limb's blend writes. | yes |
| `rigExec:rootControl`, `rigExec:effector` | The IK solve's root and the frame it reaches for. | yes |
| `rigExec:moves` | The IK control, pole and three FK controls, restated. | yes |
| `rigExec:space` | The IK solve's space, which carries its rest lengths. | no |
| `inputs:switch` | Connected to the limb's switch channel. | yes |
| `inputs:stretch`, `inputs:softDistance`, `inputs:upperScale`, `inputs:lowerScale`, `inputs:twist` | Connected to the same channels as the IK solve's inputs. | no |
| `inputs:pin` | Connected to the animator's pin channel, not to a pin the switch scales. | no |

## Parameters

### Common mover envelope

#### `rigExec:moves`

*Relationship.*

Reserved write-set relationship: exact prim/property targets.

#### `inputs:enabled`

*Type:* `bool`. *Default:* `true`.

Shape-preserving enable. Disabled movers pass their preceding
revision through unchanged (value-only edit).

#### `inputs:defaultWeight`

*Type:* `float`. *Default:* `1`.

Normalized common mover envelope in [0, 1]. With no bound
rigExec:weightObject it broadcasts over every logical element: zero
passes the incoming value through bit-for-bit and one applies the
mover's full-strength result.

#### `rigExec:weightObject`

*Relationship.*

Optional compatible total weight field, at most one target.
When bound, the weight object's values (including its own sparse or
constant fallback) supply the common envelope instead of
inputs:defaultWeight. Target, domain, and cardinality must match each
mover application; an incompatible binding is a compile error. A
multi-target mover may bind only a constant operation envelope whose
weightTarget is the mover prim itself; that one value broadcasts to
every application of the atomic mover.

### Constraint base

#### `rigExec:locked`

*Type:* `uniform bool`. *Default:* `false`.

Authoring lock metadata for application interchange. Evaluation
remains active while locked; use RigExecMoverAPI inputs:enabled or
inputs:defaultWeight to disable or blend the operation.

#### `inputs:affectTranslationX`

*Type:* `bool`. *Default:* `true`.

Per-axis translation mask. Ignored groups are a compile error.

#### `inputs:affectTranslationY`

*Type:* `bool`. *Default:* `true`.

#### `inputs:affectTranslationZ`

*Type:* `bool`. *Default:* `true`.

#### `inputs:affectRotationX`

*Type:* `bool`. *Default:* `true`.

Per-axis rotation mask. Ignored groups are a compile error.

#### `inputs:affectRotationY`

*Type:* `bool`. *Default:* `true`.

#### `inputs:affectRotationZ`

*Type:* `bool`. *Default:* `true`.

#### `inputs:affectScaleX`

*Type:* `bool`. *Default:* `true`.

Per-axis scale mask. Ignored groups are a compile error.
RigExecParentConstraint overrides the default to false to match the
FBX runtime, which leaves scale off unless it is asked for.

#### `inputs:affectScaleY`

*Type:* `bool`. *Default:* `true`.

#### `inputs:affectScaleZ`

*Type:* `bool`. *Default:* `true`.

#### `inputs:translationOffset`

*Type:* `double3`. *Default:* `(0, 0, 0)`.

Additive translation offset applied after the source blend.

#### `inputs:rotationOffset`

*Type:* `double3`. *Default:* `(0, 0, 0)`.

Additive Euler rotation offset in degrees, applied after the blend.

#### `inputs:scaleOffset`

*Type:* `double3`. *Default:* `(0, 0, 0)`.

ADDITIVE scale offset applied after the blend, which is why
its identity is (0, 0, 0) and not (1, 1, 1): the kernel computes
blendedScale + offset. A multiplicative offset would be a different
operator, not a different default.

#### `rigExec:rotationOrder`

*Type:* `uniform token`. *Default:* `"XYZ"`.

Valid values: `XYZ`, `XZY`, `YXZ`, `YZX`, `ZXY`, `ZYX`.

Euler order for the operators that compose a rotation.
Authoring it on one that does not is a compile error.

### Node parameters

#### `rigExec:ikControl`

*Relationship.*

The control the animator moves to place the IK effector.
Exactly one.

#### `rigExec:poleControl`

*Relationship.*

The IK solve's pole control. Exactly one.

#### `rigExec:fkControls`

*Relationship.*

The FK chain's three controls, root to end.

#### `rigExec:limbJoints`

*Relationship.*

The limb's three joints, root to end: the ones its blend
writes. Read as the solvers below this constraint leave them. Not
rigExec:joints, which names the joints a solver writes.

#### `rigExec:rootControl`

*Relationship.*

The IK solve's root control. Exactly one.

#### `rigExec:effector`

*Relationship.*

The frame the IK solve reaches for: the IK control or a
frame riding it. Exactly one.

#### `rigExec:space`

*Relationship.*

Optional: the IK solve's rigExec:space. Its rest-to-posed
map carries the rest bone lengths, as it does for the solve.

#### `inputs:switch`

*Type:* `double`. *Default:* `0`.

The limb's switch: connect it to the channel the blend's
inputs:weight reads.

#### `rigExec:ikValue`

*Type:* `uniform double`. *Default:* `1`.

The value of inputs:switch that means fully IK;
1 - ikValue means fully FK.

#### `rigExec:stretchPolicy`

*Type:* `uniform token`. *Default:* `"softDistance"`.

Valid values: `none`, `softDistance`.

softDistance: the limb stretches as RigExecTwoBoneIk's
softDistance policy does, and its FK chain lengthens by the same
upper and lower scales, so a match carries the bone lengths across
in them. none: the scales are published unchanged.

#### `inputs:stretch`

*Type:* `double`. *Default:* `1`.

The IK solve's inputs:stretch: connect it to the same channel.

#### `inputs:softDistance`

*Type:* `double`. *Default:* `0`.

The IK solve's inputs:softDistance: connect it to the same channel.

#### `inputs:upperScale`

*Type:* `double`. *Default:* `1`.

The IK solve's inputs:upperScale: connect it to the same
channel. A match writes this channel.

#### `inputs:lowerScale`

*Type:* `double`. *Default:* `1`.

The IK solve's inputs:lowerScale: connect it to the same
channel. A match writes this channel.

#### `inputs:pin`

*Type:* `double`. *Default:* `0`.

How far the IK solve pins its middle joint to the pole once
the limb is IK: connect it to the animator's pin channel, not to a
pin the switch itself scales.

#### `inputs:twist`

*Type:* `double`. *Default:* `0`.

The IK solve's inputs:twist, in degrees: connect it to the same channel.

#### `rigExec:scaleCalibration`

*Type:* `uniform double`. *Default:* `0`.

The IK solve's rigExec:scaleCalibration.

#### `rigExec:upperLengthOffset`

*Type:* `uniform double`. *Default:* `0`.

The IK solve's rigExec:upperLengthOffset.

#### `rigExec:lowerLengthOffset`

*Type:* `uniform double`. *Default:* `0`.

The IK solve's rigExec:lowerLengthOffset.

#### `outputs:ikValues`

*Type:* `double[]`.

Computed each frame, never authored: the IK half's channel
values that reproduce the joints. See the class doc for the layout.

#### `outputs:fkValues`

*Type:* `double[]`.

Computed each frame, never authored: the FK half's channel
values that reproduce the joints. See the class doc for the layout.

## Example

The biped's `Biped_ikfk_match.usda` layer adds one match per
limb. The arms' matches run just above the solvers; each leg's runs inside its
ankle aim's scope, after the foot-roll revisions the leg's IK reads and before
the aim revises the ankle. The picker's IK/FK buttons and the right-click
"Switch to IK / FK" author the published values.

## Tips

- Place the match after the limb's blend and before any constraint that revises its joints.
- A switch to FK also sets the pin and twist channels to zero; both belong to the IK solve.
- The FK chain must lengthen by the stretch scales in proportion, as the biped's limbs do.

## See also

- [Blend Point Frames](blend_point_frames.md)
- [Two Bone IK](two_bone_ik.md)
- [FK Chain](fk_chain.md)

---

[UsdRig](../index.md)
