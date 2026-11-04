# ![Bone 2D](../../icons/concept.png) Bone 2D

*API schema that makes a joint one bone of a 2D skeleton.*

| | |
|---|---|
| **Node type** | `RigExecBone2dAPI` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Apply `RigExecBone2dAPI` to a `RigExecJoint` that a
[Skeleton 2D](skeleton_2d.md) lists in `rigExec:joints`. The joint's
namespace parent is the bone's parent when it is a bone of the same skeleton
(otherwise the bone is a root), and its `rest:space` is relative to that
parent. Composed, the setup frame lies in the skeleton's plane: base at the
origin, the bone along +X, rotation about the plane's +Z only. The bone runs
to its first child joint.

One bone of a 2D skeleton, applied to a RigExecJoint that a
RigExecSkeleton2d lists in rigExec:joints.

Hierarchy: the bone's parent is its nearest RigExecJoint namespace
ancestor when that joint is a bone of the same skeleton; otherwise the
bone is a root of the skeleton and hangs from its start frame. A
reparent is a namespace move; an animated parent is a
RigExecSpaceSwitch whose rigExec:target is the bone (read by the
skeleton, see RigExecSkeleton2d).

Setup: the joint's rest:space is relative to its namespace parent, as
for every RigExecXformable. Composed, the bone's setup frame lies in
the start frame's XY plane: bone base at its origin, the bone along +X,
rotation about the plane's +Z only. rest:tx..rest:rz stay unauthored.
The bone's tip is its first RigExecJoint child in namespace order: a
child bone, or for a leaf an End joint (a plain RigExecJoint without
this API that no solver lists). A bone with no joint child is a pin:
its tip is its base.

Channels: avars:tx and avars:ty move the bone in its parent's frame,
avars:rz turns it (degrees, as a delta from the setup angle) and
avars:sx scales it along the bone, carrying the tip with it. A channel
that property movers revise (driven actions) is read as that chain's
result. The flip, lock, offset and target-index channels below are
animated; the index channel and the lock references are held.

Evaluation never clamps keyed or driven angles to the limits; limits
bound the lock solve, and authoring tools clamp the values they
write.

## How it works

The bone's tip is its first `RigExecJoint` child in namespace order: a
child bone, or for a leaf an `End` joint (a plain `RigExecJoint` without this
API that no solver lists). A bone with no joint child is a pin: its tip is its
base. The tip is where a target solve aims and what `avars:sx` stretches. A
child bone's rest is a value, so editing it re-solves without recompiling; an
`End` joint's offset is compiled, so editing it recompiles.

Animation lives on the joint's `avars`: `avars:tx` and `avars:ty` move the
bone in its parent's frame, `avars:rz` turns it (degrees, as a delta from the
setup angle) and `avars:sx` scales it along the bone. A channel that a property
mover or a driven action revises is read as that chain's result. The flip, lock,
offset and target-index channels are animated too; the index channel and the
lock references are held (stepped) between keys. An animated parent is a
[Space Switch](space_switch.md) on the bone, read by the skeleton.

`bone2d:colorTag`, `bone2d:showLabel` and `bone2d:shy` are display hints for
editors (a colour, a name label, membership of the shy set one view toggle
hides). Evaluation never reads them, so editing one neither recompiles nor
re-solves.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `bone2d:targetCandidates` | Ordered target bones for a tip-follow target, selected by `bone2d:targetIndex`. | no |
| `bone2d:angleControl`, `bone2d:posControl`, `bone2d:scaleControl` | At most one control bone each, in delta form. | no |

## Parameters

### Node parameters

#### `bone2d:angleLimits`

*Type:* `uniform bool`. *Default:* `false`.

Whether bone2d:minAngle/maxAngle bound the lock solve.

#### `bone2d:minAngle`

*Type:* `uniform double`. *Default:* `0`.

Lower limit in degrees, relative to the setup local angle.

#### `bone2d:maxAngle`

*Type:* `uniform double`. *Default:* `0`.

Upper limit in degrees, relative to the setup local angle.

#### `bone2d:independentAngle`

*Type:* `uniform bool`. *Default:* `false`.

Keep the setup world angle while ancestors turn: the world
angle is the setup world angle plus avars:rz. The base still
follows the parent.

#### `bone2d:scalingMode`

*Type:* `uniform token`. *Default:* `"normal"`.

Valid values: `normal`, `squashStretch`, `uniform`.

How avars:sx scales bound geometry across the bone:
normal leaves it, uniform scales it by sx too, squashStretch by
sx^(-bone2d:squashFactor). Children are never scaled across.

#### `bone2d:squashFactor`

*Type:* `uniform double`. *Default:* `1`.

Exponent of the squashStretch cross scale, in [0.01, 100].

#### `bone2d:ignoredByIk`

*Type:* `uniform bool`. *Default:* `false`.

Excluded when counting a parent's children for the lock
and target chains; pins (bones with no joint child) are always
excluded.

#### `bone2d:maxStretch`

*Type:* `uniform double`. *Default:* `1`.

Largest common scale, >= 1, the lock and target solves may
give the chain to reach an out-of-range lock point or target.

#### `bone2d:sideFlip`

*Type:* `bool`. *Default:* `false`.

Mirrors the bone's local Y (held). Composes down the hierarchy.

#### `bone2d:endFlip`

*Type:* `bool`. *Default:* `false`.

Mirrors the bone's local X about its base (held).

#### `bone2d:lock`

*Type:* `bool`. *Default:* `false`.

While true, the lock solve keeps the bone's base at
(bone2d:lockX, bone2d:lockY) and its world angle at bone2d:lockAngle
by turning its ancestors. Authoring tools write the reference with
each lock key.

#### `bone2d:lockX`

*Type:* `double`. *Default:* `0`.

Held base X in the skeleton's plane (the start frame's
space, asset space without one). Authored as a spline with held
knots, one per lock key, so the reference never slides between
keys.

#### `bone2d:lockY`

*Type:* `double`. *Default:* `0`.

Held base Y in the skeleton's plane; authored like bone2d:lockX.

#### `bone2d:lockAngle`

*Type:* `double`. *Default:* `0`.

Held world angle in degrees; authored like bone2d:lockX.

#### `bone2d:offsetX`

*Type:* `double`. *Default:* `0`.

Animation offset in the parent's frame, added to the posed
local position only (never to the rest used for binding). Authored as
a spline with held knots: 0 at the setup frame, the offset elsewhere,
so the setup frame shows the bone without it.

#### `bone2d:offsetY`

*Type:* `double`. *Default:* `0`.

Animation offset Y in the parent's frame; authored like bone2d:offsetX.

#### `bone2d:targetCandidates`

*Relationship.*

Ordered IK target options (other bones of the skeleton,
usually unparented pins). bone2d:targetIndex selects one per frame;
empty keeps no target.

#### `bone2d:targetIndex`

*Type:* `double`. *Default:* `-1`.

Index into bone2d:targetCandidates (rounded; negative or out
of range keeps no target). Authored with held knots. The owner's tip
tracks the target bone's base; a lone bone aims at it. A pin owner
moves its ancestors so its base meets the target; a pin with a
target and no parent bone is a compile error.

#### `bone2d:angleControl`

*Relationship.*

At most one bone whose rz drives this bone's rz (delta form).
Structural; the factor below scales it.

#### `bone2d:angleControlFactor`

*Type:* `uniform double`. *Default:* `0`.

Added per controller rz degree; 0.5 and -0.5 counter-move.

#### `bone2d:posControl`

*Relationship.*

At most one bone whose tx/ty drive this bone's tx/ty (delta form).

#### `bone2d:posControlFactorX`

*Type:* `uniform double`. *Default:* `0`.

Added per controller tx unit.

#### `bone2d:posControlFactorY`

*Type:* `uniform double`. *Default:* `0`.

Added per controller ty unit.

#### `bone2d:scaleControl`

*Relationship.*

At most one bone whose sx drives this bone's sx about 1.

#### `bone2d:scaleControlFactor`

*Type:* `uniform double`. *Default:* `0`.

This bone's sx multiplies by 1 + factor * (controller sx - 1).

#### `bone2d:colorTag`

*Type:* `uniform token`. *Default:* `"plain"`.

Valid values: `plain`, `red`, `orange`, `yellow`, `green`, `teal`, `blue`, `indigo`, `purple`, `pink`, `brown`, `gray`.

Display only: the colour editors draw the bone and its
channel rows in. Evaluation never reads it.

#### `bone2d:showLabel`

*Type:* `uniform bool`. *Default:* `false`.

Display only: editors draw the bone's name beside it.
Evaluation never reads it.

#### `bone2d:shy`

*Type:* `uniform bool`. *Default:* `false`.

Display only: editors may hide shy bones together with one
view toggle. Evaluation never reads it.

## Example

A forearm bone nests under the upper arm with the rest:space
(1, 0) relative to it and an `End` joint one unit along it. `avars:rz` of 30
turns it 30 degrees from the setup angle. `bone2d:angleControl` pointing at a
dial bone with `bone2d:angleControlFactor` -0.5 makes it counter-move at half
the dial's turn.

## Tips

- A pin (no joint child) is always excluded when counting a parent's children for the lock and target chains; a pin with a target and no parent bone is refused when the rig compiles.
- Evaluation never clamps keyed or driven angles to `bone2d:minAngle` and `bone2d:maxAngle`; authoring tools clamp the values they write.
- Order the children (`reorder nameChildren`) when a bone has several: the first child joint is its tip.

## See also

- [Skeleton 2D](skeleton_2d.md)
- [Bone 2D Group](bone_2d_group.md)
- [Space Switch](space_switch.md)
- [Skin Mover](skin_mover.md)

---

[UsdRig](../index.md)
