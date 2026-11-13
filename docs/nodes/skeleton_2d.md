# ![Skeleton 2D](../../icons/concept.png) Skeleton 2D

*Solves a flat cut-out skeleton: bone channels, flips, locks and targets, one frame per bone.*

| | |
|---|---|
| **Node type** | `RigExecSkeleton2d` |
| **Example** | [skeleton_2d.usda](../examples/skeleton_2d.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A 2D skeleton is one `RigExecSkeleton2d` solver claiming an ordered
list of nested `RigExecJoint` bones that carry `RigExecBone2dAPI`. The bones'
namespace is their hierarchy and each bone's `rest:space` is relative to its
parent, as for every joint. The bones lie in the XY plane of the skeleton's
start frame (the rig's asset space without one) and turn about its +Z, in
degrees. The solver composes every bone's animated channels down the hierarchy
over the bones' setup frames and publishes one frame per bone; a
[Skin Mover](skin_mover.md) with authored weights then moves artwork with
those frames.

Solves a 2D skeleton: composes every bone's channels down the
namespace hierarchy over the setup frames, applies flips, independent
angles and cross scaling, then the lock and target solves, and
publishes one frame per listed joint (view-free extraction, like the
other solvers). The published frame carries the bone's cross scale, so
a joint's computeMatrix is the bone's setup-to-pose deformation.

The skeleton hangs from rigExec:startFrame as a RigExecFkChain does:
rests and parent spaces are expressed in the start frame's rest frame,
the solve stays in its XY plane, and the published frames are composed
back through its posed frame. Without a start frame the plane is the
rig's asset space.

A RigExecSpaceSwitch whose rigExec:target is one of the bones is read
by the skeleton and never runs as its own pose step. Its
rigExec:sources are the bone's parent spaces in order: a bone of this
skeleton, a frame provider outside it, or a prim that is not a frame
provider (the RigExecRoot), which stands for the start frame (asset
space without one). The active index (rigExec:activeSpaceAttribute,
else inputs:activeSpace) is clamped to the list; a whole number selects
a space and a fractional one blends the two it lies between as the
space switch does. A bone whose active space is not a bone of the
skeleton is a chain root for locks and targets. The namespace parents
and every source must be acyclic, an outside source must not sit under
a bone of the skeleton, and the switch must not author rigExec:space,
rigExec:rotationFilters or a cleared axis mask.

## How it works

The skeleton hangs from `rigExec:startFrame` the way an
[FK Chain](fk_chain.md) does: the bones' rests and outside parent spaces are
expressed in the start frame's rest frame, the solve stays in its plane, and
the published frames are composed back through its posed frame. Per frame the
solver composes each bone over its parent, applies side and end flips and
independent angles, applies the cross scale, then runs the lock solve (cyclic
coordinate descent over each locked bone's ancestors, within the angle limits)
and the target solve (a bone's tip follows its target bone's base). A bone's
tip is its first child joint. Locks win over targets. Bones under an inactive
member of a [joint group](joint_group.md) publish their rest frames and take
no part. The published frame is the bone's setup-to-pose deformation,
including its cross scale; children compose from the pose frame, so a bone's
cross scale never shears them.

A [Space Switch](space_switch.md) whose target is a bone is read by the
skeleton and never runs as a pose step of its own: its sources are the bone's
parent spaces in order. A source may be a bone of the same skeleton, a frame
provider outside it (read as an input of the solve), or a prim that is not a
frame provider, such as the rig root, which stands for the start frame. The
active index is clamped to the list; a fractional index blends the bone's
frames under the two neighbouring spaces, the origin linearly and the angle
along the shortest arc. A bone whose active space is not one of the bones is a
chain root for locks and targets.

A bone's rest is its composed `rest:space`, or, where a pose step below the
skeleton writes the bone, that step's frame, as for every aggregate solver.
The rest must be a rotation about the plane's +Z plus a translation. A rest
that is not planar is diagnosed, the skeleton passes the rests through for that
frame, and evaluation continues. The same operation graph runs in native,
frozen and `.rigexec` evaluation. The rules without a public source are listed
in [References](../references.md#2d-skeleton-rules-without-a-public-source).

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:joints` | Ordered bones (joints with `RigExecBone2dAPI`); list position is the aggregate element index. | yes |
| `rigExec:startFrame` | The joint or control the skeleton hangs from; its XY plane is the skeleton's plane. It may not be a bone or sit under one. | no |
| `rigExec:jointGroups` | Ordered joint groups of this skeleton (prims with `RigExecJointGroupAPI`, conventionally Scopes); empty keeps every bone active. | no |

## Parameters

### Node parameters

#### `rigExec:joints`

*Relationship.*

Ordered bones (RigExecJoint with RigExecBone2dAPI). List
position is the aggregate element index; parents usually come
first, and the solve orders them itself. A pose step below the
skeleton that writes a bone hands that frame to the skeleton as the
bone's rest, as for every aggregate solver; it must be a rotation
about the start frame's +Z plus a translation.

#### `rigExec:startFrame`

*Relationship.*

Optional single provider (joint or control) the skeleton
hangs from, as for RigExecFkChain: the bones' rests and outside
parent spaces are expressed in its rest frame, the bones turn about
its +Z, and every published frame is composed back through its
posed frame. Unauthored solves in the rig's asset space. It must not
be a bone of the skeleton or sit under one.

The target is an ordinary frame input with positional read
semantics: the skeleton must execute after
whatever poses it, and the compiler warns when it does not.

#### `rigExec:startFramePolicy`

*Type:* `uniform token`. *Default:* `"none"`.

Valid values: `none`, `parent`.

Who names the start provider when rigExec:startFrame has no
authored targets, by the RigExecFkChain rule: `none` (the default)
solves in asset space; `parent` derives the nearest namespace
ancestor of the bones that is a RigExecJoint or RigExecControl and
that every bone shares, and falls back to asset space with a compile
warning when there is none. Compiled into the solve, never
authored.

#### `rigExec:jointGroups`

*Relationship.*

Ordered joint groups of this skeleton: prims carrying
RigExecJointGroupAPI (conventionally Scopes). Empty keeps every
bone active.

## Example

A leg nested Hip / Thigh / Shin, with an `End` joint under the shin,
and a pin bone Foot beside it with no joint child. The bones key
`avars:rz`; the thigh also keys `avars:sx`, which stretches it along its
length and carries the shin with it, and the foot keys `avars:tx`. Each
bone holds a sphere weight stretched along its segment to its first joint
child: at the midpoint (`rest:tx` = L/2), `inputs:scaleX` = 1 + L/(2S),
`inputs:falloffMin` = max(S/4, (L/2) S/(L/2 + S)) and `inputs:falloffMax`
= S for a segment of length L and strength S; the foot's is an unscaled
sphere about its origin. One skin mover binds the leg mesh with rows
written by `rigexec.binding.generate_skin_weights` from those spheres; the
small tab quad beside the leg lies outside every sphere, keeps zero rows
and holds still.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\skeleton_2d.usda
```

## Tips

- Angle limits bound the lock solve only; keyed or driven angles are never clamped.
- A lock chain is capped at 32 bones, and a bone with `bone2d:ignoredByIk`, or a pin, is not counted among its parent's children; a lock that the descent leaves short retries from the other bend and keeps the closer result.
- Reparent a bone by moving it in the namespace (re-expressing its rest); animate a parent with a Space Switch on the bone whose first source is its namespace parent.
- The namespace parents and every switch source must be acyclic, an outside source may not sit under a bone of the skeleton, and a switch on a bone may not author `rigExec:space`, `rigExec:rotationFilters` or a cleared axis mask.
- Bind artwork with a Skin Mover whose `rigExec:influences` are the skeleton's bones; put a Sphere Weight inside each bone, stretched along its segment, and write `rigExec:jointIndices` / `rigExec:jointWeights` with `rigexec.binding.generate_skin_weights`.
- See [2D skeletons, binding and smart-bone step tables](../concepts/skeletons-2d.md) for the whole workflow.

## See also

- [Bone 2D](bone_2d.md)
- [Joint Group](joint_group.md)
- [Space Switch](space_switch.md)
- [FK Chain](fk_chain.md)
- [Skin Mover](skin_mover.md)
- [Animated Maps](animated_maps.md)

---

[UsdRig](../index.md)
