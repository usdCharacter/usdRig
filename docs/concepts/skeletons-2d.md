---
title: 2D skeletons, binding and driven actions
summary: Flat cut-out rigs: bones, the skeleton solve, binding artwork to bones, and dials that drive whole action frames.
order: 72
---

A 2D rig moves flat artwork with a skeleton that lies in one plane. It uses
the same machinery as a 3D rig: bones are ordinary [joints](../nodes/joint.md),
the solve is one operation in the shared graph, and artwork moves through a
mover. Native, frozen and `.rigexec` evaluation run the same kernels.

## Bones

A bone is a `RigExecJoint` with `RigExecBone2dAPI`, listed in a
[Skeleton 2D](../nodes/skeleton_2d.md)'s `rigExec:joints`. Bones lie in the XY
plane of the skeleton's start frame (the rig's asset space without one) and
turn about its +Z, in degrees.

- **Hierarchy.** Bones nest in the namespace like any joints. A bone's parent
  is its nearest `RigExecJoint` ancestor when that joint is a bone of the same
  skeleton; otherwise the bone is a root of the skeleton. Reparenting is a
  namespace move, which recompiles; an animated parent is a space switch (see
  below).
- **Setup frame.** The joint's `rest:space` is relative to its namespace
  parent, as for every joint. Composed, the setup frame has the base at the
  origin, the bone along +X and rotation about the plane's +Z only;
  `rest:tx`..`rest:rz` stay unauthored.
- **Tip.** A bone runs from its base to its first child joint in namespace
  order: a child bone, or for a leaf an `End` joint (a plain `RigExecJoint`
  without the API that no solver lists). A bone with no joint child is a pin,
  whose tip is its base. The tip is where a target solve aims and what
  `avars:sx` stretches. A child bone's rest is a value (an edit re-solves
  without recompiling); an `End` joint's offset is compiled (an edit
  recompiles).
- **Channels.** `avars:tx`, `avars:ty`, `avars:rz` (a delta from the setup
  angle) and `avars:sx` animate the bone, together with the flip, lock, offset
  and target-index channels.

See [Bone 2D](../nodes/bone_2d.md) for the full list.

## Start frame

`rigExec:startFrame` names the joint or control a skeleton hangs from, as it
does for an [FK Chain](../nodes/fk_chain.md); `rigExec:startFramePolicy`
`parent` derives it from the bones' nearest joint or control ancestor. The
bones' rests and outside parent spaces are expressed in the start frame's rest
frame, the solve stays in its XY plane, and every published frame is composed
back through the start frame's posed frame. A 2D face drawn on a 3D head is a
skeleton whose start frame is the head joint: the bones turn in the head's
plane and follow it. Without a start frame the plane is the rig's asset
space. The start frame may not be a bone or sit under one.

## Space switches on bones

A [Space Switch](../nodes/space_switch.md) whose target is a bone is read by
the skeleton and never runs as a pose step of its own: the skeleton is the
bone's only writer. The switch's `rigExec:sources` are the bone's parent spaces
in order:

- a bone of the same skeleton, whose solved frame the bone follows inside the
  solve;
- a frame provider outside the skeleton, read as an input of the solve; the
  bone is then a chain root for locks and targets;
- a prim that is not a frame provider, such as the rig root, which stands for
  the start frame (asset space without one).

The active index is `rigExec:activeSpaceAttribute` (normally a held
`avars:space` on the bone) or else `inputs:activeSpace`, clamped to the list.
A whole number selects a space; a fractional one blends the bone's frames
under the two neighbouring spaces, the origin linearly and the angle along the
shortest arc, as the space switch blends its spaces. As with any space switch,
both halves of the composition come from the selected space, so at rest
channels a switch keeps the bone where it is. The namespace parents and every
source must form no cycle, an outside source may not sit under a bone of the
skeleton (it moves with the skeleton's result), and such a switch may not
author `rigExec:space`, `rigExec:rotationFilters` or a cleared axis mask.

## The skeleton solve

Per frame the solver:

1. composes each bone's channels over its parent space and the setup frames;
2. applies side and end flips, independent angles and the cross scale
   (`bone2d:scalingMode`);
3. runs control bones (a bone whose `rz`, position or `sx` another bone's
   channels drive, in delta form);
4. runs the lock solve, which turns a locked bone's ancestors so its base and
   world angle meet the held reference, within the angle limits;
5. runs the target solve, so a bone's tip follows its target bone's base
   (a pin turns its ancestors so its base meets the target);
6. publishes one frame per bone: its setup-to-pose deformation.

Locks win over targets. [Bone groups](../nodes/bone_2d_group.md) hold
alternative chains; the bones under inactive members publish their rest
frames. A group is a prim (conventionally a `Scope`) with
`RigExecBone2dGroupAPI`, whose built-in collection `members` lists the chains'
first bones explicitly. Children compose from the pose frame, so a bone's
cross scale never shears them.

`bone2d:colorTag`, `bone2d:showLabel` and `bone2d:shy` are display hints for
editors; evaluation never reads them.

## Live rests

A pose step below the skeleton that writes a bone is an ordinary stacked
chain. Its frame becomes that bone's rest, as for every aggregate solver, and
the skeleton solves over it. The step reads the bone where the namespace
composes it, its ancestors' avars included, so with a turned parent bone the
solve turns the bone about that parent again; a step meant to act on solved
bones sits above the skeleton. The rest must still be a rotation about the
plane's +Z plus a translation. A frame that is not planar is diagnosed, the
skeleton publishes its rests for that frame, and evaluation continues.

## Binding

A [Skin Mover](../nodes/skin_mover.md) binds one points property to the
skeleton's bones, which it lists in `rigExec:influences`, with authored
`rigExec:jointIndices` and `rigExec:jointWeights` in UsdSkel's layout, exactly
as a 3D skin does. Each point moves by the weighted blend of its influences'
setup-to-pose maps (Skin's `classicLinear` or `dualQuaternion`). A point bound
rigidly to one bone carries a single weight of 1; a point whose weights sum to
less than one holds toward its rest position in proportion, so a row of zeros
keeps the point where it is.

The region a bone owns is a [Capsule Weight](../nodes/capsule_weight.md)
authored inside it: with no `rigExec:segmentEnd`, its segment runs from the
bone's origin to the bone's first `RigExecJoint` child (its next bone, or its
`End` joint), the same rule that places the bone's tip, and its
`inputs:falloffMin`/`inputs:falloffMax` are the inner and outer radii of the
tube about that segment. A pin bone, with no joint child, has a zero-length
segment and its capsule is a sphere about the joint. The capsule's end is read
as the child's live frame, so it follows a stretched bone in every evaluator.

The skin's rows are generated from those capsules rather than evaluated live.
`rigexec.binding.generate_skin_weights(stage, skin_mover, point_indices=None)`
writes them:

- **Points.** The mover's `rigExec:moves` target is read at Default and
  carried into the rig's asset space by its setup transform.
- **Fields.** For each influence in `rigExec:influences`, every
  `RigExecCapsuleWeight` child is measured at its rest placement (no pose is
  evaluated), with the segment end, `inputs:scaleX/Y/Z`, falloff band and
  range policy the evaluator uses. Several capsules in one bone sum, clamped
  to 1; an influence with no capsule contributes 0 and is reported.
- **Rows.** Each point's fields are normalised over the influences, and
  `rigExec:jointIndices` index that list, so the influences need not be the
  skeleton's bones. A point outside every capsule keeps a zero row and holds
  its rest. A row lists its non-zero influences in influence order, padded
  with index 0 and weight 0; `rigExec:elementSize` is the widest row, at
  least 1.
- **Partial rows.** With `point_indices`, only those rows are replaced; the
  others keep their authored slots and `rigExec:elementSize` is recomputed
  over the result. A tool writes rigid rows (one weight of 1) and released
  rows (zeros) itself and regenerates the rest this way.

Only `rigExec:jointIndices`, `rigExec:jointWeights` and `rigExec:elementSize`
are authored, at the stage's edit target. The rows go stale when a rest or a
capsule changes, so the authoring tool regenerates them after such an edit. A
capsule bound as an ordinary mover's `rigExec:weightObject` works as an
envelope too, in 2D and 3D rigs alike.

[skeleton_2d.usda](../examples/skeleton_2d.usda) is a leg bound this way.

## Driven actions

A [Driven Action](../nodes/driven_action.md) maps a dial's angle to a whole
action frame and publishes values at that frame.

- **Driver samples.** `rigExec:driverSamples` holds the driver value at
  frames 0 to N. The frame is the one whose sample is closest to the angle,
  clamped to [0, N].
- **Side.** The action serves one side of sample 0 (`rigExec:side`); on the
  other side, and at sample 0, the frame is 0.
- **Wrap.** A driver that turns one full revolution wraps an angle outside its
  range onto the action's side first.
- **Ties.** Equal distances take the earlier or later frame
  (`rigExec:frameTies`).
- **Channels.** Each [Action Channel](../nodes/action_channel.md) child
  publishes `outputs:value`: in `offset` mode `samples[f] - samples[0]`, for
  an additive mover; in `value` mode `samples[f]`, for a blend mover whose
  envelope is the action's `outputs:active`.
- **Reading outputs.** A reader connects to an output and declares
  `rigExecReadPhase "final"` to read what the action computed in the same
  evaluation; see
  [Connected inputs](how-operators-fire.md#connected-inputs). Undeclared, it
  reads the authored value. Two interpolators that drive each other form a
  cycle, which every evaluator sets aside.

## Evaluation

The skeleton, the skin and the driven action are operations in the one
graph that [native, frozen and runtime evaluation share](baked-vs-dynamic.md).
Their kernels are USD-free headers in `libs/rigExecMath`, the same ones the
runtime compiles. Topology (point counts, bone order, channel order) is fixed
per compiled rig.

## Provenance

The blend, the per-joint descent and the other cited methods are in
[References](../references.md). The rest of the 2D rules have no public source;
see [2D skeleton rules without a public source](../references.md#2d-skeleton-rules-without-a-public-source).
