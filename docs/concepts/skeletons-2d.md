---
title: 2D skeletons, binding and smart-bone step tables
summary: Flat cut-out rigs: bones, the skeleton solve, binding artwork to bones, and dials whose step tables drive whole action frames.
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

The region a bone owns is a [Sphere Weight](../nodes/sphere_weight.md)
authored inside it and stretched along its segment. For a bone whose first
`RigExecJoint` child (its next bone, or its `End` joint, the same rule that
places the bone's tip) sits at `t` in the bone's frame, with length `L = |t|`
and strength `S`:

- the sphere sits at the segment's midpoint, its local x along the segment:
  `rest:tx = L/2` for a bone along +X, or `rest:space` = a rotation about +Z
  by `atan2(ty, tx)` times a translation by `t/2`;
- `inputs:scaleX = 1 + L/(2S)` (the scales are divisors), so the off surface
  sits `S` beyond each joint on the axis and `S` across at the midpoint;
- `inputs:falloffMin = max(S/4, (L/2) S/(L/2 + S))`, so the fully-on inner
  ellipse passes through both joints and every point of the segment has
  weight 1;
- `inputs:falloffMax = S`.

A pin bone, with no joint child, has an unscaled sphere about the joint with
`inputs:falloffMin = S/4`. An ellipsoid does not keep a constant distance
from the segment: beside a joint the off surface sits only
`S * sqrt(1 - (L/2)^2 / (L/2 + S)^2)` across (0.745 S at `L = 4S`), so wide
art next to a joint can fall outside both neighbouring spheres; widen
`inputs:scaleY` or raise the strength where that matters. The sphere's
placement and shape are values, not a live end frame: whoever edits a bone's
segment or strength re-places its sphere and regenerates the rows.

The skin's rows are generated from those spheres rather than evaluated live.
`rigexec.binding.generate_skin_weights(stage, skin_mover, point_indices=None)`
writes them:

- **Points.** The mover's `rigExec:moves` target is read at Default and
  carried into the rig's asset space by its setup transform.
- **Fields.** For each influence in `rigExec:influences`, every direct
  volume weight child (`RigExecSphereWeight`, `RigExecPlaneWeight` or
  `RigExecCurveWeight`) is measured at its rest placement (no pose is
  evaluated) through the evaluator's packet builder, with the
  `inputs:scaleX/Y/Z`, falloff band, range policy and shape settings the
  evaluator uses (a sphere's half-axis scales, a plane's axis and bounds, a
  curve's points read at Default). Several volumes in one bone sum, clamped
  to 1; an influence with none contributes 0 and is reported
  (`influences_without_volumes`).
- **Rows.** Each point's fields are normalised over the influences, and
  `rigExec:jointIndices` index that list, so the influences need not be the
  skeleton's bones. A point outside every volume keeps a zero row and holds
  its rest. A row lists its non-zero influences in influence order, padded
  with index 0 and weight 0; `rigExec:elementSize` is the widest row, at
  least 1.
- **Partial rows.** With `point_indices`, only those rows are replaced; the
  others keep their authored slots and `rigExec:elementSize` is recomputed
  over the result. A tool writes rigid rows (one weight of 1) and released
  rows (zeros) itself and regenerates the rest this way.

Only `rigExec:jointIndices`, `rigExec:jointWeights` and `rigExec:elementSize`
are authored, at the stage's edit target. The rows go stale when a rest or a
bone's sphere changes, so the authoring tool re-places the sphere and
regenerates them after such an edit. A sphere bound as an ordinary mover's
`rigExec:weightObject` works as an envelope too, in 2D and 3D rigs alike.

[skeleton_2d.usda](../examples/skeleton_2d.usda) is a leg bound this way.

## Smart-bone step tables

A smart bone is a dial: its `avars:rz` picks a whole frame of an action whose
frames 0 to N were recorded at dial angles s_0 to s_N, and each frame carries
values for other properties (a bone's angle, an opacity, a switch index, a
corrective shape). RigExec has no node of its own for this. One
[Animated Maps](../nodes/animated_maps.md) node per action is a step table
over the dial, and property movers read it.

- **The node.** `rigExec:driverAttributes` targets the dial's `avars:rz` at
  `rigExecReadPhase "final"`, so the table reads the dial after every math
  mover on it. It has no `rigExec:meshes`: the values are read through
  connections, not published as primvars.
- **The rows.** Every [Animated Map](../nodes/animated_map.md) child carries
  the same slope-0 rows, one per cell of the dial, so each map publishes its
  `rowCut` for the action frame of the dial angle. A cell holds the angles
  closest to one sample value; outer cells run to `-FLT_MAX` and `FLT_MAX`,
  so the end frames hold past the recorded range. Two cells that share a
  bound both hold it, and the row listed first wins (see
  [Animated Maps](../nodes/animated_maps.md) for how the bound is rounded so
  an exact tie keeps its winner).
- **The side.** An action serves one side of s_0. A gate row first in the
  table, `[-FLT_MAX, s_0]` with cut 0 for the positive side or `[s_0,
  FLT_MAX]` for the negative one, keeps the other side and s_0 itself at
  frame 0.
- **`weight`.** A map whose cut at frame k is k / N in float: the
  activations of the [Blend Samples](../nodes/blend_sample.md) of a
  corrective, whose [Blend Input](../nodes/blend_input.md) connects
  `inputs:weight` to `weight.outputs:value`.
- **`active`.** A map that is 0 at frame 0 and 1 on every other frame: the
  envelope (`inputs:defaultWeight`) of a value channel's blend mover.
- **Channels.** One map per driven property, with `rigExec:clampOutput`
  false so that its value can leave [0, 1]. An offset channel's cut is the
  property's change since frame 0, read by an `add`
  [Float Math Mover](../nodes/float_math_mover.md) on the property; a value
  channel's cut is the value itself, read by a `blend` mover whose envelope
  is `active`. Past the last value the channel holds it.
- **A full turn.** A dial that turns one revolution reads its angle folded
  into the action's window, s_0 + posmod(angle - s_0, 360) for the positive
  side: float math movers (`blend`, `quantize`, `multiply`, `add`) on custom
  float attributes of the node fold it, and the maps' first input reads the
  folded attribute at `final`. A second input reads the raw dial and carries
  one point row at s_0 + 360 (s_0 - 360 for the negative side) whose cut is
  the difference between the last frame's value and frame 0's, so the dial's
  own end gives the last frame.
- **Reading outputs.** A reader declares `rigExecReadPhase "final"` on the
  connected input to read what the table computed in the same evaluation;
  see [Connected inputs](how-operators-fire.md#connected-inputs).
  Undeclared, it reads the map's authored 0. A dial that another action
  drives is chained through the dial's own property (that action's `add`
  mover revises it, and this table reads it at `final`), and it settles in
  one evaluation. Two actions that drive each other's dials form a cycle,
  which every evaluator sets aside.
- **Edits.** Rows and `rigExec:clampOutput` are compiled rig identity, so
  re-recording an action recompiles; turning the dial only re-evaluates.
  `inputs:enabled` false publishes 0 on every map: offset movers then add
  nothing and value channels' blends pass the property through.

The authoring tool writes the rows from the recorded samples; RigExec only
evaluates them, like any other animated map.

## Evaluation

The skeleton, the skin and the step tables are operations in the one
graph that [native, frozen and runtime evaluation share](baked-vs-dynamic.md).
Their kernels are USD-free headers in `libs/rigExecMath`, the same ones the
runtime compiles. Topology (point counts, bone order, channel order) is fixed
per compiled rig.

## Provenance

The blend, the per-joint descent and the other cited methods are in
[References](../references.md). The rest of the 2D rules have no public source;
see [2D skeleton rules without a public source](../references.md#2d-skeleton-rules-without-a-public-source).
