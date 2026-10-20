# ![Harmonics Constraint](../../icons/concept.png) Harmonics Constraint

*Secondary motion: overshoot, lag and jiggle of one joint or control from the frames before the current one.*

| | |
|---|---|
| **Node type** | `RigExecHarmonicsConstraint` |
| **Example** | [harmonics_constraint.usda](../examples/harmonics_constraint.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

![Harmonics Constraint effect](../gifs/harmonics_constraint.gif)

Adds follow-through to one joint or control. Each change
in the target's velocity over the last few frames rings as a damped sine, and
the rings still sounding are added to the target's translation. A tail tip
overshoots when the body stops, an antenna bobs after a hop, a belly jiggles
after a step. The result depends only on the frames in a fixed window before
the current one, so scrubbing, playback, a background cache and a `.rigexec`
reader all agree. While a control is dragged, the earlier frames are the
authored animation and the current frame is the drag, so the target is held
back against the drag; the ring plays when time moves.
See [Secondary motion and cloth](../concepts/secondary-motion.md) for how the
window is evaluated, and [algorithm references](../references.md) for the
method's source.

Secondary motion for one joint or control: overshoot, lag and
jiggle from the way it moved over the frames before the current one.

The target's incoming position is measured in rigExec:space (the rig
root when none is named) at the current frame and at each earlier frame
of a fixed window. Every earlier frame's acceleration rings as a damped
sine, and the sum of the rings still sounding is added to the target's
translation, scaled by the MoverAPI envelope. The window is
rigExec:waves * rigExec:waveLength / rigExec:frequency frames, so the
result depends only on the frames in it: scrubbing, playback and a
background cache all agree. While a control is dragged the earlier
frames are the authored animation and the current frame is the dragged
pose, so the ring answers the drag.

The method is the harmonic solver of BlurHarmonics
(docs/references.md); see docs/concepts/secondary-motion.md.

## How it works

The constraint reads its target's incoming position, measured
in `rigExec:space` (the rig root when none is named), at the current frame and
at each of the `rigExec:waves * rigExec:waveLength / rigExec:frequency` frames
before it. Those earlier frames are generations of the same compiled graph at
those times, run only as far as the target's inputs reach, and they read the
authored stage. From the positions the kernel (`harmonicsKernel.h`) takes each
frame's acceleration and adds its ring: a sine of `waveLength / frequency`
frames per period under an exponential envelope set by `inputs:decay`, ending
at `inputs:termination` of the amplitude. `rigExec:matchVelocity` scales the
ring so an impulse rings with the velocity change that caused it;
`inputs:amplitude` and `inputs:axisAmplitude` scale it, per axis of the space.
The sum is added to the target's translation and blended by the common mover
envelope. Motion of the space itself does not ring, so a jiggle measured in
the hips follows a walk and answers only the motion relative to them. A
`.rigexec` reader keeps the same window of its own frames: a host pushes each
frame it plays, or replays the frames before a jump.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:moves` | The one joint or control whose translation rings. | yes |
| `rigExec:space` | A control or joint the motion is measured in. | no |

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

#### `rigExec:space`

*Relationship.*

Optional single RigExecControl or RigExecJoint whose frame the
motion is measured in. Motion of the space itself does not ring: a
jiggle measured in the hips follows a walk and answers only the
motion relative to them.

#### `rigExec:waves`

*Type:* `uniform double`. *Default:* `5`.

How many oscillations one impulse rings for.

#### `rigExec:waveLength`

*Type:* `uniform double`. *Default:* `5`.

Frames per oscillation.

#### `rigExec:frequency`

*Type:* `uniform double`. *Default:* `1`.

Phase frames one frame advances. Above 1 the ring is faster
and the window shorter; below 1 it is slower and longer.

#### `rigExec:ignoreFirstFrame`

*Type:* `uniform bool`. *Default:* `true`.

When the history is shorter than the window (a .rigexec
reader's first frames), drop the jolt of starting to move from
rest.

#### `rigExec:matchVelocity`

*Type:* `uniform bool`. *Default:* `true`.

Scale the amplitude by waveLength / (2 pi), so an impulse
rings with the velocity change that caused it. Off uses
inputs:amplitude as the ring's amplitude.

#### `inputs:amplitude`

*Type:* `double`. *Default:* `1`.

Overall multiplier on the ring.

#### `inputs:axisAmplitude`

*Type:* `double3`. *Default:* `(1, 1, 1)`.

Per-axis multipliers, along the axes of rigExec:space.

#### `inputs:decay`

*Type:* `double`. *Default:* `3`.

How fast a ring dies away; 0 rings at full amplitude until it ends.

#### `inputs:termination`

*Type:* `double`. *Default:* `0`.

The fraction of the amplitude a ring has left when it ends.

## Example

`harmonics_constraint.usda` puts a bob above a base that dashes
left and right. The `Tip` control rides the `Root` control; the constraint
rings `Tip` with four waves of six frames each, so the bob swings past the base
at every stop and settles within a second. Dragging `Root` in usdview holds
the bob back against the drag, further the further it goes, since the earlier
frames stay the authored ones; the ring itself plays when time moves.

`examples/19_HarmonicsChain.usda` turns rings into rotation down a chain: each
segment of a four-joint stalk aims at a target control one segment higher,
and a harmonics constraint rings each target, larger and slower up the stalk.
The jiggles sit below the aim constraints in the stack, and the aims run from
the base up, so the stalk leans behind each dash, whips past the stop and
settles with its segment lengths unchanged.

![examples/19_HarmonicsChain.usda: rung aim targets bend the stalk behind each dash and whip it past the stop](../gifs/harmonics_chain.gif)

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\harmonics_constraint.usda
```

Re-render the GIFs on this page with:

```bat
python docs/render_media.py --page harmonics_constraint
```

## Tips

- Window length is waves times waveLength over frequency: longer rings read more earlier frames.
- To ring a chain's rotation, ring aim targets and aim each joint at the next one up; keep the jiggles below the aims in the stack.
- Measure a jiggle in the part it hangs from (rigExec:space) so that part's own motion is not repeated.
- Use inputs:axisAmplitude to keep a ring in one plane, for a pendulum or a door.
- The first frames of a .rigexec reader have less history; rigExec:ignoreFirstFrame drops the jolt of starting from rest.

## See also

- [Cloth Mover](cloth_mover.md)
- [Aim Constraint](aim_constraint.md)
- [Control](control.md)

---

[UsdRig](../index.md)
