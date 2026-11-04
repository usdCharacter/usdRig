# ![Driven Action](../../icons/concept.png) Driven Action

*Maps a dial angle to a whole action frame and publishes driven channel values.*

| | |
|---|---|
| **Node type** | `RigExecDrivenAction` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A driven action reads a driver property, normally a dial bone's
`avars:rz`, and finds the whole action frame whose recorded driver sample is
closest. It publishes the frame, a normalised weight, an activity flag, and
the values of its `RigExecActionChannel` children at that frame. It is not a
mover: readers connect to its outputs.

Maps a dial angle to a whole action frame and publishes values
at that frame. Not a mover: readers connect to its outputs and to its
RigExecActionChannel children's outputs:value.

rigExec:driverSamples holds the driver property's value (a dial's
avars:rz: degrees from its setup angle) at action frames 0, 1, ..., N.
The action serves one side of sample 0 (rigExec:side); on the other
side, and at sample 0, the frame is 0. On its side the frame is the one
whose sample is closest to the dial angle, clamped to [0, N]. A driver
turning one full revolution wraps an angle outside its range onto the
action's side first.

## How it works

`rigExec:driverSamples` holds the driver value at action frames 0 to N. The
action serves one side of sample 0 (`rigExec:side`); on the other side, and at
sample 0, the frame is 0. A driver that turns one full revolution wraps an angle
outside its range onto the action's side first. Equal distances take the earlier
or the later frame by `rigExec:frameTies`. A disabled action publishes frame 0.

A reader that connects to `outputs:value`, `outputs:weight` or `outputs:active`
and wants the value computed in the same evaluation declares
`rigExecReadPhase = "final"` on the connected input; the reader then runs after
the action. Undeclared, the connection reads the authored value. Two
interpolators that drive each other's inputs this way form a cycle, and every
evaluator sets it aside. The driver sample table is structural: an edit
recompiles.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:driverAttributes` | Exactly one float or double property, normally a bone's `avars:rz`. | yes |

## Parameters

### Node parameters

#### `rigExec:driverAttributes`

*Relationship.*

Exactly one float or double property, normally a bone's
avars:rz, read at this relationship's rigExecReadPhase.

#### `rigExec:driverSamples`

*Type:* `uniform double[]`. *Default:* `[]`.

Driver value at each whole action frame, in the driver
property's own units; at least two.

#### `rigExec:side`

*Type:* `uniform token`. *Default:* `"fromSamples"`.

Valid values: `fromSamples`, `positive`, `negative`.

The side of sample 0 this action serves. Tools author it
from the pair rule of a dial's two actions; fromSamples judges it
from this action's samples alone.

#### `rigExec:frameTies`

*Type:* `uniform token`. *Default:* `"earlier"`.

Valid values: `earlier`, `later`.

Which frame wins when two samples are equally close.

#### `inputs:enabled`

*Type:* `bool`. *Default:* `true`.

A disabled action publishes frame 0.

#### `outputs:frame`

*Type:* `float`. *Default:* `0`.

The whole action frame.

#### `outputs:weight`

*Type:* `float`. *Default:* `0`.

outputs:frame / N, for blend inputs whose samples sit at k / N.

#### `outputs:active`

*Type:* `float`. *Default:* `0`.

1 when outputs:frame > 0, else 0.

## Example

A wrist dial's `avars:rz` drives a hand-pose action with driver samples
[0, 20, 45, 90]. At 50 degrees the closest sample is 45, so the action frame is 2.
A channel in `offset` mode connected to a float math mover's input adds the
channel's change since frame 0, and a channel in `value` mode with
`outputs:active` as a blend envelope switches a drawing on.

## Tips

- `rigExec:side` is normally authored from a dial's pair of actions; `fromSamples` judges it from the action's own samples.
- `outputs:weight` is the frame divided by N, for blend inputs whose samples sit at k / N.
- See [2D skeletons, binding and driven actions](../concepts/skeletons-2d.md#driven-actions).

## See also

- [Action Channel](action_channel.md)
- [Pose Interpolator](pose_interpolator.md)
- [Float Math Mover](float_math_mover.md)

---

[UsdRig](../index.md)
