# ![Action Channel](../../icons/concept.png) Action Channel

*One driven channel of a driven action: its value at each whole action frame.*

| | |
|---|---|
| **Node type** | `RigExecActionChannel` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A child of a [Driven Action](driven_action.md). `rigExec:samples`
holds the channel's value at whole action frames 0, 1, ...; past the last sample
the last value holds. `outputs:value` is what readers connect to.

One driven channel of its parent RigExecDrivenAction:
rigExec:samples holds the channel's value at whole action frames
0, 1, ...; past the last sample the last value holds.

## How it works

In `offset` mode the channel publishes `samples[f] - samples[0]`, for an
additive mover. In `value` mode it publishes `samples[f]`, for a blend mover
whose envelope is the action's `outputs:active`. The channels are compiled in
child order, so adding or reordering a channel recompiles.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `outputs:value` | The channel's value; connectable like a pose weight. | - |

## Parameters

### Node parameters

#### `rigExec:samples`

*Type:* `uniform double[]`. *Default:* `[]`.

#### `rigExec:channelMode`

*Type:* `uniform token`. *Default:* `"offset"`.

Valid values: `offset`, `value`.

offset publishes samples[f] - samples[0], for an additive
mover; value publishes samples[f], for a blend mover whose envelope
is the action's outputs:active.

#### `outputs:value`

*Type:* `float`. *Default:* `0`.

## Example

A channel with samples [0, 5, 12] in `offset` mode publishes 0, 5, 12 at
frames 0, 1, 2 and 12 beyond; in `value` mode with samples [1, 0.5, 0] it
publishes the samples themselves.

## Tips

- Connect with `rigExecReadPhase = "final"` to read the value computed in the same evaluation.
- An action with no channels still publishes frame, weight and active.

## See also

- [Driven Action](driven_action.md)

---

[UsdRig](../index.md)
