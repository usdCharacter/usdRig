# ![ML Output](../../icons/concept.png) ML Output

*One output of an ML mover's network, and the float it publishes.*

| | |
|---|---|
| **Node type** | `RigExecMlOutput` |
| **Example** | [ml_mover.usda](../examples/ml_mover.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Each `RigExecMlOutput` child of a `RigExecMlMover` is one of its
network's outputs, in namespace order. Its `outputs:weight` is the network's
value for the current pose after the mask; a blend input's `inputs:weight`
connects to it, as it would to a pose's weight.

One output of its parent RigExecMlMover's network, in namespace
order among its RigExecMlOutput siblings, and the float it publishes.

## How it works

Published by the ML mover's network step with every other pose
weight, so any consumer that reads a pose weight reads it the same way.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `outputs:weight` | Read by a blend input's `inputs:weight`; carries no authored connection itself. | - |

## Parameters

### Node parameters

#### `outputs:weight`

*Type:* `float`. *Default:* `0`.

The network's output for the current pose, after the mask.
A RigExecBlendInput's inputs:weight connects to it.

## Example

The ML mover's example has two outputs, `bicep` and `crease`, in
that order; the blend inputs `bicepMorph` and `creaseMorph` connect to them.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\ml_mover.usda
```

## Tips

- Add outputs in the network's output order; the builder's `AddMorph` does it with the blend input and sample.
- [RigLogic behaviours](../concepts/riglogic.md) covers the network and its provenance.

## See also

- [ML Mover](ml_mover.md)

---

[UsdRig](../index.md)
