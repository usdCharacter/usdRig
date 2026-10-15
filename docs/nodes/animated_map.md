# ![Animated Map](../../icons/concept.png) Animated Map

*One conditional table of an Animated Maps node, and its value.*

| | |
|---|---|
| **Node type** | `RigExecAnimatedMap` |
| **Example** | [animated_maps.usda](../examples/animated_maps.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

One map of its parent `RigExecAnimatedMaps`: parallel arrays of
rows (`rigExec:rowInputs`, `rowFrom`, `rowTo`, `rowSlope`, `rowCut`) and the
primvar name its value is published under.

One map of its parent RigExecAnimatedMaps, a conditional table.
Each row reads input x = rigExec:driverAttributes[rowInputs[r]] and adds
rowSlope[r] * x + rowCut[r] when rowFrom[r] <= x <= rowTo[r] (a reversed
interval is swapped). Among consecutive rows on the same input only the
first whose interval holds x contributes, so intervals sharing an
endpoint count it once. The sum is clamped to [0, 1].

## How it works

Evaluated by its parent; see Animated Maps.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `outputs:value` | The map's value; connectable like a pose weight. | - |

## Parameters

### Node parameters

#### `rigExec:rowInputs`

*Type:* `uniform int[]`. *Default:* `[]`.

Each row's input, an index into the parent's inputs.

#### `rigExec:rowFrom`

*Type:* `uniform float[]`. *Default:* `[]`.

Each row's interval start.

#### `rigExec:rowTo`

*Type:* `uniform float[]`. *Default:* `[]`.

Each row's interval end.

#### `rigExec:rowSlope`

*Type:* `uniform float[]`. *Default:* `[]`.

Each row's slope.

#### `rigExec:rowCut`

*Type:* `uniform float[]`. *Default:* `[]`.

Each row's intercept.

#### `rigExec:primvarName`

*Type:* `uniform token`. *Default:* `""`.

The primvar this map publishes on the parent's meshes,
without the primvars: namespace. Empty uses the prim's name.

#### `outputs:value`

*Type:* `float`. *Default:* `0`.

The map's value for the current pose.

## Example

In the Animated Maps example `frown` has two rows on different
inputs, so both count and their sum is clamped; `foreheadL` and `foreheadR`
have one row each.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\animated_maps.usda
```

## Tips

- An empty `rigExec:primvarName` publishes under the map's prim name.
- [RigLogic behaviours](../concepts/riglogic.md) covers the tables and their provenance.

## See also

- [Animated Maps](animated_maps.md)

---

[UsdRig](../index.md)
