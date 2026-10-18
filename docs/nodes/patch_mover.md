# ![Patch Mover](../../icons/concept.png) Patch Mover

*Sticks points onto a NURBS patch at controller-drivable UV coordinates.*

| | |
|---|---|
| **Node type** | `RigExecPatchMover` |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

Evaluates a `UsdGeomNurbsPatch` surface at normalized `(u, v)` coordinates
read from the XY components of a `UsdGeomPointBased` UV input prim and writes
the evaluated surface positions to the target's `points` property. Because
`rigExec:uvPoints` and `rigExec:patch` both honor `rigExecReadPhase` metadata,
controllers can drive the UV input points or deform the NURBS patch through
earlier movers in the stack.

UV shapes mix into the UV points before the patch is evaluated:
`rigExec:uvShapes` lists them and `inputs:uvWeights` holds one weight per
shape, by index -- typically connected, in shape order, to a control's avars
(smile, frown, mouthOpen; upperOpen, upperClosed). Shapes add to the neutral
UV points, so sliders mix, and the mix happens in UV space, so in-between
positions follow the surface rather than cutting through it.

Contact pairs keep two curves on one patch apart. `rigExec:uvContactLower`
and `rigExec:uvContactUpper` pair UV points by index, and after the shapes
mix no lower point stands above its upper partner in v: a pair that would
cross meets at the midpoint of the two. A lower lip that mouthClose lifts
past a closed upper lip, or a lower lid that rises into a closing upper lid,
presses against it instead of passing through. Additive shapes alone cannot
promise that: jawOpen plus mouthClose must close the lips, so mouthClose
alone moves them past each other.

Evaluates a UsdGeomNurbsPatch at normalized UV parameters read from
a point-based UV input prim and writes an exact native UsdGeomPointBased
points property. Both rigExec:patch and rigExec:uvPoints honor
rigExecReadPhase metadata, so controllers can deform the patch or drive the
UV input points through earlier movers. Weighted UV shapes mix into the UV
points before the patch is evaluated, so avars such as smile, frown or
mouthOpen can slide the attachment across the surface together. Contact
pairs then keep paired points from crossing in v, so two curves on one
patch -- upper and lower lips, upper and lower lids -- press together
rather than pass through each other under any mix of shapes.

## How it works

The mover reads the NURBS patch's `points`, optional `pointWeights`,
`uVertexCount`, `vVertexCount`, `uOrder`, `vOrder`, `uKnots`, `vKnots`, `uForm`
and `vForm`, together with the `points` array named by `rigExec:uvPoints` at
each relationship's declared `rigExecReadPhase`. The UV shapes, each read at
`rigExec:uvShapes`' phase with as many points as the neutral, then add to it:
`uv = neutral + sum_k w_k (shape_k - neutral)`, accumulated in double, where
`w_k` is the value of the k-th source `inputs:uvWeights` connects to (a float
or a double), or with no connection the k-th authored entry (an empty array
leaves every shape at zero). Weights are not clamped. Each contact pair `k`
then compares the v of point `rigExec:uvContactLower[k]` with the v of point
`rigExec:uvContactUpper[k]`; when the lower one is higher, both take the mean
of the two, and u is left alone. For each target vertex (or
broadcast from a single UV point when `uvPoints` has length 1), normalized
parameters `(u, v) = (uvPoint.x, uvPoint.y)` are clamped to `[0, 1]` for `open`
form or wrapped into `[0, 1)` for `closed`/`periodic` form, mapped into the
active knot spans `[knots[order - 1], knots[vertexCount]]`, and evaluated by
tensor-product Cox-de Boor basis functions (rational when `pointWeights` is
authored). The common `RigExecMoverAPI` envelope blends the candidate surface
positions over the incoming point revision.

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:patch` | Native `UsdGeomNurbsPatch` prim supplying the NURBS surface (`base` or `final` read phase). | yes |
| `rigExec:uvPoints` | Native `UsdGeomPointBased` prim or `.points` property whose XY coordinates supply normalized `(u, v)` parameters (`final` when controller-driven); the neutral the UV shapes are measured from. | yes |
| `rigExec:uvShapes` | Ordered UV shapes: `UsdGeomPointBased` prims or `.points` properties with as many points as `rigExec:uvPoints`. | no |
| `inputs:uvWeights` | One weight per UV shape, by index: connected in shape order to float or double sources (avars), or an authored `float[]`. | no |
| `rigExec:uvContactLower` / `rigExec:uvContactUpper` | Index-aligned `int[]` contact pairs: UV point `lower[k]` never rises above UV point `upper[k]` in v. | no |
| `rigExec:moves` | Exact native `point3f[]` points property to stick onto the patch. | yes |

## Parameters

### Node parameters

#### `rigExec:patch`

*Relationship.*

Native UsdGeomNurbsPatch prim supplying the NURBS surface.

#### `rigExec:uvPoints`

*Relationship.*

Native UsdGeomPointBased prim or points property whose XY
coordinates supply normalized (u, v) parameters on the patch: the
neutral the UV shapes are measured from.

#### `rigExec:uvShapes`

*Relationship.*

Ordered UV shapes: native UsdGeomPointBased prims or points
properties, each with as many points as rigExec:uvPoints. Shapes add
to the neutral, uv = neutral + sum_k w_k (shape_k - neutral), with
w_k the k-th entry of inputs:uvWeights.

#### `inputs:uvWeights`

*Type:* `float[]`. *Default:* `[]`.

One weight per rigExec:uvShapes target, by index. Connect it
to one float or double source per shape, in shape order -- a
control's avars, typically -- and connection k is the weight of
shape k. With no connection the authored array is the weights; an
empty array leaves every shape at zero.

#### `rigExec:uvContactLower`

*Type:* `uniform int[]`. *Default:* `[]`.

UV point indices that never stand above their partners in
rigExec:uvContactUpper, by index, after the UV shapes mix. A pair
that would cross meets at the midpoint of its two v values; u is
left alone. Pair one column of a lower curve with the same column
of an upper curve, sharing u in the neutral and in every shape, and
the two curves touch but never cross. Each point belongs to at most
one pair.

#### `rigExec:uvContactUpper`

*Type:* `uniform int[]`. *Default:* `[]`.

The partner of each rigExec:uvContactLower index: the UV
point that point never rises above. As long as
rigExec:uvContactLower.

## Example

Author a `UsdGeomNurbsPatch` prim and a `UsdGeomPoints` UV carrier prim whose
points lie in `[0, 1] x [0, 1]`. Drive the UV carrier's `points` with a
`RigExecMatrixMover` or `RigExecSkinMover` bound to a `RigExecControl`, then
wire `rigExec:patch` to the patch and `rigExec:uvPoints` (with
`rigExecReadPhase = "final"`) to the UV carrier.

`examples/17_PatchCurveShapes.usda` mixes UV shapes instead: the mover's
`rigExec:uvShapes` names four UV sets, and `inputs:uvWeights` connects to a
control's four float avars in the same order.

`examples/18_PatchMouth.usda` puts two curves on one patch: upper and lower
lips in one `BasisCurves`, mixed by the 27 ARKit jaw and mouth blend-shape
avars, with one contact pair per lip column so the lips touch but never
cross.

## Tips

- Set `rigExecReadPhase = "final"` on `rigExec:uvPoints` when controllers move the UV points through earlier movers.
- Set `rigExecReadPhase = "final"` on `rigExec:patch` when the NURBS patch control vertices are themselves deformed by the rig.
- A single-element `uvPoints` array broadcasts its `(u, v)` coordinate to every target point.
- Connection k of `inputs:uvWeights` weights shape k of `rigExec:uvShapes`: keep the two lists in the same order, and as long as each other.
- Shapes add to the neutral and are not clamped, so a weight past 1 exaggerates a shape and a negative one reverses it.
- Contacts move v only. Give each pair one u in the neutral and in every shape -- the same column of both curves -- and the curves cannot cross between columns either.
- A crossing pair meets at its midpoint, so a closing shape that moves both curves equally toward each other (mouthClose) leaves closed lips where they were.

## See also

- [Surface Mover](surface_mover.md)
- [Surface Binding Mover](surface_binding_mover.md)
- [Curve Mover](curve_mover.md)
- [Blend Shape Mover](blendshape_mover.md)

---

[UsdRig](../index.md)
