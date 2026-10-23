---
title: Weight display in usdview
summary: Choose volume guides and surface color mappings for resolved weight fields.
order: 75
---

Open **RigExec → Volume Weight Authoring**, select a weight object, and
choose its `guide:drawMode` in the parameter panel:

| Mode | Drawing |
| --- | --- |
| `falloff` (default) | Soft inner shell with continuous rings; dashed exterior cage |
| `cage` | Continuous inner boundary, dashed outer boundary, no fill |
| `brackets` | Short sections at cardinal positions on both boundaries |
| `shell` | Continuous boundary curves and a faint outer surface |
| `wire` | Original drawing, including the sphere's dotted rings and filled interior |
| `geometry` | Filled inner and outer surfaces |
| `none` | No volume guide; the surface overlay remains available |

The new modes apply to spheres, planes and curves. They use 30% of
`guide:wireWidth`; zero selects hairlines. Curve cages retain all bends in
their longitudinal rails but draw at most five section rings. Plane ticks
indicate an unbounded field. An inner radius of zero has no inner surface.
Shells use one quarter of the guide opacity. The default `falloff` mode
fills only the inner boundary; the exterior falloff stays unfilled.
Explicitly authored modes remain in effect. These are world-space guides:
line thickness varies with zoom, and normal viewport depth testing applies.

Under **Influence overlay**, choose the weight to paint and enable
**Show influence**. The selected object's **Surface display** parameter
(`guide:weightDisplay`) chooses the color mapping:

| Mode | Mapping |
| --- | --- |
| `tint` | Gray at zero, red at one |
| `heatmap` | Blue, cyan, green, yellow, red at 0, .25, .5, .75, 1 |
| `bands` | Five colors for [0,.2), [.2,.4), [.4,.6), [.6,.8), [.8,1] |

Colors come from the resolved field consumed by a mover. Unbound weight
objects have guides but no resolved surface overlay. Display clamps weights
to [0,1] without changing their numerical values. Surface colors interpolate
between vertices, so bands are approximate on coarse meshes; they are not
geometric iso-contour lines. Source materials and opacity remain intact when
the overlay is disabled. Choosing `none` with an overlay gives a surface-only
view. The painting target remains independent of viewport selection.

Style choices are authored by the editor and participate in undo. Evaluation
and Hydra publication do not author results onto the stage.
