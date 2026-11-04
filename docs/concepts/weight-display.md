---
title: Weight display in usdview
summary: Choose volume guides and surface color mappings for resolved weight fields.
order: 75
---

Open **RigExec → Animation Editors → Volume Weight Editor**, select a weight
object, and choose its `guide:drawMode` in the parameter panel:

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


## Editing weights

Open **RigExec > Animation Editors > Volume Weight Editor** to edit a weight.
The Weight tab contains its inner and outer boundaries, strength, inversion
and falloff profile. Shape / placement contains axis scales and placement.
Display / options contains guide styling and surface coloring. The influence
overlay remains independent of selection.

While the editor is open, drag a drawn inner shell or outer cage with the left
mouse button to expand or contract that boundary. Shift-drag a sphere to
change the nearest positive or negative axis scale; Shift-drag a bounded
plane to resize its nearest extent. These are the six half-axis controls
supported by sphere weights. Alt and Meta remain camera gestures. Escape
cancels an active drag.

Numeric and viewport drags send uncommitted values through the Hydra preview
channel, coalesced at 16 ms intervals. On release, one undoable stage commit
is queued on the owning UI thread, using the edit target and time captured
at the start of the drag. Closing the editor cancels active gestures.

Custom curve knots are edited in a detached spline. During the drag, sampled
falloff values update the surface influence through Hydra and the evaluation
graph. Release commits the spline once; Escape restores the original influence.
The temporary samples are never authored to the stage.

The optional `inputs:falloffSamples` array also works as a sampled graph input.
An empty array uses the authored profile. `.rigexec` format 27 carries it as a
runtime input; a file from formats 20–26 plays with its compiled falloff table.
