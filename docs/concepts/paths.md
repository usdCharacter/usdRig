---
title: Paths and walking
summary: Drive a rig along a BasisCurves path by arc length, author and edit the path, and steer a baked program with a moving window.
order: 70
---

A path is an ordinary `UsdGeomBasisCurves` prim. Float math movers with
`rigExec:operation = "path"` sample it by arc length, so a rig can walk,
crawl or slide along it. The path is evaluated inside the shared operation
graph like any other mover input: the native, frozen and binary evaluators
run the same kernel (`libs/rigExecMath/pathCurveKernel.h`), and nothing is
precomputed into tables.

## The operations

`path` samples the curve its `rigExec:path` relationship names. The curve
holds one curve, `linear` or `cubic` with `bezier`, `bspline` or
`catmullRom` basis, `nonperiodic` or `pinned` wrap. Its points are read in
the rig's space. The arc length sampled is `incoming − inputs:value`,
clamped to the curve: `inputs:value` is the distance the curve's first
point stands at. `rigExec:pathComponent` chooses the result:

| Component | Result |
| --- | --- |
| `x`, `y`, `z` | The sampled point, plus `rigExec:pathOffset` (local x, z) rotated by the heading |
| `heading` | The planar tangent's yaw about +Y in degrees, plus `rigExec:pathHeadingOffset`, wrapped to (−180, 180] |
| `end` | `inputs:value` plus the curve's whole length |

The heading is the rotation that takes local +Z to the tangent, the
convention a control's `ry` avar uses. A point whose tangent has no planar
direction takes its segment's chord, then an earlier segment's.

`rigExec:pathMeasure` chooses the distance: the curve's arc length
(`length`, the default) or the arc length of its projection on the XZ plane
(`planar`). A walk measures ground distance, so a path carrying terrain
height in its points' y keeps the same strides, and a host that counts its
travel on the ground keeps the same clock as the rig.

`quantize` rounds the incoming value down to `inputs:min + k × inputs:value`
for the largest integer `k`: a gait cycle's start at any distance, where a
key table would end.

## A walking rig

A six-legged walker, for example, drives its gait from three Root avars:
`pathDistance` (the distance clock), `pathStart` (the distance at the
path's first point) and `pathStop` (where the walk may end). Each leg's
cycle start is a `quantize` of the distance. Its previous and next landings,
and its origin under the body, are `path` samples at the leg's gated
distances with the leg's local offset. The swing blends between the
landings, and the difference from the origin moves the foot's IK control.
Root's `tx`, `tz` and `ry` follow the path at the current distance, and its
`ty` rides the path's height, so a path that carries ground height in its
points' y walks over terrain; every path mover measures `planar`, and each
landing takes the path's height at its own distance (under the path, not
beside it). The end-fade lowers the feet within the last
part of a stride before `min(path end, pathStop)`.

A foot's landing is a function of distance alone, so a planted foot holds
still while the body walks on. Its heading is the path's tangent at the
landing, so the body faces its travel; a fixed `pathHeadingOffset` turns it
to walk sideways or backwards.

## Authoring and editing

Animate a walk by keying the distance avar; edit the route by moving the
curve's points, in any DCC or in usdview. In usdview, selecting a path that
a rig's `path` mover names shows a handle at each point; dragging one moves
it horizontally and previews the rig live through the interactive override
lane (`point3f[]` preview slots), and releasing commits the points as one
undoable edit. Nothing is authored while dragging.

`catmullRom` passes through its points, which suits an authored route.

## A moving window

A baked program lists the path's points as a `point3f[]` input of fixed
count. A game steers the rig by rewriting them every frame: a trail of the
positions it has walked plus a few points predicted ahead. It also sets
`pathStart`, `pathDistance` and `pathStop`. Measured on a walker, the
choices that keep planted feet still are:

* Space the points evenly: record the trail at exact arc steps and predict
  at the same step. A uniform catmullRom neither overshoots nor lets a
  changed prediction reach more than one segment back. A point that moves
  every frame (the current position) beside a recorded one makes uneven,
  even zero-length, segments, and a uniform catmullRom doubles back there.
* Predict with the same steering model the walk integrates (a walker can use
  curvature that eases over distance walked, in exact arc steps on a
  millimetre grid), so a held input is predicted exactly. What remains is
  the change of input while a foot stands on a landing just ahead.
* Keep the prediction while the clock holds still, and bring `pathStop` to
  the stopping distance, so feet settle where they are; let it run out again
  before the clock moves.
* Rebase the distance clock by whole gait periods when it grows large; the
  cycle stairs and the trail's distances move together.

The binary runtime re-runs the path revisions when only the points change:
their points are part of the step's input memo, like any declared input.
