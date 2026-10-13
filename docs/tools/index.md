---
title: Animator tools
---

# Animator tools

RigExec adds its tools to `usdview` under one **RigExec** menu. They all work
on usdview's own selection, so picking a control anywhere (the viewport, a
picker, TouchPose, the outliner) is picking it everywhere: the gizmo, the Avar
Editor and the marking menu all follow.

Interactive edits reach the viewport straight away and are written to the
stage when the gesture ends, as one undoable edit.

## Selecting

<div class="grid cards" markdown>

-   :material-gesture-tap-button:{ .lg .middle } **[Control picker](picker.md)**

    ---

    The character's own picker, docked in a panel or floating over the
    viewport as the **hover picker** (++p++).

-   :material-chart-donut:{ .lg .middle } **[Marking menu](marking-menu.md)**

    ---

    Right-click a control for a radial menu: key, reset, zero, frame,
    switch IK/FK, change space, select the mirrored control.

-   :material-hand-pointing-up:{ .lg .middle } **[TouchPose](touchpose.md)**

    ---

    Touch the character's skin to select the control that moves it.

-   :material-arm-flex:{ .lg .middle } **[Limbs](limbs.md)**

    ---

    IK/FK switching that never moves the pose, pole-vector pinning,
    stretch and twist.

</div>

## Posing and editing

| Tool | Where | What it is for |
|---|---|---|
| [Viewport gizmos](../specs/viewport-gizmos.md) | **RigExec ▸ Viewport ▸ Viewport Tools** | Move, rotate and scale controls in the viewport (++q++ ++w++ ++e++ ++r++). |
| Avar Editor | **RigExec ▸ General Editors ▸ Avar Editor** | Every animation channel on the selected control, as sliders and fields. |
| [Graph editor](../specs/graph-editor.md) | **RigExec ▸ Animation Editors ▸ Graph Editor** | Animation curves of the selected channels. |
| [View cube](../specs/view-cube.md) | **RigExec ▸ Viewport ▸ View Cube** | Orbit to a named view, and back home. |
| Volume weights | **RigExec ▸ Animation Editors ▸ Volume Weight Editor** | Shape procedural weight fields. See [volume weights](../specs/volume-weights.md). |

## Inspecting

| Tool | Where | What it is for |
|---|---|---|
| Layer Stack and Layer Opinions | **RigExec ▸ General Editors** | Which layer an opinion comes from, and which one an edit lands in. |
| Exec Stack | **RigExec ▸ General Editors ▸ Exec Stack** | The evaluated movers on a prim, in order. |
| Deformer Toggles | **RigExec ▸ General Editors ▸ Deformer Toggles** | Switch deformers on and off to see what each contributes. |
| Profiler | **RigExec ▸ General Editors ▸ Profiler** | Where a frame's time goes, and how many threads it ran on. |
| Cache Strip | **RigExec ▸ General Editors ▸ Cache Strip** | Which frames are warm in the cache. See [frame cache warming](../concepts/frame-cache-warming.md). |
