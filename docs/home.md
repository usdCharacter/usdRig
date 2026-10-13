---
title: Character rigs in OpenUSD
hide:
  - navigation
  - toc
---

<div class="rx-hero" markdown>
<div class="rx-hero__grid" markdown>
<div markdown>

<span class="rx-hero__eyebrow">OpenUSD 26.08 · 0.1 alpha</span>

# Character rigs that evaluate inside USD

<p class="rx-hero__lede">RigExec evaluates controls, solvers, constraints
and deformers in memory, without changing the stage, and publishes the
posed character through Hydra for live playback in <code>usdview</code>
and other viewers.</p>

[Get started](guide/getting-started.md){ .md-button .md-button--primary }
[Browse the nodes](index.md){ .md-button }

</div>
<div class="rx-hero__media" markdown>

![The biped in usdview with the hover picker and a right-click marking menu over the viewport](images/site/hero.png)

</div>
</div>
</div>

## What it does

<div class="grid cards" markdown>

-   :material-transit-connection-variant:{ .lg .middle } **Rigs as USD**

    ---

    A rig is a piece of namespace under one `RigExecRoot`. Where a prim
    sits decides when it runs, so the hierarchy reads like the evaluation
    order.

    [:octicons-arrow-right-24: How operators fire](concepts/how-operators-fire.md)

-   :material-speedometer:{ .lg .middle } **Dynamic and baked**

    ---

    Evaluate through OpenExec while authoring, or through a baked program
    built for playback. Both share the same math kernels and produce the
    same pose.

    [:octicons-arrow-right-24: Baked and dynamic](concepts/baked-vs-dynamic.md)

-   :material-vector-polyline:{ .lg .middle } **Solvers and deformers**

    ---

    FK, two-bone and spline IK, constraints, skinning, blend shapes,
    lattices, Delta Mush, wrinkles and more: each one a documented node.

    [:octicons-arrow-right-24: Node reference](index.md)

-   :material-gesture-tap:{ .lg .middle } **Animator tools**

    ---

    Pick controls from the character's own picker, from TouchPose regions
    on the skin, or from a right-click marking menu, with IK/FK switching
    that never moves the pose.

    [:octicons-arrow-right-24: Animator tools](tools/index.md)

-   :material-cached:{ .lg .middle } **Frame cache**

    ---

    Frames warm in the background and play back from memory, while edits
    reach the viewport immediately.

    [:octicons-arrow-right-24: What warming does](concepts/frame-cache-warming.md)

-   :material-package-variant-closed:{ .lg .middle } **Beyond usdview**

    ---

    Export a rig to the `.rigexec` binary and play it with a standalone
    runtime that links no USD library.

    [:octicons-arrow-right-24: Standalone runtime](specs/standalone-runtime.md)

</div>

## Every node, animated

Each node page carries a small animated stage that plays in `usdview`.
These are rendered straight from them.

<div class="rx-strip">
  <a href="nodes/two_bone_ik/"><img src="gifs/two_bone_ik.gif" alt="Two-bone IK" loading="lazy"><span>Two-bone IK</span></a>
  <a href="nodes/spline_ik/"><img src="gifs/spline_ik.gif" alt="Spline IK" loading="lazy"><span>Spline IK</span></a>
  <a href="nodes/curve_mover/"><img src="gifs/curve_mover.gif" alt="Curve mover" loading="lazy"><span>Curve mover</span></a>
  <a href="nodes/blendshape_mover/"><img src="gifs/blendshape_mover.gif" alt="Blend shape mover" loading="lazy"><span>Blend shapes</span></a>
</div>

## Two ways to evaluate, one pose

![Baked and dynamic execution share math kernels, diverge in scheduling and data storage, and produce the same rig pose.](images/baked-vs-dynamic-execution.png)

## Status

RigExec is an early, end-to-end vertical under active development: APIs,
behaviour and file formats may change without notice. It builds against an
unmodified OpenUSD 26.08 with OpenExec. Original project code is MIT
licensed; third-party material keeps its own terms.
