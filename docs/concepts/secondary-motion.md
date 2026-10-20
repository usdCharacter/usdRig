---
title: Secondary motion and cloth
summary: How the harmonics constraint and the cloth mover read the frames before the current one, inside the one evaluation loop, live, in background caches and in .rigexec playback.
order: 36
---

Two operators answer motion rather than pose:
[Harmonics Constraint](../nodes/harmonics_constraint.md) rings a joint or
control with the way it moved, and [Cloth Mover](../nodes/cloth_mover.md)
simulates a mesh hanging from and colliding with the character. Both are
*windowed* operations: the result at a frame is a pure function of their
inputs over a fixed number of frames before it and the frame itself. Nothing
is carried from one evaluation to the next, so a frame looks the same whether
it was reached by playing forward, scrubbing backward, jumping, a background
cache, or a `.rigexec` reader.

![Harmonics Constraint: the bob overshoots each stop of its base and settles](../gifs/harmonics_constraint.gif)

![Cloth Mover: the curtain swings with its bar and drapes over the ball rolling through it](../gifs/cloth_mover.gif)

Both are rendered live from the node pages' examples,
`docs/examples/harmonics_constraint.usda` and `docs/examples/cloth_mover.usda`,
at the stage's frame rate; the rest pose is drawn in blue.

## The window

At every frame it runs, a windowed operation captures a *packet*: the inputs
its solve reads, and nothing computed from earlier frames. For the harmonics
constraint that is the target's incoming position in its measuring space; for
the cloth, the points entering the mover, every collider's points and the
parameters. The solve then reads the packets of the earlier frames, oldest
first, and the current one:

- the harmonics constraint adds, for each earlier frame, that frame's
  acceleration rung as a damped sine (the method of BlurHarmonics; see
  [references](../references.md)). Its window is
  `rigExec:waves * rigExec:waveLength / rigExec:frequency` frames, 25 by
  default.
- the cloth starts on its incoming points at the oldest frame, moving with
  them, and steps to the current one: `rigExec:substeps` steps per frame of
  extended position-based dynamics with coloured Gauss-Seidel constraints,
  tethers to the pins, and collision against the colliders' triangles swept
  between frames. Its window is `rigExec:windowFrames` frames, 16 by default.

## Earlier frames are generations of the same graph

The packets of the earlier frames come from *history ticks*: runs of the same
compiled operation graph at those times, masked to the windowed operations and
every operation they depend on. A history tick does not publish anything, and
operations outside the mask keep what they owe the next full run. There is no
second evaluator and no separate pre- or post-pass: history is produced by the
one loop, by the operations that produce the current frame.

The evaluator keeps a memo of captured packets per frame, keyed by the
program, the stage's edit serial and the upstream inputs. Playing forward
costs no history tick per frame, since each frame's own capture becomes the
next frame's history; a jump runs the ticks the memo lacks. An edit that
reaches the history clears the memo. The ring of earlier packets is placed at
the boundary before a generation, like any sampled input, and a changed ring
re-runs the windowed operation.

Inner windows chain: a windowed operation that reads another one's result
solves at the history ticks too, with its own ring, and the history reaches
back as far as the chain needs.

## Rings as rotation

The harmonics constraint rings a translation. To ring a chain's rotation,
ring targets and aim the joints at them:
[example 19](../../examples/19_HarmonicsChain.usda) aims each segment of a
stalk at a target control one segment higher and rings each target, larger
and slower up the stalk, so the stalk bends behind a move and whips past the
stop with its segment lengths unchanged. The stack runs bottom first: the
harmonics constraints, so the aims read the rung targets; whatever carries
the chain's base joint; then the aims from the base up, so each segment aims
from where the one below it left it.

![Example 19: rung aim targets bend the stalk behind each dash and whip it past the stop](../gifs/harmonics_chain.gif)

## Drags

An interactive drag sets the current frame only. The earlier frames are the
authored animation, so a dragged control rings or swings the cloth exactly as
it would if the drag had been keyed at that frame alone. Time does not move
during a drag, so the drag reads as a sudden move at that frame: a harmonics
target is held back against the drag, further the further it goes, and its
ring plays only when the frame changes. A cloth's pinned points follow the
drag while the rest of it trails behind, so a dragged bar slants its curtain
back against the drag. Releasing the drag returns to the authored result bit
for bit.

## Background caches

A frame-cache job carries the rings it was sampled with: they are part of the
job's inputs, folded into its control digest, so two frames with the same
current inputs but different motion before them never share a cache entry. A
job's frozen program solves the same window as the live one and publishes the
same bits.

## .rigexec playback

A `.rigexec` file records the windowed operations; a reader keeps a window of
its own frames. A host that plays frames in order pushes each one after it
runs; a host that jumps replays the frames before the target first:

```cpp
// Playing: run the frame, then keep its packets as history.
sampler.Apply(t, reader.get(), &error);
reader->Execute(&error);
reader->PushHistory();

// Jumping to t: rebuild the history from the frames before it.
reader->ClearHistory();
for (size_t k = reader->GetHistoryDepth(); k >= 1; --k) {
    sampler.Apply(t - double(k), reader.get(), &error);
    reader->ExecuteHistory(&error);   // runs and pushes one earlier frame
}
sampler.Apply(t, reader.get(), &error);
reader->Execute(&error);
```

Fed this way, the reader publishes what the native evaluator does, bit for
bit. A reader's first frames have less history than the window;
`rigExec:ignoreFirstFrame` keeps the harmonics constraint from ringing at the
jolt of starting from rest. The runtime plays the cloth itself
(`RigExecClothPlaybackKernel`), so a file plays in a host that loads no mover
library. `usdview`'s binary playback pushes on consecutive frames and rebuilds
the history on a jump. `rigExecPose --verify-binary` replays history for a
rig with windowed operations.

The standalone scene program (the rigpack path) refuses windowed operations.

## The cloth on the GPU

The live evaluator solves a cloth on the GPU when a host registered the
device (`usdview` and the Python module do), the cloth has at least
`RIGEXEC_GPU_CLOTH_MIN_POINTS` points (2048 by default; below that the CPU is
faster), its mover is the last on its mesh, and nothing else in the rig reads
the cloth's points. The operation body still captures the packet on the CPU;
the solve waits for the publication boundary, where it runs on the device
thread through the existing executor queue, and the mesh's normals and extent
are computed again on the result. `RIGEXEC_GPU_CLOTH=0` keeps every cloth on
the CPU.

The GPU runs the same passes in the same order as the CPU kernel, one dispatch
per pass over the whole cloth, so it agrees with the CPU to float rounding,
not bit for bit: on the test scenes the mean difference is under a hundredth
of the cloth's edge length. Background caches, history ticks and `.rigexec`
readers always solve on the CPU, and the parity tests run the CPU.

## Cost

Each evaluation of a cloth solves `rigExec:windowFrames * rigExec:substeps`
steps, 160 by default. Measured on one machine (an RTX 4090 and one core of
an i9-13900KF) for a pinned curtain swept by a sphere of as many triangles:

| Cloth points | CPU | GPU |
|---|---|---|
| 576 | 20 ms | 16 ms |
| 2304 | 148 ms | 37 ms |
| 9216 | 1.38 s | 0.23 s |

The docs example's 289-point curtain takes about 11 ms a frame on the CPU.
Collision dominates: a particle tests the collider triangles listed in its
own grid cell, which holds every triangle whose box, swept over the frame and
padded by `max(thickness, collisionDepth)`, contains the cell. Fewer
substeps, a shorter window or a smaller collision depth cost less.

## Limits

- The cloth's points and its colliders' points are read in their own local
  spaces, which must be the same space, as meshes skinned under one rig
  usually are.
- Colliders are one-sided along their outward normals; the cloth does not
  collide with itself.
- The cloth's topology, rest points and pins, and its colliders' topology,
  are fixed for a compile; editing them recompiles.
- A frame the cloth is disabled at holds no packet, so the history after it
  starts there.
- Mover plugins built for API 5 must be rebuilt for API 6, which added the
  windowed hooks ([external movers](external-movers.md#windowed-movers)).
