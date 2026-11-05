# Execute publication buffers

A successful `RigExecRuntimeReader::Execute` shares every moved point array
and every resolved weight field it retains with the views `GetPoints` and
`GetWeightFields` return. It does not copy their elements.

Publication rows retain their storage and path order while output membership
is unchanged. A membership change rebuilds the ordering; ordinary frames
replace values in those rows without allocating and sorting new path lists.
Every output builds its changed rows before any output is replaced, and the
rows a change drops return their buffers to the spare pool.

`RigExecRuntimePoints::points` is a `RigExecSharedArray<RrVec3f>`.
`RigExecRuntimeWeightField::weights` is a `RigExecSharedArray<float>`.
Copying either view copies the pointer, not the elements.

| Call | Result |
|---|---|
| `size`, `empty`, `operator[]`, `data`, `begin`, `end` | Read the elements |
| `Vector()` | The underlying `const std::vector` |
| `Share()` | A `shared_ptr<const vector>` that keeps this snapshot after the next Execute |
| `operator==` | `std::vector` equality: a NaN compares unequal, and +0 equals -0 |

Bit-exact checks `memcmp` `data()`. They do not `memcmp` the view object,
and they do not use `operator==` where a NaN payload or a signed zero matters.

`Execute` clears its publication maps after handing out the shares.
The chain keeps its own retainer. The geometry step copies points into
a spare and then swaps that spare into the chain; that copy builds the next
snapshot. While a share of the published buffer is alive, the swap does not
reuse that buffer and does not change its bytes. `Write()` copy-on-writes in
the same case.

On success, a buffer held only by the reader's previous result is returned to
a per-thread pool. The next fill of the same size takes that storage instead
of allocating. A `Share()` or a copied `RigExecRuntimePoints` /
`RigExecRuntimeWeightField` that the caller keeps is not pooled. The next
fill of that array allocates, and the kept snapshot stays byte-identical.

An empty publication shares one immutable empty vector. It is not a null view.

## usdRig_godot

This repository does not modify the sibling `usdRig_godot` tree. Playback
there that reads `GetPoints()` or `GetWeightFields()` after `Execute`
follows these rules:

* Do not deep-copy `points` and `weights` inside the success path; the views
  already share the published buffers.
* A `std::vector<RrVec3f>` or `std::vector<float>` member does not bind to
  the view. Read `points.Vector()` or `weights.Vector()` for the frame being
  drawn. `data()`, `size()`, `operator[]`, and range-for on the view itself
  also compile.
* A `const std::vector<RrVec3f> *` taken from `&moved.points` becomes
  `&moved.points.Vector()`. That address is valid while the
  `RigExecRuntimePoints` element is alive.
* The arrays from `GetPoints` and `GetWeightFields` are reused after the
  next successful `Execute` replaces them. Copying the
  `RigExecRuntimePoints` or `RigExecRuntimeWeightField`, or calling
  `Share()`, keeps that frame's bytes. The runtime then allocates the next
  spare instead of recycling the held one.
* Copy into a host mesh array from `Vector()` or `data()` when the host
  needs its own bytes. That copy belongs to the host.
* Do not `memcpy` a `RigExecSharedArray`. Its bytes are a pointer, not the
  points.
