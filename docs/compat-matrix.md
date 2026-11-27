---
title: Format and plugin API compatibility matrix
summary: Which .rigexec format versions and mover plugin API versions each repository and sidecar speaks.
order: 90
---

# Format and plugin API compatibility matrix

Two versioned contracts cross repository boundaries:

- the **`.rigexec` binary format**, stamped as `formatVersion` in every baked
  file and verified by the reader at open; and
- the **mover plugin API**, stamped into each mover plugin's metadata and
  checked by the host loader before the library is loaded.

This page lists the current values, their history, and what each sidecar in
the usdCharacter organization speaks. It documents **current behavior**, not a
stability promise: [layer readiness](../STATUS.md) still marks every layer
prototype or beta, and formats may change without notice. What changed is that
the versions are now a public contract — bump them deliberately, record the
bump here, and update the pinned sidecars.

## Current contract

| Contract | Value | Defined in | Enforced by |
|---|---|---|---|
| `.rigexec` format (writer) | **56** | `RigExecFormatVersion` in `libs/rigExecBinary/format.h` | every bake stamps it |
| `.rigexec` format (reader) | **56** | `RigExecFormatOldestReadable` in `libs/rigExecBinary/format.h` | `libs/rigExecBinary/format.cpp` refuses others at open with a re-export suggestion; a `static_assert` pins the oldest readable, and `tests/testRigExecFormat.cpp` and `tests/testRigExecRuntimeLoader.cpp` pin the accepted range |
| Mover plugin API | **8** | `RigExecMoverPluginApiVersion` in `libs/rigExec/api/moverRegistry.h` | the loader (`moverRegistry.cpp`) rejects a plugin whose metadata `RigExecMoverPlugin` value differs; `cmake/rigExecMoverPlugin.cmake` stamps the same number into plugin metadata |
| External-mover wire type IDs | plain API4 IDs, pinned | `RigExecExternalInputType` in `libs/rigExecBinary/external.h` | the format's `InputTag` numeric inventory; IDs never renumber |

Reader rules: a file older than the oldest readable version is refused with a
message telling the user to re-export; a file newer than the writer version is
refused as unsupported. A file inside the range opens; versions below the
writer's carry a note naming the writer's version.

Rebuild rules for plugins follow [external movers](concepts/external-movers.md):
the plugin ABI has never been preserved across an API bump — every API change
so far requires a rebuild, and API 4–7 plugins must rebuild for API 8 because
the handler's first field layout changed.

## Release tags

| usdRig tag | Date | `.rigexec` writes | `.rigexec` reads | Plugin API |
|---|---|---|---|---|
| `v0.1.0` | 2026-10-10 | 21 | 20–21 | 5 |
| `v0.2.0` | 2026-11-15 | 32 | 20–32 | 8 |

## .rigexec format version history

The writer version has moved with the record layout almost from the first
bake-capable commit. Dates and versions come from the `format.h` version line
in usdRig history.

| Version | Date | What it carried |
|---|---|---|
| 4 | 2026-10-04 | first FlatBuffer `.rigexec` schema (FlatBuffers 25.12.19 vendored) |
| 5 | 2026-10-05 | one validated FlatBuffer per file; the old container deleted |
| 7 | 2026-10-05 | skin layouts the sparse form cannot hold, stored raw |
| 8 | 2026-10-06 | array inputs |
| 9 | 2026-10-06 | head ops promoted into the shared step graph |
| 10 | 2026-10-06 | retired head classification and cache state removed |
| 17 | 2026-10-07 | unified evaluation: one operation graph in the file (jump from 10; 11–16 never reached `main`) |
| 18 | 2026-10-08 | rebased unified graph work |
| 19 | 2026-10-09 | range-pipelined revisions validated |
| 20 | 2026-10-09 | role-flip rebuild |
| 21 | 2026-10-09 | new movers, affine frames, extended deformer settings (the `rigexec-format-v4` merge); the only version `v0.1.0` wrote |
| 22 | 2026-10-13 | GPU and shader deformers |
| 23 | 2026-10-14 | rigs driven along BasisCurves paths |
| 24 | 2026-10-15 | ML mover and extended RBF driver |
| 25 | 2026-10-20 | harmonic deformation, GPU cloth sim, patchMover changes |
| 26 | 2026-10-21 | cloth mover anisotropic model and constraints |
| 27 | 2026-10-30 | sampled volume falloff inputs |
| 28 | 2026-11-03 | per-vertex-group Smooth and DeltaMush |
| 29 | 2026-11-04 | validation choke points removed |
| 30 | 2026-11-04 | dual-quaternion parallelism |
| 31 | 2026-11-04 | 2D skeletons as nested joints, capsule weights, generated skin rows |
| 32 | 2026-11-13 | authored normals layout, compiled vertex-normal topology |
| 33 | 2026-11-22 | the manifest: stable ids of revision, property and provider ops and value types |
| 34 | 2026-11-22 | single-chain IK joint orientation |
| 35 | 2026-11-23 | op nodes run through the op core |
| 36 | 2026-11-24 | op nodes as steps of the common schedule |
| 37 | 2026-11-25 | property revisions as op nodes reading the rig through lookups |
| 38 | 2026-11-25 | volume placements as op nodes reading pose versions through lookups |
| 39 | 2026-11-25 | weight packets as op nodes reading weight arrays through lookups |
| 40 | 2026-11-25 | weight fields as op nodes reading weight objects through lookups |
| 41 | 2026-11-26 | provider and frame-record matrices as op nodes reading rest frames and commit record gates through lookups |
| 42 | 2026-11-26 | full-strength Range matrices as MoverTransform and MatrixDeform op nodes; op chains carry their members and written groups |
| 43 | 2026-11-26 | pose commits as op nodes reading commit candidates and version lists through lookups |
| 44 | 2026-11-26 | every Range matrix as op nodes, its weight packet read from the packet's node and its enabled state stored by its MatrixDeform |
| 45 | 2026-11-27 | connected provider refreshes and provider arithmetic and affine operations as op nodes reading their inputs through lookups |
| 46 | 2026-11-27 | every Range smooth as a Smooth op node; consecutive op revisions of a point chain share one range chain |
| 47 | 2026-11-27 | Position, Rotation, Scale, Parent and Aim constraints, the MatrixMover and geometry-target deltas as op nodes reading their inputs through constraint lookups |
| 48 | 2026-11-27 | every Range delta mush as a DeltaMush op node reading its inputs through revision lookups |
| 49 | 2026-11-27 | provider values other than operation results answered by ProviderSelect op nodes reading their candidates through lookups; the SpaceExpression step retired |
| 50 | 2026-11-27 | every Range lattice as a Lattice op node reading its cages through revision lookups |
| 51 | 2026-11-27 | harmonics, single-chain IK and IK/FK match constraints as op nodes reading their inputs through constraint lookups |
| 52 | 2026-11-27 | avar bindings, space checkpoints and pose interpolators as AvarInputs, SpaceCheckpoint and PoseInterpolator op nodes reading their inputs through lookups; their steps retired |
| 53 | 2026-11-27 | the Constraint step retired: a constraint with no target has no step |
| 54 | 2026-11-27 | sparse Range blend shapes as BlendShape op nodes reading their channel weights through revision lookups keyed to the vertex groups they reach |
| 55 | 2026-11-27 | every solver kind as op nodes reading their inputs through solver lookups; the Solve step retired and the rest copies only it read deprecated |
| 56 | 2026-11-27 | rests as RestCompose op nodes reading their inputs through lookups; the RestCompose step retired; pose interpolator rows name their solve and no longer store its tables; **oldest readable from here on** |

Versions 6 and 11–16 never appeared on `main`. Files written before version 56
are outside every current reader: re-export them.

## Mover plugin API history

| API | Date | What changed |
|---|---|---|
| 1 | 2026-10-02 | initial plugin registration baseline |
| 2 | 2026-10-03 | plugin movers exported to and replayed from `.rigexec` |
| 3 | 2026-10-05 | own-chain `preceding` reads as entering points |
| 4 | 2026-10-07 | unified evaluation; the plain wire type IDs in `external.h` pinned (the "API4 type IDs") |
| 5 | 2026-10-09 | affine frames and the new movers played in `.rigexec`; what `v0.1.0` shipped |
| 6 | 2026-10-20 | handler and playback kernel grew (harmonics, GPU cloth, patchMover) |
| 7 | 2026-10-21 | naming pass over the handler surface |
| 8 | 2026-11-12 | schema type spelled as `std::string_view`; first field's layout changed |

The plugin's payload bytes are the plugin's own format: the format carries one
external-mover entry per revision with opaque compiled and frame bytes, and
the mover's decoder validates its own payload version. A plugin rebuild for a
new API therefore also re-bakes the payloads, so **bake with the same usdRig
generation the plugin and runtime come from**.

## Sidecar compatibility

Surveyed 2026-11-17 against each repository's default branch. "Tracks the
checkout" means the repository carries no version pin of its own: it builds
against a sibling `usdRig` checkout and is compatible with exactly that
checkout's range, so rebuild it after any usdRig format or API bump.

| Repository | What it consumes | Pin today | `.rigexec` it reads | Plugin API |
|---|---|---|---|---|
| [usdRig_unreal](https://github.com/usdCharacter/usdRig_unreal) | prebuilt `rigExecRuntime`/`rigExecBinary` static libs and headers vendored under `Source/ThirdParty/RigExecLib` | usdRig commit `3159dca` (2026-10-17), per its `SOURCE.txt` | 20–**24** (its runtime refuses 25+; re-bake or re-vendor) | no mover plugins; unknown external mover types pass through with a warning (`GetMissingExternalKernels`) |
| [usdRig_godot](https://github.com/usdCharacter/usdRig_godot) | compiles `rigExecRuntime`, `rigExecBinary` and `rigExecGraph` from the sibling `usdRig` checkout straight into the GDExtension | tracks the checkout | same range as the checkout it was built against | no mover plugins |
| [usdRig_maya](https://github.com/usdCharacter/usdRig_maya) | `rigExecMaya` bakes at load with the sibling usdRig's `rigExecBake --poseable` and plays the bake with the runtime from the same build; the sidecar installs against usdRig's installed SDK | tracks the checkout | bake and playback share one build, so compatible by construction | no mover plugins |
| [usdRig_blender](https://github.com/usdCharacter/usdRig_blender) | links the installed `rigExecRigging`/`rigExec`/`rigExecScene` SDK; translates `.blend` to native USD rigs | tracks the installed SDK (built against usdRig `main`) | none — it sits on the authoring side of the bake, no `.rigexec` involved | n/a; must load the SDK it was compiled against |
| `usdRig_ios` (closed) | usdview for iPad: RigExec evaluator, `rigExecImaging`, and the usdview plugins from a sibling usdRig checkout | tracks the checkout | full evaluator/playback of the checkout | follows the checkout |
| `usdRig_android` (closed) | Android port of the iPad viewer; sibling checkouts as on iOS | tracks the checkout | full evaluator/playback of the checkout | follows the checkout |
| [usdRig_movers](https://github.com/usdCharacter/usdRig_movers) | external mover plugins (Profile Mover, Curvenet Adjuster) built through `rigexec_add_mover_plugin` | **usdRig v0.1.0, API 5** (asserted in its test) | follows the host usdRig it loads into | 5 — **drift**; loading into an API 8 host is refused. Tracked in [usdRig_movers#3](https://github.com/usdCharacter/usdRig_movers/issues/3) |

The known drifts as of this survey:

- **usdRig_movers is one major API behind** (5 vs 8) and cannot load in a
  `v0.2.0` host until rebuilt — [usdRig_movers#3](https://github.com/usdCharacter/usdRig_movers/issues/3).
- **usdRig_unreal's vendored runtime reads only up to format 24**, so `.rigexec`
  files baked by current usdRig (writer 56) refuse to open there until the
  plugin re-vendors its libraries from a current usdRig commit.
- Godot, Maya, iOS and Android follow their checkouts and have no drift while
  they build against the same usdRig they bake with.

## Bumping a version: maintainer checklist

When a record layout or handler ABI changes:

1. `.rigexec`: raise `RigExecFormatVersion` in `libs/rigExecBinary/format.h`;
   if old files stop opening, raise `RigExecFormatOldestReadable`, its
   `static_assert` in `format.cpp`, and the range tests together.
2. Plugin ABI: raise `RigExecMoverPluginApiVersion` in
   `libs/rigExec/api/moverRegistry.h` and
   `RIGEXEC_MOVER_PLUGIN_API_VERSION` in `cmake/rigExecMoverPlugin.cmake` in
   the same commit, and update the rebuild paragraph in
   [external movers](concepts/external-movers.md).
3. Add a row to the history tables on this page.
4. Update the pinned consumers: re-vendor the libraries and refresh
   `SOURCE.txt` in usdRig_unreal; rebuild the usdRig_maya, usdRig_godot,
   usdRig_blender and mobile builds against the new usdRig; rebuild
   usdRig_movers (its test asserts the API number).
5. Bake fixtures shipped to sidecars (the Godot demo assets, Unreal example
   content, Maya biped bakes) need re-baking with the same generation.
