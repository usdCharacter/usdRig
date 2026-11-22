# Evaluation core parity inventory

Status: **accepted as the working inventory** (Phase 0 of the evaluation-core
milestone). This page is the ADR that every lift in that milestone cites: it
records, per operation, which body the RigExec baked path runs, which body the
RigExecRuntime playback path runs, what each one observed, what it does to
versions and caches, which test witnesses it today, and whether the pair is
destined for one body (`unify`) or stays a documented front-end difference
(`adapter-only`). Three rows are **OPEN** and need a human decision before
either side's body is deleted: the lattice `regularGrid` acceptance, and the
two gate / sparse rows that share one byte-parity question.

Vocabulary follows [the glossary](../../CONTEXT.md): **usdRig** is the project,
**RigExec** is the evaluation engine, **RigExecRuntime** is the USD-free
`.rigexec` playback engine. `RigExec*`, `rigExec*` and the runtime's `Rr*`
prefixes are established technical identifiers and are not renamed here.

## Snapshot

The behaviour snapshot this inventory freezes. Nothing in it was regenerated:
no golden, fixture, tolerance or test was edited to produce these numbers.

| Item | Value |
| --- | --- |
| Base SHA | `d489797a181feba318aebfeb2e60d619f00bd98a` (`factory/issue-64`, PR #65 head; the checkout this page was written in, clean, `git status` empty) |
| Build tree | same worktree, `build/`; `cmake --build build --target testRigExecRuntimeGeometry testRigExecRuntimeFixture` → `ninja: no work to do`, so the witness binaries match the base SHA byte for byte |
| Capture / fixture identities | `tests/fixtures/minimal.rigexec` (checked-in program `testRigExecRuntimeFixture` opens); `tests/exampleFixtures.cmake` (the bake/frame/input table `testRigExecRuntimeGeometry` and `verify_binary_*` walk); `tests/fixtures/{skeleton2d,layerBind,volumeSkin}/*.golden` (fixed scalar captures for Linux x86-64, GCC 13, Ubuntu 24.04 math — see [fixture checks](../../tests/fixtures/README.md)); the 60 frozen-path files and the `exec_crosscheck_*` rows are unchanged by this page |
| Selector | `bin/test_changed.sh --explain libs/rigExec/internal/bakedGeometry.cpp libs/rigExecRuntime/geometry.cpp libs/rigExecMath/pointBlocks.h` → libraries `rigExec`, `rigExecRuntime`; **115 direct executables**; **142 direct tests** (the `-R` regex ctest prints); `pointBlocks.h` has 6 includers (`libs/rigExec/internal/moverGraph.cpp`, `libs/rigExec/internal/bakedProgramImpl.h`, `libs/rigExecRuntime/geometry.cpp`, `tests/testRigExecGroupForwarding.cpp`, `tests/testRigExecMoverGraph.cpp`, `tests/testRigExecSurfaceKernelCache.cpp`) |

Parity-witness outcomes at the base SHA (ctest, `bin/_env.sh` sourced):

| Witness | Outcome | Detail |
| --- | --- | --- |
| `testRigExecRuntimeGeometry` | **FAILED** | 13 point mismatches over 82 stages / 276 frames. Fixtures: `examples/19_HarmonicsChain.usda` (frames 1060, 1064 — `/AntennaAsset/Geom/Stalk.points`, first point), `examples/biped/Biped.usda`, `Biped_body.usda`, `Biped_stack.usda` (frames 1–3 — `/Biped/Geom/body_geo.points`, first at point 86), `Biped_anim.usda` (frames 1, 5 — first at 1910, 13233). Sample: got `4.73964435e-14 87.3313065 -2.42181873`, want `5.90152249e-14 87.3313065 -2.42181873` |
| `testRigExecRuntimeGeometryFineClusters` (`RIGEXEC_BAKED_GRAIN_US=0`) | **FAILED** | the same 13 sites |
| `testRigExecRuntimeGeometry` with `RIGEXEC_ENABLE_SIMD=0` (scalar) | **FAILED** | the same 13 sites, same values |
| `testRigExecRuntimeGeometry` with `RIGEXEC_BAKED_GROUP_GATES=0` / `RIGEXEC_BAKED_RANGE_CHAINS=0` | **FAILED** | still exactly 13 point-mismatch sites; the knobs add failures of the test's own role/gating assertions instead (18 and 49 `FAIL` lines in total: `group chain roles and cutoff: … group N ran/did not run`) |
| `verify_binary_19_HarmonicsChain`, `verify_binary_biped_Biped`, `verify_binary_biped_Biped_body`, `verify_binary_biped_Biped_anim`, `verify_binary_biped_Biped_stack`, `verify_binary_raw_skin_layouts`, `verify_binary_raw_skin_layouts_chunked` (`rigExecPose --verify-binary`) | **FAILED** (7 of 7) | `rigExecPose --verify-binary` encodes the golden pose from the native program and from the runtime reader in one invocation (`tools/rigExecPose.cpp:564-572`), so these are in-process native↔playback byte differences, not foreign captures. They are **pose-side** rows (`jointFramesFinal`, `jointFramesBase`, `controlFrames`) one to two ULP off, e.g. `raw_skin_layouts` frame 10 `golden differs at row 10`, `…da41` → `…da42`. Still failing with `RIGEXEC_ENABLE_SIMD=0`. They belong to the value-family (pose) tasks, not to the geometry chain, and are recorded so those owners see them |
| `testRigExecRuntimeFixture` | Passed | USD-free open + execute of `tests/fixtures/minimal.rigexec` |
| `testRigExecGeometryOpsBakedParity` | Passed | baked generation agrees exactly with the dynamic walk, and every frame came from the program |
| `testRigExecGeometryOpsBakedParityCones_serial` | Passed | |
| `testRigExecFrozenContext` | Passed | frozen worker agrees with live |
| `testRigExecGroupForwarding_serial` / `_parallel` | Passed | gated chain through the native program, a frame-cache worker and the binary runtime, plus the biped |
| `testRigExecSkinSplit` | Passed | runtime serial skin loop vs parallel split |
| `testRigExecLattice` | Passed | `regularGrid = true` lattice (line 55) through live, frozen (`rigExecFrozenParity.h`) and bake-to-binary `Play` |
| `python3 -m mkdocs build --strict` | **unavailable locally** | `python3 -m mkdocs --version` → `No module named mkdocs`. The strict docs build was not run; CI's `docs.yml` owns it |

The 13 geometry mismatches are therefore **invariant** to arithmetic mode,
cluster grain, and the core's gate and range-chain Build knobs. Their cause is
not isolated by this snapshot; `examples/biped/*` deforms
`/Biped/Geom/body_geo.points` through one `RigExecSkinMover` with
`rigExec:elementSize = 10` and no `rigExec:skinningMethod` (the default
`classicLinear`), which places the search inside the skin dispatch and blend
range composition — owner `skin-dispatch-lift`.

## How to read the tables

- **oracle key** is the identity
  `libs/rigExecOracle/bakedExecCrossCheckRows.cpp` gives a step. Every geometry
  kind falls through to the same skip-only row
  (`bakedExecCrossCheckRows.cpp:1175-1211`):
  `NoEquivalent <label or kind:object:part> [target <chain target>] part <N> op
  <descriptorKey>`. Geometry steps carry no compiled label except
  `SkinTopology <mover>` (`libs/rigExec/internal/bakedGeometry.cpp:6862`), and the four
  revision kinds get the `target <chain target>` clause (1183-1188). **No
  geometry step has an OpenExec result row**: the oracle compares baked with
  OpenExec and never with RigExecRuntime. The runtime's witnesses are the ones
  in [Snapshot](#snapshot).
- **core** means the RigExec baked path in `libs/rigExec`; **playback** means
  RigExecRuntime in `libs/rigExecRuntime`.
- Line numbers are the base SHA's, and point at the function's name line.
- **similarity** is a `difflib` ratio over the two bodies after stripping the
  `RigExecBaked` / `RrGeo` prefixes and comments. It measures how far the two
  bodies have drifted, not whether they agree.
- **disposition** is `unify` (one canonical body in the evaluation core, both
  callers switched, duplicate deleted), `adapter-only` (the difference is a
  front-end fact that stays with the named front end), or `OPEN` (a human
  decision is required first).

## Step kinds

`RigExecBakedStepKind` (`libs/rigExec/internal/bakedProgramImpl.h:1331`), the wire's
`StepKind` (`libs/rigExecBinary/rigexec.fbs:118`) and the runtime's
`RigExecWireStepKind` (`libs/rigExecBinary/format.h:508`) are the same
ordinals. `RigExecBakedIsGeometryStep` (`bakedProgramImpl.h:1375`) calls eight
kinds geometry; `RrRunGeometryStep` (`libs/rigExecRuntime/geometry.cpp:9319`)
admits **seven** and rejects the rest with `<step label> is not a geometry
step` (9334-9344). `WeightField` is the eighth: the core runs it inside the
geometry half, RigExecRuntime routes it to `RrRunWeightStep` from the runtime
step dispatch (`libs/rigExecRuntime/closure.cpp:710-745`, geometry branch 741)
into `RrRunRevisionWeightField` (`geometry.cpp:9295`).

| oracle key (kind clause) | step kind | `part` | core body | playback body | observed result / skip / error | cache / version effect | witness | canonical rule or front-end owner |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `… ChainInputs …` | `ChainInputs` | unused (object = chain) | `RigExecBakedRunChainInputs` `bakedGeometry.cpp:4304` | `RrRunChainInputs` `geometry.cpp:7864` | both bind the chain base; a chain with no base leaves every revision step to return early (core `5409`, playback `9381`) | `ChainBase` content version, `ChainDirty`, base group ids | `testRigExecChainLevels`, `testRigExecRuntimeGeometry` | `unify` — owner `step-chain-inputs-lift` |
| `… InfluenceFold … target <chain> …` | `InfluenceFold` | unused | `bakedGeometry.cpp:5416-5455` → `FoldInfluences` `2661` | `RrGeoRunInfluenceFoldStep` `geometry.cpp:8349` → `RrGeoFoldInfluences` `7233` | both set `influencesChanged` / `influencesValid`; core also folds transform forms and the DQ palette safety (`5425-5452`) | `RevisionTransforms` version follows the transform bytes | `testRigExecSkinSplit`, `testRigExecRuntimeGeometry` | `unify` — owner `skin-dispatch-lift` |
| `… RevisionStatic … target <chain> …` | `RevisionStatic` | unused | `bakedGeometry.cpp:5457-5780` | `RrGeoRunRevisionStaticStep` `geometry.cpp:8409-8626` | both assemble the packet, publish the status and decide whether the revision runs; the core additionally publishes the weight-field overlay (`5557-5600`) and counts `revisionsBuilt` | `RevisionPacket` version, `revisionStaticDirty`, `deltasVersion` for a range-chain blend | `testRigExecGeometryOpsBakedParity`, `testRigExecRuntimeGeometry` | `unify` — owner `step-revision-static-lift` |
| `… RevisionChunk … target <chain> part <g> …` | `RevisionChunk` | vertex group `g` (or the chunk key for a keyed skin) | `bakedGeometry.cpp:5781-5908` | `RrGeoRunRevisionChunkStep` `geometry.cpp:8950-9079` | both apply group `part`; a gated group is skipped and the join counts it; core records gate violations through `NoteGateViolation` `3737` | `RevisionDone` / group content ids | `testRigExecGroupForwarding_*`, `testRigExecOneLoopAcceptance` (not built at the base snapshot) | `unify` — owner `step-revision-chunk-lift`; see the OPEN rows in [Divergences](#divergences) |
| `… RevisionFuse … target <chain> …` | `RevisionFuse` | unused | `bakedGeometry.cpp:5910-6057` | `RrGeoRunRevisionFuseStep` `geometry.cpp:9082-9272` | both decide apply-or-pass-through, publish `RevisionOut` and set the chain's sticky dirty bit | `RevisionOut`, `ChainDirty`, `resultVersion` on content change | `testRigExecGroupForwarding_*` | `unify` — owner `step-fuse-status-lift` |
| `… ChainStatus …` | `ChainStatus` | unused (object = chain) | `bakedGeometry.cpp:5306-5403` | `RrGeoRunChainStatusStep` `geometry.cpp:8249-8346` | both sweep the per-revision statuses and publish `ChainPoints`; a chain cut into groups is re-gathered only when its group content ids moved | `ChainPoints` content version moves exactly when the bytes do | `testRigExecRuntimeGeometry`, `testRigExecGroupForwarding_*` | `unify` — owner `step-fuse-status-lift` |
| `… Derived …` | `Derived` | unused (object = derived target) | `bakedGeometry.cpp:5205-5303` (inline in `RigExecBakedRunGeometryStep`) | `RrGeoRunDerivedStep` `geometry.cpp:8063-8246` | both recompute the derived target from the chain's published points; the core defers a GPU chain's derived step to the publication boundary (`5212-5216`), the playback path has no GPU chain | `DerivedOut` content version, held by handle | `testRigExecSurfaceBinding`, `testRigExecRuntimeGeometry` | `unify` for the CPU body; the GPU deferral is `adapter-only` — owner `step-fuse-status-lift` |
| `… WeightField …` (unpaired dispatch) | `WeightField` | field index | `bakedGeometry.cpp:2184` (Build) + `bakedSchedule.cpp:2406`, `2446` | `closure.cpp:741` routes it away from `RrRunGeometryStep` to `geometry.cpp:9295` | core treats it as a geometry-half step for diagnostics and scheduling; playback treats it as a weight step | `WeightField` result `ok`/`count`/`values` | `testRigExecWeightFields`, `testRigExecRuntimeWeights` | `adapter-only` (dispatch-half only; the field body itself pairs with the weights family) — owner `step-chain-inputs-lift` records it; body owner `weights-family-lift` |
| `NoEquivalent … <kind>:<object>:<part> …` | all eight geometry kinds | — | — | — | the oracle emits a `skipOnly`, `NoEquivalent` row for every geometry step; there is no OpenExec equivalent of a revision chunk, fuse, chain-status or derived body | none (row is skipped, never a result) | `testRigExecExecCrossCheckRows` | keep as-is: `parity-test-transition` may replace these with IR property rows but must not delete the independent OpenExec rows that do exist |
| `SnapshotFinals` (reserved) | `StepKind` value 8 | — | refused at Open | refused at Open | never written; a file holding one is refused (`rigexec.fbs:116-122`) | none | `testRigExecFormat` | no body: nothing to unify — owner `evalcore-target` keeps the ordinal reserved |
| `Snapshots` slot domain 25 (reserved) | slot domain | — | refused at Open | refused at Open | no step declares it (`rigexec.fbs:106-113`) | none | `testRigExecFormat` | no body — owner `evalcore-target` |

## Chain-internal step callers

`RigExecBakedRunGeometryStep` is not only the schedule's body: two Build-side
frame paths re-enter it, and both must keep re-entering the one canonical body
after the lift.

| site | what it does | owner |
| --- | --- | --- |
| `bakedGeometry.cpp:4999` (`ResolveWindowDevices`) | after a window-device chain's whole solve, runs that chain's `windowDeviceDerivedSteps` through the step body | `step-fuse-status-lift` |
| `bakedGeometry.cpp:5153` (`RigExecBakedResolveGpuChains`, the `runDerived` lambda) | runs the GPU chains' derived steps at the publication boundary, in parallel except on a frozen worker | `step-fuse-status-lift` |
| `bakedSchedule.cpp:2455` (inside `RunStepBody` `2388`, reached from `RigExecBakedRunStepBody` `2600`) | the single schedule seam for native, frozen and `.rigexec` | `step-chain-inputs-lift` |
| `closure.cpp:741` (inside `RrRunOpBody` `710`) | the runtime's only geometry-step seam | `step-chain-inputs-lift` |
| `bakedGeometry.cpp:4450` `RigExecBakedSkipGeometryStep` ↔ `geometry.cpp:9409` `RrSkipGeometryStep` | the paired skip switch: same per-kind flag clears (`influencesChanged`, `staticDirty`, `chunks[part].keyChanged`, `executed`, `chainBaseDirty`) | `step-fuse-status-lift` |
| `bakedGeometry.cpp:4775` `GpuStepChain`, `6074` `IsStepOfRevision`, `6869` `RigExecBakedDeclareLayoutReads` | kind switches that are scheduling metadata, not step bodies | `evalcore-target` |

## Name-matched helper pairs (layer 1)

Step/orchestration bodies, one per engine. `core` is `libs/rigExec/internal/bakedGeometry.cpp`,
`playback` is `libs/rigExecRuntime/geometry.cpp`.

| helper (oracle key = its step kind) | core | lines | playback | lines | sim | observed difference | cache / version effect | witness | disposition / owner |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `AdoptPartition` | `RigExecBakedAdoptPartition` `830` | 22 | `RrGeoAdoptPartition` `7611` | 10 | 0.15 | core refuses a null topology handle and runs the revision whole; playback adopts Build's recorded arrays and refuses only a missing `SkinTopology` record | compiled layout handles; no version move | `testRigExecSkinTopology` | `unify` body, `adapter-only` inputs (Build arrays vs decoded wire record) — `step-revision-chunk-lift` |
| `AssembleRevision` | `AssembleRevision` `3616` | 13 (+ `RigExecBakedAssembleFromLeaves`) | `RrGeoAssembleRevision` `6949` | 278 | 0.02 | core assembles through leaf reads and the resolved-input table; playback assembles from decoded wire parameters and Open-time path reads | `RevisionPacket`, `deltasVersion` | `testRigExecLeaves`, `testRigExecRuntimeInputs` | `unify` the parameter assembly, `adapter-only` the read binding — `step-revision-static-lift` |
| `ChunkTransformsView` | `2894` | 13 | `7448` | 13 | 0.99 | view only; types differ (`GfVec3f`/`RrVec3f`, `GfMatrix4d`/`RrMat4d`) | none (borrowed view) | `testRigExecSkinSplit` | `unify` — `skin-dispatch-lift` |
| `CopyMovedPoints` | `2631` | 19 | `3155` | 19 | 0.93 | byte-block copy of moved points; identical shape | `RevisionOut` content version follows the copy result | `testRigExecGroupForwarding_*` | `unify` — `step-fuse-status-lift` |
| `FoldInfluences` | `2661` | 120 | `7233` | 168 | 0.27 | core compares the influence table against the held one and reports `changed`; playback additionally re-resolves provider slots through the store and can return an error string | `RevisionTransforms`, `influencesChanged`, `gpuTouched` | `testRigExecRuntimeWeights`, `testRigExecSkinSplit` | `unify` arithmetic, `adapter-only` error text and slot resolution — `skin-dispatch-lift` |
| `FoldTransformForms` | `2814` | 25 | `7407` | 22 | 0.53 | both pick classic-linear vs dual-quat form and build the palette; playback calls its own scale-aware port (see layer 3) | palette and `dqWeightsSafe` facts | `testRigExecDualQuat`, `testRigExecDualQuatKernel` | `unify` — `skin-dispatch-lift`, port owner `dualquat-trait` |
| `FuseWholeRevision` | `3664` | 12 | `7587` | 12 | 0.75 | playback passes `RrGeoDispatch` for the parallel range; core passes `B.useSimd` and the host runner | `RevisionOut`, `resultVersion` | `testRigExecGroupForwarding_*` | `unify` — `step-fuse-status-lift` |
| `GatherChunkTransforms` | `2851` | 37 | `7464` | 53 | 0.66 | core gathers from resolved op values; playback gathers from provider slots and fails with an error when a slot names no provider | `RevisionTransforms` per chunk | `testRigExecSkinSplit` | `unify` gather order, `adapter-only` slot resolution — `skin-dispatch-lift` |
| `GroupAt` | `RigExecBakedGroupAt` `2438` | 9 | `RrGeoGroupAt` `4514` | 27 | 0.15 | core resolves the writer through `GroupWriter` and the group source memo; playback walks `revisions[version-1]` and returns the id through an out-parameter | group content ids, `ChainPoints` | `testRigExecGroupForwarding_*` | `unify` — `step-chain-inputs-lift` |
| `PublishBaseGroups` | `4224` | 31 | `4548` | 26 | 0.41 | core takes `moved` and re-publishes only then; playback publishes on its own reset marker; core's guard is `groupBounds.size() < 2`, playback's is `empty()` | `ChainBase` group refs and ids | `testRigExecGroupForwarding_*` | `unify` — `step-chain-inputs-lift` |
| `ResolveEnvelope` | `3639` | 8 | `8396` | 10 | 0.91 | same `ResolveAll` into `envelope` plus the content version note; playback comments that a failed resolve leaves the array unchanged | `envelopeOk`, `envelopeVersion` | `testRigExecEnvelopeVolumes_*` | `unify` — `envelope-sparse-lift` |
| `ResolvePoints` | `RigExecBakedResolvePoints` `3068` | 21 | `RrGeoResolvePoints` `6017` | 23 | 0.22 | core walks declared point bindings against live resolved inputs; playback walks the recorded candidate versions and answers nothing for a binding Open refused | base points, `ChainBase` version | `testRigExecRuntimeInputs`, `testRigExecPhaseBindings` | `unify` the candidate walk, `adapter-only` the source binding — `step-chain-inputs-lift` |
| `RunGroupStep` | `3818` | 141 | `8647` | 130 | 0.03 | the per-group apply: core takes acceptance, gate, halo and staging decisions from Build facts; playback takes them from decoded wire facts and returns bool with an error string | `RevisionDone` per group, `resultVersion` | `testRigExecGroupForwarding_*`, `testRigExecOneLoopAcceptance` (not built at snapshot) | `unify` — `step-revision-chunk-lift` |
| `RunLayoutOp` | `RigExecBakedRunLayoutOp` `6934` | 42 | `RrGeoRunLayoutOp` `7671` | 43 | 0.08 | both run the layout leaf (topology, weights, rest) and keep the handle for equal topology bits; core reads role facts from `leaves`, playback from decoded wire rows | `SkinTopology`, layout handles | `testRigExecSkinTopology` | `unify` — `skin-dispatch-lift` |
| `RunProjectorTarget` | `RigExecBakedRunProjectorTarget` `6324` | 16 | `RrGeoRunProjectorTarget` `7944` | 113 | 0.04 | both compute the projector target's matrix through the shared surface-projector kernel; core resolves the projector through resolved inputs, playback through the store | `DerivedOut` (matrix target), `haveMatrix` | `testRigExecSurfaceProjector`, `testRigExecSurfaceBinding` | `unify`, `adapter-only` input resolution — `surface-ops-lift` |
| `RunWholeFuse` | `4035` | 85 | `8831` | 77 | 0.25 | whole-revision fuse over every group; identical decision shape, different parallel hand-off | `RevisionOut`, `RevisionDone` ids | `testRigExecGroupForwarding_*` | `unify` — `step-fuse-status-lift` |
| `RunWholeSkinChunk` | `3971` | 34 | `8787` | 32 | 0.07 | speculative whole-skin chunk; playback returns bool and can fail with an error, core records a gate violation instead | `RevisionTransforms` chunk key | `testRigExecSkinSplit` | `unify` — `skin-dispatch-lift` |
| `SkinAcceptance` | `2926` | 16 | `7534` | 15 | 0.21 | same Refuses/Defers/Applies shape over packet, layout, envelope and DQ degeneracy; enum types differ | gate/speculative-chunk decision | `testRigExecSkinSplit`, `testRigExecOneLoopAcceptance` (not built) | `unify` — `skin-dispatch-lift` |
| `SkinPacketIsUsable` | `2916` | 4 | `7524` | 5 | 0.84 | identical predicate over `valid`, kind token, `layoutUsable`, `envelopeOk` | none | `testRigExecSkinSplit` | `unify` — `skin-dispatch-lift` |
| `SkinRange` | `2957` | 30 | `7553` | 26 | 0.66 | same range application into the staging buffer; core hands the host range runner, playback hands `RrGeoDispatch` | `RevisionTransforms` chunk content | `testRigExecSkinSplit` (serial vs split) | `unify` — `skin-dispatch-lift`; the 13 mismatch sites in [Snapshot](#snapshot) sit in this family |
| `WholeTransformsView` | `2792` | 13 | `7432` | 13 | 0.93 | view only | none | `testRigExecSkinSplit` | `unify` — `skin-dispatch-lift` |

## Per-op apply and acceptance layer (layer 2, shared with the dynamic walk)

The same apply and acceptance bodies exist a third time: the dynamic walk in
`libs/rigExec/internal/moverGraph.cpp` shares them with the baked path, and RigExecRuntime
mirrors them again. These are the bodies the lift must make single-homed; the
dynamic walk keeps calling them, it does not gain a second copy. `core` here is
`libs/rigExec/internal/moverGraph.cpp`, `playback` is `libs/rigExecRuntime/geometry.cpp`.

| body | core | playback | disposition / owner |
| --- | --- | --- | --- |
| blend envelope range / all | `RigExecBlendEnvelopeRange` `728`, `…All` `738` | `RrGeoBlendEnvelopeRange` `900`, `…All` `909` | `unify` — `envelope-sparse-lift` |
| envelope full-strength | `RigExecEnvelopeIsFullStrength` `moverGraph.h:774` | `RrGeoEnvelopeIsFullStrength` `851` | `unify` — `envelope-sparse-lift` |
| sparse-walk eligibility | `RigExecWireTakesSparseEnvelope` `moverGraph.h:499` (used by `_MatrixWalksSparse` `453`) | `RrGeoEnvelopeIsSparseWalk` `874`, aliased by `RrGeoWireTakesSparseEnvelope` `2992` | `unify` — `envelope-sparse-lift`; byte-visible risk in [Divergences](#divergences) |
| sparse-entry usability | `_SparseWalkIsUsable` `462` | `RrGeoSparseEnvelopeIsUsable` `886` | `unify` — `envelope-sparse-lift` |
| matrix kernel accept | `_MatrixKernelAccepts` `481` | `RrGeoMatrixKernelAccepts` `1152` | `unify` — `envelope-sparse-lift` |
| matrix sparse walk | `_ApplyMatrixSparseWalk` `510` (walk memo from `rigExecMath/pointBlocks.h`) | `RrGeoApplyMatrixSparseWalk` `1179` | `unify` — `envelope-sparse-lift` |
| matrix kernel range / whole | `RigExecApplyMatrixKernelRange` `248`, `RigExecApplyMatrixKernel` `696` | `RrGeoApplyMatrixKernelRange` `1099`, `RrGeoApplyMatrixKernel` `1269` | `unify` — `envelope-sparse-lift` |
| skin method / transforms / layout | `RigExecSkinMethodOf` `772`, `RigExecSkinTransformsOf` `784`, `RigExecSkinLayoutForPacket` `793`, `RigExecSkinLayoutIsUsable` `825`, `RigExecSkinTransformsAreUsable` `867` | `RrGeoSkinMethodOf` `2538`, `RrGeoSkinTransformsOf` `2472`, `RrGeoSkinLayoutForPacket` `2481`, `RrGeoSkinLayoutIsUsable` `2502`, `RrGeoSkinTransformsAreUsable` `2516` | `unify` — `skin-dispatch-lift` |
| skin kernel range / with-transforms / whole | `RigExecApplySkinKernelRange` `889`, `…WithTransforms` `952`, `RigExecApplySkinKernel` `1027` | `RrGeoApplySkinKernelRange` `2544`, `…WithTransforms` `2592`, `RrGeoApplySkinRanges` `2738` | `unify` — `skin-dispatch-lift` |
| SSE2 skin / matrix rows | none (the core's SIMD path is the Gf-based `_simd` kernels behind `RigExecSimdEnabled`) | `RrGeoNarrowSkinRows` `922`, `RrGeoApplyWeightedMatrixSimd` `982`/`1003`, `RrGeoApplyLinearBlendSkinSimd` `993`/`1047` | `adapter-only` today; see the SIMD row in [Divergences](#divergences) — owner `dualquat-trait` with the RigExecRuntime front end |
| blend-shape accept / apply | `_BlendShapeKernelAccepts` `626`, `RigExecApplyBlendShapeKernel` `1041` | `RrGeoBlendShapeKernelAccepts` `2894`, `RrGeoApplyBlendShapeKernel` `2908` | `unify` — `blend-compose-lift` |
| blend channel sum / sample | the walk's blend assembler | `RrGeoAccumulateBlendSample` `2798`, `RrGeoSumBlendChannels` `2821` | `unify` — `blend-compose-lift` |
| derived kernel | `RigExecApplyDerivedKernel` `1113` | `RrGeoApplyDerivedKernel` `2952` | `unify` — `blend-compose-lift` |
| separate-blend rule | `RigExecRevisionTakesSeparateBlend` `1527` | `RrGeoRevisionTakesSeparateBlend` `3048` | `unify` — `envelope-sparse-lift` |
| packet/kind check | `_PacketMatches` `445` | `RrGeoPacketMatches` `3057` | `unify` — `skin-dispatch-lift` |
| revision acceptance | `RigExecRevisionKernelAcceptance` `1542` | `RrGeoRevisionKernelAcceptance` `3066` | `unify`, lattice arm **OPEN** — `skin-dispatch-lift` |
| range-op set | `RigExecRevisionIsRangeOp` `1610` | no counterpart (the wire records the role) | `adapter-only` — `step-revision-chunk-lift` |
| group gate | `RigExecRevisionGateHolds` `1621` | `RrGeoGroupGateHolds` `4774` | `unify` over explicit facts; byte-visible risk **OPEN** — `gate-policy-lift` |
| prepare ranges | `RigExecPrepareRevisionRanges` `1654` | `RrGeoPrepareRevisionRanges` `3777` | `unify` — `step-revision-chunk-lift` |
| revision group body | `RigExecRunRevisionGroup` `1841` | `RrGeoRunRevisionGroup` `3927` | `unify`, lattice arm **OPEN** — `step-revision-chunk-lift` |
| revision range / kernel entry | `RigExecRunRevisionRange` `2044`, `RunRevisionKernel` `2087`, `RigExecRunRevisionKernel` `2214` | the `RrGeoRun*` step bodies above | `unify` — `step-revision-chunk-lift` |
| discardable probe kernel | `geometryDetail::RunDiscardableRevisionKernel` `2225`/`2236`, called as `RunDiscardableGeometry` at `bakedGeometry.cpp:4973` and `5863` | no counterpart (no window device in playback) | `adapter-only` — `step-fuse-status-lift` |
| wire prefix / acceptance / basis | `_WireKernelPrefix` `643`, `_WireKernelAcceptance` `666` | `RrGeoWireKernelPrefix` `2999`, `RrGeoWireKernelAcceptance` `3021` | `unify` — `wire-ops-lift` |
| runtime-only parallel helpers | the host `RigExecRangeRunner` | `RrGeoParallelPointRanges` `2670`, `RrGeoRangeRunner` `2724` | `adapter-only` (the host runner stays with each front end) — `evalcore-target` |
| point-bit equality | the `SamePoints` helpers in `bakedGeometry.cpp` | `RrGeoPointBitsEqual` `3129` | `unify` — `step-fuse-status-lift` |

## Numeric ports (layer 3)

`libs/rigExecMath/geometryKernels.h/.cpp` (2,226 lines, `pxr/base/gf/*`) and
`libs/rigExecMath/dualQuat.h/.cpp` (plus `solvers.cpp:640-712`) are the
canonical numbers; RigExecRuntime re-implements the remaining ones over `Rr*`
types inside `geometry.cpp`. The dual-quaternion family is now one body,
`libs/rigExecMath/dualQuatKernel.h`, instantiated by both engines over the
`RigExecDualQuatMath<Point, Double>` trait. The kernels in `rigExecMath` are
already USD-free in shape, so the fix is a trait over the primitive types, not
a copy.

| canonical | canonical site | playback port | disposition / owner |
| --- | --- | --- | --- |
| dual-quat normalize, blend, from-matrix, rotate, translate, transform | `rigExecMath/dualQuatKernel.h` (`RigExecDualQuat*Kernel`) | deleted (`dualquat-trait`) | `unified`: `dualQuatKernel.h` over `RigExecDualQuatMath<Point, Double>`; the `Gf` public functions in `dualQuat.cpp` are instantiations, the runtime's `RrGeo*` copy is gone |
| scaled dual-quat from-matrix, blend, transform | `dualQuatKernel.h` (`RigExecScaledDualQuat*Kernel`) | deleted (`dualquat-trait`) | `unified` |
| scale-aware blend (the stretch mean, `BlendStretch`) | `dualQuatKernel.h` (`dualQuatDetail::BlendStretch`) | deleted (`dualquat-trait`) | `unified` |
| matrix decomposition, proper-rotation and finiteness checks | `dualQuatKernel.h` (`dualQuatDetail::DecomposeMatrix`, `IsProperRotation`, `QuatIsFinite`), `geometryKernels.h:197` | deleted (`dualquat-trait`) | `unified`; the polar split reads its frame solve through the trait's `PointsToMatrix` / `MatrixToPoints`, and its 3x3 accessors through `SetRotate` / `Orthonormalize` / `ExtractRotationQuaternion` |
| 3x3 symmetric eigen solve | `rigExecMath/pointFrame.cpp` (`_SymmetricEigen3`), `dualQuatKernel.h` (`dualQuatDetail::SymmetricEigen3`) | deleted (`dualquat-trait`) | `unified` for the dual-quaternion split; `pointFrame.cpp` keeps its own copy for the pose path |
| point-to-parameter solve | `moverGraph.cpp:5100` (`RigExecWorldFromRest`) and its helpers, `pointFrame.cpp` (`RigExecPointsToParams`) | deleted (`dualquat-trait`) | `unified` for the dual-quaternion split (`dualQuatKernel.h`, `dualQuatDetail::PointsToParams`); the pose path's `RigExecWorldFromRest` still calls `pointFrame.cpp` |
| dual-quat influence gather, palette, apply | `solvers.cpp:640-712` (`RigExecGatherDualQuatInfluences`, `RigExecSkinDualQuatPalette`, `RigExecApplyDualQuatSkin`) | `RrGeoGatherDualQuatInfluences` `1806`, `RrGeoSkinDualQuatPalette` `1816`, `RrGeoApplyDualQuatSkin` `1828` | `unify` — `skin-dispatch-lift` over the `dualquat-trait` result |
| bound volume, volume correct | `geometryKernels.h:73,79` | `RrGeoBoundVolume` `1856`, `RrGeoApplyVolumeCorrect` `1873` | `unify` — `mesh-ops-lift` |
| mesh adjacency | `rigExecMath/meshConnectivity.h` | `RrGeoBuildAdjacency` `1897` | `unify` — `mesh-ops-lift` |
| Laplacian smooth | `geometryKernels.h:91,102,113` + `smoothKernel.h`, `deltaMushKernel.h` | `RrGeoApplyLaplacianSmooth` `1941` | `unify` — `mesh-ops-lift` |
| vertex normals, extent | `geometryKernels.h:211,227,262,284,291` + `normalsLayoutKernel.h` | `RrGeoComputeVertexNormals` `1996`, `RrGeoComputeExtent` `2058` | `unify` — `mesh-ops-lift` |
| lattice | `geometryKernels.h:304,314,324` + `latticeKernel.h` | `RrGeoApplyLattice` `2083` (group form via `RigExecApplyLatticeKernelGroup`) | `unify` — `mesh-ops-lift`; acceptance arm **OPEN** |
| surface project, closest point, accel | `geometryKernels.h:121,147,156,495,504,516,527` + `surfaceProjectorKernel.h`, `surfaceSnapKernel.h`, `surfaceKernelCache.h` | `RrGeoClosestPointOnTriangle` `2097`, `RrGeoApplySurfaceProject` `2134` | `unify` — `surface-ops-lift` |
| wire basis and walk | `geometryKernels.h:352,367,377,390,400,426,438,447,458,469` + `wireKernelCache.h` | `RrGeoWireBasisInputsAreUsable` `2239`, `RrGeoBuildWireBasis` `2252`, `RrGeoApplyWireBasis` `2316`, `RrGeoApplyWireBasisGroup` `2346`, `RrGeoWireInputsAreUsable` `2386`, `RrGeoApplyWire` `2398`, `RrGeoApplyWireGroup` `2432` | `unify` — `wire-ops-lift` |

## Divergences

Rows are keyed `<step kind> / <concept>`. Later tasks must not delete a body
whose row is `OPEN`. The six divergences the design confirmed are carried here
as: gates (`RevisionChunk / gates: written-group proof`), chunk and group caps,
sparse envelope, lattice acceptance, role classification, and SIMD selection.

| key | disposition | owner | evidence and rule |
| --- | --- | --- | --- |
| `RevisionChunk / gates: written-group proof` | **OPEN — needs human decision** | decision owner: the operator route through the source issue; implementation owner once decided: `gate-policy-lift` | Core gate `RigExecRevisionGateHolds` (`moverGraph.cpp:1621`) answers from op and packet facts only. Playback gate `RrGeoGroupGateHolds` (`geometry.cpp:4774`) short-circuits when every group is written (`4777-4780`), consults `RrGeoEnvelopeIsSparseWalk` (`874`) and then proves that every listed weight index lies in a written group (`4790-4800`). The core has neither the short-circuit nor the index proof, and the playback gate omits the core's `!p.latticeSettings.regularGrid` clause (`moverGraph.cpp:1637`). A single unified predicate must take explicit enabled / proof / packet facts, with `RIGEXEC_BAKED_GROUP_GATES` and the path-leaf diagnostics as core-adapter inputs and the decoded written-group flags as playback's. Blocking question: whether that predicate can change observable bytes for identical facts. The measurement in [Snapshot](#snapshot) shows the 13 byte-visible point mismatches are stable when the gate is turned off (`RIGEXEC_BAKED_GROUP_GATES=0`), so the gate is not shown to be their cause and is not shown to be byte-neutral either. Failing fixtures that must stay byte-identical across the switch: `examples/biped/Biped.usda`, `examples/biped/Biped_body.usda`, `examples/biped/Biped_anim.usda`, `examples/biped/Biped_stack.usda`, `examples/19_HarmonicsChain.usda` through `testRigExecRuntimeGeometry`; gated-chain behaviour through `testRigExecGroupForwarding_serial` / `_parallel` (both pass at the base SHA) |
| `RevisionChunk / sparse envelope` | **OPEN — needs human decision** | decision owner: the operator route through the source issue; implementation owner once decided: `envelope-sparse-lift` | The eligibility predicates are textually the same test (`moverGraph.h:499` vs `geometry.cpp:874`), but they are consulted at different places: the core gate calls `RigExecWireTakesSparseEnvelope`; the playback gate calls `RrGeoEnvelopeIsSparseWalk` (`geometry.cpp:4782`) and the playback matrix and range paths call it three more times (`1160`, `1279`, `3980`). The memo rules (`RigExecSparseWalkMemo` in `libs/rigExecMath/pointBlocks.h`) exist on the core side; the playback walk keeps its own records (`geometry.cpp:1179`). One predicate plus the memo validity rules and the dense fallback is the plan. Same blocking question and the same failing fixtures as the gate row; `testRigExecGroupForwarding_*` (biped arm edit through the sparse Matrix group memo) passes at the base SHA and is the byte-parity witness for the memo. |
| `RevisionStatic / lattice regularGrid acceptance` | **OPEN — needs human decision** | decision owner: the operator route through the source issue; implementation owner once decided: `skin-dispatch-lift` (acceptance) and `step-revision-chunk-lift` (group body) | The core defers: `RigExecRevisionKernelAcceptance` returns `Acceptance::Deferred` for a regular grid (`moverGraph.cpp:1577-1581`), so a speculative chunk answers the question and the fuse decides. Playback answers `Applies` at acceptance (`geometry.cpp:3101-3114` has no `regularGrid` clause) and then refuses inside the group body with a hard `false` (`geometry.cpp:4023-4027`, "a regular grid has no group form"). Both agree today on the **role**: the core pins a non-legacy lattice to `Whole` (`ClassifyRevision`, `bakedGeometry.cpp:1405-1416`) and playback reads that role from the wire (`geometry.cpp:5618-5620`), so no regular-grid lattice runs as a Range group and the two answers never meet on a shipped fixture. `testRigExecLattice` (`regularGrid = true` at line 55, live + frozen + bake-to-binary) **passes at the base SHA on both paths**, so it is the fixture named for this row and it does not yet distinguish them: a case that puts a regular-grid lattice in a gated Range chain must be authored before either side switches, or the decision must be recorded as "keep the roles, delete one answer". Do not resolve it by relaxing a tolerance. |
| `RevisionChunk / chunk and group caps` | `adapter-only` | `step-revision-chunk-lift` (core Build side), runtime decode owned by the same task's playback half | The Build knobs — six partition knobs, `RIGEXEC_BAKED_CHUNK_ALWAYS` (`bakedGeometry.cpp:639`), `_CHUNK_VERTS` (`669`), `_MAX_CHUNKS` (`683`), `_RANGE_CHAINS` (`693`), `_GROUP_VERTS` (`701`), `_GROUP_CAP` (`710`), plus the gate knob `_GROUP_GATES` (`720`) — are Build-time partition facts. The wire records the result (`table Revision` `rigexec.fbs:1347-1397`: `chunks`, `chunk_base`, `partition_topology`, `partition_element_size`, `partition_index_count`, `partition_point_count`, `chunked`, `partition_candidates`, `partition_producer_*`). RigExecRuntime has no knob and must keep binding exactly the recorded partition. The 13 mismatches did not move when `_GROUP_GATES` or `_RANGE_CHAINS` changed, so caps are not their cause. |
| `InfluenceFold / role classification` | `adapter-only` | `step-revision-chunk-lift` | `ClassifyRevision` (`bakedGeometry.cpp:1388`) plus `GateDefault` (`1651`), `GateRevision` (`1734`) and the role pins decide Legacy/Range/Whole at Build; playback derives `RrGeoRole` (`geometry.cpp:4089`, set at `5618-5620`) from the file and never re-decides. Classification stays a core-adapter Build function; the evaluation core consumes the resulting facts. No runtime port may be added. |
| `RevisionChunk / SIMD selection` | `adapter-only` | RigExecRuntime front end; recorded by `evalcore-target` | Playback picks SSE2 kernels at compile time (`geometry.cpp:63-70`, `RIGEXEC_DISABLE_SSE2`) and reads `RIGEXEC_ENABLE_SIMD` once per reader at Open (`libs/rigExecRuntime/open.cpp:65`). The core reads `RIGEXEC_ENABLE_SIMD` once at library load (`moverGraph.cpp:214`, `moverGraph.h:508`) and passes the answer as `useSimd`. The scalar-vs-SIMD accumulation difference is documented in [tests/fixtures/README.md](../../tests/fixtures/README.md): the SIMD skin path accumulates floats, scalar accumulates doubles, so no capture is shared between modes. This stays a front-end build difference; the two reads are **not** unified and no engine-identity flag is added. `cmake/rigexec_flags.cmake` remains the authority for precise floating-point flags on every translation unit that instantiates the shared code. |
| `RevisionChunk / Deferred is a tri-state` | `unify` | `skin-dispatch-lift` | `RigExecRevisionAcceptance` and `RrGeoAcceptance` already spell `Refuses`/`Applies`/`Deferred` (`moverGraph.h:791`, `geometry.cpp:865`). `Deferred` must stay "the run answers", never a hold: `Deferred` continues to mean the same on both paths after the acceptance body is unified. The lattice arm above is the one case where the two bodies disagree about it. |
| `ChainInputs / base group publication` | `unify` | `step-chain-inputs-lift` | `PublishBaseGroups` (`bakedGeometry.cpp:4224`) vs `RrGeoPublishBaseGroups` (`geometry.cpp:4548`): same publication rule, different moved markers and guards (`size() < 2` vs `empty()`). Chain points are byte-compared by `testRigExecRuntimeGeometry`, so the merged body must keep group identity and content versions identical. |
| `ChainStatus / chain point gather` | `unify` | `step-fuse-status-lift` | `bakedGeometry.cpp:5306-5403` vs `geometry.cpp:8249-8346`: the core re-gathers through the group source memo (`RigExecBakedGroupSourceAt`), the runtime through `RrGeoGroupAt`'s id out-parameter. Same `ChainPoints` content-version rule; one body must keep the identity short-circuit. |
| `Derived / GPU deferral` | `adapter-only` | `step-fuse-status-lift` | The core skips a GPU chain's derived step at `bakedGeometry.cpp:5212-5216` and re-runs it at the publication boundary (`5153`). RigExecRuntime has no GPU chain. The skip stays in the core adapter around the shared body. |
| `WeightField / dispatch half` | `adapter-only` | `step-chain-inputs-lift` (dispatch), `weights-family-lift` (body) | The core counts `WeightField` in the geometry half (`bakedProgramImpl.h:1379`); the runtime routes it to its weight dispatch (`closure.cpp:741` → `geometry.cpp:9295`). Only the half/diagnostic ordering differs; no body is deleted by the geometry chain. |
| `RevisionStatic / oracle coverage (every geometry kind)` | `adapter-only` | `geometry-parity-audit`, then `parity-test-transition` | Every geometry row in the OpenExec cross-check is a `NoEquivalent`, `skipOnly` descriptor (`bakedExecCrossCheckRows.cpp:1175-1211`), and `rigExecOracle` is not linked into the runtime, imaging or playback tests (`CMakeLists.txt:262-280`). The runtime's evidence is bit-for-bit comparison in `tests/testRigExecRuntimeGeometry.cpp`, the shared drivers in `tests/rigExecRuntimeDrive.h` (26 includers) and `tests/rigExecFrozenParity.h` (12 includers), `tests/fixtures/minimal.rigexec` with `testRigExecRuntimeFixture`, and `rigExecPose --verify-binary` (61 `verify_binary_*` entries). That coverage must not be relaxed while the bodies merge; the independent OpenExec rows that do exist stay. |
| `FrameMatrix / native↔playback pose bytes` (out of geometry scope, recorded here) | `unify` (value families) | `pose-steps-family-lift` and the other family tasks | The `verify_binary_*` failures in [Snapshot](#snapshot) are 1–2 ULP differences in `jointFramesFinal`, `jointFramesBase` and `controlFrames` between the native program and playback in one invocation. They are pose-side, and the geometry chain must not be blamed for them or fix them by editing a capture. |

## Rules carried into the lifts

- One canonical body per row marked `unify`; both callers switch and the
  duplicate is deleted in the same checkpoint. A forwarding switch that still
  calls two host revision loops is not a lift.
- The oracle rows in `libs/rigExecOracle/bakedExecCrossCheckRows.cpp` compare
  baked with OpenExec. Never treat them as runtime coverage: the runtime is
  witnessed only by the tests named in [Snapshot](#snapshot).
- Topology, partitions, group bounds and roles are fixed per compiled rig;
  playback binds what the file recorded and never recomputes a partition.
- Scalar/SIMD arithmetic differences stay front-end facts with the explicit
  reference configuration named in
  [tests/fixtures/README.md](../../tests/fixtures/README.md).
  Nothing in the milestone relaxes a tolerance, excludes a case, or replaces a
  golden to make a row agree.
- See [Baked and dynamic evaluation](../concepts/baked-vs-dynamic.md) for the
  unified-loop rule this inventory serves, and
  [External movers](../concepts/external-movers.md) for the Whole-only contract
  of plugin movers.
