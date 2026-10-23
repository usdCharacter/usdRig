// Exact frame-input and compiled rig constant fingerprints.

#include "frozenContextInternal.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec4f.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace frozenDetail {

uint64_t
_HashBytes(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    while (size >= 8) {
        uint64_t word;
        std::memcpy(&word, bytes, sizeof(word));
        hash = _MixWord(hash, word);
        bytes += 8;
        size -= 8;
    }
    if (size > 0) {
        // Zero-padded, with the length mixed in so a short tail cannot
        // equal a longer stream's shared prefix. An empty hash stays a
        // no-op, as before.
        uint64_t tail = 0;
        std::memcpy(&tail, bytes, size);
        hash = _MixWord(hash, tail + size);
    }
    return hash;
}

uint64_t
_HashString(uint64_t hash, const char *text)
{
    return _HashBytes(hash, text, std::strlen(text));
}

// Mixes a VtValue's bytes. Returns false for a type this function does not
// name, having mixed only the type name: the caller reports the digest
// inexact rather than risk two different frames sharing one key.
bool
_HashVtValue(uint64_t *hash, const VtValue &value)
{
    *hash = _HashString(*hash, value.GetTypeName().c_str());
    if (value.IsEmpty()) {
        return true;
    }
#define _RIGEXEC_FROZEN_SCALAR(type)                  \
    if (value.IsHolding<type>()) {                    \
        const type &held = value.UncheckedGet<type>(); \
        *hash = _HashBytes(*hash, &held, sizeof(held)); \
        return true;                                  \
    }
    _RIGEXEC_FROZEN_SCALAR(bool)
    _RIGEXEC_FROZEN_SCALAR(int)
    _RIGEXEC_FROZEN_SCALAR(float)
    _RIGEXEC_FROZEN_SCALAR(double)
    _RIGEXEC_FROZEN_SCALAR(GfVec2f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3d)
    _RIGEXEC_FROZEN_SCALAR(GfVec3i)
    _RIGEXEC_FROZEN_SCALAR(GfVec4f)
    _RIGEXEC_FROZEN_SCALAR(GfMatrix4d)
#undef _RIGEXEC_FROZEN_SCALAR
#define _RIGEXEC_FROZEN_ARRAY(type, element)                       \
    if (value.IsHolding<type>()) {                                 \
        const type &held = value.UncheckedGet<type>();              \
        const size_t count = held.size();                          \
        *hash = _HashBytes(*hash, &count, sizeof(count));           \
        if (!held.empty()) {                                       \
            *hash = _HashBytes(*hash, held.cdata(),                 \
                               held.size() * sizeof(element));      \
        }                                                          \
        return true;                                               \
    }
    _RIGEXEC_FROZEN_ARRAY(VtBoolArray, bool)
    _RIGEXEC_FROZEN_ARRAY(VtIntArray, int)
    _RIGEXEC_FROZEN_ARRAY(VtFloatArray, float)
    _RIGEXEC_FROZEN_ARRAY(VtDoubleArray, double)
    _RIGEXEC_FROZEN_ARRAY(VtVec2fArray, GfVec2f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3fArray, GfVec3f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3dArray, GfVec3d)
    _RIGEXEC_FROZEN_ARRAY(VtVec4fArray, GfVec4f)
#undef _RIGEXEC_FROZEN_ARRAY
    if (value.IsHolding<std::string>()) {
        *hash = _HashString(*hash, value.UncheckedGet<std::string>().c_str());
        return true;
    }
    if (value.IsHolding<TfToken>()) {
        *hash = _HashString(*hash, value.UncheckedGet<TfToken>().GetText());
        return true;
    }
    if (value.IsHolding<SdfPath>()) {
        *hash = _HashString(
            *hash, value.UncheckedGet<SdfPath>().GetString().c_str());
        return true;
    }
    if (value.IsHolding<VtStringArray>()) {
        const VtStringArray &held = value.UncheckedGet<VtStringArray>();
        const size_t count = held.size();
        *hash = _HashBytes(*hash, &count, sizeof(count));
        for (const std::string &entry : held) {
            *hash = _HashString(*hash, entry.c_str());
        }
        return true;
    }
    return false;
}

} // namespace frozenDetail

void
RigExecFrameInputs::Add(const SdfPath &path, const VtValue &value,
                        bool hasValue, bool viaChain, bool valueBlocked)
{
    RigExecSampledInput sampled;
    sampled.path = path;
    sampled.value = value;
    sampled.hasValue = hasValue;
    sampled.viaChain = viaChain;
    sampled.valueBlocked = valueBlocked;
    values.push_back(sampled);
}

const std::vector<RigExecPurityFinding> &
RigExecFrozenPurityAudit()
{
    // One row per unit examined. "Examined" means read for static,
    // thread-local, and member state surviving a call, plus every lock and
    // every USD handle a worker could reach through it; the verdict is what
    // the frozen path may do with the unit.
    static const std::vector<RigExecPurityFinding> audit = {
        {"rigExecMath/* (avarScale, dualQuat, pointFrame, rbf, "
         "singleChainIk, solvers, splineIk, envelope, geometryKernels, "
         "propertyMath, weightFields)",
         RigExecFrozenPurity::Pure,
         "free functions over their arguments; no static, thread-local, or "
         "member state anywhere in the directory"},
        {"rigExecMath extended deformer and frame kernels (deltaMushKernel "
         "settings, latticeKernel regular grid, surfaceSnapKernel, "
         "affineFrameKernels)",
         RigExecFrozenPurity::Pure,
         "free functions over their arguments; settings arrive in the "
         "packet or the op inputs, and text settings parse from views"},
        {"provider affine frame expressions (rigExecScene/providerProgram.cpp "
         "AffineFrame ops)",
         RigExecFrozenPurity::Pure,
         "ordinary provider ops over declared attribute values and provider "
         "point frames; the expression table is namespace-scope, built at "
         "load; token values are read as text, never interned"},
        {"built-in external movers (movers/armatureMover.cpp, "
         "movers/surfaceBindingMover.cpp)",
         RigExecFrozenPurity::Pure,
         "assembly and application read only the declared inputs and the "
         "provider values; payload keys are plain strings; no token built "
         "from text, no lock"},
        {"libs/rigExec/solverKernels.{h,cpp}",
         RigExecFrozenPurity::Pure,
         "ribbon/twist/rotation glue over caller buffers; no statics, no "
         "locks, no USD"},
        {"libs/rigExec/moverKernels.cpp exec callbacks and helpers",
         RigExecFrozenPurity::Pure,
         "only static const tokens and the SIMD switch, read once at "
         "library load (RigExecSimdEnabled); per-evaluation state lives in "
         "the VdfContext, never in the kernel"},
        {"libs/rigExec/computations.cpp frame helpers",
         RigExecFrozenPurity::Pure,
         "static free functions over frames and params; guards and packing "
         "only, no retained state"},
        {"libs/rigExec/weightPackets.cpp packet math",
         RigExecFrozenPurity::Pure,
         "one token table, touched at Build; assembly over caller "
         "buffers"},
        {"baked step bodies (bakedPose/bakedGeometry/bakedWeights/"
         "bakedVerify.cpp)",
         RigExecFrozenPurity::Pure,
         "a step reads declared slots and writes declared slots; per-step "
         "diagnostics merge in canonical order; outputs persist between "
         "jobs and exact value keys control propagation. Bodies consume "
         "copied source facts and typed values without USD queries; API4 "
         "plugins receive immutable compile data and declared typed inputs"},
        {"property operations (bakedProperties.cpp)",
         RigExecFrozenPurity::Pure,
         "ordinary shared-graph operations over sampled leaves, explicit "
         "overrides, and bound property versions; no separate head executor"},
        {"reader walks (bakedProperties.cpp RigExecBakedResolveReaderWalk)",
         RigExecFrozenPurity::Pure,
         "every declared read that a chain result or a record "
         "can answer -- a chain-routed binding, a path leaf read through "
         "the resolved inputs -- resolves from head leaves, override slots, "
         "chain finals and records: no USD, no path built, no lock; live "
         "and a frozen job call the same function"},
        {"shared operation graph executor",
         RigExecFrozenPurity::Pure,
         "one readiness and value-cutoff loop for native, frozen, and runtime; "
         "caller-owned workspaces retain exact semantic state. Dispatch "
         "callbacks choose serial or parallel execution without changing dependencies"},
        {"per-point geometry kernels (moverGraph.cpp parallel regions)",
         RigExecFrozenPurity::Pure,
         "range-independent per-point math; the per-context serial hook "
         "(D4) keeps frozen runs on the serial variants, which compute "
         "byte-identical numbers"},
        {"RigExecBakedProgramImpl structure (steps, edges, clusters, "
         "cones, walk, rest and ladder op declarations)",
         RigExecFrozenPurity::CompiledRigPinned,
         "immutable after Build; safe as the programDigest check names it, "
         "never as a live read of per-frame working state"},
        {"skin layout handles (bakedGeometry.cpp SkinTopology ops)",
         RigExecFrozenPurity::Pure,
         "each layout operation consumes current sampled leaves and declared "
         "property versions; unchanged semantic layouts retain their handle. "
         "Geometry adopts the handle inside its owning operation"},
        {"ladder tables (restM through rotOrder)",
         RigExecFrozenPurity::Pure,
         "RestCompose and LadderCompose are graph operations over copied "
         "source facts and declared provider values; each owns its output slot"},
        {"upstream layer (upstream, lastUpstream, upstreamOn, "
         "upstreamChanged)",
         RigExecFrozenPurity::Pure,
         "per program: a frozen job sets its clone's from the vector's "
         "admitted values and diffs them against the snapshot's by "
         "RigExecBakedPlaceUpstream with the oracle placement off -- map "
         "compares and leaf marks, no stage, no admission-set build; every "
         "read they reach was sampled through the same values on the UI "
         "thread"},
        {"skin/blend bindings (shared_ptr<const> topologies and layouts)",
         RigExecFrozenPurity::CompiledRigPinned,
         "immutable snapshots resolved at Build/prologue; the worker runs "
         "from its own references, never from the live caches"},
        {"RigExecStaticInputCache VALUES",
         RigExecFrozenPurity::CompiledRigPinned,
         "admission promises the same answer at every time code; safe as "
         "sampled values, while the cache OBJECT is the row below"},
        {"UsdStage, UsdAttribute, UsdAttributeQuery, UsdPrim, "
         "UsdGeomXformCache",
         RigExecFrozenPurity::LiveOnly,
         "every prologue read; sampled into the input vector on the UI "
         "thread, never named by the context, inputs, arena, or runner"},
        {"RigExecRigEvaluator (resolved inputs, live graphs, tap sets, "
         "overrides, profiler, compiled rig rests)",
         RigExecFrozenPurity::LiveOnly,
         "live state by definition; the program's captured pointers to it "
         "are why workers run a private arena, never the live program"},
        {"RigExecBakedProgramImpl live pointers (evaluator, stage, "
         "resolvedInputs, profiler, "
         "guideTaps)",
         RigExecFrozenPurity::LiveOnly,
         "snapshot construction clears live pointers; each frozen lane "
         "rebinds only its private resolver, profiler, and copied compiled rig tables"},
        {"ExecUsdSystem / TapSet / Snapshot (dynamic path)",
         RigExecFrozenPurity::LiveOnly,
         "owning-thread source capture and optional reference checks only; "
         "workers consume fresh detached facts and compiled operations"},
        {"wire-basis memo (moverGraph.h RigExecWireBasisCache, per owner)",
         RigExecFrozenPurity::Pure,
         "one per revision (GeomRevision::wireBasis) and one per dynamic "
         "chain graph, each touched only by the step or task that runs it, "
         "so no lock; answer-preserving (full-input compare on hit, a pure "
         "rebuild on a miss); a frozen clone copies the revision's map, "
         "whose entries are immutable and shared, and memoizes into its "
         "own copy"},
        {"revision kernel memo (rigExecMath/surfaceKernelCache.h: "
         "adjacency, delta-mush rest, wrinkle topology, lattice basis)",
         RigExecFrozenPurity::Pure,
         "one per revision (GeomRevision::surfaceCache; the runtime's "
         "revision scratch), touched only by the step that runs the "
         "revision, so no lock; answer-preserving (raw-bit compare of every "
         "input an entry reads, a pure rebuild on a miss); a frozen clone "
         "shares the immutable entries and replaces only its own slots; "
         "before a run dispatches, the thread running the program points "
         "revisions' equal lattice binds at one immutable instance "
         "(RigExecBakedShareLatticeBinds), and a bind over the budget is "
         "streamed rather than retained"},
        {"RigExecStaticInputCache / RigExecBlendSampleCache OBJECTS",
         RigExecFrozenPurity::LiveOnly,
         "single-threaded or notice-invalidated live state (THREAD rule); "
         "workers use sampled values and held bindings, never the caches"},
        {"RigExecSkinTopologyCache",
         RigExecFrozenPurity::LiveOnly,
         "dynamic only; baked and frozen build layouts in the SkinTopology "
         "op"},
        {"calibration/timing frame counters (program framesSeen fields)",
         RigExecFrozenPurity::LiveOnly,
         "per-program diagnostic counters the owning Run epilogue counts; "
         "frozen runs never enable calibration or step timing, so workers "
         "never touch them"},
        {"registry mutex / imaging bridge / snapshot store writes",
         RigExecFrozenPurity::LiveOnly,
         "Stream E's publish fence; background completions publish into "
         "the cache only, never into the snapshot, and never take the "
         "registry lock"},
    };
    return audit;
}

uint64_t
RigExecFrameCacheCompiledRigSignature(const RigExecRigEvaluator &evaluator)
{
    uint64_t h = evaluator.GetCompiledRigSignature();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return h;
    }
    const uint64_t builds = uint64_t(evaluator.GetBakedProgramBuildCount());
    h = _HashBytes(h, &builds, sizeof(builds));
    // Structure only: the binding-table shapes, never the
    // constant values. A value patch moves the constant digest the
    // control half folds -- not this -- so it opens a new key namespace
    // instead of evicting the compiled rig.
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const uint64_t constants = uint64_t(B.avarConstants.size());
    h = _HashBytes(h, &constants, sizeof(constants));
    const uint64_t bindings = uint64_t(B.avarConstantBindings.size());
    h = _HashBytes(h, &bindings, sizeof(bindings));
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        const uint64_t slot = uint64_t(binding.slot);
        h = _HashBytes(h, &slot, sizeof(slot));
    }
    const uint64_t varying = uint64_t(B.avarBindings.size());
    h = _HashBytes(h, &varying, sizeof(varying));
    const uint64_t table = uint64_t(B.avars.size());
    h = _HashBytes(h, &table, sizeof(table));
    return h;
}

uint64_t
RigExecCompiledRigConstantDigest(const RigExecBakedProgram &program)
{
    return RigExecFrozenAvarRegionDigest(program);
}

std::vector<uint64_t>
RigExecConstantRegionsForPaths(const RigExecBakedProgram &program,
                               const std::vector<SdfPath> &paths)
{
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    std::vector<uint64_t> regions;
    for (const SdfPath &path : paths) {
        const auto found = B.patchableAvars.find(path);
        if (found == B.patchableAvars.end()) {
            continue;
        }
        const uint64_t region =
            RigExecConstantRegionForBinding(found->second);
        if (regions.empty() || regions.back() != region) {
            // patchableAvars iterates in path order, not binding order,
            // so contiguity is not guaranteed; the sort below fixes it.
            regions.push_back(region);
        }
    }
    std::sort(regions.begin(), regions.end());
    regions.erase(std::unique(regions.begin(), regions.end()),
                  regions.end());
    return regions;
}

bool
RigExecCaptureFrozenAvarPatch(const RigExecFrozenProgram &base,
                                const RigExecBakedProgram &live,
                                std::shared_ptr<const RigExecFrozenAvarPatch> *out,
                                std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no snapshot to patch into");
    }
    if (!base.program || !base.bindings) return fail("empty frozen snapshot");
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    const RigExecBakedProgramImpl &S = *base.program;
    // Live's avar slots, carried below, hold the upstream values its last
    // run placed; the copy keeps the snapshot's table as its history, so
    // the two must name the same table.
    if (L.lastUpstream != S.lastUpstream) {
        return fail("upstream inputs moved since the snapshot: re-freeze, "
                    "do not patch");
    }
    // The same program object, or at least the same shape: a rebuild is
    // re-frozen, never patched.
    if (S.avarConstantBindings.size() != L.avarConstantBindings.size() ||
        S.avarConstants.size() != L.avarConstants.size()) {
        return fail("avar region changed shape: re-freeze, do not patch");
    }
    if (S.avars.size() != L.avars.size()) return fail("avar table changed shape");
    auto patch = std::make_shared<RigExecFrozenAvarPatch>();
    patch->basis = &base;
    for (size_t i = 0; i < L.avarConstantBindings.size(); ++i) {
        const auto &binding = L.avarConstantBindings[i];
        if (binding.slot >= L.avars.size() || binding.slot != S.avarConstantBindings[i].slot)
            return fail("avar slot changed: re-freeze, do not patch");
        patch->avars.push_back({binding.input.constant, L.avars[binding.slot],
                                binding.input.varying});
    }
    patch->constants = L.avarConstants;
    patch->promoted = L.promotedAvars;
    patch->varyingInputs = L.varyingInputs;
    patch->editSerial = L.editSerial;
    patch->valueEditSerial = L.valueEditSerial;
    patch->programStamp = L.programStamp;
    *out = std::move(patch);
    return true;
}

bool
RigExecApplyFrozenAvarPatch(const RigExecFrozenProgram &base,
    const RigExecFrozenAvarPatch &patch,
    std::shared_ptr<const RigExecFrozenProgram> *out, std::string *error)
{
    if (!base.program || !base.bindings) {
        if (error) *error = "empty frozen snapshot";
        return false;
    }
    const auto &S = *base.program;
    if (!out || patch.basis != &base ||
        patch.avars.size() != S.avarConstantBindings.size() ||
        patch.constants.size() != S.avarConstants.size()) {
        if (error) *error = "captured avar patch does not match frozen snapshot";
        return false;
    }
    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    snapshot->program = base.program;
    snapshot->bindings = base.bindings;
    auto edits = std::make_shared<RigExecFrozenAvarPatch>(patch);
    edits->basis = snapshot.get();
    snapshot->avarPatch = std::move(edits);
    *out = std::move(snapshot);
    return true;
}

bool
RigExecPatchFrozenAvarConstants(const RigExecFrozenProgram &base,
    const RigExecBakedProgram &live,
    std::shared_ptr<const RigExecFrozenProgram> *out, std::string *error)
{
    std::shared_ptr<const RigExecFrozenAvarPatch> patch;
    return RigExecCaptureFrozenAvarPatch(base, live, &patch, error) &&
        RigExecApplyFrozenAvarPatch(base, *patch, out, error);
}

} // namespace rigExec
