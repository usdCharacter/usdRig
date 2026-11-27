// The baked program's implementation state, and the helpers its build and run
// halves share.
// bakedProgram.cpp is the only translation unit the evaluator declares a
// friend, so everything the frame path needs out of RigExecRigEvaluator is
// captured ONCE, at Build, into the pointers below; bakedPose.cpp and
// bakedGeometry.cpp then read the program rather than the evaluator.
#ifndef RIGEXEC_BAKED_PROGRAM_IMPL_H
#define RIGEXEC_BAKED_PROGRAM_IMPL_H

#include "rigExecSchema/schemaNames.h"
#include "rigExecScene/weightProgram.h"
#include "rigExecScene/geometrySceneTranslation.h"
#include "rigExec/internal/bakedClusters.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/compiledProgram.h"
#include "rigExecScene/autoClavicleGraph.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/gpuDeformTranslation.h"
#include "rigExec/internal/scalarReferenceAdapter.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/profiler.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExec/internal/valueAddress.h"
#include "rigExec/api/types.h"
#include "rigExec/internal/weightPackets.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecGraph/opGraph.h"
#include "rigExecGraph/opValues.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecScene/providerProgram.h"
#include "rigExecScene/solverProgram.h"
#include "rigExecScene/constraintProgram.h"
#include "rigExecScene/poseProgram.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"
#include "rigExecMath/surfaceKernelCache.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecRigEvaluator;
struct RigExecRigPose;
struct RigExecHeadLeafConstants;
struct RigExecFrozenSamplerMemo;

TF_DEFINE_PRIVATE_TOKENS(
    _bakedImplTokens,
    ((avarsTx, SchemaAttrText::avarsTx))
    ((avarsTy, SchemaAttrText::avarsTy))
    ((avarsTz, SchemaAttrText::avarsTz))
    ((avarsSx, SchemaAttrText::avarsSx))
    ((avarsSy, SchemaAttrText::avarsSy))
    ((avarsSz, SchemaAttrText::avarsSz))
    ((avarsRx, SchemaAttrText::avarsRx))
    ((avarsRy, SchemaAttrText::avarsRy))
    ((avarsRz, SchemaAttrText::avarsRz))
    ((avarsRspin, SchemaAttrText::avarsRspin))
    ((avarsUnitScaleFactor, SchemaAttrText::avarsUnitScaleFactor))
    ((avarsRotationOrder, SchemaAttrText::avarsRotationOrder))
    ((avarsRotationSign, SchemaAttrText::avarsRotationSign))
    ((avarsDefaultSpace, SchemaAttrText::avarsDefaultSpace))
);

// Shared helpers.

// Provider compose uses the same arithmetic as detached scene kernels.
inline GfMatrix4d
RigExecBakedComposeAvars(double tx, double ty, double tz, double sx, double sy,
                         double sz, double rx, double ry, double rz,
                         double spin, const TfToken &order)
{
    return RigExecComposePoseAvars(tx, ty, tz, sx, sy, sz, rx, ry, rz, spin,
                                   order);
}

inline RigExecEulerOrder
RigExecBakedParseEulerOrder(const TfToken &t)
{
    if (t == "XZY")
        return RigExecEulerOrder::XZY;
    if (t == "YXZ")
        return RigExecEulerOrder::YXZ;
    if (t == "YZX")
        return RigExecEulerOrder::YZX;
    if (t == "ZXY")
        return RigExecEulerOrder::ZXY;
    if (t == "ZYX")
        return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}

// Exec hands a FRAME between computations and every matrix-typed consumer
// turns it back into a matrix (_SpaceFromFrame in computations.cpp). The
// round trip is not the identity in floating point, so the program performs
// it wherever exec does -- otherwise a deep chain drifts.
inline GfMatrix4d
RigExecBakedRoundTrip(const GfMatrix4d &m)
{
    return RigExecRoundTripT<GfMatrix3d, GfVec3d>(m);
}

inline bool
RigExecBakedUsable(const RigExecPointFrame &frame)
{
    return RigExecConstraintFrameUsable(frame);
}

// True when \p attribute or anything it resolves through carries animation,
// which is what makes the value unbakeable. A single time sample counts: it
// reads back at a numeric time but not at Default, so capturing it would make
// the answer depend on which time code the caller passed.
inline bool
RigExecBakedAnimatedOrConnected(const UsdAttribute &attribute)
{
    if (!attribute) {
        return false;
    }
    // A single time sample counts as animated: it reads back at a numeric
    // time but not at Default, so capturing it would make the answer depend
    // on which time code the caller passed.
    if (attribute.ValueMightBeTimeVarying() ||
        attribute.GetNumTimeSamples() > 0) {
        return true;
    }
    // A connection means the value comes from somewhere this check has not
    // looked at. HasAuthoredConnections first: see _AuthoredConnections in
    // rigEvaluator.cpp.
    SdfPathVector connections;
    return attribute.HasAuthoredConnections() &&
           attribute.GetConnections(&connections) && !connections.empty();
}

// The attribute GetAttribute would end up reading, plus why it cannot be
// captured.
template <class T>
inline void
RigExecBakedClassifyInput(const UsdAttribute &attribute, UsdTimeCode time,
                          const std::set<SdfPath> &chainTargets, bool *viaChain,
                          bool *varying, UsdAttribute *selected,
                          SdfPathVector *walk = nullptr)
{
    std::set<SdfPath> visiting;
    std::vector<UsdAttribute> fallback;
    UsdAttribute a = attribute;
    while (a && visiting.insert(a.GetPath()).second) {
        if (walk) {
            walk->push_back(a.GetPath());
        }
        if (chainTargets.count(a.GetPath())) {
            *viaChain = true;
        }
        // Both halves of the OR are needed and neither implies the other.
        // ValueMightBeTimeVarying is false for an attribute whose strongest
        // opinion is exactly ONE time sample of a non-composable type
        // (UsdStage::_ValueMightBeTimeVaryingFromResolveInfo), and the
        // capture below reads at Default, which never sees a time sample --
        // so a single-keyed avar would be captured as its schema fallback.
        // GetNumTimeSamples is 0 for a Ts-spline valued attribute in this
        // USD build while it varies. This is the same predicate
        // RigExecBakedAnimatedOrConnected applies further down, for the
        // same reason.
        if (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0) {
            *varying = true;
        }
        fallback.push_back(a);
        SdfPathVector connections;
        a.GetConnections(&connections);
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    for (auto it = fallback.rbegin(); it != fallback.rend(); ++it) {
        T probe{};
        // A keyed attribute with no default answers at every frame but not
        // at Default, so it is probed at its first key; otherwise the input
        // has no handle and every frame reads its fallback.
        const bool keyed = it->GetNumTimeSamples() > 0 || it->HasSpline();
        if (it->Get(&probe, time) ||
            (keyed && it->Get(&probe, UsdTimeCode::EarliestTime()))) {
            *selected = *it;
            return;
        }
    }
}

/// \p name pre-interned: the parallel resolve lanes pass file-scope tokens,
/// because TfToken(const char *) takes the token registry's lock.
template <class T>
inline RigExecBakedInput<T>
RigExecBakedBindInput(const UsdPrim &prim, const TfToken &name, T fallback,
                      UsdTimeCode time, const std::set<SdfPath> &chainTargets,
                      SdfPathVector *walk = nullptr)
{
    RigExecBakedInput<T> input;
    input.constant = fallback;
    if (!prim) {
        return input;
    }
    const UsdAttribute attribute = prim.GetAttribute(name);
    if (!attribute) {
        return input;
    }
    input.head = attribute;
    bool viaChain = false, varying = false;
    UsdAttribute selected;
    RigExecBakedClassifyInput<T>(attribute, time, chainTargets, &viaChain,
                                 &varying, &selected, walk);
    if (viaChain) {
        input.varying = true;
        input.resolvedAttr = attribute;
        return input;
    }
    if (varying) {
        input.varying = true;
        if (selected) {
            input.query = UsdAttributeQuery(selected);
        }
        return input;
    }
    if (selected) {
        selected.Get(&input.constant, time);
    }
    return input;
}

// What \p input resolves to at \p time. Only the leaf samplers call it (live:
// RigExecBakedSampleLeaves in the prologue; frozen: the worker prologue over
// its patched constants); step bodies and the input fill read the sampled
// leaf through RigExecBakedLeafRead.
template <class T>
inline T
RigExecBakedRead(const RigExecBakedInput<T> &input,
                 const RigExecResolvedInputs &resolved, UsdTimeCode time,
                 const std::vector<char> *overridden = nullptr,
                 const std::map<SdfPath, VtValue> *upstream = nullptr,
                 const std::vector<char> *upstreamOn = nullptr)
{
    RIGEXEC_PURITY_CHECK();
    // Detached jobs have already resolved overrides and upstream sources.
    if (input.sourceBacked && !input.head)
        return input.constant;
    T value = input.sourceBacked ? input.sourceFallback : input.constant;
    const auto flagged = [&input](const std::vector<char> *flags) {
        return input.overrideIndex >= 0 && flags &&
               size_t(input.overrideIndex) < flags->size() &&
               (*flags)[size_t(input.overrideIndex)];
    };
    // A held drag is placed by reading THE LONG WAY. GetAttribute is the same
    // walk exec's accessor performs and _resolvedInputs already carries the
    // override, so the answer is the dynamic one rather than a second
    // approximation of it -- and the pinned query, which knows nothing about
    // the drag, is bypassed for as long as it stands. An upstream value on
    // a hop is read the same way, through the upstream layer in front of
    // the stage.
    if (flagged(overridden) || flagged(upstreamOn)) {
        resolved.GetAttributeOverStageLayer(input.head, time, upstream, &value);
        return value;
    }
    if (input.sourceBacked) {
        // A live source without a usable query retains the argument fallback.
        if (!input.query.IsValid())
            return input.sourceFallback;
        input.query.Get(&value, time);
        return value;
    }
    if (!input.varying) {
        return value;
    }
    if (input.resolvedAttr) {
        resolved.GetAttributeOverStageLayer(input.resolvedAttr, time, upstream,
                                            &value);
        return value;
    }
    if (input.query.IsValid()) {
        input.query.Get(&value, time);
    }
    return value;
}

// Sampled leaves.
// One leaf per binding frozenDetail::_ForEachPatchableInput visits, numbered
// in its order. RigExecBakedSampleLeaves fills them on the owning thread
// before the region from RigExecBakedRead, under the re-sample rules it
// states; everything after it reads the leaf. A leaf has no fold or varying
// bit of its own: it reads its binding's, so Build and
// RigExecProgramAvarPatch stay the only writers of what a binding is.

/// The values the stored source keys were built from, at each entry of
/// `index`, and the generation they share with those keys
/// (RigExecOpAdapterState::sourceWatchSerial): a copy or restore of the
/// adapter state without this watch leaves the two generations apart, and
/// the next run rebuilds every key. Owner of the program only.
struct RigExecBakedSourceWatch {
    std::shared_ptr<const RigExecBakedSourceLeafIndex> index;
    uint64_t serial = 0;
    /// Typed entries' values, at their pool positions.
    RigExecBakedLeafPools typed;
    /// Per entry: the Head, PathValue and Override values, and the
    /// PathVersion version, the AvarConstant's bits and a Head's type match.
    std::vector<VtValue> values;
    std::vector<uint64_t> words;
    bool publishWeightFields = false;
    /// Per op: whether its last key build was exact, and this run's verdict
    /// that its key can have moved.
    std::vector<char> exact, dirty;
    /// The leafFlagCompiledRig and avarConstantSerial at the last visit, the head
    /// override slots non-empty then, the ops its verdict marked dirty (each
    /// once, ascending) and the standing ones (untracked or inexact).
    uint64_t flagCompiledRig = 0, avarSerial = 0;
    std::vector<uint32_t> overrideSlotsHeld, dirtyList, standing;
    bool standingStale = true;
    /// RIGEXEC_SPARSE_SOURCE_WATCH (default on), read with the index.
    bool sparseVisit = true;
};

/// Publishes \p value at \p key into \p map, in one comparison when the
/// caller walks its keys in ascending order.
///
/// \p inOrder is the caller's promise that every key it publishes into this
/// map is strictly greater than the last one it published, which makes the
/// map's end the correct hint and the insertion O(1) rather than a search
/// from the root. Strictly greater matters twice over: it is what makes the
/// hint right, and it is what makes an emplace the same publication as the
/// assignment it replaced, because no key is ever published twice. Without
/// the promise this IS that assignment.
template <class Map, class Value>
inline void
RigExecBakedEmplace(Map *map, bool inOrder, const SdfPath &key,
                    const Value &value)
{
    if (inOrder) {
        map->emplace_hint(map->end(), key, value);
    } else {
        (*map)[key] = value;
    }
}

// True when \p input has to be re-read; used only to size the report.
template <class T>
inline bool
RigExecBakedIsVarying(const RigExecBakedInput<T> &input)
{
    return input.varying;
}
inline constexpr const char *const RigExecBakedAvarNames[11] = {
    SchemaAttrText::avarsTx,
    SchemaAttrText::avarsTy,
    SchemaAttrText::avarsTz,
    SchemaAttrText::avarsSx,
    SchemaAttrText::avarsSy,
    SchemaAttrText::avarsSz,
    SchemaAttrText::avarsRx,
    SchemaAttrText::avarsRy,
    SchemaAttrText::avarsRz,
    SchemaAttrText::avarsRspin,
    SchemaAttrText::avarsUnitScaleFactor};
inline constexpr double RigExecBakedAvarDefaults[11] = {0, 0, 0, 1, 1, 1,
                                                        0, 0, 0, 0, 1};
inline const char *
RigExecBakedOpName(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return "matrix";
    case RigExecRevisionOp::Skin:
        return "skin";
    case RigExecRevisionOp::BlendShape:
        return "blendShape";
    case RigExecRevisionOp::VolumeCorrect:
        return "volumeCorrect";
    case RigExecRevisionOp::Smooth:
        return "smooth";
    case RigExecRevisionOp::DeltaMush:
        return "deltaMush";
    case RigExecRevisionOp::Wrinkle:
        return "wrinkle";
    case RigExecRevisionOp::Lattice:
        return "lattice";
    case RigExecRevisionOp::SurfaceProject:
        return "surfaceProject";
    case RigExecRevisionOp::Ribbon:
        return "ribbon";
    case RigExecRevisionOp::Wire:
        return SchemaValueText::wire;
    case RigExecRevisionOp::EmitGuidePoints:
        return "emitGuidePoints";
    case RigExecRevisionOp::External:
        return "external";
    case RigExecRevisionOp::RecomputeNormals:
        return "recomputeNormals";
    case RigExecRevisionOp::RecomputeExtent:
        return "recomputeExtent";
    case RigExecRevisionOp::SurfaceProjector:
        return "surfaceProjector";
    case RigExecRevisionOp::ShaderDials:
        return "shaderDials";
    }
    return "unknown";
}

RigExecBakedReaderWalk RigExecBakedBuildWeightScalarRead(
    RigExecBakedProgramImpl *, const UsdAttribute &, const std::vector<int> &,
    RigExecBakedWeightScalarReadIndex *index = nullptr);
float RigExecBakedReadWeightScalar(const RigExecBakedProgramImpl &,
                                   const RigExecBakedReaderWalk &,
                                   const std::vector<int> &, float);
bool RigExecBakedResolveWalkValue(const RigExecBakedProgramImpl &,
                                  const RigExecBakedWalk &, VtValue *);

class RigExecBakedExecCheckRows;

/// RigExecBakedSampleLeaves' index of the patchable inputs, built on first
/// sparse use by the program object it describes and rebuilt whenever
/// `owner` or `leafCount` differs, so a clone or a rebuild never follows
/// another program's pointers. Owner thread.
struct RigExecBakedSparseInputs {
    const void *owner = nullptr;
    size_t leafCount = 0;
    /// Per leaf id: the input, a RigExecBakedInput<T>* with T named by
    /// leafRefs[id].type.
    std::vector<void *> input;
    /// Leaf ids of the inputs whose `varying` was set at the build, walk < 0.
    std::vector<uint32_t> varying;
    /// Override number n's leaf ids: numberLeaves[numberBegin[n],
    /// numberBegin[n + 1]); walk < 0 only.
    std::vector<uint32_t> numberBegin, numberLeaves;
    /// Per leaf id, the pass stamp that last visited it.
    std::vector<uint64_t> visited;
    uint64_t pass = 0;
};

struct RigExecBakedGeomRevision {
    RigExecGeometryRecord kernelRecord;
    RigExecSceneGeometryDescriptor sceneGeometry;
    SdfPath moverPath;
    /// moverPath spelled at Build, for the bodies' diagnostics and
    /// status addresses.
    std::string moverPathText;
    SdfPath target;
    UsdPrim moverPrim;
    RigExecRevisionOp op = RigExecRevisionOp::Skin;
    RigExecRevisionBinding binding;
    /// This revision's blend channels, in `binding.blendInputs` order.
    /// Empty for every operation but a blend shape.
    std::vector<RigExecBakedGeomBlendChannel> blendChannels;
    bool blendChunked = false;
    std::vector<RigExecBlendChannel> blendSamples;
    std::vector<int> influenceSlots;
    int transformSlot = -1;
    /// The rigExec:transformSpace provider's slot, or -1.
    int transformSpaceSlot = -1;
    /// The rigExec:space provider's slot, or -1: the prim whose own rest->pose
    /// map carries the whole rig, normally a TRS master. Read out of the SAME
    /// base/final matrix table as the two above, because that table is what
    /// computeMatrix publishes.
    int carrySpaceSlot = -1;
    /// The geometry-domain constraint whose delta IS this revision's
    /// transform, as an index into the dense delta tables, or -1. Joined
    /// on the mover path at Build, because that is the key the dynamic
    /// walk's own hand-off uses.
    int constraintDelta = -1;
    /// The solver whose aggregate supplies values.driverFrames, as an
    /// index into `solvers`, or -1 when this revision reads none. The
    /// dynamic path takes it off a per-revision tap on the solver's
    /// computePointFrameArray; the program already holds that aggregate,
    /// so the tap becomes a slot. Always -1: IsBakeable refuses driver frames
    /// on mover.
    int driverFramesSolver = -1;
    bool finalPhase = false;
    /// rigExec:pointFrame == "posed": a skin already carried
    /// this cluster's points into the rig's posed frame, so the
    /// measured offset is conjugated by the rig's carry -- or, with no
    /// rigExec:space named, by the measuring space's own scale.
    bool posedPoints = false;
    /// Compile's judgement that none of this skin mover's layout arrays
    /// can change within the compiled rig, so the packet may carry the layout
    /// by handle out of the evaluator's cache instead of re-reading and
    /// re-validating 105k elements every frame.
    bool skinTopologyFixed = false;
    /// The node's last published status, which outlives an evaluation it
    /// did not take part in -- so does its diagnostic.
    TfToken resultStatus;
    /// The revision's applied points, which `currentSource` publishes,
    /// and the buffer its chunks write. An applying fuse swaps an
    /// unchunked revision's two buffers instead of copying one into the
    /// other; a chunked skin keeps its ranges in staging.
    std::vector<GfVec3f> output;
    std::vector<GfVec3f> stagingOutput;
    // Last evaluated packet/status and result, so an unchanged input
    // re-publishes instead of re-running.
    RigExecMoverParameters lastParameters;
    /// A DERIVED revision's `auxPoints` -- the chain's final points, up
    /// to 315KB of them -- held here as the handle the chain published
    /// rather than copied into `lastParameters` above, which is left
    /// with that field empty. The comparison agrees with the elementwise
    /// one by construction: VtArray is copy-on-write and its operator==
    /// short-circuits on a shared buffer, so this is the same test with
    /// the identity case taken first. Empty, and never read, for every
    /// other revision.
    VtVec3fArray lastAuxPoints;
    RigExecMoverStatus lastStatus;
    bool ran = false;
    /// The chain runs as one GPU program (RigExecBakedGeomChain::gpu): this
    /// revision's chunks and fuse move no points, and a blend keeps its
    /// channels unsummed in `gpuChannels` for the translation. `gpuApplies`
    /// is the fuse's apply decision.
    bool gpuOwned = false;
    std::vector<RigExecBlendChannel> gpuChannels;
    bool gpuApplies = false;
    /// A step that feeds the program (its packet, its influence table or
    /// its fuse) ran this run: the boundary translates the chain. Each step
    /// writes only its own revision's flag.
    bool gpuTouched = false;
    /// This revision's own wire-basis memo: only its step runs its
    /// kernel (a range-pipelined revision's RevisionStatic, which its
    /// range steps follow), so nothing else touches it. A clone shares
    /// the entries.
    RigExecWireBasisCache wireBasis;
    /// Mutated only by this revision's unchunked body/projector, or by a
    /// range-pipelined revision's RevisionStatic; its range steps only
    /// read it. Copies share immutable entries; each clone replaces its
    /// own cache slots. AssignChainRoles decides whether it pins topology.
    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> surfaceCache;
    /// A read phase named this revision as the point in the chain it
    /// wants the target's points from (the dynamic walk records them
    /// after it). Decided at bake out of the evaluator's
    /// _nativePhaseCheckpoints; BindPointReads binds `preceding` reads of
    /// another chain and AtPrim reads to it. The plan also names the
    /// predecessor of an own-chain `preceding` reader, but that read
    /// binds to the version entering the reader and reads no record.
    bool snapshotAfter = false;
    /// One per `binding.phases` entry, in the map's order.
    std::vector<RigExecBakedPointsBinding> pointBindings;
    /// For an AtPrim `binding.transformPhase` only: the `frameRecords`
    /// of the transform provider whose writer (constraint or solver) is
    /// the phase's prim or under it, newest first. The fold takes the first valid one,
    /// and otherwise keeps the dense-table matrix. `influenceRecords`
    /// is the same per `influenceSlots` entry.
    std::vector<int> transformRecords;
    std::vector<std::vector<int>> influenceRecords;
    /// This node is new to the rig's geometry state and its creation has not
    /// been reported yet. Cleared by AdoptGeometryStateFrom for a node the
    /// outgoing program already held.
    bool created = true;

    /// The influence table the PACKET carries, which for a skin revision
    /// is identity and nothing else: the packet is assembled before the
    /// matrices are folded, precisely so a chunk can start on its own
    /// joints without waiting for every joint of the rig. The assembler
    /// checks the table's shape and its elements' finiteness, which
    /// identities pass; the real table's check is the fold's, and the
    /// fuse ANDs it in where the assembler's would have landed. Written
    /// once, at Build.
    std::vector<GfMatrix4d> packetInfluences;
    /// InfluenceFold writes these; every operation but a skin assembles
    /// against them. Sized at Build, never resized in the region.
    std::vector<GfMatrix4d> influences;
    GfMatrix4d transform{1.0};
    /// Whether `transform` holds one at all this run: a bound transform
    /// provider, or a geometry-domain constraint's delta. Written by the
    /// fold beside the table, because "there is a matrix" is part of what
    /// the table says.
    bool haveTransform = false;
    /// rigExec:space's matrix this run, out of the same base/final
    /// table as `transform`, and whether the revision named one at all.
    /// Written by the fold, because the fold is the step that declares
    /// the matrix reads; the assemble only reads what the fold wrote.
    /// A cluster conjugates its offset by it, a posed transform-driven
    /// wire carries its control polygons by it.
    GfMatrix4d carry{1.0};
    bool haveCarry = false;
    /// The resolved inputs overlaid with what this revision's point
    /// bindings resolve to (RigExecBakedOverlayPointReads), built only
    /// when the revision declares a read phase. Per revision, never one
    /// buffer shared by the walk.
    RigExecResolvedInputs revisionInputs;
    /// RevisionStatic writes these three.
    ///
    /// `status` is the status OF THE PACKET, which for a skin revision
    /// is not the whole answer: the packet carries identities where the
    /// influence matrices would be, so a table the dynamic path's
    /// assembler would have failed leaves this saying "ok" and
    /// `influencesValid` below saying otherwise. The fuse ANDs the two
    /// and publishes `moverFailed` where the dynamic path publishes it;
    /// read `influencesValid` beside this one, never this one alone.
    RigExecMoverParameters parameters;
    /// An External revision's payload, assembled with `parameters`.
    RigExecExternalRevision external;
    RigExecMoverStatus status;
    /// Whether this revision has to run at all: chain dirty so far, or
    /// it never ran, or its packet or status moved. A VALUE comparison,
    /// never a dirtiness flag -- a control dragged back to where it
    /// started must not count as executed.
    bool executed = false;
    /// The revision's vertex chunks, in vertex order, covering
    /// [0, pointCount) exactly once. Always at least one; more only for
    /// a skin revision whose layout the compiled rig fixed (see
    /// RigExecBakedPartitionRevision).
    std::vector<RigExecBakedGeomChunk> chunks;
    /// Where this revision's chunks start in the RevisionOut domain.
    int chunkBase = 0;
    /// The handle the chunk keys describe, so a frame can tell in O(1)
    /// whether they still describe the vertices: null until the first
    /// adoption, then the last handle whose arrays were Build's
    /// (RigExecBakedAdoptPartition). The keys themselves never move
    /// after Build.
    std::shared_ptr<const RigExecSkinTopology> partitionTopology;
    /// The `jointIndices` and element size Build cut the keys from.
    /// The indices are held only while the revision is chunked.
    VtIntArray partitionIndices;
    int partitionElementSize = 0;
    size_t partitionIndexCount = 0;
    size_t partitionPointCount = 0;
    /// The keys are used -- more than one chunk, so an influence outside
    /// a chunk's key is an influence that chunk will not see.
    bool chunked = false;
    /// The natural vertex cut and its exact typed matrix producer sets.
    /// Bounds count distinct producers, independently of graph scheduling.
    size_t partitionCandidates = 0;
    int partitionProducerMin = 0;
    int partitionProducerMax = 0;
    size_t partitionDistinctReads = 0;
    std::vector<std::vector<std::pair<uint8_t, int>>> partitionProducerSets;

    /// The layout half of the skin kernel's validation (the matrix half
    /// is `influencesValid` below).
    bool layoutUsable = false;
    bool dqWeightsSafe = false;
    /// The envelope, resolved ONCE at the full point count because it
    /// resolves atomically, and the predicate that says the blend is the
    /// identity and the resolution therefore dead. A skin's, and a dense
    /// wire's separate blend.
    std::vector<float> envelope;
    bool envelopeOk = false;
    bool fullStrength = false;
    /// Whether the revision applies, decided by RevisionStatic over
    /// `precedingCount` points from the validation its kernel runs first
    /// (RigExecRevisionKernelAcceptance; a skin's from the packet half,
    /// the fuse ANDing in the fold's `influencesValid`). Part of the
    /// packet: the fuse selects by it and every chunk's `ok` agrees with
    /// it. Deferred leaves the answer to the chunks.
    RigExecRevisionAcceptance acceptance = RigExecRevisionAcceptance::Refuses;
    /// The points this revision is applied to, which is the size every
    /// chunk writes within.
    size_t precedingCount = 0;
    /// The packet or the status moved, or the revision never ran. The
    /// chain's sticky bit and the influence table's compare are ORed in
    /// by the fuse; this is only what the static half saw.
    bool staticDirty = false;
    /// `inputs:defaultWeight` as the stage reads it, and the value the
    /// last run read.
    ///
    /// The one number the FUSE would otherwise have to read off the
    /// stage, for the one diagnostic it can emit. A source reads it
    /// instead, and compares it, so that the fuse is a pure function of
    /// its declared slots and a run that skips the fuse is a run in
    /// which this did not move either.
    float defaultWeight = 0, lastDefaultWeight = 0;
    /// The partition no longer describes the layout the packet carries,
    /// so the keys cannot be trusted and the fuse runs the revision
    /// whole rather than a chunk deforming a vertex against an identity.
    bool partitionStale = false;

    /// rigExec:weightObject as an index into `weightObjects`, or -1.
    /// The packet itself is shared -- one per object per frame, however
    /// many movers bind it -- so what a revision holds is the index.
    int weightObject = -1;
    int weightField = -1;
    /// Whether the weight object weights the MOVER (one declared target, and it
    /// is this mover) rather than the points. Decided at Build because it is a
    /// relationship read, and the relationship is compiled rig state.
    bool weightOperationDomain = false;
    /// The property the published field says it weights: the mover for
    /// an operation-domain object, the chain's target otherwise.
    SdfPath weightFieldTarget;
    /// The field is measured against the points ENTERING this revision,
    /// so the shared packet is not this revision's answer: it patches a
    /// copy of its own, exactly the fields the dynamic path patches, and
    /// assembles against that.
    bool weightCurrentPhase = false;
    RigExecWeightPacket currentPhasePacket;
    /// The field this revision published this run, and whether it
    /// published one at all. Written by RevisionStatic -- which is where
    /// the dynamic path publishes it, from the packet the mover is about
    /// to consume -- and drained by the epilogue in chain order, which
    /// shares the array rather than copying it.
    VtFloatArray publishedWeightValues;
    bool weightFieldPublished = false;
    /// The content versions the RevisionPacket key carries in place of
    /// `envelope`'s and `publishedWeightValues`' bytes: RevisionStatic
    /// bumps each exactly when it leaves its array holding other bytes
    /// than it published last (RigExecBakedNoteFloats).
    uint64_t envelopeVersion = 0, weightValuesVersion = 0;
    /// `publishedWeightValues` holds the resolve of WeightPacket op
    /// value `weightPacketValue` at its revision
    /// `weightValuesPacketRevision` over `weightValuesCount` points
    /// (`weightValuesHeld`), which RevisionStatic reuses while both
    /// stand. Never held for a current-phase revision. The value id is
    /// set at compile, which drops the claim.
    int weightPacketValue = -1;
    bool weightValuesHeld = false;
    uint64_t weightValuesPacketRevision = 0;
    size_t weightValuesCount = 0;
    /// RevisionStatic's resolve buffer: compared with the published
    /// array before it replaces it. Contents are not state.
    std::vector<float> resolveScratch;

    /// Every influence matrix finite and affine. For a skin revision the
    /// packet cannot answer this -- it is assembled before the matrices
    /// are folded -- so the fuse ANDs this in where the dynamic path's
    /// assembler would have failed the packet.
    bool influencesValid = false;
    /// The table moved since the last run, which is the half of the
    /// executed decision the packet comparison does not carry.
    bool influencesChanged = false;
    /// The table in the two forms the kernels want it in, written by
    /// the fold for every skin revision: an unchunked one's single
    /// chunk skins against them, and so does the fuse when it has to run
    /// a revision whole. A chunked revision's chunks keep their own
    /// beside these and never read them -- but the fuse may only READ
    /// this slot, so the fold writes them regardless.
    std::vector<float> rows;
    std::vector<RigExecScaledDualQuat> palette;
    /// Which buffer holds the chain's points AFTER this revision: this
    /// revision's index when its own output was kept, the index of an
    /// earlier revision when it was not, and -1 for the chain's base.
    int currentSource = -1;
    /// An unchunked revision's chunk wrote `stagingOutput` and no fuse
    /// has applied it yet; while false, `output` holds the chunk's
    /// latest result.
    bool stagingFresh = false;
    /// An exact copy of the points the last RevisionDone publication
    /// carried when the revision passed through: the baseline the next
    /// publication is compared against. While it applied, the baseline
    /// is `output` itself.
    std::vector<GfVec3f> passedPoints;
    /// The content version RevisionDone and ChainDirty key the points
    /// by: the fuse bumps it exactly when the points it publishes differ,
    /// byte for byte, from the ones its previous publication carried.
    uint64_t doneVersion = 0;
    /// The compiled rig skin layout the packet carries: the SkinTopology
    /// op's `layoutHandle`, adopted where the geometry prologue resolves
    /// a topology (a chain revision whose chain read a base, a derived
    /// revision with a base). Null is a refused layout, which is why
    /// `topologyResolved` and not the pointer says whether the prologue
    /// answered. Both are exported.
    std::shared_ptr<const RigExecSkinTopology> topology;
    bool topologyResolved = false;
    /// The SkinTopology head op of a `skinTopologyFixed` revision.
    /// `layoutLeaves` are the layout's three reads
    /// (RigExecDeclareSkinLayoutLeaves), sampled on the owning thread
    /// before the op runs, and `layoutOverlay` the overlay entry each
    /// last saw at its path. `layoutFixed` is RigExecSkinLayoutIsFixed:
    /// its topology half (`layoutTopologyFixed`) asked at Build only, as
    /// an edit to either path rebuilds the program, and its weights half
    /// asked at Build and again when a value edit reaches the weights or
    /// the program stamp moves; `layoutFixedChanged` says
    /// the last sample moved it. `layoutHandle` is the op's output: null
    /// while the layout is not compiled rig state, else the layout the leaves
    /// describe, the same object for as long as they describe the same
    /// layout. `layoutCandidate` is the last non-null handle, handed back
    /// when a rebuild finds the same layout so that the packet still
    /// compares equal; AdoptGeometryStateFrom carries it into a rebuilt
    /// program.
    RigExecBakedPathLeaves layoutLeaves;
    std::vector<VtValue> layoutOverlay;
    bool layoutTopologyFixed = false;
    bool layoutFixed = false;
    bool layoutFixedChanged = false;
    bool layoutRan = false;
    bool layoutOutputChanged = false;
    std::shared_ptr<const RigExecSkinTopology> layoutHandle;
    std::shared_ptr<const RigExecSkinTopology> layoutCandidate;
    /// Every read RevisionStatic and the packet assembly make, sampled
    /// in the geometry prologue. For an operation the leaves assemble
    /// (`leaves.decl.assembles`), the assembler's reads and the blend
    /// gather's; for every chain revision, `defaultWeightLeaf`.
    RigExecBakedPathLeaves leaves;
    /// RevisionStatic's `inputs:defaultWeight` read, as a key of
    /// `leaves`; -1 for a derived revision, which reads none.
    int defaultWeightLeaf = -1;
    /// An External revision's payload leaf: what the plugin's
    /// assembleExternal answered in the geometry prologue, on the owning
    /// thread. Volatile: the plugin's reads are opaque, so it is
    /// assembled on every run that reaches the prologue, and
    /// `externalPayloadChanged` says only whether it differs
    /// (operator==) from the last one; RevisionStatic still decides by
    /// comparing the whole packet. Held only where the leaves assemble
    /// the revision (`leaves.decl.assembles`): a plugin revision bound
    /// to a value the region computes (a declared phase, a weight
    /// object, a transform, a carry, influences, driver frames, a
    /// constraint delta or blend channels) calls the plugin from
    /// RevisionStatic through the stage assembler.
    RigExecExternalPayload externalPayload;
    bool externalPayloadChanged = false;
    /// A windowed external revision's entry in `windowedOps`, or -1;
    /// the run its chunk body hands the kernel; and the compiled rig
    /// build its solve keeps, owned by that body (a clone shares it).
    int window = -1;
    RigExecWindowRun windowRun;
    std::shared_ptr<const void> windowCache;
    /// The live program solves this windowed revision on a device at
    /// the publication boundary (RigExecBakedSelectWindowDevices): its
    /// chunk body captures and leaves the ticks it would solve over in
    /// `deviceTicks`, and `devicePending` until the boundary solves.
    bool windowDevice = false;
    bool devicePending = false;
    std::vector<RigExecWindowPacketPtr> deviceTicks;
    /// The chain's resultVersion the boundary last published.
    uint64_t deviceResultVersion = ~uint64_t(0);
    /// Range-pipelined (Build state): `chunks` are the chain's partition
    /// (chunked false, keys empty); range step k writes range k of
    /// `output`, which always holds this revision's full version, so
    /// `currentSource` is the revision itself; the fuse is a join that
    /// publishes RevisionDone from the range versions without reading
    /// points.
    bool rangeRole = false;
    /// A Range matrix (Build state): op nodes compute it, a MoverTransform
    /// its transform and a MatrixDeform range chain its groups, so it has no
    /// fold, static or chunk steps. Its weightBlend is pinned to
    /// opMatrixRadial, or read each frame when opMatrixRadialLeaf.
    bool opMatrix = false;
    /// A Range smooth (Build state): a Smooth range chain member computes
    /// its groups over the pinned topology read at Build, with no static or
    /// chunk steps.
    bool opSmooth = false;
    VtIntArray opSmoothCounts, opSmoothIndices;
    bool OpNodes() const
    {
        return opMatrix || opSmooth;
    }
    bool opMatrixRadial = false;
    bool opMatrixRadialLeaf = false;
    /// Filled by RevisionStatic (RigExecPrepareRevisionRanges), read by
    /// the range steps. A memo: never shadowed, restored or compared.
    RigExecRevisionRangeInputs rangeInputs;
    /// The range versions the join last published, one per chunk.
    std::vector<uint64_t> joinSeen;
    /// Ranges whose kernel refused after an Applies acceptance this run (an
    /// invariant violation; they passed through). Written by the join.
    uint32_t rangeRefusals = 0;
    /// A cycle set aside one of its ranges or its join (compile state,
    /// set before each region by the set-aside reset): the revision
    /// passes the base through like an excluded fuse, `currentSource`
    /// -1, and its join publishes that version.
    bool rangeSetAside = false;
    RigExecBakedRevisionRole role = RigExecBakedRevisionRole::Legacy;
    /// Range/Whole: per chain group (size G), whether this revision
    /// publishes it (a Range revision's gated groups; every group of a
    /// Whole one).
    std::vector<char> groupWritten;
    /// Range/Whole: per chain group, the chain index of the revision whose
    /// slot holds group g of the version entering this one; -1 the base.
    std::vector<int> enteringWriter;
    /// Range Smooth and DeltaMush (Build): per group, ascending, the
    /// other groups holding a point its results read. Its group step
    /// reads those groups of the entering version beside its own.
    std::vector<std::vector<int>> haloGroups;
    /// Range Smooth and DeltaMush: per group, its step's table of
    /// entering group points (size G, null where not read); only that
    /// step writes it.
    std::vector<std::vector<const GfVec3f *>> haloPoints;
    /// Range DeltaMush: per group, its step's plan and scratch.
    std::vector<RigExecDeltaMushGroupState<GfVec3f, GfVec3d>> mushGroups;
    /// Range/Whole: size G, indexed by group; only written groups are ever
    /// published. Range: written by the group's step; Whole: chunk g owns
    /// the computed half, the fuse the published half (pointBlocks.h).
    std::vector<RigExecGroupState<GfVec3f>> groups;
    /// The join's or fuse's last publication: the content id of each group
    /// of version r + 1. Read by the next join or fuse and by ChainStatus.
    std::vector<RigExecGroupSource> groupIds;
    /// Whole non-per-point op: the entering version gathered for its chunk.
    std::vector<GfVec3f> wholeEntering;
    /// Build: what the role and the gate rest on.
    std::vector<RigExecBakedRolePin> pins;
    /// RevisionStatic, BlendShape of a range chain: content version of
    /// parameters.blendDeltas (memcmp), keyed in place of the bytes.
    uint64_t deltasVersion = 0;
    /// The blend delta buffer the next assembly sums into: the one the
    /// packet before last held. Private to this revision's static step,
    /// never published; a clone drops it.
    std::vector<GfVec3f> deltasSpare;
    /// RevisionStatic, Lattice: parameters.restPoints holds the base at
    /// `restBaseVersion` (kept without a copy while it stands).
    bool restBaseHeld = false;
    uint64_t restBaseVersion = 0;
    /// SkinTopology op: bumped when `layoutHandle` becomes another object;
    /// the SkinTopology and RevisionPacket keys carry it, not the bytes.
    uint64_t layoutSerial = 0;
};

struct RigExecBakedGeomChain {
    SdfPath target;
    UsdAttributeQuery baseQuery;
    std::vector<RigExecBakedGeomRevision> revisions;
    VtVec3fArray sampledBase;
    bool sampledHaveBase = false;
    VtVec3fArray lastBase;
    VtVec3fArray result;
    /// ChainInput, ChainBase and ChainPoints key their points by these
    /// content versions, each bumped exactly when the array's bytes
    /// differ from the ones last published. `publishedInput` is the
    /// sampled base ChainInput last published, held by handle.
    VtVec3fArray publishedInput;
    uint64_t inputVersion = 0, baseVersion = 0, resultVersion = 0;
    /// The other half of the published double buffer. Publication
    /// alternates between `result` and this one, so the array a consumer
    /// may still hold from last frame is never the one being written --
    /// which is the COW aliasing rule VtArray leaves to its callers.
    VtVec3fArray spare;
    bool haveResult = false;
    /// The base attribute read at this time, by the prologue. False
    /// skips every step of the chain, including its counters.
    bool haveBase = false;
    /// Whether the chain's points moved before its first revision, which
    /// is where the sticky chain-dirty bit starts.
    bool baseDirty = false;
    /// This chain's schedule has to be built and that has not been
    /// reported yet. The dynamic path rebuilds a chain's schedule when
    /// the identity sequence of its revisions changed, and not for a
    /// packet that merely holds different numbers.
    bool scheduleDirty = true;
    /// Range-pipelined (Build state): G + 1 group bounds, the last N0;
    /// empty for a chain that is not.
    std::vector<int> groupBounds;
    /// The count the groups were cut from (a pin: another count bails).
    size_t groupPointCount = 0;
    /// The base as groups (ChainInputs, the one writer): refs into
    /// `baseOwner`, each version bumped exactly when its bytes move.
    std::shared_ptr<const VtVec3fArray> baseOwner;
    std::vector<RigExecGroupState<GfVec3f>> baseGroups;
    /// ChainStatus: the group ids `result` and `spare` were gathered from.
    std::vector<RigExecGroupSource> resultIds, spareIds;
    // Derived maintenance reads this chain's FINAL points.
    struct Derived {
        SdfPath target;
        /// `target` spelled at Build, for the body's diagnostic.
        std::string targetText;
        UsdAttributeQuery baseQuery;
        RigExecBakedGeomRevision revision;
        VtVec3fArray sampledBase;
        bool sampledHaveBase = false;
        VtVec3fArray lastBase;
        VtVec3fArray result;
        /// The other half of the published double buffer; see the
        /// chain's.
        VtVec3fArray spare;
        /// DerivedOut's content version of `result`, as the chain's.
        uint64_t resultVersion = 0;
        bool haveResult = false;
        /// As the chain's, read by the prologue.
        bool haveBase = false;
        bool baseDirty = false;
        /// A surface projector target (RigExecIsDerivedMatrixOp): no
        /// authored base, and a MATRIX published in place of a vec3f
        /// array -- when this run measured one or its ray missed.
        bool matrixTarget = false;
        GfMatrix4d matrix{1.0};
        bool haveMatrix = false;
    };
    std::vector<Derived> derived;
    /// A GPU chain (RigExecBakedSelectGpuChains): ChainStatus translates its
    /// packets into `gpuFrame` and RigExecBakedResolveGpuChains deforms
    /// it at the publication boundary, then runs its derived steps.
    bool gpu = false;
    /// ChainStatus ran this generation; `gpuTranslated`: and translated.
    bool gpuPending = false;
    bool gpuTranslated = false;
    RigExecGpuDeformTranslation gpuTranslation;
    RigExecGpuDeformFrame gpuFrame;
    /// What `result` was deformed from, so an unchanged frame keeps it.
    std::vector<float> gpuLastFrame;
    uint64_t gpuLastSerial = 0;
    uint64_t gpuLastBaseVersion = ~uint64_t(0);
    std::vector<GfVec3f> gpuScratch;
    /// Names this chain's device buffers; a copy (a frozen clone's
    /// chain) gets its own, and destruction frees them.
    RigExecGpuDeviceKey gpuDeviceKey;
    /// The Derived steps that read this chain, in program order.
    std::vector<int> gpuDerivedSteps;
    /// For a chain whose last revision solves on a device: its Derived
    /// steps, which the boundary runs again on the solved points.
    std::vector<int> windowDeviceDerivedSteps;
    /// The ops whose values the boundary writes outside the graph (the
    /// chain's ChainStatus and its Derived steps), re-keyed there.
    std::vector<uint32_t> gpuKeyOps;
    /// Shader mode: what ChainStatus publishes in place of the points,
    /// the version its frame carries, and the extent sample with the
    /// layout and base it was chosen for.
    std::shared_ptr<const RigExecGpuDeformPublication> gpuPublication;
    uint64_t gpuFrameVersion = 0;
    std::vector<int32_t> gpuExtentSample;
    uint64_t gpuExtentSerial = 0;
    uint64_t gpuExtentBaseVersion = ~uint64_t(0);
};

// One entry per weight object the compiled rig reaches, in DEPENDENCY ORDER
// (post-order over rigExec:baseWeight and rigExec:inputWeights, children
// before parents), so one forward pass per frame builds every packet and
// a composed object finds its inputs already built.
// The sharing matters as much as the order: exec's
// Relationship().TargetedObjects<RigExecWeightPacket>() accessor gives
// one packet per weight object per generation no matter how many movers
// bind it, so the table is keyed by the object, not by the consumer.
struct RigExecBakedWeightObject {
    SdfPath path;
    TfToken type;
    TfToken representation, rangePolicy;
    bool envelopeOnly = false;
    RigExecWeightFieldFacts fieldFacts;
    RigExecBakedPathLeaves fieldLeaves;
    std::vector<SdfPath> fieldFrozenKeys;
    /// fieldLeaves index of a volume's inputs:falloffSamples, else -1.
    int fieldFalloffSamplesLeaf = -1;
    int fieldKind = -1, fieldRepresentation = -1, fieldCombineMode = -1;
    int fieldAxis = -1;
    bool fieldAxisAttribute = false, fieldBoundsAttribute = false;
    bool fieldClamp = false, fieldBounded = false, fieldUnbounded = false;
    std::string fieldName, fieldAxisName, fieldBoundsName;
    // RigExecStaticWeight: every field is uniform, so all three fold --
    // but they are still REGISTERED, so a drag on a painted weight can
    // be placed.
    std::vector<float> values;
    std::vector<int> indices;
    RigExecBakedInput<float> defaultWeight;
    /// rigExec:baseWeight, as an index into `weightObjects`, or -1.
    int base = -1;
    /// rigExec:inputWeights in AUTHORED order (subtract and overlay are
    /// order dependent), as indices into `weightObjects`.
    std::vector<int> inputs;
    // RigExecDynamicWeight.
    RigExecBakedInput<float> driver, scale, bias;
    // Combine.
    TfToken combineMode;
    RigExecBakedInput<float> strength, invert;
    /// The combine's own rigExec:weightTarget, read for its SIZE alone.
    std::vector<UsdAttribute> combineTargetPoints;
    /// Roughly how many elements this object's packet carries, for the
    /// cost model alone. Measured once at Build off the same arrays the
    /// chain point counts are measured from; a packet whose field turns
    /// out to be a different size costs the schedule a bin, never an
    /// answer.
    size_t costElements = 1;

    /// The provider slot the volume is posed into, and therefore the
    /// BASE frame its field is placed against -- base and not final,
    /// because the evaluator overrides every seeded provider's
    /// computePointFrame with its base frame before the authoritative
    /// snapshot, and a mover's packet is what that snapshot carries.
    int providerSlot = -1;
    RigExecBakedInput<float> falloffMin, falloffMax;
    RigExecBakedInput<float> scaleX, scaleY, scaleZ;
    RigExecBakedInput<float> scaleXPos, scaleYPos, scaleZPos, scaleXNeg,
        scaleYNeg, scaleZNeg;
    RigExecBakedInput<float> extentU, extentV;
    TfToken planeAxis, planeBounds;
    /// The points-bearing relationships, as the attributes their
    /// AUTHORED targets name, in authored order.
    ///
    /// Authored and not canonicalized: exec reaches these through
    /// Relationship().TargetedObjects<GfVec3f>(computeValue), which
    /// computes a value on each targeted OBJECT, so a target naming a
    /// prim rather than its .points contributes nothing there however
    /// readily the CPU oracle infers one. A mover copies exec, so this
    /// list holds exactly the properties exec would have reached. No
    /// shipped rig authors the prim form -- every weightTarget in the
    /// tree and in the fixtures names `.points` -- so the two readings
    /// agree, and where they would not, this is the
    /// one that is a mover's answer.
    std::vector<UsdAttribute> targetPoints, samplePoints, curvePoints;
    /// The compiled rig's resampled falloff remap, copied from falloffLuts.
    std::vector<float> falloffCurve;
    /// pointLeaves index of a volume's inputs:falloffSamples, else -1.
    /// Packets read this leaf; the field reads fieldFalloffSamplesLeaf,
    /// because the property pass samples field leaves before the
    /// geometry prologue samples point leaves.
    int falloffSamplesLeaf = -1;
    std::array<VtVec3fArray, 3> packetPointViews;
    std::array<std::vector<GfVec3f>, 3> packetPointScratch;
    /// The authored falloff samples a packet's lookup last lent.
    VtFloatArray packetFalloffSamples;
    /// The point gathers' reads, one key per attribute of
    /// `targetPoints`, `samplePoints`, `curvePoints` and
    /// `combineTargetPoints` in that order, each list starting at its
    /// index below.
    RigExecBakedPathLeaves pointLeaves;
    /// Per pointLeaves key, the frozen sampler's synthetic sample key.
    std::vector<SdfPath> pointFrozenKeys;
    size_t targetLeaves = 0, sampleLeaves = 0, curveLeaves = 0,
           combineLeaves = 0;
};

/// Where a program's runs stand. A run that completes moves a Cold program
/// to Ran, or to RanUnadmitted when its stage frames were not admitted; a
/// failed run makes it Cold. Retired (ReleaseStageReferences) is final and
/// refuses every later run.
enum class RigExecBakedRunState : uint8_t { Cold, RanUnadmitted, Ran, Retired };

/// Op nodes the baked schedule runs as OpNode, OpChainHead and
/// OpChainRange steps (eval::OpStepLayout): the layout, the inputs fixed at
/// Build and the named outputs. Immutable once built; clones share it.
struct RigExecBakedOpProgram {
    eval::OpStepLayout layout;
    std::vector<std::pair<std::string, RigExecOutputId>> outputs;
    /// Per node: its settings as its payload encode writes them, and
    /// whether they could be written.
    std::vector<std::vector<uint8_t>> payloads;
    std::vector<char> savable;
    /// Per chain, member and vertex group: the groups the member's
    /// neighbourhood input reads there; empty for a member without one.
    std::vector<std::vector<std::vector<std::vector<uint32_t>>>> halo;
    /// Per node and per range chain: the revision id whose fold, static and
    /// chunk steps they stand in for, else -1.
    std::vector<int> nodeRevision, chainRevision;
};

/// Op nodes a test adds for a rig (RigExecBakedProgramTesting::
/// SetOpInjector) to the description Build is writing, after the property
/// chains' nodes: their inputs from the rig or fixed, and named outputs.
/// Which of the program's families an op node computes.
enum class RigExecBakedOpFamily : uint8_t {
    Property,
    VolumePlacement,
    WeightPacket,
    /// One weight object of a weight field: object the field, part the
    /// node's place in the field's nodeObjects.
    WeightField,
    /// A provider's final (part 1) or base (part 0) matrix: object the slot.
    ProviderMatrix,
    /// A frame record's matrix: object the frameRecords index.
    FrameMatrix,
    /// The write-back of commit `object` (a walk step) through PoseCommit.
    PoseCommit,
    /// A connected provider's refresh: object the providerRefreshes index.
    ProviderRefresh,
    /// A provider program's arithmetic or affine operation: object its
    /// index in the provider program's ops.
    ProviderOp,
    /// The mover of constraint commit `object`: part -1 its solve, 0 the
    /// delta it measures against its geometry target's transform.
    Constraint,
};

/// Where an op node's step comes from: its family, that family's object and
/// part, and the step kind whose identity (descriptor category and
/// subindex, cost, exec-check row name, trace domain) the step keeps.
struct RigExecBakedOpOrigin {
    RigExecBakedOpFamily family = RigExecBakedOpFamily::Property;
    RigExecBakedStepKind kind = RigExecBakedStepKind::OpNode;
    int32_t object = -1;
    int32_t part = -1;
};

/// The kind, object and part a step is identified, costed and reported by.
struct RigExecBakedStepIdentity {
    RigExecBakedStepKind kind = RigExecBakedStepKind::OpNode;
    int object = -1;
    int part = -1;
};

struct RigExecBakedOpInjection {
    RigExecRigDescBuilder *builder = nullptr;
    std::vector<eval::OpHostInput> hostInputs;
    /// An input fixed at Build: the node and port it feeds and its value.
    struct Constant {
        uint32_t node = 0;
        uint32_t port = 0;
        VtValue value;
    };
    std::vector<Constant> constants;
    /// Lookups the hostInputs' Lookup entries index from 0, and values
    /// stored after their nodes run.
    std::vector<eval::OpLookup> lookups;
    std::vector<eval::OpHostOutput> hostOutputs;
    std::vector<std::pair<std::string, RigExecOutputId>> outputs;
};

struct RigExecBakedProgramImpl {
    using ConstraintArrays = RigExecBakedConstraintArray;
    using NativeXformSource = RigExecBakedNativeXformSource;
    using Ladder = RigExecBakedLadder;
    using PoseInterpolator = RigExecBakedPoseInterpolator;
    using AvarBinding = RigExecBakedAvarBinding;
    using Solver = RigExecBakedSolver;
    using Constraint = RigExecBakedConstraint;
    using WindowedOp = RigExecBakedWindowedOp;
    using SpaceSwitch = RigExecBakedSpaceSwitch;
    using GeomChunk = RigExecBakedGeomChunk;
    using GeomBlendChannel = RigExecBakedGeomBlendChannel;
    using GeomRevision = RigExecBakedGeomRevision;
    using GeomChain = RigExecBakedGeomChain;
    using WeightObject = RigExecBakedWeightObject;
    RigExecRigEvaluator *evaluator = nullptr;
    UsdStageRefPtr stage;
    /// The rig's asset root: the parent of the RigExecRoot, which is the
    /// space every control frame the rig publishes is expressed in, and the
    /// prim a plain Xformable's transform is measured relative to. The
    /// xform-derived provider seeds are measured relative to this prim.
    SdfPath assetRootPath;

    // Captured once, at Build, inside the one translation unit the evaluator
    // declares a friend. Pointers rather than copies wherever the value can
    // move between frames -- a drag rewrites the override list, a cache fills
    // in, the profiler is switched on mid-session -- so the program reads what
    // the evaluator holds NOW and can never answer from a stale copy.
    RigExecResolvedInputs *resolvedInputs = nullptr;
    RigExecProfiler *profiler = nullptr;
    bool recordOpTimings = false;
    const std::vector<RigExecValueOverride> *interactiveOverrides = nullptr;
    /// Joint -> the ordered stack of (solver, element) that write it, which
    /// is where the epilogue recovers the ELEMENT a writer that published
    /// nothing was bound to. rigExec:joints is an ordered write, so a joint
    /// may carry several entries; the last one supplies its base frame.
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
        *jointSolverBinding = nullptr;
    // Compile-owned normalization. No evaluator table is edited and no
    // original SCC body can become runnable when its writer is removed.
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
        ownedJointSolverBinding;
    RigExecOpExclusionProof cycleExclusionProof;
    std::map<SdfPath, std::string> cycleSkipReasons;
    /// The observational guide request, which exists only while a consumer
    /// asked for one, and the runtime toggle beside it. Read per frame rather
    /// than folded: either can move without the compiled rig moving.
    const bool *solverGuidesEnabled = nullptr;
    /// Whether the compiled rig has any property chain at all. A chain appearing or
    /// disappearing is a structural edit, which recompiles and rebuilds this
    /// program, so this one is a captured constant and not a pointer.
    bool hasPropertyChains = false;

    // The slot table is the ordered UNION of the two provider families: the
    // RigExec providers exec seeds (_nativeProviderPaths) and the plain
    // Xformables a constraint targets (_xformDerivedProviders). Both are keyed
    // by SdfPath, whose order IS namespace DFS pre-order, so a provider's
    // parent always has a lower slot and one forward pass composes the whole
    // hierarchy.
    std::vector<SdfPath> paths;
    /// `paths` spelled once at Build on the owning thread, one text per slot.
    /// Step bodies and worker-side keys read this text: SdfPath::GetString
    /// interns under Sdf's table locks. Immutable, so clones share it.
    std::shared_ptr<const std::vector<std::string>> pathTexts;
    std::map<SdfPath, int> index;
    std::vector<RigExecBakedSlotKind> slotKind;
    /// Captured prim activity, distinct from individual attribute availability.
    /// Production capture stores one immutable compiled rig bit per provider slot.
    std::vector<char> providerActive;
    /// Nearest COMPOSE ancestor -- the nearest FirstFramePose slot above this one.
    /// The compose ladder inherits from it, because exec's NamespaceAncestor
    /// resolves only RigExec provider types and skips anything else.
    std::vector<int> parent;
    /// Nearest ancestor slot of ANY kind: the pure namespace climb that finds
    /// the closest revised ancestor a propagated descendant rides.
    std::vector<int> propParent;

    // A plain Xformable a constraint targets has no rest chain and no avars:
    // its pose is whatever the stage says its transform is, measured relative
    // to the asset root. The run reads that in its PROLOGUE -- it is a stage
    // read, which no step may make -- and leaves the frame in the slot's
    // FIRST version, where the compose would have left one.
    /// The XformDerived slots, ascending, and the prim each one reads.
    std::vector<int> xformSlots;
    RigExecRequiredStageFramesAdmission requiredStageFramesAdmission;
    std::vector<UsdPrim> xformPrimsBySlot;
    /// The asset root every relative transform is measured against, which is
    /// the rig prim's parent (rigEvaluator.cpp's `assetRoot`).
    UsdPrim assetRoot;
    /// Per entry of `xformSlots`: the relative transform this run read, and
    /// the one the run before it read. The seed is a SOURCE -- it
    /// reads outside the program and always runs -- so what makes its
    /// readers dirty is the two compared by VALUE, never "the time moved".
    /// It is also what `providerBaseXforms` publishes.
    std::vector<GfMatrix4d> xformBase, lastXformBase;

    std::vector<ConstraintArrays> constraintArrays;
    /// Channels the prologue read since Build. Test observable.
    uint64_t constraintArrayReads = 0;
    /// The frozen plain sampler's memo of the channels whose read cannot
    /// move with the time, per constraintArrays entry: the sample one read
    /// added (whether it added one, and its value, hasValue and blocked
    /// bits), kept while the program stamp, the evaluator's stage edit
    /// serial and the Default-ness of the time it was read under stand, as
    /// headLeafConstants is. UI thread only, written through a const
    /// program by the sampler; never cloned.
    struct FrozenArraySample {
        uint8_t variance = ConstraintArrays::kVarianceUnknown;
        bool present = false, hasValue = false, blocked = false;
        VtValue value;
    };
    mutable std::vector<std::array<FrozenArraySample, 4>> frozenArraySamples;
    mutable uint64_t frozenArrayStamp = 0;
    mutable uint64_t frozenArraySerial = 0;
    mutable bool frozenArrayDefault = false;
    mutable bool frozenArrayBuilt = false;

    std::vector<NativeXformSource> nativeSources;
    /// Per entry: the frame the prologue read, whether it read at all, and
    /// the pair the run before left -- the value comparison that dirties the
    /// constraint steps reading it, because a source is never dirtied by
    /// "the time moved".
    std::vector<RigExecPointFrame> nativeFrames, lastNativeFrames;
    std::vector<char> nativeFrameOk, lastNativeFrameOk;

    /// One per slot; an xform-derived slot's is unused and stays default.
    std::vector<Ladder> ladders;
    std::map<SdfPath, std::set<SdfPath>> poseDependencyPaths;
    std::shared_ptr<const RigExecSceneDescriptors>
        sceneDescriptors; ///< detached compile facts
    RigExecProviderProgram providerProgram;
    /// Build publishes the provider graph once; frozen snapshots and worker
    /// lanes share these immutable descriptors, never their typed values.
    std::shared_ptr<const RigExecProviderProgram> compiledProviderProgram;
    const RigExecProviderProgram &GetProviderProgram() const
    {
        return compiledProviderProgram ? *compiledProviderProgram
                                       : providerProgram;
    }
    RigExecTypedValueStore providerValues{0};
    /// Per provider op: the authority and the unavailable input its op node
    /// stored, read when it publishes its result in the same step.
    std::vector<std::pair<uint8_t, int32_t>> providerOpPending;
    /// Per provider op: the op node origin it runs as, or -1 for an
    /// Attribute op, which runs as a SpaceExpression step.
    std::vector<int> providerOpOrigin;
    RigExecBakedPathLeaves providerLeaves;
    std::vector<uint8_t> providerLeafBlocked;
    /// Persistent invalidation counts; sampling may consume mustSample before
    /// the frozen sampler refreshes its independently held raw inputs.
    std::vector<uint64_t> providerLeafEditSerial;
    /// Built by RigExecBakedCompileOpGraph; null when the provider leaves
    /// are not one dense run of values, and every run re-keys them all.
    std::shared_ptr<const RigExecBakedSpaceLeafIndex> spaceLeafIndex;
    /// Per provider leaf, the kRigExecSpaceLeaf* reasons its next
    /// publication re-keys it; sized with spaceLeafIndex. The sampler sets
    /// Sampled and publication rewrites the byte, so it outlives a sample
    /// that no run published.
    std::vector<uint8_t> spaceLeafRekey;
    /// How many provider leaves the last publication re-keyed.
    size_t spaceLeafKeys = 0;
    std::vector<RigExecValueId> providerLeafValues;
    std::vector<int> providerLeafChains;
    std::vector<int> providerRoutedReads;
    std::vector<SdfPath> providerFrozenKeys;
    std::vector<int> providerExternalSlots;
    struct ProviderFrameInput {
        RigExecValueId value = RigExecNoProviderValue;
        int slot = -1;
        bool base = false;
        uint32_t version = 0;
        SdfPath reader;
    };
    std::vector<ProviderFrameInput> providerFrameInputs;
    std::vector<std::vector<int>> poseProviderInputs;
    std::vector<char> connectedPoseProviders;
    std::vector<int> providerParentRawLeaves;
    /// RIGEXEC_VERIFY_PROVIDER_PRUNE: holders the compacted program fails
    /// (a surviving op that is not a root and whose output nothing reads, an
    /// id out of range, a sampled row that is not a SpaceLeaf read). Zero
    /// when the knob is off.
    size_t providerPruneViolations = 0;
    /// Provider steps the last Build removed (0 with RIGEXEC_PROVIDER_PRUNE=0)
    /// and the diagnostic roots it kept besides the routed inputs (which are
    /// always kept): structurally invalid ops and provider cycle members.
    size_t providerStepsPruned = 0, providerPruneRoots = 0;
    std::vector<RigExecValueId> providerRefreshTemplates;
    std::vector<size_t> providerRefreshTemplateOps;
    struct ProviderRefresh {
        struct Carry {
            int slot = -1;
            uint32_t baseRead = 0, finRead = 0, baseWrite = 0, finWrite = 0;
            std::vector<int> blockingSlots;
        };
        std::string key;
        SdfPath reader;
        int slot = -1;
        size_t checkpoint = 0;
        RigExecValueId baseValue = RigExecNoProviderValue,
                       currentValue = RigExecNoProviderValue;
        uint32_t baseRead = 0, finRead = 0, baseWrite = 0, finWrite = 0;
        std::vector<Carry> carries;
        std::vector<std::pair<uint32_t, uint32_t>> priorConstraints;
    };
    std::vector<ProviderRefresh> providerRefreshes;
    std::vector<std::vector<uint32_t>> providerRefreshBefore;
    std::string providerRefreshError;
    std::vector<int> interveningSlots;
    std::vector<SdfPath> interveningAnchors;
    /// True when any channel of any ladder is read per frame. False is the
    /// ordinary rig, and it is what keeps the recompute off the frame path.
    bool ladderVarying = false;
    /// Every override index a ladder channel registered, sorted and unique.
    /// Consulted only while a drag stands, to decide whether that drag is
    /// one of THESE inputs.
    std::vector<int> ladderOverrides;
    /// Per slot, whether the REST CHAIN reaching it can move within the
    /// compiled rig: its own rest channels, or any ancestor's. A solver measures
    /// its description from these, so it is the question a solver asks.
    std::vector<char> restChainVaries;

    /// A non-identity AUTHORED posed:space, per slot. Exec returns its frame
    /// directly and reads neither the avars nor the parent, so the compose
    /// takes the same branch: the flag is the "authored" test
    /// computations.cpp makes, re-taken on any frame the ladder is
    /// recomputed on, because an animated posed:space can cross identity.
    std::vector<char> posedAuthored;
    std::vector<GfMatrix4d> posedAuthoredM;
    std::vector<GfMatrix4d> restM; // asset-space rest
    std::vector<std::array<GfVec3d, 4>> restPts;
    std::vector<RigExecPointFrame> restFrames;
    std::vector<GfMatrix4d> selfD; // default:space
    std::vector<GfMatrix4d> posedD, parentSpaceM;
    std::vector<char> parentSpaceAuthored;
    std::vector<GfMatrix4d> lastPosedD, lastParentSpaceM;
    std::vector<char> lastParentSpaceAuthored;
    std::vector<unsigned char> lastRotationSign;
    std::vector<GfMatrix4d> parentDinv; // parent default^-1
    std::vector<TfToken> rotOrder;
    /// The frame round trips exec performs between its ladder computations,
    /// which a deep chain drifts without. Per slot, in slot order, because
    /// a child reads its parent's.
    std::vector<GfMatrix4d> restRoundTrip, defaultRoundTrip;
    /// What the last compose of each slot left, for the move comparison
    /// that sets `restChanged` and `ladderChanged`. Seeded by Build.
    std::vector<GfMatrix4d> lastRestM, lastSelfD, lastParentDinv,
        lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<TfToken> lastRotOrder;
    /// The rest tier's outputs that moved this run: per slot, and as the
    /// slots in the order they moved, which is what the closure seeds from
    /// and what the next tier run clears.
    std::vector<char> restChanged, ladderChanged;
    std::vector<int> restMoved, ladderMoved;
    /// The rotation order a rest or default offset composes in, and the
    /// order an empty `avars:rotationOrder` stands for. Built at Build, so
    /// no compose builds a token from text.
    TfToken xyzToken;
    /// Per provider slot: the scale avars are read and DISCARDED.
    ///
    /// A volume weight is a RigExecXformable whose point frame is composed
    /// with readScaleAvars = false, because a volume's shape is
    /// inputs:scaleX/Y/Z's alone and a transform scale left in the placement
    /// would deform the field without deforming the rigid guide a viewer
    /// draws. Exec expresses that by never binding avars:sx/sy/sz at all --
    /// so the discard has to happen at COMPOSE time and not by zeroing the
    /// captured constants, which an override or an animated channel would
    /// walk straight past.
    std::vector<char> noScaleAvars;

    /// Per slot, avars:rotationSign packed by RigExecRotationSignMask. A
    /// mirrored limb declares it so the same avar value turns both sides the
    /// same way; it multiplies the avar at compose time, never the frame,
    /// so a slot's rest pose is untouched whatever it holds.
    std::vector<unsigned char> rotationSign;

    std::vector<PoseInterpolator> poseInterpolators;
    /// Every weight property the interpolators publish, its value this run,
    /// and property path -> slot for the blend channel that reads one.
    std::vector<SdfPath> poseWeightPaths;
    std::vector<float> poseWeights;
    std::map<SdfPath, int> poseWeightIndex;

    std::vector<double> avarConstants;     // providers * 11
    std::vector<AvarBinding> avarBindings; // the varying ones only
    /// The rest, kept only so a drag can reach one. Walked per frame while an
    /// interactive override stands and never otherwise.
    std::vector<AvarBinding> avarConstantBindings;
    /// Avar property path -> its index in avarConstantBindings, for the
    /// constants a value edit can PATCH rather than rebuild: resolved from
    /// the attribute itself (a walk of one, no connection upstream), so the
    /// value a notice names is exactly the value the slot holds. See
    /// RigExecBakedProgram::ApplyAvarValueEdits.
    std::map<SdfPath, size_t> patchableAvars;
    /// Per provider slot p (flat avar slot / 11), its bindings in each list:
    /// avarBindings[avarBindingBegin[p], avarBindingBegin[p + 1]) and the
    /// same over avarConstantBindings with avarConstantBindingBegin. Both
    /// lists are appended in ascending flat slot order, so a provider's
    /// bindings are one run in list order. Built with the lists
    /// (RigExecBakedIndexAvarBindings) and never written by a run; read
    /// through RigExecBakedAvarBindingRange.
    std::vector<uint32_t> avarBindingBegin;
    std::vector<uint32_t> avarConstantBindingBegin;
    /// Indices into avarConstantBindings of the patchable avars an edit has
    /// since animated (a spline key on a released drag). Read per frame the
    /// long way, like a varying binding, until an edit makes them constant
    /// again. Kept apart from avarBindings so the patch map's indices hold.
    std::vector<size_t> promotedAvars;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

    std::vector<double> avars;
    std::vector<GfMatrix4d> posedM;
    /// The pose frames, in SSA form.
    ///
    /// Slot i's FIRST version is entry i -- what the compose writes, and for
    /// an xform-derived slot what the seed leaves -- and every later writer
    /// of that slot gets storage of its own beyond the slot table: entry
    /// [N, ...) is one commit's answer for one slot, written by that commit
    /// and by nothing else. A reader binds at Build to the exact entry
    /// holding the version live at its own point in the program, so a read
    /// is one indirection and no version is ever overwritten by a later one.
    ///
    /// That is what makes "a skipped step keeps last run's values" true
    /// without a restore closure: the value a clean reader wants is still
    /// in the storage its writer left it in, however many times the SLOT
    /// was revised afterwards. A step that declares a write it does not
    /// perform carries the version it found into its own storage, so every
    /// version a reader can name is well formed whatever a run decided.
    ///
    /// Cost: Sigma|writes| frames, 96 bytes each -- a biped's walk is under
    /// 200 KB -- allocated once at Build and never resized in a run.
    std::vector<RigExecPointFrame> base, fin;
    /// Slot -> the entry holding its LAST version, which is what the
    /// matrices and the publication read. Sized N at Build.
    std::vector<uint32_t> finLast, baseLast;

    /// This frame's property-chain results, per target. The prologue fills it
    /// and the pose half publishes it; program-owned so the two halves cannot
    /// be handed different maps.
    std::map<SdfPath, VtValue> propertyResults;

    /// Every record an AtPrim transform phase can read, in walk order (by
    /// `commit`, then the commit's target or output order), and what each
    /// one's FrameMatrix step wrote
    /// this run. Kept across runs like deltaValues: a step the cone skipped
    /// left the record it would write again.
    std::vector<RigExecBakedFrameRecord> frameRecords;
    std::vector<GfMatrix4d> frameMatrix;
    std::vector<char> frameMatrixValid;

    /// The number of RigExecBakedPointsBinding ids handed out at Build.
    int pointBindingCount = 0;
    /// Test-only, off unless RigExecBakedProgramTesting::CapturePointReads
    /// turned it on: indexed by binding id, each entry written only by its
    /// binding's reader. Never cloned into a frozen program.
    bool capturePointReads = false;
    std::vector<RigExecBakedPointCapture> pointCaptures;

    // What computeMatrix publishes, over dense slots: the whole
    // AuthoritativeSnapshot request is a re-derivation of values the walk
    // already holds. Program-owned rather than a per-frame allocation,
    // because BOTH halves of a frame read them, the geometry half must see
    // exactly the matrix the pose half published, and a 267 KB zero-fill per
    // frame is work that belongs at Build.
    // One ProviderMatrix step writes each entry the program can read, and
    // no larger, so there is no on-demand fill and no per-frame reset: an
    // entry no step writes simply keeps a value nothing reads.
    std::vector<GfMatrix4d> finalMatrix, baseMatrix;
    /// Whether some step or the epilogue reads this slot's final / base
    /// matrix, which is what decides that a ProviderMatrix step exists for
    /// it. Build fills both.
    std::vector<char> needFinal, needBase;

    std::vector<Solver> solvers;
    /// The solver slots no batch runs, in dependency order. Their Solve
    /// steps sit after the whole walk, because the frames the dynamic path's
    /// guide request hands them are the FINAL ones.
    std::vector<int> guideSolvers;
    std::map<SdfPath, int> solverIndex;
    std::vector<RigExecPointFrameArray> aggregates;

    std::vector<Constraint> constraints;

    std::vector<WindowedOp> windowedOps;
    /// Canonical-op mask of a history tick: the windowed ops and every op
    /// one depends on, closed under predecessors.
    std::vector<char> historyMask;
    /// Set for a history tick: the masked graph runs, windowed bodies
    /// capture, and nothing is published.
    bool historyTick = false;
    /// Per windowed op, written only by that op's body: the packet captured
    /// at this run's tick.
    std::vector<RigExecWindowPacketPtr> windowCaptures;
    /// Per windowed op: the earlier ticks' packets, placed at the boundary
    /// before a run, and whether they differ from the ring its body last
    /// read (which seeds the op).
    std::vector<RigExecWindowRing> windowRings;
    std::vector<char> windowRingChanged;
    /// Per windowed op, advanced whenever its placed ring changes: the
    /// values its op publishes key by it, so a solve over a new history
    /// publishes a change even when this frame's inputs hold still. Per
    /// canonical op, the windowed op it is, or -1.
    std::vector<uint64_t> windowRingSerial;
    std::vector<int32_t> windowOfOp;
    /// The fewest points a device revision solves on the device for
    /// (RigExecGpuClothMinPoints), read when the devices are selected.
    size_t windowDeviceMinPoints = 0;

    struct WalkStep {
        bool solverBatch = false;
        size_t level = 0;
        int index = 0; // constraint slot
        std::vector<int> batchSolvers;
        // Descendant propagation, decided at bake: the closest changed ancestor
        // each descendant rides, with the ownership-blocking rule already
        // applied.
        std::vector<std::pair<int, int>> propagate;
    };
    std::vector<WalkStep> walkSteps;

    // Built from the tables above, once, at the end of Build. `steps` is in
    // program order, and every edge points forward in it.
    std::vector<RigExecBakedStep> steps;
    std::vector<RigExecBakedStep> excludedSteps;
    struct SpaceCheckpoint {
        std::string key;
        int anchor = -1;
        std::vector<int> recompose;
        std::vector<RigExecSpaceCheckpointInput> kernelInputs;
    };
    std::vector<SpaceCheckpoint> switchFrameContexts;
    std::vector<GfMatrix4d> switchFrames;
    struct AutoClavicle {
        struct FrameRead {
            RigExecValueId value = UINT64_MAX;
            int slot = -1;
            std::string computation;
            std::vector<int> recompose;
        };
        struct ScalarRead {
            RigExecValueId value = UINT64_MAX;
            bool isFloat = false;
            RigExecBakedInput<float> narrow;
            RigExecBakedInput<double> wide;
        };
        int slot = -1;
        RigExecBoundAutoClavicle operation;
        RigExecTypedValueStore values;
        std::vector<FrameRead> frames;
        std::vector<ScalarRead> scalars;
    };
    std::vector<AutoClavicle> autoClavicles;
    std::vector<int> autoClavicleBySlot;
    std::vector<SpaceSwitch> spaceSwitches;
    /// Per provider slot: its switch's index, or -1. Read once per slot by
    /// the compose, so the ordinary rig pays one array lookup and nothing
    /// else for a feature it does not use.
    std::vector<int> spaceSwitchBySlot;

    /// Set when the compose groups could not be put in dependency order.
    /// With every switch read bound to its version this never happens on a
    /// rig the compile accepted; Build refuses the program if it does.
    bool composeCycle = false;

    std::vector<RigExecBakedComposeGroup> composeGroups;
    /// One per walk entry, in walk order; `walkSteps[w]` and `commits[w]`
    /// describe the same commit.
    std::vector<RigExecBakedCommit> commits;
    /// (chain, revision) per dense revision id, and (chain, derived) per
    /// dense derived id. Both are assigned chain by chain, so one chain's
    /// revisions are a contiguous range and a chain-wide read is one range.
    std::vector<std::pair<int, int>> revisionIndex;
    std::vector<std::pair<int, int>> derivedIndex;
    /// First and last-plus-one revision id of each chain.
    std::vector<int> chainRevisionBegin, chainRevisionEnd;
    /// The RevisionOut domain is indexed by CHUNK, not by revision: a
    /// revision's chunks write disjoint vertex ranges of one buffer, and two
    /// steps that declared the same slot would be ordered against each other
    /// for a conflict they do not have. These are the dense chunk ids --
    /// [chunkBase, chunkBase + chunkCount) per revision, handed out revision
    /// by revision, so one chain's chunks are a contiguous range too.
    std::vector<int> revisionChunkBase, revisionChunkCount;
    std::vector<int> chainChunkBegin, chainChunkEnd;
    /// The RevisionFuse step of each revision id: the only writer of its
    /// RevisionDone and ChainDirty slots, and so the producer of point
    /// version r + 1 of its chain (RigExecBakedPointVersion).
    std::vector<int> revisionFuseStep;
    /// The layout (RigExecBakedLayoutRevision index) of every SkinTopology
    /// step, set where the op graph is compiled, so head preparation clears
    /// their changed flags without walking every step.
    std::vector<int> skinTopologyLayouts;

    /// The partition of `steps` the parallel executor runs, chosen once at
    /// Build. Its grain comes from the cost model and the machine's
    /// concurrency, never from a measurement.
    RigExecBakedClustering clustering;
    // Settings a body path needs, read from the environment once at Build so
    // that no body reads the environment or a function-local static.
    /// RIGEXEC_BAKED_CHUNK_VERTS: vertices one chunk covers before the cap.
    size_t chunkVertexTarget = RigExecGeometryParallelThreshold;
    /// RIGEXEC_GPU_DEFORM_STATS: the owning thread prints device-chain
    /// timings at the GPU boundary. Never printed on a frozen worker.
    bool gpuDeformStats = false;
    /// RIGEXEC_INPUT_REPLAY, read once at Build: Run refuses an unrecorded
    /// entry only under capture.
    bool inputReplayCapture = false;
    /// Schedule diagnostics, read once at Build by
    /// RigExecBakedReadScheduleSettings (bakedSchedule.h).
    bool scheduleParallel = false;
    bool scheduleReport = false;
    bool scheduleCalibrate = false;
    bool scheduleStepTiming = false;
    bool traceAllSteps = false;
    int calibrationFrames = 8;
    int stepTimingFrames = 8;
    /// RIGEXEC_BAKED_MAX_CHUNKS: the most chunks one revision is cut into.
    size_t chunkCap = 32;
    /// RIGEXEC_BAKED_RANGE_CHAINS (default on), read at Build: whether a chain
    /// of more than chunkVertexTarget points is range-pipelined.
    bool rangeChains = true;
    /// RIGEXEC_BAKED_GROUP_VERTS (1024) and RIGEXEC_BAKED_GROUP_CAP
    /// (RigExecBakedDefaultGroupCap), read at Build: a range chain's group
    /// target and cap (both role modes).
    size_t groupVertexTarget = 1024;
    size_t groupCap = RigExecBakedDefaultGroupCap;
    /// RIGEXEC_BAKED_GROUP_GATES (default on), read at Build.
    bool groupGates = true;
    /// The evaluator's role mode and export keep-set when Build ran.
    RigExecBakedRoleMode roleMode = RigExecBakedRoleMode::Live;
    std::set<SdfPath> exportKeep;
    /// Export: the attribute paths the pins rest on (GetExportPinnedPaths).
    std::set<SdfPath> exportPinnedPaths;
    /// A relaxed counter that a copy or an assignment starts at zero, so it
    /// keeps the program copyable (as PurityCounter does) while bodies call
    /// fetch_add on it directly.
    struct ZeroOnCopyCounter : std::atomic<uint64_t> {
        ZeroOnCopyCounter() : std::atomic<uint64_t>(0)
        {
        }
        ZeroOnCopyCounter(const ZeroOnCopyCounter &) : std::atomic<uint64_t>(0)
        {
        }
        ZeroOnCopyCounter &operator=(const ZeroOnCopyCounter &)
        {
            store(0, std::memory_order_relaxed);
            return *this;
        }
    };
    /// Gated packets that failed RigExecRevisionGateHolds while applying, and
    /// Range skins that met a stale partition, this run (a pin hole); counted
    /// by bodies, reported by the owner after the region. Relaxed: nothing is
    /// published through it.
    ZeroOnCopyCounter gateViolations;
    /// The revision id owning each RevisionOut slot (chunk id). Build state.
    std::vector<int> chunkRevision;
    /// RIGEXEC_VERIFY_RANGE_CHAINS, read at compile: after the region the
    /// owner recomputes every range-pipelined revision whole and counts
    /// disagreements with its ranges here.
    bool verifyRangeChains = false;
    size_t rangeVerifyMismatches = 0;
    /// RIGEXEC_ENABLE_SIMD, as RigExecSimdEnabled() answers it.
    bool useSimd = true;
    /// Where eligible point chains deform (RigExecGpuDeformMode), chosen at
    /// Build by RigExecBakedSelectGpuChains.
    RigExecGpuDeformMode gpuDeformMode = RigExecGpuDeformMode::Off;
    /// True while RigExecBakedResolveGpuChains runs the derived steps of
    /// GPU chains at the publication boundary.
    bool gpuBoundary = false;
    /// GeomChain::gpuDerivedSteps hold the compiled step order.
    bool gpuDerivedIndexed = false;
    /// RIGEXEC_PURITY_AUDIT: whether RunStepBody hands its bodies
    /// purityViolations to count reads under RIGEXEC_PURITY_CHECK.
    bool purityAudit = false;
    /// Body reads counted under purityAudit. Relaxed, because nothing is
    /// published through it; the test thread reads it after the run joins.
    /// A copy or move starts at zero, so it does not make the program
    /// immovable; _CloneImpl copies purityAudit and zeroes this, so a frozen
    /// clone counts its own.
    struct PurityCounter {
        std::atomic<uint64_t> count{0};
        PurityCounter() = default;
        PurityCounter(const PurityCounter &)
        {
        }
        PurityCounter &operator=(const PurityCounter &)
        {
            count.store(0, std::memory_order_relaxed);
            return *this;
        }
    };
    PurityCounter purityViolations;

    // What a run may SKIP. The sets are Build's; everything below them is the
    // last run's answer, kept so that this run's sources can be compared with
    // it by VALUE. There is no "time changed" and no "overridden" predicate
    // deciding whether a source ran: sources always run and their outputs are
    // compared, which is what makes an override on a routed prim, a released
    // drag, a rebuilt skin layout and a moved keyframe all reach the graph
    // through one test.
    RigExecBakedCones cones;
    /// The steps this run decided to run. Every step outside it keeps its
    /// slots, its diagnostics and its structural counters, whether or not
    /// its cluster runs.
    RigExecBakedClusterSet closedSteps;
    /// The clusters that hold a step of `closedSteps`: what the parallel
    /// executor dispatches and counts. The frozen serial runner and a
    /// partial cone re-run still run whole clusters of it, which is a
    /// superset of the closed steps and so the same answer.
    RigExecBakedClusterSet closed;
    /// The avar table as the last run left it, for the per-provider compare.
    std::vector<double> lastAvars;
    /// The override flags as the last run left them, so that the run AFTER a
    /// drag is released re-runs what the drag was holding.
    std::vector<char> lastOverridden;
    /// One flag per override index whose input a stage VALUE edit reached
    /// since the last run: set by RigExecBakedProgram::ApplyValueEdits,
    /// OR-ed across notices, and consumed only by the closure's override
    /// rule, beside `lastOverridden` -- the step that reads such an input
    /// re-runs once, exactly as it does for the run after a drag is lifted.
    /// Pending bits survive a generation the program did not answer (a
    /// dynamic fallback), and a frozen clone carries them, because the
    /// clone's first run is the one that owes them.
    std::vector<char> edited;
    bool anyEdited = false;
    /// How many notices ApplyValueEdits has marked inputs for, and, per
    /// override index, the count at the last one that marked it. Unlike
    /// `edited`, nothing consumes these: a frozen snapshot records the count
    /// it was taken or patched at, and a later patch marks every index
    /// edited since -- including edits the live program has already run
    /// and cleared, which the snapshot's own state has never seen.
    uint64_t valueEditSerial = 0;
    std::vector<uint64_t> editSerial;
    /// Whether each chain's base read at all last run.
    std::vector<char> lastHaveBase;
    UsdTimeCode lastTime = UsdTimeCode::Default();
    RigExecBakedRunState runState = RigExecBakedRunState::Cold;
    /// Bumped by the evaluator for a notice that does NOT invalidate the
    /// program -- a value edit on an input the frame path re-reads, which is
    /// IsInvalidatedBy's documented gap. One run of everything answers it.
    /// Set/ClearInteractiveOverrides do NOT bump it: an override is a source
    /// value like any other and is compared like one.
    uint64_t programStamp = 0;
    uint64_t lastProgramStamp = 0;
    /// Whether the closure this run computed trusts nothing it holds (a
    /// forced run, or a moved program stamp). Set by
    /// RigExecBakedComputeClosure before any region step runs and read by
    /// the Solve step's rest-refresh gate: such a run can follow a
    /// generation whose rest tier moved rests and whose region never ran.
    bool closureFull = false;
    /// How many clusters the last run ran, and how many there are, for the
    /// schedule run report.
    size_t lastClosedClusters = 0;
    /// How many steps the last run's closure held, sources it seeded from
    /// included.
    size_t lastClosedSteps = 0;

    std::vector<int> jointSlots;
    std::vector<SdfPath> jointPaths;
    std::vector<int> controlSlots;
    std::vector<SdfPath> controlPaths;
    std::vector<std::pair<SdfPath, int>> solverArrays; // guide publication
    /// The order to fill the published maps in: the publication list's
    /// indices, sorted by path.
    ///
    /// A frame publishes its keys into five ordered maps. Filled in path
    /// order, each key belongs immediately past the one before it,
    /// so a hint at the map's end makes the insertion one comparison. The
    /// order is a PERMUTATION and not the list itself, because the lists
    /// arrive from the evaluator in binding order, which is not path order
    /// in general. Everything the walk ORDER is observable through (the degenerate
    /// frame diagnostics) stays in the list's own order; only the map
    /// filling moves.
    std::vector<int> jointPublishOrder;
    std::vector<int> controlPublishOrder;
    std::vector<int> solverPublishOrder;
    /// Whether the sorted orders above are STRICTLY ascending -- which they
    /// are unless a list names one path twice. A repeated path would make an
    /// emplace keep the first value where an assignment keeps the last, so
    /// such a list is published by assignment in list order -- and the sort
    /// that built the permutation is stable, so "the last" is still the last
    /// of the two in the publication list.
    bool jointPathsAscending = false;
    bool controlPathsAscending = false;
    bool solverArraysAscending = false;
    /// Under RIGEXEC_BAKED_STEP_TIMING, the sum over the frames watched of
    /// what each third of a frame cost, in microseconds, and how many frames
    /// are in the sums. Untouched -- and unread -- when the variable is off.
    double timedPrologueUs = 0;
    double timedRegionUs = 0;
    double timedEpilogueUs = 0;
    size_t timedFrames = 0;
    /// Frames the calibration and the step timing have watched on this
    /// program, counted by the Run epilogue on the owning thread.
    int calibrationFramesSeen = 0;
    int stepTimingFramesSeen = 0;
    /// The same sums for the cold runs a measurement leaves out
    /// (`coldRunExcluded`), which the table reports on a line of their own.
    double coldPrologueUs = 0, coldRegionUs = 0, coldEpilogueUs = 0;
    size_t coldFrames = 0;
    /// Whether a cold run -- the program's first, or a forced one (a
    /// requested full run or a moved program stamp), which runs nearly every
    /// op -- folds into the per-step accumulators. Set at Build: only
    /// calibration asks for it, because its fit has to cover a first frame.
    bool measureColdRuns = false;
    /// Whether the last measured run was cold and so left out of the
    /// per-step accumulators and the step timing's frame sums. Set by the
    /// executor's owner before dispatch on every measured run (never on the
    /// cone verifier's second pass) and read by the bodies and by Run.
    bool coldRunExcluded = false;
    /// Set while the cone verifier's second, whole-program pass runs --
    /// always, not only when a step timing asked, so that the flag means
    /// one thing -- and read by both executors. That pass is the
    /// instrument, not the frame: letting it into the per-step accumulators
    /// would make every step of a verified frame report two runs, and the
    /// table would describe a frame nobody asked for.
    bool measurementSuspended = false;
    /// The steps that may hold a nonzero stamp (startUs/endUs or an op
    /// stamp): the candidates of every run since the last clear whose ops
    /// stamped, listed by the owner after the join. Only a candidate takes a
    /// stamp, so every other step's stamps are zero. Indices into `steps`,
    /// whose order Build fixes before any run;
    /// RigExecBakedRunStatistics::Restore only puts back stamps such a run
    /// wrote since the last clear. RigExecBakedClearRunStamps zeroes these
    /// steps' stamps and empties the list.
    std::vector<uint32_t> stampedSteps;
    /// Clusters whose timing fields the last profiled run set (an index may
    /// repeat); the next run resets only these, and the replay records only
    /// these. Cleared by the translation.
    std::vector<uint32_t> timedClusters;

    /// What the epilogue visits instead of every step: the step lists Build
    /// fixes and the steps holding diagnostics or lines now. Indexed by
    /// RigExecBakedIndexEpilogue once the canonical order is fixed, and
    /// again wherever step output is assigned outside a body; the owner
    /// folds the ops that ran after each join (RigExecBakedFoldHeldSteps).
    struct EpilogueIndex {
        /// Per solver, 1 where a Solve step publishes it: an SCC-excluded
        /// solver has none.
        std::vector<char> aliveSolver;
        /// The RevisionStatic, ChainStatus and Derived steps, ascending:
        /// the geometry publication's.
        std::vector<uint32_t> geometrySteps;
        RigExecHeldSteps held;
        /// RIGEXEC_VERIFY_EPILOGUE_LISTS, read at Build: every block is also
        /// swept from all steps, and a difference counted and TF_VERIFYed.
        bool verify = false;
        size_t mismatches = 0;
    } epilogue;

    /// Per joint, whether this run's final frame earned a published matrix.
    /// Written by the diagnostic pass, read by the fill pass; sized at
    /// Build, so the epilogue allocates nothing.
    std::vector<char> jointMatrixPublished;

    std::vector<GeomChain> chains;

    /// What each geometry-domain constraint measured: the delta between its
    /// solved frame and its target's own authored transform, which is the
    /// matrix the Matrix revision that same mover contributes rides. Dense
    /// and indexed by `Constraint::deltaBase`, because it is written by a
    /// STEP -- a shared std::map is an allocation and a race, whatever the
    /// slots say -- and read by the InfluenceFold of the revision whose
    /// mover produced it, which is the same hand-off the dynamic walk's own
    /// constraintDeltas map performs, in the same direction.
    ///
    /// It is a SLOT and not a per-run delta: what a skipped constraint step
    /// leaves here is what it would have measured again, so it is kept
    /// across runs rather than cleared at the head of one.
    std::vector<GfMatrix4d> deltaValues;
    std::vector<char> deltaPresent;
    /// Per geometry-domain constraint, the target's own transform relative
    /// to the asset root -- a STAGE read, made by the prologue, and a source
    /// the cone compares by value. It is the AUTHORED transform even when
    /// the target is a RigExecJoint: RigExecXformable inherits Xformable and
    /// the dynamic walk measures against the stage unconditionally.
    std::vector<SdfPath> deltaBasePaths;
    std::vector<GfMatrix4d> deltaBaseMatrix, lastDeltaBaseMatrix;
    std::vector<char> deltaBaseOk, lastDeltaBaseOk;

    std::vector<WeightObject> weightObjects;
    std::vector<RigExecWeightRecord> weightProgram;
    std::vector<char> weightCycleBlocked; ///< immutable common SCC outcome
    struct WeightField {
        enum class Form : uint8_t {
            EnvelopeProperty = 0,
            EnvelopeConstraint = 1,
            Revision = 2
        };
        enum class PlacementPhase : uint8_t { Final = 0, Base = 1 };
        Form form = Form::Revision;
        PlacementPhase placementPhase = PlacementPhase::Base;
        int object = -1;
        int consumer = -1;
        int part = 0;
        std::vector<int> volumes;
        std::vector<int> availableChains;
        bool changed = false;
        std::vector<RigExecBakedReaderWalk> scalarReads;
        std::vector<int> scalarObjects, scalarMembers;
        std::vector<int> objects; ///< immutable reachable object identities
        std::vector<int> scalarReadIndex; ///< object * 19 + member, built once
        struct PointInput {
            int object = -1, leaf = -1;
            RigExecBakedPointsBinding binding;
        };
        std::vector<PointInput> pointReads;
        /// The weight objects the field's op nodes weigh, each after the
        /// objects it composes; the field's own object last.
        std::vector<int> nodeObjects;
        /// A weight object of the closure that composes itself.
        bool cyclic = false;
        std::vector<float> values;
        size_t count = 0;
        bool ok = false;
        std::string error;
    };
    RigExecCompiledGraph opGraph;
    RigExecOpWorkspace opWorkspace;
    RigExecOpExecution opExecution;
    RigExecOpAdapterState opAdapter;
    /// The op nodes the schedule runs, and this program's run of them; null
    /// without op nodes.
    std::shared_ptr<const RigExecBakedOpProgram> opProgram;
    std::shared_ptr<eval::OpStepState> opState;
    /// RIGEXEC_VERIFY_CHAIN_VERSIONS, read at compile. After a run, the
    /// owner rebuilds each published point-version key over the points'
    /// bytes (RigExecBakedChainContentKey) and checks the two told the same
    /// change; `chainContentKeys` holds the last such key per value id.
    bool verifyChainVersions = false;
    std::vector<std::string> chainContentKeys;
    size_t chainVersionMismatches = 0;
    /// RIGEXEC_VERIFY_PACKET_VERSIONS, read at compile: the same check for
    /// the RevisionPacket's array versions (RigExecBakedPacketContentKey),
    /// with `packetContentKeys` the last such key per value id.
    bool verifyPacketVersions = false;
    std::vector<std::string> packetContentKeys;
    size_t packetVersionMismatches = 0;
    /// What the source keys read and the values they were built from: a
    /// run rebuilds only the keys of ops whose entries moved.
    RigExecBakedSourceWatch sourceWatch;
    /// Watch entries compared by the last visit (a cost counter).
    size_t sourceWatchVisits = 0;
    /// RIGEXEC_VERIFY_SOURCE_KEYS, read at compile: every run also rebuilds
    /// each source key the watch kept and counts those whose bytes or
    /// exactness moved.
    bool verifySourceKeys = false;
    size_t sourceKeyMismatches = 0;
    /// How many keys of ops that read state the last run built.
    size_t sourceKeysBuilt = 0;
    std::shared_ptr<RigExecBakedExecCheckRows> execCheckRows;
    std::function<void(uint32_t)> opBeforeBody,
        opAfterBody; ///< opt-in test observation
    std::vector<WeightField> weightFields;
    std::vector<GfMatrix4d> volumePlacementBase;
    /// Discovery allocates identities on entry, including authored back-edges.
    /// The canonical graph diagnoses cycles after all producer reads exist.
    std::map<SdfPath, int> weightIndex;
    /// Per provider slot, where the volume at that slot is placed: what that
    /// slot's VolumePlacement op node stores (WeightFrames[slot]), read by
    /// the weight field and by pose.weightFrames. The weight field and every
    /// publication (live and frozen) read only the placedVolumes slots.
    /// Non-volume slots stay identity and are never read. Kept across runs
    /// like deltaValues, and carried by a retained frozen workspace: a run
    /// that skips a slot's node keeps its last placement; its inputs are
    /// unchanged.
    std::vector<GfMatrix4d> volumePlacement;
    /// Per provider slot: 1 where the dynamic walk places this volume (a key
    /// of the evaluator's _volumeWeightMatrixTaps). A subset of
    /// noScaleAvars: a volume outside the rig that only a constraint names
    /// is a provider but is not tapped, so the walk neither places nor
    /// publishes it, and neither do the weight field view and pose.weightFrames.
    std::vector<char> placedVolumes;
    /// Weight objects whose field is measured against the points AS THEY
    /// STAND at the revision that binds them, rather than the authored base
    /// (the evaluator's _currentPhaseWeights). A combine is in here when
    /// anything inside it is.
    std::set<SdfPath> currentPhaseWeights;

    /// Every volumetric weight object's baked falloff remap, by prim path.
    ///
    /// Copied out of the evaluator's own _falloffLutOverrides at Build, not
    /// recomputed: a falloff curve is structural -- it is resampled once at
    /// Compile and replayed unchanged -- and these are literally the bytes
    /// exec receives.
    std::map<SdfPath, std::vector<float>> falloffLuts;
    /// This frame's packet per weight object, as its weight packet op node
    /// stores it. Sized at Build and never resized in a run, like every other
    /// slot: a consumer holds a pointer into it for the whole region.
    std::vector<RigExecWeightPacket> weightPackets;

    // What the bake looked at, so a notice can be answered without rebuilding
    // and without guessing. Four sets, because a notice asks four different
    // questions and one set would have to answer the bluntest of them:
    //  * `rebuild` -- properties whose VALUE decided something the program
    //    holds: a folded constant, or the selection of a per-frame query.
    //    A changed-info notice on one of these rebuilds the program.
    //  * `named` -- every property the bake ASKED ABOUT, found or not. A
    //    resync is a property appearing, disappearing or being retargeted,
    //    and it also invalidates any retained query on that property, so
    //    this is a superset of `rebuild`: it includes the inputs whose
    //    values are re-read per frame and the names that resolved to
    //    nothing.
    //  * `prims` -- every prim the bake read anything from, for a resync
    //    that names a whole subtree rather than one property.
    //  * `xformPrims` -- the Xforms between an intervening-Xform candidate
    //    and its anchor. Bakeability accepted those because they compose to
    //    the identity, which is a judgement about their composed
    //    transform and not about one attribute, so any property of theirs
    //    counts.
    // A changed-info notice that misses all of them is a value edit on an
    // input the frame path re-reads, and needs nothing.
    std::set<SdfPath> rebuild;
    // The part of `rebuild` folded for its SHAPE (FoldShape): whether it is
    // keyed decides a solver mode, so its first time sample rebuilds even
    // where it is also a patchable avar.
    std::set<SdfPath> shapeFolded;
    // Every attribute on the walk of an input read the long way each frame
    // (RigExecBakedInput::resolvedAttr). A property resync there that only
    // brings a value -- the first value a layer authors for a switch or a
    // dial, and the undo that takes it away -- is a value edit, routed like
    // one (_LiveInputResync), not a change of structure.
    std::set<SdfPath> liveInputPaths;
    std::set<SdfPath> sourceBackedPaths;
    std::set<SdfPath> named;
    std::set<SdfPath> prims;
    std::set<SdfPath> xformPrims;

    // One flag per registered input; see RigExecBakedRead. Kept as a dense
    // vector so the frame path costs an index rather than a map lookup per
    // read.
    std::vector<char> overridden;
    /// Property path -> the inputs reading it, for the placeable case.
    std::map<SdfPath, std::vector<int>> overridableInputs;

    /// The sampled leaves (RigExecBakedNumberLeaves, RigExecBakedSampleLeaves).
    /// Prologue state: the sampler writes them before the region, on the
    /// owning thread, and nothing after it does.
    RigExecBakedLeafPools leaves;
    /// Leaf id -> pool and index, in _ForEachPatchableInput order.
    std::vector<RigExecBakedLeafRef> leafRefs;
    /// Override number -> the leaf of the binding holding it, or -1.
    std::vector<int> leafOfOverride;
    /// Every path on a leaf's walk -> the leaves reading it. Built from
    /// `overridableInputs`, which files every hop of a registered binding's
    /// walk under its number; Build only, read on the owning thread.
    std::map<SdfPath, std::vector<uint32_t>> leafByPath;
    /// The overrides SetOverrides placed through the routed arm (no number),
    /// by path, and the table the last sample compared against. A leaf filed
    /// under a path whose value differs between the two is re-sampled.
    std::map<SdfPath, VtValue> routedOverrides;
    std::map<SdfPath, VtValue> lastRoutedOverrides;
    /// The upstream layer (RigExecBakedProgram::SetUpstreamInputs): this
    /// run's admitted values by path, and the table the last prologue
    /// placed. Every hop read through the layer answers from `upstream`
    /// before the stage; the prologue compares the two by value (rule 8).
    std::map<SdfPath, VtValue> upstream;
    std::map<SdfPath, VtValue> lastUpstream;
    /// Per override number: an upstream value stands on a hop of its walk,
    /// so its binding reads the long way, through the layer.
    std::vector<char> upstreamOn;
    /// Per override number: a value on a hop of its walk was placed, moved
    /// or lifted this run. The closure's override seed; never a standing
    /// flag, so a value that stays costs nothing.
    std::vector<char> upstreamChanged;
    /// Something in `upstream` moved this run.
    bool upstreamMovedThisRun = false;
    /// RigExecBakedProgram::GetUpstreamAdmissible and GetUpstreamWeightFieldReads,
    /// built on first use (`upstreamSetsBuilt`).
    std::map<SdfPath, TfType> upstreamAdmissible;
    std::set<SdfPath> upstreamWeightFieldReads;
    bool upstreamSetsBuilt = false;
    /// Leaves re-read by every sample so far, Build's included. Test
    /// observable; written only by the sampler.
    uint64_t leafSamples = 0;
    /// Every path leaf (RigExecBakedPathLeaves of a chain revision, a
    /// derived target or a weight object), in program order; path leaf id
    /// `leafRefs.size() + i` is entry i, filed in `leafByPath` under every
    /// path its read can reach.
    std::vector<RigExecBakedPathLeafRef> pathLeafRefs;
    /// The chain results the path leaves last compared against, and a serial
    /// that moves whenever they differ from the run's (a path leaf whose
    /// sample saw another serial re-reads).
    std::map<SdfPath, VtValue> pathLeafChainResults;
    uint64_t pathLeafChainSerial = 0;
    /// Path leaves re-read by every sample so far. Test observable; written
    /// only by RigExecBakedSamplePathLeaves.
    uint64_t pathLeafSamples = 0;
    /// Executes that built operation keys, which read the path leaves'
    /// versions (RigExecBakedSetPathLeaf). Owner of the program only.
    uint64_t pathLeafRun = 0;
    /// Every standing interactive override's path (prim.attribute, or
    /// prim.computation), sorted and unique; RigExecBakedPrepareOverridePaths.
    std::vector<SdfPath> overridePathsSorted;
    /// RIGEXEC_PATH_LEAF_GATING (default on), RIGEXEC_VERIFY_PATH_LEAF_GATING;
    /// read at Build. Verify mode counts gated keys whose fresh read differed.
    bool pathLeafGating = true, verifyPathLeafGating = false;
    size_t pathLeafGateMismatches = 0;
    /// Incremented when the first sample pass clears the typed leaves'
    /// `changed` flags: flags and `changedList`s describe exactly the typed
    /// writes since then.
    uint64_t leafFlagCompiledRig = 0;
    /// Incremented by every call that may move an avar binding's constant
    /// or variance: RigExecProgramAvarPatch, RigExecPatchFrozenAvarConstants,
    /// and a frozen job's input patch when an avar constant's bits moved.
    uint64_t avarConstantSerial = 0;
    /// RIGEXEC_SPARSE_SAMPLING (default on), RIGEXEC_VERIFY_SPARSE_SAMPLING;
    /// read at Build. Verify mode counts inputs the sparse pass missed.
    bool sparseSampling = true, verifySparseSampling = false;
    size_t sparseSamplingMismatches = 0;
    /// Inputs whose resample predicate the binding sampler evaluated.
    uint64_t leafVisits = 0;
    RigExecBakedSparseInputs sparseInputs;
    /// Every property a chain writes: RigExecBakedBuildContext::chainTargets
    /// as Build classified the inputs against it, kept so a bake can
    /// recompute each input's walk the same way.
    std::set<SdfPath> chainTargets;
    /// Prims whose reads already route through the generation's resolved
    /// inputs (property-chain movers, geometry movers), so an override on
    /// any of their properties reaches them with nothing else to do.
    std::set<SdfPath> resolvedRoutedPrims;
    /// Every attribute (and any other target) a read through the resolved
    /// inputs can reach by following authored connections out of a
    /// `resolvedRoutedPrims` prim, hop by hop. The bake records the prims of
    /// those readers and never the sources their connections name, so a
    /// source on a prim nothing else reads would otherwise look read by
    /// nothing: the program itself is still right (the reader re-resolves
    /// and compares by value), but the frame cache would be told no read
    /// path moved and keep frames the edit changed.
    ///
    /// Filled on the notice thread, only when a notice names a property the
    /// rest of the index does not place, and answered for the program stamp
    /// it was filled under. Every change that can move a connection -- a
    /// connection or target field, a resync, layer metadata -- is refused by
    /// the value-edit routing or rebuilds the program, and a refusal bumps
    /// the stamp, so a stamp that has not moved is a connection graph that
    /// has not either. Nothing in a run reads it, and a frozen clone never
    /// routes a notice, so it is not cloned.
    mutable std::set<SdfPath> connectedSources;
    mutable uint64_t connectedSourcesStamp = 0;
    mutable bool connectedSourcesFilled = false;
    /// True while the dense avar table still holds a value written for a
    /// drag; see the input block in Run.
    bool avarsDisturbed = false;
    /// Properties folded into bake state. An override here cannot be placed
    /// without rebaking, so it forces the dynamic path instead.
    std::set<SdfPath> folded;
    bool anyOverridden = false;
    /// RigExecBakedProgram::SetPublishWeightFields.
    bool publishWeightFields = true;

    /// Property operation metadata. Build state: the chains,
    /// records, leaves, override slots and steps. Everything after
    /// the head prefix is prologue state, written on the owning thread before
    /// the region.
    std::vector<RigExecCrossDomainRead> crossDomainReads;
    std::vector<std::string> crossDomainErrors;
    std::map<SdfPath, int> crossDomainOrdinals;
    std::vector<RigExecBakedPropertyChain> propertyChains;
    std::vector<RigExecBakedPropertyRecord> propertyRecords;
    /// The PropertyVersion domain's size: every chain's versions, then the
    /// records.
    uint32_t propertyVersionCount = 0;
    /// Per op node a family of the program runs: where its step comes
    /// from. Nodes past its end are injected.
    std::vector<RigExecBakedOpOrigin> opNodeOrigins;
    /// Per op lookup: the keys or points a property read lends its op
    /// input until its next run.
    std::vector<VtArray<GfVec2f>> propertyKeyHold;
    std::vector<VtVec3fArray> propertyPointHold;
    /// Per op lookup: a ChainVersion lookup's gathered points, when the
    /// version is not one buffer.
    std::vector<std::vector<GfVec3f>> chainVersionGather;
    std::vector<RigExecBakedHeadLeaf> headLeaves;
    /// Every path a head op reads an override at, and its slot.
    std::map<SdfPath, uint32_t> headOverrideSlots;
    /// The same slots keyed as an override names its property, (prim,
    /// attribute), so placement builds no path: the frozen worker runs it
    /// and must not take the path table's lock.
    std::map<std::pair<SdfPath, TfToken>, uint32_t> headOverrideSlotsByName;
    /// Per version id: its value, and whether its chain was valid when it
    /// was written.
    std::vector<RigExecBakedPropertyValue> propertyValues;
    std::vector<char> propertyVersionValid;
    /// Per PropertyVersion id, records included: moved this run.
    std::vector<char> propertyChanged;
    /// Per chain: valid this run, and its final version as published.
    std::vector<char> chainValid;
    std::vector<VtValue> chainFinal;
    /// Per record: its value, and whether an override on one of its hops
    /// stands it aside this run.
    std::vector<VtValue> recordValues;
    std::vector<int>
        propertyRecordById; ///< immutable PropertyResult id to record index, -1 for chain versions
    std::vector<char> recordStoodAside;
    /// Independent reference publication markers, enabled only for checks.
    std::optional<RigExecOraclePublicationContext> oraclePublications;
    std::map<SdfPath, RigExecWeightReferenceContext> oracleWeightInputs;
    /// Per override slot: the interactive override standing there this run
    /// (empty when none), last run's, and whether it moved.
    std::vector<VtValue> headOverrides, lastHeadOverrides;
    std::vector<char> headOverrideMoved;
    /// What the head leaves were last sampled at, and the last tier run.
    bool headLeavesSampled = false;
    UsdTimeCode headLeafTime = UsdTimeCode::Default();
    uint64_t headLeafStamp = 0;
    bool headEverRan = false;
    uint64_t headStamp = 0;
    /// Test observables: head leaves re-read and head ops executed since
    /// Build.
    uint64_t headLeafSamples = 0;
    uint64_t headOpsRun = 0;
    /// The frozen samplers' table of the constant head leaves, kept while
    /// the state it was read under stands (RigExecHeadLeafConstants). UI
    /// thread only, written through a const program by the samplers; never
    /// cloned, because a snapshot never samples. A re-read whose every
    /// entry is bitwise the standing one keeps the standing table, so
    /// frames sampled on either side of an edit that left the constants
    /// alone share one.
    mutable std::shared_ptr<const RigExecHeadLeafConstants> headLeafConstants;
    /// The state headLeafConstants was last read under: the program stamp,
    /// the evaluator's stage edit serial and whether the time was Default.
    mutable uint64_t headLeafConstantsStamp = 0;
    mutable uint64_t headLeafConstantsSerial = 0;
    mutable bool headLeafConstantsDefault = false;
    /// Each keyed head leaf's `editSerial` when headLeafConstants last
    /// answered for it, in RigExecForEachHeadLeaf order.
    mutable std::vector<uint64_t> headLeafConstantsLeafSerials;
    /// The frozen samplers' memo of the leaf reads that cannot move with
    /// the time and of recorded digest orders (RigExecFrozenSamplerMemo),
    /// one per Default-ness of the time ([1] Default), so samples that
    /// alternate the two keep both. UI thread only and never cloned, like
    /// headLeafConstants.
    mutable std::array<std::shared_ptr<RigExecFrozenSamplerMemo>, 2>
        frozenSamplerMemo;
    /// RIGEXEC_VERIFY_FROZEN_STATIC, read at Build: the samplers re-read
    /// every memoized leaf and check it, and the vector's digest, against
    /// the unmemoized answer.
    bool verifyFrozenStatic = false;
    /// RIGEXEC_VERIFY_CLONE_KEYS, read by RigExecFreezeProgram: an inherited
    /// clone verdict is also computed and a disagreement counted here.
    bool verifyCloneKeys = false;
    size_t cloneVerdictMismatches = 0;
    /// The reads after the head tier that a chain result or a record can
    /// answer (RigExecBakedReaderWalk), bound at Build; per walk, whether
    /// something it depends on moved this run (RigExecBakedNoteReaderWalks)
    /// and whether the value read through it did (set by the samplers).
    std::vector<RigExecBakedReaderWalk> readerWalks;
    std::vector<char> readerWalkMoved, readerWalkChanged;
    /// Per provider slot, the PropertyVersion ids its chain-routed avar
    /// binding declares. The avar table is compared by value, so these
    /// validate and re-sample; they seed nothing.
    std::vector<std::vector<uint32_t>> avarHeadReads;
};

inline bool
RigExecBakedIsHeadDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::PropertyResult ||
           domain == RigExecBakedSlotDomain::Rest ||
           domain == RigExecBakedSlotDomain::Ladder ||
           domain == RigExecBakedSlotDomain::SkinTopology;
}

inline std::vector<uint32_t>
RigExecBakedHeadIndices(const RigExecBakedProgramImpl &B)
{
    std::vector<uint32_t> indices;
    for (size_t i = 0; i < B.steps.size(); ++i)
        if (B.steps[i].isHead)
            indices.push_back(uint32_t(i));
    return indices;
}

/// \p input's sampled leaf, or its constant when it has none.
template <class T>
inline T
RigExecBakedLeaf(const RigExecBakedProgramImpl &program,
                 const RigExecBakedInput<T> &input)
{
    if (input.leaf < 0) {
        return input.constant;
    }
    return T(program.leaves.Of<T>().value[size_t(input.leaf)]);
}

bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, double *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, float *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, int *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, bool *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, TfToken *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfMatrix4d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3f *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtVec3fArray *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtArray<GfVec2f> *out);

/// The read every step body and the prologue's input fill make: the value
/// RigExecBakedRead answered for \p input when the leaf was last sampled,
/// which the re-sample rules make this run's answer. No stage, no resolved
/// inputs, no lock.
template <class T>
inline T
RigExecBakedLeafRead(const RigExecBakedProgramImpl &program,
                     const RigExecBakedInput<T> &input)
{
    TF_VERIFY(input.leaf >= 0 || !input.varying);
    T value =
        input.walk >= 0 ? input.constant : RigExecBakedLeaf(program, input);
    if (input.walk >= 0)
        RigExecBakedResolveReaderWalk(program, input.walk, &value);
    return value;
}

/// The path leaves path leaf \p ref belongs to, or null when the program no
/// longer holds its owner.
inline RigExecBakedPathLeaves *
RigExecBakedPathLeavesOf(RigExecBakedProgramImpl *program,
                         const RigExecBakedPathLeafRef &ref)
{
    switch (ref.owner) {
    case RigExecBakedPathLeafOwner::Provider:
        return &program->providerLeaves;
    case RigExecBakedPathLeafOwner::Revision:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].revisions.size()) {
            return &program->chains[ref.a].revisions[ref.b].leaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::Derived:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].derived.size()) {
            return &program->chains[ref.a].derived[ref.b].revision.leaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::Weight:
        if (ref.a < program->weightObjects.size()) {
            return &program->weightObjects[ref.a].pointLeaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::WeightField:
        return ref.a < program->weightObjects.size()
                   ? &program->weightObjects[ref.a].fieldLeaves
                   : nullptr;
    case RigExecBakedPathLeafOwner::RevisionLayout:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].revisions.size()) {
            return &program->chains[ref.a].revisions[ref.b].layoutLeaves;
        }
        return nullptr;
    case RigExecBakedPathLeafOwner::DerivedLayout:
        if (ref.a < program->chains.size() &&
            ref.b < program->chains[ref.a].derived.size()) {
            return &program->chains[ref.a].derived[ref.b].revision.layoutLeaves;
        }
        return nullptr;
    }
    return nullptr;
}

inline const RigExecBakedPathLeaves *
RigExecBakedPathLeavesOf(const RigExecBakedProgramImpl &program,
                         const RigExecBakedPathLeafRef &ref)
{
    return RigExecBakedPathLeavesOf(
        const_cast<RigExecBakedProgramImpl *>(&program), ref);
}

/// Every writer of a provider leaf's value or blocked flag calls this when
/// either moved, so the next publication re-keys leaf \p k.
inline void
RigExecBakedNoteSpaceLeafSampled(RigExecBakedProgramImpl *program, size_t k)
{
    if (k < program->spaceLeafRekey.size()) {
        program->spaceLeafRekey[k] |= kRigExecSpaceLeafSampled;
    }
}

/// The one way a typed leaf's sampled value is recorded: `changed[k]`
/// becomes \p moved, which must compare bits (a NaN that stays NaN has not
/// moved, -0 is not 0), and k is listed in `changedList` when the byte goes
/// 0 -> 1. Owner thread (or the frozen worker on its own clone).
template <class Pool>
inline void
RigExecBakedNoteLeafWrite(Pool *pool, size_t k, bool moved)
{
    if (k >= pool->changed.size())
        return;
    if (moved && !pool->changed[k])
        pool->changedList.push_back(uint32_t(k));
    pool->changed[k] = moved ? 1 : 0;
}

/// Sets `mustSample[k]` and lists k in `markedList` when the byte goes
/// 0 -> 1.
template <class Pool>
inline void
RigExecBakedMarkPoolLeaf(Pool *pool, size_t k)
{
    if (k >= pool->mustSample.size())
        return;
    if (pool->editSerial.size() <= k) {
        pool->editSerial.resize(pool->mustSample.size(), 0);
    }
    ++pool->editSerial[k];
    if (pool->mustSample[k])
        return;
    pool->mustSample[k] = 1;
    pool->markedList.push_back(uint32_t(k));
}

/// Sets leaf \p id's `mustSample` byte, so the next sample re-reads it. An
/// id past the binding leaves names a path leaf.
inline void
RigExecBakedMarkLeaf(RigExecBakedProgramImpl *program, uint32_t id)
{
    if (id >= program->leafRefs.size()) {
        const size_t i = id - program->leafRefs.size();
        if (i >= program->pathLeafRefs.size()) {
            const size_t h = i - program->pathLeafRefs.size();
            if (h < program->headLeaves.size()) {
                program->headLeaves[h].mustSample = 1;
                ++program->headLeaves[h].editSerial;
            }
            return;
        }
        const RigExecBakedPathLeafRef &ref = program->pathLeafRefs[i];
        RigExecBakedPathLeaves *leaves = RigExecBakedPathLeavesOf(program, ref);
        if (leaves && ref.key < leaves->mustSample.size()) {
            leaves->mustSample[ref.key] = 1;
            if (ref.owner == RigExecBakedPathLeafOwner::Provider &&
                ref.key < program->providerLeafEditSerial.size())
                ++program->providerLeafEditSerial[ref.key];
        }
        return;
    }
    const RigExecBakedLeafRef &ref = program->leafRefs[id];
    program->leaves.ForEach([&ref](auto &pool) {
        using T = typename std::decay_t<decltype(pool)>::Type;
        if (RigExecBakedLeafTraits<T>::type == ref.type) {
            RigExecBakedMarkPoolLeaf(&pool, ref.index);
        }
    });
}

/// Whether RigExecRevisionLeafHops lists every path where \p key's read can
/// meet the resolved-input overlay: false only for a Dial key whose flavour
/// is neither Resolved nor ResolvedOnly. (bakedGeometry.cpp)
bool RigExecRevisionLeafHopsComplete(const RigExecRevisionLeafKey &key);

/// Fills `overridePathsSorted` from `*interactiveOverrides`. Owner thread,
/// first thing in Run's prologue. (bakedGeometry.cpp)
void RigExecBakedPrepareOverridePaths(RigExecBakedProgramImpl *program);

/// Whether binding leaf \p id (a `leafRefs` id) moved in this run's sample.
inline bool
RigExecBakedLeafChanged(const RigExecBakedProgramImpl &program, uint32_t id)
{
    if (id >= program.leafRefs.size()) {
        return false;
    }
    const RigExecBakedLeafRef &ref = program.leafRefs[id];
    const auto at = [&ref](const auto &pool) {
        return ref.index < pool.changed.size() && pool.changed[ref.index] != 0;
    };
    const RigExecBakedLeafPools &pools = program.leaves;
    switch (ref.type) {
    case RigExecBakedLeafType::Double:
        return at(pools.Of<double>());
    case RigExecBakedLeafType::Float:
        return at(pools.Of<float>());
    case RigExecBakedLeafType::Int:
        return at(pools.Of<int>());
    case RigExecBakedLeafType::Bool:
        return at(pools.Of<bool>());
    case RigExecBakedLeafType::Token:
        return at(pools.Of<TfToken>());
    case RigExecBakedLeafType::Matrix4d:
        return at(pools.Of<GfMatrix4d>());
    case RigExecBakedLeafType::Vec3d:
        return at(pools.Of<GfVec3d>());
    case RigExecBakedLeafType::Vec3f:
        return at(pools.Of<GfVec3f>());
    }
    return false;
}

/// Numbers one leaf per binding _ForEachPatchableInput visits, in its order,
/// seeded with the binding's constant, and files each under every path
/// `overridableInputs` gives its override number. Then numbers every path
/// leaf after them (`pathLeafRefs`), filed under every path its read can
/// reach, and every head leaf after those, filed under its path. Renumbers
/// from scratch, so Build calls it again once every binding exists.
const VtValue *RigExecBakedSpaceLeafOverlay(const RigExecBakedProgramImpl &,
                                            size_t);
GfMatrix4d RigExecBakedProviderParentRaw(const RigExecBakedProgramImpl &,
                                         int slot);
bool
RigExecBakedBuildSpaces(RigExecBakedProgramImpl *program, UsdTimeCode capture,
                        std::string *error,
                        const RigExecSceneDescriptors *scene = nullptr,
                        const UsdStageWeakPtr &sceneStage = UsdStageWeakPtr(),
                        uint64_t sceneSerial = 0);
bool RigExecBakedBindOwnPropertySpaces(RigExecBakedProgramImpl *program,
                                       std::string *error);
/// Build, once every non-provider step and cross-domain read is declared and
/// before the provider leaves are numbered: binds the routed inputs' reads,
/// erases the refresh templates and, unless RIGEXEC_PROVIDER_PRUNE=0, drops
/// every provider op, sampled row, external and frame input no consumer or
/// diagnostic root reaches, renumbering the survivors densely in their
/// order. False with \p error when a holder cannot be compacted.
bool RigExecBakedPruneProviderProgram(RigExecBakedProgramImpl *program,
                                      std::string *error);
/// Pushes an op node origin per provider op that is not an Attribute,
/// after pruning and before the op program is built, so the provider op
/// nodes are numbered as their origins are.
void RigExecBakedPlanProviderOps(RigExecBakedProgramImpl *program);
void RigExecBakedBuildSpaceSteps(RigExecBakedProgramImpl *program);
void RigExecBakedSampleSpaces(RigExecBakedProgramImpl *program,
                              UsdTimeCode time, bool all);
void RigExecBakedRunSpaceOp(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step);
void RigExecBakedPlanProviderRefreshes(RigExecBakedProgramImpl *program);
bool RigExecBakedBindProviderRefresh(
    RigExecBakedProgramImpl *program, uint32_t index,
    std::vector<uint32_t> *base, std::vector<uint32_t> *fin,
    const std::function<uint32_t(bool, int)> &allocate,
    RigExecProviderContextIndex *contextIndex,
    const std::vector<std::vector<size_t>> &externalInputsBySlot);
/// The providerRefreshes index op step \p step refreshes, or -1 for any
/// other step.
int RigExecBakedProviderRefreshOf(const RigExecBakedProgramImpl &program,
                                  const RigExecBakedStep &step);
/// The provider op index op step \p step runs, or -1 for any other step.
int RigExecBakedProviderOpOf(const RigExecBakedProgramImpl &program,
                             const RigExecBakedStep &step);

void RigExecBakedNumberLeaves(RigExecBakedProgramImpl *program);

/// Resolves \p leaves' keys on \p stage at Build: each key's attribute, the
/// paths its read can reach and whether it can move with the time; seeds
/// every value with its key's fallback. Owning thread.
void RigExecBakedBindPathLeaves(const UsdStageRefPtr &stage,
                                RigExecBakedPathLeaves *leaves);

/// Restarts \p leaves' content versions at their current values: version
/// 0, which no run has read yet.
void RigExecBakedResetPathLeafVersions(RigExecBakedPathLeaves *leaves);

/// Stores \p value as key \p k of \p leaves, \p run being the program's
/// `pathLeafRun`. The key's version moves exactly when the bytes differ
/// from those the last run to read it saw (RigExecBakedHeadValueSame), so
/// writes between two runs that end on the same bytes leave it unmoved.
/// Returns whether the bytes differ from the value replaced. Every writer
/// of a geometry owner's `values` goes through here.
bool RigExecBakedSetPathLeaf(RigExecBakedPathLeaves *leaves, size_t k,
                             VtValue value, uint64_t run);

/// Whether operation keys carry the content versions of \p ref's owner's
/// leaves in place of their values: every geometry owner. Provider leaves
/// keep their values.
inline bool
RigExecBakedPathLeafVersioned(const RigExecBakedPathLeafRef &ref)
{
    return ref.owner != RigExecBakedPathLeafOwner::Provider;
}

/// Re-reads, through RigExecSampleRevisionLeaf over the generation's
/// resolved inputs at \p time, every key of \p leaves that one of these says
/// can have moved since its last sample, and sets its `changed` byte:
///  1. the first sample, a moved program stamp, or \p all (a forced run);
///  2. an interactive override standing now or at the last sample on one of
///     the key's hops (`overridePathsSorted`), the only paths its read
///     consults the overlay at; once after, through `overrideReached`. A
///     key whose hops are not complete (RigExecRevisionLeafHopsComplete)
///     re-reads while any override stands and once after, as does every
///     key but a topology one (`compiledRig`) under RIGEXEC_PATH_LEAF_GATING=0;
///  3. a moved time, for a key read at the time that can vary with it (or
///     when the time moves to or from Default);
///  4. its `mustSample` byte: a value edit reached one of its paths
///     (ApplyValueEdits files it under each), and its time variance is
///     re-derived with the read;
///  5. moved chain results (`pathLeafChainSerial`).
/// A key with a reader walk is read through it
/// (RigExecBakedSampleWalkedPathLeaf), not through the resolved inputs.
/// Keys in \p skip are not read; each is marked to re-read when it next is
/// not skipped. Every other key keeps its value. Owning thread, prologue
/// only.
void RigExecBakedSamplePathLeaves(RigExecBakedProgramImpl *program,
                                  RigExecBakedPathLeaves *leaves,
                                  UsdTimeCode time, bool all,
                                  const std::vector<int> &skip = {});

/// Whether key \p k of \p leaves can read another value at another time,
/// as the next sample answers it: `varying[k]` while that answer is current
/// (RigExecBakedPathLeaves::VarianceStale false), else the rebind's own
/// question (RigExecRevisionLeafHops) asked again without storing it.
/// Owning thread.
bool RigExecBakedLeafVaryingNow(const RigExecBakedProgramImpl &program,
                                const RigExecBakedPathLeaves &leaves, size_t k);

/// Which leaves one RigExecBakedSampleLeaves pass reads.
enum class RigExecBakedLeafPass : uint8_t { All, BeforeHead, ChainRouted };

/// Prologue step 1's upstream half, after the resolved inputs are cleared
/// and before the interactive overrides are placed. Compares `upstream`
/// with `lastUpstream` by value; for every path placed, moved or lifted
/// (rule 8) it marks the leaves `leafByPath` files there and sets
/// `upstreamChanged` for the override numbers filed there, and it rebuilds
/// `upstreamOn`. Then places every value on an `upstreamWeightFieldReads` path into
/// the resolved inputs. `upstreamChanged` holds this run's moves only.
/// Runs on the thread that owns \p program: live's owning thread, or a
/// frozen worker over its private clone, which passes \p placeWeightFieldReads false:
/// it builds no admission set (that reads the stage), and the freeze
/// refuses every rig with an object the weight field resolves.
void RigExecBakedPlaceUpstream(RigExecBakedProgramImpl *program,
                               bool placeWeightFieldReads = true);

/// Re-reads, through RigExecBakedRead at \p time, every leaf that one of
/// these says can have moved since it was last sampled, and sets its
/// `changed` byte by bitwise comparison:
///  1. the first run of the program, a moved program stamp, or \p all from
///     Build or a forced run;
///  2. \p all from a frozen job, whose clone carries live's flags and time;
///  3. a moved time, for a varying binding;
///  4. an override on its number now or at the last run: while a drag
///     stands and once after it lifts, since a drag scrubbed at a held frame
///     keeps both flags set;
///  5. an edit on its number (`edited`), or its `mustSample` byte: a value
///     edit that reached one of its paths (ApplyValueEdits), or
///     RigExecProgramAvarPatch;
///  6. a value in `routedOverrides` that differs from `lastRoutedOverrides`
///     on one of its paths;
///  7. for a binding with a reader walk, a head leaf, override slot or
///     declared version of that walk that moved (`readerWalkMoved`);
///  8. an upstream value placed, moved or lifted on one of its paths
///     (RigExecBakedPlaceUpstream sets `mustSample`).
/// Every other leaf keeps its value: none of its read's inputs can have
/// moved for it. A binding with a reader walk is read through it
/// (RigExecBakedReadWalked), any other through RigExecBakedRead, the long
/// way through the upstream layer while a value stands on its number
/// (`upstreamOn`). Owning thread, prologue only.
///
/// Without rules 1-2 a pass visits only the inputs rules 3-6 and 8 can
/// select: the pools' `markedList`s, the varying inputs and the promoted
/// avars when the time moved, and the inputs under an override number set
/// in `overridden`, `lastOverridden` or `edited` (`sparseInputs`).
/// RIGEXEC_SPARSE_SAMPLING=0 visits every input; both read the same leaves.
///
/// \p pass splits the live prologue around the head tier: BeforeHead reads
/// every leaf but the chain-routed ones (a binding with `resolvedAttr` or a
/// reader walk), whose walks no chain result can reach, and applies rule 6;
/// ChainRouted reads the rest once the property chains are published and
/// RigExecBakedNoteReaderWalks has run. All does both, in one pass.
void
RigExecBakedSampleLeaves(RigExecBakedProgramImpl *program, UsdTimeCode time,
                         bool all,
                         RigExecBakedLeafPass pass = RigExecBakedLeafPass::All);

/// Emits one OpNode head step per revision of every property chain (the
/// first also reads the base and writes version 0), numbers the op nodes
/// (`firstNode`), the PropertyVersion domain (`versionBase`, record ids)
/// and sizes the head tier's run state. Each step declares the head leaves
/// and override slots its lookups read and, as head reads, the version
/// before it and every chain version or record one of its walks meets.
/// Build only.
void RigExecBakedBuildPropertySteps(RigExecBakedProgramImpl *program);

/// Re-reads every head leaf that can have moved since its last sample and
/// sets its `changed` byte by bitwise comparison:
///  1. the first sample, a moved program stamp, or \p all (a forced run);
///  2. its `mustSample` byte (a value edit reached its path), which also
///     re-derives whether it varies;
///  3. a moved time, for a leaf that varies (or a time moving to or from
///     Default).
/// A head leaf is a raw stage value, so overrides and chain results never
/// move it: the walk resolver reads those beside it. Owning thread,
/// prologue only.
void RigExecBakedSampleHeadLeaves(RigExecBakedProgramImpl *program,
                                  UsdTimeCode time, bool all);

/// \p leaf's value at \p time as RigExecBakedSampleHeadLeaves reads it: the
/// attribute's own typed value, empty when it has none of the leaf's type.
/// Reads the stage: owning thread (the live prologue, or the frozen sampler
/// on the UI thread).
VtValue RigExecBakedReadHeadLeaf(const RigExecBakedHeadLeaf &leaf,
                                 UsdTimeCode time);

/// Whether \p value holds exactly \p leaf's type.
bool RigExecBakedHeadLeafHolds(const RigExecBakedHeadLeaf &leaf,
                               const VtValue &value);

/// Whether \p leaf's attribute might hold a different value at another
/// time: the predicate RigExecBakedSampleHeadLeaves re-derives `varying`
/// with. A leaf that answers false reads one value at every numeric time
/// code. Reads the stage: owning thread.
bool RigExecBakedHeadLeafVaries(const RigExecBakedHeadLeaf &leaf);

/// The origin of op step \p step's node; null for an injected node and for
/// any other step.
const RigExecBakedOpOrigin *
RigExecBakedOriginOf(const RigExecBakedProgramImpl &B,
                     const RigExecBakedStep &step);
/// The step's own kind, object and part; for an op node whose origin keeps
/// another kind's identity, that kind with the origin's object and part.
RigExecBakedStepIdentity
RigExecBakedIdentityOf(const RigExecBakedProgramImpl &B,
                       const RigExecBakedStep &step);
/// The revisions op chain step \p step computes: a head's members', a range
/// step's segment's; empty for any other step.
void RigExecBakedOpStepRevisions(const RigExecBakedProgramImpl &B,
                                 const RigExecBakedStep &step,
                                 std::vector<int> *ids);
/// The property chain and part (1 to its revision count) op step \p step
/// runs: revision part, and for part 1 also the chain's base. False for any
/// other step.
bool RigExecBakedPropertyPartOf(const RigExecBakedProgramImpl &B,
                                const RigExecBakedStep &step, size_t *chain,
                                size_t *part);
/// Adds the property chains' op nodes to \p builder, numbered from 0 as
/// RigExecBakedBuildPropertySteps numbered them, with the lookups their
/// inputs read and the versions they store. False with the reason when a
/// chain cannot run: its target is gone or a revision names no operation.
bool RigExecBakedAddPropertyOps(const RigExecBakedProgramImpl &B,
                                RigExecRigDescBuilder *builder,
                                eval::OpStepLayout *layout, std::string *error);
/// Resolves property lookup \p lookup, the layout's \p index-th, into
/// \p out, a value of its input's \p type. False with the error that stops
/// the evaluation: a missing or non-finite base, a failed weight field.
bool RigExecBakedLookUpProperty(RigExecBakedProgramImpl &B,
                                const eval::OpLookup &lookup, size_t index,
                                const RigExecTypeDesc &type, void *out,
                                std::string *error);
/// Stores a property node's value in version \p output.object; the base's
/// store also marks its chain valid. Sets the version's `propertyChanged`
/// byte by bitwise comparison.
bool RigExecBakedStorePropertyVersion(RigExecBakedProgramImpl &B,
                                      const eval::OpHostOutput &output,
                                      const void *value, std::string *error);
/// Adds a VolumePlacement op node per VolumePlacement origin to
/// \p builder, numbered as RigExecBakedBuildWeightSteps numbered them after
/// the property nodes: its frame looked up at the slot's last fin or base
/// version, its placement stored in volumePlacement or volumePlacementBase.
/// Adds the weight packet op node of each WeightPacket origin to
/// \p builder, numbered after the VolumePlacement nodes: StaticWeights,
/// DynamicWeights, CombineWeights or VolumeWeights fed by the nodes of the
/// objects it composes and by WeightArrays lookups, its packet stored in
/// weightPackets. False with the reason for a weight object that can make
/// no packet: an unknown representation, policy, mode, axis or bounds, no
/// base for a remapped packet, no placement, target or curve for a volume,
/// or painted weights that fail whatever the frame.
/// A node's output another node reads directly.
struct RigExecBakedOpFeed {
    RigExecNodeId node = kRigExecInvalidIndex;
    const RigExecPortHandle *port = nullptr;
};
/// ... and in \p packets, per weight object, the output its packet comes
/// from, or none.
bool RigExecBakedAddWeightPacketOps(const RigExecBakedProgramImpl &B,
                                    RigExecRigDescBuilder *builder,
                                    eval::OpStepLayout *layout,
                                    std::vector<RigExecBakedOpFeed> *packets,
                                    std::string *error);
/// Resolves a WeightArrays lookup: input \p lookup.read of weight object
/// \p lookup.object, an array lent in place.
bool RigExecBakedLookUpWeight(RigExecBakedProgramImpl &B,
                              const eval::OpLookup &lookup,
                              const RigExecTypeDesc &type, void *out,
                              std::string *error);
/// Adds the op nodes of each WeightField origin of a revision's field
/// (\p revisions) or of an envelope's: per weight object of the field,
/// StaticWeightField, DynamicWeightField, CombineWeightFields or a Sphere,
/// Plane or Curve weight, fed by the nodes of the objects it composes and by
/// WeightFieldInputs lookups (a volume's in-flight points by a ChainVersion
/// lookup), the root's weights stored in weightFields. False with the reason
/// for a field no frame can weigh.
/// Per weight field it adds, \p fields gets the output its weights come
/// from.
bool RigExecBakedAddWeightFieldOps(const RigExecBakedProgramImpl &B,
                                   RigExecRigDescBuilder *builder,
                                   eval::OpStepLayout *layout, bool revisions,
                                   std::vector<RigExecBakedOpFeed> *fields,
                                   std::string *error);
/// Resolves a WeightFieldInputs lookup: input \p lookup.read of weight
/// object \p lookup.part as weight field \p lookup.object reads it.
bool RigExecBakedLookUpWeightField(const RigExecBakedProgramImpl &B,
                                   const eval::OpLookup &lookup,
                                   const RigExecTypeDesc &type, void *out,
                                   std::string *error);
/// Resolves a ChainVersion lookup: the points entering revision
/// \p lookup.part of point chain \p lookup.object, lent in place, or
/// gathered into chainVersionGather[\p index].
bool RigExecBakedLookUpChainVersion(RigExecBakedProgramImpl &B, uint32_t index,
                                    const eval::OpLookup &lookup,
                                    const RigExecTypeDesc &type, void *out,
                                    std::string *error);
/// Stores a weight field's weights as weightFields[output.object].
bool RigExecBakedStoreWeightField(RigExecBakedProgramImpl &B,
                                  const eval::OpHostOutput &output,
                                  const void *value, std::string *error);
/// Stores a weight packet op's packet as weightPackets[output.object].
bool RigExecBakedStoreWeightPacket(RigExecBakedProgramImpl &B,
                                   const eval::OpHostOutput &output,
                                   const void *value, std::string *error);
void RigExecBakedAddVolumePlacementOps(const RigExecBakedProgramImpl &B,
                                       RigExecRigDescBuilder *builder,
                                       eval::OpStepLayout *layout);
/// The ProviderMatrix, FrameMatrix or ProviderRefresh op node of origin
/// \p o, numbered \p o.
bool RigExecBakedAddSpaceOp(const RigExecBakedProgramImpl &B,
                            RigExecRigDescBuilder *builder,
                            eval::OpStepLayout *layout, size_t o,
                            std::string *error);
/// The arithmetic and affine provider operations as op nodes, numbered as
/// their origins are; their origins are the last pushed.
bool RigExecBakedAddProviderOps(const RigExecBakedProgramImpl &B,
                                RigExecRigDescBuilder *builder,
                                eval::OpStepLayout *layout, std::string *error);
/// Resolves a RestFrame, CommitRecordGate, ProviderFrame, ProviderParentRaw
/// or ProviderInputs lookup.
bool RigExecBakedLookUpSpace(const RigExecBakedProgramImpl &B,
                             const eval::OpLookup &lookup,
                             const RigExecTypeDesc &type, void *out,
                             std::string *error);
/// Stores a provider's final or base matrix, a frame record's matrix or
/// validity, a refresh's frames or a provider op's result.
bool RigExecBakedStoreSpace(RigExecBakedProgramImpl &B,
                            const eval::OpHostOutput &output, const void *value,
                            std::string *error);
/// Resolves a pose version lookup: the frame at fin (part 0) or base
/// (part 1) version \p lookup.object, as a rig PointFrame.
bool RigExecBakedLookUpPoseVersion(const RigExecBakedProgramImpl &B,
                                   const eval::OpLookup &lookup,
                                   const RigExecTypeDesc &type, void *out,
                                   std::string *error);
/// Stores a VolumePlacement node's placement of slot \p output.object.
bool RigExecBakedStoreVolumePlacement(RigExecBakedProgramImpl &B,
                                      const eval::OpHostOutput &output,
                                      const void *value, std::string *error);
/// The PoseCommit op node of origin \p o, numbered \p o.
bool RigExecBakedAddPoseCommitOp(const RigExecBakedProgramImpl &B,
                                 RigExecRigDescBuilder *builder,
                                 eval::OpStepLayout *layout, size_t o,
                                 std::string *error);
/// Resolves a CommitVersions or CommitCandidates lookup.
bool RigExecBakedLookUpCommit(const RigExecBakedProgramImpl &B,
                              const eval::OpLookup &lookup,
                              const RigExecTypeDesc &type, void *value,
                              std::string *error);
/// Stores a PoseCommit node's frames in the versions its commit writes, or
/// which of its candidates are present.
bool RigExecBakedStoreCommit(RigExecBakedProgramImpl &B,
                             const eval::OpHostOutput &output,
                             const void *value, std::string *error);
/// The failure \p error of commit \p commit's PoseCommit node, as its
/// mover reports it.
std::string RigExecBakedCommitFailure(const RigExecBakedProgramImpl &B,
                                      int commit, const std::string &error);
/// The output of the op node that solves constraint walk step \p w.
const RigExecPortHandle &
RigExecBakedConstraintOut(const RigExecBakedProgramImpl &B, size_t w);
/// Whether the constraint walk step \p w commits solves as op nodes.
bool RigExecBakedConstraintRunsAsOps(const RigExecBakedProgramImpl &B,
                                     size_t w);
/// The Constraint op node of origin \p o, numbered \p o.
bool RigExecBakedAddConstraintOp(const RigExecBakedProgramImpl &B,
                                 RigExecRigDescBuilder *builder,
                                 eval::OpStepLayout *layout, size_t o,
                                 std::string *error);
namespace eval {
struct ConstraintStepCache;
}
/// Resolves a constraint lookup (eval::OpConstraintLookupSource) through
/// the calling step's \p cache.
bool RigExecBakedLookUpConstraint(RigExecBakedProgramImpl &B,
                                  eval::ConstraintStepCache &cache,
                                  const eval::OpLookup &lookup,
                                  const RigExecTypeDesc &type, void *value,
                                  std::string *error);
/// Stores a constraint node's solved frame or measured delta from \p step,
/// which takes its diagnostic lines.
bool RigExecBakedStoreConstraint(RigExecBakedProgramImpl &B,
                                 RigExecBakedStep *step,
                                 eval::ConstraintStepCache &cache,
                                 const eval::OpHostOutput &output,
                                 const void *value, std::string *error);
/// The failure \p error of a node of constraint commit \p w, as its mover
/// reports it.
std::string RigExecBakedConstraintFailure(const RigExecBakedProgramImpl &B,
                                          int w, const std::string &error);
/// The cross-domain read diagnostics of revision \p part of \p chain.
void RigExecBakedPropertyInputLines(const RigExecBakedProgramImpl &B,
                                    size_t chain, size_t part,
                                    std::vector<std::string> *lines);
/// After the step writing version \p part of \p chain ran or replayed:
/// when the version moved, the records that read it (value and `changed`
/// byte) and, for the chain's last part, the chain's final value.
void RigExecBakedFinishPropertyStep(RigExecBakedProgramImpl *program,
                                    size_t chain, size_t part);

/// Publishes chains [\p begin, \p end) as _EvaluatePropertyChains leaves
/// them: each valid chain's final version at its target and each record
/// not standing aside at its consumer, into the generation's resolved
/// inputs and, when \p results, into `propertyResults`. A skipped chain
/// publishes nothing, so an override on its target stays. Runs after the
/// head tier on every run, whether its ops ran or replayed.
void RigExecBakedPublishPropertyChains(RigExecBakedProgramImpl *program,
                                       size_t begin, size_t end, bool results);
inline void
RigExecBakedPublishPropertyChains(RigExecBakedProgramImpl *program)
{
    RigExecBakedPublishPropertyChains(program, 0,
                                      program->propertyChains.size(), true);
}

/// Places this run's interactive overrides into the head tier's override
/// slots (the last at a path wins, as the overlay takes them), marks each
/// slot whose value moved since the last run and each record an override
/// stands aside, and clears every PropertyVersion `changed` byte except a
/// record's whose hops' overrides moved. The head tier's first act on every
/// run.
void RigExecBakedPlaceHeadOverrides(RigExecBakedProgramImpl *program);

/// Whether two head values are the same bit for bit (a signed zero or a NaN
/// payload is a difference); empty equals only empty.
bool RigExecBakedHeadValueSame(const VtValue &a, const VtValue &b);

/// Each of \p paths spelled once, on the owning thread, for
/// RigExecBakedProgramImpl::pathTexts.
inline std::shared_ptr<const std::vector<std::string>>
RigExecBakedSpellPathTexts(const std::vector<SdfPath> &paths)
{
    auto texts = std::make_shared<std::vector<std::string>>();
    texts->reserve(paths.size());
    RigExecPathText pathText;
    for (const SdfPath &path : paths) {
        texts->push_back(pathText(path));
    }
    return texts;
}

/// What reader walk \p walk answers after this run's head tier and
/// publication: RigExecResolvedInputs::GetAttribute over the published
/// overlay, restated over head leaves, override slots, chain finals and
/// records. False, leaving \p out alone, where that read answers nothing.
/// Pure: no stage, no path work, no lock; a frozen worker calls it.
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, double *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, float *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, int *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, bool *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, TfToken *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfMatrix4d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3d *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, GfVec3f *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtVec3fArray *out);
bool RigExecBakedResolveReaderWalk(const RigExecBakedProgramImpl &program,
                                   int walk, VtArray<GfVec2f> *out);

/// RigExecBakedRead for a binding with a reader walk: the walk read while
/// an override or an upstream value (\p upstreamOn) stands on its number or
/// the binding varies (the walk is the one from its head, over head leaves
/// that read the upstream layer), else its constant.
template <class T>
inline T
RigExecBakedReadWalked(const RigExecBakedProgramImpl &program,
                       const RigExecBakedInput<T> &input,
                       const std::vector<char> &overridden,
                       const std::vector<char> *upstreamOn = nullptr)
{
    T value = input.constant;
    const auto on = [&input](const std::vector<char> &flags) {
        return input.overrideIndex >= 0 &&
               size_t(input.overrideIndex) < flags.size() &&
               flags[size_t(input.overrideIndex)];
    };
    const bool flagged = on(overridden) || (upstreamOn && on(*upstreamOn));
    if (flagged || input.varying) {
        RigExecBakedResolveReaderWalk(program, input.walk, &value);
    }
    return value;
}

/// Whether a move of version \p id is shadowed for a reader by \p shadowed
/// (a reader walk's or a step's): it is read only past records, and none
/// of them stands aside this run.
bool RigExecBakedReadIsShadowed(
    const RigExecBakedProgramImpl &program,
    const std::vector<std::pair<uint32_t, uint32_t>> &shadowed, uint32_t id);

/// After the head tier and its publication: marks each reader walk whose
/// head leaves, override slots or declared versions moved this run
/// (`readerWalkMoved`), and clears `readerWalkChanged`. Owning thread.
void RigExecBakedNoteReaderWalks(RigExecBakedProgramImpl *program);

/// RigExecSampleRevisionLeaf for a path-leaf key whose read has reader walk
/// \p walk, over the head tier's state: Resolved reads the walk, else the
/// head's own value, else the key's fallback; ResolvedOnly the walk, else
/// the fallback; Dial the walk as the dial attribute's type, cast to
/// double. Pure.
VtValue RigExecBakedSampleWalkedPathLeaf(const RigExecBakedProgramImpl &program,
                                         const RigExecRevisionLeafKey &key,
                                         int walk);

/// Fills every step's `reads` from the reader walks in its
/// `readerWalks`, after RigExecBakedDeclareInputDependencies, and
/// `avarHeadReads`. Build only.
void RigExecBakedDeclareHeadReads(RigExecBakedProgramImpl *program);

/// Appends to every region step's `reads` the Rest and Ladder slots its
/// body indexes: ComposeSubtree the ladders of its group, of the parents
/// it composes against and of every switch source, space and recomposed
/// slot; Solve the rests of its `restSlots`; Constraint the rests of its
/// IK targets (without animated translations) and of a matrix mover's
/// sources, and the ladder of its space; ProviderMatrix and FrameMatrix
/// their slot's rest; PoseInterpolator its driver's and parent's rests;
/// Derived the rests of its projector slots. Build only, after
/// RigExecBakedDeclareHeadReads.
std::vector<RigExecBakedSlotRange>
RigExecBakedRequiredRestReads(const RigExecBakedProgramImpl &program,
                              const RigExecBakedStep &step);

void RigExecBakedDeclareRestReads(RigExecBakedProgramImpl *program);

/// The region half of the head tier's validation, once the steps exist:
/// every head output a step or an avar slot declares has a head producer,
/// and every reader walk a step reads declares each chain target and record
/// consumer it meets ("walk <path> meets chain target without declaring
/// it"). False with the first violation in \p error.
bool RigExecBakedValidateHeadReads(const RigExecBakedProgramImpl &program,
                                   std::string *error);

/// The matrix table \p revision reads at its own phase: FinalMatrix for a
/// final-phase revision, BaseMatrix otherwise.
inline RigExecBakedSlotDomain
RigExecBakedOwnMatrixDomain(
    const RigExecBakedProgramImpl::GeomRevision &revision)
{
    return revision.finalPhase ? RigExecBakedSlotDomain::FinalMatrix
                               : RigExecBakedSlotDomain::BaseMatrix;
}

/// Calls visit(domain, slot) for every provider matrix \p revision reads.
/// Transform, transformSpace, carrySpace and influences are read at the
/// revision's own phase; for a matrix target (RigExecIsDerivedMatrixOp) the
/// first three are visited at BOTH phases, because its projector frames are
/// built from base and final of all three (RigExecBakedProjectorFrames). The
/// ProviderMatrix need tables and the steps' declared reads both come from
/// here.
template <class Visit>
void
RigExecBakedForEachMatrixRead(
    const RigExecBakedProgramImpl::GeomRevision &revision, Visit &&visit)
{
    const RigExecBakedSlotDomain own = RigExecBakedOwnMatrixDomain(revision);
    const bool bothPhases = RigExecIsDerivedMatrixOp(revision.op);
    for (const int slot : {revision.transformSlot, revision.transformSpaceSlot,
                           revision.carrySpaceSlot}) {
        if (slot < 0)
            continue;
        if (bothPhases) {
            visit(RigExecBakedSlotDomain::BaseMatrix, slot);
            visit(RigExecBakedSlotDomain::FinalMatrix, slot);
        } else {
            visit(own, slot);
        }
    }
    for (const int slot : revision.influenceSlots) {
        if (slot >= 0)
            visit(own, slot);
    }
}

// What committing an input records, and where it can be recorded.
// A committed input records two kinds of fact. Its override NUMBER is
// order-bearing: numbers are handed out from one running counter, so the
// order inputs are committed in IS the numbering the program carries, and
// the entry of `overridableInputs` a number is appended to is ordered by it.
// Everything else is order-free -- two counts that sum, and paths landing
// in the sets of the invalidation index, where inserting a path a second
// time is a no-op whenever it happens.
// RigExecBakedRecordBind and RigExecBakedRecordFold state both kinds once,
// against a sink that receives them, and there are two sinks:
//  * RigExecBakedProgramSink writes straight into the program, numbering
//    from `overridden.size()`. This is the serial commit, CommitBind's, one
//    input at a time in program order.
//  * RigExecBakedCommitShard gathers one CHUNK of a parallel pass into
//    vectors, numbering from zero within the chunk. A chunk's first real
//    number is the running counter at the phase's entry plus every number
//    the chunks before it handed out -- a prefix sum, known only once every
//    chunk has counted -- and RigExecBakedMergeCommitShards adds it.
// So a parallel commit cannot drift from the serial one: the rule is
// written here once and only WHERE the facts land differs. Chunks are
// contiguous runs of the program order and are merged in that order, which
// is what makes the numbers, and each `overridableInputs` entry's order,
// the serial ones exactly.

/// The serial sink: every fact straight into \p program.
struct RigExecBakedProgramSink {
    RigExecBakedProgramImpl *program = nullptr;

    int Number()
    {
        const int index = int(program->overridden.size());
        program->overridden.push_back(0);
        return index;
    }
    void Overridable(const SdfPath &path, int index)
    {
        program->overridableInputs[path].push_back(index);
    }
    void Bound(bool varying)
    {
        ++program->boundInputs;
        if (varying)
            ++program->varyingInputs;
    }
    void Prim(const SdfPath &path)
    {
        program->prims.insert(path);
    }
    void Named(const SdfPath &path)
    {
        program->named.insert(path);
    }
    void Rebuild(const SdfPath &path)
    {
        program->rebuild.insert(path);
    }
    void Folded(const SdfPath &path)
    {
        program->folded.insert(path);
    }
    void Live(const SdfPath &path)
    {
        program->liveInputPaths.insert(path);
    }
};

/// The parallel sink: one chunk's facts, gathered on whichever thread runs
/// the chunk and touching nothing shared.
struct RigExecBakedCommitShard {
    SdfPathVector prims, named, rebuild, folded, live;
    /// (path, number RELATIVE to the chunk's first) per overridable walk
    /// step, in the order the program sink would have appended them.
    std::vector<std::pair<SdfPath, int>> overridable;
    int numbered = 0;
    size_t boundInputs = 0;
    size_t varyingInputs = 0;

    int Number()
    {
        return numbered++;
    }
    void Overridable(const SdfPath &path, int index)
    {
        overridable.emplace_back(path, index);
    }
    void Bound(bool varying)
    {
        ++boundInputs;
        if (varying)
            ++varyingInputs;
    }
    void Prim(const SdfPath &path)
    {
        prims.push_back(path);
    }
    void Named(const SdfPath &path)
    {
        named.push_back(path);
    }
    void Rebuild(const SdfPath &path)
    {
        rebuild.push_back(path);
    }
    void Folded(const SdfPath &path)
    {
        folded.push_back(path);
    }
    void Live(const SdfPath &path)
    {
        live.push_back(path);
    }

    /// Sorts and dedups the four path vectors, and sorts `overridable` by
    /// (path, number), so the merge inserts ascending runs. Run at the end
    /// of the chunk, on its own thread: this is the part of the set
    /// insertion that CAN be parallel.
    void Seal();
};

/// Folds \p shards into \p program in chunk order, and returns each shard's
/// first override number.
///
/// Shard k's numbers start where shard k-1's stop, and shard 0's where the
/// program's running counter stands on entry -- this phase continues the
/// numbering the phases before it started, it does not restart it. The
/// caller adds shard k's first number to every input shard k numbered.
std::vector<int>
RigExecBakedMergeCommitShards(RigExecBakedProgramImpl *program,
                              std::vector<RigExecBakedCommitShard> *shards);

/// Registers \p input, read as \p name on \p prim through \p walk, into
/// \p sink: the bound/varying counts, the invalidation index, and -- for an
/// input with a head -- an override number and the paths an override on it
/// can stand on. What CommitBind records, for either sink.
template <class Sink, class T>
void
RigExecBakedRecordBind(Sink *sink, const UsdPrim &prim, const TfToken &name,
                       RigExecBakedInput<T> *input, const SdfPathVector &walk)
{
    if (prim && prim.GetAttribute(name)) {
        sink->Bound(input->varying);
    }
    if (prim) {
        sink->Prim(prim.GetPath());
        // Named even when absent: a resync that CREATES this property is
        // an input appearing, and the bake captured the default it did
        // not find.
        sink->Named(prim.GetPath().AppendProperty(name));
    }
    // The registration proper: an input with no head has nothing an
    // override could stand on, and no number.
    if (!input->head) {
        return;
    }
    input->overrideIndex = sink->Number();
    // EVERY attribute on the resolution walk, not only the head one.
    // RigExecBakedClassifyInput followed an authored connection chain and
    // may have captured the value several hops upstream; an interactive
    // override standing on one of those hops is an override on this input,
    // and GetAttribute -- which is what the flag makes RigExecBakedRead use
    // -- consults the generation's resolved values at every step of the
    // same walk. Keyed by the head alone, such an override found no binding
    // and was reported placeable with nothing placed.
    for (const SdfPath &path : walk) {
        sink->Overridable(path, input->overrideIndex);
    }
    if (walk.empty()) {
        sink->Overridable(input->head.GetPath(), input->overrideIndex);
    }
    sink->Prim(input->head.GetPrim().GetPath());
    for (const SdfPath &path : walk) {
        sink->Named(path);
    }
    // A chain-resolved input redoes the whole walk live every frame, so
    // nothing about it was captured. A query was pinned to ONE attribute of
    // the walk, so every other attribute on it decided that choice; with a
    // walk of one there is no choice left to invalidate, and a value moving
    // on it -- including being cleared, which drops the input back to the
    // same fallback both paths use -- is answered by the query itself.
    const bool live = input->sourceBacked || input->resolvedAttr ||
                      (input->varying && walk.size() == 1);
    if (live) {
        if (input->resolvedAttr) {
            for (const SdfPath &path : walk) {
                sink->Live(path);
            }
            if (walk.empty()) {
                sink->Live(input->head.GetPath());
            }
        }
        return;
    }
    for (const SdfPath &path : walk) {
        sink->Rebuild(path);
    }
}

/// Records \p name on \p prim as read for its VALUE into \p sink; see
/// RigExecBakedBuildContext::Fold, which is this into the program.
template <class Sink>
void
RigExecBakedRecordFold(Sink *sink, const UsdPrim &prim, const TfToken &name)
{
    if (!prim) {
        return;
    }
    const SdfPath path = prim.GetPath().AppendProperty(name);
    sink->Rebuild(path);
    sink->Folded(path);
    sink->Named(path);
    sink->Prim(prim.GetPath());
}

// The compiled rig, in terms the program can name.
// RigExecRigEvaluator's compiled structures are private nested types, so only
// bakedProgram.cpp can read them. It restates the parts each domain's bake
// needs as the plain records below and hands those over; the per-domain bake
// functions then depend on the SHAPE of the compiled rig rather than on the
// evaluator, which is what keeps the friendship to one file.

/// One frame constraint of the compiled walk.
struct RigExecBakedConstraintSpec {
    SdfPath moverPath;
    TfToken schemaType;
    SdfPathVector targets;
    /// Per target, whether a read phase asked for its frame as of this
    /// constraint (the evaluator's _nativePhaseCheckpoints membership). A
    /// SingleChainIK names its whole chain here, and the dynamic walk
    /// records every one of them.
    std::vector<char> snapshotTargets;
    /// SingleChainIK: the inferred joint chain, first joint through end, and
    /// the two bindings it resolves beside its sources.
    SdfPathVector ikChain;
    /// Parallel to ikChain: 1 where a pose step BELOW this constraint wrote
    /// that joint, so the chain measures from what that step left.
    std::vector<char> ikRestLive;
    SdfPath effector, effectorXform;
    SdfPathVector poleObjects, poleObjectXforms;
    /// _IkUsesAnimatedTs over the chain, answered where the evaluator's
    /// private state is visible. "autoDetect" resolves against it; the other
    /// two modes ignore it.
    bool ikUsesAnimatedTs = false;
    /// One source path per binding, in the compiled order, and beside it the
    /// plain Xformable the binding reads off the stage when the walk holds
    /// no frame for it (the compiled _FrameSourceBinding::xformPath). Empty
    /// for a RigExec provider, whose frame the walk always has.
    SdfPathVector sources;
    SdfPathVector sourceXforms;
    /// Non-empty when rigExec:moves named <prim>.points: the constraint
    /// writes the GEOMETRY domain and revises no transform.
    SdfPath pointsTarget;
    /// Empty when the aim constraint named no world-up object.
    SdfPath worldUpObject;
    SdfPath worldUpXform;
    /// rigExec:space on a rotation constraint; empty when none is named.
    SdfPath space;
    /// rigExec:blendShear and rigExec:worldUpRotationOnly, compiled.
    bool blendShear = false;
    bool worldUpRotationOnly = false;
    /// rigExec:weightBlend == "radial" (transform-domain matrix mover).
    bool radialBlend = false;
    /// rigExec:weightObject, empty when the constraint binds none.
    SdfPath weightObject;
};

/// One geometry revision of a compiled chain.
struct RigExecBakedRevisionSpec {
    SdfPath moverPath;
    SdfPath target;
    RigExecRevisionOp op = RigExecRevisionOp::Skin;
    RigExecRevisionBinding binding;
    bool transformFinalPhase = false;
    /// rigExec:pointFrame == "posed": the points this cluster moves
    /// were already carried into the rig's posed frame by a skin. See
    /// RigExecClusterInPointFrame.
    bool transformPosedPoints = false;
    bool skinTopologyFixed = false;
    /// Whether a read phase asked for the target's points as of this
    /// revision (the evaluator's _nativePhaseCheckpoints membership).
    bool snapshotAfter = false;
};

/// One geometry chain and the derived targets maintained from its result.
struct RigExecBakedChainSpec {
    SdfPath target;
    std::vector<RigExecBakedRevisionSpec> revisions;
    std::vector<RigExecBakedRevisionSpec> derived;
};

/// One entry of the interleaved solver/constraint walk.
struct RigExecBakedWalkEntry {
    bool solverBatch = false;
    /// Solver batches only: the batch's dependency level, and its solvers in
    /// batch order with the (joint, element) outputs each one publishes.
    size_t level = 0;
    SdfPathVector batchSolvers;
    std::vector<std::vector<std::pair<SdfPath, int>>> solverJoints;
    /// Parallel to solverJoints: whether a read phase asked for that joint's
    /// frame as of that solver (the evaluator's _nativePhaseCheckpoints
    /// membership).
    std::vector<std::vector<char>> solverSnapshotJoints;
    /// Per solver, the joints whose REST is LIVE: a pose step below this
    /// solver in the rig's hierarchical stack already wrote them, so the
    /// solver measures from the frame that step left rather than from the
    /// authored rest. Empty on every rig with no pose step below a
    /// solver that writes one of its joints, which is every shipped rig.
    std::vector<SdfPathVector> solverLiveRestJoints;
    /// Constraints only.
    RigExecBakedConstraintSpec constraint;
};

/// The Build-time scratch every per-domain bake function shares.
///
/// Members shared by every per-domain bake function -- refuse, fold,
/// readToken, targets, bind, slotOf -- so that every domain records the
/// same invalidation facts for the same read.
/// Dropping one of them is how a captured constant stops being invalidated,
/// which is silent until an edit produces a wrong answer.
struct RigExecBakedBuildContext {
    RigExecBakedProgramImpl *program = nullptr;
    UsdStageRefPtr stage;
    /// Where a captured value is read, and where input classification probes
    /// so that a selection cannot depend on the caller's time code.
    UsdTimeCode capture = UsdTimeCode::Default();
    UsdTimeCode probe = UsdTimeCode::Default();
    /// Every property a chain writes; an input resolving through one cannot
    /// be captured.
    std::set<SdfPath> chainTargets;
    /// Each RigExecRibbon's resolved driver-points attribute, restated from
    /// the evaluator's own compiled map (which only Build's file can name).
    std::map<SdfPath, SdfPath> ribbonDriverPoints;
    /// The aggregate solvers no batch runs, in dependency order: a guide-only
    /// RigExecBlendPointFrames may read another one's aggregate, and the
    /// order is what makes the reader run second.
    std::vector<SdfPath> guideOnlySolvers;
    /// Every aggregate solver of the compiled rig, baked or not: a blend input that
    /// names one must already be baked, or the bake is refused.
    std::set<SdfPath> aggregateSolvers;
    std::vector<std::string> *reasons = nullptr;
    bool ok = true;

    /// Records one reason the compiled rig cannot be expressed, and refuses it.
    void Refuse(const std::string &what, const SdfPath &where);
    /// Records \p name as read for its VALUE: an edit rebuilds the program
    /// and an interactive override on it cannot be placed.
    void Fold(const UsdPrim &prim, const TfToken &name);
    /// Records \p name as read for its SHAPE -- whether it is authored at
    /// all. An edit rebuilds; an override, which authors nothing, places.
    void FoldShape(const UsdPrim &prim, const TfToken &name);
    /// Fold plus the plain uniform-token read the dynamic path performs.
    TfToken ReadToken(const UsdPrim &prim, const TfToken &name,
                      const TfToken &fallback);
    /// Fold plus the relationship's targets.
    SdfPathVector Targets(const UsdPrim &prim, const TfToken &name);
    /// The provider slot of \p path, or -1.
    ///
    /// EITHER kind: the table is the ordered union of the exec-seeded
    /// providers and the plain Xformables a constraint targets, and both
    /// kinds are LIVE -- the prologue seeds an xform-derived slot from the
    /// stage and a constraint revises it like any other target. So a
    /// `SlotOf(x) < 0` refusal reads "is no provider slot at all", NOT "is
    /// not a pose provider". A caller that means "publishes
    /// computeRestFrame" or "is composed from avars" must test
    /// slotKind[slot] == FirstFramePose itself, as the solver rest binding does.
    int SlotOf(const SdfPath &path) const;
    /// The RESOLVING half of Bind: reads the stage and decides what the
    /// channel is, recording nothing.
    ///
    /// Every call is a read of the composed stage and of `chainTargets`, so
    /// calls for distinct channels are independent and may run concurrently
    /// -- which is the point: a rig's providers carry about twenty channels
    /// each and resolving them is most of what Build costs. The compile-time
    /// structure digest already reads this stage from its own thread, so
    /// concurrent readers are not a new assumption here.
    ///
    /// The caller must commit every result afterwards: through CommitBind,
    /// ON ONE THREAD and in program order, or through RigExecBakedRecordBind
    /// into a RigExecBakedCommitShard per contiguous chunk of that order,
    /// merged by RigExecBakedMergeCommitShards. Skipping that leaves the
    /// input unregistered -- no override index, nothing in the invalidation
    /// index -- which is a silently wrong program rather than a failure.
    template <class T>
    RigExecBakedInput<T> ResolveBind(const UsdPrim &prim, const TfToken &name,
                                     T fallback, SdfPathVector *walk,
                                     bool sourceValue = false) const;

    /// The RECORDING half: the counters, the invalidation index and the
    /// override index.
    ///
    /// Order-bearing, and that is why it is separate rather than locked.
    /// It hands out `overrideIndex` from a running counter, so the order
    /// these are committed in IS the numbering a program carries. Call it in
    /// the same order the bindings appear in the program and a parallel
    /// resolve produces a bake identical to a serial one, byte for byte;
    /// call it in completion order and the program still works while no two
    /// builds of one rig agree. Large loops commit via shards
    /// (RigExecBakedRecordBind).
    template <class T>
    void CommitBind(const UsdPrim &prim, const TfToken &name,
                    RigExecBakedInput<T> *input, const SdfPathVector &walk);

    /// Binds \p name as a per-frame input, registering it for overrides and
    /// for invalidation. Resolve and commit together, for the call sites
    /// that have no reason to separate them.
    template <class T>
    RigExecBakedInput<T> Bind(const UsdPrim &prim, const TfToken &name,
                              T fallback, bool sourceValue = false);
};

inline void
RigExecBakedBuildContext::FoldShape(const UsdPrim &prim, const TfToken &name)
{
    if (prim) {
        const SdfPath path = prim.GetPath().AppendProperty(name);
        program->rebuild.insert(path);
        program->shapeFolded.insert(path);
        program->named.insert(path);
        program->prims.insert(prim.GetPath());
    }
}

template <class T>
RigExecBakedInput<T>
RigExecBakedBuildContext::Bind(const UsdPrim &prim, const TfToken &name,
                               T fallback, bool sourceValue)
{
    SdfPathVector walk;
    RigExecBakedInput<T> input =
        ResolveBind(prim, name, fallback, &walk, sourceValue);
    CommitBind(prim, name, &input, walk);
    return input;
}

/// Whether \p name is an animation channel a bake reads per frame rather
/// than folding: an `avars:` property (the schema's namespace for what an
/// animator authors) other than the transform channels, which a value edit
/// patches in place (RigExecProgramAvarPatch), and the channels that choose
/// how a frame composes, which are structure.
inline bool
RigExecBakedIsLiveChannel(const TfToken &name)
{
    if (!TfStringStartsWith(name, SchemaNamePrefix::avars)) {
        return false;
    }
    const auto &T = *_bakedImplTokens;
    const TfToken *const notLive[] = {&T.avarsTx,
                                      &T.avarsTy,
                                      &T.avarsTz,
                                      &T.avarsSx,
                                      &T.avarsSy,
                                      &T.avarsSz,
                                      &T.avarsRx,
                                      &T.avarsRy,
                                      &T.avarsRz,
                                      &T.avarsRspin,
                                      &T.avarsUnitScaleFactor,
                                      &T.avarsRotationOrder,
                                      &T.avarsRotationSign,
                                      &T.avarsDefaultSpace};
    for (const TfToken *channel : notLive) {
        if (name == *channel) {
            return false;
        }
    }
    return true;
}

template <class T>
RigExecBakedInput<T>
RigExecBakedBuildContext::ResolveBind(const UsdPrim &prim, const TfToken &name,
                                      T fallback, SdfPathVector *walk,
                                      bool sourceValue) const
{
    RigExecBakedInput<T> input = RigExecBakedBindInput(
        prim, name, fallback, capture, chainTargets, walk);
    // An input that reads an animation channel -- on its head or on any hop
    // of its connection walk -- is read per frame from the start, the way a
    // keyed one is, instead of being folded. Switches, spaces and dials are
    // edited interactively, a folded value moves only by rebaking the
    // program, and every consumer already handles an input that is varying
    // at bind (a keyed channel is one). Decided here, before anything reads
    // `varying` or copies the constant, so no consumer precomputes from a
    // value that can move.
    if (!input.varying && input.head) {
        bool live = RigExecBakedIsLiveChannel(input.head.GetName());
        for (size_t i = 0; walk && !live && i < walk->size(); ++i) {
            const SdfPath &hop = (*walk)[i];
            live = hop.IsPropertyPath() &&
                   RigExecBakedIsLiveChannel(hop.GetNameToken());
        }
        if (live) {
            input.varying = true;
            input.query = UsdAttributeQuery();
            input.resolvedAttr = input.head;
            return input;
        }
    }
    // Only opted-in direct numeric arguments sample ordinary source edits.
    if constexpr (std::is_same_v<T, double> || std::is_same_v<T, GfVec3d>) {
        if (sourceValue && !input.varying && input.head &&
            input.head.GetTypeName().GetType() ==
                (std::is_same_v<T, double> ? SdfValueTypeNames->Double
                                           : SdfValueTypeNames->Double3)
                    .GetType() &&
            walk && walk->size() == 1 &&
            walk->front() == input.head.GetPath() &&
            !input.head.ValueMightBeTimeVarying() &&
            input.head.GetNumTimeSamples() == 0 && !input.head.HasSpline()) {
            input.sourceBacked = true;
            input.sourceFallback = fallback;
            input.query = UsdAttributeQuery(input.head);
        }
    }
    // A selection that moves with the time code is read the long way,
    // through this generation's resolved inputs, rather than through a
    // query pinned to the wrong attribute.
    if (input.varying && input.query.IsValid()) {
        bool viaChain = false, varying = false;
        UsdAttribute atProbe;
        RigExecBakedClassifyInput<T>(prim.GetAttribute(name), probe,
                                     chainTargets, &viaChain, &varying,
                                     &atProbe);
        if (atProbe.GetPath() != input.query.GetAttribute().GetPath()) {
            input.query = UsdAttributeQuery();
            input.resolvedAttr = prim.GetAttribute(name);
        }
    }
    // Splines read via resolved inputs: a pinned query misses spline edits.
    if (input.varying && input.query.IsValid() &&
        input.query.GetAttribute().HasSpline()) {
        input.query = UsdAttributeQuery();
        input.resolvedAttr = prim.GetAttribute(name);
    }
    return input;
}

template <class T>
void
RigExecBakedBuildContext::CommitBind(const UsdPrim &prim, const TfToken &name,
                                     RigExecBakedInput<T> *input,
                                     const SdfPathVector &walk)
{
    RigExecBakedProgramSink sink{program};
    RigExecBakedRecordBind(&sink, prim, name, input, walk);
}

// The per-domain halves of Build and Run.
// Build calls the two builders in program order and Run calls the two
// executors in program order; each pair lives in one file so that a domain's
// bake and its frame path are read side by side.

/// Bakes the interleaved solver/constraint walk into \p ctx's program.
void RigExecBakedBuildWalk(RigExecBakedBuildContext *ctx,
                           const std::vector<RigExecBakedWalkEntry> &walk);

/// Bakes the geometry chains and their derived targets into \p ctx's program.
void
RigExecBakedBuildGeometry(RigExecBakedBuildContext *ctx,
                          const std::vector<RigExecBakedChainSpec> &chains);

/// Binds source and namespace parent anchors to typed provider compose outputs.
bool RigExecBakedBindSpaceSwitchVersions(RigExecBakedProgramImpl *program,
                                         std::string *error = nullptr);
void RigExecBakedRunSpaceCheckpoint(RigExecBakedProgramImpl *program,
                                    RigExecBakedStep *step);

/// Appends the pose half of the program in program order: the compose
/// subtrees, then one Solve and one commit per walk entry, then the
/// rest->pose matrices and the phased-read finals.
void RigExecBakedBuildPoseSteps(RigExecBakedProgramImpl *program);

/// Appends the geometry half: per revision an InfluenceFold, a
/// RevisionStatic, its chunks and a RevisionFuse, then the chain's status
/// sweep, then its derived targets.
void RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program);
/// The op nodes' steps (bakedOpNodes.cpp), after the geometry steps: what
/// RigExecBakedProgramTesting::SetOpInjector hands, compiled for the
/// schedule, with a run of it. False with the reason when they cannot run.
bool RigExecBakedBuildOpSteps(RigExecBakedProgramImpl *program,
                              std::string *error);
/// Whether \p kind is an op step.
inline bool
RigExecBakedIsOpStep(RigExecBakedStepKind kind)
{
    return kind == RigExecBakedStepKind::OpNode ||
           kind == RigExecBakedStepKind::OpChainHead ||
           kind == RigExecBakedStepKind::OpChainRange;
}
/// Runs op step \p step; on failure marks it failed with the reason.
void RigExecBakedRunOpStep(RigExecBakedProgramImpl *program,
                           RigExecBakedStep *step);
/// The op nodes' named outputs into \p pose's opOutputs.
void RigExecBakedPublishOpOutputs(const RigExecBakedProgramImpl &program,
                                  RigExecRigPose *pose);
/// A clone's own run of the source's op nodes, pointed at the clone.
bool RigExecBakedCloneOpState(const RigExecBakedProgramImpl &source,
                              RigExecBakedProgramImpl *clone,
                              std::string *error);
/// Op value \p slot's version in \p domain (OpValue or OpGroupValue).
bool RigExecBakedOpValueVersion(const RigExecBakedProgramImpl &program,
                                RigExecBakedSlotDomain domain, uint32_t slot,
                                uint64_t *version);

/// Runs one pose step. Never touches the pose: everything it produces goes
/// into \p step.
void RigExecBakedRunPoseStep(RigExecBakedProgramImpl *program,
                             RigExecBakedStep *step, UsdTimeCode time);

/// Runs one geometry step, under the same rule.
void RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                                 RigExecBakedStep *step, UsdTimeCode time);

/// The matrix \p record holds this run, rest -> the recorded frame, into
/// \p matrix. False exactly where the walk records nothing: a constraint's
/// exit records no frame (`recordAfter`), or records targets[0] alone
/// (`recordEveryTarget`) and this is another target; a solver published no
/// element for the joint (`present`); or the frame is unusable or does not
/// decompose. \p matrix is written either way.
bool RigExecBakedEvalFrameRecord(const RigExecBakedProgramImpl &program,
                                 const RigExecBakedFrameRecord &record,
                                 GfMatrix4d *matrix);

/// The points \p binding resolves to this run: its first candidate whose
/// chain read a base, as `*points` / `*count`. False for the tail, and then
/// the reader reads the resolved input. Valid until the candidate's chain
/// next runs.
bool RigExecBakedResolvePoints(const RigExecBakedProgramImpl &program,
                               const RigExecBakedPointsBinding &binding,
                               const GfVec3f **points, size_t *count);

/// Sets \p revision's `revisionInputs` to \p resolved overlaid with what each
/// of its point bindings resolves to, and appends the "resolved to nothing"
/// line for each tail that asks for one. The live and frozen assembles both
/// build their overlay here. Does nothing for a revision with no phases.
void
RigExecBakedOverlayPointReads(RigExecBakedProgramImpl *program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const RigExecResolvedInputs &resolved,
                              std::vector<std::string> *diagnostics);

/// \p revision's packet from its path leaves, for an operation they
/// assemble (`leaves.decl.assembles`): the provider values as the stage
/// assembly takes them, the phase overlay (RigExecBakedOverlayPointReads
/// over the generation's resolved inputs), the blend gather from leaves --
/// which writes each dense sample's `lastPoints`, as the stage gather did --
/// and RigExecAssembleFromLeaves. \p basePoints are the chain's authored
/// base for a chain revision and its final points for a derived one. Reads
/// no stage: the live RevisionStatic and Derived bodies and the frozen
/// RevisionStatic all assemble here. \p missing: see
/// RigExecRevisionLeafView. \p layoutPointCount is the owning chain's raw
/// base cardinality for sparse layouts; the default uses basePointCount.
RigExecMoverParameters
RigExecBakedAssembleFromLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               const GfVec3f *basePoints, size_t basePointCount,
                               std::vector<std::string> *diagnostics,
                               std::vector<std::string> *missing = nullptr,
                               size_t layoutPointCount = size_t(-1),
                               RigExecExternalRevision *external = nullptr);

/// Appends the final and base VolumePlacement op step of each volume
/// provider slot, then one weight packet op step per weight object in the
/// table's dependency order -- all of it between the pose half and the
/// geometry half.
void RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program);

/// Replaces \p frames (pose.weightFrames) with one entry per placedVolumes
/// slot, from volumePlacement: the dynamic walk's key set, for live and
/// frozen runs alike, whichever placement steps this run's closure held.
void RigExecBakedPublishVolumePlacements(const RigExecBakedProgramImpl &program,
                                         std::map<SdfPath, GfMatrix4d> *frames);

// Build-only declaration sink; resolved reads are never persistent step state.
struct RigExecBakedDependencySink {
    RigExecBakedStep *step;
    bool resolvedReads = false;
};
std::vector<char>
RigExecBakedResolvedReaders(const RigExecBakedProgramImpl &program);

/// Every per-frame input one weight object's step reads.
void RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedDependencySink *sink);

/// Resets the per-run DELTAS a geometry step would have written, for a run
/// that skipped it.
///
/// A revision's state is two kinds of thing: values that outlive a run it
/// sat out -- the influence table, the packet, the points, the status -- and
/// flags that say how this run's values differ from the last run's. The
/// values are exactly what a skipped step is keeping; the flags are
/// statements about a comparison that was never made, and a later step
/// reading one would be told a change happened this run when it happened
/// during the last one. A skipped step compared nothing and therefore found
/// nothing, which is what this writes.
void RigExecBakedSkipGeometryStep(RigExecBakedProgramImpl *program,
                                  RigExecBakedStep *step);

/// Resolves the rest chain of slots [\p begin, \p end) from the bound
/// channels of RigExecBakedProgramImpl::ladders, in slot order, as their
/// leaves were last sampled: `restM`, `restFrames`, `restPts` and
/// `restRoundTrip`. A parent outside the range must be composed already.
///
/// ONE definition, called from Build (once, over every slot, at the capture
/// time) and by the RestCompose head op. computations.cpp resolves the same
/// computations per provider per evaluation; the frame round trips exec
/// performs between them are reproduced, not simplified away, because a
/// deep chain drifts without them.
///
/// \p trackMoves compares each slot's `restM` with `lastRestM` and, where it
/// moved, sets `restChanged`, appends to `restMoved` and updates
/// `lastRestM`. Build passes false and seeds the comparison buffers itself.
void RigExecBakedComposeRestRange(RigExecBakedProgramImpl *program, int begin,
                                  int end, bool trackMoves);

/// The default-space ladder of slots [\p begin, \p end), as above:
/// `posedAuthored`, `posedAuthoredM`, `selfD`, `defaultRoundTrip`,
/// `parentDinv` and `rotOrder`. Reads the range's rests and its parents'
/// rests and ladders, which must be composed already. \p trackMoves
/// compares `selfD`, `parentDinv`, `posedAuthored`, `posedAuthoredM` and
/// `rotOrder` with their `last` buffers and records a move in
/// `ladderChanged` and `ladderMoved`.
void RigExecBakedComposeLadderRange(RigExecBakedProgramImpl *program, int begin,
                                    int end, bool trackMoves);

/// One RestCompose (part 0) and one LadderCompose (part 1) head op per
/// compose group, appended to `steps`, with their declared reads: the
/// parent rests and ladders outside the group, the group's own rests for
/// the ladder, and the PropertyVersion ids the chain-routed channels'
/// reader walks declare. The caller sorts and validates the tier.
void RigExecBakedBuildRestSteps(RigExecBakedProgramImpl *program);

/// Fills each rest and ladder op's `bindingLeaves` and `varyingLeaves`
/// from the ladder channels' leaves. Called after the last
/// RigExecBakedNumberLeaves of Build.
void RigExecBakedNoteRestLeaves(RigExecBakedProgramImpl *program);

/// The pose half of the prologue: the bound inputs, once per run.
void RigExecBakedBuildAvarSteps(RigExecBakedProgramImpl *program);
void RigExecBakedRunAvarOp(RigExecBakedProgramImpl *program,
                           RigExecBakedStep *step);

/// Fills avarBindingBegin and avarConstantBindingBegin from the two binding
/// lists, which must be sorted by flat slot (verified). Build calls it once
/// the lists are complete; a program assembled by hand calls it after
/// filling them.
void RigExecBakedIndexAvarBindings(RigExecBakedProgramImpl *program);

/// Provider slot \p provider's bindings in the list \p begin indexes, as
/// [first, second) list positions: exactly the entries whose slot / 11 is
/// \p provider, in list order. Empty for a negative provider or one past
/// every binding.
inline std::pair<size_t, size_t>
RigExecBakedAvarBindingRange(const std::vector<uint32_t> &begin,
                             int64_t provider)
{
    if (provider < 0 || size_t(provider) + 1 >= begin.size()) {
        return {0, 0};
    }
    return {begin[size_t(provider)], begin[size_t(provider) + 1]};
}

/// The solver half of the prologue: every ribbon's live driver-curve points,
/// read off the stage and compared with the last run's.
///
/// A source in the re-execution sense -- it reads outside the program and its
/// comparison is what dirties the solvers that read it -- but a prologue
/// pass rather than a step, for the same reason the bound inputs are one:
/// it takes the stage's locks, and a step body may not.
void RigExecBakedRunSolverSources(RigExecBakedProgramImpl *program,
                                  UsdTimeCode time);

/// The geometry half of the prologue: every chain's and derived target's
/// authored base, the point-count-moved reset, the node-creation accounting,
/// and the adoption of each fixed skin revision's layout handle (topology,
/// topologyResolved and the partition's adoption) wherever a base reads.
/// Also samples the weight objects' and the revisions' path leaves
/// (RigExecBakedSamplePathLeaves; \p all re-reads every one).
void RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                     UsdTimeCode time, RigExecRigPose *pose,
                                     bool all);

/// Hands the revisions whose retained lattice binds are equal one shared,
/// immutable bind (RigExecLatticeBindSharing). The thread that runs the
/// program calls it while no step runs, before dispatch and again after
/// the join, so a freeze copies binds already shared: native and frozen
/// alike.
void RigExecBakedShareLatticeBinds(RigExecBakedProgramImpl *program);

/// SkinTopology op \p r's revision: entry r of `revisionIndex`, then of
/// `derivedIndex` past those; null past both.
RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(RigExecBakedProgramImpl *program, size_t r);
const RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(const RigExecBakedProgramImpl &program, size_t r);

/// Declares and binds the layout leaves of every `skinTopologyFixed`
/// revision and appends one SkinTopology head op per such revision
/// (`object` = its RigExecBakedLayoutRevision index), writing SkinTopology
/// slot `object` and reading only its leaves. Build, owning thread.
void RigExecBakedBuildLayoutSteps(RigExecBakedProgramImpl *program);

/// Appends SkinTopology[r] to the `reads` of the RevisionStatic,
/// RevisionChunk and RevisionFuse steps of every revision a SkinTopology op
/// serves. Build, once the geometry steps exist.
void RigExecBakedDeclareLayoutReads(RigExecBakedProgramImpl *program);

/// Re-reads \p revision's layout leaves at \p time through the generation's
/// resolved inputs, and re-asks RigExecSkinLayoutWeightsAreFixed (the
/// topology half stands from Build), when one of these
/// says they can have moved since the last sample:
///  1. the first sample, \p all (a forced run), or a moved program stamp;
///  2. a leaf's `mustSample` byte: a value edit or a routed override reached
///     one of the three paths (`leafByPath`);
///  3. the overlay entry at one of the three paths differs from the one the
///     last sample saw (an override placed, moved or lifted there).
/// The time alone moves nothing: a fixed layout reads the same at every
/// time, as the evaluator's cache holds it across frames. A layout that is
/// not fixed is not read. Sets each leaf's `changed` byte and
/// `layoutFixedChanged`. Owning thread, prologue only.
void
RigExecBakedSampleLayoutLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               UsdTimeCode time, bool all);

/// Whether \p revision's skin layout is fixed, as the next sample answers
/// it: false while its topology half (`layoutTopologyFixed`, Build) is;
/// else GeomRevision::layoutFixed while that answer is current (its weights
/// leaf's VarianceStale false), else RigExecSkinLayoutWeightsAreFixed asked
/// again, without storing it. Owning thread.
bool RigExecBakedLayoutFixedNow(
    const RigExecBakedProgramImpl &program,
    const RigExecBakedProgramImpl::GeomRevision &revision);

/// The SkinTopology op's body over \p revision's leaves: a null handle while
/// the layout is not fixed, else RigExecBuildSkinTopology of the leaves,
/// handed back as `layoutCandidate` when that holds the same layout. Writes
/// `layoutHandle` and `layoutCandidate` and nothing else. Pure.
void RigExecBakedRunLayoutOp(const RigExecBakedProgramImpl &,
                             RigExecBakedProgramImpl::GeomRevision *revision);

/// The pose half of the epilogue: the joint and control publication, the
/// solver guides and the property-domain results.
///
/// Returns false where the walk returns false from the same line: a
/// joint with a valid, non-degenerate final frame whose rest or frame is
/// not usable needs exec.
bool RigExecBakedPublishPose(RigExecBakedProgramImpl *program,
                             RigExecRigPose *pose);

/// Publishes each geometry chain and derived target in chain order.
void RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                                 RigExecRigPose *pose);

/// What ChainStatus publishes for \p chain: its points, or in shader mode
/// its program (RigExecGpuDeformPublication).
VtValue
RigExecBakedChainPublication(const RigExecBakedProgramImpl::GeomChain &chain);

/// Chooses the point chains that run as one GPU program, from the mode
/// RIGEXEC_GPU_DEFORM names. A chain qualifies when every revision has a
/// program op and no step outside the chain reads its points, versions or
/// statuses; its derived steps run at the publication boundary. Build
/// calls it once every read is declared.
void RigExecBakedSelectGpuChains(RigExecBakedProgramImpl *program);

/// The publication boundary of GPU chains: deforms every chain ChainStatus
/// translated this generation on the registered runner (or its CPU kernels
/// where none is registered or the device refused), publishes the points
/// as ChainStatus would, and runs the chain's derived steps. Owner thread,
/// after the graph has joined.
void RigExecBakedResolveGpuChains(RigExecBakedProgramImpl *program,
                                  UsdTimeCode time);

/// Builds RigExecBakedProgramImpl::epilogue from the steps as they stand,
/// keeping `verify` and `mismatches`. Owner thread.
void RigExecBakedIndexEpilogue(RigExecBakedProgramImpl *program);
/// Indexes the epilogue unless it already matches the steps and solvers.
void RigExecBakedEnsureEpilogueIndex(RigExecBakedProgramImpl *program);
/// Notes the output of every op the last run ran: a body empties its step's
/// output first (BeginRun) and is the only run-time writer of it. Owner
/// thread, after the join, failed run or not.
void RigExecBakedFoldHeldSteps(RigExecBakedProgramImpl *program);
/// Under `epilogue.verify`: the Build lists against a sweep of the steps.
void RigExecBakedVerifyEpilogueIndex(RigExecBakedProgramImpl *program);

/// The step-output blocks of the epilogue, each in its fixed order:
/// property revisions by (chain, part) with their lines, then the walk's
/// other non-geometry steps with their lines, then the pose interpolators'
/// diagnostics, then the geometry steps' diagnostics; each in step order.
enum class RigExecBakedStepLines { Property, Walk, Interpolator, Geometry };
/// Appends \p block from the held steps; under `epilogue.verify` also
/// builds it from every step and counts and TF_VERIFYs a difference.
void RigExecBakedAppendStepLines(RigExecBakedProgramImpl *program,
                                 RigExecBakedStepLines block,
                                 std::vector<std::string> *out);
/// Appends the lines of the last run's lowest failed op
/// (RigExecOpExecution::firstFailed), the failure every evaluator reports.
void RigExecBakedAppendFailedOpLines(const RigExecBakedProgramImpl &program,
                                     std::vector<std::string> *out);

/// Construct the token tables the step bodies of each translation unit use,
/// on the calling thread. Build calls them, with RigExecRevisionKernelTouchTokens
/// and RigExecWeightPacketsTouchTokens, so a table's lazy construction --
/// which builds tokens from text -- never runs first on a worker.
void RigExecBakedGeometryTouchTokens();
void RigExecBakedWeightsTouchTokens();
void RigExecFrozenGeometryTouchTokens();

/// RIGEXEC_BAKED_CHUNK_VERTS and RIGEXEC_BAKED_MAX_CHUNKS, read by Build
/// into chunkVertexTarget and chunkCap.
size_t RigExecBakedChunkVertexTargetFromEnvironment();
size_t RigExecBakedChunkCapFromEnvironment();

/// Whether \p revision applies this run, from what RevisionStatic and the fold
/// published (a range-pipelined revision's acceptance is never Deferred): the
/// one decision its range steps and its join share.
inline bool
RigExecBakedRevisionApplies(
    const RigExecBakedProgramImpl::GeomRevision &revision)
{
    const bool packetValid =
        revision.parameters.valid &&
        (revision.op != RigExecRevisionOp::Skin || revision.influencesValid);
    return packetValid && revision.status.AllowsApply() &&
           revision.acceptance == RigExecRevisionAcceptance::Applies;
}
/// RIGEXEC_BAKED_RANGE_CHAINS; read once at Build (bakedGeometry.cpp).
bool RigExecBakedRangeChainsFromEnvironment();
/// The RIGEXEC_VERIFY_RANGE_CHAINS judge (bakedGeometry.cpp): owner thread,
/// after the region; returns this run's mismatches.
size_t RigExecBakedVerifyRangeChains(RigExecBakedProgramImpl *program);

/// Group \p g of point version \p version of \p chain (0 the base; v what
/// revision v - 1 left): the ref its writer published; the base group when a
/// set-aside revision lies between g's writer and \p version.
const RigExecPointsRef<GfVec3f> &
RigExecBakedGroupAt(const RigExecBakedProgramImpl::GeomChain &chain,
                    size_t version, size_t g);
/// That group's content id (writer slot and version).
RigExecGroupSource RigExecBakedGroupSourceAt(const RigExecBakedProgramImpl &B,
                                             int chain, size_t version,
                                             size_t g);
/// Declares the read of group \p g of version \p version of chain \p chain:
/// ChainBase for the base (or past a set-aside revision), else the writer's
/// RevisionOut slot.
void RigExecBakedDeclareGroupRead(const RigExecBakedProgramImpl &B, int chain,
                                  size_t version, size_t g,
                                  std::vector<RigExecBakedSlotRange> *reads);
/// The RevisionOut slot of group \p g as revision \p id publishes it (Range:
/// chunkBase + g; Whole: chunkBase + chunks.size() + g), or -1 when unwritten.
int RigExecBakedGroupSlot(const RigExecBakedProgramImpl &B, int id, size_t g);
/// Publishes \p count points at \p elements as group \p g of revision
/// \p output.object (an id of revisionIndex), which writes that group: the
/// entering group passed through when they hold its bits, else a copy in
/// one of the group's own buffers; the group's version moves only when its
/// bits do. False with the reason for a revision or group it lacks or a
/// count other than the group's.
bool RigExecBakedStoreChainGroup(RigExecBakedProgramImpl &B,
                                 const eval::OpHostOutput &output, uint32_t g,
                                 const void *elements, size_t count,
                                 std::string *error);
/// Whether every role pin holds for this run's sampled values. Owning thread
/// after RigExecBakedRunGeometryPrologue; the frozen worker after
/// _FrozenPrologue. Reach from *interactiveOverrides and upstream against the
/// hops; never reads overrideReached; no TfToken from text, no SdfPath text,
/// no TF_ diagnostics.
bool RigExecBakedRolesStand(const RigExecBakedProgramImpl &program);
/// Whether every solver count Build fixed (a ribbon's rigExec:sampleCount)
/// equals this run's sampled value; RigExecBakedRolesStand asks it.
bool RigExecBakedSolverCountsStand(const RigExecBakedProgramImpl &program);
/// The points of version \p version as one contiguous array: in place for the
/// base, a Legacy buffer or one-buffer slices, else gathered into the
/// reader's \p scratch. Valid until the calling step returns.
void RigExecBakedVersionPoints(const RigExecBakedProgramImpl::GeomChain &chain,
                               size_t version, std::vector<GfVec3f> *scratch,
                               const GfVec3f **points, size_t *count);
size_t RigExecBakedGroupVertexTargetFromEnvironment();
size_t RigExecBakedGroupCapFromEnvironment();
bool RigExecBakedGroupGatesFromEnvironment();
/// VtFloatArray form of RigExecBakedNoteFloats: replaces \p field by
/// \p scratch's values (assign, never a detach copy) and bumps \p version
/// exactly when the bytes differ.
void RigExecBakedNoteFloats(VtFloatArray *field, std::vector<float> *scratch,
                            uint64_t *version);

/// Cuts \p revision's vertices into chunks, from \p indices and
/// \p elementSize, and records both as the partition's arrays (the indices
/// only when the cut has more than one chunk). Build only: each chunk step
/// declares its reads from its key.
///
/// Contiguous ranges of \p program's chunkVertexTarget vertices, capped at
/// its chunkCap (the range grows to meet the cap); each range's
/// key is the union of its vertices' influence positions; adjacent ranges
/// merge while they stay under the vertex cap and one key contains the
/// other, because a range that waits for a superset of another's joints is
/// not waiting any longer for holding both. Vertex order is never permuted.
void
RigExecBakedPartitionRevision(const RigExecBakedProgramImpl &program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const VtIntArray &indices, int elementSize);

/// Makes \p revision's adopted `topology` the handle its chunk keys
/// describe when it carries the partition's arrays (the same `jointIndices`
/// and element size; the weights may differ). Otherwise the partition is
/// left as it is, and `partitionStale` runs the revision whole until a
/// handle with those arrays is adopted again or the program is rebuilt.
/// Nothing for an unchunked revision or a null handle. Called where the
/// handle is adopted: the live geometry prologue and the frozen worker.
void RigExecBakedAdoptRevisionLayout(RigExecBakedProgramImpl::GeomRevision *);
void RigExecBakedRunChainInputs(RigExecBakedProgramImpl *, RigExecBakedStep *);
void
RigExecBakedPrepareDerivedBase(RigExecBakedProgramImpl::GeomChain::Derived *,
                               RigExecBakedStep *);
void
RigExecBakedAdoptPartition(RigExecBakedProgramImpl::GeomRevision *revision);

/// The partition statistics of every skin revision, for the schedule report.
///
/// Per revision: how many chunks, how many vertices each holds, the key
/// sizes, how many chunks depend on half the influences or more, and each
/// chunk's influence positions.
std::string RigExecBakedGeometryReport(const RigExecBakedProgramImpl &program);

/// Records \p input against \p step: what a frame can move it with.
///
/// Three answers, and each is a different way a value can arrive: it is a
/// function of time (a keyframe), it resolves through the generation's
/// resolved inputs every frame (a property chain writes it), or an
/// interactive override can be placed on it. Anything else was folded at
/// bake and cannot move without a rebuild.
///
/// It is here rather than beside one domain's step builders because all
/// three domains declare inputs and a domain that declared them its own way
/// would be a step the cone cannot dirty.
template <class T>
inline void
RigExecBakedNoteInput(const RigExecBakedInput<T> &input,
                      RigExecBakedDependencySink *sink)
{
    RigExecBakedStep *step = sink->step;
    step->varyingInputs = step->varyingInputs || input.varying;
    sink->resolvedReads = sink->resolvedReads || bool(input.resolvedAttr);
    if (input.overrideIndex >= 0) {
        step->overrideInputs.push_back(input.overrideIndex);
    }
    if (input.walk >= 0) {
        step->readerWalks.push_back(input.walk);
        if (input.leaf >= 0)
            step->walkedBindingLeaves.emplace_back(
                RigExecBakedLeafTraits<T>::type, uint32_t(input.leaf));
    }
}

/// Bakes the weight object at \p path, and everything it composes, into
/// \p ctx's table; returns its index, or -1 when there is nothing there.
///
/// Memoized on RigExecBakedProgramImpl::weightIndex, so a weight object ten
/// movers bind is baked once and every consumer gets the same index; the
/// same map carries an under-way marker, so a composition cycle is refused
/// rather than recursed into. Binds every readable field through the build
/// context, which is what keeps the invalidation index and interactive-
/// override placement honest with no further code.
int RigExecBakedBakeWeightObject(RigExecBakedBuildContext *ctx,
                                 const SdfPath &path);

/// The numeric time Build probes input selections at: the stage's start
/// time code when it authors a time-code range, else 0. Never Default.
UsdTimeCode RigExecBakedProbeTime(const UsdStageRefPtr &stage);

/// What the weight field resolution needs beyond a
/// weight object's composition and its scalar reads: the answers the field
/// takes from the stage alone. Read at one time.
/// Fills \p facts for the weight object at \p path, reading the stage as
/// the weight field does at \p time.
inline void
RigExecBakedDescribeWeightFieldFacts(const RigExecRigEvaluator &evaluator,
                                     const SdfPath &path, UsdTimeCode time,
                                     RigExecWeightFieldFacts *facts)
{
    RigExecBakedProgram::DescribeWeightFieldFacts(evaluator, path, time, facts);
}

/// A weight object the weight field reaches that no WeightPacket step bakes: a
/// constraint's or a property mover's envelope, or an object one composes
/// (the constraint step's envelope arm, bakedPose.cpp). Composed as
/// RigExecBakedBakeWeightObject composes, and registered nowhere: the
/// weight field reads its scalars through the
/// generation's resolved inputs (_ResolvedRead), so each read is kept as
/// the attribute that walk starts from plus the field's fallback.
struct RigExecBakedEnvelopeObject {
    struct Read {
        UsdAttribute head; ///< invalid when the prim has no such attribute
        float fallback = 0.0f;
    };
    SdfPath path;
    TfToken type, representation, rangePolicy, combineMode;
    std::vector<float> values;
    std::vector<int> indices;
    /// rigExec:baseWeight's first target and rigExec:inputWeights in
    /// authored order, as indices into the field's table: below
    /// program.weightObjects.size() a step-backed object, else
    /// weightObjects.size() plus an index into the envelope list.
    int base = -1;
    std::vector<int> inputs;
    Read defaultWeight, driver, scale, bias, strength, invert;
};

/// Composes every weight object a constraint envelope (a weight object and
/// no points target) or a property mover's rigExec:weightObject[0] reaches
/// through rigExec:baseWeight and rigExec:inputWeights that \p program
/// does not already bake, depth first, so the list is in dependency order.
/// \p index maps each composed path to its field-table index. False with
/// the reason on what the weight field could not resolve structurally -- a
/// missing prim, a type that is no weight object or is volumetric, an
/// unknown token, a second base, a cycle -- which Compile rejects first.
inline bool
RigExecBakedComposeEnvelopeObjects(
    const RigExecRigEvaluator &evaluator,
    const RigExecBakedProgramImpl &program,
    std::vector<RigExecBakedEnvelopeObject> *objects,
    std::map<SdfPath, int> *index, std::string *error)
{
    return RigExecBakedProgram::ComposeEnvelopeObjects(evaluator, program,
                                                       objects, index, error);
}

/// One of the evaluator's property chains as a bake states it: the
/// target, which runChain arm its type selects, and per revision what
/// _EvaluatePropertyChains binds and reads on the mover
/// (rigEvaluatorProperties.cpp). The reads stay attributes; the bake turns
/// each into a walk.
struct RigExecBakedPropertyChainDesc {
    /// The runChain arm: the target's type name is float, double or
    /// matrix4d, else the GfVec3f-backed arm.
    enum class ValueType : uint8_t { Float = 0, Double, Matrix4d, Vec3f };
    struct Revision {
        SdfPath mover;
        /// rigExec:operation as _ReadOperation reads it (the attribute's
        /// own value at Default), parsed; false when it does not parse.
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        /// The inputs _BindInput binds; invalid when the mover has none.
        UsdAttribute enabled, defaultWeight, value, minimum, maximum;
        /// rigExec:weightObject's first target; empty when it has none.
        SdfPath weightObject;
        int weightField = -1;
        /// inputs:keys and inputs:tangents at Default, read only for a
        /// float or double chain whose operation is curve (the only arm
        /// that reads them).
        std::vector<GfVec2f> keys;
        bool hasTangentsAttr = false;
        std::vector<GfVec2f> tangents;
        /// The path operation's curve points and settings, for a float or
        /// double chain whose operation is path.
        UsdAttribute pathPoints;
        RigExecPathSettings path;
    };
    /// A RigExecPhasedConnection on this chain.
    struct Phased {
        SdfPath consumer;
        /// Float or Double when the consumer has that type name, else the
        /// chain's own type: RigExecPhasedConsumerValue converts between
        /// float and double and passes every other value through.
        ValueType consumerType = ValueType::Float;
        size_t applied = 0;
        /// RigExecPhasedConnection::hops: the consumer, then each attribute
        /// its walk passes before the target.
        SdfPathVector hops;
    };
    SdfPath target;
    UsdAttribute targetAttr;
    ValueType valueType = ValueType::Float;
    std::vector<Revision> revisions;
    std::vector<Phased> phased;
};

/// Restates every property chain of \p evaluator's standing compile, in
/// _propertyChainOrder, with its phased connections in
/// _phasedConnections order. False with the reason when a chain cannot be
/// stated for a file read at one time: a curve revision whose keys or
/// tangents are animated or connected, or (refused by Compile first) a
/// missing target or mover.
inline bool
RigExecBakedDescribePropertyChains(
    const RigExecRigEvaluator &evaluator,
    std::vector<RigExecBakedPropertyChainDesc> *chains, std::string *error)
{
    return RigExecBakedProgram::DescribePropertyChains(evaluator, chains,
                                                       error);
}

/// Records, per pose step, which of its baked inputs a run can move.
///
/// A Solve and a Constraint read their own parameters off the stage every
/// frame -- weights, offsets, axis masks, world-up vectors -- so neither is
/// a pure function of its declared slots. Build walks those inputs once here
/// and leaves each step saying whether any of them varies with time and
/// which override indices reach it, which is what lets the dirty set name
/// the steps a keyframe or a standing drag can have moved instead of
/// re-running the walk for either.
void RigExecBakedDeclareInputDependencies(RigExecBakedProgramImpl *program);

struct RigExecBakedLadderTables {
    std::vector<GfMatrix4d> restM;
    std::vector<std::array<GfVec3d, 4>> restPts;
    std::vector<RigExecPointFrame> restFrames;
    std::vector<GfMatrix4d> selfD, parentDinv, posedD, parentSpaceM;
    std::vector<char> parentSpaceAuthored;
    std::vector<unsigned char> rotationSign;
    std::vector<TfToken> rotOrder;
    std::vector<GfMatrix4d> restRoundTrip, defaultRoundTrip;
    std::vector<char> posedAuthored;
    std::vector<GfMatrix4d> posedAuthoredM;
};

/// Shadows a run's mutable state for RIGEXEC_BAKED_VERIFY_CONES: Capture,
/// Restore, re-run forced, Compare values and diagnostics. Debug only.
///
/// Not compared (scratch, not answers):
///  * Build-only state: slot tables, input bindings, step graph,
///    clustering, cones.
///  * notice-only state: `valueEditSerial`, `editSerial`, the routing's
///    `connectedSources`.
///  * prologue output (once per generation, before either pass):
///    `avarsDisturbed`, `overridden`/`anyOverridden`/`folded`, `leaves`,
///    `routedOverrides`/`lastRoutedOverrides`, `leafSamples`, the geometry
///    prologue's sampler pools and routing stamps.
///  * cone bookkeeping: `closedSteps`, `closed`, `lastAvars`,
///    `lastOverridden`, `edited`/`anyEdited`, `lastHaveBase`, `lastTime`,
///    `runState`, `lastProgramStamp`, `spaceLeafRekey`.
///  * each chain's `spare`; only `result` is an answer.
///  * each revision's `deltasSpare`, and its `revisionInputs` (captured and
///    restored, not compared).
///  * run statistics, restored by RigExecBakedRunStatistics:
///    `lastClosedClusters`, `lastClosedSteps`, `spaceLeafKeys`, the
///    clustering's `lastRunTimed`, the closed-operation sets, the full-run
///    flag, cluster `readyUs`/`startUs`/`endUs`, step `startUs`/`endUs` and
///    memo/publication stamps.
///  * per-step `measured*` calibration accumulators.
///  * op-node run values in `opState`: the forced pass rewrites every one;
///    their versions are restored.
/// Invariant: every mutable field of RigExecBakedProgramImpl,
/// RigExecBakedStep, GeomChain, GeomRevision and GeomChunk is compared or
/// listed here.
struct RigExecBakedRunShadow {
    /// Copies everything a step reads or writes out of \p program.
    void Capture(const RigExecBakedProgramImpl &program);
    /// Puts it back, so that a second run starts where the first one did.
    void Restore(RigExecBakedProgramImpl *program) const;
    /// Appends one line per disagreement between this shadow and
    /// \p program's current state, and returns how many there were.
    size_t Compare(const RigExecBakedProgramImpl &program,
                   std::vector<std::string> *differences) const;

    std::vector<RigExecBakedPropertyValue> values;
    std::vector<char> versionValid, changed, chainValid, recordStoodAside;
    std::vector<VtValue> chainFinal, recordValues;
    std::map<SdfPath, VtValue> propertyResults;
    RigExecResolvedInputs resolvedInputs;
    bool hasResolvedInputs = false;
    std::vector<VtValue> headOverrides, lastHeadOverrides;
    std::vector<char> headOverrideMoved, readerWalkMoved, readerWalkChanged;

    RigExecBakedLadderTables tables;
    std::vector<GfMatrix4d> lastRestM, lastSelfD, lastParentDinv,
        lastPosedAuthoredM;
    std::vector<char> lastPosedAuthored;
    std::vector<GfMatrix4d> lastPosedD, lastParentSpaceM;
    std::vector<char> lastParentSpaceAuthored;
    std::vector<unsigned char> lastRotationSign;
    std::vector<TfToken> lastRotOrder;
    std::vector<char> restChanged, ladderChanged;
    std::vector<int> restMoved, ladderMoved;
    uint64_t opsRun = 0;

    struct StepState {
        std::vector<std::string> diagnostics, lines;
        RigExecBakedStepCounters counters;
    };
    struct ChunkState {
        std::vector<GfMatrix4d> transforms;
        std::vector<float> rows;
        std::vector<RigExecScaledDualQuat> palette;
        bool keyChanged = false, ok = false;
    };
    struct BlendSampleState {
        std::shared_ptr<const RigExecBlendSampleLayout> layout;
        std::vector<GfVec3f> lastPoints;
        bool layoutRefused = false;
        bool layoutKeyed = false;
        uint64_t layoutOffsetsVersion = 0;
        uint64_t layoutIndicesVersion = 0;
        size_t layoutPointCount = 0;
        uint64_t layoutBuilds = 0;
    };
    struct RevisionState {
        std::vector<std::vector<BlendSampleState>> blendSamples;
        std::vector<RigExecBlendChannel> computedBlendSamples;
        std::shared_ptr<const RigExecSkinTopology> topology, partitionTopology;
        bool topologyResolved = false;
        std::shared_ptr<const RigExecSkinTopology> layoutHandle,
            layoutCandidate;
        bool layoutFixed = false, layoutRan = false;
        std::vector<GfVec3f> output;
        std::vector<GfVec3f> stagingOutput, passedPoints;
        bool stagingFresh = false;
        uint64_t doneVersion = 0;
        std::vector<GfMatrix4d> packetInfluences, influences;
        RigExecResolvedInputs revisionInputs;
        std::vector<float> rows, envelope;
        std::vector<RigExecScaledDualQuat> palette;
        std::vector<ChunkState> chunks;
        RigExecMoverParameters parameters, lastParameters;
        RigExecExternalRevision external;
        VtVec3fArray lastAuxPoints;
        RigExecMoverStatus status, lastStatus;
        TfToken resultStatus;
        GfMatrix4d transform{1.0};
        size_t precedingCount = 0;
        int currentSource = -1;
        float defaultWeight = 0, lastDefaultWeight = 0;
        bool haveTransform = false, ran = false, executed = false,
             created = false;
        bool influencesValid = false, influencesChanged = false;
        bool staticDirty = false, partitionStale = false;
        bool layoutUsable = false, envelopeOk = false, fullStrength = false;
        RigExecRevisionAcceptance acceptance =
            RigExecRevisionAcceptance::Refuses;
        VtFloatArray publishedWeightValues;
        RigExecWeightPacket currentPhasePacket;
        bool weightFieldPublished = false;
        uint64_t envelopeVersion = 0, weightValuesVersion = 0;
        bool weightValuesHeld = false;
        uint64_t weightValuesPacketRevision = 0;
        size_t weightValuesCount = 0;
        std::vector<uint64_t> joinSeen;
        uint32_t rangeRefusals = 0;
        /// Copied as states (refs and own[] handles), never as bytes.
        std::vector<RigExecGroupState<GfVec3f>> groups;
        std::vector<RigExecGroupSource> groupIds;
        uint64_t deltasVersion = 0;
        bool restBaseHeld = false;
        uint64_t restBaseVersion = 0, layoutSerial = 0;
    };
    struct DerivedState {
        RevisionState revision;
        VtVec3fArray result, spare, lastBase;
        uint64_t resultVersion = 0;
        bool haveResult = false, haveBase = false, baseDirty = false;
        GfMatrix4d matrix{1.0};
        bool haveMatrix = false;
    };
    struct ChainState {
        std::vector<RevisionState> revisions;
        std::vector<DerivedState> derived;
        VtVec3fArray lastBase, result, spare, publishedInput;
        uint64_t inputVersion = 0, baseVersion = 0, resultVersion = 0;
        bool haveResult = false, haveBase = false, baseDirty = false,
             scheduleDirty = false;
        std::shared_ptr<const VtVec3fArray> baseOwner;
        std::vector<RigExecGroupState<GfVec3f>> baseGroups;
        std::vector<RigExecGroupSource> resultIds, spareIds;
    };
    struct SolverState {
        std::vector<RigExecPointFrame> outFrames;
        std::vector<char> outPresent;
        std::vector<int> fallbackSlots;
    };
    struct CommitState {
        std::vector<char> present, deltaOk;
        std::vector<RigExecPointFrame> frames, staged;
        std::vector<GfMatrix4d> deltas;
        std::vector<uint8_t> outcome;
        std::vector<RigExecConstraintSource> sources;
        bool abandoned = true;
        bool recordAfter = true, recordEveryTarget = true;
    };

    std::optional<RigExecOraclePublicationContext> oraclePublications;
    std::map<SdfPath, RigExecWeightReferenceContext> oracleWeightInputs;
    RigExecTypedValueStore providerValues{0};
    std::vector<GfMatrix4d> switchFrames;
    RigExecOpAdapterState opAdapter;
    /// Travels with opAdapter: its sourceWatchSerial names this watch's
    /// generation, so a restore keeps the kept keys and their watch a pair.
    RigExecBakedSourceWatch sourceWatch;
    /// Travels with opAdapter as well: its OpValue and OpGroupValue keys
    /// spell these op-node value versions.
    std::vector<uint64_t> opValueVersions, opGroupVersions;
    std::vector<std::string> chainContentKeys, packetContentKeys;
    RigExecOpExecution opExecution;
    std::vector<double> avars;
    std::vector<float> poseWeights;
    std::vector<RigExecWeightPacket> weightPackets;
    std::vector<GfMatrix4d> posedM, finalMatrix, baseMatrix;
    std::vector<RigExecPointFrame> base, fin;
    std::vector<RigExecPointFrameArray> aggregates;
    std::vector<SolverState> solvers;
    std::vector<CommitState> commits;
    std::vector<ChainState> chains;
    std::vector<StepState> steps;
    std::vector<GfMatrix4d> deltaValues;
    std::vector<char> deltaPresent;
    std::vector<RigExecBakedProgramImpl::WeightField> weightFields;
    std::vector<GfMatrix4d> volumePlacementBase;
    std::vector<GfMatrix4d> volumePlacement;
    std::vector<GfMatrix4d> frameMatrix;
    std::vector<char> frameMatrixValid;
    bool avarsDisturbed = false;
};

/// The run statistics of the generation that produced the pose.
///
/// RIGEXEC_BAKED_VERIFY_CONES runs the frame a second time, forced, and that
/// pass writes the same bookkeeping the first one did: how many clusters the
/// closure held, and the intervals the run report and the profiler trace
/// print. But a verification pass is not a generation -- it publishes
/// nothing -- so what an observer asks the program afterwards has to be the
/// cone run's answer. Without this, GetClustersRunLastGeneration() reports
/// every cluster whenever the verifier is on, and the assertions that prove
/// a cone skipped anything hold or fail on whether the verifier is on rather
/// than on the cone.
///
/// The per-STEP intervals are here for the same reason and not a weaker one:
/// the epilogue replays them into the profiler after the second pass, so
/// leaving them would put a trace of the verification pass beside a cluster
/// table of the cone run and let a reader believe the two describe one run.
struct RigExecBakedRunStatistics {
    /// Takes the statistics \p program currently holds.
    explicit RigExecBakedRunStatistics(const RigExecBakedProgramImpl &program);
    /// Puts them back.
    void Restore(RigExecBakedProgramImpl *program) const;

    struct ClusterTimes {
        uint64_t readyUs = 0, startUs = 0, endUs = 0;
        std::thread::id runner;
    };
    struct StepTimes {
        uint64_t startUs = 0, endUs = 0;
        /// The op stamps the trace reads. memoEndNs and bodyEndNs need no
        /// copy: only a measurement writes them, and the second pass runs
        /// with measurementSuspended, so it leaves the first pass's.
        uint64_t memoStartNs = 0, publishEndNs = 0;
        std::thread::id runner;
    };
    RigExecOpExecution opExecution;
    std::vector<ClusterTimes> clusters;
    std::vector<StepTimes> steps;
    RigExecBakedClusterSet selectedClusters, selectedSteps;
    bool closureFull = false;
    size_t closedClusters = 0;
    size_t closedSteps = 0;
    size_t spaceLeafKeys = 0;
    size_t sourceKeysBuilt = 0;
    bool timed = false;
    /// The program's timedClusters, restored with the cluster times.
    std::vector<uint32_t> timedClusters;
};

/// Whether RIGEXEC_BAKED_VERIFY_CONES asks a run to prove its cone.
bool RigExecBakedVerifyConesRequested();

/// A projector target's provider world frames, out of the program's rest
/// frames and its base and final matrix tables.
RigExecSurfaceProjectorFrames RigExecBakedProjectorFrames(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomRevision &revision);

/// One projector target of \p chain, run against its authored base and
/// final points over the reads its path leaves hold
/// (RigExecReadProjectorTargetFromLeaves): what the live and frozen Derived
/// steps do for a matrix target. Reads no stage.
bool
RigExecBakedRunProjectorTarget(const RigExecBakedProgramImpl &B,
                               const RigExecBakedProgramImpl::GeomChain &chain,
                               RigExecBakedProgramImpl::GeomRevision &revision,
                               GfMatrix4d *matrix,
                               std::vector<std::string> *diagnostics);

/// The step graph, for this library's own compile, schedule and replay code.
/// The program keeps the graph private; only this route reaches it, and this
/// header is not installed.
struct RigExecBakedProgramAccess {
    static const RigExecBakedProgramImpl &
    StepGraph(const RigExecBakedProgram &program);
};

/// Test-only access to evaluator state a baked run must not touch.
struct RigExecBakedProgramTesting {
    /// Enables live per-operation exec witnesses. Build and Evaluate are
    /// owning-thread operations; native bodies only copy their bound values.
    static bool EnableExecCrossCheck(const RigExecBakedProgram &program,
                                     std::string *error = nullptr);
    /// The same rows, captured from the bodies but never judged by OpenExec.
    static bool EnableExecRowCapture(const RigExecBakedProgram &program,
                                     std::string *error = nullptr);
    static std::shared_ptr<RigExecBakedExecCheckRows>
    ExecCrossCheckRows(const RigExecBakedProgram &program);
    /// Builds after this one call \p injector on the program being built;
    /// when it returns true, its op nodes run as steps of the program.
    /// Null stops. Set it on the thread that builds, while none builds.
    static void
    SetOpInjector(std::function<bool(const RigExecBakedProgramImpl &,
                                     RigExecBakedOpInjection *)>
                      injector);
    /// Whether a point binding fills its gather buffer with NaN as it is
    /// destroyed, so a reader that outlives the binding reads NaN on every
    /// platform. Set it on the thread that runs, while none runs.
    static void SetPoisonReleasedGathers(bool on);
    static void SetOpObservers(const RigExecBakedProgram &program,
                               std::function<void(uint32_t)> before,
                               std::function<void(uint32_t)> after);
    /// Clears \p program's point captures and has every later run record,
    /// per point binding, the binding's answer as its reader took it.
    /// Returns false and captures nothing under the parallel schedule: the
    /// capture is a test aid and stays off the parallel executor.
    static bool CapturePointReads(const RigExecBakedProgram &program);
    /// The condition behind IsBakeable's and Build's solver-checkpoint
    /// guard: the solvers in \p snapshots (joint -> read-phase movers) that
    /// no batched solver commits for that joint. No compiled rig reaches it,
    /// so it is tested on hand-built maps.
    static std::vector<SdfPath> SolverCheckpointsWithoutAnOutput(
        const std::map<SdfPath, std::set<SdfPath>> &snapshots,
        const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
        const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
            &solverJoints,
        const std::set<SdfPath> &batched);
    /// A copy of \p program's solver \p index with the Solve step's rest
    /// refresh (RefreshSolverRests, bakedPose.cpp) applied to it, from the
    /// rests and `fin` the program holds now. The program is not modified.
    /// \p index must be below `solvers.size()`.
    static RigExecBakedProgramImpl::Solver
    RefreshedSolverRests(const RigExecBakedProgram &program, size_t index);
    /// The ten rest and ladder tables the bake exports.
    using LadderTables = RigExecBakedLadderTables;
    /// \p program's ten tables as they stand.
    static LadderTables LadderTablesOf(const RigExecBakedProgram &program);
    /// Assembles every revision of \p program whose packet its path leaves
    /// assemble (chain revisions whose chain read a base this run, and
    /// non-matrix derived targets) twice, from the leaves the last run
    /// sampled and from the stage through \p resolved at \p time, as the
    /// stage assembly did before the leaves, and returns one line per
    /// revision whose packets, statuses or lines differ or whose leaf
    /// assembly read a role its declaration lacks (named). A projector's
    /// matrix target compares its reads the same way
    /// (RigExecReadProjectorTargetFromLeaves against
    /// RigExecReadProjectorTarget). \p edit, when set, changes a copy of
    /// each declaration first (by mover path). Leaves the program as it
    /// found it. \p compared counts the revisions.
    static std::vector<std::string> ShadowAssembly(
        const RigExecBakedProgram &program,
        const RigExecResolvedInputs &resolved, UsdTimeCode time,
        const std::function<void(const SdfPath &, RigExecRevisionLeafDecl *)>
            &edit,
        size_t *compared);
};

VtValue RigExecBakedResolvePathLeaf(const RigExecBakedProgramImpl &,
                                    const RigExecBakedPathLeaves &, size_t);
/// Revision \p id's inputs:enabled, inputs:defaultWeight and whether its
/// rigExec:weightBlend is radial, as its packet assembly reads them this run.
bool RigExecBakedRevisionLeaf(const RigExecBakedProgramImpl &B, uint32_t id,
                              bool *enabled, float *weight, bool *radial,
                              std::string *error);
/// Revision \p id, which op nodes compute, enabled or not this frame: its
/// status, and its weight display from the packet its op reads.
bool RigExecBakedStoreRevisionEnabled(RigExecBakedProgramImpl &B, uint32_t id,
                                      bool enabled, std::string *error);
bool
RigExecBakedGeometryReadsLeaf(const RigExecBakedProgramImpl::GeomRevision &,
                              const RigExecBakedStep &, size_t);
/// Exact captured current API4 input heads and connection hops. No stage access.
bool RigExecBakedHasExternalInputRoute(const RigExecBakedProgramImpl &,
                                       const SdfPath &);
const std::vector<VtValue> &
RigExecBakedResolvePathLeaves(const RigExecBakedProgramImpl &,
                              const RigExecBakedPathLeaves &);
void RigExecBakedResetWeightField(RigExecBakedProgramImpl *, int);

/// RigExecIkFkMatchConstraint (bakedIkFkMatch.cpp): the publication of every
/// match's values and presented frames into \p pose.
void RigExecBakedPresentIkFkMatches(const RigExecBakedProgramImpl &B,
                                    RigExecRigPose *pose);

/// RigExecHarmonicsConstraint (bakedHarmonics.cpp): one run of its step.
/// Captures the windowed op's packet and, when it solves, fills \p commit's
/// one target; a pass-through leaves the commit abandoned.
void RigExecBakedRunHarmonics(RigExecBakedProgramImpl &B,
                              RigExecBakedStep *step,
                              RigExecBakedProgramImpl::Constraint &c,
                              RigExecBakedCommit &commit);
/// Settles every windowed op's step, canonical op and inner windowed ops,
/// and the history-tick mask, from the compiled graph. Build calls it once
/// the graph is compiled; it sizes the capture and ring tables.
bool RigExecBakedSettleWindowedOps(RigExecBakedProgramImpl *program,
                                   std::string *error);
/// Places \p rings (one per windowed op, missing ones empty) for the next
/// run, marking each op whose ring changed. Owner thread, between runs.
void RigExecBakedPlaceWindowRings(RigExecBakedProgramImpl *program,
                                  const std::vector<RigExecWindowRing> &rings);
/// Marks the windowed revisions the live program solves on a device: the
/// last revision of a chain nothing outside it reads, whose handler has a
/// device solve, which no windowed op depends on, while a device runner is
/// registered (rigExec/internal/gpuCloth.h). Build calls it after the windowed ops
/// settle; a frozen clone clears the marks.
void RigExecBakedSelectWindowDevices(RigExecBakedProgramImpl *program);

struct RigExecIkFkMatchChannels;

/// A control's channels as the compose reads them, for the match kernels.
void RigExecBakedMatchChannelsOf(const RigExecBakedProgramImpl &B, int slot,
                                 RigExecIkFkMatchChannels *out);

/// Space matching (bakedSpaces.cpp): \p sw's matchValues for the control in
/// \p slot, which the switch just composed onto \p posed from \p input,
/// with a refusal's diagnostic appended to \p diagnostics, and the
/// publication of every switch's values into \p pose.
void RigExecBakedRunSpaceMatch(const RigExecBakedProgramImpl &B,
                               RigExecBakedProgramImpl::SpaceSwitch &sw,
                               int slot, const RigExecSpaceSwitchInputs &input,
                               const RigExecPointFrame &posed,
                               std::vector<std::string> *diagnostics);
void RigExecBakedPresentSpaceMatches(const RigExecBakedProgramImpl &B,
                                     RigExecRigPose *pose);

} // namespace rigExec

#endif // RIGEXEC_BAKED_PROGRAM_IMPL_H
