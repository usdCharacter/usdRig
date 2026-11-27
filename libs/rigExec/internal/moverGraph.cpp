// RigExec compiled mover graph. See moverGraph.h.
#include "rigExecSchema/schemaNames.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecMath/blendRangeKernel.h"
#include "rigExecMath/blendShapeKernel.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/api/moverRegistry.h"
#include "moverGraph.h"
#include "rigExecEvalCore/gate.h"
#include "rigExecEvalCore/skin.h"
#include "parallel.h"
#include "pathText.h"
#include "frameExtraction.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/normalsLayoutKernel.h"
#include "rigExecMath/ribbonKernel.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/hash.h"
#include "pxr/base/work/loops.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

// The packet kind RigExecAssembleParameters stamps on each operation. The
// revision kernels check it before they run: a packet assembled for one
// operation arriving at another's kernel is a compile bug, and the revision
// passes through rather than running the wrong maths on it.
TF_DEFINE_PRIVATE_TOKENS(
    _kindTokens,
    ((matrix, "matrix"))
    ((skin, "skin"))
    ((blendShape, "blendShape"))
    ((volumeCorrect, "volumeCorrect"))
    ((smooth, "smooth"))
    ((deltaMush, "deltaMush"))
    ((wrinkle, "wrinkle"))
    ((lattice, "lattice"))
    ((surfaceProject, "surfaceProject"))
    ((ribbon, "ribbon"))
    ((wire, rigExec::SchemaValueText::wire))
    ((emitGuidePoints, "emitGuidePoints"))
    ((external, "external"))
    ((recomputeNormals, "recomputeNormals"))
    ((recomputeExtent, "recomputeExtent"))
    ((surfaceProjector, "surfaceProjector"))
    ((shaderDials, "shaderDials"))
);

// The attribute and value names the PER-FRAME assemblers read. Hoisted out of
// their bodies because TfToken(const char *) takes the token registry's spin
// lock on every construction -- on the hit path as much as on the miss path --
// and an assembler runs once per revision per frame, inside a baked step that
// is not allowed to take a lock at all (libs/rigExec/internal/bodyPurity.h). The
// bind-time readers below keep their inline tokens: they run once per
// generation, off any step.
TF_DEFINE_PRIVATE_TOKENS(
    _attrTokens,
    ((weightObjectRel, rigExec::SchemaAttrText::rigExecWeightObject))
    ((enabled, rigExec::SchemaAttrText::inputsEnabled))
    ((defaultWeight, rigExec::SchemaAttrText::inputsDefaultWeight))
    ((jointIndices, rigExec::SchemaAttrText::rigExecJointIndices))
    ((jointWeights, rigExec::SchemaAttrText::rigExecJointWeights))
    ((elementSize, rigExec::SchemaAttrText::rigExecElementSize))
    ((skinningMethod, rigExec::SchemaAttrText::rigExecSkinningMethod))
    ((deltaSpace, rigExec::SchemaAttrText::rigExecDeltaSpace))
    ((divisions, rigExec::SchemaAttrText::rigExecDivisions))
    ((restPoints, rigExec::SchemaAttrText::inputsRestPoints))
    ((iterations, rigExec::SchemaAttrText::inputsIterations))
    ((topology, rigExec::SchemaAttrText::inputsTopology))
    ((neighborDistance, rigExec::SchemaAttrText::inputsNeighborDistance))
    ((restLengthScale, rigExec::SchemaAttrText::inputsRestLengthScale))
    ((stretchStiffness, rigExec::SchemaAttrText::inputsStretchStiffness))
    ((compressionStiffness, rigExec::SchemaAttrText::inputsCompressionStiffness))
    ((bendStiffness, rigExec::SchemaAttrText::inputsBendStiffness))
    ((maxDisplacement, rigExec::SchemaAttrText::inputsMaxDisplacement))
    ((pinBorders, rigExec::SchemaAttrText::inputsPinBorders))
    ((pinPoints, rigExec::SchemaAttrText::inputsPinPoints))
    ((tangentPlaneCollisions, rigExec::SchemaAttrText::inputsTangentPlaneCollisions))
    ((tangentPlaneInset, rigExec::SchemaAttrText::inputsTangentPlaneInset))
    ((wrinkleScale, rigExec::SchemaAttrText::inputsWrinkleScale))
    ((smoothingIterations, rigExec::SchemaAttrText::inputsSmoothingIterations))
    ((weightBlend, rigExec::SchemaAttrText::rigExecWeightBlend))
    ((pointFrame, rigExec::SchemaAttrText::rigExecPointFrame))
    ((step, rigExec::SchemaAttrText::inputsStep))
    ((distanceWeight, rigExec::SchemaAttrText::inputsDistanceWeight))
    ((displacement, rigExec::SchemaAttrText::inputsDisplacement))
    ((driverWeights, rigExec::SchemaAttrText::inputsDriverWeights))
    ((driverBaseWeights, rigExec::SchemaAttrText::inputsDriverBaseWeights))
    ((driverDeltaFrame, rigExec::SchemaAttrText::rigExecDriverDeltaFrame))
    ((dropoffDistance, rigExec::SchemaAttrText::inputsDropoffDistance))
    ((rayOrigin, rigExec::SchemaAttrText::rigExecRayOrigin))
    ((rayDirection, rigExec::SchemaAttrText::rigExecRayDirection))
    ((rayUp, rigExec::SchemaAttrText::rigExecRayUp))
    ((shaderOffset, rigExec::SchemaAttrText::rigExecShaderOffset))
    ((projectionMode, rigExec::SchemaAttrText::rigExecProjectionMode))
    ((smoothing, rigExec::SchemaAttrText::inputsSmoothing))
    ((frameTransport, rigExec::SchemaAttrText::inputsFrameTransport))
    ((smoothWeights, rigExec::SchemaAttrText::inputsSmoothWeights))
    ((edges, rigExec::SchemaAttrText::inputsEdges))
    ((onlySmooth, rigExec::SchemaAttrText::inputsOnlySmooth))
    ((computationToTarget, rigExec::SchemaAttrText::inputsComputationToTarget))
    ((evaluation, rigExec::SchemaAttrText::rigExecEvaluation))
    ((interpolationU, rigExec::SchemaAttrText::rigExecInterpolationU))
    ((interpolationV, rigExec::SchemaAttrText::rigExecInterpolationV))
    ((interpolationW, rigExec::SchemaAttrText::rigExecInterpolationW))
    ((origin, rigExec::SchemaAttrText::rigExecOrigin))
    ((spacing, rigExec::SchemaAttrText::rigExecSpacing))
    ((strength, rigExec::SchemaAttrText::rigExecStrength))
    ((mask, rigExec::SchemaAttrText::rigExecMask))
    ((pointSpace, rigExec::SchemaAttrText::rigExecPointSpace))
    ((cageMatrix, rigExec::SchemaAttrText::rigExecCageMatrix))
    ((surfaceMatrix, rigExec::SchemaAttrText::rigExecSurfaceMatrix))
    ((targetMatrix, rigExec::SchemaAttrText::rigExecTargetMatrix))
    ((snapMode, rigExec::SchemaAttrText::rigExecSnapMode))
    ((offset, rigExec::SchemaAttrText::rigExecOffset))
    ((triangles, rigExec::SchemaAttrText::rigExecTriangles))
);

// The values those reads fall back to, and the three states a revision's
// status reports. Same reason.
TF_DEFINE_PRIVATE_TOKENS(
    _valueTokens,
    ((classicLinear, "classicLinear"))
    ((target, "target"))
    ((disabled, "disabled"))
    ((ok, "ok"))
    ((moverFailed, "moverFailed"))
    ((cloth, "cloth"))
    ((surfaceStruts, "surfaceStruts"))
    ((rest, "rest"))
    ((local, "local"))
    ((material, "material"))
    ((radial, "radial"))
    ((vertex, "vertex"))
    ((legacy, "legacy"))
    ((regularGrid, "regularGrid"))
    ((bspline, "bspline"))
    ((common, "common"))
    ((onSurface, "onSurface"))
    ((inside, "inside"))
    ((outside, "outside"))
    ((outsideSurface, "outsideSurface"))
);

namespace rigExec {

namespace {

const TfToken _noKindToken;
const TfToken &
_RevisionKindToken(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return _kindTokens->matrix;
    case RigExecRevisionOp::Skin:
        return _kindTokens->skin;
    case RigExecRevisionOp::BlendShape:
        return _kindTokens->blendShape;
    case RigExecRevisionOp::VolumeCorrect:
        return _kindTokens->volumeCorrect;
    case RigExecRevisionOp::Smooth:
        return _kindTokens->smooth;
    case RigExecRevisionOp::DeltaMush:
        return _kindTokens->deltaMush;
    case RigExecRevisionOp::Wrinkle:
        return _kindTokens->wrinkle;
    case RigExecRevisionOp::Lattice:
        return _kindTokens->lattice;
    case RigExecRevisionOp::SurfaceProject:
        return _kindTokens->surfaceProject;
    case RigExecRevisionOp::Ribbon:
        return _kindTokens->ribbon;
    case RigExecRevisionOp::Wire:
        return _kindTokens->wire;
    case RigExecRevisionOp::EmitGuidePoints:
        return _kindTokens->emitGuidePoints;
    case RigExecRevisionOp::External:
        return _kindTokens->external;
    case RigExecRevisionOp::RecomputeNormals:
        return _kindTokens->recomputeNormals;
    case RigExecRevisionOp::RecomputeExtent:
        return _kindTokens->recomputeExtent;
    case RigExecRevisionOp::SurfaceProjector:
        return _kindTokens->surfaceProjector;
    case RigExecRevisionOp::ShaderDials:
        return _kindTokens->shaderDials;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently mismatched packet.
    return _noKindToken;
}

// RIGEXEC_ENABLE_SIMD, read once while the library loads: a namespace-scope
// constant, so a kernel on a worker reads it with no guard and no lock.
const bool _simdEnabled = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);

} // namespace

bool
RigExecSimdEnabled()
{
    return _simdEnabled;
}

void
RigExecRevisionKernelTouchTokens()
{
    (void)_kindTokens.Get();
    (void)_attrTokens.Get();
    (void)_valueTokens.Get();
}

// Public alias of the table above: the frozen assembler stamps the same
// kind tokens onto worker-assembled packets.
const TfToken &
RigExecRevisionKindToken(RigExecRevisionOp op)
{
    return _RevisionKindToken(op);
}

namespace {

// EvalCore's envelope and matrix bodies (rigExecEvalCore/envelope.h) over
// the Gf numbers. The radial identity shortcut is licensed where the float
// environment is the default one.
struct _EnvelopeMath : eval::MatrixKernelMath<GfVec3f, GfVec3d, GfMatrix4d> {
    static bool FloatEnvironmentIsDefault()
    {
        return RigExecFloatEnvironmentIsDefault();
    }
};

// EvalCore's skin bodies (rigExecEvalCore/skin.h) over the Gf numbers, as
// in bakedGeometry.cpp's BakedSkinMath: the SIMD blend and narrowed rows
// are the rigExecMath kernels, the split is the Gf dual-quat palette.
struct _SkinMath {
    using Vec3f = GfVec3f;
    using Vec3d = GfVec3d;
    using Mat4 = GfMatrix4d;
    using ScaledDualQuat = RigExecScaledDualQuat;
    static Vec3f ToFloat(const Vec3d &v)
    {
        return Vec3f(v);
    }
    static Vec3d ToDouble(const Vec3f &v)
    {
        return Vec3d(v);
    }
    static void NarrowSkinRows(const Mat4 &t, float *rows)
    {
        RigExecNarrowSkinRows(t, rows);
    }
    static void
    ApplyLinearBlendSkinSimd(const Vec3f *in, Vec3f *out,
                             const eval::SkinLayoutViewT<_SkinMath> &part,
                             const float *rows)
    {
        RigExecSkinLayout layout;
        layout.transforms = part.transforms;
        layout.transformCount = part.transformCount;
        layout.indices = part.indices;
        layout.weights = part.weights;
        layout.indexCount = part.indexCount;
        layout.elementSize = part.elementSize;
        layout.pointCount = part.pointCount;
        RigExecApplyLinearBlendSkinSimd(in, out, layout, rows);
    }
};

// The packet as EvalCore's EnvelopePacketView: representation and range
// policy classified to its integer codes, the arrays borrowed.
eval::EnvelopePacketView
_EnvelopeView(const RigExecWeightPacket &w)
{
    eval::EnvelopePacketView v;
    v.valid = w.valid;
    v.defaultWeight = w.defaultWeight;
    v.values = w.values.data();
    v.valueCount = w.values.size();
    v.indices = w.indices.data();
    v.indexCount = w.indices.size();
    v.rangePolicyEmpty = w.rangePolicy.IsEmpty();
    v.representation = w.representation == SchemaValueText::constant
                           ? eval::kWeightConstant
                           : (w.representation == SchemaValueText::sparse
                                  ? eval::kWeightSparse
                                  : (w.representation == SchemaValueText::dense
                                         ? eval::kWeightDense
                                         : eval::kWeightRepresentationUnknown));
    v.rangePolicy = w.rangePolicy == SchemaValueText::strict
                        ? eval::kWeightPolicyStrict
                        : (w.rangePolicy == SchemaValueText::clamp
                               ? eval::kWeightPolicyClamp
                               : eval::kWeightPolicyUnknown);
    return v;
}

// The packet as EvalCore's SkinPacketView: the method classified by
// RigExecSkinMethodOf, the layout arrays borrowed from the handle or the
// packet.
eval::SkinPacketView
_SkinPacket(const RigExecMoverParameters &p)
{
    eval::SkinPacketView v;
    const RigExecSkinMethod method = RigExecSkinMethodOf(p);
    v.method = method == RigExecSkinMethod::ClassicLinear
                   ? eval::kSkinMethodClassicLinear
                   : (method == RigExecSkinMethod::DualQuaternion
                          ? eval::kSkinMethodDualQuaternion
                          : eval::kSkinMethodUnknown);
    if (const RigExecSkinTopology *const topology = p.skinTopology.get()) {
        v.hasTopology = true;
        v.topologyValidated = topology->validated;
        v.topologyInfluenceCount = topology->influenceCount;
        v.topologyIndices = topology->indices.data();
        v.topologyWeights = topology->weights.data();
        v.topologyIndexCount = topology->indices.size();
        v.topologyElementSize = topology->elementSize;
    }
    v.indices = p.skinIndices.data();
    v.weights = p.skinWeights.data();
    v.indexCount = p.skinIndices.size();
    v.weightCount = p.skinWeights.size();
    v.elementSize = p.skinElementSize;
    return v;
}

// The influence tables as EvalCore's view: the same borrowed arrays.
eval::SkinTransformsViewT<_SkinMath>
_EvalSkinTransforms(const RigExecSkinTransformsView &view)
{
    eval::SkinTransformsViewT<_SkinMath> out;
    out.transforms = view.transforms;
    out.transformCount = view.transformCount;
    out.rows = view.rows;
    out.palette = view.palette;
    out.paletteSize = view.paletteSize;
    return out;
}

} // namespace

bool
RigExecWireTakesSparseEnvelope(const RigExecWeightPacket &w)
{
    return eval::EnvelopeIsSparseWalk(_EnvelopeView(w));
}

bool
RigExecEnvelopeIsFullStrength(const RigExecWeightPacket &envelope)
{
    return eval::EnvelopeIsFullStrength(_EnvelopeView(envelope));
}

// The matrix kernel, shared by the mover-graph revision node and by the
// baked program. Unlike the point3f[] ops the envelope is NOT a separate
// blend here: the weighted-matrix rule folds it into the movement itself
// (p' = q + w (T q - q)), so resolving it is part of the kernel.
// One definition, so a second caller cannot drift into a different movement
// -- including over the SIMD choice, which must be the same on both paths or
// the two disagree in the last bits.
void
RigExecApplyMatrixKernelRange(const RigExecMoverParameters &p,
                              const float *envelope, size_t begin, size_t end,
                              GfVec3f *pts, bool useSimd)
{
    eval::ApplyMatrixKernelRange<_EnvelopeMath>(
        p.radialWeight, p.transform, envelope, begin, end, pts, pts, useSimd);
}

namespace {

uint64_t
_HashBytes(uint64_t h, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h = (h ^ bytes[i]) * 1099511628211ull;
    }
    return h;
}

} // namespace

// The inputs a basis was built from, kept beside it so a hit compares them
// in full. Immutable once inserted: copies of a cache share it.
struct RigExecWireBasisCache::_Entry {
    std::vector<GfVec2f> binds;
    std::vector<int> indices;
    std::vector<double> knots;
    int order = 0;
    size_t controlPoints = 0;
    size_t meshPoints = 0;
    double dropoff = 0.0;
    std::shared_ptr<const RigExecWireBasis> basis;
};

std::shared_ptr<const RigExecWireBasis>
RigExecWireBasisCache::Get(const RigExecMoverParameters &p,
                           const std::vector<int> &indices, size_t meshPoints)
{
    const auto matches = [&](const _Entry &e) {
        if (e.order != p.curveOrder || e.controlPoints != p.restPoints.size() ||
            e.meshPoints != meshPoints || e.indices != indices ||
            e.knots.size() != p.curveKnots.size() ||
            e.binds.size() != p.wireBindCoords.size() ||
            std::memcmp(&e.dropoff, &p.dropoffDistance, sizeof(double)))
            return false;
        for (size_t i = 0; i < e.knots.size(); ++i)
            if (std::memcmp(&e.knots[i], &p.curveKnots[i], sizeof(double)))
                return false;
        for (size_t i = 0; i < e.binds.size(); ++i)
            for (int axis = 0; axis < 2; ++axis) {
                const float a = e.binds[i][axis], b = p.wireBindCoords[i][axis];
                if (std::memcmp(&a, &b, sizeof(a)))
                    return false;
            }
        return true;
    };
    if (_last && matches(*_last))
        return _last->basis;
    uint64_t h = 1469598103934665603ull;
    h = _HashBytes(h, p.wireBindCoords.cdata(),
                   p.wireBindCoords.size() * sizeof(GfVec2f));
    h = _HashBytes(h, indices.data(), indices.size() * sizeof(int));
    h = _HashBytes(h, p.curveKnots.data(),
                   p.curveKnots.size() * sizeof(double));
    const size_t controlPoints = p.restPoints.size();
    h = _HashBytes(h, &p.curveOrder, sizeof(p.curveOrder));
    h = _HashBytes(h, &controlPoints, sizeof(controlPoints));
    h = _HashBytes(h, &meshPoints, sizeof(meshPoints));
    h = _HashBytes(h, &p.dropoffDistance, sizeof(p.dropoffDistance));

    const auto found = _entries.find(h);
    if (found != _entries.end()) {
        const _Entry &e = *found->second;
        if (matches(e)) {
            _last = found->second;
            return e.basis;
        }
    }
    auto basis = std::make_shared<RigExecWireBasis>();
    if (!RigExecBuildWireBasis(p.wireBindCoords.cdata(),
                               p.wireBindCoords.size(), meshPoints, indices,
                               p.curveOrder, p.curveKnots, controlPoints,
                               p.dropoffDistance, basis.get())) {
        return nullptr;
    }
    ++_builds;
    auto entry = std::make_shared<_Entry>();
    entry->binds.assign(p.wireBindCoords.cbegin(), p.wireBindCoords.cend());
    entry->indices = indices;
    entry->knots = p.curveKnots;
    entry->order = p.curveOrder;
    entry->controlPoints = controlPoints;
    entry->meshPoints = meshPoints;
    entry->dropoff = p.dropoffDistance;
    entry->basis = basis;
    if (_entries.size() >= kCapacity) {
        // Edits accumulate entries nothing will ask for again.
        _entries.clear();
    }
    _last = entry;
    _entries[h] = std::move(entry);
    return basis;
}

namespace {
// The full-strength matrix movement, which rewrites every point: \p in and
// \p out may be the same array, since point i is written only after it is
// read.
void
ApplyMatrixFullStrength(const RigExecMoverParameters &p, const GfVec3f *in,
                        GfVec3f *out, size_t count, bool useSimd)
{
    eval::ApplyMatrixKernelRange<_EnvelopeMath>(
        p.radialWeight, p.transform, nullptr, 0, count, in, out, useSimd);
}
} // namespace

// The validation the matrix, blend-shape and wire kernels run before they
// write, each ONE definition that the kernel runs and
// RigExecRevisionKernelAcceptance decides by, so the two cannot disagree.
namespace {

// The packet check every revision kernel makes first.
bool
_PacketMatches(RigExecRevisionOp op, const RigExecMoverParameters &p)
{
    return p.valid && p.kind == _RevisionKindToken(op);
}

// Every entry a sparse walk names is usable over \p count points: in range,
// strictly ascending, finite and in [0, 1]. Validated exactly as ResolveAll
// would, so the same packets fail.
bool
_SparseWalkIsUsable(const RigExecWeightPacket &w, size_t count)
{
    return eval::SparseEnvelopeIsUsable(_EnvelopeView(w), count);
}

// RigExecApplyMatrixKernel's validation over \p count points: a full-strength
// envelope, a sparse walk whose entries are usable, or a dense resolve. Given
// \p weights, the dense resolve writes into it (ResolveAll validates exactly
// as ResolvesAll does), so the kernel validates once.
bool
_MatrixKernelAccepts(const RigExecMoverParameters &p, size_t count,
                     std::vector<float> *weights)
{
    return eval::MatrixKernelAccepts(_EnvelopeView(p.weights), p.weights, count,
                                     weights);
}

// The entries [*first, *last) of the ascending \p indices that name a point
// in [begin, end).
void
_SparseEntriesIn(const std::vector<int> &indices, size_t begin, size_t end,
                 size_t *first, size_t *last)
{
    const auto below = [](int index, size_t bound) {
        return index < 0 || size_t(index) < bound;
    };
    const auto from =
        std::lower_bound(indices.begin(), indices.end(), begin, below);
    *first = size_t(from - indices.begin());
    *last = size_t(std::lower_bound(from, indices.end(), end, below) -
                   indices.begin());
}

// RigExecApplyBlendShapeKernel's validation over \p count points: one delta
// per point and an envelope that resolves, into \p envelope when given (left
// untouched at full strength). A surface-frame transport after it reads the
// entering points and can still refuse them.
bool
_BlendShapeKernelAccepts(const RigExecMoverParameters &p, size_t count,
                         std::vector<float> *envelope)
{
    if (p.blendDeltas.size() != count) {
        return false;
    }
    if (RigExecEnvelopeIsFullStrength(p.weights)) {
        return true;
    }
    return envelope ? p.weights.ResolveAll(count, envelope)
                    : p.weights.ResolvesAll(count);
}

// The checks the wire operation makes over \p count points before it builds
// or applies anything: both control polygons alike and evaluable, then a
// sparse walk's entries, or a dense walk's one bind coordinate per point.
bool
_WireKernelPrefix(const RigExecMoverParameters &p, size_t count)
{
    if (p.auxPoints.size() != p.restPoints.size()) {
        return false;
    }
    const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    if (!rest.IsValid() || !posed.IsValid()) {
        return false;
    }
    if (RigExecWireTakesSparseEnvelope(p.weights)) {
        return _SparseWalkIsUsable(p.weights, count);
    }
    // A sparse bind table needs a sparse envelope.
    return p.wireBindCoords.size() == count;
}

// The wire operation's answer over \p count points: its prefix, then the
// checks of the basis build or the dense walk it goes on to make
// (RigExecWireBasisInputsAreUsable, RigExecWireInputsAreUsable). An empty
// bind table is Deferred: its answer is whether the table has storage, which
// a memoized basis never asks.
RigExecRevisionAcceptance
_WireKernelAcceptance(const RigExecMoverParameters &p, size_t count)
{
    using Acceptance = RigExecRevisionAcceptance;
    if (!_WireKernelPrefix(p, count)) {
        return Acceptance::Refuses;
    }
    if (p.wireBindCoords.empty()) {
        return Acceptance::Deferred;
    }
    if (RigExecWireTakesSparseEnvelope(p.weights)) {
        // The indices were proved in range above, and a memoized basis exists
        // only for inputs that passed these same checks.
        return RigExecWireBasisInputsAreUsable(
                   p.wireBindCoords.cdata(), p.wireBindCoords.size(), count,
                   p.weights.indices.size(), p.curveOrder, p.curveKnots,
                   p.restPoints.size())
                   ? Acceptance::Applies
                   : Acceptance::Refuses;
    }
    const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    return RigExecWireInputsAreUsable(rest, posed, p.wireBindCoords.cdata(),
                                      p.wireBindCoords.size(), count)
               ? Acceptance::Applies
               : Acceptance::Refuses;
}

} // namespace

bool
RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts, bool useSimd)
{
    return eval::ApplyMatrixKernel<_EnvelopeMath>(_EnvelopeView(p.weights),
                                                  p.weights, p.radialWeight,
                                                  p.transform, pts, useSimd);
}

void
RigExecBlendEnvelopeRange(const GfVec3f *preceding, const float *envelope,
                          size_t begin, size_t end, GfVec3f *blended)
{
    eval::BlendEnvelopeRange<GfVec3f>(preceding, envelope, begin, end, blended);
}

void
RigExecBlendEnvelopeAll(const GfVec3f *preceding, const float *envelope,
                        size_t count, GfVec3f *blended)
{
    eval::BlendEnvelopeAll<GfVec3f>(preceding, envelope, count, blended,
                                    RigExecGeometryRanges());
}

// The skin kernel has one definition for every caller, so none can drift into
// a different deformation.
// The skin kernel comes in three pieces, because a chunked caller needs the
// per-vertex body without the decisions AROUND it -- and every one of those
// decisions is a statement about the WHOLE array, not about a vertex: the
// layout's element shape, index range and weights; the influence table's
// finite/affine check; the method token. So the validation splits in two by
// what it reads (RigExecSkinLayoutIsUsable, RigExecSkinTransformsAreUsable),
// the per-vertex body becomes RigExecApplySkinKernelRange, and this function is
// validate, then run every vertex.

RigExecSkinMethod
RigExecSkinMethodOf(const RigExecMoverParameters &p)
{
    if (p.skinningMethod == "classicLinear") {
        return RigExecSkinMethod::ClassicLinear;
    }
    if (p.skinningMethod == "dualQuaternion") {
        return RigExecSkinMethod::DualQuaternion;
    }
    return RigExecSkinMethod::Unknown;
}

RigExecSkinTransformsView
RigExecSkinTransformsOf(const RigExecMoverParameters &p)
{
    const eval::SkinTransformsViewT<_SkinMath> evalView =
        eval::SkinTransformsOf<_SkinMath>(p.skinTransforms.data(),
                                          p.skinTransforms.size());
    RigExecSkinTransformsView view;
    view.transforms = evalView.transforms;
    view.transformCount = evalView.transformCount;
    view.rows = evalView.rows;
    view.palette = evalView.palette;
    view.paletteSize = evalView.paletteSize;
    return view;
}

RigExecSkinLayout
RigExecSkinLayoutForPacket(const RigExecMoverParameters &p,
                           const RigExecSkinTransformsView &transforms,
                           size_t pointCount)
{
    // The incoming revision IS the rest pose the influences were bound
    // against. A blend shape upstream of the skin is skinned -- exactly as a
    // skinCluster skins its input geometry and UsdSkel applies blend shapes
    // before skinning -- and when the skin is first in its chain the incoming
    // points are the authored base. No separate rest input is needed for
    // that, and every skinning method reads the same gather below.
    const eval::SkinLayoutViewT<_SkinMath> evalLayout =
        eval::SkinLayoutForPacket<_SkinMath>(
            _SkinPacket(p), _EvalSkinTransforms(transforms), pointCount);
    RigExecSkinLayout layout;
    layout.transforms = evalLayout.transforms;
    layout.transformCount = evalLayout.transformCount;
    layout.indices = evalLayout.indices;
    layout.weights = evalLayout.weights;
    layout.indexCount = evalLayout.indexCount;
    layout.elementSize = evalLayout.elementSize;
    layout.pointCount = evalLayout.pointCount;
    return layout;
}

bool
RigExecSkinLayoutIsUsable(const RigExecMoverParameters &p, size_t pointCount)
{
    return eval::SkinLayoutIsUsable(_SkinPacket(p), pointCount,
                                    p.skinTransforms.size());
}

bool
RigExecSkinTransformsAreUsable(const GfMatrix4d *transforms, size_t count)
{
    return eval::SkinTransformsAreUsable<_SkinMath>(transforms, count);
}

bool
RigExecApplySkinKernelRange(const RigExecMoverParameters &p,
                            const RigExecSkinTransformsView &transforms,
                            size_t begin, size_t end, std::vector<GfVec3f> *pts,
                            bool useSimd)
{
    return eval::ApplySkinKernelRange<_SkinMath>(
        _SkinPacket(p), _EvalSkinTransforms(transforms), begin, end, pts,
        useSimd);
}

bool
RigExecApplySkinKernelWithTransforms(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms, std::vector<GfVec3f> *pts,
    bool useSimd)
{
    return eval::ApplySkinKernelWithTransforms<_SkinMath>(
        _SkinPacket(p), _EvalSkinTransforms(transforms), pts, useSimd,
        !p.skinTopology, RigExecGeometryRanges());
}

bool
RigExecApplySkinKernel(const RigExecMoverParameters &p,
                       std::vector<GfVec3f> *pts, bool useSimd)
{
    return RigExecApplySkinKernelWithTransforms(p, RigExecSkinTransformsOf(p),
                                                pts, useSimd);
}

// The blend-shape kernel, shared by the mover-graph revision node and by the
// baked program. Like the matrix kernel and unlike the point3f[] ops the
// envelope is NOT a separate blend: the deltas are added to the preceding
// revision and the result blended back against it in one pass, so resolving
// the envelope is part of the kernel.
// One definition, so a second caller cannot drift into a different blend.
bool
RigExecApplyBlendShapeKernel(const RigExecMoverParameters &p,
                             std::vector<GfVec3f> *pts,
                             RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    const size_t count = pts->size();
    // Resolve the common envelope up front so a cardinality failure fails the
    // application before any element is written (the in-place write cannot be
    // rolled back).
    std::vector<float> envelope;
    if (!_BlendShapeKernelAccepts(p, count, &envelope)) {
        return false;
    }
    const bool fullStrength = RigExecEnvelopeIsFullStrength(p.weights);

    std::vector<GfVec3f> transported;
    const std::vector<GfVec3f> *deltas = &p.blendDeltas;
    if (p.blendSurfaceFrame) {
        // The incoming points ARE the posed surface the offsets ride on.
        const auto *rest =
            cache ? cache->TransportRest(p.restPoints, p.topologyCounts,
                                         p.topologyIndices)
                  : nullptr;
        const bool ok =
            cache ? (rest && RigExecApplyTransportWithRestData(
                                 *pts, p.topologyCounts, p.topologyIndices,
                                 p.blendDeltas, *rest, &transported))
                  : RigExecTransportSurfaceOffsets(
                        p.restPoints, *pts, p.topologyCounts, p.topologyIndices,
                        p.blendDeltas, &transported);
        if (!ok) {
            return false;
        }
        deltas = &transported;
    }

    // Per point, reading two arrays and writing a third at the same index --
    // the same independent sub-problem RigExecBlendEnvelopeAll already
    // splits, and for the same reason: dividing the range changes nothing
    // about the arithmetic, only who performs it. Everything above stays on
    // this thread, because the envelope resolve and the surface transport
    // are statements about the WHOLE array and fail it atomically.
    // The blend-shape revision's cost is dominated by assembling the
    // channels (bakedGeometry.cpp, the per-sample GetAttribute reads), not by
    // this loop, so splitting it is not a speedup on its own. The split is
    // kept because it is correct, free and matches every other point kernel.
    GfVec3f *const points = pts->data();
    const GfVec3f *const delta = deltas->data();
    const float *const weight = envelope.data();
    const auto blendRange = [points, delta, weight, fullStrength](size_t begin,
                                                                  size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const GfVec3f preceding = points[i];
            points[i] = RigExecBlendEnvelope(preceding, preceding + delta[i],
                                             fullStrength ? 1.0f : weight[i]);
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        count >= RigExecGeometryParallelThreshold) {
        WorkParallelForN(count, blendRange, RigExecGeometryGrainSize);
        return true;
    }
    blendRange(0, count);
    return true;
}

// The derived-maintenance kernel, shared by the mover-graph revision node and
// by the baked program: normal3f[] and float3[] hosts recomputed from the
// final same-generation points rather than from the preceding revision.
// Self-enveloping, like the matrix and blend-shape kernels.
// One definition, so the size rules and the envelope cannot drift between the
// two paths that maintain the same property.
bool
RigExecApplyDerivedKernel(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          const GfVec3f *authored, size_t authoredCount,
                          std::vector<GfVec3f> *result)
{
    if (!result || (authoredCount && !authored))
        return false;
    const bool extent = op == RigExecRevisionOp::RecomputeExtent;
    std::vector<GfVec3f> values;
    if (extent) {
        values = RigExecComputeExtent(p.auxPoints, p.widths);
    } else {
        const RigExecVertexNormalsTopology *const topology =
            p.normalsTopology &&
                    p.normalsTopology->Fits(p.auxPoints.size(),
                                            p.topologyCounts.size(),
                                            p.topologyIndices.size())
                ? p.normalsTopology.get()
                : nullptr;
        values = topology
                     ? RigExecComputeVertexNormals(
                           p.auxPoints.data(), p.auxPoints.size(),
                           p.topologyCounts.data(), p.topologyCounts.size(),
                           p.topologyIndices.data(), p.topologyIndices.size(),
                           *topology, RigExecGeometryRanges())
                     : RigExecComputeVertexNormals(
                           p.auxPoints.data(), p.auxPoints.size(),
                           p.topologyCounts.data(), p.topologyCounts.size(),
                           p.topologyIndices.data(), p.topologyIndices.size(),
                           RigExecGeometryRanges());
        if (!values.empty() &&
            !RigExecLayoutNormals(&values, p.topologyIndices.data(),
                                  p.topologyIndices.size(), p.normalsPerCorner,
                                  p.normalsLeftHanded))
            return false;
    }
    if (values.empty() || (extent && values.size() != 2) ||
        authoredCount != values.size())
        return false;
    if (!RigExecEnvelopeIsFullStrength(p.weights)) {
        std::vector<float> envelope;
        if (!p.weights.ResolveAll(values.size(), &envelope))
            return false;
        for (size_t i = 0; i < values.size(); ++i)
            values[i] =
                RigExecBlendEnvelope(authored[i], values[i], envelope[i]);
    }
    // Writes *out only after every refusal check passes.
    *result = std::move(values);
    return true;
}

bool
RigExecResolveNormalsLayout(const SdfPath &target, const TfToken &interpolation,
                            bool indexed, const TfToken &orientation,
                            RigExecRevisionBinding *binding, std::string *why)
{
    const auto refuse = [&](const std::string &reason) {
        if (why)
            *why = reason;
        return false;
    };
    if (indexed)
        return refuse("indexed normals " + target.GetString() +
                      " cannot be maintained on a moved mesh; author them "
                      "unindexed");
    const TfToken &layout =
        interpolation.IsEmpty() ? UsdGeomTokens->vertex : interpolation;
    if (layout != UsdGeomTokens->vertex && layout != UsdGeomTokens->varying &&
        layout != UsdGeomTokens->faceVarying)
        return refuse("normals " + target.GetString() + " with " +
                      layout.GetString() +
                      " interpolation cannot be maintained on a moved mesh; "
                      "author vertex, varying or faceVarying normals");
    binding->normalsPerCorner = layout == UsdGeomTokens->faceVarying;
    binding->normalsLeftHanded = orientation == UsdGeomTokens->leftHanded;
    return true;
}

void
RigExecAttachNormalsTopology(RigExecRevisionBinding *binding, size_t pointCount,
                             const VtIntArray &counts,
                             const VtIntArray &indices)
{
    binding->normalsTopology =
        std::make_shared<const RigExecVertexNormalsTopology>(
            RigExecBuildVertexNormalsTopology(pointCount, counts.cdata(),
                                              counts.size(), indices.cdata(),
                                              indices.size()));
}

bool
RigExecApplyDerivedKernel(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          std::vector<GfVec3f> *pts)
{
    return pts &&
           RigExecApplyDerivedKernel(op, p, pts->data(), pts->size(), pts);
}

// Revision ops shared by the mover-graph node and the baked program.
// ONE definition, called by the mover-graph revision node and by the baked
// program: a second copy of a deformation agrees on the fixtures that exist
// and drifts on the ones that do not.
// The envelope is NOT applied here for the ops that take a separate blend --
// RigExecRunRevisionKernel wraps this, which is where the "apply once" rule
// lives; matrix, blendShape and the two derived recomputations fold it into
// their own arithmetic and are routed there instead.
namespace {
bool
ApplyRevisionKernel(RigExecRevisionOp op, const RigExecMoverParameters &p,
                    std::vector<GfVec3f> *pts, bool useSimd,
                    RigExecWireBasisCache *wireBasis,
                    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                    bool discardable, const RigExecExternalRevision *external)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return RigExecApplyMatrixKernel(p, pts, useSimd);
    case RigExecRevisionOp::Skin:
        return RigExecApplySkinKernel(p, pts, useSimd);
    case RigExecRevisionOp::BlendShape:
        return RigExecApplyBlendShapeKernel(p, pts, cache);
    case RigExecRevisionOp::VolumeCorrect:
        RigExecApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
        return true;
    case RigExecRevisionOp::Smooth:
        if (cache) {
            const auto *adj = cache->MeshAdjacency(
                pts->size(), p.topologyCounts, p.topologyIndices);
            if (adj)
                RigExecApplyLaplacianSmoothWithAdjacency(pts, *adj, p.strength);
        } else
            RigExecApplyLaplacianSmooth(pts, p.topologyCounts,
                                        p.topologyIndices, p.strength);
        return true;
    case RigExecRevisionOp::DeltaMush: {
        // The retained rest state is built for the packet's smoothing
        // settings; the computation-space map is applied around it.
        const auto *rest =
            cache ? cache->MushRest(p.restPoints, p.topologyCounts,
                                    p.topologyIndices, p.mushIterations,
                                    p.mushStep, p.mushPinBorders,
                                    p.mushDistanceWeight, p.mushSettings)
                  : nullptr;
        if (cache && !rest)
            return false;
        return RigExecApplyDeltaMush(
            pts, p.restPoints, p.topologyCounts, p.topologyIndices,
            p.mushIterations, p.mushStep, p.mushPinBorders,
            p.mushDistanceWeight, p.mushDisplacement, p.mushSettings,
            p.mushComputationToTarget, rest);
    }
    case RigExecRevisionOp::Wrinkle: {
        const auto *topology =
            cache ? cache->WrinkleTopology(p.restPoints.size(),
                                           p.topologyCounts, p.topologyIndices,
                                           p.wrinkleSettings.topology,
                                           p.wrinkleSettings.neighborDistance)
                  : nullptr;
        if (cache && !topology)
            return false;
        return RigExecApplyWrinkle(pts, p.restPoints, p.topologyCounts,
                                   p.topologyIndices, p.wrinkleSettings,
                                   topology);
    }
    case RigExecRevisionOp::Lattice:
        if (p.latticeSettings.regularGrid)
            return RigExecApplyLatticeGridKernel<GfVec3f, GfVec3d>(
                pts, p.auxPointsB, p.divisions, p.latticeSettings,
                p.targetToLattice, p.latticeToTarget, p.cageToLattice);
        if (p.restPoints.size() != pts->size()) {
            return false; // cardinality mismatch fails atomically
        }
        RigExecApplyLattice(pts, p.restPoints, p.auxPoints, p.auxPointsB,
                            p.divisions, cache);
        return true;
    case RigExecRevisionOp::SurfaceProject: {
        if (RigExecSurfaceSnapIsLegacy(p.surfaceSettings, p.targetToSurface,
                                       p.surfaceToTarget, p.surfaceToMetric)) {
            RigExecApplySurfaceProject(pts, p.auxPoints, p.topologyCounts,
                                       p.topologyIndices, p.strength);
            return true;
        }
        const auto *triangles =
            cache ? cache->SnapTriangles(p.auxPoints.size(), p.topologyCounts,
                                         p.topologyIndices,
                                         p.surfaceSettings.triangles)
                  : nullptr;
        if (cache && !triangles)
            return false;
        return RigExecApplySurfaceSnapKernel<GfVec3f, GfVec3d>(
            pts, p.auxPoints, p.topologyCounts, p.topologyIndices,
            p.surfaceSettings, p.targetToSurface, p.surfaceToTarget,
            p.surfaceToMetric, triangles, RigExecGeometryRanges(),
            cache ? &cache->SnapScratch() : nullptr);
    }
    case RigExecRevisionOp::EmitGuidePoints:
        if (p.frames.GetSize() != pts->size()) {
            return false;
        }
        for (size_t i = 0; i < pts->size(); ++i) {
            (*pts)[i] = GfVec3f(p.frames.frames[i].Origin());
        }
        return true;
    case RigExecRevisionOp::Ribbon:
        // Rest-relative rigid transport: per-sample maps from the
        // aggregate's rest frames to its posed frames.
        return RigExecApplyRibbonKernel<RigExecDualQuatMath<GfVec3f, GfVec3d>>(
            pts->data(), pts->size(), p.bindCoords.data(), p.bindCoords.size(),
            p.frames.GetSize(),
            [&](size_t k) -> const auto & { return p.frames.rests[k]; },
            [&](size_t k) -> const auto & {
                return p.frames.frames[k].points;
            });
    case RigExecRevisionOp::Wire: {
        // The checks before anything is built or applied; the basis build
        // and RigExecApplyWire below make the rest of the decision's.
        if (!_WireKernelPrefix(p, pts->size())) {
            return false;
        }
        const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder,
                                     &p.curveKnots};
        const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                      &p.curveKnots};
        // A sparse zero-default envelope: evaluate only the named points.
        // RigExecRunRevisionKernel routes a wire here with its envelope
        // unapplied only when this packet shape is what it holds.
        if (RigExecWireTakesSparseEnvelope(p.weights)) {
            const RigExecWeightPacket &w = p.weights;
            std::shared_ptr<const RigExecWireBasis> basis;
            if (wireBasis) {
                basis = wireBasis->Get(p, w.indices, pts->size());
            } else {
                auto built = std::make_shared<RigExecWireBasis>();
                if (RigExecBuildWireBasis(
                        p.wireBindCoords.cdata(), p.wireBindCoords.size(),
                        pts->size(), w.indices, p.curveOrder, p.curveKnots,
                        p.restPoints.size(), p.dropoffDistance, built.get())) {
                    basis = std::move(built);
                }
            }
            if (!basis) {
                return false;
            }
            return RigExecApplyWireBasis(pts, *basis, w.indices, w.values,
                                         p.restPoints, p.auxPoints);
        }
        const auto *restEvals =
            wireBasis
                ? wireBasis->restEvaluations.Get(rest, p.wireBindCoords.cdata(),
                                                 p.wireBindCoords.size(),
                                                 p.dropoffDistance)
                : nullptr;
        // Per-point and independent, so the range splits across threads;
        // a small mesh stays on this thread.
        bool ok = true;
        if (RigExecParallelEvaluationEnabled() &&
            !RigExecFrozenSerialActive() && pts->size() >= 4096) {
            std::atomic<bool> good(true);
            WorkParallelForN(pts->size(), [&](size_t b, size_t e) {
                if (!RigExecApplyWire(
                        pts, rest, posed, p.wireBindCoords.cdata(),
                        p.wireBindCoords.size(), p.dropoffDistance, b, e,
                        restEvals ? restEvals->data() : nullptr,
                        restEvals ? restEvals->size() : 0)) {
                    good = false;
                }
            });
            ok = good;
        } else {
            ok = RigExecApplyWire(pts, rest, posed, p.wireBindCoords.cdata(),
                                  p.wireBindCoords.size(), p.dropoffDistance, 0,
                                  pts->size(),
                                  restEvals ? restEvals->data() : nullptr,
                                  restEvals ? restEvals->size() : 0);
        }
        return ok;
    }
    case RigExecRevisionOp::External: {
        const RigExecMoverHandler *handler =
            external ? external->handler : nullptr;
        if (handler && external->window) {
            // A windowed revision (windowHistory.h): capture this tick, then
            // solve over the earlier ticks and it -- unless the run only
            // captures, when the points stand as they entered.
            RigExecWindowRun &run = *external->window;
            if (!handler->captureWindow || !handler->applyWindow)
                return false;
            std::vector<const RigExecWindowPacket *> ticks;
            // Owns this tick's packet through the solve: the capture slot
            // keeps the packet it holds when the two are equal.
            RigExecWindowPacketPtr captured;
            const RigExecWindowRing none;
            if (run.solveTicks) {
                // The boundary's half of a deferred solve: the body captured.
                for (const RigExecWindowPacketPtr &tick : *run.solveTicks)
                    ticks.push_back(tick.get());
                if (ticks.empty() || !ticks.back())
                    return false;
            } else {
                auto packet = std::make_shared<RigExecWindowPacket>();
                if (handler->captureWindow(external->data, *pts, packet.get()))
                    captured = std::move(packet);
                if (run.capture &&
                    !RigExecWindowPacketsEqual(*run.capture, captured))
                    *run.capture = captured;
                if (run.captureOnly)
                    return true;
                if (!captured)
                    return false;
                const RigExecWindowRing &ring = run.ring ? *run.ring : none;
                if (run.deferred) {
                    // The solve waits for the publication boundary; the
                    // points pass through until then.
                    run.deferred->clear();
                    size_t begin = ring.size();
                    while (begin > 0 && ring[begin - 1])
                        --begin;
                    run.deferred->assign(ring.begin() + std::ptrdiff_t(begin),
                                         ring.end());
                    run.deferred->push_back(captured);
                    return true;
                }
                RigExecWindowUsableTail(ring, captured.get(), &ticks);
            }
            const size_t enteringCount = pts->size();
            std::vector<GfVec3f> candidate;
            std::vector<GfVec3f> *output = pts;
            if (!discardable) {
                candidate = *pts;
                output = &candidate;
            }
            const bool solved =
                (run.device && handler->applyWindowDevice &&
                 handler->applyWindowDevice(external->data, ticks.data(),
                                            ticks.size(), output, run.cache)) ||
                handler->applyWindow(external->data, ticks.data(), ticks.size(),
                                     output, run.cache);
            if (!solved || output->size() != enteringCount)
                return false;
            for (const GfVec3f &point : *output) {
                if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                    !std::isfinite(point[2]))
                    return false;
            }
            if (!discardable)
                pts->swap(candidate);
            return true;
        }
        if (!handler || !handler->applyExternal)
            return false;
        const size_t enteringCount = pts->size();
        std::vector<GfVec3f> candidate;
        std::vector<GfVec3f> *output = pts;
        if (!discardable) {
            candidate = *pts;
            output = &candidate;
        }
        if (!handler->applyExternal(external->data, output) ||
            output->size() != enteringCount)
            return false;
        for (const GfVec3f &point : *output) {
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                !std::isfinite(point[2]))
                return false;
        }
        if (!discardable)
            pts->swap(candidate);
        return true;
    }
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        return RigExecApplyDerivedKernel(op, p, pts);
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
        // Matrix targets: evaluated by RigExecEvaluateProjectorTarget,
        // never through a point kernel.
        return false;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently skipped revision.
    return false;
}

} // namespace

bool
RigExecApplyRevisionKernel(RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           std::vector<GfVec3f> *pts, bool useSimd,
                           RigExecWireBasisCache *wireBasis,
                           RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                           const RigExecExternalRevision *external)
{
    return ApplyRevisionKernel(op, p, pts, useSimd, wireBasis, cache, false,
                               external);
}

bool
RigExecRevisionTakesSeparateBlend(RigExecRevisionOp op,
                                  const RigExecWeightPacket &w)
{
    return eval::RevisionTakesSeparateBlend(static_cast<int>(op),
                                            _EnvelopeView(w));
}

RigExecRevisionAcceptance
RigExecRevisionKernelAcceptance(RigExecRevisionOp op,
                                const RigExecMoverParameters &p, size_t count,
                                const bool *envelopeResolves)
{
    using Acceptance = RigExecRevisionAcceptance;
    if (!_PacketMatches(op, p)) {
        return Acceptance::Refuses;
    }
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return _MatrixKernelAccepts(p, count, nullptr) ? Acceptance::Applies
                                                       : Acceptance::Refuses;
    case RigExecRevisionOp::BlendShape:
        if (!_BlendShapeKernelAccepts(p, count, nullptr)) {
            return Acceptance::Refuses;
        }
        // The surface-frame transport reads the entering points.
        return p.blendSurfaceFrame ? Acceptance::Deferred : Acceptance::Applies;
    case RigExecRevisionOp::Wire: {
        const Acceptance wire = _WireKernelAcceptance(p, count);
        if (wire == Acceptance::Refuses) {
            return Acceptance::Refuses;
        }
        // The "apply once" envelope resolves at the full count or the
        // revision fails, whatever the kernel answered.
        if (RigExecRevisionTakesSeparateBlend(op, p.weights) &&
            !RigExecEnvelopeIsFullStrength(p.weights) &&
            !(envelopeResolves ? *envelopeResolves
                               : p.weights.ResolvesAll(count))) {
            return Acceptance::Refuses;
        }
        return wire;
    }
    case RigExecRevisionOp::Lattice:
        // A regular grid can refuse at any point it reads: only running it
        // answers.
        if (p.latticeSettings.regularGrid) {
            return Acceptance::Deferred;
        }
        // The kernel refuses only a rest-point count other than the entering
        // one; an invalid cage passes every point through. Then the "apply
        // once" envelope resolves at the full count or the revision fails.
        // Validated here whatever \p envelopeResolves says: RevisionStatic
        // resolves only a wire's envelope before it asks.
        if (p.restPoints.size() != count) {
            return Acceptance::Refuses;
        }
        if (!RigExecEnvelopeIsFullStrength(p.weights) &&
            !p.weights.ResolvesAll(count)) {
            return Acceptance::Refuses;
        }
        return Acceptance::Applies;
    case RigExecRevisionOp::Smooth:
        // The kernel never refuses: an invalid topology passes the points
        // through. Only the "apply once" envelope can fail the revision;
        // validated here, as a lattice's is, since RevisionStatic resolves
        // only a wire's before it asks.
        if (!RigExecEnvelopeIsFullStrength(p.weights) &&
            !p.weights.ResolvesAll(count)) {
            return Acceptance::Refuses;
        }
        return Acceptance::Applies;
    default:
        return Acceptance::Deferred;
    }
}

bool
RigExecRevisionIsRangeOp(RigExecRevisionOp op)
{
    return op == RigExecRevisionOp::Matrix || op == RigExecRevisionOp::Wire ||
           op == RigExecRevisionOp::Lattice || op == RigExecRevisionOp::Skin ||
           op == RigExecRevisionOp::BlendShape ||
           op == RigExecRevisionOp::Smooth ||
           op == RigExecRevisionOp::DeltaMush;
}

bool
RigExecRevisionGateHolds(RigExecRevisionOp op, const RigExecMoverParameters &p)
{
    // The kernel half of a group gate (gate.h): the packet facts alone,
    // with no written-group proof. The enable half stays with the caller.
    eval::GroupGateFacts facts;
    facts.enabled = true;
    facts.op = int(op);
    facts.envelope = _EnvelopeView(p.weights);
    facts.skinMethod = _SkinPacket(p).method;
    facts.blendSurfaceFrame = p.blendSurfaceFrame;
    facts.regularGrid = p.latticeSettings.regularGrid;
    return eval::GroupGateHolds(facts);
}

RigExecRevisionAcceptance
RigExecPrepareRevisionRanges(RigExecRevisionOp op,
                             const RigExecMoverParameters &p, size_t count,
                             RigExecWireBasisCache *wireBasis,
                             RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                             RigExecRevisionRangeInputs *prepared,
                             uint64_t restVersion)
{
    using Acceptance = RigExecRevisionAcceptance;
    prepared->wireBasis.reset();
    prepared->wireRestEvaluations = nullptr;
    prepared->latticeBasis = nullptr;
    prepared->latticeBind.reset();
    prepared->blendWeights.clear();
    prepared->mushRest.reset();
    if (op == RigExecRevisionOp::DeltaMush) {
        // What fails the whole kernel: the packet, the "apply once" envelope,
        // the rest data, the displacement and the computation map. Every
        // other failure is a point's own.
        if (!_PacketMatches(op, p) ||
            (!RigExecEnvelopeIsFullStrength(p.weights) &&
             !p.weights.ResolvesAll(count))) {
            return Acceptance::Refuses;
        }
        std::shared_ptr<const RigExecDeltaMushRestData<GfVec3f, GfVec3d>> rest;
        if (cache) {
            if (cache->MushRest(p.restPoints, p.topologyCounts,
                                p.topologyIndices, p.mushIterations, p.mushStep,
                                p.mushPinBorders, p.mushDistanceWeight,
                                p.mushSettings)) {
                rest = cache->RetainedMushRest();
            }
        } else {
            auto built =
                std::make_shared<RigExecDeltaMushRestData<GfVec3f, GfVec3d>>();
            if (RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(
                    p.restPoints, p.topologyCounts, p.topologyIndices,
                    p.mushIterations, p.mushStep, p.mushPinBorders,
                    p.mushDistanceWeight, p.mushSettings, built.get())) {
                rest = std::move(built);
            }
        }
        const double d = p.mushDisplacement;
        if (!rest || !rest->validated || rest->rest.size() != count ||
            !std::isfinite(d) ||
            (rest->settings.smoothing == RigExecDeltaMushSmoothing::Rest &&
             (d < 0 || d > 1)) ||
            !deltaMushDetail::ComputationMapValid(p.mushComputationToTarget)) {
            return Acceptance::Refuses;
        }
        prepared->mushRest = std::move(rest);
        return Acceptance::Applies;
    }
    if (op == RigExecRevisionOp::BlendShape) {
        // RigExecRevisionKernelAcceptance's BlendShape answer, by the
        // kernel's own validation; an envelope below full strength resolves
        // into the inputs by the same call.
        if (!_PacketMatches(op, p) ||
            !_BlendShapeKernelAccepts(p, count, &prepared->blendWeights)) {
            prepared->blendWeights.clear();
            return Acceptance::Refuses;
        }
        if (p.blendSurfaceFrame) {
            // The transport reads the entering points: Deferred, nothing
            // prepared.
            prepared->blendWeights.clear();
            return Acceptance::Deferred;
        }
        return Acceptance::Applies;
    }
    const Acceptance acceptance = RigExecRevisionKernelAcceptance(op, p, count);
    if (acceptance != Acceptance::Applies) {
        return acceptance;
    }
    if (op == RigExecRevisionOp::Wire) {
        // The memos ApplyRevisionKernel's wire arm reads, from the same calls.
        const RigExecWeightPacket &w = p.weights;
        if (RigExecWireTakesSparseEnvelope(w)) {
            if (wireBasis) {
                prepared->wireBasis = wireBasis->Get(p, w.indices, count);
            } else {
                auto built = std::make_shared<RigExecWireBasis>();
                if (RigExecBuildWireBasis(
                        p.wireBindCoords.cdata(), p.wireBindCoords.size(),
                        count, w.indices, p.curveOrder, p.curveKnots,
                        p.restPoints.size(), p.dropoffDistance, built.get())) {
                    prepared->wireBasis = std::move(built);
                }
            }
        } else if (wireBasis) {
            const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder,
                                         &p.curveKnots};
            prepared->wireRestEvaluations = wireBasis->restEvaluations.Get(
                rest, p.wireBindCoords.cdata(), p.wireBindCoords.size(),
                p.dropoffDistance);
        }
    } else if (op == RigExecRevisionOp::Lattice && cache) {
        // The basis RigExecApplyLatticeKernel looks up, asked exactly where
        // the kernel asks: past its pass-through checks, with its bind box.
        GfVec3f lo(0.0f), size(0.0f);
        if (RigExecLatticeKernelSetup(
                count, p.restPoints.data(), p.restPoints.size(),
                p.auxPoints.data(), p.auxPoints.size(), p.auxPointsB.data(),
                p.auxPointsB.size(), p.divisions[0], p.divisions[1],
                p.divisions[2], &lo, &size,
                static_cast<std::vector<GfVec3f> *>(nullptr))) {
            prepared->latticeBasis = cache->LatticeBasis(
                p.restPoints.data(), p.restPoints.size(), lo, size,
                p.divisions[0], p.divisions[1], p.divisions[2], restVersion);
            if (prepared->latticeBasis) {
                prepared->latticeBind = cache->RetainedLatticeBind();
            }
        }
    }
    return acceptance;
}

// Each arm is RunRevisionKernel restricted to [begin, end), reading the
// group's entering points at \p in and writing its result at \p out: the same
// kernel over the same packet, through the group forms of its loops, with
// every whole-array input indexed absolutely. A prepared input that is
// missing or sized for other points also answers false (an invariant
// violation the caller counts), never a guess.
bool
RigExecRunRevisionGroup(RigExecRevisionOp op, const RigExecMoverParameters &p,
                        const RigExecRevisionRangeInputs &prepared,
                        const RigExecSkinTransformsView *skin,
                        const GfVec3f *in, GfVec3f *out, size_t count,
                        size_t begin, size_t end, const float *separateEnvelope,
                        bool useSimd, bool *untouched,
                        const RigExecGroupPoints<GfVec3f> *halo,
                        RigExecDeltaMushGroupState<GfVec3f, GfVec3d> *mush)
{
    if (untouched) {
        *untouched = false;
    }
    if (begin > end || end > count ||
        (begin < end && (!in || !out || in == out)) ||
        !RigExecRevisionIsRangeOp(op) || op == RigExecRevisionOp::Matrix ||
        op == RigExecRevisionOp::Smooth || !_PacketMatches(op, p)) {
        return false;
    }
    const size_t n = end - begin;
    // The group's result is its entering points: reported, or copied when
    // the caller asked for the result in \p out.
    const auto passThrough = [&]() {
        if (untouched) {
            *untouched = true;
        } else {
            std::copy(in, in + n, out);
        }
        return true;
    };
    if (n == 0) {
        return passThrough();
    }
    // Every arm below that runs in place on \p out first seeds it with the
    // entering points, as RunRevisionKernel copies the whole array.
    const auto seed = [&]() { std::copy(in, in + n, out); };
    const RigExecWeightPacket &w = p.weights;
    if (op == RigExecRevisionOp::BlendShape) {
        // Target space only: the surface-frame transport reads every
        // entering point. The envelope is folded in per point, as the whole
        // kernel folds it: the delta added and blended back in one step.
        const bool fullStrength = RigExecEnvelopeIsFullStrength(w);
        if (p.blendSurfaceFrame || p.blendDeltas.size() != count ||
            (!fullStrength && prepared.blendWeights.size() != count)) {
            return false;
        }
        const GfVec3f *const delta = p.blendDeltas.data() + begin;
        const float *const weight =
            fullStrength ? nullptr : prepared.blendWeights.data() + begin;
        for (size_t k = 0; k < n; ++k) {
            const GfVec3f preceding = in[k];
            out[k] = RigExecBlendEnvelope(preceding, preceding + delta[k],
                                          fullStrength ? 1.0f : weight[k]);
        }
        return true;
    }
    if (op == RigExecRevisionOp::Wire && RigExecWireTakesSparseEnvelope(w)) {
        // The envelope is the walk's own: no separate blend.
        if (!prepared.wireBasis) {
            return false;
        }
        size_t first = 0, last = 0;
        _SparseEntriesIn(w.indices, begin, end, &first, &last);
        if (first == last) {
            return passThrough();
        }
        seed();
        return RigExecApplyWireBasisGroup(out, begin, end, *prepared.wireBasis,
                                          w.indices, w.values, p.restPoints,
                                          p.auxPoints);
    }
    // A dense wire, a lattice or a skin, blended over the entering points
    // afterwards unless the envelope is at full strength.
    const bool blend = !RigExecEnvelopeIsFullStrength(w);
    if (blend && !separateEnvelope) {
        return false;
    }
    if (op == RigExecRevisionOp::Wire) {
        seed();
        const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder,
                                     &p.curveKnots};
        const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                      &p.curveKnots};
        const std::vector<GfVec3f> *evaluations = prepared.wireRestEvaluations;
        if (!RigExecApplyWireGroup(out, begin, end, count, rest, posed,
                                   p.wireBindCoords.cdata(),
                                   p.wireBindCoords.size(), p.dropoffDistance,
                                   evaluations ? evaluations->data() : nullptr,
                                   evaluations ? evaluations->size() : 0)) {
            return false;
        }
    } else if (op == RigExecRevisionOp::Lattice) {
        // A regular grid runs whole (its revision is never Range), and the
        // group form is the Bernstein evaluation's.
        if (p.latticeSettings.regularGrid || p.restPoints.size() != count) {
            return false; // cardinality mismatch fails atomically
        }
        // Seeds \p out itself, which is all a refused setup leaves there.
        RigExecApplyLatticeKernelGroup(
            in, out, count, begin, end, p.restPoints.data(),
            p.restPoints.size(), p.auxPoints.data(), p.auxPoints.size(),
            p.auxPointsB.data(), p.auxPointsB.size(), p.divisions[0],
            p.divisions[1], p.divisions[2], prepared.latticeBasis);
    } else if (op == RigExecRevisionOp::DeltaMush) {
        if (!prepared.mushRest || !halo || !mush || halo->At(begin) != in ||
            !RigExecApplyDeltaMushGroup<GfVec3f, GfVec3d, GfMatrix4d>(
                prepared.mushRest, p.mushDisplacement,
                p.mushComputationToTarget, *halo, begin, end, out, mush)) {
            return false;
        }
    } else {
        // Skin: the group's part of the layout against the table it reads
        // (\p skin, or the packet's own), blended over the entering points
        // afterwards unless the envelope is at full strength.
        const RigExecSkinTransformsView view =
            skin ? *skin : RigExecSkinTransformsOf(p);
        return eval::ApplySkinGroupRange<_SkinMath>(
            _SkinPacket(p), !blend, separateEnvelope, _EvalSkinTransforms(view),
            in, count, begin, end, useSimd, out);
    }
    if (blend) {
        RigExecBlendEnvelopeRange(in, separateEnvelope + begin, 0, n, out);
    }
    return true;
}

bool
RigExecRunRevisionRange(RigExecRevisionOp op, const RigExecMoverParameters &p,
                        const RigExecRevisionRangeInputs &prepared,
                        const GfVec3f *entering, size_t count, size_t begin,
                        size_t end, const float *separateEnvelope,
                        std::vector<GfVec3f> *out, bool useSimd,
                        bool *untouched)
{
    if (untouched) {
        *untouched = false;
    }
    if (!out || out->size() != count || begin > end || end > count ||
        (count && !entering) || (count && entering == out->data())) {
        return false;
    }
    // The whole entering array as one group, for an op that reads neighbours.
    const int bounds[2] = {0, int(count)};
    const GfVec3f *const points[1] = {entering};
    RigExecGroupPoints<GfVec3f> whole;
    whole.bounds = bounds;
    whole.groups = 1;
    whole.points = points;
    RigExecDeltaMushGroupState<GfVec3f, GfVec3d> mush;
    return RigExecRunRevisionGroup(
        op, p, prepared, nullptr, entering + begin, out->data() + begin, count,
        begin, end, separateEnvelope, useSimd, untouched, &whole, &mush);
}

// One revision, envelope included: the packet check, the full-strength fast
// path, RigExecApplyRevisionKernel and the "apply once" blend against the
// preceding revision.
// ONE definition of "apply once", called by the mover-graph revision node and
// by the baked geometry loop. Two hand-written wrappers would have to agree
// about which operations blend and which fold the envelope into their own
// arithmetic, and the rigs that would show a disagreement are the ones no
// fixture happened to have.
namespace {
// \p source, when set, holds the \p sourceCount points entering the revision
// and \p pts receives the result out of place: the matrix kernel reads them
// where they are, and every other operation copies them into \p pts first,
// bit for bit, and then runs in place. The "apply once" blend reads the
// entering points from \p source rather than from a copy of them.
bool
RunRevisionKernel(RigExecRevisionOp op, const RigExecMoverParameters &p,
                  std::vector<GfVec3f> *pts, bool useSimd,
                  RigExecWireBasisCache *wireBasis,
                  RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                  bool discardable, const std::vector<float> *envelope,
                  const RigExecExternalRevision *external,
                  const GfVec3f *source = nullptr, size_t sourceCount = 0)
{
    if (!_PacketMatches(op, p)) {
        return false;
    }
    if (source) {
        if (op == RigExecRevisionOp::Matrix &&
            RigExecEnvelopeIsFullStrength(p.weights)) {
            pts->resize(sourceCount);
            ApplyMatrixFullStrength(p, source, pts->data(), sourceCount,
                                    useSimd);
            return true;
        }
        pts->assign(source, source + sourceCount);
    }
    if (!RigExecRevisionTakesSeparateBlend(op, p.weights)) {
        return ApplyRevisionKernel(op, p, pts, useSimd, wireBasis, cache,
                                   discardable, external);
    }

    // A delta mush whose envelope covers a small part of the mesh only needs
    // that part. The masked kernel is bit for bit the full kernel on the
    // envelope's support and leaves every other point as it entered, which is
    // what the blend below returns there anyway (zero weight returns the
    // preceding point). A whole-head finish limited to the eyelids costs a
    // tenth as much. It is substituted only where its verdict provably equals
    // the full kernel's (RigExecDeltaMushMaskPreservesVerdict), so hosts that
    // always run the full kernel -- binary playback, the uncached path --
    // still agree. The full kernel runs when the map is not the identity, no
    // rest cache exists, the support is half the mesh or more, or the verdict
    // cannot be established (vertex transport, extreme magnitudes).
    if (op == RigExecRevisionOp::DeltaMush && cache &&
        !RigExecEnvelopeIsFullStrength(p.weights) &&
        p.mushComputationToTarget == GfMatrix4d(1)) {
        const size_t count = pts->size();
        std::vector<float> resolved;
        const float *weights =
            envelope && envelope->size() == count ? envelope->data() : nullptr;
        if (!weights) {
            if (!p.weights.ResolveAll(count, &resolved)) {
                return false;
            }
            weights = resolved.data();
        }
        std::vector<char> active(count, 0);
        size_t activeCount = 0;
        for (size_t i = 0; i < count; ++i) {
            // The blend's own zero test: "weight <= 0" returns the preceding
            // point; anything else (including NaN) reaches the arithmetic.
            if (!(weights[i] <= 0.0f)) {
                active[i] = 1;
                ++activeCount;
            }
        }
        if (activeCount * 2 <= count) {
            const auto *rest = cache->MushRest(
                p.restPoints, p.topologyCounts, p.topologyIndices,
                p.mushIterations, p.mushStep, p.mushPinBorders,
                p.mushDistanceWeight, p.mushSettings);
            if (!rest) {
                return false;
            }
            if (RigExecDeltaMushMaskPreservesVerdict(
                    *pts, *rest, p.mushDisplacement, RigExecGeometryRanges())) {
                std::vector<GfVec3f> preceding;
                if (!source) {
                    preceding = *pts;
                }
                if (!RigExecApplyDeltaMushWithRestDataMasked<GfVec3f, GfVec3d>(
                        pts, *rest, p.mushDisplacement, active,
                        RigExecGeometryRanges(), cache->MushMaskMemo()) ||
                    pts->size() != count) {
                    return false;
                }
                RigExecBlendEnvelopeAll(source ? source : preceding.data(),
                                        weights, count, pts->data());
                return true;
            }
        }
    }

    // A constant envelope at exactly full strength makes the blend below the
    // identity at every point, so the copy of the preceding revision, the
    // resolved envelope array and the blend loop are all dead. The predicate
    // lives in moverGraph.h next to the packet it reads; see
    // RigExecEnvelopeIsFullStrength.
    const bool fullStrengthEnvelope = RigExecEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<GfVec3f> preceding;
    if (!fullStrengthEnvelope && !source) {
        preceding = *pts;
    }
    if (!ApplyRevisionKernel(op, p, pts, useSimd, wireBasis, cache, discardable,
                             external)) {
        return false;
    }
    if (pts->size() != precedingSize) {
        return false;
    }
    if (!fullStrengthEnvelope) {
        // \p envelope, when it covers these points, is this packet's already
        // resolved by the same ResolveAll (RevisionStatic); otherwise it
        // resolves here.
        std::vector<float> resolved;
        const float *weights = envelope && envelope->size() == pts->size()
                                   ? envelope->data()
                                   : nullptr;
        if (!weights) {
            if (!p.weights.ResolveAll(pts->size(), &resolved)) {
                return false;
            }
            weights = resolved.data();
        }
        RigExecBlendEnvelopeAll(source ? source : preceding.data(), weights,
                                pts->size(), pts->data());
    }
    return true;
}

} // namespace

bool
RigExecRunRevisionKernel(RigExecRevisionOp op, const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts, bool useSimd,
                         RigExecWireBasisCache *wireBasis,
                         RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                         const RigExecExternalRevision *external)
{
    return RunRevisionKernel(op, p, pts, useSimd, wireBasis, cache, false,
                             nullptr, external);
}

namespace geometryDetail {
bool
RunDiscardableRevisionKernel(RigExecRevisionOp op,
                             const RigExecMoverParameters &p,
                             std::vector<GfVec3f> *pts, bool useSimd,
                             RigExecWireBasisCache *wireBasis,
                             RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                             const std::vector<float> *envelope,
                             const RigExecExternalRevision *external)
{
    return RunRevisionKernel(op, p, pts, useSimd, wireBasis, cache, true,
                             envelope, external);
}
bool
RunDiscardableRevisionKernel(RigExecRevisionOp op,
                             const RigExecMoverParameters &p, const GfVec3f *in,
                             size_t count, std::vector<GfVec3f> *out,
                             bool useSimd, RigExecWireBasisCache *wireBasis,
                             RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache,
                             const std::vector<float> *envelope,
                             const RigExecExternalRevision *external)
{
    // An empty entering array may come with no storage at all.
    if (!in) {
        out->clear();
        return RunRevisionKernel(op, p, out, useSimd, wireBasis, cache, true,
                                 envelope, external);
    }
    return RunRevisionKernel(op, p, out, useSimd, wireBasis, cache, true,
                             envelope, external, in, count);
}
} // namespace geometryDetail

bool
RigExecRevisionBinding::operator==(const RigExecRevisionBinding &o) const
{
    return moverPath == o.moverPath && target == o.target &&
           transform == o.transform && transformSpace == o.transformSpace &&
           carrySpace == o.carrySpace && influences == o.influences &&
           weightObject == o.weightObject && base == o.base &&
           topologyCounts == o.topologyCounts &&
           topologyIndices == o.topologyIndices && cagePoints == o.cagePoints &&
           surfacePoints == o.surfacePoints && bindCoords == o.bindCoords &&
           driverFrames == o.driverFrames &&
           driverCurvePoints == o.driverCurvePoints &&
           driverCurveOrder == o.driverCurveOrder &&
           driverCurveKnots == o.driverCurveKnots &&
           driverTransformCount == o.driverTransformCount &&
           driverSpaceCount == o.driverSpaceCount &&
           driverBaseTransformCount == o.driverBaseTransformCount &&
           widths == o.widths && normalsPerCorner == o.normalsPerCorner &&
           normalsLeftHanded == o.normalsLeftHanded &&
           shaderDials == o.shaderDials &&
           meshWorldInverse == o.meshWorldInverse &&
           blendInputs == o.blendInputs && blendSamples == o.blendSamples &&
           phases == o.phases && transformPhase == o.transformPhase &&
           externalSchema == o.externalSchema &&
           externalCompileData == o.externalCompileData &&
           externalStructure == o.externalStructure &&
           externalCompileSources == o.externalCompileSources &&
           externalInputs == o.externalInputs;
}

std::optional<RigExecRevisionOp>
RigExecRevisionOpForSchema(const TfToken &schemaType, const TfToken &curveMode)
{
    // Resolved by the mover's own TU (see movers/): most movers name a
    // fixed op, the curve mover resolves its authored mode, and anything
    // without a row revises no point chain.
    const RigExecMoverHandler *handler = RigExecFindMoverHandler(schemaType);
    if (!handler) {
        return std::nullopt;
    }
    return handler->resolveOp(curveMode);
}

std::string
RigExecReadPhase::GetAsString() const
{
    switch (kind) {
    case RigExecReadPhaseKind::Base:
        return "base";
    case RigExecReadPhaseKind::Preceding:
        return "preceding";
    case RigExecReadPhaseKind::Final:
        return "final";
    case RigExecReadPhaseKind::AtPrim: {
        std::string text;
        if (!RigExecSpellPathText(prim, &text)) {
            text = prim.GetString();
        }
        return text;
    }
    }
    return "base";
}

bool
RigExecParseReadPhase(const std::string &authored, RigExecReadPhase *phase,
                      std::string *error)
{
    if (!phase) {
        return false;
    }
    if (authored.empty() || authored == "base") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Base, SdfPath()};
        return true;
    }
    if (authored == "preceding") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Preceding, SdfPath()};
        return true;
    }
    if (authored == "final") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Final, SdfPath()};
        return true;
    }
    // Anything else must be an absolute prim path. A relative path would have
    // to be resolved against something, and there are two equally plausible
    // somethings here (the mover, the target), so it is rejected rather than
    // guessed.
    if (!SdfPath::IsValidPathString(authored)) {
        if (error) {
            *error = "'" + authored +
                     "' is not base, preceding, final, or a valid prim path";
        }
        return false;
    }
    const SdfPath path(authored);
    if (!path.IsAbsolutePath() || !path.IsPrimPath()) {
        if (error) {
            *error = "'" + authored +
                     "' must be an ABSOLUTE prim path (or base, preceding, "
                     "final)";
        }
        return false;
    }
    *phase = RigExecReadPhase{RigExecReadPhaseKind::AtPrim, path};
    return true;
}

namespace {

// Interned once, at load: some readers resolve a phase every frame, and
// interning a name takes the token registry's lock.
const TfToken _readPhaseField(RigExecReadPhaseMetadataName);

} // namespace

bool
RigExecResolveReadPhase(const UsdObject &property, RigExecReadPhase *phase,
                        std::string *error)
{
    if (!phase) {
        return false;
    }
    *phase = RigExecReadPhase();
    if (!property.IsValid()) {
        return true;
    }
    std::string authored;
    if (!property.GetMetadata(_readPhaseField, &authored) || authored.empty()) {
        return true;
    }
    std::string why;
    if (!RigExecParseReadPhase(authored, phase, &why)) {
        if (error) {
            *error = property.GetPath().GetString() + ": " +
                     RigExecReadPhaseMetadataName + " " + why;
        }
        return false;
    }
    return true;
}

namespace {

TfToken
_Token(const UsdPrim &prim, const TfToken &attr, const TfToken &fallback)
{
    TfToken value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
        a.Get(&value);
    }
    return value;
}

// Every scalar mover input consults the generation's resolved property set
// before the authored stage. This is what lets a property-domain mover drive
// another mover's common envelope without a second evaluation model.
template <class T>
T
_Read(const UsdPrim &prim, const TfToken &attr, T fallback, UsdTimeCode time,
      const RigExecResolvedInputs *resolved)
{
    T value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
        if (resolved && resolved->GetAttribute(a, time, &value)) {
            return value;
        }
        a.Get(&value, time);
    }
    return value;
}

float
_Float(const UsdPrim &prim, const TfToken &attr, float fallback,
       UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    return _Read<float>(prim, attr, fallback, time, resolved);
}

// A scalar mover input read through its connections; a .rigexec runtime
// evaluates it over the file's input slots as a path read row.
template <class T>
T
_RecordedInput(const UsdPrim &prim, const TfToken &name, T fallback,
               UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    return _Read<T>(prim, name, fallback, time, resolved);
}

// Forward declaration: defined below.
bool _Enabled(const UsdPrim &prim, UsdTimeCode time,
              const RigExecResolvedInputs *resolved);

// A structural token on the mover (rigExec:weightBlend, rigExec:pointFrame,
// rigExec:driverDeltaFrame), read off the authored stage. The runtime and
// the frozen replay have no stage to ask; they replay this read (the file
// holds it as a path read), and a read nobody holds replays as the
// fallback. Guarded on the prim because the frozen replay calls the
// assemblers with none.
TfToken
_RecordedToken(const UsdPrim &prim, const TfToken &name,
               const TfToken &fallback, UsdTimeCode time)
{
    TfToken value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        a.Get(&value, time);
    }
    return value;
}

} // namespace

std::shared_ptr<const RigExecSkinTopology>
RigExecSkinTopologyCache::Resolve(
    const SdfPath &mover,
    const std::function<bool(RigExecSkinTopology *)> &build)
{
    // Held across the build as well as the lookup: concurrent chain tasks
    // would otherwise insert into the same map at once, and building the
    // same layout twice would only waste the read.
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _entries.find(mover);
    if (found != _entries.end()) {
        // Including a remembered refusal, which is a null entry.
        return found->second;
    }
    auto built = std::make_shared<RigExecSkinTopology>();
    if (!build(built.get())) {
        _entries.emplace(mover, nullptr);
        return nullptr;
    }
    // The layout this mover had before the last Clear(). If the fresh read
    // produced the same arrays -- which is what an edit anywhere else on the
    // stage produces -- hand back the SAME pointer, so the mover's packet
    // still compares equal and the per-point kernel does not re-run for a
    // binding that did not move. One array compare per notice, against one
    // kernel pass per notice.
    const auto candidate = _candidates.find(mover);
    if (candidate != _candidates.end() && candidate->second &&
        *candidate->second == *built) {
        return _entries.emplace(mover, candidate->second).first->second;
    }
    return _entries.emplace(mover, std::move(built)).first->second;
}

std::shared_ptr<const RigExecBlendSampleLayout>
RigExecBlendSampleCache::Resolve(
    const SdfPath &sample,
    const std::function<bool(RigExecBlendSampleLayout *)> &build)
{
    // Held across the build as well as the lookup, exactly as the skin
    // topology cache does: concurrent chain tasks would otherwise insert
    // into the same map at once.
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _entries.find(sample);
    if (found != _entries.end()) {
        // Including a remembered refusal, which is a null entry.
        return found->second;
    }
    auto built = std::make_shared<RigExecBlendSampleLayout>();
    if (!build(built.get())) {
        _entries.emplace(sample, nullptr);
        return nullptr;
    }
    const auto candidate = _candidates.find(sample);
    if (candidate != _candidates.end() && candidate->second &&
        *candidate->second == *built) {
        return _entries.emplace(sample, candidate->second).first->second;
    }
    return _entries.emplace(sample, std::move(built)).first->second;
}

std::shared_ptr<const RigExecBlendSampleLayout>
RigExecBlendSampleCache::AdoptExclusive(
    const SdfPath &sample,
    std::shared_ptr<const RigExecBlendSampleLayout> layout, bool refused)
{
    if (refused) {
        _entries[sample] = nullptr;
        return nullptr;
    }
    if (!layout)
        return nullptr;
    const auto active = _entries.find(sample);
    if (active != _entries.end() && active->second &&
        (active->second == layout ||
         RigExecSameBlendLayout(*active->second, *layout)))
        return active->second;
    const auto candidate = _candidates.find(sample);
    if (candidate != _candidates.end() && candidate->second &&
        RigExecSameBlendLayout(*candidate->second, *layout))
        layout = candidate->second;
    _entries[sample] = layout;
    return layout;
}

RigExecRevisionBinding
RigExecResolveRevisionBinding(const UsdPrim &moverPrim, const SdfPath &target,
                              const std::map<SdfPath, SdfPath> &frameChainHeads)
{
    RigExecRevisionBinding binding;
    if (!moverPrim) {
        return binding;
    }
    binding.moverPath = moverPrim.GetPath();
    binding.target = target;

    // Per-mover inputs, bound by the mover's own TU (see movers/). A
    // type with no row, or a row with no binder, binds nothing here.
    if (const RigExecMoverHandler *handler =
            RigExecFindMoverHandler(moverPrim.GetTypeName())) {
        binding.handler = handler;
        binding.externalSchema = moverPrim.GetTypeName();
        if (handler->bind) {
            const SdfPath ownerPath = target.GetPrimPath();
            const RigExecMoverBindContext ctx{moverPrim, target, ownerPath,
                                              frameChainHeads, &binding};
            handler->bind(ctx);
        }
        if (handler->declareExternalInputs) {
            const RigExecMoverBindContext ctx{moverPrim, target,
                                              target.GetPrimPath(),
                                              frameChainHeads, &binding};
            handler->declareExternalInputs(ctx, &binding.externalInputs);
        }
    }

    // Every point-chain mover may narrow its application with a weight object.
    const SdfPathVector weights =
        RigExecRelationshipTargets(moverPrim, _attrTokens->weightObjectRel);
    if (!weights.empty()) {
        binding.weightObject = weights[0];
    }

    return binding;
}

RigExecMoverParameters
RigExecAssembleMatrixParameters(const UsdPrim &moverPrim,
                                const GfMatrix4d *transform,
                                const RigExecWeightPacket *weights,
                                UsdTimeCode time,
                                const RigExecResolvedInputs *resolved)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->matrix;

    // Read in the assembly every path shares, not only in the exec builder:
    // the baked program fills these parameters through this function.
    params.radialWeight =
        _RecordedToken(moverPrim, _attrTokens->weightBlend, TfToken(), time) ==
        _valueTokens->radial;
    // The USD paths take rigExec:pointFrame from compile (it selects
    // RigExecClusterInPointFrame in the fold); it is read here too, and the
    // runtime's fold replays that read to make the same choice.
    _RecordedToken(moverPrim, _attrTokens->pointFrame, TfToken(), time);

    params.enabled = _Enabled(moverPrim, time, resolved);
    if (!params.enabled) {
        params.valid = true; // disabled is an ordinary pass-through
        return params;
    }

    if (!transform) {
        return params; // MoverFailed
    }
    params.weights = weights ? *weights
                             : RigExecWeightPacket::Constant(
                                   _Float(moverPrim, _attrTokens->defaultWeight,
                                          1.0f, time, resolved));
    if (!params.weights.valid) {
        return params; // invalid common envelope => MoverFailed
    }
    // The matrix must be finite and affine.
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite((*transform)[i][j])) {
                return params;
            }
        }
    }
    if ((*transform)[0][3] != 0 || (*transform)[1][3] != 0 ||
        (*transform)[2][3] != 0 || (*transform)[3][3] != 1) {
        return params;
    }
    params.transform = *transform;
    params.valid = true;
    return params;
}

RigExecMoverStatus
RigExecStatusForParameters(const RigExecMoverParameters &parameters,
                           const std::string &moverText)
{
    RigExecMoverStatus status;
    if (!parameters.enabled) {
        status.state = _valueTokens->disabled;
    } else if (parameters.valid) {
        status.state = _valueTokens->ok;
    } else {
        status.state = _valueTokens->moverFailed;
        // First bad canonical public address: reports the failed mover's
        // own path.
        status.firstBadAddress = moverText;
    }
    return status;
}

RigExecMoverStatus
RigExecStatusForParameters(const RigExecMoverParameters &parameters,
                           const SdfPath &moverPath)
{
    // Spelled only on the failure arm.
    if (parameters.enabled && !parameters.valid) {
        return RigExecStatusForParameters(parameters, moverPath.GetString());
    }
    return RigExecStatusForParameters(parameters, std::string());
}

namespace {

// An upstream array at \p path: authored-level, so it answers a read that
// bypasses the resolved inputs (rest data read at Default).
template <typename A>
bool
_StageUpstreamArray(const std::map<SdfPath, VtValue> *upstream,
                    const SdfPath &path, A *value)
{
    if (!upstream || path.IsEmpty()) {
        return false;
    }
    const auto it = upstream->find(path);
    if (it == upstream->end() || !it->second.IsHolding<A>()) {
        return false;
    }
    *value = it->second.UncheckedGet<A>();
    return true;
}

// Reads a typed array from an exact property path on the mover's stage.
// \p resolved is null for BIND-TIME reads and only for those. A rest cage, a
// rest geometry, a bind-time topology: those are the authored neutral pose the
// deformation is measured against, so a read phase has nothing to say about
// them -- and serving one the same phased value as the live read makes the two
// operands equal and the whole deformation an identity. \p upstream, when
// given, answers before the stage at any time.
template <typename T>
std::vector<T>
_Array(const UsdPrim &moverPrim, const SdfPath &path, UsdTimeCode time,
       const RigExecResolvedInputs *resolved = nullptr,
       const std::map<SdfPath, VtValue> *upstream = nullptr)
{
    std::vector<T> out;
    if (path.IsEmpty() || !moverPrim) {
        return out;
    }
    VtArray<T> value;
    if ((resolved && resolved->Get(path, &value)) ||
        _StageUpstreamArray(upstream, path, &value)) {
        out.assign(value.begin(), value.end());
        return out;
    }
    if (const UsdAttribute a = moverPrim.GetStage()->GetAttributeAtPath(path)) {
        // At the EVALUATED time, not Default: the kernels read these same
        // inputs through exec computeValue at the current time, so an
        // animated cage/surface/topology would otherwise silently diverge.
        a.Get(&value, time);
    }
    out.assign(value.begin(), value.end());
    return out;
}

bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved)
{
    bool enabled = true;
    const UsdAttribute a =
        prim ? prim.GetAttribute(_attrTokens->enabled) : UsdAttribute();
    if (a) {
        if (resolved && resolved->GetAttribute(a, time, &enabled)) {
            return enabled;
        }
        a.Get(&enabled, time);
    }
    return enabled;
}

// The extended deformer settings past their reads, shared by the stage and
// leaf assemblers so the two cannot disagree. Each answers false where its
// arm leaves the packet invalid. \p influences is the table of the frame
// providers binding.influences names, or null. Matrices are row-vector
// affine maps; the identity defaults reproduce the plain deformation.

// deltaMush: the smoothing settings and the computation-to-target map,
// carried by the frame provider when one is bound. A computation space
// other than the points' own needs authored rest points (the base is in the
// points' space) unless the mover only smooths.
bool
_DeltaMushSettings(const TfToken &smoothing, const TfToken &transport,
                   std::vector<float> smoothWeights, std::vector<int> edges,
                   bool onlySmooth, const GfMatrix4d &computationToTarget,
                   const RigExecRevisionBinding &binding,
                   const std::vector<GfMatrix4d> *influences,
                   RigExecMoverParameters *params)
{
    RigExecDeltaMushSettings &s = params->mushSettings;
    if (!RigExecParseDeltaMushSmoothing(smoothing.GetText(), &s.smoothing) ||
        !RigExecParseDeltaMushFrameTransport(transport.GetText(),
                                             &s.frameTransport)) {
        return false;
    }
    s.smoothWeights = std::move(smoothWeights);
    s.edges = std::move(edges);
    s.onlySmooth = onlySmooth;
    params->mushComputationToTarget = computationToTarget;
    const bool needsRest =
        computationToTarget != GfMatrix4d(1.0) || !binding.influences.empty();
    if (!binding.influences.empty()) {
        if (!influences || influences->size() != 1) {
            return false;
        }
        params->mushComputationToTarget *= (*influences)[0];
    }
    return !(params->restPoints.empty() && needsRest && !s.onlySmooth);
}

// Lattice, regular grid: the interpolations, the grid placement, the mask
// and the cage/target frames (each carried by its frame provider when two
// are bound) as coordinate maps in \p pointSpace.
bool
_LatticeGridSettings(const TfToken (&interpolation)[3], const GfVec3f &origin,
                     const GfVec3f &spacing, float strength,
                     std::vector<float> mask, GfMatrix4d cage,
                     GfMatrix4d target, const TfToken &pointSpace,
                     const RigExecRevisionBinding &binding,
                     const std::vector<GfMatrix4d> *influences,
                     RigExecMoverParameters *params)
{
    RigExecLatticeSettings &s = params->latticeSettings;
    for (int axis = 0; axis < 3; ++axis) {
        if (!RigExecLatticeInterpolationFromString(
                interpolation[axis].GetText(), &s.interpolation[axis])) {
            return false;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        s.origin[axis] = origin[axis];
        s.spacing[axis] = spacing[axis];
    }
    s.strength = strength;
    s.mask = std::move(mask);
    if (!RigExecSurfaceSnapValidMatrix(cage) ||
        !RigExecSurfaceSnapValidMatrix(target)) {
        return false;
    }
    if (!binding.influences.empty()) {
        if (!influences || influences->size() != 2) {
            return false;
        }
        cage *= (*influences)[0];
        target *= (*influences)[1];
        if (!RigExecSurfaceSnapCanonicalComputedMatrix(&cage) ||
            !RigExecSurfaceSnapCanonicalComputedMatrix(&target)) {
            return false;
        }
    }
    return RigExecLatticeCoordinateMaps(
        pointSpace.GetText(), cage, target, &params->targetToLattice,
        &params->latticeToTarget, &params->cageToLattice);
}

// Surface snap: the mode, the clearance offset, the mask, the explicit
// triangles and the surface/target frames (each carried by its frame
// provider when two are bound) as maps in \p pointSpace.
bool
_SurfaceSnapSettings(const TfToken &snapMode, float offset,
                     std::vector<float> mask, std::vector<int> triangles,
                     GfMatrix4d surface, GfMatrix4d target,
                     const TfToken &pointSpace,
                     const RigExecRevisionBinding &binding,
                     const std::vector<GfMatrix4d> *influences,
                     RigExecMoverParameters *params)
{
    RigExecSurfaceSnapSettings &s = params->surfaceSettings;
    if (snapMode == _valueTokens->onSurface) {
        s.mode = RigExecSurfaceSnapMode::OnSurface;
    } else if (snapMode == _valueTokens->inside) {
        s.mode = RigExecSurfaceSnapMode::Inside;
    } else if (snapMode == _valueTokens->outside) {
        s.mode = RigExecSurfaceSnapMode::Outside;
    } else if (snapMode == _valueTokens->outsideSurface) {
        s.mode = RigExecSurfaceSnapMode::OutsideSurface;
    } else {
        return false;
    }
    s.offset = offset;
    s.mask = std::move(mask);
    s.triangles = std::move(triangles);
    if (!RigExecSurfaceSnapValidMatrix(surface) ||
        !RigExecSurfaceSnapValidMatrix(target)) {
        return false;
    }
    if (!binding.influences.empty()) {
        if (!influences || influences->size() != 2) {
            return false;
        }
        surface *= (*influences)[0];
        target *= (*influences)[1];
        if (!RigExecSurfaceSnapCanonicalComputedMatrix(&surface) ||
            !RigExecSurfaceSnapCanonicalComputedMatrix(&target)) {
            return false;
        }
    }
    if (pointSpace == _valueTokens->local) {
        params->targetToSurface =
            target * RigExecSurfaceSnapAffineInverse(surface);
        params->surfaceToTarget =
            surface * RigExecSurfaceSnapAffineInverse(target);
    } else if (pointSpace == _valueTokens->common) {
        params->targetToSurface = RigExecSurfaceSnapAffineInverse(surface);
        params->surfaceToMetric = RigExecSurfaceSnapAffineInverse(surface);
        params->surfaceToTarget = surface;
    } else {
        return false;
    }
    return true;
}

// A surface projection packet's validity once its settings are read: a
// surface, faces or explicit triangles, a finite offset, and a mask (when
// given) over the \p basePointCount points it moves.
bool
_SurfaceProjectValid(const RigExecMoverParameters &params,
                     size_t basePointCount)
{
    const RigExecSurfaceSnapSettings &s = params.surfaceSettings;
    return !params.auxPoints.empty() &&
           (!params.topologyCounts.empty() || !s.triangles.empty()) &&
           std::isfinite(s.offset) &&
           (s.mask.empty() || s.mask.size() == basePointCount);
}

} // namespace

std::shared_ptr<const RigExecSkinTopology>
RigExecResolveSkinTopology(const UsdPrim &moverPrim, size_t influenceCount,
                           UsdTimeCode time,
                           const RigExecResolvedInputs *resolved,
                           RigExecSkinTopologyCache *cache)
{
    RIGEXEC_PURITY_CHECK();
    if (!cache || !moverPrim) {
        return nullptr;
    }
    const SdfPath primPath = moverPrim.GetPath();
    const SdfPath indicesPath =
        primPath.AppendProperty(_attrTokens->jointIndices);
    const SdfPath weightsPath =
        primPath.AppendProperty(_attrTokens->jointWeights);
    // The layout is compiled rig state, so everything about it that does not involve
    // the influence MATRICES is settled once and shared: the two array reads,
    // the copy into the packet, and the per-element range and weight checks.
    // What is left per frame is the matrix table -- which is the only part of
    // the layout an animated rig changes.
    return cache->Resolve(primPath, [&](RigExecSkinTopology *topology) {
        // Whether the layout is compiled rig state is re-asked HERE, where the
        // cache is filled, and not only where the graph was compiled.
        // Authoring a time sample on jointWeights (or connecting it)
        // moves no compiled rig signature, so it does not recompile; it does send a
        // notice, and a notice clears this cache -- so this is the one
        // place that sees the stage as it is now. Refusing puts the
        // packet back on the per-frame arrays, which is what Compile
        // would have done had the sample been there. Costs one answer per
        // notice.
        if (!RigExecSkinLayoutIsFixed(moverPrim)) {
            return false;
        }
        // This build fills the compiled rig cache (a varying layout is
        // refused above), so these reads are compiled rig state the file's
        // skin topology carries.
        const std::vector<int> indices =
            _Array<int>(moverPrim, indicesPath, time, resolved);
        const std::vector<float> weights =
            _Array<float>(moverPrim, weightsPath, time, resolved);
        int elementSize = 1;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->elementSize)) {
            if (!resolved || !resolved->GetAttribute(a, time, &elementSize)) {
                a.Get(&elementSize, time);
            }
        }
        RigExecBuildSkinTopology(indices, weights, elementSize, influenceCount,
                                 topology);
        return true;
    });
}

namespace {

// Whether none of \p names on \p moverPrim might vary with time or has an
// authored connection; false for an invalid prim.
bool
_SkinLayoutAttributesAreFixed(const UsdPrim &moverPrim,
                              std::initializer_list<TfToken> names)
{
    if (!moverPrim) {
        return false;
    }
    for (const TfToken &name : names) {
        const UsdAttribute a = moverPrim.GetAttribute(name);
        if (a && (a.ValueMightBeTimeVarying() || a.HasAuthoredConnections())) {
            return false;
        }
    }
    return true;
}

} // namespace

bool
RigExecSkinLayoutIsFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(moverPrim, {_attrTokens->jointIndices,
                                                     _attrTokens->jointWeights,
                                                     _attrTokens->elementSize});
}

bool
RigExecSkinLayoutTopologyIsFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(
        moverPrim, {_attrTokens->jointIndices, _attrTokens->elementSize});
}

bool
RigExecSkinLayoutWeightsAreFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(moverPrim,
                                         {_attrTokens->jointWeights});
}

void
RigExecBuildSkinTopology(TfSpan<const int> indices, TfSpan<const float> weights,
                         int elementSize, size_t influenceCount,
                         RigExecSkinTopology *topology)
{
    topology->indices.assign(indices.begin(), indices.end());
    topology->weights.assign(weights.begin(), weights.end());
    topology->elementSize = elementSize;
    topology->influenceCount = influenceCount;
    if (topology->elementSize < 1 ||
        topology->weights.size() != topology->indices.size() ||
        topology->indices.size() % size_t(topology->elementSize) != 0) {
        return;
    }
    topology->pointCount =
        topology->indices.size() / size_t(topology->elementSize);
    // Exactly RigExecSkinLayout::Validate's shape, range and weight rules,
    // against a table whose SIZE is compiled rig state. The matrices themselves are
    // checked every frame by the caller.
    if (topology->influenceCount == 0) {
        return;
    }
    for (size_t i = 0; i < topology->indices.size(); ++i) {
        const int index = topology->indices[i];
        if (index < 0 || size_t(index) >= topology->influenceCount) {
            return;
        }
        const float weight = topology->weights[i];
        if (!std::isfinite(weight) || weight < 0.0f) {
            return;
        }
    }
    topology->validated = true;
}

RigExecMoverParameters
RigExecAssembleSkinParameters(
    const UsdPrim &moverPrim,
    const std::vector<GfMatrix4d> *influenceTransforms,
    const RigExecWeightPacket *weights, UsdTimeCode time,
    const RigExecResolvedInputs *resolved,
    RigExecSkinTopologyCache *topologyCache,
    const std::shared_ptr<const RigExecSkinTopology> *resolvedTopology)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->skin;
    params.enabled = _Enabled(moverPrim, time, resolved);
    if (!params.enabled) {
        params.valid = true; // disabled is an ordinary pass-through
        return params;
    }
    if (!moverPrim || !influenceTransforms) {
        return params; // MoverFailed
    }
    params.weights = weights ? *weights
                             : RigExecWeightPacket::Constant(
                                   _Float(moverPrim, _attrTokens->defaultWeight,
                                          1.0f, time, resolved));
    if (!params.weights.valid) {
        return params; // invalid common envelope => MoverFailed
    }

    params.skinTransforms = *influenceTransforms;
    const SdfPath primPath = moverPrim.GetPath();
    const SdfPath indicesPath =
        primPath.AppendProperty(_attrTokens->jointIndices);
    const SdfPath weightsPath =
        primPath.AppendProperty(_attrTokens->jointWeights);
    const auto readElementSize = [&moverPrim, time, resolved](int *out) {
        *out = 1;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->elementSize)) {
            if (!resolved || !resolved->GetAttribute(a, time, out)) {
                a.Get(out, time);
            }
        }
    };
    params.skinningMethod = _valueTokens->classicLinear;
    if (const UsdAttribute a =
            moverPrim.GetAttribute(_attrTokens->skinningMethod)) {
        if (!resolved ||
            !resolved->GetAttribute(a, time, &params.skinningMethod)) {
            a.Get(&params.skinningMethod, time);
        }
    }

    if (resolvedTopology) {
        // Already answered by a caller that may not take the cache's lock
        // where it assembles.
        params.skinTopology = *resolvedTopology;
    } else if (topologyCache) {
        params.skinTopology =
            RigExecResolveSkinTopology(moverPrim, influenceTransforms->size(),
                                       time, resolved, topologyCache);
    }
    // Null means the cache refused this mover -- the layout can move within
    // the compiled rig after all -- so the packet falls through to the per-frame
    // arrays below.
    if (params.skinTopology) {
        params.skinElementSize = params.skinTopology->elementSize;
        if (!params.skinTopology->validated ||
            params.skinTransforms.size() !=
                params.skinTopology->influenceCount) {
            return params;
        }
        // The frame half of RigExecSkinLayout::Validate.
        for (const GfMatrix4d &m : params.skinTransforms) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (!std::isfinite(m[r][c])) {
                        return params;
                    }
                }
            }
            if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
                return params;
            }
        }
        if (params.skinningMethod != "classicLinear" &&
            params.skinningMethod != "dualQuaternion") {
            return params;
        }
        params.valid = true;
        return params;
    }

    // The cache refused this mover, so the layout reads per frame.
    params.skinIndices = _Array<int>(moverPrim, indicesPath, time, resolved);
    params.skinWeights = _Array<float>(moverPrim, weightsPath, time, resolved);
    readElementSize(&params.skinElementSize);

    // The point count is not known here, so the layout is checked against
    // its own length; the kernel re-checks against the points it receives.
    // Validated now rather than only in the kernel so a bad packet reports
    // MoverFailed through the status rather than by a silent pass-through.
    if (params.skinElementSize < 1 ||
        params.skinWeights.size() != params.skinIndices.size() ||
        params.skinIndices.size() % size_t(params.skinElementSize) != 0) {
        return params;
    }
    RigExecSkinLayout layout;
    layout.transforms = params.skinTransforms.data();
    layout.transformCount = params.skinTransforms.size();
    layout.indices = params.skinIndices.data();
    layout.weights = params.skinWeights.data();
    layout.indexCount = params.skinIndices.size();
    layout.elementSize = size_t(params.skinElementSize);
    layout.pointCount = layout.indexCount / layout.elementSize;
    if (!layout.Validate()) {
        return params;
    }
    // Both declared tokens assemble; the kernel is what has (or lacks) a
    // branch for them, and the compiler is what tells the author.
    if (params.skinningMethod != "classicLinear" &&
        params.skinningMethod != "dualQuaternion") {
        return params;
    }
    params.valid = true;
    return params;
}

namespace {

// The native view of a blend sample for blendShapeKernel.h.
struct _BlendSampleTraits {
    static float Activation(const RigExecBlendSampleData &s)
    {
        return s.activation;
    }
    static const RigExecBlendSampleLayout *
    Layout(const RigExecBlendSampleData &s)
    {
        return s.layout.get();
    }
    static const GfVec3f *DensePoints(const RigExecBlendSampleData &s)
    {
        return s.PointData();
    }
    static size_t DenseCount(const RigExecBlendSampleData &s)
    {
        return s.PointCount();
    }
};

} // namespace

bool
RigExecSumBlendChannels(const std::vector<RigExecBlendChannel> &channels,
                        const GfVec3f *base, size_t baseCount,
                        std::vector<GfVec3f> *deltas)
{
    // The frozen assembler calls this too, and a frozen frame runs on its
    // own thread alone.
    const auto forRange = [](size_t n, const auto &fn) {
        if (RigExecParallelEvaluationEnabled() &&
            !RigExecFrozenSerialActive() &&
            n >= RigExecGeometryParallelThreshold)
            WorkParallelForN(n, fn, RigExecGeometryGrainSize);
        else
            fn(0, n);
    };
    return RigExecSumBlendChannelsKernel<_BlendSampleTraits>(
        channels, [](const RigExecBlendChannel &c) { return c.weight; }, base,
        baseCount, deltas, forRange);
}

bool
RigExecSumBlendChannels(const std::vector<RigExecBlendChannel> &channels,
                        const std::vector<GfVec3f> &base,
                        std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels, base.data(), base.size(), deltas);
}

RigExecMoverParameters
RigExecAssembleParameters(const UsdPrim &moverPrim, RigExecRevisionOp op,
                          const RigExecRevisionBinding &binding,
                          const RigExecProviderValues &values, UsdTimeCode time,
                          RigExecExternalRevision *external)
{
    RIGEXEC_PURITY_CHECK();
    if (op == RigExecRevisionOp::Matrix) {
        return RigExecAssembleMatrixParameters(
            moverPrim, values.transform, values.weights, time, values.resolved);
    }
    if (op == RigExecRevisionOp::Skin) {
        return RigExecAssembleSkinParameters(
            moverPrim, values.influenceTransforms, values.weights, time,
            values.resolved, values.skinTopologyCache, values.skinTopology);
    }
    RigExecMoverParameters params;
    const bool synthesizedDerived = op == RigExecRevisionOp::RecomputeNormals ||
                                    op == RigExecRevisionOp::RecomputeExtent;
    // Derived maintenance has no authored mover. The owning gprim is only a
    // convenient topology/property source and must not accidentally acquire
    // mover semantics from custom attributes with familiar names.
    params.enabled =
        synthesizedDerived ? true : _Enabled(moverPrim, time, values.resolved);

    switch (op) {
    case RigExecRevisionOp::BlendShape:
        params.kind = _kindTokens->blendShape;
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params.kind = _kindTokens->volumeCorrect;
        break;
    case RigExecRevisionOp::Smooth:
        params.kind = _kindTokens->smooth;
        break;
    case RigExecRevisionOp::DeltaMush:
        params.kind = _kindTokens->deltaMush;
        break;
    case RigExecRevisionOp::Wrinkle:
        params.kind = _kindTokens->wrinkle;
        break;
    case RigExecRevisionOp::Lattice:
        params.kind = _kindTokens->lattice;
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.kind = _kindTokens->surfaceProject;
        break;
    case RigExecRevisionOp::Ribbon:
        params.kind = _kindTokens->ribbon;
        break;
    case RigExecRevisionOp::Wire:
        params.kind = _kindTokens->wire;
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params.kind = _kindTokens->emitGuidePoints;
        break;
    case RigExecRevisionOp::External:
        params.kind = _kindTokens->external;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = _kindTokens->recomputeNormals;
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = _kindTokens->recomputeExtent;
        break;
    case RigExecRevisionOp::SurfaceProjector:
        params.kind = _kindTokens->surfaceProjector;
        break;
    case RigExecRevisionOp::ShaderDials:
        params.kind = _kindTokens->shaderDials;
        break;
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break; // handled above
    }

    if (!params.enabled) {
        params.valid = true; // disabled is an ordinary pass-through
        return params;
    }

    // One envelope contract for every operation. A bound object is the total
    // field and supersedes the scalar fallback; without one, synthesize the
    // normalized constant packet supplied by RigExecMoverAPI.
    params.weights =
        synthesizedDerived
            ? RigExecWeightPacket::Constant(1.0f)
            : (values.weights ? *values.weights
                              : RigExecWeightPacket::Constant(_Float(
                                    moverPrim, _attrTokens->defaultWeight, 1.0f,
                                    time, values.resolved)));
    if (!params.weights.valid) {
        return params; // MoverFailed, preserving the preceding revision
    }

    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        if (values.lendBlendDeltas) {
            params.blendDeltas.swap(values.blendDeltas);
        } else {
            params.blendDeltas = values.blendDeltas;
        }
        const TfToken space =
            _Token(moverPrim, _attrTokens->deltaSpace, _valueTokens->target);
        if (space != "target" && space != "surfaceFrame")
            break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            params.restPoints = values.basePoints;
            params.topologyCounts = _Array<int>(
                moverPrim, binding.topologyCounts, time, values.resolved);
            params.topologyIndices = _Array<int>(
                moverPrim, binding.topologyIndices, time, values.resolved);
            if (params.topologyCounts.empty())
                break;
        }
        params.valid = values.blendFromChannels || !params.blendDeltas.empty();
        break;
    }

    case RigExecRevisionOp::VolumeCorrect:
        params.strength = 1.0f;
        // The correction reference is the bound volume of the authored base.
        if (!values.basePoints.empty()) {
            params.referenceVolume = RigExecBoundVolume(
                values.basePoints.data(), values.basePoints.size());
            params.valid = true;
        }
        break;

    case RigExecRevisionOp::Smooth:
        params.strength = 1.0f;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
                                            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
                                             time, values.resolved);
        params.valid = !params.topologyCounts.empty();
        break;

    case RigExecRevisionOp::DeltaMush: {
        params.restPoints = _Array<GfVec3f>(
            moverPrim,
            moverPrim.GetPath().AppendProperty(_attrTokens->restPoints),
            UsdTimeCode::Default(), values.resolved);
        // The extended settings; their defaults are the plain deformation.
        if (!_DeltaMushSettings(
                _RecordedInput<TfToken>(
                    moverPrim, _attrTokens->smoothing, _valueTokens->rest,
                    UsdTimeCode::Default(), values.resolved),
                _RecordedInput<TfToken>(moverPrim, _attrTokens->frameTransport,
                                        _valueTokens->vertex,
                                        UsdTimeCode::Default(),
                                        values.resolved),
                _Array<float>(moverPrim,
                              moverPrim.GetPath().AppendProperty(
                                  _attrTokens->smoothWeights),
                              time, values.resolved),
                _Array<int>(
                    moverPrim,
                    moverPrim.GetPath().AppendProperty(_attrTokens->edges),
                    UsdTimeCode::Default(), values.resolved),
                _RecordedInput<bool>(moverPrim, _attrTokens->onlySmooth, false,
                                     time, values.resolved),
                _RecordedInput<GfMatrix4d>(
                    moverPrim, _attrTokens->computationToTarget,
                    GfMatrix4d(1.0), time, values.resolved),
                binding, values.influenceTransforms, &params)) {
            break;
        }
        if (params.restPoints.empty()) {
            params.restPoints = values.basePoints;
        }
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
                                            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
                                             time, values.resolved);
        params.mushIterations = _RecordedInput<int>(
            moverPrim, _attrTokens->iterations, 10, time, values.resolved);
        params.mushStep = _RecordedInput<float>(moverPrim, _attrTokens->step,
                                                0.5f, time, values.resolved);
        params.mushPinBorders = _RecordedInput<bool>(
            moverPrim, _attrTokens->pinBorders, true, time, values.resolved);
        params.mushDistanceWeight =
            _RecordedInput<float>(moverPrim, _attrTokens->distanceWeight, 0.0f,
                                  time, values.resolved);
        params.mushDisplacement = _RecordedInput<float>(
            moverPrim, _attrTokens->displacement, 1.0f, time, values.resolved);
        params.valid =
            !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RigExecRevisionOp::Wrinkle: {
        params.restPoints = _Array<GfVec3f>(
            moverPrim,
            moverPrim.GetPath().AppendProperty(_attrTokens->restPoints),
            UsdTimeCode::Default(), values.resolved);
        if (params.restPoints.empty())
            params.restPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
                                            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
                                             time, values.resolved);
        auto &settings = params.wrinkleSettings;
        const TfToken topology = _RecordedInput<TfToken>(
            moverPrim, _attrTokens->topology, _valueTokens->cloth,
            UsdTimeCode::Default(), values.resolved);
        if (topology != _valueTokens->cloth &&
            topology != _valueTokens->surfaceStruts)
            break;
        settings.topology = topology == _valueTokens->cloth
                                ? RigExecWrinkleTopology::Cloth
                                : RigExecWrinkleTopology::SurfaceStruts;
        settings.pinPoints = _Array<int>(
            moverPrim,
            moverPrim.GetPath().AppendProperty(_attrTokens->pinPoints),
            UsdTimeCode::Default(), values.resolved);
        settings.iterations = _RecordedInput<int>(
            moverPrim, _attrTokens->iterations, 80, time, values.resolved);
        settings.neighborDistance = _RecordedInput<int>(
            moverPrim, _attrTokens->neighborDistance, 2, time, values.resolved);
        settings.restLengthScale =
            _RecordedInput<float>(moverPrim, _attrTokens->restLengthScale, 1.0f,
                                  time, values.resolved);
        settings.stretchStiffness =
            _RecordedInput<float>(moverPrim, _attrTokens->stretchStiffness,
                                  1.0f, time, values.resolved);
        settings.compressionStiffness =
            _RecordedInput<float>(moverPrim, _attrTokens->compressionStiffness,
                                  1.0f, time, values.resolved);
        settings.bendStiffness = _RecordedInput<float>(
            moverPrim, _attrTokens->bendStiffness, 0.1f, time, values.resolved);
        settings.maxDisplacement =
            _RecordedInput<float>(moverPrim, _attrTokens->maxDisplacement, 0.2f,
                                  time, values.resolved);
        settings.pinBorders = _RecordedInput<bool>(
            moverPrim, _attrTokens->pinBorders, true, time, values.resolved);
        settings.tangentPlaneCollisions =
            _RecordedInput<bool>(moverPrim, _attrTokens->tangentPlaneCollisions,
                                 true, time, values.resolved);
        settings.tangentPlaneInset =
            _RecordedInput<float>(moverPrim, _attrTokens->tangentPlaneInset,
                                  0.0f, time, values.resolved);
        settings.wrinkleScale = _RecordedInput<float>(
            moverPrim, _attrTokens->wrinkleScale, 1.0f, time, values.resolved);
        settings.smoothingIterations =
            _RecordedInput<int>(moverPrim, _attrTokens->smoothingIterations, 0,
                                time, values.resolved);
        params.valid =
            !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RigExecRevisionOp::External: {
        RigExecExternalPayload payload;
        RigExecAssembleExternalPayload(moverPrim, binding, values, time,
                                       &payload);
        params.valid = payload.valid;
        if (external) {
            external->handler = payload.handler;
            external->schema = payload.schema;
            external->data = std::move(payload.data);
        }
        break;
    }

    case RigExecRevisionOp::Lattice: {
        // rigExec:evaluation picks the Bernstein cage ("legacy") or the
        // regular grid; only the grid reads the settings after it.
        const TfToken evaluation = _RecordedInput<TfToken>(
            moverPrim, _attrTokens->evaluation, _valueTokens->legacy, time,
            values.resolved);
        if (evaluation != _valueTokens->legacy &&
            evaluation != _valueTokens->regularGrid)
            break;
        params.latticeSettings.regularGrid =
            evaluation == _valueTokens->regularGrid;
        if (params.latticeSettings.regularGrid) {
            const auto read = [&](const TfToken &name, auto fallback) {
                return _RecordedInput<decltype(fallback)>(
                    moverPrim, name, fallback, time, values.resolved);
            };
            const TfToken interpolation[3] = {
                read(_attrTokens->interpolationU, _valueTokens->bspline),
                read(_attrTokens->interpolationV, _valueTokens->bspline),
                read(_attrTokens->interpolationW, _valueTokens->bspline)};
            const VtFloatArray mask = read(_attrTokens->mask, VtFloatArray());
            if (!_LatticeGridSettings(
                    interpolation, read(_attrTokens->origin, GfVec3f(-0.5f)),
                    read(_attrTokens->spacing, GfVec3f(1.0f)),
                    read(_attrTokens->strength, 1.0f),
                    std::vector<float>(mask.begin(), mask.end()),
                    read(_attrTokens->cageMatrix, GfMatrix4d(1.0)),
                    read(_attrTokens->targetMatrix, GfMatrix4d(1.0)),
                    read(_attrTokens->pointSpace, _valueTokens->local), binding,
                    values.influenceTransforms, &params)) {
                break;
            }
        }
        if (values.retainedRest && !values.retainedRest->empty()) {
            params.restPoints.swap(*values.retainedRest);
        } else {
            params.restPoints = values.basePoints;
        }
        // Operand order matters and is not symmetric: the shared applier is
        // RigExecApplyLattice(points, restPoints, restCage, posedCage, divs),
        // so auxPoints is the BIND-TIME cage and auxPointsB the live one --
        // matching _BuildLatticeMoverParameters. Reversing them is invisible
        // at rest (the two cages are equal) and inverts the deformation as
        // soon as the cage moves.
        // The rest cage is the cage at Default time, read directly; nothing
        // is authored for it. The live cage is the same attribute at the evaluated time.
        params.auxPoints = _Array<GfVec3f>(
            moverPrim, binding.cagePoints, UsdTimeCode::Default(),
            /*resolved=*/nullptr, values.upstream);
        params.auxPointsB = _Array<GfVec3f>(moverPrim, binding.cagePoints, time,
                                            values.resolved);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->divisions)) {
            a.Get(&params.divisions, time);
        }
        // Same cardinality contract as the kernel: a cage that does not match
        // the declared lattice resolution fails atomically instead of
        // indexing garbage.
        const size_t cageCount = size_t(params.divisions[0]) *
                                 size_t(params.divisions[1]) *
                                 size_t(params.divisions[2]);
        const int minimum = params.latticeSettings.regularGrid ? 1 : 2;
        params.valid =
            params.divisions[0] >= minimum && params.divisions[1] >= minimum &&
            params.divisions[2] >= minimum &&
            (params.latticeSettings.regularGrid ||
             params.auxPoints.size() == cageCount) &&
            params.auxPointsB.size() == cageCount && !params.restPoints.empty();
        break;
    }

    case RigExecRevisionOp::SurfaceProject: {
        // Fixed at full, matching _BuildSurfaceMoverParameters:
        // attach/project maps fully. RigExecSurfaceMover declares no
        // inputs:strength.
        params.strength = 1.0f;
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.surfacePoints,
                                           time, values.resolved);
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
                                            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
                                             time, values.resolved);
        {
            const auto read = [&](const TfToken &name, auto fallback) {
                return _RecordedInput<decltype(fallback)>(
                    moverPrim, name, fallback, time, values.resolved);
            };
            const VtFloatArray mask = read(_attrTokens->mask, VtFloatArray());
            const VtIntArray triangles =
                read(_attrTokens->triangles, VtIntArray());
            if (!_SurfaceSnapSettings(
                    read(_attrTokens->snapMode, _valueTokens->onSurface),
                    read(_attrTokens->offset, 0.0f),
                    std::vector<float>(mask.begin(), mask.end()),
                    std::vector<int>(triangles.begin(), triangles.end()),
                    read(_attrTokens->surfaceMatrix, GfMatrix4d(1.0)),
                    read(_attrTokens->targetMatrix, GfMatrix4d(1.0)),
                    read(_attrTokens->pointSpace, _valueTokens->local), binding,
                    values.influenceTransforms, &params)) {
                break;
            }
        }
        params.valid = _SurfaceProjectValid(params, values.BasePointCount());
        break;
    }

    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints: {
        const RigExecPointFrameArray *frames = values.driverFrames;
        if (!frames || frames->IsEmpty() ||
            frames->rests.size() != frames->GetSize()) {
            break; // MoverFailed
        }
        params.frames = *frames;
        if (op == RigExecRevisionOp::Ribbon) {
            params.bindCoords = _Array<GfVec2f>(moverPrim, binding.bindCoords,
                                                time, values.resolved);
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }

    case RigExecRevisionOp::Wire: {
        // Posed control points at the declared phase; rest control points
        // are the curve's AUTHORED ones, which is what the bind coordinates
        // were computed against.
        params.restPoints = _Array<GfVec3f>(
            moverPrim, binding.driverCurvePoints, UsdTimeCode::Default(),
            /*resolved=*/nullptr, values.upstream);
        if (binding.driverTransformCount > 0) {
            // Posed control points from the providers: C_j = C0_j +
            // w_j (M_j C0_j - C0_j), M_j the transform measured in its
            // space. A periodic curve's repeated points wrap onto the
            // unique ones, so one entry per unique point is enough.
            const size_t t = size_t(binding.driverTransformCount);
            const size_t s = size_t(binding.driverSpaceCount);
            const size_t bt = size_t(binding.driverBaseTransformCount);
            const std::vector<GfMatrix4d> *table = values.influenceTransforms;
            if (!table || table->size() < t + s + bt) {
                break; // MoverFailed
            }
            const auto floats = [&](const TfToken &name) {
                VtFloatArray out;
                if (const UsdAttribute a = moverPrim.GetAttribute(name)) {
                    if (!values.resolved ||
                        !values.resolved->GetAttribute(a, time, &out)) {
                        a.Get(&out, time);
                    }
                }
                return out;
            };
            const VtFloatArray weights = floats(_attrTokens->driverWeights);
            const VtFloatArray baseWeights =
                floats(_attrTokens->driverBaseWeights);
            // Which frame the wire's points are already in, and which frame
            // the driver's offset is applied in. The runtime and the frozen
            // replay read the same tokens, so they make the same choice.
            RigExecWireDriverFrame frame;
            frame.posedPoints =
                _RecordedToken(moverPrim, _attrTokens->pointFrame,
                               _valueTokens->rest, time) == "posed";
            frame.posedDelta =
                _RecordedToken(moverPrim, _attrTokens->driverDeltaFrame,
                               _valueTokens->local, time) == "posed";
            frame.carry = values.carry;
            RigExecPoseWireDrivers(*table, t, s, bt, weights, baseWeights,
                                   frame, &params.restPoints,
                                   &params.auxPoints);
        } else {
            params.auxPoints = _Array<GfVec3f>(
                moverPrim, binding.driverCurvePoints, time, values.resolved);
        }
        {
            VtIntArray order;
            if (_StageUpstreamArray(values.upstream, binding.driverCurveOrder,
                                    &order)) {
                if (!order.empty()) {
                    params.curveOrder = order[0];
                }
            } else if (const UsdAttribute a =
                           moverPrim.GetStage()->GetAttributeAtPath(
                               binding.driverCurveOrder)) {
                if (a.Get(&order, UsdTimeCode::Default()) && !order.empty()) {
                    params.curveOrder = order[0];
                }
            }
        }
        params.curveKnots = _Array<double>(
            moverPrim, binding.driverCurveKnots, UsdTimeCode::Default(),
            /*resolved=*/nullptr, values.upstream);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->dropoffDistance)) {
            float dropoff = 0.0f;
            a.Get(&dropoff, time);
            params.dropoffDistance = dropoff;
        }
        if (!binding.bindCoords.IsEmpty()) {
            if (!values.resolved ||
                !values.resolved->Get(binding.bindCoords,
                                      &params.wireBindCoords)) {
                if (const UsdAttribute a =
                        moverPrim.GetStage()->GetAttributeAtPath(
                            binding.bindCoords)) {
                    a.Get(&params.wireBindCoords, time);
                }
            }
        }
        const RigExecNurbsCurve rest{&params.restPoints, params.curveOrder,
                                     &params.curveKnots};
        params.valid = rest.IsValid() &&
                       params.auxPoints.size() == params.restPoints.size() &&
                       !params.wireBindCoords.empty();
        break;
    }

    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Derived maintenance reads the final same-generation points, which
        // the caller supplies as the base value for this revision.
        params.auxPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
                                            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
                                             time, values.resolved);
        // Authored widths widen the extent bounds; omitting them silently
        // under-reports the bound of a curves/points gprim.
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths =
                _Array<float>(moverPrim, binding.widths, time, values.resolved);
        }
        if (op == RigExecRevisionOp::RecomputeNormals) {
            params.normalsPerCorner = binding.normalsPerCorner;
            params.normalsLeftHanded = binding.normalsLeftHanded;
            params.normalsTopology = binding.normalsTopology;
        }
        // Vertex normals need the adjacency; without it the kernel reports
        // MoverFailed rather than emitting garbage normals. Extent needs only
        // the points (matches _BuildRecomputeNormals/ExtentParameters).
        params.valid = !params.auxPoints.empty() &&
                       (op == RigExecRevisionOp::RecomputeExtent ||
                        !params.topologyCounts.empty());
        break;

    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break;
    }

    return params;
}

void
RigExecAssembleExternalPayload(const UsdPrim &moverPrim,
                               const RigExecRevisionBinding &binding,
                               const RigExecProviderValues &values,
                               UsdTimeCode time,
                               RigExecExternalPayload *payload)
{
    payload->schema = moverPrim.GetTypeName();
    payload->handler = binding.handler;
    std::vector<VtValue> inputs;
    inputs.reserve(binding.externalInputs.size());
    for (const auto &key : binding.externalInputs) {
        inputs.push_back(RigExecSampleRevisionLeaf(
            key, moverPrim.GetStage()->GetAttributeAtPath(key.path),
            values.resolved, time, values.upstream));
    }
    const RigExecExternalProviderValues pureValues(values);
    const RigExecExternalInputContext ctx{binding, pureValues, inputs};
    payload->valid = payload->handler && payload->handler->assembleExternal &&
                     payload->handler->assembleExternal(ctx, &payload->data);
}

namespace {

// The iterative deformers' scalar inputs, in the stage assembler's order,
// each with its fallback and the packet field it fills.
template <class Fn>
void
_IterativeScalars(RigExecRevisionOp op, RigExecMoverParameters &params, Fn &&fn)
{
    if (op == RigExecRevisionOp::DeltaMush) {
        fn(_attrTokens->iterations, 10, params.mushIterations);
        fn(_attrTokens->step, 0.5f, params.mushStep);
        fn(_attrTokens->pinBorders, true, params.mushPinBorders);
        fn(_attrTokens->distanceWeight, 0.0f, params.mushDistanceWeight);
        fn(_attrTokens->displacement, 1.0f, params.mushDisplacement);
    } else if (op == RigExecRevisionOp::Wrinkle) {
        RigExecWrinkleSettings &s = params.wrinkleSettings;
        fn(_attrTokens->iterations, 80, s.iterations);
        fn(_attrTokens->neighborDistance, 2, s.neighborDistance);
        fn(_attrTokens->restLengthScale, 1.0f, s.restLengthScale);
        fn(_attrTokens->stretchStiffness, 1.0f, s.stretchStiffness);
        fn(_attrTokens->compressionStiffness, 1.0f, s.compressionStiffness);
        fn(_attrTokens->bendStiffness, 0.1f, s.bendStiffness);
        fn(_attrTokens->maxDisplacement, 0.2f, s.maxDisplacement);
        fn(_attrTokens->pinBorders, true, s.pinBorders);
        fn(_attrTokens->tangentPlaneCollisions, true, s.tangentPlaneCollisions);
        fn(_attrTokens->tangentPlaneInset, 0.0f, s.tangentPlaneInset);
        fn(_attrTokens->wrinkleScale, 1.0f, s.wrinkleScale);
        fn(_attrTokens->smoothingIterations, 0, s.smoothingIterations);
    }
}

template <class T>
constexpr RigExecRevisionLeafType
_ScalarLeafType()
{
    if constexpr (std::is_same_v<T, bool>) {
        return RigExecRevisionLeafType::Bool;
    } else if constexpr (std::is_same_v<T, int>) {
        return RigExecRevisionLeafType::Int;
    } else {
        static_assert(std::is_same_v<T, float>, "an iterative scalar type");
        return RigExecRevisionLeafType::Float;
    }
}

} // namespace

bool
RigExecRevisionOpAssemblesFromLeaves(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::BlendShape:
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::Smooth:
    case RigExecRevisionOp::SurfaceProject:
    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints:
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle:
    case RigExecRevisionOp::Lattice:
    case RigExecRevisionOp::Wire:
    case RigExecRevisionOp::External:
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
        return true;
    }
    return false;
}

bool
RigExecRevisionLeafRoleIsTopology(RigExecRevisionLeafRole role)
{
    switch (role) {
    case RigExecRevisionLeafRole::ElementSize:
    case RigExecRevisionLeafRole::JointIndices:
    case RigExecRevisionLeafRole::TopologyCounts:
    case RigExecRevisionLeafRole::TopologyIndices:
    case RigExecRevisionLeafRole::Divisions:
    case RigExecRevisionLeafRole::CurveOrder:
    case RigExecRevisionLeafRole::CurveKnots:
    case RigExecRevisionLeafRole::MushEdges:
    case RigExecRevisionLeafRole::SnapTriangles:
        return true;
    default:
        return false;
    }
}

void
RigExecDeclareRevisionLeaves(RigExecRevisionOp op, const SdfPath &moverPath,
                             const RigExecRevisionBinding &binding,
                             RigExecRevisionLeafDecl *decl)
{
    using Role = RigExecRevisionLeafRole;
    using Type = RigExecRevisionLeafType;
    using Time = RigExecRevisionLeafTime;
    using Flavour = RigExecRevisionLeafFlavour;
    decl->assembles = RigExecRevisionOpAssemblesFromLeaves(op);
    if (!decl->assembles) {
        return;
    }
    const auto add = [decl](Role role, const SdfPath &path, Type type,
                            Time time, Flavour flavour, VtValue fallback) {
        if (path.IsEmpty()) {
            return; // _Array's empty path: the read answers an empty array
        }
        decl->roles[size_t(role)] =
            decl->Add({path, type, time, flavour, std::move(fallback)});
    };
    const auto mover = [&moverPath](const TfToken &name) {
        return moverPath.AppendProperty(name);
    };
    // Every arm's reads in its own order. A read only some branch takes is
    // declared regardless: the sample is a superset, the assembly reads the
    // leaf exactly where the stage assembler reads the stage.
    const auto envelope = [&]() {
        add(Role::Enabled, mover(_attrTokens->enabled), Type::Bool,
            Time::AtTime, Flavour::Resolved, VtValue(true));
        add(Role::DefaultWeight, mover(_attrTokens->defaultWeight), Type::Float,
            Time::AtTime, Flavour::Resolved, VtValue(1.0f));
    };
    const auto array = [&](Role role, const SdfPath &path, Type type,
                           VtValue empty) {
        add(role, path, type, Time::AtTime, Flavour::OverlayThenRaw,
            std::move(empty));
    };
    const auto topology = [&]() {
        array(Role::TopologyCounts, binding.topologyCounts, Type::IntArray,
              VtValue(VtIntArray()));
        array(Role::TopologyIndices, binding.topologyIndices, Type::IntArray,
              VtValue(VtIntArray()));
    };
    switch (op) {
    case RigExecRevisionOp::Matrix:
        // rigExec:pointFrame is read and discarded, as the stage assembler
        // reads it: the runtime's fold replays the read.
        add(Role::WeightBlend, mover(_attrTokens->weightBlend), Type::Token,
            Time::AtTime, Flavour::Raw, VtValue(TfToken()));
        add(Role::PointFrame, mover(_attrTokens->pointFrame), Type::Token,
            Time::AtTime, Flavour::Raw, VtValue(TfToken()));
        envelope();
        break;
    case RigExecRevisionOp::Skin:
        envelope();
        add(Role::SkinningMethod, mover(_attrTokens->skinningMethod),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->classicLinear));
        array(Role::JointIndices, mover(_attrTokens->jointIndices),
              Type::IntArray, VtValue(VtIntArray()));
        array(Role::JointWeights, mover(_attrTokens->jointWeights),
              Type::FloatArray, VtValue(VtFloatArray()));
        add(Role::ElementSize, mover(_attrTokens->elementSize), Type::Int,
            Time::AtTime, Flavour::Resolved, VtValue(1));
        break;
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Synthesized maintenance: no enable and no envelope read.
        topology();
        if (op == RigExecRevisionOp::RecomputeExtent) {
            array(Role::Widths, binding.widths, Type::FloatArray,
                  VtValue(VtFloatArray()));
        }
        break;
    case RigExecRevisionOp::BlendShape:
        envelope();
        add(Role::DeltaSpace, mover(_attrTokens->deltaSpace), Type::Token,
            Time::AtDefault, Flavour::Raw, VtValue(_valueTokens->target));
        topology();
        break;
    case RigExecRevisionOp::Smooth:
        envelope();
        topology();
        break;
    case RigExecRevisionOp::SurfaceProject:
        envelope();
        array(Role::SurfacePoints, binding.surfacePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        topology();
        // The snap settings, each read through the resolved inputs.
        add(Role::SnapMode, mover(_attrTokens->snapMode), Type::Token,
            Time::AtTime, Flavour::Resolved, VtValue(_valueTokens->onSurface));
        add(Role::SnapOffset, mover(_attrTokens->offset), Type::Float,
            Time::AtTime, Flavour::Resolved, VtValue(0.0f));
        add(Role::Mask, mover(_attrTokens->mask), Type::FloatArray,
            Time::AtTime, Flavour::Resolved, VtValue(VtFloatArray()));
        add(Role::SnapTriangles, mover(_attrTokens->triangles), Type::IntArray,
            Time::AtTime, Flavour::Resolved, VtValue(VtIntArray()));
        add(Role::SourceMatrix, mover(_attrTokens->surfaceMatrix),
            Type::Matrix4d, Time::AtTime, Flavour::Resolved,
            VtValue(GfMatrix4d(1.0)));
        add(Role::TargetMatrix, mover(_attrTokens->targetMatrix),
            Type::Matrix4d, Time::AtTime, Flavour::Resolved,
            VtValue(GfMatrix4d(1.0)));
        add(Role::PointSpace, mover(_attrTokens->pointSpace), Type::Token,
            Time::AtTime, Flavour::Resolved, VtValue(_valueTokens->local));
        break;
    case RigExecRevisionOp::Ribbon:
        envelope();
        array(Role::BindCoords, binding.bindCoords, Type::Vec2fArray,
              VtValue(VtArray<GfVec2f>()));
        break;
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::EmitGuidePoints:
        envelope();
        break;
    case RigExecRevisionOp::External:
        envelope();
        decl->externalBegin = int(decl->keys.size());
        for (const auto &key : binding.externalInputs)
            decl->Add(key);
        break;
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle: {
        envelope();
        add(Role::RestPoints, mover(_attrTokens->restPoints), Type::Vec3fArray,
            Time::AtDefault, Flavour::OverlayThenRaw, VtValue(VtVec3fArray()));
        topology();
        if (op == RigExecRevisionOp::Wrinkle) {
            add(Role::WrinkleTopology, mover(_attrTokens->topology),
                Type::Token, Time::AtDefault, Flavour::Resolved,
                VtValue(_valueTokens->cloth));
            add(Role::PinPoints, mover(_attrTokens->pinPoints), Type::IntArray,
                Time::AtDefault, Flavour::OverlayThenRaw,
                VtValue(VtIntArray()));
        } else {
            // deltaMush's extended settings, ahead of its scalars.
            add(Role::MushSmoothing, mover(_attrTokens->smoothing), Type::Token,
                Time::AtDefault, Flavour::Resolved,
                VtValue(_valueTokens->rest));
            add(Role::MushFrameTransport, mover(_attrTokens->frameTransport),
                Type::Token, Time::AtDefault, Flavour::Resolved,
                VtValue(_valueTokens->vertex));
            array(Role::SmoothWeights, mover(_attrTokens->smoothWeights),
                  Type::FloatArray, VtValue(VtFloatArray()));
            add(Role::MushEdges, mover(_attrTokens->edges), Type::IntArray,
                Time::AtDefault, Flavour::OverlayThenRaw,
                VtValue(VtIntArray()));
            add(Role::OnlySmooth, mover(_attrTokens->onlySmooth), Type::Bool,
                Time::AtTime, Flavour::Resolved, VtValue(false));
            add(Role::ComputationToTarget,
                mover(_attrTokens->computationToTarget), Type::Matrix4d,
                Time::AtTime, Flavour::Resolved, VtValue(GfMatrix4d(1.0)));
        }
        decl->scalarBegin = int(decl->keys.size());
        RigExecMoverParameters fields;
        _IterativeScalars(
            op, fields, [&](const TfToken &name, auto fallback, auto &) {
                using T = decltype(fallback);
                decl->Add({mover(name), _ScalarLeafType<T>(), Time::AtTime,
                           Flavour::Resolved, VtValue(fallback)});
            });
        break;
    }
    case RigExecRevisionOp::Lattice:
        envelope();
        // The bind-time cage reads past the resolved inputs (and the phase
        // overlay); the live cage through them.
        add(Role::RestCage, binding.cagePoints, Type::Vec3fArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtVec3fArray()));
        array(Role::LiveCage, binding.cagePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        add(Role::Divisions, mover(_attrTokens->divisions), Type::Vec3i,
            Time::AtTime, Flavour::Raw,
            VtValue(RigExecMoverParameters().divisions));
        // The evaluation and the regular grid's settings, each read through
        // the resolved inputs. A Bernstein lattice (evaluation "legacy") reads
        // only the evaluation.
        add(Role::LatticeEvaluation, mover(_attrTokens->evaluation),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->legacy));
        add(Role::InterpolationU, mover(_attrTokens->interpolationU),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->bspline));
        add(Role::InterpolationV, mover(_attrTokens->interpolationV),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->bspline));
        add(Role::InterpolationW, mover(_attrTokens->interpolationW),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->bspline));
        add(Role::GridOrigin, mover(_attrTokens->origin), Type::Vec3f,
            Time::AtTime, Flavour::Resolved, VtValue(GfVec3f(-0.5f)));
        add(Role::GridSpacing, mover(_attrTokens->spacing), Type::Vec3f,
            Time::AtTime, Flavour::Resolved, VtValue(GfVec3f(1.0f)));
        add(Role::GridStrength, mover(_attrTokens->strength), Type::Float,
            Time::AtTime, Flavour::Resolved, VtValue(1.0f));
        add(Role::Mask, mover(_attrTokens->mask), Type::FloatArray,
            Time::AtTime, Flavour::Resolved, VtValue(VtFloatArray()));
        add(Role::SourceMatrix, mover(_attrTokens->cageMatrix), Type::Matrix4d,
            Time::AtTime, Flavour::Resolved, VtValue(GfMatrix4d(1.0)));
        add(Role::TargetMatrix, mover(_attrTokens->targetMatrix),
            Type::Matrix4d, Time::AtTime, Flavour::Resolved,
            VtValue(GfMatrix4d(1.0)));
        add(Role::PointSpace, mover(_attrTokens->pointSpace), Type::Token,
            Time::AtTime, Flavour::Resolved, VtValue(_valueTokens->local));
        break;
    case RigExecRevisionOp::Wire:
        envelope();
        add(Role::CurveRest, binding.driverCurvePoints, Type::Vec3fArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtVec3fArray()));
        add(Role::DriverWeights, mover(_attrTokens->driverWeights),
            Type::FloatArray, Time::AtTime, Flavour::Resolved,
            VtValue(VtFloatArray()));
        add(Role::DriverBaseWeights, mover(_attrTokens->driverBaseWeights),
            Type::FloatArray, Time::AtTime, Flavour::Resolved,
            VtValue(VtFloatArray()));
        add(Role::PointFrame, mover(_attrTokens->pointFrame), Type::Token,
            Time::AtTime, Flavour::Raw, VtValue(_valueTokens->rest));
        add(Role::DeltaFrame, mover(_attrTokens->driverDeltaFrame), Type::Token,
            Time::AtTime, Flavour::Raw, VtValue(_valueTokens->local));
        array(Role::CurveLive, binding.driverCurvePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        add(Role::CurveOrder, binding.driverCurveOrder, Type::IntArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtIntArray()));
        add(Role::CurveKnots, binding.driverCurveKnots, Type::DoubleArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtDoubleArray()));
        // Read only where the attribute stands: an empty fallback.
        add(Role::Dropoff, mover(_attrTokens->dropoffDistance), Type::Float,
            Time::AtTime, Flavour::Raw, VtValue());
        array(Role::BindCoords, binding.bindCoords, Type::Vec2fArray,
              VtValue(VtArray<GfVec2f>()));
        break;
    case RigExecRevisionOp::SurfaceProjector:
        // RigExecReadProjectorTarget's reads, through the generation's
        // resolved inputs alone (a projector declares no phase).
        add(Role::RayOrigin, mover(_attrTokens->rayOrigin), Type::Vec3d,
            Time::AtTime, Flavour::Resolved, VtValue(GfVec3d(0.0, 0.0, 0.0)));
        add(Role::RayDirection, mover(_attrTokens->rayDirection), Type::Vec3d,
            Time::AtTime, Flavour::Resolved, VtValue(GfVec3d(0.0, 0.0, 1.0)));
        add(Role::RayUp, mover(_attrTokens->rayUp), Type::Vec3d, Time::AtTime,
            Flavour::Resolved, VtValue(GfVec3d(0.0, 1.0, 0.0)));
        add(Role::ShaderOffset, mover(_attrTokens->shaderOffset),
            Type::Matrix4d, Time::AtTime, Flavour::Resolved,
            VtValue(GfMatrix4d(1.0)));
        add(Role::ProjectionMode, mover(_attrTokens->projectionMode),
            Type::Token, Time::AtTime, Flavour::Raw,
            VtValue(_valueTokens->material));
        topology();
        break;
    case RigExecRevisionOp::ShaderDials:
        // One dial per source, in order (compile admits property paths
        // only; a missing attribute answers 0).
        decl->dialBegin = int(decl->keys.size());
        for (const SdfPath &dial : binding.shaderDials) {
            decl->Add({dial, Type::Dial, Time::AtTime, Flavour::Resolved,
                       VtValue(0.0)});
        }
        break;
    }
}

void
RigExecDeclareSkinLayoutLeaves(const SdfPath &moverPath,
                               RigExecRevisionLeafDecl *decl)
{
    using Role = RigExecRevisionLeafRole;
    using Type = RigExecRevisionLeafType;
    using Time = RigExecRevisionLeafTime;
    using Flavour = RigExecRevisionLeafFlavour;
    decl->roles[size_t(Role::JointIndices)] = decl->Add(
        {moverPath.AppendProperty(_attrTokens->jointIndices), Type::IntArray,
         Time::AtTime, Flavour::OverlayThenRaw, VtValue(VtIntArray())});
    decl->roles[size_t(Role::JointWeights)] = decl->Add(
        {moverPath.AppendProperty(_attrTokens->jointWeights), Type::FloatArray,
         Time::AtTime, Flavour::OverlayThenRaw, VtValue(VtFloatArray())});
    decl->roles[size_t(Role::ElementSize)] =
        decl->Add({moverPath.AppendProperty(_attrTokens->elementSize),
                   Type::Int, Time::AtTime, Flavour::Resolved, VtValue(1)});
}

namespace {

template <class T> struct _IsVtArray : std::false_type {};
template <class E> struct _IsVtArray<VtArray<E>> : std::true_type {};

// An upstream array at \p path in \p layer: authored-level, so it answers a
// raw read at any time, Default included, as a stage holding it would.
template <class T>
bool
_UpstreamArray(const std::map<SdfPath, VtValue> *layer, const SdfPath &path,
               T *value)
{
    if constexpr (_IsVtArray<T>::value) {
        if (layer && !layer->empty()) {
            const auto it = layer->find(path);
            if (it != layer->end() && it->second.IsHolding<T>()) {
                *value = it->second.UncheckedGet<T>();
                return true;
            }
        }
    }
    return false;
}

// One typed read, site for site: \p flavour over \p fallback. An array read
// of any flavour answers from the upstream \p layer before the stage.
template <class T>
T
_LeafRead(RigExecRevisionLeafFlavour flavour, const SdfPath &path,
          const UsdAttribute &a, const RigExecResolvedInputs *resolved,
          const std::map<SdfPath, VtValue> *layer, UsdTimeCode time, T value)
{
    switch (flavour) {
    case RigExecRevisionLeafFlavour::Raw:
        if (_UpstreamArray(layer, path, &value)) {
            return value;
        }
        if (a) {
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::Resolved:
        if (a) {
            if (resolved &&
                resolved->GetAttributeOverStageLayer(a, time, layer, &value)) {
                return value;
            }
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::ResolvedOnly:
        if (resolved) {
            resolved->GetAttributeOverStageLayer(a, time, layer, &value);
        }
        return value;
    case RigExecRevisionLeafFlavour::OverlayThenRaw:
        if (resolved && resolved->Get(path, &value)) {
            return value;
        }
        if (_UpstreamArray(layer, path, &value)) {
            return value;
        }
        if (a) {
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::Present:
        break;
    }
    return value;
}

template <class T>
VtValue
_LeafSample(const RigExecRevisionLeafKey &key, const UsdAttribute &a,
            const RigExecResolvedInputs *resolved,
            const std::map<SdfPath, VtValue> *layer, UsdTimeCode time)
{
    if (key.flavour == RigExecRevisionLeafFlavour::Raw &&
        key.fallback.IsEmpty()) {
        T value;
        if (_UpstreamArray(layer, key.path, &value) ||
            (a && a.Get(&value, time)))
            return VtValue(value); // A successful empty array is still present.
        return VtValue(); // Failed Get must not synthesize T{} presence.
    }
    const T fallback =
        key.fallback.IsHolding<T>() ? key.fallback.UncheckedGet<T>() : T();
    return VtValue(_LeafRead<T>(key.flavour, key.path, a, resolved, layer, time,
                                fallback));
}

} // namespace

VtValue
RigExecSampleRevisionLeaf(const RigExecRevisionLeafKey &key,
                          const UsdAttribute &attribute,
                          const RigExecResolvedInputs *resolved,
                          UsdTimeCode time,
                          const std::map<SdfPath, VtValue> *upstream)
{
    if (key.flavour == RigExecRevisionLeafFlavour::Present) {
        return VtValue(resolved && resolved->Find(key.path) != nullptr);
    }
    const UsdTimeCode at = key.time == RigExecRevisionLeafTime::AtDefault
                               ? UsdTimeCode::Default()
                               : time;
    // A site that reads only an attribute that exists: nothing stands.
    if (key.flavour == RigExecRevisionLeafFlavour::Raw &&
        key.fallback.IsEmpty() && !attribute) {
        return VtValue();
    }
    switch (key.type) {
    case RigExecRevisionLeafType::Double:
        return _LeafSample<double>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Vec3f:
        return _LeafSample<GfVec3f>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Bool:
        return _LeafSample<bool>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Int:
        return _LeafSample<int>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Float:
        return _LeafSample<float>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Token:
        return _LeafSample<TfToken>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::IntArray:
        return _LeafSample<VtIntArray>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::FloatArray:
        return _LeafSample<VtFloatArray>(key, attribute, resolved, upstream,
                                         at);
    case RigExecRevisionLeafType::Vec2fArray:
        return _LeafSample<VtArray<GfVec2f>>(key, attribute, resolved, upstream,
                                             at);
    case RigExecRevisionLeafType::Vec3fArray:
        return _LeafSample<VtVec3fArray>(key, attribute, resolved, upstream,
                                         at);
    case RigExecRevisionLeafType::Vec3i:
        return _LeafSample<GfVec3i>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Vec3d:
        return _LeafSample<GfVec3d>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Matrix4d:
        return _LeafSample<GfMatrix4d>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::DoubleArray:
        return _LeafSample<VtDoubleArray>(key, attribute, resolved, upstream,
                                          at);
    case RigExecRevisionLeafType::Dial: {
        // RigExecReadProjectorTarget's dial read, through \p resolved.
        double value = 0.0;
        const UsdAttribute &a = attribute;
        if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
            float asFloat = 0.0f;
            if (!(resolved && resolved->GetAttributeOverStageLayer(
                                  a, at, upstream, &asFloat))) {
                a.Get(&asFloat, at);
            }
            value = double(asFloat);
        } else if (a) {
            if (!(resolved && resolved->GetAttributeOverStageLayer(
                                  a, at, upstream, &value))) {
                a.Get(&value, at);
            }
        }
        return VtValue(value);
    }
    }
    return key.fallback;
}

void
RigExecRevisionLeafHops(const RigExecRevisionLeafKey &key,
                        const UsdAttribute &attribute,
                        std::vector<SdfPath> *hops, bool *varying)
{
    hops->clear();
    *varying = false;
    hops->push_back(key.path);
    const bool atTime = key.time == RigExecRevisionLeafTime::AtTime;
    if (key.flavour == RigExecRevisionLeafFlavour::Present || !attribute) {
        return;
    }
    *varying = atTime && attribute.ValueMightBeTimeVarying();
    if (key.flavour != RigExecRevisionLeafFlavour::Resolved &&
        key.flavour != RigExecRevisionLeafFlavour::ResolvedOnly) {
        return;
    }
    // GetAttribute's walk: single authored connections, followed until a hop
    // has none or more than one, or the walk meets itself. The float read of
    // a double hop recurses from that hop over the same walk.
    std::set<SdfPath> visiting{attribute.GetPath()};
    UsdAttribute a = attribute;
    while (a) {
        SdfPathVector connections;
        if (a.HasAuthoredConnections()) {
            a.GetConnections(&connections);
        }
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
        if (!a) {
            // A target with no attribute yet ends the walk, and is still
            // where the read can reach: authoring it changes the answer.
            hops->push_back(connections[0]);
            break;
        }
        if (!visiting.insert(a.GetPath()).second) {
            break;
        }
        hops->push_back(a.GetPath());
        *varying = *varying || (atTime && a.ValueMightBeTimeVarying());
    }
}

namespace {

// The leaves of one covered operation, read where the stage assembler reads
// the stage.
struct _Leaves {
    const RigExecRevisionLeafView &view;

    const VtValue *At(RigExecRevisionLeafRole role) const
    {
        const int k = view.decl ? view.decl->Role(role) : -1;
        if (k < 0 || !view.values || size_t(k) >= view.values->size()) {
            return nullptr;
        }
        return &(*view.values)[size_t(k)];
    }

    // A scalar the operation always reads: an undeclared one is reported
    // (when asked) and answers the site's fallback.
    template <class T>
    T Scalar(RigExecRevisionLeafRole role, const T &fallback,
             const char *name) const
    {
        const VtValue *v = At(role);
        if (!v) {
            if (view.missing) {
                view.missing->push_back(name);
            }
            return fallback;
        }
        return v->IsHolding<T>() ? v->UncheckedGet<T>() : fallback;
    }

    // _Array: the leaf's array copied out, empty when undeclared. A points
    // read through the resolved inputs takes the phase overlay first; a raw
    // one (the lattice's bind-time cage, the wire's rest curve) never does.
    template <class T> std::vector<T> Array(RigExecRevisionLeafRole role) const
    {
        std::vector<T> out;
        const VtValue *v = At(role);
        if (!v) {
            return out;
        }
        if constexpr (std::is_same_v<T, GfVec3f>) {
            const RigExecRevisionLeafKey &key =
                view.decl->keys[size_t(view.decl->Role(role))];
            VtVec3fArray phased;
            if (view.phased &&
                key.flavour == RigExecRevisionLeafFlavour::OverlayThenRaw &&
                view.phased->Get(key.path, &phased)) {
                out.assign(phased.begin(), phased.end());
                return out;
            }
        }
        if (v->IsHolding<VtArray<T>>()) {
            const VtArray<T> &held = v->UncheckedGet<VtArray<T>>();
            out.assign(held.begin(), held.end());
        }
        return out;
    }

    // The leaf's VtArray itself, shared rather than copied; empty when
    // undeclared or holding another type.
    template <class T> VtArray<T> Shared(RigExecRevisionLeafRole role) const
    {
        const VtValue *v = At(role);
        return v && v->IsHolding<VtArray<T>>() ? v->UncheckedGet<VtArray<T>>()
                                               : VtArray<T>();
    }

    // A read only an existing attribute answers: false when the leaf is
    // empty (or undeclared, which is reported when asked).
    template <class T>
    bool IfPresent(RigExecRevisionLeafRole role, const char *name, T *out) const
    {
        const VtValue *v = At(role);
        if (!v) {
            if (view.missing) {
                view.missing->push_back(name);
            }
            return false;
        }
        if (!v->IsHolding<T>()) {
            return false;
        }
        *out = v->UncheckedGet<T>();
        return true;
    }

    // Key \p k of the declaration as \p T, or \p fallback.
    template <class T> T Key(int k, const T &fallback) const
    {
        if (k < 0 || !view.values || size_t(k) >= view.values->size()) {
            return fallback;
        }
        const VtValue &v = (*view.values)[size_t(k)];
        return v.IsHolding<T>() ? v.UncheckedGet<T>() : fallback;
    }

    bool Enabled() const
    {
        return Scalar<bool>(RigExecRevisionLeafRole::Enabled, true,
                            SchemaAttrText::inputsEnabled);
    }
    float DefaultWeight() const
    {
        return Scalar<float>(RigExecRevisionLeafRole::DefaultWeight, 1.0f,
                             SchemaAttrText::inputsDefaultWeight);
    }
};

RigExecMoverParameters
_MatrixFromLeaves(const _Leaves &L, const RigExecProviderValues &values)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->matrix;
    params.radialWeight =
        L.Scalar<TfToken>(RigExecRevisionLeafRole::WeightBlend, TfToken(),
                          SchemaAttrText::rigExecWeightBlend) == "radial";
    params.enabled = L.Enabled();
    if (!params.enabled) {
        params.valid = true; // disabled is an ordinary pass-through
        return params;
    }
    const GfMatrix4d *transform = values.transform;
    if (!transform) {
        return params; // MoverFailed
    }
    params.weights = values.weights
                         ? *values.weights
                         : RigExecWeightPacket::Constant(L.DefaultWeight());
    if (!params.weights.valid) {
        return params; // invalid common envelope => MoverFailed
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite((*transform)[i][j])) {
                return params;
            }
        }
    }
    if ((*transform)[0][3] != 0 || (*transform)[1][3] != 0 ||
        (*transform)[2][3] != 0 || (*transform)[3][3] != 1) {
        return params;
    }
    params.transform = *transform;
    params.valid = true;
    return params;
}

RigExecMoverParameters
_SkinFromLeaves(const _Leaves &L, const RigExecProviderValues &values)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->skin;
    params.enabled = L.Enabled();
    if (!params.enabled) {
        params.valid = true; // disabled is an ordinary pass-through
        return params;
    }
    if (!values.influenceTransforms) {
        return params; // MoverFailed
    }
    params.weights = values.weights
                         ? *values.weights
                         : RigExecWeightPacket::Constant(L.DefaultWeight());
    if (!params.weights.valid) {
        return params; // invalid common envelope => MoverFailed
    }
    params.skinTransforms = *values.influenceTransforms;
    params.skinningMethod = L.Scalar<TfToken>(
        RigExecRevisionLeafRole::SkinningMethod, _valueTokens->classicLinear,
        SchemaAttrText::rigExecSkinningMethod);
    // The layout the caller resolved; the cache is the stage assembler's.
    if (values.skinTopology) {
        params.skinTopology = *values.skinTopology;
    }
    if (params.skinTopology) {
        params.skinElementSize = params.skinTopology->elementSize;
        if (!params.skinTopology->validated ||
            params.skinTransforms.size() !=
                params.skinTopology->influenceCount) {
            return params;
        }
        for (const GfMatrix4d &m : params.skinTransforms) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (!std::isfinite(m[r][c])) {
                        return params;
                    }
                }
            }
            if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
                return params;
            }
        }
        if (params.skinningMethod != "classicLinear" &&
            params.skinningMethod != "dualQuaternion") {
            return params;
        }
        params.valid = true;
        return params;
    }
    // No resolved layout: the per-frame arrays.
    params.skinIndices = L.Array<int>(RigExecRevisionLeafRole::JointIndices);
    params.skinWeights = L.Array<float>(RigExecRevisionLeafRole::JointWeights);
    params.skinElementSize =
        L.Scalar<int>(RigExecRevisionLeafRole::ElementSize, 1,
                      SchemaAttrText::rigExecElementSize);
    if (params.skinElementSize < 1 ||
        params.skinWeights.size() != params.skinIndices.size() ||
        params.skinIndices.size() % size_t(params.skinElementSize) != 0) {
        return params;
    }
    RigExecSkinLayout layout;
    layout.transforms = params.skinTransforms.data();
    layout.transformCount = params.skinTransforms.size();
    layout.indices = params.skinIndices.data();
    layout.weights = params.skinWeights.data();
    layout.indexCount = params.skinIndices.size();
    layout.elementSize = size_t(params.skinElementSize);
    layout.pointCount = layout.indexCount / layout.elementSize;
    if (!layout.Validate()) {
        return params;
    }
    if (params.skinningMethod != "classicLinear" &&
        params.skinningMethod != "dualQuaternion") {
        return params;
    }
    params.valid = true;
    return params;
}

// The kind, enable and envelope every leaf-assembled revision but a
// matrix's and a skin's starts from. False when that completes the packet:
// a disabled revision passes through, an invalid envelope fails.
bool
_AssembleCommon(RigExecRevisionOp op, const _Leaves &L,
                const RigExecProviderValues &values,
                RigExecMoverParameters *params)
{
    const bool synthesizedDerived = op == RigExecRevisionOp::RecomputeNormals ||
                                    op == RigExecRevisionOp::RecomputeExtent;
    params->enabled = synthesizedDerived ? true : L.Enabled();
    switch (op) {
    case RigExecRevisionOp::BlendShape:
        params->kind = _kindTokens->blendShape;
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params->kind = _kindTokens->volumeCorrect;
        break;
    case RigExecRevisionOp::Smooth:
        params->kind = _kindTokens->smooth;
        break;
    case RigExecRevisionOp::SurfaceProject:
        params->kind = _kindTokens->surfaceProject;
        break;
    case RigExecRevisionOp::Ribbon:
        params->kind = _kindTokens->ribbon;
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params->kind = _kindTokens->emitGuidePoints;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params->kind = _kindTokens->recomputeNormals;
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params->kind = _kindTokens->recomputeExtent;
        break;
    case RigExecRevisionOp::DeltaMush:
        params->kind = _kindTokens->deltaMush;
        break;
    case RigExecRevisionOp::Wrinkle:
        params->kind = _kindTokens->wrinkle;
        break;
    case RigExecRevisionOp::Lattice:
        params->kind = _kindTokens->lattice;
        break;
    case RigExecRevisionOp::Wire:
        params->kind = _kindTokens->wire;
        break;
    case RigExecRevisionOp::External:
        params->kind = _kindTokens->external;
        break;
    case RigExecRevisionOp::SurfaceProjector:
        params->kind = _kindTokens->surfaceProjector;
        break;
    case RigExecRevisionOp::ShaderDials:
        params->kind = _kindTokens->shaderDials;
        break;
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break; // handled above
    }
    if (!params->enabled) {
        params->valid = true; // disabled is an ordinary pass-through
        return false;
    }
    params->weights =
        synthesizedDerived
            ? RigExecWeightPacket::Constant(1.0f)
            : (values.weights
                   ? *values.weights
                   : RigExecWeightPacket::Constant(L.DefaultWeight()));
    // An invalid envelope is MoverFailed, preserving the preceding revision.
    return params->weights.valid;
}

// An External revision's declared inputs, in declaration order: a points
// read at the time through a phase-honouring flavour takes the phase
// overlay's array at its path, every other its sampled leaf. False when a
// leaf is missing.
bool
_ExternalInputs(const RigExecRevisionBinding &binding,
                const RigExecRevisionLeafView &leaves,
                std::vector<VtValue> *inputs)
{
    inputs->clear();
    inputs->reserve(binding.externalInputs.size());
    for (size_t i = 0; i < binding.externalInputs.size(); ++i) {
        const auto &key = binding.externalInputs[i];
        const size_t slot = size_t(leaves.decl->externalBegin) + i;
        if (slot >= leaves.values->size())
            break;
        const VtValue *phased =
            leaves.phased ? leaves.phased->Find(key.path) : nullptr;
        const bool phaseRead =
            key.time == RigExecRevisionLeafTime::AtTime &&
            (key.flavour == RigExecRevisionLeafFlavour::Resolved ||
             key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly ||
             key.flavour == RigExecRevisionLeafFlavour::OverlayThenRaw);
        inputs->push_back(phased && phaseRead &&
                                  key.type ==
                                      RigExecRevisionLeafType::Vec3fArray
                              ? *phased
                              : (*leaves.values)[slot]);
    }
    return inputs->size() == binding.externalInputs.size();
}

} // namespace

RigExecMoverParameters
RigExecAssembleFromLeaves(RigExecRevisionOp op,
                          const RigExecRevisionBinding &binding,
                          const RigExecRevisionLeafView &leaves,
                          const RigExecProviderValues &values,
                          RigExecExternalRevision *external)
{
    using Role = RigExecRevisionLeafRole;
    const _Leaves L{leaves};
    if (op == RigExecRevisionOp::Matrix) {
        return _MatrixFromLeaves(L, values);
    }
    if (op == RigExecRevisionOp::Skin) {
        return _SkinFromLeaves(L, values);
    }
    RigExecMoverParameters params;
    if (!_AssembleCommon(op, L, values, &params)) {
        return params;
    }
    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        if (values.lendBlendDeltas) {
            params.blendDeltas.swap(values.blendDeltas);
        } else {
            params.blendDeltas = values.blendDeltas;
        }
        const TfToken space =
            L.Scalar<TfToken>(Role::DeltaSpace, _valueTokens->target,
                              SchemaAttrText::rigExecDeltaSpace);
        if (space != "target" && space != "surfaceFrame")
            break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            values.CopyBasePoints(&params.restPoints);
            params.topologyCounts = L.Array<int>(Role::TopologyCounts);
            params.topologyIndices = L.Array<int>(Role::TopologyIndices);
            if (params.topologyCounts.empty())
                break;
        }
        params.valid = values.blendFromChannels || !params.blendDeltas.empty();
        break;
    }
    case RigExecRevisionOp::VolumeCorrect:
        params.strength = 1.0f;
        if (values.BasePointCount() != 0) {
            params.referenceVolume = RigExecBoundVolume(
                values.BasePointData(), values.BasePointCount());
            params.valid = true;
        }
        break;
    case RigExecRevisionOp::Smooth:
        params.strength = 1.0f;
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        params.valid = !params.topologyCounts.empty();
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.strength = 1.0f;
        params.auxPoints = L.Array<GfVec3f>(Role::SurfacePoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        if (!_SurfaceSnapSettings(
                L.Scalar<TfToken>(Role::SnapMode, _valueTokens->onSurface,
                                  SchemaAttrText::rigExecSnapMode),
                L.Scalar<float>(Role::SnapOffset, 0.0f,
                                SchemaAttrText::rigExecOffset),
                L.Array<float>(Role::Mask), L.Array<int>(Role::SnapTriangles),
                L.Scalar<GfMatrix4d>(Role::SourceMatrix, GfMatrix4d(1.0),
                                     SchemaAttrText::rigExecSurfaceMatrix),
                L.Scalar<GfMatrix4d>(Role::TargetMatrix, GfMatrix4d(1.0),
                                     SchemaAttrText::rigExecTargetMatrix),
                L.Scalar<TfToken>(Role::PointSpace, _valueTokens->local,
                                  SchemaAttrText::rigExecPointSpace),
                binding, values.influenceTransforms, &params)) {
            break;
        }
        params.valid = _SurfaceProjectValid(params, values.BasePointCount());
        break;
    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints: {
        const RigExecPointFrameArray *frames = values.driverFrames;
        if (!frames || frames->IsEmpty() ||
            frames->rests.size() != frames->GetSize()) {
            break; // MoverFailed
        }
        params.frames = *frames;
        if (op == RigExecRevisionOp::Ribbon) {
            params.bindCoords = L.Array<GfVec2f>(Role::BindCoords);
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        values.CopyBasePoints(&params.auxPoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths = L.Array<float>(Role::Widths);
        } else {
            params.normalsPerCorner = binding.normalsPerCorner;
            params.normalsLeftHanded = binding.normalsLeftHanded;
            params.normalsTopology = binding.normalsTopology;
        }
        params.valid = !params.auxPoints.empty() &&
                       (op == RigExecRevisionOp::RecomputeExtent ||
                        !params.topologyCounts.empty());
        break;
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle: {
        params.restPoints = L.Array<GfVec3f>(Role::RestPoints);
        if (op == RigExecRevisionOp::DeltaMush &&
            !_DeltaMushSettings(
                L.Scalar<TfToken>(Role::MushSmoothing, _valueTokens->rest,
                                  SchemaAttrText::inputsSmoothing),
                L.Scalar<TfToken>(Role::MushFrameTransport,
                                  _valueTokens->vertex,
                                  SchemaAttrText::inputsFrameTransport),
                L.Array<float>(Role::SmoothWeights),
                L.Array<int>(Role::MushEdges),
                L.Scalar<bool>(Role::OnlySmooth, false,
                               SchemaAttrText::inputsOnlySmooth),
                L.Scalar<GfMatrix4d>(Role::ComputationToTarget, GfMatrix4d(1.0),
                                     SchemaAttrText::inputsComputationToTarget),
                binding, values.influenceTransforms, &params)) {
            break;
        }
        if (params.restPoints.empty())
            values.CopyBasePoints(&params.restPoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        if (op == RigExecRevisionOp::Wrinkle) {
            const TfToken topology =
                L.Scalar<TfToken>(Role::WrinkleTopology, _valueTokens->cloth,
                                  SchemaAttrText::inputsTopology);
            if (topology != _valueTokens->cloth &&
                topology != _valueTokens->surfaceStruts) {
                break;
            }
            params.wrinkleSettings.topology =
                topology == _valueTokens->cloth
                    ? RigExecWrinkleTopology::Cloth
                    : RigExecWrinkleTopology::SurfaceStruts;
            params.wrinkleSettings.pinPoints = L.Array<int>(Role::PinPoints);
        }
        // The scalars, key by key in the declaration's order; an operation
        // whose declaration holds none answers each site's fallback.
        int k = leaves.decl ? leaves.decl->scalarBegin : -1;
        _IterativeScalars(
            op, params, [&](const TfToken &name, auto fallback, auto &field) {
                using T = decltype(fallback);
                if (k < 0) {
                    if (leaves.missing) {
                        leaves.missing->push_back(name.GetString());
                    }
                    field = fallback;
                    return;
                }
                field = L.Key<T>(k++, fallback);
            });
        params.valid =
            !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RigExecRevisionOp::Lattice: {
        const TfToken evaluation =
            L.Scalar<TfToken>(Role::LatticeEvaluation, _valueTokens->legacy,
                              SchemaAttrText::rigExecEvaluation);
        if (evaluation != _valueTokens->legacy &&
            evaluation != _valueTokens->regularGrid) {
            break;
        }
        params.latticeSettings.regularGrid =
            evaluation == _valueTokens->regularGrid;
        if (params.latticeSettings.regularGrid) {
            const TfToken interpolation[3] = {
                L.Scalar<TfToken>(Role::InterpolationU, _valueTokens->bspline,
                                  SchemaAttrText::rigExecInterpolationU),
                L.Scalar<TfToken>(Role::InterpolationV, _valueTokens->bspline,
                                  SchemaAttrText::rigExecInterpolationV),
                L.Scalar<TfToken>(Role::InterpolationW, _valueTokens->bspline,
                                  SchemaAttrText::rigExecInterpolationW)};
            if (!_LatticeGridSettings(
                    interpolation,
                    L.Scalar<GfVec3f>(Role::GridOrigin, GfVec3f(-0.5f),
                                      SchemaAttrText::rigExecOrigin),
                    L.Scalar<GfVec3f>(Role::GridSpacing, GfVec3f(1.0f),
                                      SchemaAttrText::rigExecSpacing),
                    L.Scalar<float>(Role::GridStrength, 1.0f,
                                    SchemaAttrText::rigExecStrength),
                    L.Array<float>(Role::Mask),
                    L.Scalar<GfMatrix4d>(Role::SourceMatrix, GfMatrix4d(1.0),
                                         SchemaAttrText::rigExecCageMatrix),
                    L.Scalar<GfMatrix4d>(Role::TargetMatrix, GfMatrix4d(1.0),
                                         SchemaAttrText::rigExecTargetMatrix),
                    L.Scalar<TfToken>(Role::PointSpace, _valueTokens->local,
                                      SchemaAttrText::rigExecPointSpace),
                    binding, values.influenceTransforms, &params)) {
                break;
            }
        }
        // auxPoints is the BIND-TIME cage and auxPointsB the live one, as
        // the stage assembler orders them.
        if (values.retainedRest && !values.retainedRest->empty()) {
            params.restPoints.swap(*values.retainedRest);
        } else {
            values.CopyBasePoints(&params.restPoints);
        }
        params.auxPoints = L.Array<GfVec3f>(Role::RestCage);
        params.auxPointsB = L.Array<GfVec3f>(Role::LiveCage);
        params.divisions = L.Scalar<GfVec3i>(Role::Divisions, params.divisions,
                                             SchemaAttrText::rigExecDivisions);
        const size_t cageCount = size_t(params.divisions[0]) *
                                 size_t(params.divisions[1]) *
                                 size_t(params.divisions[2]);
        // A regular grid places its nodes itself: it needs no bind-time
        // cage and may be one node thick along an axis.
        const int minimum = params.latticeSettings.regularGrid ? 1 : 2;
        params.valid =
            params.divisions[0] >= minimum && params.divisions[1] >= minimum &&
            params.divisions[2] >= minimum &&
            (params.latticeSettings.regularGrid ||
             params.auxPoints.size() == cageCount) &&
            params.auxPointsB.size() == cageCount && !params.restPoints.empty();
        break;
    }
    case RigExecRevisionOp::Wire: {
        params.restPoints = L.Array<GfVec3f>(Role::CurveRest);
        if (binding.driverTransformCount > 0) {
            const size_t t = size_t(binding.driverTransformCount);
            const size_t s = size_t(binding.driverSpaceCount);
            const size_t bt = size_t(binding.driverBaseTransformCount);
            const std::vector<GfMatrix4d> *table = values.influenceTransforms;
            if (!table || table->size() < t + s + bt) {
                break; // MoverFailed
            }
            const VtFloatArray weights = L.Shared<float>(Role::DriverWeights);
            const VtFloatArray baseWeights =
                L.Shared<float>(Role::DriverBaseWeights);
            RigExecWireDriverFrame frame;
            frame.posedPoints =
                L.Scalar<TfToken>(Role::PointFrame, _valueTokens->rest,
                                  SchemaAttrText::rigExecPointFrame) == "posed";
            frame.posedDelta =
                L.Scalar<TfToken>(Role::DeltaFrame, _valueTokens->local,
                                  SchemaAttrText::rigExecDriverDeltaFrame) ==
                "posed";
            frame.carry = values.carry;
            RigExecPoseWireDrivers(*table, t, s, bt, weights, baseWeights,
                                   frame, &params.restPoints,
                                   &params.auxPoints);
        } else {
            params.auxPoints = L.Array<GfVec3f>(Role::CurveLive);
        }
        const VtIntArray order = L.Shared<int>(Role::CurveOrder);
        if (!order.empty()) {
            params.curveOrder = order[0];
        }
        params.curveKnots = L.Array<double>(Role::CurveKnots);
        float dropoff = 0.0f;
        if (L.IfPresent<float>(Role::Dropoff,
                               SchemaAttrText::inputsDropoffDistance,
                               &dropoff)) {
            params.dropoffDistance = dropoff;
        }
        params.wireBindCoords = L.Shared<GfVec2f>(Role::BindCoords);
        const RigExecNurbsCurve rest{&params.restPoints, params.curveOrder,
                                     &params.curveKnots};
        params.valid = rest.IsValid() &&
                       params.auxPoints.size() == params.restPoints.size() &&
                       !params.wireBindCoords.empty();
        break;
    }
    case RigExecRevisionOp::External: {
        if (external)
            external->handler = binding.handler;
        if (!binding.handler || !binding.handler->assembleExternal ||
            !leaves.decl || !leaves.values || leaves.decl->externalBegin < 0) {
            if (leaves.missing)
                leaves.missing->push_back("external inputs");
            break;
        }
        if (external)
            external->schema = binding.externalSchema;
        std::vector<VtValue> inputs;
        const bool gathered = _ExternalInputs(binding, leaves, &inputs);
        const RigExecExternalProviderValues pureValues(values);
        const RigExecExternalInputContext ctx{binding, pureValues, inputs};
        VtValue data;
        params.valid =
            gathered && binding.handler->assembleExternal(ctx, &data);
        if (external)
            external->data = std::move(data);
        break;
    }
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break;
    }
    return params;
}

RigExecMoverParameters
RigExecGatherExternalInputs(const RigExecRevisionBinding &binding,
                            const RigExecRevisionLeafView &leaves,
                            const RigExecProviderValues &values,
                            std::vector<VtValue> *inputs)
{
    inputs->clear();
    const _Leaves L{leaves};
    RigExecMoverParameters params;
    if (!_AssembleCommon(RigExecRevisionOp::External, L, values, &params)) {
        return params;
    }
    if (!leaves.decl || !leaves.values || leaves.decl->externalBegin < 0) {
        if (leaves.missing)
            leaves.missing->push_back("external inputs");
        return params;
    }
    params.valid = _ExternalInputs(binding, leaves, inputs);
    return params;
}

bool
RigExecExternalPayloadIsRead(const RigExecRevisionLeafView &leaves,
                             const RigExecProviderValues &values)
{
    const _Leaves L{leaves};
    if (!L.Enabled()) {
        return false;
    }
    return values.weights
               ? values.weights->valid
               : RigExecWeightPacket::Constant(L.DefaultWeight()).valid;
}

void
RigExecReadProjectorTargetFromLeaves(RigExecRevisionOp op,
                                     const RigExecRevisionBinding &binding,
                                     const RigExecRevisionLeafView &leaves,
                                     RigExecProjectorReads *reads)
{
    using Role = RigExecRevisionLeafRole;
    const _Leaves L{leaves};
    if (op == RigExecRevisionOp::ShaderDials) {
        reads->dials.clear();
        const int begin = leaves.decl ? leaves.decl->dialBegin : -1;
        for (size_t i = 0; i < binding.shaderDials.size(); ++i) {
            reads->dials.push_back(
                begin < 0 ? 0.0 : L.Key<double>(begin + int(i), 0.0));
        }
        return;
    }
    reads->rayOrigin =
        L.Scalar<GfVec3d>(Role::RayOrigin, GfVec3d(0.0, 0.0, 0.0),
                          SchemaAttrText::rigExecRayOrigin);
    reads->rayDirection =
        L.Scalar<GfVec3d>(Role::RayDirection, GfVec3d(0.0, 0.0, 1.0),
                          SchemaAttrText::rigExecRayDirection);
    reads->rayUp = L.Scalar<GfVec3d>(Role::RayUp, GfVec3d(0.0, 1.0, 0.0),
                                     SchemaAttrText::rigExecRayUp);
    reads->shaderOffset =
        L.Scalar<GfMatrix4d>(Role::ShaderOffset, GfMatrix4d(1.0),
                             SchemaAttrText::rigExecShaderOffset);
    reads->reproject =
        L.Scalar<TfToken>(Role::ProjectionMode, _valueTokens->material,
                          SchemaAttrText::rigExecProjectionMode) == "reproject";
    reads->faceVertexCounts = L.Array<int>(Role::TopologyCounts);
    reads->faceVertexIndices = L.Array<int>(Role::TopologyIndices);
}

// Source adapter and pure projector arithmetic survive executor retirement.
GfMatrix4d
RigExecWorldFromRest(const std::array<GfVec3d, 4> &restPoints,
                     const GfMatrix4d &restToPose)
{
    GfMatrix4d rest(1.0);
    RigExecPointsToMatrix(RigExecIdentityLandmarks(), restPoints, &rest);
    return rest * restToPose;
}

void
RigExecReadProjectorTarget(const UsdPrim &projectorPrim, RigExecRevisionOp op,
                           const RigExecRevisionBinding &binding,
                           const RigExecResolvedInputs *resolved,
                           UsdTimeCode time, RigExecProjectorReads *reads)
{
    if (!projectorPrim) {
        return;
    }
    if (op == RigExecRevisionOp::ShaderDials) {
        // The dials, each read through the generation's resolved inputs so
        // a property chain revising one is what the shader sees.
        const UsdStageRefPtr stage = projectorPrim.GetStage();
        reads->dials.clear();
        for (const SdfPath &dial : binding.shaderDials) {
            const UsdAttribute a = stage->GetAttributeAtPath(dial);
            double value = 0.0;
            if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                float asFloat = 0.0f;
                if (!(resolved && resolved->GetAttribute(a, time, &asFloat))) {
                    a.Get(&asFloat, time);
                }
                value = double(asFloat);
            } else if (a) {
                if (!(resolved && resolved->GetAttribute(a, time, &value))) {
                    a.Get(&value, time);
                }
            }
            reads->dials.push_back(value);
        }
        return;
    }
    reads->rayOrigin =
        _RecordedInput<GfVec3d>(projectorPrim, _attrTokens->rayOrigin,
                                GfVec3d(0.0, 0.0, 0.0), time, resolved);
    reads->rayDirection =
        _RecordedInput<GfVec3d>(projectorPrim, _attrTokens->rayDirection,
                                GfVec3d(0.0, 0.0, 1.0), time, resolved);
    reads->rayUp =
        _RecordedInput<GfVec3d>(projectorPrim, _attrTokens->rayUp,
                                GfVec3d(0.0, 1.0, 0.0), time, resolved);
    reads->shaderOffset =
        _RecordedInput<GfMatrix4d>(projectorPrim, _attrTokens->shaderOffset,
                                   GfMatrix4d(1.0), time, resolved);
    reads->reproject =
        _RecordedToken(projectorPrim, _attrTokens->projectionMode,
                       _valueTokens->material, time) == "reproject";
    reads->faceVertexCounts =
        _Array<int>(projectorPrim, binding.topologyCounts, time, resolved);
    reads->faceVertexIndices =
        _Array<int>(projectorPrim, binding.topologyIndices, time, resolved);
}

bool
RigExecRunProjectorTarget(RigExecRevisionOp op,
                          const RigExecRevisionBinding &binding,
                          const RigExecSurfaceProjectorFrames &frames,
                          const RigExecProjectorReads &reads,
                          const std::vector<GfVec3f> &basePoints,
                          const std::vector<GfVec3f> &finalPoints,
                          const std::string &who, GfMatrix4d *matrix,
                          std::vector<std::string> *diagnostics,
                          RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    if (op == RigExecRevisionOp::ShaderDials) {
        *matrix = RigExecPackShaderDialsT<GfMatrix4d>(reads.dials);
        return true;
    }
    if (op != RigExecRevisionOp::SurfaceProjector) {
        return false;
    }
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
    in.rayOrigin = reads.rayOrigin;
    in.rayDirection = reads.rayDirection;
    in.rayUp = reads.rayUp;
    in.shaderOffset = reads.shaderOffset;
    in.reproject = reads.reproject;
    in.hasSource = frames.named[0] && frames.resolved[0];
    in.sourceBase = frames.base[0];
    in.sourceFinal = frames.final[0];
    in.sourceSpaceNamed = frames.named[1];
    in.hasSourceSpace = frames.named[1] && frames.resolved[1];
    in.sourceSpaceBase = frames.base[1];
    in.sourceSpaceFinal = frames.final[1];
    in.spaceNamed = frames.named[2];
    in.hasSpace = frames.named[2] && frames.resolved[2];
    in.spaceFinal = frames.final[2];
    in.worldToMesh = binding.meshWorldInverse;
    return RigExecSolveSurfaceProjectorT(
        in, basePoints, finalPoints, reads.faceVertexCounts,
        reads.faceVertexIndices,
        static_cast<RigExecVertexNormalsFn>(&RigExecComputeVertexNormals), who,
        matrix, diagnostics, cache);
}

bool
RigExecEvaluateProjectorTarget(const UsdPrim &projectorPrim,
                               RigExecRevisionOp op,
                               const RigExecRevisionBinding &binding,
                               const RigExecSurfaceProjectorFrames &frames,
                               const std::vector<GfVec3f> &basePoints,
                               const std::vector<GfVec3f> &finalPoints,
                               const RigExecResolvedInputs *resolved,
                               UsdTimeCode time, GfMatrix4d *matrix,
                               std::vector<std::string> *diagnostics)
{
    if (!projectorPrim) {
        return false;
    }
    RigExecProjectorReads reads;
    RigExecReadProjectorTarget(projectorPrim, op, binding, resolved, time,
                               &reads);
    // Only the surface projector names its mover in a diagnostic.
    std::string who;
    if (op == RigExecRevisionOp::SurfaceProjector &&
        !RigExecSpellPathText(binding.moverPath, &who)) {
        who = binding.moverPath.GetString();
    }
    return RigExecRunProjectorTarget(op, binding, frames, reads, basePoints,
                                     finalPoints, who, matrix, diagnostics);
}

} // namespace rigExec
