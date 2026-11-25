#include "rigExecScene/geometryOps.h"

#include "rigExecOps/deform/blendShapeOp.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/derivedOps.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/projectorOps.h"
#include "rigExecOps/deform/ribbonOp.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wireOp.h"
#include "rigExecOps/deform/wrinkleOp.h"
#include "rigExecRigTypes/rigTypes.h"

#include "rigExec/api/opFields.h"
#include "rigExec/api/registry.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <memory>
#include <optional>

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(_tokens,
                         (sparse)(strict)(clamp)(classicLinear)(dualQuaternion));

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

Mat4d
ToStd(const GfMatrix4d &m)
{
    Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

RigExecArray<Vec3f>
ToStd(const std::vector<GfVec3f> &v)
{
    RigExecArray<Vec3f> out(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        out[i] = Vec3f(v[i][0], v[i][1], v[i][2]);
    return out;
}

template <class Range>
RigExecArray<Vec2f>
ToStd2(const Range &v)
{
    RigExecArray<Vec2f> out(v.size());
    size_t i = 0;
    for (const GfVec2f &x : v)
        out[i++] = Vec2f(x[0], x[1]);
    return out;
}

rigExecRig::PointLandmarks
ToStd(const std::array<GfVec3d, 4> &points)
{
    rigExecRig::PointLandmarks out;
    for (size_t k = 0; k < 4; ++k)
        out[k] = Vec3d(points[k][0], points[k][1], points[k][2]);
    return out;
}

rigExecRig::PointFrameArray
ToStd(const RigExecPointFrameArray &a)
{
    rigExecRig::PointFrameArray out;
    out.frames = RigExecArray<rigExecRig::PointFrame>(a.frames.size());
    for (size_t k = 0; k < a.frames.size(); ++k) {
        out.frames[k].points = ToStd(a.frames[k].points);
        out.frames[k].flags = a.frames[k].flags;
    }
    out.rests = RigExecArray<rigExecRig::PointLandmarks>(a.rests.size());
    for (size_t k = 0; k < a.rests.size(); ++k)
        out.rests[k] = ToStd(a.rests[k]);
    return out;
}

/// A sparse zero-default field the matrix and wire ops walk entry by entry,
/// as eval::EnvelopeIsSparseWalk decides it for every evaluator.
bool
SparseWalk(const RigExecWeightPacket &w)
{
    return w.valid && w.representation == _tokens->sparse &&
           w.defaultWeight == 0.0f && w.indices.size() == w.values.size() &&
           (w.rangePolicy.IsEmpty() || w.rangePolicy == _tokens->strict ||
            w.rangePolicy == _tokens->clamp);
}

template <class T>
RigExecArray<T>
ToArray(const std::vector<T> &v)
{
    return RigExecArray<T>(v.data(), v.size());
}

bool
Valid(const RigExecInputId &id)
{
    return id.index != kRigExecInvalidIndex;
}

/// The skin binding: the compiled layout when the packet has one.
const std::vector<int> &
SkinIndices(const RigExecMoverParameters &p)
{
    return p.skinTopology ? p.skinTopology->indices : p.skinIndices;
}

const std::vector<float> &
SkinWeights(const RigExecMoverParameters &p)
{
    return p.skinTopology ? p.skinTopology->weights : p.skinWeights;
}

int
SkinElementSize(const RigExecMoverParameters &p)
{
    return p.skinTopology ? p.skinTopology->elementSize : p.skinElementSize;
}

} // namespace

/// One payload of each op that has one, filled from a packet by
/// BuildPayload.
struct RigExecGeometryPayloadScratch {
    rigExecOps::MatrixDeformPayload matrix;
    rigExecOps::SkinPayload skin;
    rigExecOps::DeltaMushPayload mush;
    rigExecOps::WrinklePayload wrinkle;
    rigExecOps::LatticePayload lattice;
    rigExecOps::SurfaceProjectPayload surface;
    rigExecOps::WirePayload wire;
    rigExecOps::NormalsPayload normals;
    rigExecOps::BlendShapePayload blend;
    /// The field values of a candidate and of a node's payload, compared
    /// for an op that declares its payload fields.
    RigExecFieldValues candidateFields;
    RigExecFieldValues nodeFields;
};

void
RigExecGeometryPayloadScratchDeleter::operator()(
    RigExecGeometryPayloadScratch *scratch) const
{
    std::default_delete<RigExecGeometryPayloadScratch>()(scratch);
}

RigExecGeometryPayloadScratchPtr
RigExecMakeGeometryPayloadScratch()
{
    return RigExecGeometryPayloadScratchPtr(
        std::make_unique<RigExecGeometryPayloadScratch>().release());
}

namespace {

/// Sets \p payload to \p op's payload from \p p, kept in \p s, or to null
/// for an op without one. False for a skinning method the op does not
/// know. A blend shape's channels are not filled.
bool
BuildPayload(RigExecRevisionOp op, const RigExecMoverParameters &p,
             RigExecGeometryPayloadScratch &s, const void **payload)
{
    *payload = nullptr;
    switch (op) {
    case RigExecRevisionOp::Matrix:
        s.matrix.radial = p.radialWeight;
        *payload = &s.matrix;
        break;
    case RigExecRevisionOp::Skin:
        if (p.skinningMethod == _tokens->classicLinear)
            s.skin.method = rigExecOps::SkinMethod::ClassicLinear;
        else if (p.skinningMethod == _tokens->dualQuaternion)
            s.skin.method = rigExecOps::SkinMethod::DualQuaternion;
        else
            return false;
        s.skin.elementSize = SkinElementSize(p);
        *payload = &s.skin;
        break;
    case RigExecRevisionOp::DeltaMush:
        s.mush.iterations = p.mushIterations;
        s.mush.step = p.mushStep;
        s.mush.pinBorders = p.mushPinBorders;
        s.mush.distanceWeight = p.mushDistanceWeight;
        s.mush.displacement = p.mushDisplacement;
        s.mush.settings = p.mushSettings;
        *payload = &s.mush;
        break;
    case RigExecRevisionOp::Wrinkle:
        s.wrinkle.settings = p.wrinkleSettings;
        *payload = &s.wrinkle;
        break;
    case RigExecRevisionOp::Lattice:
        s.lattice.divisions = {p.divisions[0], p.divisions[1], p.divisions[2]};
        s.lattice.settings = p.latticeSettings;
        *payload = &s.lattice;
        break;
    case RigExecRevisionOp::SurfaceProject:
        s.surface.settings = p.surfaceSettings;
        *payload = &s.surface;
        break;
    case RigExecRevisionOp::Wire:
        s.wire.order = p.curveOrder;
        s.wire.knots = p.curveKnots;
        s.wire.dropoffDistance = p.dropoffDistance;
        *payload = &s.wire;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        s.normals.perCorner = p.normalsPerCorner;
        s.normals.leftHanded = p.normalsLeftHanded;
        *payload = &s.normals;
        break;
    case RigExecRevisionOp::BlendShape:
        s.blend.surfaceFrame = p.blendSurfaceFrame;
        *payload = &s.blend;
        break;
    default:
        break;
    }
    return true;
}

/// Whether \p a and \p b, payloads of \p d, hold the same field values.
bool
SameFields(const RigExecOpDesc &d, const void *a, const void *b,
           RigExecGeometryPayloadScratch &s)
{
    if (!d.payloadValues)
        return false;
    if (s.candidateFields.FieldCount() != d.payloadFieldCount)
        s.candidateFields = RigExecFieldValues(d.payloadFieldCount);
    if (s.nodeFields.FieldCount() != d.payloadFieldCount)
        s.nodeFields = RigExecFieldValues(d.payloadFieldCount);
    d.payloadValues(a, s.candidateFields);
    d.payloadValues(b, s.nodeFields);
    return s.candidateFields == s.nodeFields;
}

/// Whether \p p builds the payload \p node was built with.
bool
SamePayload(RigExecRevisionOp op, const RigExecMoverParameters &p,
            const RigExecGeometryOpNode &node,
            RigExecGeometryPayloadScratch *scratch)
{
    std::optional<RigExecGeometryPayloadScratch> local;
    if (!scratch)
        scratch = &local.emplace();
    const void *candidate = nullptr;
    if (!BuildPayload(op, p, *scratch, &candidate))
        return false;
    if (!candidate || !node.payload)
        return !candidate && !node.payload;
    const RigExecOpHandle *handle = RigExecGeometryOpHandle(op);
    const RigExecOpDesc &d = *RigExecRegistry::Get().Op(*handle).desc;
    if (d.payloadFieldCount)
        return SameFields(d, candidate, node.payload.get(), *scratch);
    return d.payload.equal && d.payload.equal(candidate, node.payload.get());
}

/// Whether a write of \p p into \p node would refuse moved settings.
bool
SettingsMoved(const RigExecMoverParameters &p,
              const RigExecGeometryOpNode &node,
              RigExecGeometryPayloadScratch *scratch)
{
    return !SamePayload(node.op, p, node, scratch) ||
           ((node.op == RigExecRevisionOp::Matrix ||
             node.op == RigExecRevisionOp::Wire) &&
            Valid(node.weightIndices) != SparseWalk(p.weights));
}

} // namespace

const RigExecOpHandle *
RigExecGeometryOpHandle(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return &rigExecOps::MatrixDeform();
    case RigExecRevisionOp::Skin:
        return &rigExecOps::Skin();
    case RigExecRevisionOp::BlendShape:
        return &rigExecOps::BlendShape();
    case RigExecRevisionOp::VolumeCorrect:
        return &rigExecOps::VolumeCorrect();
    case RigExecRevisionOp::Smooth:
        return &rigExecOps::Smooth();
    case RigExecRevisionOp::Lattice:
        return &rigExecOps::Lattice();
    case RigExecRevisionOp::SurfaceProject:
        return &rigExecOps::SurfaceProject();
    case RigExecRevisionOp::Ribbon:
        return &rigExecOps::Ribbon();
    case RigExecRevisionOp::Wire:
        return &rigExecOps::Wire();
    case RigExecRevisionOp::EmitGuidePoints:
        return &rigExecOps::GuidePoints();
    case RigExecRevisionOp::RecomputeNormals:
        return &rigExecOps::RecomputeNormals();
    case RigExecRevisionOp::RecomputeExtent:
        return &rigExecOps::RecomputeExtent();
    case RigExecRevisionOp::DeltaMush:
        return &rigExecOps::DeltaMush();
    case RigExecRevisionOp::Wrinkle:
        return &rigExecOps::Wrinkle();
    case RigExecRevisionOp::SurfaceProjector:
        return &rigExecOps::SurfaceProjector();
    case RigExecRevisionOp::ShaderDials:
        return &rigExecOps::ShaderDials();
    case RigExecRevisionOp::External:
        return nullptr;
    }
    return nullptr;
}

bool
RigExecGeometryOpSupported(RigExecRevisionOp op)
{
    return op != RigExecRevisionOp::BlendShape &&
           !RigExecIsDerivedMatrixOp(op) && RigExecGeometryOpHandle(op);
}

bool
RigExecAddGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                     RigExecRigDescBuilder *builder,
                     const RigExecGeometryOpNode *upstream,
                     RigExecGeometryOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null geometry op destination");
    RigExecGeometryOpNode result;
    result.op = op;
    auto kept = std::make_shared<RigExecGeometryPayloadScratch>();
    const void *payload = nullptr;
    if (!BuildPayload(op, p, *kept, &payload))
        return Fail(error, "skin: unknown skinning method");
    const RigExecPortHandle *in = nullptr, *out = nullptr, *envelope = nullptr;
    switch (op) {
    case RigExecRevisionOp::Matrix: {
        result.node = builder->AddOp(rigExecOps::MatrixDeform(), payload);
        in = &rigExecOps::MatrixDeformPoints();
        out = &rigExecOps::MatrixDeformOut();
        result.transform = builder->BindInput(
            result.node, rigExecOps::MatrixDeformTransform());
        if (SparseWalk(p.weights)) {
            result.weightIndices = builder->BindInput(
                result.node, rigExecOps::MatrixDeformWeightIndices());
            result.weightValues = builder->BindInput(
                result.node, rigExecOps::MatrixDeformWeights());
        } else {
            envelope = &rigExecOps::MatrixDeformEnvelope();
        }
        break;
    }
    case RigExecRevisionOp::Skin: {
        result.node = builder->AddOp(rigExecOps::Skin(), payload);
        in = &rigExecOps::SkinPoints();
        out = &rigExecOps::SkinOut();
        envelope = &rigExecOps::SkinEnvelope();
        result.transforms =
            builder->BindInput(result.node, rigExecOps::SkinTransforms());
        result.skinIndices =
            builder->BindInput(result.node, rigExecOps::SkinIndices());
        result.skinWeights =
            builder->BindInput(result.node, rigExecOps::SkinWeights());
        break;
    }
    case RigExecRevisionOp::VolumeCorrect:
        result.node = builder->AddOp(rigExecOps::VolumeCorrect());
        in = &rigExecOps::VolumeCorrectPoints();
        out = &rigExecOps::VolumeCorrectOut();
        envelope = &rigExecOps::VolumeCorrectEnvelope();
        result.reference = builder->BindInput(
            result.node, rigExecOps::VolumeCorrectReference());
        result.strength = builder->BindInput(
            result.node, rigExecOps::VolumeCorrectStrength());
        break;
    case RigExecRevisionOp::Smooth:
        result.node = builder->AddOp(rigExecOps::Smooth());
        in = &rigExecOps::SmoothPoints();
        out = &rigExecOps::SmoothOut();
        envelope = &rigExecOps::SmoothEnvelope();
        result.counts =
            builder->BindInput(result.node, rigExecOps::SmoothCounts());
        result.indices =
            builder->BindInput(result.node, rigExecOps::SmoothIndices());
        result.strength =
            builder->BindInput(result.node, rigExecOps::SmoothStrength());
        break;
    case RigExecRevisionOp::DeltaMush: {
        result.node = builder->AddOp(rigExecOps::DeltaMush(), payload);
        in = &rigExecOps::DeltaMushPoints();
        out = &rigExecOps::DeltaMushOut();
        envelope = &rigExecOps::DeltaMushEnvelope();
        result.rest =
            builder->BindInput(result.node, rigExecOps::DeltaMushRest());
        result.counts =
            builder->BindInput(result.node, rigExecOps::DeltaMushCounts());
        result.indices =
            builder->BindInput(result.node, rigExecOps::DeltaMushIndices());
        result.transform = builder->BindInput(
            result.node, rigExecOps::DeltaMushComputationToTarget());
        break;
    }
    case RigExecRevisionOp::Wrinkle: {
        result.node = builder->AddOp(rigExecOps::Wrinkle(), payload);
        in = &rigExecOps::WrinklePoints();
        out = &rigExecOps::WrinkleOut();
        envelope = &rigExecOps::WrinkleEnvelope();
        result.rest =
            builder->BindInput(result.node, rigExecOps::WrinkleRest());
        result.counts =
            builder->BindInput(result.node, rigExecOps::WrinkleCounts());
        result.indices =
            builder->BindInput(result.node, rigExecOps::WrinkleIndices());
        break;
    }
    case RigExecRevisionOp::Lattice: {
        result.node = builder->AddOp(rigExecOps::Lattice(), payload);
        in = &rigExecOps::LatticePoints();
        out = &rigExecOps::LatticeOut();
        envelope = &rigExecOps::LatticeEnvelope();
        result.posedCage =
            builder->BindInput(result.node, rigExecOps::LatticePosedCage());
        if (p.latticeSettings.regularGrid) {
            result.toSpace = builder->BindInput(
                result.node, rigExecOps::LatticeTargetToLattice());
            result.fromSpace = builder->BindInput(
                result.node, rigExecOps::LatticeLatticeToTarget());
            result.metric = builder->BindInput(
                result.node, rigExecOps::LatticeCageToLattice());
        } else {
            result.rest = builder->BindInput(result.node,
                                             rigExecOps::LatticeRestPoints());
            result.restCage =
                builder->BindInput(result.node, rigExecOps::LatticeRestCage());
        }
        break;
    }
    case RigExecRevisionOp::SurfaceProject: {
        result.node = builder->AddOp(rigExecOps::SurfaceProject(), payload);
        in = &rigExecOps::SurfaceProjectPoints();
        out = &rigExecOps::SurfaceProjectOut();
        envelope = &rigExecOps::SurfaceProjectEnvelope();
        result.surface = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectSurface());
        result.counts =
            builder->BindInput(result.node, rigExecOps::SurfaceProjectCounts());
        result.indices = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectIndices());
        result.strength = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectStrength());
        result.toSpace = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectTargetToSurface());
        result.fromSpace = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectSurfaceToTarget());
        result.metric = builder->BindInput(
            result.node, rigExecOps::SurfaceProjectSurfaceToMetric());
        break;
    }
    case RigExecRevisionOp::Ribbon:
        result.node = builder->AddOp(rigExecOps::Ribbon());
        in = &rigExecOps::RibbonPoints();
        out = &rigExecOps::RibbonOut();
        envelope = &rigExecOps::RibbonEnvelope();
        result.bindCoords =
            builder->BindInput(result.node, rigExecOps::RibbonBindCoords());
        result.frames =
            builder->BindInput(result.node, rigExecOps::RibbonFrames());
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        result.node = builder->AddOp(rigExecOps::GuidePoints());
        in = &rigExecOps::GuidePointsPoints();
        out = &rigExecOps::GuidePointsOut();
        envelope = &rigExecOps::GuidePointsEnvelope();
        result.frames =
            builder->BindInput(result.node, rigExecOps::GuidePointsFrames());
        break;
    case RigExecRevisionOp::Wire: {
        result.node = builder->AddOp(rigExecOps::Wire(), payload);
        in = &rigExecOps::WirePoints();
        out = &rigExecOps::WireOut();
        result.rest =
            builder->BindInput(result.node, rigExecOps::WireRestCurve());
        result.curve = builder->BindInput(result.node, rigExecOps::WireCurve());
        result.bindCoords =
            builder->BindInput(result.node, rigExecOps::WireBindCoords());
        // The field's shape picks the ports.
        if (SparseWalk(p.weights)) {
            result.weightIndices = builder->BindInput(
                result.node, rigExecOps::WireWeightIndices());
            result.weightValues =
                builder->BindInput(result.node, rigExecOps::WireWeights());
        } else {
            envelope = &rigExecOps::WireEnvelope();
        }
        break;
    }
    case RigExecRevisionOp::RecomputeNormals: {
        result.node = builder->AddOp(rigExecOps::RecomputeNormals(), payload);
        in = &rigExecOps::RecomputeNormalsAuthored();
        out = &rigExecOps::RecomputeNormalsOut();
        envelope = &rigExecOps::RecomputeNormalsEnvelope();
        result.finalPoints = builder->BindInput(
            result.node, rigExecOps::RecomputeNormalsPoints());
        result.counts = builder->BindInput(
            result.node, rigExecOps::RecomputeNormalsCounts());
        result.indices = builder->BindInput(
            result.node, rigExecOps::RecomputeNormalsIndices());
        break;
    }
    case RigExecRevisionOp::RecomputeExtent:
        result.node = builder->AddOp(rigExecOps::RecomputeExtent());
        in = &rigExecOps::RecomputeExtentAuthored();
        out = &rigExecOps::RecomputeExtentOut();
        envelope = &rigExecOps::RecomputeExtentEnvelope();
        result.finalPoints = builder->BindInput(
            result.node, rigExecOps::RecomputeExtentPoints());
        result.widths = builder->BindInput(result.node,
                                           rigExecOps::RecomputeExtentWidths());
        break;
    default:
        return Fail(error, "geometry op has no deform op yet");
    }
    if (upstream)
        builder->Connect(upstream->node, *upstream->outPort, result.node, *in);
    else
        result.points = builder->BindInput(result.node, *in);
    if (envelope)
        result.envelope = builder->BindInput(result.node, *envelope);
    result.outPort = out;
    result.out = builder->BindOutput(result.node, *out);
    if (payload)
        result.payload = std::shared_ptr<const void>(std::move(kept), payload);
    *node = result;
    return true;
}

bool
RigExecWriteGeometryOpInputs(const RigExecMoverParameters &p,
                             const RigExecGeometryOpNode &node,
                             const std::vector<GfVec3f> *points,
                             size_t pointCount, RigExecEvaluator *evaluator,
                             std::string *error,
                             RigExecGeometryPayloadScratch *scratch)
{
    if (!evaluator)
        return Fail(error, "null evaluator");
    if (SettingsMoved(p, node, scratch))
        return Fail(error, "op settings changed");
    if (Valid(node.envelope)) {
        std::vector<float> resolved;
        if (!p.weights.ResolveAll(pointCount, &resolved))
            return Fail(error, "envelope does not resolve over the points");
        const RigExecArray<float> envelope = ToArray(resolved);
        if (!evaluator->SetInput(node.envelope, &envelope, error))
            return false;
    } else if (Valid(node.weightIndices)) {
        if (!SparseWalk(p.weights))
            return Fail(error, "weight field is no longer sparse");
        const RigExecArray<int> indices = ToArray(p.weights.indices);
        const RigExecArray<float> values = ToArray(p.weights.values);
        if (!evaluator->SetInput(node.weightIndices, &indices, error) ||
            !evaluator->SetInput(node.weightValues, &values, error))
            return false;
    }
    if (Valid(node.points)) {
        if (!points || points->size() != pointCount)
            return Fail(error, "points missing or of another length");
        const RigExecArray<Vec3f> in = ToStd(*points);
        if (!evaluator->SetInput(node.points, &in, error))
            return false;
    }
    switch (node.op) {
    case RigExecRevisionOp::Matrix: {
        const Mat4d transform = ToStd(p.transform);
        return evaluator->SetInput(node.transform, &transform, error);
    }
    case RigExecRevisionOp::Skin: {
        RigExecArray<Mat4d> transforms(p.skinTransforms.size());
        for (size_t i = 0; i < p.skinTransforms.size(); ++i)
            transforms[i] = ToStd(p.skinTransforms[i]);
        const RigExecArray<int> indices = ToArray(SkinIndices(p));
        const RigExecArray<float> weights = ToArray(SkinWeights(p));
        return evaluator->SetInput(node.transforms, &transforms, error) &&
               evaluator->SetInput(node.skinIndices, &indices, error) &&
               evaluator->SetInput(node.skinWeights, &weights, error);
    }
    case RigExecRevisionOp::VolumeCorrect:
        return evaluator->SetInput(node.reference, &p.referenceVolume, error) &&
               evaluator->SetInput(node.strength, &p.strength, error);
    case RigExecRevisionOp::Smooth: {
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        return evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error) &&
               evaluator->SetInput(node.strength, &p.strength, error);
    }
    case RigExecRevisionOp::DeltaMush: {
        const RigExecArray<Vec3f> rest = ToStd(p.restPoints);
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        const Mat4d map = ToStd(p.mushComputationToTarget);
        return evaluator->SetInput(node.rest, &rest, error) &&
               evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error) &&
               evaluator->SetInput(node.transform, &map, error);
    }
    case RigExecRevisionOp::BlendShape: {
        if (!Valid(node.rest))
            return true;
        const RigExecArray<Vec3f> rest = ToStd(p.restPoints);
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        return evaluator->SetInput(node.rest, &rest, error) &&
               evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error);
    }
    case RigExecRevisionOp::Wrinkle: {
        const RigExecArray<Vec3f> rest = ToStd(p.restPoints);
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        return evaluator->SetInput(node.rest, &rest, error) &&
               evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error);
    }
    case RigExecRevisionOp::Lattice: {
        const RigExecArray<Vec3f> posed = ToStd(p.auxPointsB);
        if (!evaluator->SetInput(node.posedCage, &posed, error))
            return false;
        if (Valid(node.restCage)) {
            const RigExecArray<Vec3f> rest = ToStd(p.restPoints);
            const RigExecArray<Vec3f> cage = ToStd(p.auxPoints);
            return evaluator->SetInput(node.rest, &rest, error) &&
                   evaluator->SetInput(node.restCage, &cage, error);
        }
        const Mat4d to = ToStd(p.targetToLattice),
                    from = ToStd(p.latticeToTarget),
                    cage = ToStd(p.cageToLattice);
        return evaluator->SetInput(node.toSpace, &to, error) &&
               evaluator->SetInput(node.fromSpace, &from, error) &&
               evaluator->SetInput(node.metric, &cage, error);
    }
    case RigExecRevisionOp::SurfaceProject: {
        const RigExecArray<Vec3f> surface = ToStd(p.auxPoints);
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        const Mat4d to = ToStd(p.targetToSurface),
                    from = ToStd(p.surfaceToTarget),
                    metric = ToStd(p.surfaceToMetric);
        return evaluator->SetInput(node.surface, &surface, error) &&
               evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error) &&
               evaluator->SetInput(node.strength, &p.strength, error) &&
               evaluator->SetInput(node.toSpace, &to, error) &&
               evaluator->SetInput(node.fromSpace, &from, error) &&
               evaluator->SetInput(node.metric, &metric, error);
    }
    case RigExecRevisionOp::Ribbon: {
        const RigExecArray<Vec2f> binds = ToStd2(p.bindCoords);
        const rigExecRig::PointFrameArray frames = ToStd(p.frames);
        return evaluator->SetInput(node.bindCoords, &binds, error) &&
               evaluator->SetInput(node.frames, &frames, error);
    }
    case RigExecRevisionOp::EmitGuidePoints: {
        const rigExecRig::PointFrameArray frames = ToStd(p.frames);
        return evaluator->SetInput(node.frames, &frames, error);
    }
    case RigExecRevisionOp::Wire: {
        const RigExecArray<Vec3f> rest = ToStd(p.restPoints);
        const RigExecArray<Vec3f> posed = ToStd(p.auxPoints);
        const RigExecArray<Vec2f> binds = ToStd2(p.wireBindCoords);
        return evaluator->SetInput(node.rest, &rest, error) &&
               evaluator->SetInput(node.curve, &posed, error) &&
               evaluator->SetInput(node.bindCoords, &binds, error);
    }
    case RigExecRevisionOp::RecomputeNormals: {
        const RigExecArray<Vec3f> final = ToStd(p.auxPoints);
        const RigExecArray<int> counts = ToArray(p.topologyCounts);
        const RigExecArray<int> indices = ToArray(p.topologyIndices);
        return evaluator->SetInput(node.finalPoints, &final, error) &&
               evaluator->SetInput(node.counts, &counts, error) &&
               evaluator->SetInput(node.indices, &indices, error);
    }
    case RigExecRevisionOp::RecomputeExtent: {
        const RigExecArray<Vec3f> final = ToStd(p.auxPoints);
        const RigExecArray<float> widths = ToArray(p.widths);
        return evaluator->SetInput(node.finalPoints, &final, error) &&
               evaluator->SetInput(node.widths, &widths, error);
    }
    default:
        return Fail(error, "geometry op has no deform op yet");
    }
}

namespace {

Vec3d
ToStd(const GfVec3d &v)
{
    return Vec3d(v[0], v[1], v[2]);
}

bool
SameSettings(const RigExecProjectorReads &a, const RigExecProjectorReads &b)
{
    return a.rayOrigin == b.rayOrigin && a.rayDirection == b.rayDirection &&
           a.rayUp == b.rayUp && a.shaderOffset == b.shaderOffset &&
           a.reproject == b.reproject;
}

/// The kernel's own reading of a frame slot: named and resolved. The space
/// slot (2) has no base.
bool
Fed(const RigExecSurfaceProjectorFrames &frames, int k)
{
    return frames.named[k] && frames.resolved[k];
}

} // namespace

bool
RigExecGeometryMatrixOpSupported(RigExecRevisionOp op)
{
    return RigExecIsDerivedMatrixOp(op) && RigExecGeometryOpHandle(op);
}

bool
RigExecAddGeometryMatrixOp(RigExecRevisionOp op,
                           const RigExecSurfaceProjectorFrames &frames,
                           const RigExecProjectorReads &reads,
                           RigExecRigDescBuilder *builder,
                           RigExecGeometryMatrixNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "no builder or node");
    RigExecGeometryMatrixNode result;
    result.op = op;
    if (op == RigExecRevisionOp::ShaderDials) {
        result.node = builder->AddOp(rigExecOps::ShaderDials());
        result.dials =
            builder->BindInput(result.node, rigExecOps::ShaderDialsDials());
        result.outPort = &rigExecOps::ShaderDialsOut();
    } else if (op == RigExecRevisionOp::SurfaceProjector) {
        rigExecOps::SurfaceProjectorPayload payload;
        payload.rayOrigin = ToStd(reads.rayOrigin);
        payload.rayDirection = ToStd(reads.rayDirection);
        payload.rayUp = ToStd(reads.rayUp);
        payload.shaderOffset = ToStd(reads.shaderOffset);
        payload.reproject = reads.reproject;
        payload.sourceSpaceNamed = frames.named[1];
        payload.spaceNamed = frames.named[2];
        result.settings = reads;
        result.settings.dials.clear();
        result.settings.faceVertexCounts.clear();
        result.settings.faceVertexIndices.clear();
        const RigExecNodeId n =
            builder->AddOp(rigExecOps::SurfaceProjector(), &payload);
        result.node = n;
        result.basePoints =
            builder->BindInput(n, rigExecOps::SurfaceProjectorBasePoints());
        result.points =
            builder->BindInput(n, rigExecOps::SurfaceProjectorPoints());
        result.counts =
            builder->BindInput(n, rigExecOps::SurfaceProjectorCounts());
        result.indices =
            builder->BindInput(n, rigExecOps::SurfaceProjectorIndices());
        result.worldToMesh =
            builder->BindInput(n, rigExecOps::SurfaceProjectorWorldToMesh());
        const RigExecPortHandle *bases[3] = {
            &rigExecOps::SurfaceProjectorSourceBase(),
            &rigExecOps::SurfaceProjectorSourceSpaceBase(), nullptr};
        const RigExecPortHandle *finals[3] = {
            &rigExecOps::SurfaceProjectorSourceFinal(),
            &rigExecOps::SurfaceProjectorSourceSpaceFinal(),
            &rigExecOps::SurfaceProjectorSpaceFinal()};
        for (int k = 0; k < 3; ++k) {
            result.resolved[k] = Fed(frames, k);
            if (!result.resolved[k])
                continue;
            if (bases[k])
                result.frameBase[k] = builder->BindInput(n, *bases[k]);
            result.frameFinal[k] = builder->BindInput(n, *finals[k]);
        }
        result.outPort = &rigExecOps::SurfaceProjectorOut();
    } else {
        return Fail(error, "not a derived matrix op");
    }
    result.out = builder->BindOutput(result.node, *result.outPort);
    *node = result;
    return true;
}

bool
RigExecWriteGeometryMatrixInputs(const RigExecRevisionBinding &binding,
                                 const RigExecSurfaceProjectorFrames &frames,
                                 const RigExecProjectorReads &reads,
                                 const std::vector<GfVec3f> &base,
                                 const std::vector<GfVec3f> &final,
                                 const RigExecGeometryMatrixNode &node,
                                 RigExecEvaluator *evaluator,
                                 std::string *error)
{
    if (!evaluator)
        return Fail(error, "no evaluator");
    if (node.op == RigExecRevisionOp::ShaderDials) {
        const RigExecArray<double> dials = ToArray(reads.dials);
        return evaluator->SetInput(node.dials, &dials, error);
    }
    for (int k = 0; k < 3; ++k)
        if (Fed(frames, k) != node.resolved[k])
            return Fail(error, "projector frame resolution changed");
    if (!SameSettings(reads, node.settings))
        return Fail(error, "projector ray settings changed");
    const RigExecArray<Vec3f> basePoints = ToStd(base);
    const RigExecArray<Vec3f> points = ToStd(final);
    const RigExecArray<int> counts = ToArray(reads.faceVertexCounts);
    const RigExecArray<int> indices = ToArray(reads.faceVertexIndices);
    const Mat4d worldToMesh = ToStd(binding.meshWorldInverse);
    if (!evaluator->SetInput(node.basePoints, &basePoints, error) ||
        !evaluator->SetInput(node.points, &points, error) ||
        !evaluator->SetInput(node.counts, &counts, error) ||
        !evaluator->SetInput(node.indices, &indices, error) ||
        !evaluator->SetInput(node.worldToMesh, &worldToMesh, error))
        return false;
    for (int k = 0; k < 3; ++k) {
        if (!node.resolved[k])
            continue;
        const Mat4d b = ToStd(frames.base[k]);
        const Mat4d f = ToStd(frames.final[k]);
        if ((Valid(node.frameBase[k]) &&
             !evaluator->SetInput(node.frameBase[k], &b, error)) ||
            !evaluator->SetInput(node.frameFinal[k], &f, error))
            return false;
    }
    return true;
}

namespace {

std::vector<Vec3f>
ToStdVector(const std::vector<GfVec3f> &v)
{
    std::vector<Vec3f> out(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        out[i] = Vec3f(v[i][0], v[i][1], v[i][2]);
    return out;
}

bool
SameSample(const RigExecGeometryOpNode::BlendSample &a,
           const RigExecBlendSampleData &b)
{
    if (a.activation != b.activation || a.layout != b.layout)
        return false;
    if (a.layout)
        return true;
    const size_t n = b.PointCount();
    return a.points.size() == n &&
           std::equal(a.points.begin(), a.points.end(), b.PointData());
}

} // namespace

bool
RigExecAddGeometryBlendOp(const std::vector<RigExecBlendChannel> &channels,
                          const RigExecMoverParameters &p,
                          RigExecRigDescBuilder *builder,
                          const RigExecGeometryOpNode *upstream,
                          RigExecGeometryOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null geometry op destination");
    RigExecGeometryOpNode result;
    result.op = RigExecRevisionOp::BlendShape;
    rigExecOps::BlendShapePayload payload;
    payload.surfaceFrame = p.blendSurfaceFrame;
    payload.channels.resize(channels.size());
    result.blendSamples.resize(channels.size());
    for (size_t c = 0; c < channels.size(); ++c)
        for (const RigExecBlendSampleData &s : channels[c].samples) {
            rigExecOps::BlendShapeSample sample;
            sample.activation = s.activation;
            RigExecGeometryOpNode::BlendSample kept;
            kept.activation = s.activation;
            kept.layout = s.layout;
            if (s.layout) {
                auto layout = std::make_shared<rigExecOps::BlendShapeLayout>();
                layout->offsets = ToStdVector(s.layout->offsets);
                layout->indices = s.layout->indices;
                layout->pointCount = s.layout->pointCount;
                layout->valid = s.layout->valid;
                sample.layout = std::move(layout);
            } else {
                kept.points.assign(s.PointData(),
                                   s.PointData() + s.PointCount());
                sample.points = ToStdVector(kept.points);
            }
            payload.channels[c].samples.push_back(std::move(sample));
            result.blendSamples[c].push_back(std::move(kept));
        }
    const RigExecNodeId n = builder->AddOp(rigExecOps::BlendShape(), &payload);
    result.node = n;
    result.channelWeights =
        builder->BindInput(n, rigExecOps::BlendShapeWeights());
    result.base = builder->BindInput(n, rigExecOps::BlendShapeBase());
    if (p.blendSurfaceFrame) {
        result.rest = builder->BindInput(n, rigExecOps::BlendShapeRest());
        result.counts = builder->BindInput(n, rigExecOps::BlendShapeCounts());
        result.indices = builder->BindInput(n, rigExecOps::BlendShapeIndices());
    }
    if (upstream)
        builder->Connect(upstream->node, *upstream->outPort, n,
                         rigExecOps::BlendShapePoints());
    else
        result.points = builder->BindInput(n, rigExecOps::BlendShapePoints());
    result.envelope = builder->BindInput(n, rigExecOps::BlendShapeEnvelope());
    result.outPort = &rigExecOps::BlendShapeOut();
    result.out = builder->BindOutput(n, *result.outPort);
    auto kept = std::make_shared<RigExecGeometryPayloadScratch>();
    const void *settings = nullptr;
    BuildPayload(result.op, p, *kept, &settings);
    result.payload = std::shared_ptr<const void>(std::move(kept), settings);
    *node = std::move(result);
    return true;
}

bool
RigExecWriteGeometryBlendInputs(
    const std::vector<RigExecBlendChannel> &channels,
    const RigExecMoverParameters &p, const RigExecGeometryOpNode &node,
    const std::vector<GfVec3f> &base, const std::vector<GfVec3f> *points,
    size_t pointCount, RigExecEvaluator *evaluator, std::string *error,
    RigExecGeometryPayloadScratch *scratch)
{
    if (node.op != RigExecRevisionOp::BlendShape)
        return Fail(error, "not a blend shape node");
    if (channels.size() != node.blendSamples.size())
        return Fail(error, "blend channels changed");
    for (size_t c = 0; c < channels.size(); ++c) {
        const auto &kept = node.blendSamples[c];
        const auto &now = channels[c].samples;
        if (kept.size() != now.size())
            return Fail(error, "blend samples changed");
        for (size_t k = 0; k < kept.size(); ++k)
            if (!SameSample(kept[k], now[k]))
                return Fail(error, "blend samples changed");
    }
    if (!RigExecWriteGeometryOpInputs(p, node, points, pointCount, evaluator,
                                      error, scratch))
        return false;
    RigExecArray<float> weights(channels.size());
    for (size_t c = 0; c < channels.size(); ++c)
        weights[c] = channels[c].weight;
    const RigExecArray<Vec3f> measured = ToStd(base);
    return evaluator->SetInput(node.channelWeights, &weights, error) &&
           evaluator->SetInput(node.base, &measured, error);
}

bool
RigExecGeometryOpNodeStale(RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           const std::vector<RigExecBlendChannel> *channels,
                           const RigExecGeometryOpNode &node,
                           RigExecGeometryPayloadScratch *scratch)
{
    if (node.op != op || SettingsMoved(p, node, scratch))
        return true;
    if (!channels)
        return false;
    if (channels->size() != node.blendSamples.size())
        return true;
    for (size_t c = 0; c < channels->size(); ++c) {
        const auto &kept = node.blendSamples[c];
        const auto &now = (*channels)[c].samples;
        if (kept.size() != now.size())
            return true;
        for (size_t k = 0; k < kept.size(); ++k)
            if (!SameSample(kept[k], now[k]))
                return true;
    }
    return false;
}

bool
RigExecGeometryMatrixNodeStale(RigExecRevisionOp op,
                               const RigExecSurfaceProjectorFrames &frames,
                               const RigExecProjectorReads &reads,
                               const RigExecGeometryMatrixNode &node)
{
    if (node.op != op)
        return true;
    if (op == RigExecRevisionOp::ShaderDials)
        return false;
    for (int k = 0; k < 3; ++k)
        if (Fed(frames, k) != node.resolved[k])
            return true;
    return !SameSettings(reads, node.settings);
}

} // namespace rigExec
