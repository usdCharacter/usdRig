#include "rigExecScene/geometryOps.h"

#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/ribbonOp.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wireOp.h"
#include "rigExecOps/deform/wrinkleOp.h"
#include "rigExecRigTypes/rigTypes.h"

#include "pxr/base/tf/staticTokens.h"

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(_tokens, (sparse)(strict)(clamp));

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

/// A sparse zero-default field the wire op folds into its motion, as
/// RigExecWireTakesSparseEnvelope decides it for the native wire.
bool
WireSparse(const RigExecWeightPacket &w)
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

bool
RigExecGeometryOpSupported(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::Smooth:
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle:
    case RigExecRevisionOp::Lattice:
    case RigExecRevisionOp::SurfaceProject:
    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints:
    case RigExecRevisionOp::Wire:
        return true;
    default:
        return false;
    }
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
    const RigExecPortHandle *in = nullptr, *out = nullptr, *envelope = nullptr;
    switch (op) {
    case RigExecRevisionOp::Matrix: {
        rigExecOps::MatrixDeformPayload payload;
        payload.radial = p.radialWeight;
        result.node = builder->AddOp(rigExecOps::MatrixDeform(), &payload);
        in = &rigExecOps::MatrixDeformPoints();
        out = &rigExecOps::MatrixDeformOut();
        envelope = &rigExecOps::MatrixDeformEnvelope();
        result.transform = builder->BindInput(
            result.node, rigExecOps::MatrixDeformTransform());
        break;
    }
    case RigExecRevisionOp::Skin: {
        rigExecOps::SkinPayload payload;
        // The authored method token is read once, here.
        if (p.skinningMethod == "classicLinear")
            payload.method = rigExecOps::SkinMethod::ClassicLinear;
        else if (p.skinningMethod == "dualQuaternion")
            payload.method = rigExecOps::SkinMethod::DualQuaternion;
        else
            return Fail(error, "skin: unknown skinning method");
        payload.elementSize = SkinElementSize(p);
        result.node = builder->AddOp(rigExecOps::Skin(), &payload);
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
        rigExecOps::DeltaMushPayload payload;
        payload.iterations = p.mushIterations;
        payload.step = p.mushStep;
        payload.pinBorders = p.mushPinBorders;
        payload.distanceWeight = p.mushDistanceWeight;
        payload.displacement = p.mushDisplacement;
        payload.settings = p.mushSettings;
        result.node = builder->AddOp(rigExecOps::DeltaMush(), &payload);
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
        rigExecOps::WrinklePayload payload;
        payload.settings = p.wrinkleSettings;
        result.node = builder->AddOp(rigExecOps::Wrinkle(), &payload);
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
        rigExecOps::LatticePayload payload;
        payload.divisions = {p.divisions[0], p.divisions[1], p.divisions[2]};
        payload.settings = p.latticeSettings;
        result.node = builder->AddOp(rigExecOps::Lattice(), &payload);
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
        rigExecOps::SurfaceProjectPayload payload;
        payload.settings = p.surfaceSettings;
        result.node = builder->AddOp(rigExecOps::SurfaceProject(), &payload);
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
        rigExecOps::WirePayload payload;
        payload.order = p.curveOrder;
        payload.knots = p.curveKnots;
        payload.dropoffDistance = p.dropoffDistance;
        result.node = builder->AddOp(rigExecOps::Wire(), &payload);
        in = &rigExecOps::WirePoints();
        out = &rigExecOps::WireOut();
        result.rest =
            builder->BindInput(result.node, rigExecOps::WireRestCurve());
        result.curve = builder->BindInput(result.node, rigExecOps::WireCurve());
        result.bindCoords =
            builder->BindInput(result.node, rigExecOps::WireBindCoords());
        // The field's shape picks the ports, as it picks the native path.
        if (WireSparse(p.weights)) {
            result.weightIndices = builder->BindInput(
                result.node, rigExecOps::WireWeightIndices());
            result.weightValues =
                builder->BindInput(result.node, rigExecOps::WireWeights());
        } else {
            envelope = &rigExecOps::WireEnvelope();
        }
        break;
    }
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
    *node = result;
    return true;
}

bool
RigExecWriteGeometryOpInputs(const RigExecMoverParameters &p,
                             const RigExecGeometryOpNode &node,
                             const std::vector<GfVec3f> *points,
                             size_t pointCount, RigExecEvaluator *evaluator,
                             std::string *error)
{
    if (!evaluator)
        return Fail(error, "null evaluator");
    if (Valid(node.envelope)) {
        std::vector<float> resolved;
        if (!p.weights.ResolveAll(pointCount, &resolved))
            return Fail(error, "envelope does not resolve over the points");
        const RigExecArray<float> envelope = ToArray(resolved);
        if (!evaluator->SetInput(node.envelope, &envelope, error))
            return false;
    } else if (Valid(node.weightIndices)) {
        if (!WireSparse(p.weights))
            return Fail(error, "wire field is no longer sparse");
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
    default:
        return Fail(error, "geometry op has no deform op yet");
    }
}

} // namespace rigExec
