#include "rigExecScene/geometryOps.h"

#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/skinOp.h"

namespace rigExec {

namespace {

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
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
    default:
        return Fail(error, "geometry op has no deform op yet");
    }
    if (upstream)
        builder->Connect(upstream->node, *upstream->outPort, result.node, *in);
    else
        result.points = builder->BindInput(result.node, *in);
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
    std::vector<float> resolved;
    if (!p.weights.ResolveAll(pointCount, &resolved))
        return Fail(error, "envelope does not resolve over the points");
    const RigExecArray<float> envelope = ToArray(resolved);
    if (!evaluator->SetInput(node.envelope, &envelope, error))
        return false;
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
    default:
        return Fail(error, "geometry op has no deform op yet");
    }
}

} // namespace rigExec
