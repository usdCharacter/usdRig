#include "rigExecScene/propertyOps.h"

#include "rigExecOps/property/mathOps.h"

#include <cmath>

namespace rigExec {

namespace {

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3f;

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

Vec3f
ToStd(const GfVec3f &v)
{
    return Vec3f(v[0], v[1], v[2]);
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

bool
Finite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

/// The record's curve keys and tangents as RigExecRunProperty reads them.
bool
CurveKeys(const RigExecFloatPropertyRecord &record,
          rigExecOps::MathPayload *payload)
{
    const GfVec2f *keys = record.keyData ? record.keyData : record.keys.data();
    const size_t keyCount =
        record.keyData ? record.keyCount : record.keys.size();
    if (!keyCount || !RigExecValidateLinearKeys(keys, keyCount))
        return false;
    for (size_t i = 0; i < keyCount; ++i)
        payload->keys.emplace_back(keys[i][0], keys[i][1]);
    const GfVec2f *tangents =
        record.tangentData ? record.tangentData : record.tangents.data();
    const size_t tangentCount =
        record.tangentData ? record.tangentCount : record.tangents.size();
    if (record.hasTangents && tangentCount) {
        if (tangentCount != keyCount)
            return false;
        for (size_t i = 0; i < tangentCount; ++i)
            payload->tangents.emplace_back(tangents[i][0], tangents[i][1]);
    }
    return true;
}

} // namespace

bool
RigExecAddPropertyOp(const RigExecScenePropertyDescriptor &descriptor,
                     RigExecRigDescBuilder *builder,
                     const RigExecPropertyOpNode *upstream,
                     RigExecPropertyOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null property op destination");
    const std::string where = descriptor.mover.GetString();
    RigExecPropertyOpNode result;
    result.kind = descriptor.record.index();
    if (upstream && upstream->kind != result.kind)
        return Fail(error, where + ": upstream property op has another type");
    rigExecOps::MathPayload payload;
    const RigExecOpHandle *op = nullptr;
    const RigExecPortHandle *in = nullptr, *out = nullptr;
    if (const auto *f =
            std::get_if<RigExecFloatPropertyRecord>(&descriptor.record)) {
        if (!f->opValid)
            return Fail(error, where + ": no valid operation");
        payload.op = f->op;
        if (f->op == RigExecPropertyOp::Curve && !CurveKeys(*f, &payload))
            return Fail(error, where + ": curve keys are not usable");
        if (f->op == RigExecPropertyOp::Path)
            payload.path = f->path;
        op = &rigExecOps::FloatMath();
        in = &rigExecOps::FloatIn();
        out = &rigExecOps::FloatOut();
    } else if (const auto *v =
                   std::get_if<RigExecVec3PropertyRecord>(&descriptor.record)) {
        if (!v->opValid)
            return Fail(error, where + ": no valid operation");
        payload.op = v->op;
        op = &rigExecOps::Vec3fMath();
        in = &rigExecOps::Vec3fIn();
        out = &rigExecOps::Vec3fOut();
    } else {
        const auto &m =
            std::get<RigExecMatrixPropertyRecord>(descriptor.record);
        if (!m.opValid)
            return Fail(error, where + ": no valid operation");
        payload.op = m.op;
        op = &rigExecOps::MatrixMath();
        in = &rigExecOps::MatrixIn();
        out = &rigExecOps::MatrixOut();
    }
    result.node = builder->AddOp(*op, &payload);
    if (upstream)
        builder->Connect(upstream->node, *out, result.node, *in);
    else
        result.in = builder->BindInput(result.node, *in);
    result.out = builder->BindOutput(result.node, *out);
    switch (result.kind) {
    case 0:
        result.value =
            builder->BindInput(result.node, rigExecOps::FloatValue());
        result.min = builder->BindInput(result.node, rigExecOps::FloatMin());
        result.max = builder->BindInput(result.node, rigExecOps::FloatMax());
        result.weight =
            builder->BindInput(result.node, rigExecOps::FloatWeight());
        if (payload.op == RigExecPropertyOp::Path)
            result.pathPoints =
                builder->BindInput(result.node, rigExecOps::FloatPathPoints());
        break;
    case 1:
        result.value =
            builder->BindInput(result.node, rigExecOps::Vec3fValue());
        result.min = builder->BindInput(result.node, rigExecOps::Vec3fMin());
        result.max = builder->BindInput(result.node, rigExecOps::Vec3fMax());
        result.weight =
            builder->BindInput(result.node, rigExecOps::Vec3fWeight());
        break;
    default:
        result.value =
            builder->BindInput(result.node, rigExecOps::MatrixValue());
        result.weight =
            builder->BindInput(result.node, rigExecOps::MatrixWeight());
        break;
    }
    *node = result;
    return true;
}

bool
RigExecWritePropertyOpInputs(const RigExecScenePropertyDescriptor &descriptor,
                             const RigExecPropertyOpNode &node, float envelope,
                             RigExecEvaluator *evaluator, std::string *error)
{
    if (!evaluator)
        return Fail(error, "null evaluator");
    const std::string where = descriptor.mover.GetString();
    if (const auto *f =
            std::get_if<RigExecFloatPropertyRecord>(&descriptor.record)) {
        if (!std::isfinite(f->value) || !std::isfinite(f->minimum) ||
            !std::isfinite(f->maximum))
            return Fail(error, where + ": non-finite value, min or max");
        RigExecArray<Vec3f> points;
        if (node.pathPoints.index != kRigExecInvalidIndex) {
            RigExecPathCurve curve;
            curve.points =
                f->pathPointData ? f->pathPointData->data() : nullptr;
            curve.pointCount = f->pathPointCount;
            curve.basis = f->path.basis;
            if (!f->path.valid || !RigExecPathCurveValid(curve))
                return Fail(error, where + ": path has no valid curve");
            points.Resize(f->pathPointCount);
            for (size_t i = 0; i < f->pathPointCount; ++i)
                points[i] = ToStd(f->pathPointData[i]);
        }
        return evaluator->SetInput(node.value, &f->value, error) &&
               evaluator->SetInput(node.min, &f->minimum, error) &&
               evaluator->SetInput(node.max, &f->maximum, error) &&
               evaluator->SetInput(node.weight, &envelope, error) &&
               (node.pathPoints.index == kRigExecInvalidIndex ||
                evaluator->SetInput(node.pathPoints, &points, error));
    }
    if (const auto *v =
            std::get_if<RigExecVec3PropertyRecord>(&descriptor.record)) {
        if (!Finite(v->value) || !Finite(v->minimum) || !Finite(v->maximum))
            return Fail(error, where + ": non-finite value, min or max");
        const Vec3f value = ToStd(v->value), lo = ToStd(v->minimum),
                    hi = ToStd(v->maximum);
        return evaluator->SetInput(node.value, &value, error) &&
               evaluator->SetInput(node.min, &lo, error) &&
               evaluator->SetInput(node.max, &hi, error) &&
               evaluator->SetInput(node.weight, &envelope, error);
    }
    const auto &m = std::get<RigExecMatrixPropertyRecord>(descriptor.record);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (!std::isfinite(m.value[r][c]))
                return Fail(error, where + ": non-finite value");
    const Mat4d value = ToStd(m.value);
    return evaluator->SetInput(node.value, &value, error) &&
           evaluator->SetInput(node.weight, &envelope, error);
}

} // namespace rigExec
