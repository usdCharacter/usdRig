#include "rigExecScene/poseOps.h"

#include "rigExec/api/types.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/solver/splineIkOps.h"

#include "pxr/base/gf/math.h"

#include <algorithm>
#include <cstring>

namespace rigExec {

namespace {

using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

Vec3d
ToStd(const GfVec3d &v)
{
    return Vec3d(v[0], v[1], v[2]);
}

PointLandmarks
ToStd(const std::array<GfVec3d, 4> &points)
{
    PointLandmarks out;
    for (size_t k = 0; k < 4; ++k)
        out[k] = ToStd(points[k]);
    return out;
}

PointFrame
ToStd(const RigExecPointFrame &frame)
{
    PointFrame out;
    out.points = ToStd(frame.points);
    out.flags = frame.flags;
    return out;
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

std::array<GfVec3d, 4>
FromStd(const PointLandmarks &points)
{
    std::array<GfVec3d, 4> out;
    for (size_t k = 0; k < 4; ++k)
        out[k] = GfVec3d(points[k][0], points[k][1], points[k][2]);
    return out;
}

RigExecPointFrame
FromStd(const PointFrame &frame)
{
    RigExecPointFrame out;
    out.points = FromStd(frame.points);
    out.flags = frame.flags;
    return out;
}

/// \p output's frames and rests from an op's frame array, from \p first on.
void
FromStd(const PointFrameArray &a, size_t first, RigExecPointFrameArray *output)
{
    output->frames.clear();
    output->rests.clear();
    for (size_t k = first; k < a.frames.Size(); ++k)
        output->frames.push_back(FromStd(a.frames[k]));
    for (size_t k = first; k < a.rests.Size(); ++k)
        output->rests.push_back(FromStd(a.rests[k]));
}

/// Bitwise, so a NaN setting matches itself.
template <class T>
bool
Same(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <class T>
bool
SameVector(const std::vector<T> &a, const std::vector<T> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}

bool
SameAxes(const rigExecOps::ConstraintAxes &a,
         const rigExecOps::ConstraintAxes &b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

rigExecOps::ConstraintAxes
Axes(const RigExecConstraintAxisMask &m)
{
    return {m.x, m.y, m.z};
}

rigExecOps::ConstraintEulerOrder
Order(RigExecEulerOrder order)
{
    return rigExecOps::ConstraintEulerOrder(uint8_t(order));
}

void
Landmarks(const std::vector<std::array<GfVec3d, 4>> &in,
          RigExecArray<PointLandmarks> *out)
{
    out->Resize(in.size());
    for (size_t k = 0; k < in.size(); ++k)
        (*out)[k] = ToStd(in[k]);
}

template <class T>
void
Assign(const std::vector<T> &in, RigExecArray<T> *out)
{
    out->Resize(in.size());
    if (!in.empty())
        std::memcpy(out->Data(), in.data(), in.size() * sizeof(T));
}

template <class T>
bool
Set(RigExecEvaluator *e, const RigExecInputId &id, const T &value,
    std::string *error)
{
    return e->SetInput(id, &value, error);
}

// Solvers.

rigExecOps::FkChainPayload
FkPayload(const RigExecSolverRecord &s)
{
    rigExecOps::FkChainPayload p;
    const int base = s.hasStart ? 1 : 0;
    if (base)
        p.parentIndices.push_back(-1);
    for (size_t k = 0; k < s.controlRests.size(); ++k)
        p.parentIndices.push_back(s.parentRelative ? base - 1
                                                   : int(k) + base - 1);
    p.scaleSegments = s.scaleSegments;
    return p;
}

rigExecOps::TwoBoneIkPayload
IkPayload(const RigExecSolverRecord &s)
{
    rigExecOps::TwoBoneIkPayload p;
    p.softDistancePolicy = s.ikParams.softDistancePolicy;
    p.scaleSegments = s.ikParams.scaleSegments;
    p.scaleCalibration = s.ikParams.limb.scaleCalibration;
    return p;
}

rigExecOps::SplineIkPayload
SplinePayload(const RigExecSolverRecord &s, const RigExecSolverInputs &input)
{
    rigExecOps::SplineIkPayload p;
    p.restLengthChain = s.splineRestMode == RigExecSplineIkRestLength::Chain;
    p.minLengthRatio = input.minLengthRatio;
    p.aimRootTangent = s.splineParams.aimRootTangent;
    return p;
}

/// Per blend element, whether a live joint rest is the out rest.
std::vector<char>
BlendLive(const RigExecSolverRecord &s, size_t n)
{
    std::vector<char> live(n, 0);
    for (size_t k = 0; k < n; ++k)
        live[k] = k < s.restIsLive.size() && s.restIsLive[k] &&
                  k < s.jointRests.size();
    return live;
}

/// The live-rest re-basing every twist and ribbon frame goes through.
bool
WriteLive(const RigExecSolverRecord &s, const RigExecSolverOpNode &node,
          RigExecPoseOpScratch *scratch, RigExecEvaluator *e,
          std::string *error)
{
    Landmarks(s.jointRests, &scratch->jointRests);
    scratch->live.Resize(s.restIsLive.size());
    for (size_t k = 0; k < s.restIsLive.size(); ++k)
        scratch->live[k] = s.restIsLive[k] ? 1 : 0;
    return Set(e, node.jointRests, scratch->jointRests, error) &&
           Set(e, node.jointRestLive, scratch->live, error);
}

// Constraints.

rigExecOps::ConstraintPayload
ConstraintPayloadOf(const RigExecConstraintRecord &r,
                    const RigExecConstraintInputs &input)
{
    rigExecOps::ConstraintPayload p;
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        p.offset = ToStd(r.position.offset);
        p.affect = Axes(r.position.affect);
        break;
    case RigExecConstraintKind::Rotation:
        p.offset = ToStd(r.rotation.offsetDegrees);
        p.affect = Axes(r.rotation.affect);
        p.rotationOrder = Order(r.rotation.rotationOrder);
        break;
    case RigExecConstraintKind::Scale:
        p.offset = ToStd(r.scale.offset);
        p.affect = Axes(r.scale.affect);
        p.blendShear = r.scale.blendShear;
        break;
    case RigExecConstraintKind::Parent:
        p.translationAxes = Axes(r.parent.translationAxes);
        p.rotationAxes = Axes(r.parent.rotationAxes);
        p.scaleAxes = Axes(r.parent.scaleAxes);
        p.rotationOrder = Order(r.parent.rotationOrder);
        p.blendShear = r.parent.blendShear;
        for (const RigExecConstraintSource &source : input.sources) {
            p.translationOffsets.push_back(ToStd(source.translationOffset));
            p.rotationOffsetsDegrees.push_back(
                ToStd(source.rotationOffsetDegrees));
        }
        break;
    case RigExecConstraintKind::Aim:
        p.offset = ToStd(r.aim.rotationOffsetDegrees);
        p.affect = Axes(r.aim.affectRotation);
        p.rotationOrder = Order(r.aim.rotationOrder);
        p.localAimVector = ToStd(r.aim.localAimVector);
        p.localUpVector = ToStd(r.aim.localUpVector);
        p.preserveInputUp = r.aim.preserveInputUp;
        break;
    default:
        break;
    }
    return p;
}

bool
SamePayload(const rigExecOps::ConstraintPayload &a,
            const rigExecOps::ConstraintPayload &b)
{
    return Same(a.offset, b.offset) && SameAxes(a.affect, b.affect) &&
           SameAxes(a.translationAxes, b.translationAxes) &&
           SameAxes(a.rotationAxes, b.rotationAxes) &&
           SameAxes(a.scaleAxes, b.scaleAxes) &&
           SameVector(a.translationOffsets, b.translationOffsets) &&
           SameVector(a.rotationOffsetsDegrees, b.rotationOffsetsDegrees) &&
           a.rotationOrder == b.rotationOrder && a.blendShear == b.blendShear &&
           Same(a.localAimVector, b.localAimVector) &&
           Same(a.localUpVector, b.localUpVector) &&
           a.preserveInputUp == b.preserveInputUp;
}

/// Whether \p kind reads the carry, and \p r's world-up presence.
bool
TakesSpace(RigExecConstraintKind kind)
{
    return kind == RigExecConstraintKind::Rotation ||
           kind == RigExecConstraintKind::Parent;
}

double
ConstraintWeight(const RigExecConstraintRecord &r)
{
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        return r.position.weight;
    case RigExecConstraintKind::Rotation:
        return r.rotation.weight;
    case RigExecConstraintKind::Scale:
        return r.scale.weight;
    case RigExecConstraintKind::Parent:
        return r.parent.weight;
    case RigExecConstraintKind::Aim:
        return r.aim.weight;
    default:
        return r.singleChain.weight;
    }
}

} // namespace

bool
RigExecSolverOpSupported(const RigExecSolverRecord &s)
{
    if (s.degenerate)
        return false;
    switch (s.kind) {
    case RigExecSolverKind::FkChain:
        return s.jointGroups.empty();
    case RigExecSolverKind::TwoBoneIk:
    case RigExecSolverKind::TwistDistribution:
    case RigExecSolverKind::Ribbon:
        return true;
    case RigExecSolverKind::BlendPointFrames:
        return !s.blendRotationRejected;
    case RigExecSolverKind::SplineIk:
        return s.splineCount > 0;
    case RigExecSolverKind::Skeleton2d:
        return false;
    }
    return false;
}

RigExecSolverOpAnswer
RigExecAnswerSolverOp(const RigExecSolverRecord &s,
                      const RigExecSolverInputs &input,
                      RigExecPointFrameArray *output, std::string *error)
{
    output->frames.clear();
    output->rests.clear();
    if (s.kind == RigExecSolverKind::FkChain &&
        input.controls.size() != s.controlRests.size()) {
        Fail(error, "FK control/rest cardinality mismatch");
        return RigExecSolverOpAnswer::Failed;
    }
    if (s.kind != RigExecSolverKind::BlendPointFrames)
        return RigExecSolverOpAnswer::Run;
    const auto *a = input.blendA, *b = input.blendB;
    if (!a || !b) {
        if (a || b)
            *output = a ? *a : *b;
        return RigExecSolverOpAnswer::Done;
    }
    const size_t n = a->GetSize();
    if (n == 0 || n != b->GetSize() || a->rests.size() != n)
        return RigExecSolverOpAnswer::Done;
    return RigExecSolverOpAnswer::Run;
}

bool
RigExecAddSolverOp(const RigExecSolverRecord &s,
                   const RigExecSolverInputs &input,
                   RigExecRigDescBuilder *builder, RigExecSolverOpNode *node,
                   std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null solver op destination");
    if (!RigExecSolverOpSupported(s))
        return Fail(error, "solver kind has no op");
    RigExecSolverOpNode r;
    r.kind = s.kind;
    RigExecRigDescBuilder &b = *builder;
    switch (s.kind) {
    case RigExecSolverKind::FkChain: {
        r.fk = FkPayload(s);
        const RigExecNodeId id = b.AddOp(rigExecOps::FkChain(), &r.fk);
        r.controls = b.BindInput(id, rigExecOps::FkChainControls());
        r.outRests = b.BindInput(id, rigExecOps::FkChainOutRests());
        r.out = b.BindOutput(id, rigExecOps::FkChainOut());
        break;
    }
    case RigExecSolverKind::TwoBoneIk: {
        r.ik = IkPayload(s);
        const RigExecNodeId id = b.AddOp(rigExecOps::TwoBoneIk(), &r.ik);
        using namespace rigExecOps;
        r.root = b.BindInput(id, TwoBoneIkRoot());
        r.effector = b.BindInput(id, TwoBoneIkEffector());
        r.pole = b.BindInput(id, TwoBoneIkPole());
        r.rests = b.BindInput(id, TwoBoneIkRests());
        r.space = b.BindInput(id, TwoBoneIkSpace());
        r.bend = b.BindInput(id, TwoBoneIkBend());
        r.upperOffset = b.BindInput(id, TwoBoneIkUpperOffset());
        r.lowerOffset = b.BindInput(id, TwoBoneIkLowerOffset());
        r.stretch = b.BindInput(id, TwoBoneIkStretch());
        r.softness = b.BindInput(id, TwoBoneIkSoftness());
        r.pin = b.BindInput(id, TwoBoneIkPin());
        r.upperScale = b.BindInput(id, TwoBoneIkUpperScale());
        r.lowerScale = b.BindInput(id, TwoBoneIkLowerScale());
        r.softDistance = b.BindInput(id, TwoBoneIkSoftDistance());
        r.twist = b.BindInput(id, TwoBoneIkTwist());
        r.out = b.BindOutput(id, TwoBoneIkOut());
        break;
    }
    case RigExecSolverKind::BlendPointFrames: {
        if (!input.blendA)
            return Fail(error, "frame blend op needs its first input");
        r.logScale = s.scaleMode == RigExecScaleBlend::Log;
        r.blendLive = BlendLive(s, input.blendA->GetSize());
        rigExecOps::BlendFramesPayload payload;
        payload.logScale = r.logScale;
        for (const char live : r.blendLive) {
            using namespace rigExecOps;
            const RigExecNodeId id = b.AddOp(BlendFrames(), &payload);
            RigExecSolverOpNode::Blend e;
            e.a = b.BindInput(id, BlendFramesA());
            e.b = b.BindInput(id, BlendFramesB());
            e.rest = b.BindInput(id, BlendFramesRest());
            e.weight = b.BindInput(id, BlendFramesWeight());
            if (live)
                e.outRest = b.BindInput(id, BlendFramesOutRest());
            e.out = b.BindOutput(id, BlendFramesOut());
            r.blend.push_back(e);
        }
        break;
    }
    case RigExecSolverKind::TwistDistribution: {
        using namespace rigExecOps;
        const RigExecNodeId id = b.AddOp(DistributeTwist());
        r.start = b.BindInput(id, DistributeTwistStart());
        r.end = b.BindInput(id, DistributeTwistEnd());
        r.startRest = b.BindInput(id, DistributeTwistStartRest());
        r.endRest = b.BindInput(id, DistributeTwistEndRest());
        r.weights = b.BindInput(id, DistributeTwistWeights());
        r.turns = b.BindInput(id, DistributeTwistTurns());
        r.jointRests = b.BindInput(id, DistributeTwistJointRests());
        r.jointRestLive = b.BindInput(id, DistributeTwistJointRestLive());
        r.out = b.BindOutput(id, DistributeTwistOut());
        break;
    }
    case RigExecSolverKind::Ribbon: {
        using namespace rigExecOps;
        r.sampleCount = input.ribbonSampleCount;
        SampleRibbonFramesPayload payload;
        payload.sampleCount = r.sampleCount;
        const RigExecNodeId id = b.AddOp(SampleRibbonFrames(), &payload);
        r.posed = b.BindInput(id, SampleRibbonFramesPosed());
        r.restCurve = b.BindInput(id, SampleRibbonFramesRest());
        r.jointRests = b.BindInput(id, SampleRibbonFramesJointRests());
        r.jointRestLive = b.BindInput(id, SampleRibbonFramesJointRestLive());
        r.out = b.BindOutput(id, SampleRibbonFramesOut());
        break;
    }
    case RigExecSolverKind::SplineIk: {
        using namespace rigExecOps;
        r.spline = SplinePayload(s, input);
        const RigExecNodeId id = b.AddOp(SplineIk(), &r.spline);
        r.restJoints = b.BindInput(id, SplineIkRestJoints());
        r.restRoot = b.BindInput(id, SplineIkRestRoot());
        r.restMid = b.BindInput(id, SplineIkRestMid());
        r.restEnd = b.BindInput(id, SplineIkRestEnd());
        r.root = b.BindInput(id, SplineIkRoot());
        r.mid = b.BindInput(id, SplineIkMid());
        r.end = b.BindInput(id, SplineIkEnd());
        r.volumeWeights = b.BindInput(id, SplineIkVolumeWeights());
        r.preserveVolume = b.BindInput(id, SplineIkPreserveVolume());
        r.midFollow = b.BindInput(id, SplineIkMidFollow());
        r.roll = b.BindInput(id, SplineIkRoll());
        r.twist = b.BindInput(id, SplineIkTwist());
        r.out = b.BindOutput(id, SplineIkOut());
        break;
    }
    case RigExecSolverKind::Skeleton2d:
        return Fail(error, "solver kind has no op");
    }
    *node = std::move(r);
    return true;
}

bool
RigExecWriteSolverOpInputs(const RigExecSolverRecord &s,
                           const RigExecSolverInputs &input,
                           const RigExecSolverOpNode &node,
                           RigExecPoseOpScratch *scratch, RigExecEvaluator *e,
                           std::string *error)
{
    if (!e || !scratch || node.kind != s.kind)
        return Fail(error, "solver op node does not match the record");
    switch (s.kind) {
    case RigExecSolverKind::FkChain: {
        const rigExecOps::FkChainPayload payload = FkPayload(s);
        if (!SameVector(payload.parentIndices, node.fk.parentIndices) ||
            payload.scaleSegments != node.fk.scaleSegments)
            return Fail(error, "FK chain settings moved");
        // The start frame leads as a root control; its output is dropped.
        const size_t base = s.hasStart ? 1 : 0, n = input.controls.size();
        const bool jointBasis = s.jointRests.size() == n;
        PointFrameArray &controls = scratch->frames;
        controls.frames.Resize(n + base);
        controls.rests.Resize(n + base);
        scratch->landmarks.Resize(n + base);
        if (base) {
            controls.frames[0] = ToStd(input.start);
            controls.rests[0] = ToStd(s.startRest);
            scratch->landmarks[0] = controls.rests[0];
        }
        for (size_t k = 0; k < n; ++k) {
            controls.frames[k + base] = ToStd(input.controls[k]);
            controls.rests[k + base] = ToStd(s.controlRests[k]);
            const bool live =
                jointBasis && k < s.restIsLive.size() && s.restIsLive[k];
            scratch->landmarks[k + base] =
                live ? ToStd(s.jointRests[k]) : controls.rests[k + base];
        }
        return Set(e, node.controls, controls, error) &&
               Set(e, node.outRests, scratch->landmarks, error);
    }
    case RigExecSolverKind::TwoBoneIk: {
        const rigExecOps::TwoBoneIkPayload payload = IkPayload(s);
        if (!input.refreshIkParams ||
            payload.softDistancePolicy != node.ik.softDistancePolicy ||
            payload.scaleSegments != node.ik.scaleSegments ||
            !Same(payload.scaleCalibration, node.ik.scaleCalibration))
            return Fail(error, "two-bone IK settings moved");
        GfMatrix4d space = input.ikSpace;
        if (input.hasSpace) {
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(input.spaceRest, input.space, &delta))
                space = delta * space;
        }
        scratch->landmarks.Resize(3);
        for (size_t k = 0; k < 3; ++k)
            scratch->landmarks[k] = ToStd(s.ikRests[k]);
        return Set(e, node.root, ToStd(input.root), error) &&
               Set(e, node.effector, ToStd(input.end), error) &&
               Set(e, node.pole, ToStd(input.pole), error) &&
               Set(e, node.rests, scratch->landmarks, error) &&
               Set(e, node.space, ToStd(space), error) &&
               Set(e, node.bend, input.bend, error) &&
               Set(e, node.upperOffset, input.upperOffset, error) &&
               Set(e, node.lowerOffset, input.lowerOffset, error) &&
               Set(e, node.stretch, input.stretch, error) &&
               Set(e, node.softness, input.softness, error) &&
               Set(e, node.pin, input.pin, error) &&
               Set(e, node.upperScale, input.upperScale, error) &&
               Set(e, node.lowerScale, input.lowerScale, error) &&
               Set(e, node.softDistance, input.softDistance, error) &&
               Set(e, node.twist, input.limbTwist, error);
    }
    case RigExecSolverKind::BlendPointFrames: {
        const auto *a = input.blendA, *b = input.blendB;
        if (!a || !b)
            return Fail(error, "frame blend op needs both inputs");
        const size_t n = a->GetSize();
        if (node.logScale != (s.scaleMode == RigExecScaleBlend::Log) ||
            node.blendLive != BlendLive(s, n))
            return Fail(error, "frame blend settings moved");
        const double weight = std::min(std::max(input.blendWeight, 0.0), 1.0);
        for (size_t k = 0; k < n; ++k) {
            const RigExecSolverOpNode::Blend &x = node.blend[k];
            if (!Set(e, x.a, ToStd(a->frames[k]), error) ||
                !Set(e, x.b, ToStd(b->frames[k]), error) ||
                !Set(e, x.rest, ToStd(a->rests[k]), error) ||
                !Set(e, x.weight, weight, error) ||
                (node.blendLive[k] &&
                 !Set(e, x.outRest, ToStd(s.jointRests[k]), error)))
                return false;
        }
        return true;
    }
    case RigExecSolverKind::TwistDistribution: {
        Assign(s.twistWeights, &scratch->doubles);
        return Set(e, node.start, ToStd(input.root), error) &&
               Set(e, node.end, ToStd(input.end), error) &&
               Set(e, node.startRest, ToStd(s.twistStartRest), error) &&
               Set(e, node.endRest, ToStd(s.twistEndRest), error) &&
               Set(e, node.weights, scratch->doubles, error) &&
               Set(e, node.turns, input.twistTurns, error) &&
               WriteLive(s, node, scratch, e, error);
    }
    case RigExecSolverKind::Ribbon: {
        if (input.ribbonSampleCount != node.sampleCount)
            return Fail(error, "ribbon sample count moved");
        const auto points = [](const std::vector<GfVec3f> &v,
                               RigExecArray<Vec3f> *out) {
            out->Resize(v.size());
            for (size_t i = 0; i < v.size(); ++i)
                (*out)[i] = Vec3f(v[i][0], v[i][1], v[i][2]);
        };
        points(input.ribbonPoints, &scratch->posed);
        points(s.ribbonRestPoints, &scratch->rest);
        return Set(e, node.posed, scratch->posed, error) &&
               Set(e, node.restCurve, scratch->rest, error) &&
               WriteLive(s, node, scratch, e, error);
    }
    case RigExecSolverKind::SplineIk: {
        const rigExecOps::SplineIkPayload payload = SplinePayload(s, input);
        if (!input.refreshSplineParams ||
            payload.restLengthChain != node.spline.restLengthChain ||
            !Same(payload.minLengthRatio, node.spline.minLengthRatio) ||
            payload.aimRootTangent != node.spline.aimRootTangent)
            return Fail(error, "spline IK settings moved");
        // The rests carried by the space's rest-to-pose map, as native does.
        GfMatrix4d delta(1.0);
        const bool spaced =
            input.hasSpace &&
            RigExecPointsToMatrix(input.spaceRest, input.space, &delta) &&
            delta != GfMatrix4d(1.0);
        const auto rest = [&](const RigExecPointFrame &frame) {
            return ToStd(spaced ? RigExecTransformFrame(frame, delta) : frame);
        };
        scratch->frameList.Resize(s.splineRestFrames.size());
        for (size_t k = 0; k < s.splineRestFrames.size(); ++k)
            scratch->frameList[k] = rest(s.splineRestFrames[k]);
        Assign(s.splineRestWeights, &scratch->doubles);
        return Set(e, node.restJoints, scratch->frameList, error) &&
               Set(e, node.restRoot, rest(s.splineRootRest), error) &&
               Set(e, node.restMid, rest(s.splineMidRest), error) &&
               Set(e, node.restEnd, rest(s.splineEndRest), error) &&
               Set(e, node.root, ToStd(input.root), error) &&
               Set(e, node.mid, ToStd(input.mid), error) &&
               Set(e, node.end, ToStd(input.end), error) &&
               Set(e, node.volumeWeights, scratch->doubles, error) &&
               Set(e, node.preserveVolume, input.preserveVolume, error) &&
               Set(e, node.midFollow, input.midFollowWeight, error) &&
               Set(e, node.roll, GfDegreesToRadians(input.rollDegrees),
                   error) &&
               Set(e, node.twist, GfDegreesToRadians(input.twistDegrees),
                   error);
    }
    case RigExecSolverKind::Skeleton2d:
        break;
    }
    return Fail(error, "solver kind has no op");
}

bool
RigExecReadSolverOpOutput(const RigExecSolverRecord &s,
                          const RigExecSolverInputs &input,
                          const RigExecSolverOpNode &node,
                          const RigExecEvaluator &e,
                          RigExecPointFrameArray *output)
{
    output->frames.clear();
    output->rests.clear();
    if (s.kind == RigExecSolverKind::BlendPointFrames) {
        const RigExecPointFrameArray &a = *input.blendA;
        for (size_t k = 0; k < node.blend.size(); ++k) {
            const auto *frame =
                static_cast<const PointFrame *>(e.GetOutput(node.blend[k].out));
            if (!frame)
                return false;
            output->frames.push_back(FromStd(*frame));
            output->rests.push_back(node.blendLive[k] ? s.jointRests[k]
                                                      : a.rests[k]);
        }
        return true;
    }
    const auto *out =
        static_cast<const PointFrameArray *>(e.GetOutput(node.out));
    if (!out)
        return false;
    FromStd(*out, s.kind == RigExecSolverKind::FkChain && s.hasStart ? 1 : 0,
            output);
    // Native publishes the unspaced joint rests.
    if (s.kind == RigExecSolverKind::SplineIk)
        output->rests = s.splineJointRests;
    return true;
}

bool
RigExecConstraintOpSupported(RigExecConstraintKind kind)
{
    switch (kind) {
    case RigExecConstraintKind::Position:
    case RigExecConstraintKind::Rotation:
    case RigExecConstraintKind::Scale:
    case RigExecConstraintKind::Parent:
    case RigExecConstraintKind::Aim:
    case RigExecConstraintKind::SingleChainIk:
        return true;
    case RigExecConstraintKind::IkFkMatch:
    case RigExecConstraintKind::Harmonics:
        return false;
    }
    return false;
}

bool
RigExecAddConstraintOp(const RigExecConstraintRecord &r,
                       const RigExecConstraintInputs &input,
                       RigExecRigDescBuilder *builder,
                       RigExecConstraintOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null constraint op destination");
    if (!RigExecConstraintOpSupported(r.kind))
        return Fail(error, "constraint kind has no op");
    RigExecRigDescBuilder &b = *builder;
    RigExecConstraintOpNode n;
    n.kind = r.kind;
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        using namespace rigExecOps;
        SingleChainIkPayload payload;
        payload.singleChain =
            r.singleChain.mode == RigExecSingleChainIkMode::SingleChain;
        payload.preserveJointOrientation =
            r.singleChain.preserveJointOrientation;
        n.singleChain = payload.singleChain;
        n.preserveJointOrientation = payload.preserveJointOrientation;
        const RigExecNodeId id = b.AddOp(SingleChainIk(), &payload);
        n.chain = b.BindInput(id, SingleChainIkChain());
        n.effector = b.BindInput(id, SingleChainIkEffector());
        n.pole = b.BindInput(id, SingleChainIkPole());
        n.twist = b.BindInput(id, SingleChainIkTwist());
        n.weight = b.BindInput(id, SingleChainIkWeight());
        n.stretch = b.BindInput(id, SingleChainIkStretch());
        n.out = b.BindOutput(id, SingleChainIkOut());
        *node = std::move(n);
        return true;
    }
    n.payload = ConstraintPayloadOf(r, input);
    n.hasSpace = TakesSpace(r.kind) && input.carry.has_value();
    n.hasWorldUp = r.kind == RigExecConstraintKind::Aim &&
                   r.aim.worldUpDirection.has_value();
    using namespace rigExecOps;
    const RigExecPortHandle *in = nullptr, *sources = nullptr,
                            *weights = nullptr, *weight = nullptr,
                            *space = nullptr, *out = nullptr;
    const RigExecOpHandle *op = nullptr;
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        op = &PositionConstraint();
        in = &PositionConstraintIn();
        sources = &PositionConstraintSources();
        weights = &PositionConstraintSourceWeights();
        weight = &PositionConstraintWeight();
        out = &PositionConstraintOut();
        break;
    case RigExecConstraintKind::Rotation:
        op = &RotationConstraint();
        in = &RotationConstraintIn();
        sources = &RotationConstraintSources();
        weights = &RotationConstraintSourceWeights();
        weight = &RotationConstraintWeight();
        space = &RotationConstraintSpace();
        out = &RotationConstraintOut();
        break;
    case RigExecConstraintKind::Scale:
        op = &ScaleConstraint();
        in = &ScaleConstraintIn();
        sources = &ScaleConstraintSources();
        weights = &ScaleConstraintSourceWeights();
        weight = &ScaleConstraintWeight();
        out = &ScaleConstraintOut();
        break;
    case RigExecConstraintKind::Parent:
        op = &ParentConstraint();
        in = &ParentConstraintIn();
        sources = &ParentConstraintSources();
        weights = &ParentConstraintSourceWeights();
        weight = &ParentConstraintWeight();
        space = &ParentConstraintSpace();
        out = &ParentConstraintOut();
        break;
    case RigExecConstraintKind::Aim:
        op = &AimConstraint();
        in = &AimConstraintIn();
        weight = &AimConstraintWeight();
        out = &AimConstraintOut();
        break;
    default:
        return Fail(error, "constraint kind has no op");
    }
    const RigExecNodeId id = b.AddOp(*op, &n.payload);
    n.in = b.BindInput(id, *in);
    n.weight = b.BindInput(id, *weight);
    if (sources) {
        n.sources = b.BindInput(id, *sources);
        n.sourceWeights = b.BindInput(id, *weights);
    }
    if (n.hasSpace)
        n.space = b.BindInput(id, *space);
    if (r.kind == RigExecConstraintKind::Aim) {
        n.target = b.BindInput(id, AimConstraintTarget());
        if (n.hasWorldUp)
            n.worldUp = b.BindInput(id, AimConstraintWorldUp());
    }
    n.out = b.BindOutput(id, *out);
    *node = std::move(n);
    return true;
}

bool
RigExecWriteConstraintOpInputs(const RigExecConstraintRecord &r,
                               const RigExecConstraintInputs &input,
                               const RigExecConstraintOpNode &node,
                               RigExecPoseOpScratch *scratch,
                               RigExecEvaluator *e, std::string *error)
{
    if (!e || !scratch || node.kind != r.kind)
        return Fail(error, "constraint op node does not match the record");
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        const RigExecSingleChainIkParams &p = r.singleChain;
        if (node.singleChain !=
                (p.mode == RigExecSingleChainIkMode::SingleChain) ||
            node.preserveJointOrientation != p.preserveJointOrientation)
            return Fail(error, "single-chain IK settings moved");
        PointFrameArray &chain = scratch->frames;
        chain.frames.Resize(input.chain.size());
        chain.rests.Resize(0);
        for (size_t k = 0; k < input.chain.size(); ++k)
            chain.frames[k] = ToStd(input.chain[k]);
        return Set(e, node.chain, chain, error) &&
               Set(e, node.effector, ToStd(input.effector), error) &&
               Set(e, node.pole, ToStd(p.pole), error) &&
               Set(e, node.twist, p.twistDegrees, error) &&
               Set(e, node.weight, p.weight, error) &&
               Set(e, node.stretch, p.stretch, error);
    }
    if (!SamePayload(node.payload, ConstraintPayloadOf(r, input)) ||
        node.hasSpace != (TakesSpace(r.kind) && input.carry.has_value()) ||
        node.hasWorldUp != (r.kind == RigExecConstraintKind::Aim &&
                            r.aim.worldUpDirection.has_value()))
        return Fail(error, "constraint settings moved");
    if (!Set(e, node.in, ToStd(input.incoming), error) ||
        !Set(e, node.weight, ConstraintWeight(r), error))
        return false;
    if (node.hasSpace && !Set(e, node.space, ToStd(*input.carry), error))
        return false;
    if (r.kind == RigExecConstraintKind::Aim)
        return Set(e, node.target, ToStd(input.aimTarget), error) &&
               (!node.hasWorldUp ||
                Set(e, node.worldUp, ToStd(*r.aim.worldUpDirection), error));
    PointFrameArray &sources = scratch->frames;
    sources.frames.Resize(input.sources.size());
    sources.rests.Resize(0);
    scratch->doubles.Resize(input.sources.size());
    for (size_t k = 0; k < input.sources.size(); ++k) {
        sources.frames[k] = ToStd(input.sources[k].frame);
        scratch->doubles[k] = input.sources[k].normalizedWeight;
    }
    return Set(e, node.sources, sources, error) &&
           Set(e, node.sourceWeights, scratch->doubles, error);
}

bool
RigExecReadConstraintOpOutput(const RigExecConstraintRecord &r,
                              const RigExecConstraintInputs &input,
                              const RigExecConstraintOpNode &node,
                              const RigExecEvaluator &e,
                              RigExecConstraintResult *result)
{
    result->chain.clear();
    result->frame = input.incoming;
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        const auto *out =
            static_cast<const PointFrameArray *>(e.GetOutput(node.out));
        if (!out)
            return false;
        for (const PointFrame &frame : out->frames)
            result->chain.push_back(FromStd(frame));
        return true;
    }
    const auto *out = static_cast<const PointFrame *>(e.GetOutput(node.out));
    if (!out)
        return false;
    result->frame = FromStd(*out);
    return true;
}

} // namespace rigExec
