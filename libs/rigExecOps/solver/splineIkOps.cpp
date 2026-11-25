#include "rigExecOps/solver/splineIkOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/splineIkKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <array>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SplineIk)

RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRestJoints)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRestRoot)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRestMid)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRestEnd)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRoot)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkMid)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkEnd)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkVolumeWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkPreserveVolume)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkMidFollow)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkRoll)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkTwist)
RIGEXEC_DEFINE_PORT_HANDLE(SplineIkOut)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

/// SplineIk's declared fields, in saved order.
enum SplineField : uint32_t {
    kRestLengthChain,
    kMinLengthRatio,
    kAimRootTangent
};

struct Rest {
    std::array<Vec3d, 4> cvs{};
    PointFrame rootControl, midControl, endControl;
    std::vector<PointFrame> joints;
    std::vector<double> segmentLengths;
    double restArcLength = 0.0;
    std::vector<double> volumeWeights;
};

struct Controls {
    PointFrame root, mid, end;
};

struct Params {
    double preserveVolume = 1.0;
    double midFollowWeight = 0.5;
    double roll = 0.0;
    double twist = 0.0;
    double minLengthRatio = 0.0;
    bool aimRootTangent = false;
};

struct Joint {
    PointFrame frame;
    Vec3d scale{1.0, 1.0, 1.0};
    double arcDistance = 0.0;
    double arcParam = 0.0;
    double twist = 0.0;
};

struct Result {
    std::array<Vec3d, 4> cvs{};
    double arcLength = 0.0;
    double ratio = 1.0;
    double roll = 0.0;
    double twist = 0.0;
    std::vector<Joint> joints;
};

template <class T>
std::vector<T>
ToVector(const RigExecArray<T> *a)
{
    return a ? std::vector<T>(a->begin(), a->end()) : std::vector<T>();
}

bool
SplineIkOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    SplineIkPayload p;
    p.restLengthChain = FlagAt(batch, kRestLengthChain, k);
    p.minLengthRatio = ValueAt<double>(batch, kMinLengthRatio, k);
    p.aimRootTangent = FlagAt(batch, kAimRootTangent, k);
    const auto *restJoints =
        In<RigExecArray<PointFrame>>(io, SplineIkRestJoints());
    const std::vector<double> volume =
        ToVector(In<RigExecArray<double>>(io, SplineIkVolumeWeights()));
    const size_t n = restJoints->Size();
    if (n == 0)
        return Fail(error, "SplineIk", "needs at least one joint");
    if (!volume.empty() && volume.size() != n)
        return Fail(error, "SplineIk", "needs one volume weight per joint");
    Rest rest;
    rigExec::RigExecSplineIkMakeRestKernel<Vec3d>(
        ToVector(restJoints), *In<PointFrame>(io, SplineIkRestRoot()),
        *In<PointFrame>(io, SplineIkRestMid()),
        *In<PointFrame>(io, SplineIkRestEnd()), volume, p.restLengthChain,
        &rest);
    const Controls controls = {*In<PointFrame>(io, SplineIkRoot()),
                               *In<PointFrame>(io, SplineIkMid()),
                               *In<PointFrame>(io, SplineIkEnd())};
    Params params;
    params.preserveVolume = InOr(io, SplineIkPreserveVolume(), 1.0);
    params.midFollowWeight = InOr(io, SplineIkMidFollow(), 0.5);
    params.roll = InOr(io, SplineIkRoll(), 0.0);
    params.twist = InOr(io, SplineIkTwist(), 0.0);
    params.minLengthRatio = p.minLengthRatio;
    params.aimRootTangent = p.aimRootTangent;
    Result result;
    // A false solve still lays out every joint, flagged degenerate.
    rigExec::RigExecSolveSplineIkKernel<Vec3f, Vec3d, PointFrame>(
        rest, controls, params, &result);
    PointFrameArray &out = Out<PointFrameArray>(io, SplineIkOut());
    out.frames.Assign(n);
    out.rests.Assign(n);
    for (size_t i = 0; i < n; ++i) {
        out.frames[i] = result.joints[i].frame;
        out.rests[i] = (*restJoints)[i].points;
    }
    return true;
}

bool
CheckSplineIk(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t joints = CheckElements(check, SplineIkRestJoints());
    if (joints == 0)
        return Fail(error, "SplineIk", "needs at least one joint");
    const size_t volume = CheckElements(check, SplineIkVolumeWeights());
    if (Fed(check, SplineIkVolumeWeights()) && volume != 0 &&
        !SameCount(volume, joints))
        return Fail(error, "SplineIk", "needs one volume weight per joint");
    return true;
}

bool
RunSplineIk(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return SplineIkOne(batch, k, error); });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SplineIkRestJoints(), &rigExecRig::PointFrameList(), kRigExecPortInput,
     "restJoints"},
    {&SplineIkRestRoot(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "restRoot"},
    {&SplineIkRestMid(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "restMid"},
    {&SplineIkRestEnd(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "restEnd"},
    {&SplineIkRoot(), &rigExecRig::PointFrameType(), kRigExecPortInput, "root"},
    {&SplineIkMid(), &rigExecRig::PointFrameType(), kRigExecPortInput, "mid"},
    {&SplineIkEnd(), &rigExecRig::PointFrameType(), kRigExecPortInput, "end"},
    {&SplineIkVolumeWeights(), &rigExecStd::DoubleArray(), kOptional,
     "volumeWeights"},
    {&SplineIkPreserveVolume(), &rigExecStd::DoubleType(), kOptional,
     "preserveVolume"},
    {&SplineIkMidFollow(), &rigExecStd::DoubleType(), kOptional,
     "midFollowWeight"},
    {&SplineIkRoll(), &rigExecStd::DoubleType(), kOptional, "roll"},
    {&SplineIkTwist(), &rigExecStd::DoubleType(), kOptional, "twist"},
    {&SplineIkOut(), &rigExecRig::PointFrameArrayType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &SplineIkRestJoints()},
};

void
SplineValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SplineIkPayload *>(payload);
    SetFlag(out, kRestLengthChain, p.restLengthChain);
    out.Set(kMinLengthRatio, p.minLengthRatio);
    SetFlag(out, kAimRootTangent, p.aimRootTangent);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Flag("restLengthChain"),
    Value("minLengthRatio", rigExecStd::DoubleType()),
    Flag("aimRootTangent"),
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SplineIk";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunSplineIk;
    d.validate = &CheckSplineIk;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &SplineValues;
    return d;
}();

} // namespace

void
RegisterSplineIkOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SplineIk(), kDesc);
}

} // namespace rigExecOps
