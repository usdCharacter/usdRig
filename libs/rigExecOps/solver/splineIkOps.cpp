#include "rigExecOps/solver/splineIkOps.h"

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

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;

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
RunSplineIk(const void *payload, void *, const RigExecOpIo &io,
            std::string *error)
{
    const auto &p = *static_cast<const SplineIkPayload *>(payload);
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
    out.frames = RigExecArray<PointFrame>(n);
    out.rests = RigExecArray<PointLandmarks>(n);
    for (size_t i = 0; i < n; ++i) {
        out.frames[i] = result.joints[i].frame;
        out.rests[i] = (*restJoints)[i].points;
    }
    return true;
}

} // namespace

void
RegisterSplineIkOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &frame = rigExecRig::PointFrameType();
    const auto &d = rigExecStd::DoubleType();

    static const rigExec::RigExecPortDesc ports[] = {
        {&SplineIkRestJoints(), &rigExecRig::PointFrameList(),
         kRigExecPortInput, "restJoints"},
        {&SplineIkRestRoot(), &frame, kRigExecPortInput, "restRoot"},
        {&SplineIkRestMid(), &frame, kRigExecPortInput, "restMid"},
        {&SplineIkRestEnd(), &frame, kRigExecPortInput, "restEnd"},
        {&SplineIkRoot(), &frame, kRigExecPortInput, "root"},
        {&SplineIkMid(), &frame, kRigExecPortInput, "mid"},
        {&SplineIkEnd(), &frame, kRigExecPortInput, "end"},
        {&SplineIkVolumeWeights(), &rigExecStd::DoubleArray(), kOptional,
         "volumeWeights"},
        {&SplineIkPreserveVolume(), &d, kOptional, "preserveVolume"},
        {&SplineIkMidFollow(), &d, kOptional, "midFollowWeight"},
        {&SplineIkRoll(), &d, kOptional, "roll"},
        {&SplineIkTwist(), &d, kOptional, "twist"},
        {&SplineIkOut(), &rigExecRig::PointFrameArrayType(), kRigExecPortOutput,
         "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "SplineIk";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunSplineIk;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<SplineIkPayload>();
        return d;
    }();
    registrar.Add(SplineIk(), desc);
}

} // namespace rigExecOps
