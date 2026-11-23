#include "rigExecOps/solver/chainSolverOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/chainSolverKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <array>
#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(FkChain)
RIGEXEC_DEFINE_OP_HANDLE(TwoBoneIk)

RIGEXEC_DEFINE_PORT_HANDLE(FkChainControls)
RIGEXEC_DEFINE_PORT_HANDLE(FkChainOutRests)
RIGEXEC_DEFINE_PORT_HANDLE(FkChainOut)

RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkRoot)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkEffector)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkPole)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkRests)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkSpace)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkBend)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkUpperOffset)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkLowerOffset)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkStretch)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkSoftness)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkPin)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkUpperScale)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkLowerScale)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkSoftDistance)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkTwist)
RIGEXEC_DEFINE_PORT_HANDLE(TwoBoneIkOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Landmarks = RigExecArray<PointLandmarks>;
using namespace opIo;

struct Element {
    PointLandmarks restPoints;
    PointLandmarks posePoints;
    PointLandmarks outRestPoints;
    bool hasOutRest = false;
    int parentIndex = -1;
};

struct IkParams {
    double upperLength = 1.0;
    double lowerLength = 1.0;
    double stretch = 1.0;
    double softness = 0.0;
    double preferredBendRadians = 0.0;
    bool softDistancePolicy = false;
    rigExec::RigExecLimbStretch limb;
    double twistRadians = 0.0;
    bool scaleSegments = false;
    Mat4d space = Mat4d(1.0);
};

bool
RunFkChain(const void *payload, void *, const RigExecOpIo &io,
           std::string *error)
{
    const auto &p = *static_cast<const FkChainPayload *>(payload);
    const PointFrameArray &controls =
        *In<PointFrameArray>(io, FkChainControls());
    const Landmarks *outRests = In<Landmarks>(io, FkChainOutRests());
    const size_t n = controls.frames.Size();
    if (controls.rests.Size() != n || p.parentIndices.size() != n)
        return Fail(error, "FkChain",
                    "needs one rest and one parent index per control");
    if (outRests && outRests->Size() != n)
        return Fail(error, "FkChain", "needs one output rest per control");
    std::vector<Element> elements(n);
    for (size_t i = 0; i < n; ++i) {
        const int parent = p.parentIndices[i];
        if (parent >= int(i) || parent < -1)
            return Fail(error, "FkChain", "parents must come first");
        Element &e = elements[i];
        e.restPoints = controls.rests[i];
        e.posePoints = controls.frames[i].points;
        e.hasOutRest = outRests != nullptr;
        if (outRests)
            e.outRestPoints = (*outRests)[i];
        e.parentIndex = parent;
    }
    std::vector<PointFrame> frames =
        rigExec::RigExecSolveFkChainKernel<Vec3f, Vec3d, PointFrame>(elements);
    if (p.scaleSegments)
        rigExec::RigExecScaleFkSegmentsKernel<Vec3d>(elements, &frames);
    PointFrameArray &out = Out<PointFrameArray>(io, FkChainOut());
    out.frames = RigExecArray<PointFrame>(frames.data(), frames.size());
    out.rests = outRests ? *outRests : controls.rests;
    return true;
}

bool
RunTwoBoneIk(const void *payload, void *, const RigExecOpIo &io,
             std::string *error)
{
    const auto &p = *static_cast<const TwoBoneIkPayload *>(payload);
    const Landmarks &restList = *In<Landmarks>(io, TwoBoneIkRests());
    if (restList.Size() != 3)
        return Fail(error, "TwoBoneIk", "needs three joint rests");
    const std::array<PointLandmarks, 3> rests = {restList[0], restList[1],
                                                 restList[2]};
    IkParams params;
    params.space = InOr(io, TwoBoneIkSpace(), Mat4d(1.0));
    params.preferredBendRadians = InOr(io, TwoBoneIkBend(), 0.0);
    params.stretch = double(InOr(io, TwoBoneIkStretch(), 1.0f));
    params.softness = double(InOr(io, TwoBoneIkSoftness(), 0.0f));
    params.softDistancePolicy = p.softDistancePolicy;
    params.scaleSegments = p.scaleSegments;
    rigExec::RigExecTwoBoneIkLengthsKernel(
        rests, params.space, InOr(io, TwoBoneIkUpperOffset(), 0.0),
        InOr(io, TwoBoneIkLowerOffset(), 0.0), &params.upperLength,
        &params.lowerLength);
    // RigExecSetTwoBoneLimbParams.
    params.limb.stretch = params.stretch;
    params.limb.pin = double(InOr(io, TwoBoneIkPin(), 0.0f));
    params.limb.upperScale = InOr(io, TwoBoneIkUpperScale(), 1.0);
    params.limb.lowerScale = InOr(io, TwoBoneIkLowerScale(), 1.0);
    params.limb.scaleCalibration = p.scaleCalibration;
    params.limb.softDistance =
        double(InOr(io, TwoBoneIkSoftDistance(), 0.0f)) *
        rigExec::RigExecTwoBoneIkSpaceFactorKernel(rests, params.space);
    params.twistRadians =
        double(InOr(io, TwoBoneIkTwist(), 0.0f)) * std::acos(-1.0) / 180.0;
    const auto frames = rigExec::RigExecSolveTwoBoneIkKernel<Vec3f, Vec3d>(
        *In<PointFrame>(io, TwoBoneIkRoot()),
        *In<PointFrame>(io, TwoBoneIkEffector()),
        *In<PointFrame>(io, TwoBoneIkPole()), rests, params);
    PointFrameArray &out = Out<PointFrameArray>(io, TwoBoneIkOut());
    out.frames = RigExecArray<PointFrame>(frames.data(), frames.size());
    out.rests = restList;
    return true;
}

} // namespace

void
RegisterChainSolverOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &frame = rigExecRig::PointFrameType();
    const auto &frames = rigExecRig::PointFrameArrayType();
    const auto &landmarks = rigExecRig::PointLandmarksList();
    const auto &f = rigExecStd::FloatType();
    const auto &d = rigExecStd::DoubleType();

    static const rigExec::RigExecPortDesc fkPorts[] = {
        {&FkChainControls(), &frames, kRigExecPortInput, "controls"},
        {&FkChainOutRests(), &landmarks, kOptional, "outRests"},
        {&FkChainOut(), &frames, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc ikPorts[] = {
        {&TwoBoneIkRoot(), &frame, kRigExecPortInput, "root"},
        {&TwoBoneIkEffector(), &frame, kRigExecPortInput, "effector"},
        {&TwoBoneIkPole(), &frame, kRigExecPortInput, "pole"},
        {&TwoBoneIkRests(), &landmarks, kRigExecPortInput, "rests"},
        {&TwoBoneIkSpace(), &rigExecStd::Mat4dType(), kOptional, "space"},
        {&TwoBoneIkBend(), &d, kOptional, "preferredBendRadians"},
        {&TwoBoneIkUpperOffset(), &d, kOptional, "upperLengthOffset"},
        {&TwoBoneIkLowerOffset(), &d, kOptional, "lowerLengthOffset"},
        {&TwoBoneIkStretch(), &f, kOptional, "stretch"},
        {&TwoBoneIkSoftness(), &f, kOptional, "softness"},
        {&TwoBoneIkPin(), &f, kOptional, "pin"},
        {&TwoBoneIkUpperScale(), &d, kOptional, "upperScale"},
        {&TwoBoneIkLowerScale(), &d, kOptional, "lowerScale"},
        {&TwoBoneIkSoftDistance(), &f, kOptional, "softDistance"},
        {&TwoBoneIkTwist(), &f, kOptional, "twist"},
        {&TwoBoneIkOut(), &frames, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc fk = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "FkChain";
        d.ports = fkPorts;
        d.portCount = std::size(fkPorts);
        d.run = &RunFkChain;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
        d.payload = rigExec::RigExecPayloadHandlingFor<FkChainPayload>();
        return d;
    }();
    static const rigExec::RigExecOpDesc ik = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "TwoBoneIk";
        d.ports = ikPorts;
        d.portCount = std::size(ikPorts);
        d.run = &RunTwoBoneIk;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
        d.payload = rigExec::RigExecPayloadHandlingFor<TwoBoneIkPayload>();
        return d;
    }();
    registrar.Add(FkChain(), fk);
    registrar.Add(TwoBoneIk(), ik);
}

} // namespace rigExecOps
