#include "rigExecOps/solver/chainSolverOps.h"

#include "rigExecOps/fieldIo.h"
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

using rigExec::RigExecOpBatch;
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
using namespace fieldIo;

/// FkChain's declared fields, in saved order.
enum FkField : uint32_t { kParentCount, kParents, kScaleSegments };
/// TwoBoneIk's.
enum IkField : uint32_t { kSoftDistancePolicy, kIkScaleSegments, kCalibration };

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
CheckFkChain(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const auto parents = ListAt<int32_t>(check.batch, kParents, check.k);
    const size_t n = parents.Size();
    for (size_t i = 0; i < n; ++i)
        if (parents[i] >= int(i) || parents[i] < -1)
            return Fail(error, "FkChain", "parents must come first");
    if (const PointFrameArray *controls =
            CheckValue<PointFrameArray>(check, FkChainControls()))
        if (controls->frames.Size() != n || controls->rests.Size() != n)
            return Fail(error, "FkChain",
                        "needs one rest and one parent index per control");
    if (Fed(check, FkChainOutRests()) &&
        !SameCount(CheckElements(check, FkChainOutRests()), n))
        return Fail(error, "FkChain", "needs one output rest per control");
    return true;
}

bool
CheckTwoBoneIk(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, TwoBoneIkRests()), 3))
        return Fail(error, "TwoBoneIk", "needs three joint rests");
    return true;
}

bool
FkChainOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto parents = ListAt<int32_t>(batch, kParents, k);
    const PointFrameArray &controls =
        *In<PointFrameArray>(io, FkChainControls());
    const Landmarks *outRests = In<Landmarks>(io, FkChainOutRests());
    const size_t n = controls.frames.Size();
    if (controls.rests.Size() != n || parents.Size() != n)
        return Fail(error, "FkChain",
                    "needs one rest and one parent index per control");
    if (outRests && outRests->Size() != n)
        return Fail(error, "FkChain", "needs one output rest per control");
    std::vector<Element> elements(n);
    for (size_t i = 0; i < n; ++i) {
        const int parent = parents[i];
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
    if (FlagAt(batch, kScaleSegments, k))
        rigExec::RigExecScaleFkSegmentsKernel<Vec3d>(elements, &frames);
    PointFrameArray &out = Out<PointFrameArray>(io, FkChainOut());
    out.frames.Assign(frames.data(), frames.size());
    out.rests = outRests ? *outRests : controls.rests;
    return true;
}

bool
RunFkChain(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return FkChainOne(batch, k, error); });
}

bool
TwoBoneIkOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    TwoBoneIkPayload p;
    p.softDistancePolicy = FlagAt(batch, kSoftDistancePolicy, k);
    p.scaleSegments = FlagAt(batch, kIkScaleSegments, k);
    p.scaleCalibration = ValueAt<double>(batch, kCalibration, k);
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
    out.frames.Assign(frames.data(), frames.size());
    out.rests = restList;
    return true;
}

bool
RunTwoBoneIk(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(
        batch, failed, [&](size_t k) { return TwoBoneIkOne(batch, k, error); });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kFkPorts[] = {
    {&FkChainControls(), &rigExecRig::PointFrameArrayType(), kRigExecPortInput,
     "controls"},
    {&FkChainOutRests(), &rigExecRig::PointLandmarksList(), kOptional,
     "outRests"},
    {&FkChainOut(), &rigExecRig::PointFrameArrayType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &FkChainControls()},
};

const rigExec::RigExecPortDesc kIkPorts[] = {
    {&TwoBoneIkRoot(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "root"},
    {&TwoBoneIkEffector(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "effector"},
    {&TwoBoneIkPole(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "pole"},
    {&TwoBoneIkRests(), &rigExecRig::PointLandmarksList(), kRigExecPortInput,
     "rests"},
    {&TwoBoneIkSpace(), &rigExecStd::Mat4dType(), kOptional, "space"},
    {&TwoBoneIkBend(), &rigExecStd::DoubleType(), kOptional,
     "preferredBendRadians"},
    {&TwoBoneIkUpperOffset(), &rigExecStd::DoubleType(), kOptional,
     "upperLengthOffset"},
    {&TwoBoneIkLowerOffset(), &rigExecStd::DoubleType(), kOptional,
     "lowerLengthOffset"},
    {&TwoBoneIkStretch(), &rigExecStd::FloatType(), kOptional, "stretch"},
    {&TwoBoneIkSoftness(), &rigExecStd::FloatType(), kOptional, "softness"},
    {&TwoBoneIkPin(), &rigExecStd::FloatType(), kOptional, "pin"},
    {&TwoBoneIkUpperScale(), &rigExecStd::DoubleType(), kOptional,
     "upperScale"},
    {&TwoBoneIkLowerScale(), &rigExecStd::DoubleType(), kOptional,
     "lowerScale"},
    {&TwoBoneIkSoftDistance(), &rigExecStd::FloatType(), kOptional,
     "softDistance"},
    {&TwoBoneIkTwist(), &rigExecStd::FloatType(), kOptional, "twist"},
    {&TwoBoneIkOut(), &rigExecRig::PointFrameArrayType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Fixed, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 3},
};

void
FkValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const FkChainPayload *>(payload);
    SetList(out, kParentCount, p.parentIndices);
    SetFlag(out, kScaleSegments, p.scaleSegments);
}

void
IkValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const TwoBoneIkPayload *>(payload);
    SetFlag(out, kSoftDistancePolicy, p.softDistancePolicy);
    SetFlag(out, kIkScaleSegments, p.scaleSegments);
    out.Set(kCalibration, p.scaleCalibration);
}

const rigExec::RigExecFieldDesc kFkFields[] = {
    Count("parentCount"),
    Items("parentIndices", rigExecStd::IntType(), kParentCount),
    Flag("scaleSegments"),
};

const rigExec::RigExecFieldDesc kIkFields[] = {
    Flag("softDistancePolicy"),
    Flag("scaleSegments"),
    Value("scaleCalibration", rigExecStd::DoubleType()),
};

const rigExec::RigExecOpDesc kFk = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "FkChain";
    d.ports = kFkPorts;
    d.portCount = std::size(kFkPorts);
    d.runBatch = &RunFkChain;
    d.validate = &CheckFkChain;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFkFields;
    d.payloadFieldCount = std::size(kFkFields);
    d.payloadVersion = 1;
    d.payloadValues = &FkValues;
    return d;
}();

const rigExec::RigExecOpDesc kIk = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "TwoBoneIk";
    d.ports = kIkPorts;
    d.portCount = std::size(kIkPorts);
    d.runBatch = &RunTwoBoneIk;
    d.validate = &CheckTwoBoneIk;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kIkFields;
    d.payloadFieldCount = std::size(kIkFields);
    d.payloadVersion = 1;
    d.payloadValues = &IkValues;
    return d;
}();

} // namespace

void
RegisterChainSolverOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(FkChain(), kFk);
    registrar.Add(TwoBoneIk(), kIk);
}

} // namespace rigExecOps
