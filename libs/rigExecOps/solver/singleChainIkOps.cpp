#include "rigExecOps/solver/singleChainIkOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/singleChainIkKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecRuntime/poseMath.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SingleChainIk)

RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkChain)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkEffector)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkPole)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkTwist)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkWeight)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkStretch)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkOut)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkRests)
RIGEXEC_DEFINE_PORT_HANDLE(SingleChainIkStatus)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

using Math = rigExec::RrPoseMathT<PointFrame>;

/// SingleChainIk's declared fields, in saved order.
enum IkField : uint32_t { kSingleChain, kPreserveJointOrientation };

/// What the solve reads and works in, kept so a steady frame allocates
/// nothing.
struct SingleChainIkState {
    std::vector<PointFrame> current, solved;
    rigExec::RigExecSingleChainIkWorkspace<Vec3f, Vec3d> workspace;
};

bool
SingleChainIkOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    SingleChainIkPayload p;
    p.singleChain = FlagAt(batch, kSingleChain, k);
    p.preserveJointOrientation = FlagAt(batch, kPreserveJointOrientation, k);
    const PointFrameArray &chain =
        *In<PointFrameArray>(io, SingleChainIkChain());
    const PointFrame &effector = *In<PointFrame>(io, SingleChainIkEffector());
    rigExec::RigExecSingleChainIkKernelParams<Vec3d> params;
    params.mode = p.singleChain ? 1 : 0;
    params.pole = InOr(io, SingleChainIkPole(), Vec3d(0.0, 1.0, 0.0));
    params.twistDegrees = InOr(io, SingleChainIkTwist(), 0.0);
    params.weight = InOr(io, SingleChainIkWeight(), 1.0);
    params.stretch = InOr(io, SingleChainIkStretch(), 0.0);
    params.preserveJointOrientation = p.preserveJointOrientation;
    PointFrameArray &out = Out<PointFrameArray>(io, SingleChainIkOut());
    uint32_t &status = Out<uint32_t>(io, SingleChainIkStatus());
    const auto passThrough = [&](SingleChainIkResult result) {
        out.frames.Assign(chain.frames.Data(), chain.frames.Size());
        out.rests = chain.rests;
        status = uint32_t(result);
        return true;
    };
    if (std::isfinite(params.weight) && params.weight <= 0.0)
        return passThrough(SingleChainIkResult::Passed);
    if (!(effector.flags & rigExecRig::kPointFrameValid))
        return passThrough(SingleChainIkResult::Unsolved);
    SingleChainIkState &state = *static_cast<SingleChainIkState *>(
        rigExec::RigExecBatchState(batch, k));
    std::vector<PointFrame> &current = state.current;
    current.assign(chain.frames.begin(), chain.frames.end());
    if (const PointFrameArray *rests =
            In<PointFrameArray>(io, SingleChainIkRests())) {
        if (rests->frames.Size() != chain.frames.Size())
            return Fail(error, "SingleChainIk",
                        "has " + std::to_string(rests->frames.Size()) +
                            " rests for " +
                            std::to_string(chain.frames.Size()) + " joints");
        if (!rigExec::eval::PrepareRestDerivedIkChain<Math>(
                chain.frames.Data(), rests->frames.Data(), chain.frames.Size(),
                current.data()))
            return passThrough(SingleChainIkResult::Unprepared);
    }
    std::vector<PointFrame> &solved = state.solved;
    rigExec::RigExecSolveSingleChainIkInto<Vec3f, Vec3d>(
        current, effector, params, state.workspace, &solved);
    out.frames.Assign(solved.data(), solved.size());
    out.rests = chain.rests;
    bool usable = solved.size() == chain.frames.Size();
    for (const PointFrame &frame : solved)
        usable = usable &&
                 rigExec::eval::FrameUsable<Math>(frame.points, frame.flags);
    status = uint32_t(usable ? SingleChainIkResult::Solved
                             : SingleChainIkResult::Unsolved);
    return true;
}

bool
RunSingleChainIk(const RigExecOpBatch &batch, size_t *failed,
                 std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SingleChainIkOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SingleChainIkChain(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "chain"},
    {&SingleChainIkEffector(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "effector"},
    {&SingleChainIkPole(), &rigExecStd::Vec3dType(), kOptional, "pole"},
    {&SingleChainIkTwist(), &rigExecStd::DoubleType(), kOptional,
     "twistDegrees"},
    {&SingleChainIkWeight(), &rigExecStd::DoubleType(), kOptional, "weight"},
    {&SingleChainIkStretch(), &rigExecStd::DoubleType(), kOptional, "stretch"},
    {&SingleChainIkOut(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortOutput, "out", rigExec::RigExecOutputBound::Input,
     &SingleChainIkChain()},
    {&SingleChainIkRests(), &rigExecRig::PointFrameArrayType(), kOptional,
     "rests"},
    {&SingleChainIkStatus(), &rigExecStd::UIntType(), kRigExecPortOutput,
     "status"},
};

void
IkValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SingleChainIkPayload *>(payload);
    SetFlag(out, kSingleChain, p.singleChain);
    SetFlag(out, kPreserveJointOrientation, p.preserveJointOrientation);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Flag("singleChain"),
    Flag("preserveJointOrientation"),
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SingleChainIk";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunSingleChainIk;
    d.state = opIo::StateOf<SingleChainIkState>();
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &IkValues;
    return d;
}();

} // namespace

void
RegisterSingleChainIkOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SingleChainIk(), kDesc);
}

} // namespace rigExecOps
