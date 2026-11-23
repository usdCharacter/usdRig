#include "rigExecOps/solver/singleChainIkOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/singleChainIkKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

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

namespace {

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;

bool
RunSingleChainIk(const void *payload, void *, const RigExecOpIo &io,
                 std::string *)
{
    const auto &p = *static_cast<const SingleChainIkPayload *>(payload);
    const PointFrameArray &chain =
        *In<PointFrameArray>(io, SingleChainIkChain());
    rigExec::RigExecSingleChainIkKernelParams<Vec3d> params;
    params.mode = p.singleChain ? 1 : 0;
    params.pole = InOr(io, SingleChainIkPole(), Vec3d(0.0, 1.0, 0.0));
    params.twistDegrees = InOr(io, SingleChainIkTwist(), 0.0);
    params.weight = InOr(io, SingleChainIkWeight(), 1.0);
    params.stretch = InOr(io, SingleChainIkStretch(), 0.0);
    params.preserveJointOrientation = p.preserveJointOrientation;
    const std::vector<PointFrame> current(chain.frames.begin(),
                                          chain.frames.end());
    const std::vector<PointFrame> solved =
        rigExec::RigExecSolveSingleChainIkKernel<Vec3f, Vec3d>(
            current, *In<PointFrame>(io, SingleChainIkEffector()), params);
    PointFrameArray &out = Out<PointFrameArray>(io, SingleChainIkOut());
    out.frames = RigExecArray<PointFrame>(solved.data(), solved.size());
    out.rests = chain.rests;
    return true;
}

} // namespace

void
RegisterSingleChainIkOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &frames = rigExecRig::PointFrameArrayType();
    const auto &d = rigExecStd::DoubleType();

    static const rigExec::RigExecPortDesc ports[] = {
        {&SingleChainIkChain(), &frames, kRigExecPortInput, "chain"},
        {&SingleChainIkEffector(), &rigExecRig::PointFrameType(),
         kRigExecPortInput, "effector"},
        {&SingleChainIkPole(), &rigExecStd::Vec3dType(), kOptional, "pole"},
        {&SingleChainIkTwist(), &d, kOptional, "twistDegrees"},
        {&SingleChainIkWeight(), &d, kOptional, "weight"},
        {&SingleChainIkStretch(), &d, kOptional, "stretch"},
        {&SingleChainIkOut(), &frames, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "SingleChainIk";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunSingleChainIk;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
        d.payload = rigExec::RigExecPayloadHandlingFor<SingleChainIkPayload>();
        return d;
    }();
    registrar.Add(SingleChainIk(), desc);
}

} // namespace rigExecOps
