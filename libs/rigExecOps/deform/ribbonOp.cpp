#include "rigExecOps/deform/ribbonOp.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/ribbonKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Ribbon)
RIGEXEC_DEFINE_OP_HANDLE(GuidePoints)

RIGEXEC_DEFINE_PORT_HANDLE(RibbonPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonBindCoords)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonFrames)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonOut)

RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsPoints)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsFrames)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

bool
RibbonOne(const RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, RibbonPoints());
    const auto &bind = *In<RigExecArray<Vec2f>>(io, RibbonBindCoords());
    const auto &frames = *In<rigExecRig::PointFrameArray>(io, RibbonFrames());
    if (frames.rests.Size() != frames.frames.Size())
        return Fail(error, "Ribbon", "rests are not parallel to frames");
    Points &out = Out<Points>(io, RibbonOut());
    out = in;
    if (!rigExec::RigExecApplyRibbonKernel<
            rigExec::RigExecDualQuatMath<Vec3f, Vec3d>>(
            out.Data(), out.Size(), bind.Data(), bind.Size(),
            frames.frames.Size(),
            [&](size_t k) -> const auto & { return frames.rests[k]; },
            [&](size_t k) -> const auto & { return frames.frames[k].points; }))
        return Fail(error, "Ribbon",
                    "needs one bind per point and two frame samples");
    return BlendEnvelope(in, In<Envelope>(io, RibbonEnvelope()), &out, "Ribbon",
                         error);
}

bool
GuidePointsOne(const RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, GuidePointsPoints());
    const auto &frames =
        *In<rigExecRig::PointFrameArray>(io, GuidePointsFrames());
    if (frames.frames.Size() != in.Size())
        return Fail(error, "GuidePoints", "needs one frame per point");
    Points &out = Out<Points>(io, GuidePointsOut());
    out.Assign(in.Size());
    for (size_t i = 0; i < in.Size(); ++i) {
        const Vec3d &o = frames.frames[i].points[0];
        out[i] = Vec3f(float(o[0]), float(o[1]), float(o[2]));
    }
    return BlendEnvelope(in, In<Envelope>(io, GuidePointsEnvelope()), &out,
                         "GuidePoints", error);
}

bool
RunRibbon(const rigExec::RigExecOpBatch &batch, size_t *failed,
          std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return RibbonOne(rigExec::RigExecBatchInstance(batch, k), error);
    });
}

bool
RunGuidePoints(const rigExec::RigExecOpBatch &batch, size_t *failed,
               std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return GuidePointsOne(rigExec::RigExecBatchInstance(batch, k), error);
    });
}

rigExec::RigExecOpDesc
FrameDesc(const char *name, const rigExec::RigExecPortDesc *ports,
          size_t portCount, rigExec::RigExecOpBatchFn runBatch,
          uint32_t capabilities)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpFromLeaves | capabilities;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&RibbonPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&RibbonBindCoords(), &rigExecStd::Vec2fArray(), kRigExecPortInput,
     "bindCoords"},
    {&RibbonFrames(), &rigExecRig::PointFrameArrayType(), kRigExecPortInput,
     "frames"},
    {&RibbonEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&RibbonOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &RibbonPoints()},
};

const rigExec::RigExecPortDesc kGuidePorts[] = {
    {&GuidePointsPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&GuidePointsFrames(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "frames"},
    {&GuidePointsEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&GuidePointsOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &GuidePointsPoints()},
};

const rigExec::RigExecOpDesc kRibbonDesc =
    FrameDesc("Ribbon", kPorts, std::size(kPorts), &RunRibbon,
              rigExec::kRigExecOpSeparateBlend);

const rigExec::RigExecOpDesc kGuideDesc =
    FrameDesc("GuidePoints", kGuidePorts, std::size(kGuidePorts),
              &RunGuidePoints, rigExec::kRigExecOpSeparateBlend);

} // namespace

void
RegisterRibbonOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Ribbon(), kRibbonDesc);
    registrar.Add(GuidePoints(), kGuideDesc);
}

} // namespace rigExecOps
