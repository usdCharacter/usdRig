#include "rigExecOps/constraint/constraintOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/constraintKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(PositionConstraint)
RIGEXEC_DEFINE_OP_HANDLE(RotationConstraint)
RIGEXEC_DEFINE_OP_HANDLE(ScaleConstraint)
RIGEXEC_DEFINE_OP_HANDLE(ParentConstraint)
RIGEXEC_DEFINE_OP_HANDLE(AimConstraint)

RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSpace)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSpace)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintTarget)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintWorldUp)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;

struct Source {
    PointFrame frame;
    double normalizedWeight = 1.0;
    Vec3d translationOffset{0.0, 0.0, 0.0};
    Vec3d rotationOffsetDegrees{0.0, 0.0, 0.0};
};

struct PositionParams {
    Vec3d offset;
    ConstraintAxes affect;
    double weight;
};

struct RotationParams {
    Vec3d offsetDegrees;
    ConstraintAxes affect;
    ConstraintEulerOrder rotationOrder;
    double weight;
    const Mat4d *carry;
};

struct ScaleParams {
    Vec3d offset;
    ConstraintAxes affect;
    double weight;
    bool blendShear;
};

struct ParentParams {
    ConstraintAxes translationAxes;
    ConstraintAxes rotationAxes;
    ConstraintAxes scaleAxes;
    ConstraintEulerOrder rotationOrder;
    double weight;
    bool blendShear;
    const Mat4d *carry;
};

struct AimParams {
    Vec3d localAimVector;
    Vec3d localUpVector;
    const Vec3d *worldUpDirection;
    bool preserveInputUp;
    Vec3d rotationOffsetDegrees;
    ConstraintAxes affectRotation;
    ConstraintEulerOrder rotationOrder;
    double weight;
};

/// The sources with their weights and Parent offsets, or false with
/// \p error set when a parallel array has the wrong length.
bool
GatherSources(const char *op, const ConstraintPayload &p, const RigExecOpIo &io,
              const RigExecPortHandle &sourcesPort,
              const RigExecPortHandle &weightsPort, bool withOffsets,
              std::vector<Source> *sources, std::string *error)
{
    const auto &frames = In<PointFrameArray>(io, sourcesPort)->frames;
    const auto *weights = In<RigExecArray<double>>(io, weightsPort);
    const size_t n = frames.Size();
    if (weights && weights->Size() != n)
        return Fail(error, op, "needs one source weight per source");
    if (withOffsets &&
        ((!p.translationOffsets.empty() && p.translationOffsets.size() != n) ||
         (!p.rotationOffsetsDegrees.empty() &&
          p.rotationOffsetsDegrees.size() != n)))
        return Fail(error, op, "needs no offsets or one per source");
    sources->resize(n);
    for (size_t i = 0; i < n; ++i) {
        Source &s = (*sources)[i];
        s.frame = frames[i];
        s.normalizedWeight = weights ? (*weights)[i] : 1.0;
        if (withOffsets && !p.translationOffsets.empty())
            s.translationOffset = p.translationOffsets[i];
        if (withOffsets && !p.rotationOffsetsDegrees.empty())
            s.rotationOffsetDegrees = p.rotationOffsetsDegrees[i];
    }
    return true;
}

/// The solved frame, or the input when the constraint could not solve it.
void
Commit(const PointFrame &in, const PointFrame &solved, PointFrame *out)
{
    *out = rigExec::constraintDetail::IsUsableFrame(solved) ? solved : in;
}

bool
RunPosition(const void *payload, void *, const RigExecOpIo &io,
            std::string *error)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    std::vector<Source> sources;
    if (!GatherSources("PositionConstraint", p, io, PositionConstraintSources(),
                       PositionConstraintSourceWeights(), false, &sources,
                       error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, PositionConstraintIn());
    const PositionParams params{p.offset, p.affect,
                                *In<double>(io, PositionConstraintWeight())};
    Commit(in,
           rigExec::RigExecApplyPositionConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, PositionConstraintOut()));
    return true;
}

bool
RunRotation(const void *payload, void *, const RigExecOpIo &io,
            std::string *error)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    std::vector<Source> sources;
    if (!GatherSources("RotationConstraint", p, io, RotationConstraintSources(),
                       RotationConstraintSourceWeights(), false, &sources,
                       error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, RotationConstraintIn());
    const RotationParams params{p.offset, p.affect, p.rotationOrder,
                                *In<double>(io, RotationConstraintWeight()),
                                In<Mat4d>(io, RotationConstraintSpace())};
    Commit(in,
           rigExec::RigExecApplyRotationConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, RotationConstraintOut()));
    return true;
}

bool
RunScale(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    std::vector<Source> sources;
    if (!GatherSources("ScaleConstraint", p, io, ScaleConstraintSources(),
                       ScaleConstraintSourceWeights(), false, &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, ScaleConstraintIn());
    const ScaleParams params{p.offset, p.affect,
                             *In<double>(io, ScaleConstraintWeight()),
                             p.blendShear};
    Commit(in,
           rigExec::RigExecApplyScaleConstraintKernel<Vec3f, Vec3d>(in, sources,
                                                                    params),
           &Out<PointFrame>(io, ScaleConstraintOut()));
    return true;
}

bool
RunParent(const void *payload, void *, const RigExecOpIo &io,
          std::string *error)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    std::vector<Source> sources;
    if (!GatherSources("ParentConstraint", p, io, ParentConstraintSources(),
                       ParentConstraintSourceWeights(), true, &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, ParentConstraintIn());
    const ParentParams params{p.translationAxes,
                              p.rotationAxes,
                              p.scaleAxes,
                              p.rotationOrder,
                              *In<double>(io, ParentConstraintWeight()),
                              p.blendShear,
                              In<Mat4d>(io, ParentConstraintSpace())};
    Commit(in,
           rigExec::RigExecApplyParentConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, ParentConstraintOut()));
    return true;
}

bool
RunAim(const void *payload, void *, const RigExecOpIo &io, std::string *)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    const PointFrame &in = *In<PointFrame>(io, AimConstraintIn());
    const AimParams params{p.localAimVector,
                           p.localUpVector,
                           In<Vec3d>(io, AimConstraintWorldUp()),
                           p.preserveInputUp,
                           p.offset,
                           p.affect,
                           p.rotationOrder,
                           *In<double>(io, AimConstraintWeight())};
    Commit(in,
           rigExec::RigExecApplyAimConstraintKernel<Vec3f, Vec3d>(
               in, *In<Vec3d>(io, AimConstraintTarget()), params),
           &Out<PointFrame>(io, AimConstraintOut()));
    return true;
}

rigExec::RigExecOpDesc
ConstraintDesc(const char *name, const rigExec::RigExecPortDesc *ports,
               size_t portCount, rigExec::RigExecOpRunFn run)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.run = run;
    d.capabilities = rigExec::kRigExecOpBakeable;
    d.payload = rigExec::RigExecPayloadHandlingFor<ConstraintPayload>();
    return d;
}

} // namespace

void
RegisterConstraintOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &frame = rigExecRig::PointFrameType();
    const auto &frames = rigExecRig::PointFrameArrayType();
    const auto &weights = rigExecStd::DoubleArray();
    const auto &d = rigExecStd::DoubleType();
    const auto &m = rigExecStd::Mat4dType();
    const auto &v = rigExecStd::Vec3dType();

    static const rigExec::RigExecPortDesc positionPorts[] = {
        {&PositionConstraintIn(), &frame, kRigExecPortInput, "in"},
        {&PositionConstraintSources(), &frames, kRigExecPortInput, "sources"},
        {&PositionConstraintSourceWeights(), &weights, kOptional,
         "sourceWeights"},
        {&PositionConstraintWeight(), &d, kRigExecPortInput, "weight"},
        {&PositionConstraintOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc rotationPorts[] = {
        {&RotationConstraintIn(), &frame, kRigExecPortInput, "in"},
        {&RotationConstraintSources(), &frames, kRigExecPortInput, "sources"},
        {&RotationConstraintSourceWeights(), &weights, kOptional,
         "sourceWeights"},
        {&RotationConstraintWeight(), &d, kRigExecPortInput, "weight"},
        {&RotationConstraintSpace(), &m, kOptional, "space"},
        {&RotationConstraintOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc scalePorts[] = {
        {&ScaleConstraintIn(), &frame, kRigExecPortInput, "in"},
        {&ScaleConstraintSources(), &frames, kRigExecPortInput, "sources"},
        {&ScaleConstraintSourceWeights(), &weights, kOptional, "sourceWeights"},
        {&ScaleConstraintWeight(), &d, kRigExecPortInput, "weight"},
        {&ScaleConstraintOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc parentPorts[] = {
        {&ParentConstraintIn(), &frame, kRigExecPortInput, "in"},
        {&ParentConstraintSources(), &frames, kRigExecPortInput, "sources"},
        {&ParentConstraintSourceWeights(), &weights, kOptional,
         "sourceWeights"},
        {&ParentConstraintWeight(), &d, kRigExecPortInput, "weight"},
        {&ParentConstraintSpace(), &m, kOptional, "space"},
        {&ParentConstraintOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc aimPorts[] = {
        {&AimConstraintIn(), &frame, kRigExecPortInput, "in"},
        {&AimConstraintTarget(), &v, kRigExecPortInput, "target"},
        {&AimConstraintWorldUp(), &v, kOptional, "worldUp"},
        {&AimConstraintWeight(), &d, kRigExecPortInput, "weight"},
        {&AimConstraintOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc positionDesc =
        ConstraintDesc("PositionConstraint", positionPorts,
                       std::size(positionPorts), &RunPosition);
    static const rigExec::RigExecOpDesc rotationDesc =
        ConstraintDesc("RotationConstraint", rotationPorts,
                       std::size(rotationPorts), &RunRotation);
    static const rigExec::RigExecOpDesc scaleDesc = ConstraintDesc(
        "ScaleConstraint", scalePorts, std::size(scalePorts), &RunScale);
    static const rigExec::RigExecOpDesc parentDesc = ConstraintDesc(
        "ParentConstraint", parentPorts, std::size(parentPorts), &RunParent);
    static const rigExec::RigExecOpDesc aimDesc =
        ConstraintDesc("AimConstraint", aimPorts, std::size(aimPorts), &RunAim);
    registrar.Add(PositionConstraint(), positionDesc);
    registrar.Add(RotationConstraint(), rotationDesc);
    registrar.Add(ScaleConstraint(), scaleDesc);
    registrar.Add(ParentConstraint(), parentDesc);
    registrar.Add(AimConstraint(), aimDesc);
}

} // namespace rigExecOps
