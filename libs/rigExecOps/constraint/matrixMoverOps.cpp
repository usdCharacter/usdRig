#include "rigExecOps/constraint/matrixMoverOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/poseMath.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(MatrixMover)
RIGEXEC_DEFINE_OP_HANDLE(ConstraintDelta)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverIn)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverTransform)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverTransformRest)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverSpace)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverSpaceRest)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverWeight)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixMoverOut)

RIGEXEC_DEFINE_PORT_HANDLE(ConstraintDeltaSolved)
RIGEXEC_DEFINE_PORT_HANDLE(ConstraintDeltaBase)
RIGEXEC_DEFINE_PORT_HANDLE(ConstraintDeltaBaseOk)
RIGEXEC_DEFINE_PORT_HANDLE(ConstraintDeltaWeight)
RIGEXEC_DEFINE_PORT_HANDLE(ConstraintDeltaDelta)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointLandmarks;
using rigExecStd::Mat4d;
using Math = rigExec::RrPoseMathT<PointFrame>;
namespace eval = rigExec::eval;
using namespace opIo;
using namespace fieldIo;

enum MatrixMoverField : uint32_t { kRadialBlend };

/// A source's rest landmarks: the identity landmarks unfed or not valid.
PointLandmarks
RestOf(const RigExecOpIo &io, const rigExec::RigExecPortHandle &port)
{
    const PointFrame *rest = In<PointFrame>(io, port);
    if (!rest || (rest->flags & eval::pose_ops::kPoseFrameValid) == 0)
        return eval::pose_ops::IdentityLandmarks<Math>();
    return rest->points;
}

bool
CheckMatrixMover(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (Fed(check, MatrixMoverSpaceRest()) && !Fed(check, MatrixMoverSpace()))
        return Fail(error, "MatrixMover", "a space rest needs Space");
    return true;
}

bool
MatrixMoverOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const PointFrame &in = *In<PointFrame>(io, MatrixMoverIn());
    const PointFrame *space = In<PointFrame>(io, MatrixMoverSpace());
    const PointLandmarks transformRest = RestOf(io, MatrixMoverTransformRest());
    const PointLandmarks spaceRest = RestOf(io, MatrixMoverSpaceRest());
    PointFrame &out = Out<PointFrame>(io, MatrixMoverOut());
    out.flags = in.flags;
    switch (eval::MatrixMoverCandidate<Math>(
        in.points, transformRest,
        In<PointFrame>(io, MatrixMoverTransform())->points,
        space ? &spaceRest : nullptr, space ? &space->points : nullptr,
        FlagAt(batch, kRadialBlend, k), *In<double>(io, MatrixMoverWeight()),
        &out.points)) {
    case eval::MatrixMoverFailure::None:
        return true;
    case eval::MatrixMoverFailure::Transform:
        return Fail(error, "MatrixMover",
                    "could not resolve rigExec:transform");
    case eval::MatrixMoverFailure::Space:
        break;
    }
    return Fail(error, "MatrixMover",
                "could not resolve rigExec:transformSpace");
}

bool
RunMatrixMover(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return MatrixMoverOne(batch, k, error);
    });
}

bool
ConstraintDeltaOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    Mat4d &delta = Out<Mat4d>(io, ConstraintDeltaDelta());
    if (*In<double>(io, ConstraintDeltaWeight()) <= 0.0) {
        delta = Math::Identity4();
        return true;
    }
    const PointFrame &solved = *In<PointFrame>(io, ConstraintDeltaSolved());
    if (!eval::MeasureConstraintDelta<Math>(
            solved.points, solved.flags, *In<Mat4d>(io, ConstraintDeltaBase()),
            *In<uint8_t>(io, ConstraintDeltaBaseOk()) != 0, &delta))
        return Fail(error, "ConstraintDelta",
                    "could not measure its delta against the target's "
                    "authored transform");
    return true;
}

bool
RunConstraintDelta(const RigExecOpBatch &batch, size_t *failed,
                   std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ConstraintDeltaOne(batch, k, error);
    });
}

void
MatrixMoverValues(const void *payload, RigExecFieldValues &out)
{
    SetFlag(out, kRadialBlend,
            static_cast<const MatrixMoverPayload *>(payload)->radialBlend);
}

const rigExec::RigExecFieldDesc kMatrixMoverFields[] = {
    Flag("radialBlend"),
};

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kMatrixMoverPorts[] = {
    {&MatrixMoverIn(), &rigExecRig::PointFrameType(), kRigExecPortInput, "in"},
    {&MatrixMoverTransform(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "transform"},
    {&MatrixMoverTransformRest(), &rigExecRig::PointFrameType(), kOptional,
     "transformRest"},
    {&MatrixMoverSpace(), &rigExecRig::PointFrameType(), kOptional, "space"},
    {&MatrixMoverSpaceRest(), &rigExecRig::PointFrameType(), kOptional,
     "spaceRest"},
    {&MatrixMoverWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&MatrixMoverOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kConstraintDeltaPorts[] = {
    {&ConstraintDeltaSolved(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "solved"},
    {&ConstraintDeltaBase(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "base"},
    {&ConstraintDeltaBaseOk(), &rigExecStd::BoolType(), kRigExecPortInput,
     "baseOk"},
    {&ConstraintDeltaWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&ConstraintDeltaDelta(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "delta"},
};

const rigExec::RigExecOpDesc kMatrixMoverDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "MatrixMover";
    d.ports = kMatrixMoverPorts;
    d.portCount = std::size(kMatrixMoverPorts);
    d.runBatch = &RunMatrixMover;
    d.validate = &CheckMatrixMover;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kMatrixMoverFields;
    d.payloadFieldCount = std::size(kMatrixMoverFields);
    d.payloadVersion = 1;
    d.payloadValues = &MatrixMoverValues;
    return d;
}();

const rigExec::RigExecOpDesc kConstraintDeltaDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "ConstraintDelta";
    d.ports = kConstraintDeltaPorts;
    d.portCount = std::size(kConstraintDeltaPorts);
    d.runBatch = &RunConstraintDelta;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    return d;
}();

} // namespace

void
RegisterMatrixMoverOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(MatrixMover(), kMatrixMoverDesc);
    registrar.Add(ConstraintDelta(), kConstraintDeltaDesc);
}

} // namespace rigExecOps
