// RigExecVolumeCorrectMover: everything about the volume-correct mover. A
// volume-correct mover preserves the authored base bound volume against the
// deformation the chain has applied so far. This TU owns its revision
// binder and its parity-oracle branch, and registers the row that points
// at them. Compile validation is the generic points-target + single-target
// rules.
#include "rigExecSchema/schemaNames.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/moverPacket.h"

#include "rigExecMath/geometryKernels.h"

using rigExec::RigExecMoverParameters;

PXR_NAMESPACE_USING_DIRECTIVE

// Interned once: callbacks and validators run on parallel lanes, and
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_volumeCorrectTokens,
    (volumeCorrect));

namespace {

void
_BindVolumeCorrectMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    binding.base = ctx.target;
}

rigExec::RigExecOracleResult
_OracleVolumeCorrectMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const rigExec::RigExecOracleScene &stage = ctx.stage;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    VtVec3fArray &points = *ctx.points;
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    VtVec3fArray base;
    if (rigExec::RigExecOracleAttribute a = stage->GetAttributeAtPath(target)) {
        a.Get(&base, time);
    }
    const double reference =
        rigExec::RigExecBoundVolume(base.cdata(), base.size());
    rigExec::RigExecApplyVolumeCorrect(&scratch, reference, 1.0f);
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        rigExec::SchemaTypeName::RigExecVolumeCorrectMover,
        &rigExec::RigExecFixedMoverOp<
            rigExec::RigExecRevisionOp::VolumeCorrect>,
        rigExec::RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.bind = &_BindVolumeCorrectMover;
    handler.oracle = &_OracleVolumeCorrectMover;
    return handler;
}

} // namespace

RIGEXEC_REGISTER_MOVER(_MakeHandler());
