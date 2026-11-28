#include "constraintProgram.h"
#include "rigExecMath/switchMathKernel.h"
#include <cmath>
namespace rigExec {
bool
RigExecConstraintFrameUsable(const RigExecPointFrame &frame)
{
    return RigExecFrameUsableT(frame.points, frame.IsValid(),
                               frame.IsDegenerate());
}
bool
RigExecRunConstraint(const RigExecConstraintRecord &record,
                     const RigExecConstraintInputs &input,
                     RigExecConstraintResult *output, std::string *error)
{
    if (!output) {
        if (error)
            *error = "null constraint output";
        return false;
    }
    output->chain.clear();
    output->frame = input.incoming;
    switch (record.kind) {
    case RigExecConstraintKind::Position:
        output->frame = RigExecApplyPositionConstraint(
            input.incoming, input.sources, record.position);
        return true;
    case RigExecConstraintKind::Rotation: {
        auto params = record.rotation;
        params.carry = input.carry ? &*input.carry : nullptr;
        output->frame = RigExecApplyRotationConstraint(input.incoming,
                                                       input.sources, params);
        return true;
    }
    case RigExecConstraintKind::Scale:
        output->frame = RigExecApplyScaleConstraint(
            input.incoming, input.sources, record.scale);
        return true;
    case RigExecConstraintKind::Parent: {
        auto params = record.parent;
        params.carry = input.carry ? &*input.carry : nullptr;
        output->frame =
            RigExecApplyParentConstraint(input.incoming, input.sources, params);
        return true;
    }
    case RigExecConstraintKind::Aim:
        output->frame = RigExecApplyAimConstraint(input.incoming,
                                                  input.aimTarget, record.aim);
        return true;
    case RigExecConstraintKind::SingleChainIk:
        output->chain = RigExecSolveSingleChainIk(input.chain, input.effector,
                                                  record.singleChain);
        return true;
    case RigExecConstraintKind::IkFkMatch:
        if (error)
            *error =
                "an IK/FK match runs in the pose walk, not as a frame kernel";
        return false;
    case RigExecConstraintKind::Harmonics:
        if (error)
            *error = "a harmonics constraint reads its history window in the "
                     "pose walk, not as a frame kernel";
        return false;
    }
    if (error)
        *error = "unknown constraint kind";
    return false;
}
} // namespace rigExec
