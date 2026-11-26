#ifndef RIGEXEC_OPS_CONSTRAINT_CONSTRAINT_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_CONSTRAINT_OPS_H

/// The FBX-equivalent Position, Rotation, Scale, Parent and Aim constraints on
/// rigExecRig::PointFrame. The kernel is rigExecMath/constraintKernel.h.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstdint>

namespace rigExecOps {

/// The axes a constraint governs; the others keep the input's.
struct ConstraintAxes {
    bool x = true;
    bool y = true;
    bool z = true;

    bool operator==(const ConstraintAxes &o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

/// Euler orders, applied left to right in row-vector convention.
enum class ConstraintEulerOrder : uint8_t { XYZ, XZY, YXZ, YZX, ZXY, ZYX };

/// What every constraint op is configured with at compile: the choices the
/// rig fixes. Each op reads the fields its kind uses; every value a frame can
/// change is an input port.
struct ConstraintPayload {
    ConstraintEulerOrder rotationOrder = ConstraintEulerOrder::XYZ;
    /// Scale and Parent: with all three scale axes governed, blend shear.
    bool blendShear = false;
    /// Aim without a fed WorldUp: keep the input's up (true) or take the
    /// minimum swing (false, FBX worldUpType None).
    bool preserveInputUp = true;

    bool operator==(const ConstraintPayload &o) const
    {
        return rotationOrder == o.rotationOrder && blendShear == o.blendShear &&
               preserveInputUp == o.preserveInputUp;
    }
};

const rigExec::RigExecOpHandle &PositionConstraint();
const rigExec::RigExecOpHandle &RotationConstraint();
const rigExec::RigExecOpHandle &ScaleConstraint();
const rigExec::RigExecOpHandle &ParentConstraint();
const rigExec::RigExecOpHandle &AimConstraint();

// Every op: In (PointFrame), Weight (Double, clamped to [0, 1]), Out
// (PointFrame). A frame the constraint cannot solve comes back as In flagged
// degenerate. Axis masks are BoolArray inputs of three (x, y, z).
//
// Position, Rotation, Scale and Parent: Sources (PointFrameArray, rests
// unread) and optional SourceWeights (DoubleArray, one per source; unfed,
// each source weighs 1). Rotation and Parent: optional Space (Mat4d), the
// rest-to-pose map of the provider carrying the rig, stripped before the
// axis mask and restored after it.
//
// Position, Rotation, Scale and Aim: optional Offset (Vec3d; unfed, zero):
// Position and Scale add it to the sources' mean, Rotation adds it in
// degrees to the mean Euler angles, Aim to the aimed Euler angles; and
// optional Affect (the governed axes, Aim's rotation axes; unfed, all).
//
// Parent: optional TranslationAxes and RotationAxes (unfed, all) and
// ScaleAxes (unfed, none); optional TranslationOffsets and RotationOffsets
// (Vec3dArray, degrees for rotation; unfed, zero; fed, one per source).
//
// Aim: Target (Vec3d), optional WorldUp (Vec3d), which pins roll, and
// optional AimVector (unfed, +X) and UpVector (unfed, +Y) in the target's
// local frame.
const rigExec::RigExecPortHandle &PositionConstraintIn();
const rigExec::RigExecPortHandle &PositionConstraintSources();
const rigExec::RigExecPortHandle &PositionConstraintSourceWeights();
const rigExec::RigExecPortHandle &PositionConstraintWeight();
const rigExec::RigExecPortHandle &PositionConstraintOffset();
const rigExec::RigExecPortHandle &PositionConstraintAffect();
const rigExec::RigExecPortHandle &PositionConstraintOut();

const rigExec::RigExecPortHandle &RotationConstraintIn();
const rigExec::RigExecPortHandle &RotationConstraintSources();
const rigExec::RigExecPortHandle &RotationConstraintSourceWeights();
const rigExec::RigExecPortHandle &RotationConstraintWeight();
const rigExec::RigExecPortHandle &RotationConstraintSpace();
const rigExec::RigExecPortHandle &RotationConstraintOffset();
const rigExec::RigExecPortHandle &RotationConstraintAffect();
const rigExec::RigExecPortHandle &RotationConstraintOut();

const rigExec::RigExecPortHandle &ScaleConstraintIn();
const rigExec::RigExecPortHandle &ScaleConstraintSources();
const rigExec::RigExecPortHandle &ScaleConstraintSourceWeights();
const rigExec::RigExecPortHandle &ScaleConstraintWeight();
const rigExec::RigExecPortHandle &ScaleConstraintOffset();
const rigExec::RigExecPortHandle &ScaleConstraintAffect();
const rigExec::RigExecPortHandle &ScaleConstraintOut();

const rigExec::RigExecPortHandle &ParentConstraintIn();
const rigExec::RigExecPortHandle &ParentConstraintSources();
const rigExec::RigExecPortHandle &ParentConstraintSourceWeights();
const rigExec::RigExecPortHandle &ParentConstraintWeight();
const rigExec::RigExecPortHandle &ParentConstraintSpace();
const rigExec::RigExecPortHandle &ParentConstraintTranslationAxes();
const rigExec::RigExecPortHandle &ParentConstraintRotationAxes();
const rigExec::RigExecPortHandle &ParentConstraintScaleAxes();
const rigExec::RigExecPortHandle &ParentConstraintTranslationOffsets();
const rigExec::RigExecPortHandle &ParentConstraintRotationOffsets();
const rigExec::RigExecPortHandle &ParentConstraintOut();

const rigExec::RigExecPortHandle &AimConstraintIn();
const rigExec::RigExecPortHandle &AimConstraintTarget();
const rigExec::RigExecPortHandle &AimConstraintWorldUp();
const rigExec::RigExecPortHandle &AimConstraintWeight();
const rigExec::RigExecPortHandle &AimConstraintOffset();
const rigExec::RigExecPortHandle &AimConstraintAffect();
const rigExec::RigExecPortHandle &AimConstraintAimVector();
const rigExec::RigExecPortHandle &AimConstraintUpVector();
const rigExec::RigExecPortHandle &AimConstraintOut();

} // namespace rigExecOps

#endif
