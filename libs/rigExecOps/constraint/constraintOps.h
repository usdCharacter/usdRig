#ifndef RIGEXEC_OPS_CONSTRAINT_CONSTRAINT_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_CONSTRAINT_OPS_H

/// The FBX-equivalent Position, Rotation, Scale, Parent and Aim
/// constraints on rigExecRig::PointFrame. The kernel is
/// rigExecMath/constraintKernel.h, shared with the native and zero-USD
/// runtimes. USD-free.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

/// The axes a constraint governs; the others keep the input's.
struct ConstraintAxes {
    bool x = true;
    bool y = true;
    bool z = true;
};

/// Euler orders, applied left to right in row-vector convention.
enum class ConstraintEulerOrder : uint8_t { XYZ, XZY, YXZ, YZX, ZXY, ZYX };

/// What every constraint op is configured with at compile. Each op reads
/// the fields its kind uses.
struct ConstraintPayload {
    /// Position and Scale: added to the sources' mean. Rotation: degrees
    /// added to the sources' mean Euler angles. Aim: degrees added to the
    /// aimed Euler angles.
    rigExecStd::Vec3d offset{0.0, 0.0, 0.0};
    /// Position, Rotation, Scale: the governed axes. Aim: rotation axes.
    ConstraintAxes affect;
    /// Parent.
    ConstraintAxes translationAxes;
    ConstraintAxes rotationAxes;
    ConstraintAxes scaleAxes{false, false, false};
    /// Parent: per-source local offsets, empty or one per source.
    std::vector<rigExecStd::Vec3d> translationOffsets;
    std::vector<rigExecStd::Vec3d> rotationOffsetsDegrees;
    ConstraintEulerOrder rotationOrder = ConstraintEulerOrder::XYZ;
    /// Scale and Parent: with all three scale axes governed, blend shear.
    bool blendShear = false;
    /// Aim.
    rigExecStd::Vec3d localAimVector{1.0, 0.0, 0.0};
    rigExecStd::Vec3d localUpVector{0.0, 1.0, 0.0};
    /// Aim without a fed WorldUp: keep the input's up (true) or take the
    /// minimum swing (false, FBX worldUpType None).
    bool preserveInputUp = true;
};

const rigExec::RigExecOpHandle &PositionConstraint();
const rigExec::RigExecOpHandle &RotationConstraint();
const rigExec::RigExecOpHandle &ScaleConstraint();
const rigExec::RigExecOpHandle &ParentConstraint();
const rigExec::RigExecOpHandle &AimConstraint();

// Every op: In (PointFrame), Weight (Double, clamped to [0, 1]), Out
// (PointFrame). A frame the constraint cannot solve comes back as In flagged
// degenerate.
//
// Position, Rotation, Scale and Parent: Sources (PointFrameArray, rests
// unread) and optional SourceWeights (DoubleArray, one per source; unfed,
// each source weighs 1). Rotation and Parent: optional Space (Mat4d), the
// rest-to-pose map of the provider carrying the rig, stripped before the
// axis mask and restored after it.
//
// Aim: Target (Vec3d) and optional WorldUp (Vec3d), which pins roll.
const rigExec::RigExecPortHandle &PositionConstraintIn();
const rigExec::RigExecPortHandle &PositionConstraintSources();
const rigExec::RigExecPortHandle &PositionConstraintSourceWeights();
const rigExec::RigExecPortHandle &PositionConstraintWeight();
const rigExec::RigExecPortHandle &PositionConstraintOut();

const rigExec::RigExecPortHandle &RotationConstraintIn();
const rigExec::RigExecPortHandle &RotationConstraintSources();
const rigExec::RigExecPortHandle &RotationConstraintSourceWeights();
const rigExec::RigExecPortHandle &RotationConstraintWeight();
const rigExec::RigExecPortHandle &RotationConstraintSpace();
const rigExec::RigExecPortHandle &RotationConstraintOut();

const rigExec::RigExecPortHandle &ScaleConstraintIn();
const rigExec::RigExecPortHandle &ScaleConstraintSources();
const rigExec::RigExecPortHandle &ScaleConstraintSourceWeights();
const rigExec::RigExecPortHandle &ScaleConstraintWeight();
const rigExec::RigExecPortHandle &ScaleConstraintOut();

const rigExec::RigExecPortHandle &ParentConstraintIn();
const rigExec::RigExecPortHandle &ParentConstraintSources();
const rigExec::RigExecPortHandle &ParentConstraintSourceWeights();
const rigExec::RigExecPortHandle &ParentConstraintWeight();
const rigExec::RigExecPortHandle &ParentConstraintSpace();
const rigExec::RigExecPortHandle &ParentConstraintOut();

const rigExec::RigExecPortHandle &AimConstraintIn();
const rigExec::RigExecPortHandle &AimConstraintTarget();
const rigExec::RigExecPortHandle &AimConstraintWorldUp();
const rigExec::RigExecPortHandle &AimConstraintWeight();
const rigExec::RigExecPortHandle &AimConstraintOut();

} // namespace rigExecOps

#endif
