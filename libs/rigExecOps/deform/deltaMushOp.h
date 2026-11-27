#ifndef RIGEXEC_OPS_DEFORM_DELTA_MUSH_OP_H
#define RIGEXEC_OPS_DEFORM_DELTA_MUSH_OP_H

/// Delta mush: smooths the points, then restores the rest pose's detail in
/// frames transported from the smoothed rest. The kernel is
/// rigExecMath/deltaMushKernel.h.

#include "rigExecMath/deltaMushSettings.h"
#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExecOps {

/// What a delta mush is configured with at compile.
struct DeltaMushPayload {
    int iterations = 10;
    double step = 0.5;
    bool pinBorders = true;
    double distanceWeight = 0.0;
    double displacement = 1.0;
    rigExec::RigExecDeltaMushSettings settings;

    bool operator==(const DeltaMushPayload &o) const
    {
        return iterations == o.iterations && step == o.step &&
               pinBorders == o.pinBorders &&
               distanceWeight == o.distanceWeight &&
               displacement == o.displacement && settings == o.settings;
    }
};

const rigExec::RigExecOpHandle &DeltaMush();

// Points and Rest (Vec3fArray), Counts and Indices (IntArray), optional
// ComputationToTarget (Mat4d, unfed identity; the rest is already in
// computation space) and Envelope (FloatArray, as the point ops), Out.
// An unusable computation map or topology is an error.
//
// A mover's own inputs come as Enabled (Bool; false passes the points
// through), DefaultWeight (Float; finite and in [0, 1] or an error) and a
// weight object's packet as Packet (PointWeights; an error when it does not
// resolve), at most one of Envelope, DefaultWeight and Packet fed; Written
// (IntArray, ascending [begin, end) pairs) the points the revision writes,
// an error when the packet's weights reach past them. Step, DistanceWeight
// and Displacement (Float), PinBorders and OnlySmooth (Bool), Smoothing and
// FrameTransport (Int, the settings' enum values) and SmoothWeights
// (FloatArray) override the payload's settings when fed. Frame (Mat4d)
// composes after ComputationToTarget. Base (Vec3fArray) is the rest when
// Rest is empty, which a map other than the identity or a Frame refuses
// unless the mover only smooths.
const rigExec::RigExecPortHandle &DeltaMushPoints();
const rigExec::RigExecPortHandle &DeltaMushRest();
const rigExec::RigExecPortHandle &DeltaMushCounts();
const rigExec::RigExecPortHandle &DeltaMushIndices();
const rigExec::RigExecPortHandle &DeltaMushComputationToTarget();
const rigExec::RigExecPortHandle &DeltaMushEnvelope();
const rigExec::RigExecPortHandle &DeltaMushOut();
const rigExec::RigExecPortHandle &DeltaMushEnabled();
const rigExec::RigExecPortHandle &DeltaMushDefaultWeight();
const rigExec::RigExecPortHandle &DeltaMushPacket();
const rigExec::RigExecPortHandle &DeltaMushWritten();
const rigExec::RigExecPortHandle &DeltaMushStep();
const rigExec::RigExecPortHandle &DeltaMushPinBorders();
const rigExec::RigExecPortHandle &DeltaMushDistanceWeight();
const rigExec::RigExecPortHandle &DeltaMushDisplacement();
const rigExec::RigExecPortHandle &DeltaMushSmoothing();
const rigExec::RigExecPortHandle &DeltaMushFrameTransport();
const rigExec::RigExecPortHandle &DeltaMushSmoothWeights();
const rigExec::RigExecPortHandle &DeltaMushOnlySmooth();
const rigExec::RigExecPortHandle &DeltaMushFrame();
const rigExec::RigExecPortHandle &DeltaMushBase();

} // namespace rigExecOps

#endif
