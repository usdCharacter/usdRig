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
const rigExec::RigExecPortHandle &DeltaMushPoints();
const rigExec::RigExecPortHandle &DeltaMushRest();
const rigExec::RigExecPortHandle &DeltaMushCounts();
const rigExec::RigExecPortHandle &DeltaMushIndices();
const rigExec::RigExecPortHandle &DeltaMushComputationToTarget();
const rigExec::RigExecPortHandle &DeltaMushEnvelope();
const rigExec::RigExecPortHandle &DeltaMushOut();

} // namespace rigExecOps

#endif
