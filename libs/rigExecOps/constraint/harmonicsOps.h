#ifndef RIGEXEC_OPS_CONSTRAINT_HARMONICS_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_HARMONICS_OPS_H

/// Harmonic secondary motion: a Windowed op whose capture is its target's
/// incoming origin in its space, and whose result adds the ringing offset of
/// the earlier ticks' captures. The kernel is rigExecMath/harmonicsKernel.h.

#include "rigExecMath/harmonicsKernel.h"
#include "rigExecOps/library.h"

#include <cstdint>

namespace rigExecOps {

struct HarmonicsPayload {
    rigExec::RigExecHarmonicsSettings settings;
    bool matchVelocity = true;
    /// The Space port is fed.
    bool hasSpace = false;
    /// The window's earlier ticks.
    uint32_t frames = 0;

    bool operator==(const HarmonicsPayload &o) const
    {
        return settings == o.settings && matchVelocity == o.matchVelocity &&
               hasSpace == o.hasSpace && frames == o.frames;
    }
};

/// What a Harmonics node's Out holds.
enum class HarmonicsResult : uint32_t {
    /// In: the tick only captures, the op is disabled or weightless, or
    /// nothing rings yet.
    Passed = 0,
    /// In moved by the ringing offset.
    Solved = 1,
    /// In: its origin could not be measured in its space, or in a space
    /// that is degenerate.
    Unmeasured = 2,
    UnmeasuredInDegenerateSpace = 3,
};

const rigExec::RigExecOpHandle &Harmonics();

// In (PointFrame), optional Space (Mat4d, the measuring space), Enabled
// (Bool), Weight (Double, unchecked), Amplitude, Decay and Termination
// (Double), AxisAmplitude (Vec3d). Out (PointFrame) and Status (UInt, a
// HarmonicsResult). A non-finite incoming frame or parameter of a node that
// solves is an error.
const rigExec::RigExecPortHandle &HarmonicsIn();
const rigExec::RigExecPortHandle &HarmonicsSpace();
const rigExec::RigExecPortHandle &HarmonicsEnabled();
const rigExec::RigExecPortHandle &HarmonicsWeight();
const rigExec::RigExecPortHandle &HarmonicsAmplitude();
const rigExec::RigExecPortHandle &HarmonicsDecay();
const rigExec::RigExecPortHandle &HarmonicsTermination();
const rigExec::RigExecPortHandle &HarmonicsAxisAmplitude();
const rigExec::RigExecPortHandle &HarmonicsOut();
const rigExec::RigExecPortHandle &HarmonicsStatus();

} // namespace rigExecOps

#endif
