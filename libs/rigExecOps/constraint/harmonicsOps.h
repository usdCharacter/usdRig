#ifndef RIGEXEC_OPS_CONSTRAINT_HARMONICS_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_HARMONICS_OPS_H

/// Harmonic secondary motion: a Windowed op whose capture is its target's
/// incoming origin in its space, and whose result adds the ringing offset of
/// the earlier ticks' captures. The kernel is rigExecMath/harmonicsKernel.h,
/// shared with the native and zero-USD runtimes. USD-free.

#include "rigExecMath/harmonicsKernel.h"
#include "rigExecOps/library.h"

#include <cstdint>

namespace rigExecOps {

struct HarmonicsPayload {
    rigExec::RigExecHarmonicsSettings settings;
    bool matchVelocity = true;
    /// The Space port is fed; a singular Space measures nothing.
    bool hasSpace = false;
    /// The window's earlier ticks.
    uint32_t frames = 0;
};

/// Why Out is In rather than the shifted frame, or Applied.
enum class HarmonicsStatus : uint32_t {
    Applied,
    /// Captured only, disabled, zero weight, or no ringing history yet.
    PassedThrough,
    /// In is not a usable frame: nothing was captured.
    Unusable,
    NoMeasure,
    NoMeasureDegenerateSpace,
    NonFinite,
    InvalidResult,
};

const rigExec::RigExecOpHandle &Harmonics();

// In (PointFrame), optional Space (Mat4d, the measuring space), Enabled
// (Bool), Weight (Double, unchecked), Amplitude, Decay and Termination
// (Double), AxisAmplitude (Vec3d). Out (PointFrame) and Status (UInt, a
// HarmonicsStatus).
const rigExec::RigExecPortHandle &HarmonicsIn();
const rigExec::RigExecPortHandle &HarmonicsSpace();
const rigExec::RigExecPortHandle &HarmonicsEnabled();
const rigExec::RigExecPortHandle &HarmonicsWeight();
const rigExec::RigExecPortHandle &HarmonicsAmplitude();
const rigExec::RigExecPortHandle &HarmonicsDecay();
const rigExec::RigExecPortHandle &HarmonicsTermination();
const rigExec::RigExecPortHandle &HarmonicsAxisAmplitude();
const rigExec::RigExecPortHandle &HarmonicsOut();
const rigExec::RigExecPortHandle &HarmonicsStatusOut();

} // namespace rigExecOps

#endif
