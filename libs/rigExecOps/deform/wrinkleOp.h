#ifndef RIGEXEC_OPS_DEFORM_WRINKLE_OP_H
#define RIGEXEC_OPS_DEFORM_WRINKLE_OP_H

/// Wrinkle: guided quasistatic edge constraints that buckle compressed regions
/// of a mesh into wrinkles. The kernel is rigExecMath/wrinkleKernel.h.

#include "rigExecMath/wrinkleSettings.h"
#include "rigExecOps/library.h"

namespace rigExecOps {

/// What a wrinkle is configured with at compile.
struct WrinklePayload {
    rigExec::RigExecWrinkleSettings settings;

    bool operator==(const WrinklePayload &o) const
    {
        return settings == o.settings;
    }
};

const rigExec::RigExecOpHandle &Wrinkle();

// Points, RestPoints (Vec3fArray, one per point; empty reads Base when it
// is fed), Counts and Indices (IntArray, the rest mesh's faces), Envelope
// (optional FloatArray, one per point), Out. Optional overrides of the
// payload, each looked up per frame: Settings (DoubleArray of the twelve
// scalars in RigExecWrinkleSettings order, iterations to
// smoothingIterations, flags 0 or 1), Topology (Int: 0 cloth, 1
// surfaceStruts) and PinPoints (IntArray). Enabled, DefaultWeight, Packet
// and Written as VolumeCorrect's. Runs once over every point. Unusable
// settings, topology or points are an error.
const rigExec::RigExecPortHandle &WrinklePoints();
const rigExec::RigExecPortHandle &WrinkleRest();
const rigExec::RigExecPortHandle &WrinkleCounts();
const rigExec::RigExecPortHandle &WrinkleIndices();
const rigExec::RigExecPortHandle &WrinkleEnvelope();
const rigExec::RigExecPortHandle &WrinkleOut();
const rigExec::RigExecPortHandle &WrinkleBase();
const rigExec::RigExecPortHandle &WrinkleSettings();
const rigExec::RigExecPortHandle &WrinkleTopology();
const rigExec::RigExecPortHandle &WrinklePinPoints();
const rigExec::RigExecPortHandle &WrinkleEnabled();
const rigExec::RigExecPortHandle &WrinkleDefaultWeight();
const rigExec::RigExecPortHandle &WrinklePacket();
const rigExec::RigExecPortHandle &WrinkleWritten();

} // namespace rigExecOps

#endif
