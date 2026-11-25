#ifndef RIGEXEC_OPS_DEFORM_CLOTH_OP_H
#define RIGEXEC_OPS_DEFORM_CLOTH_OP_H

/// Cloth: a windowed cloth solve on the CPU. Each run records the tick's
/// entering points, collider points and parameters, then simulates the
/// window of recorded ticks up to the current one. The kernel is
/// rigExecMath/clothKernel.h, reached through rigExecBinary/clothPlayback.h.

#include "rigExecBinary/clothPlayback.h"
#include "rigExecOps/library.h"

#include <cstdint>

namespace rigExecOps {

/// The compiled cloth rig: mesh, rest shape, pins, solver settings, frame
/// length, collider topology and the constraint sets' material model.
struct ClothPayload {
    rigExec::RigExecClothCompiledData compiledRig;

    bool operator==(const ClothPayload &o) const
    {
        return compiledRig == o.compiledRig;
    }
};

const rigExec::RigExecOpHandle &Cloth();

// Points (Vec3fArray), Parameters (FloatArray, RigExecClothParameterCount
// in packet order), Colliders (Vec3fArray each, in compiled rig order; any
// number), Envelope (optional FloatArray, one per point), Out. A history
// tick that only records passes the points through; a solve that fails
// fails the evaluation, the tick staying recorded.
const rigExec::RigExecPortHandle &ClothPoints();
const rigExec::RigExecPortHandle &ClothParameters();
const rigExec::RigExecPortHandle &ClothColliders();
const rigExec::RigExecPortHandle &ClothEnvelope();
const rigExec::RigExecPortHandle &ClothOut();

} // namespace rigExecOps

#endif
