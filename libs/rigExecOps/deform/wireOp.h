#ifndef RIGEXEC_OPS_DEFORM_WIRE_OP_H
#define RIGEXEC_OPS_DEFORM_WIRE_OP_H

/// Wire: each point moves by f(d) * (C(u) - C0(u)), C the posed curve, C0
/// the rest curve, (u, d) its bind coordinates and f = 1 - smoothstep(0,
/// dropoff, d). The kernel is rigExecMath/wireKernel.h, shared with the
/// baked and zero-USD runtimes. USD-free.

#include "rigExecOps/library.h"

#include <vector>

namespace rigExecOps {

/// The curve layout and falloff, fixed at compile.
struct WirePayload {
    int order = 4;
    std::vector<double> knots; // control point count + order
    double dropoffDistance = 0.0;
};

const rigExec::RigExecOpHandle &Wire();

// Points, RestCurve and Curve (Vec3fArray; the curves' control points),
// BindCoords (Vec2fArray, one per point), Out. Either an Envelope
// (FloatArray, blended after, as the point ops) or a sparse weight field:
// WeightIndices (IntArray, in range) and Weights (FloatArray, parallel),
// folded into the motion of only the named points, with BindCoords one per
// point or one per index. Both at once, or curves, knots or binds that do
// not fit, are errors.
const rigExec::RigExecPortHandle &WirePoints();
const rigExec::RigExecPortHandle &WireRestCurve();
const rigExec::RigExecPortHandle &WireCurve();
const rigExec::RigExecPortHandle &WireBindCoords();
const rigExec::RigExecPortHandle &WireEnvelope();
const rigExec::RigExecPortHandle &WireWeightIndices();
const rigExec::RigExecPortHandle &WireWeights();
const rigExec::RigExecPortHandle &WireOut();

} // namespace rigExecOps

#endif
