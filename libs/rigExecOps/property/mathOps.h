#ifndef RIGEXEC_OPS_PROPERTY_MATH_OPS_H
#define RIGEXEC_OPS_PROPERTY_MATH_OPS_H

/// The property math ops: one float, double, vec3f or matrix4d value revised
/// by an operation and mixed back toward the incoming value by the weight.
/// The arithmetic is rigExecMath/propertyMathKernel.h.

#include "rigExecOps/library.h"
#include "rigExecMath/pathCurveKernel.h"
#include "rigExecMath/propertyMathKernel.h"
#include "rigExecStdTypes/stdTypes.h"

#include <vector>

namespace rigExecOps {

using rigExec::RigExecPropertyOp;

/// What every math op is configured with at compile: the operation and the
/// path operation's settings; the path's curve points arrive on PathPoints.
struct MathPayload {
    RigExecPropertyOp op = RigExecPropertyOp::Add;
    rigExec::RigExecPathSettings path;

    bool operator==(const MathPayload &o) const
    {
        return op == o.op && path == o.path;
    }
};

/// out = mix(in, op(in), weight), on a float. Every operation.
const rigExec::RigExecOpHandle &FloatMath();
/// The vec3f peer: add, multiply, clamp, remap and blend, per component.
const rigExec::RigExecOpHandle &Vec3fMath();
/// The matrix4d peer: multiply (in * value) and blend.
const rigExec::RigExecOpHandle &MatrixMath();
/// FloatMath on a double: computed in float on float(in), widened back.
const rigExec::RigExecOpHandle &DoubleMath();

// Ports. Each op has In, Weight, Enabled and Out; Value, Min and Max where
// its operations read them; FloatMath and DoubleMath also have PathPoints,
// and Keys and Tangents, the curve's (input, output) and (in slope, out
// slope) pairs, tangents none or one per key. All but In and Weight are
// optional: an unfed one reads as zero, identity for MatrixMath's Value,
// empty for the arrays and true for Enabled. A disabled node's Out is its
// In. Weight is a float everywhere; it must be finite and in [0, 1], and
// operands and the result finite, or the evaluation fails.
const rigExec::RigExecPortHandle &FloatIn();
const rigExec::RigExecPortHandle &FloatValue();
const rigExec::RigExecPortHandle &FloatMin();
const rigExec::RigExecPortHandle &FloatMax();
const rigExec::RigExecPortHandle &FloatWeight();
const rigExec::RigExecPortHandle &FloatPathPoints();
const rigExec::RigExecPortHandle &FloatOut();
const rigExec::RigExecPortHandle &FloatEnabled();
const rigExec::RigExecPortHandle &FloatKeys();
const rigExec::RigExecPortHandle &FloatTangents();

const rigExec::RigExecPortHandle &DoubleIn();
const rigExec::RigExecPortHandle &DoubleValue();
const rigExec::RigExecPortHandle &DoubleMin();
const rigExec::RigExecPortHandle &DoubleMax();
const rigExec::RigExecPortHandle &DoubleWeight();
const rigExec::RigExecPortHandle &DoublePathPoints();
const rigExec::RigExecPortHandle &DoubleOut();
const rigExec::RigExecPortHandle &DoubleEnabled();
const rigExec::RigExecPortHandle &DoubleKeys();
const rigExec::RigExecPortHandle &DoubleTangents();

const rigExec::RigExecPortHandle &Vec3fIn();
const rigExec::RigExecPortHandle &Vec3fValue();
const rigExec::RigExecPortHandle &Vec3fMin();
const rigExec::RigExecPortHandle &Vec3fMax();
const rigExec::RigExecPortHandle &Vec3fWeight();
const rigExec::RigExecPortHandle &Vec3fOut();
const rigExec::RigExecPortHandle &Vec3fEnabled();

const rigExec::RigExecPortHandle &MatrixIn();
const rigExec::RigExecPortHandle &MatrixValue();
const rigExec::RigExecPortHandle &MatrixWeight();
const rigExec::RigExecPortHandle &MatrixOut();
const rigExec::RigExecPortHandle &MatrixEnabled();

} // namespace rigExecOps

#endif
