#ifndef RIGEXEC_OPS_PROPERTY_MATH_OPS_H
#define RIGEXEC_OPS_PROPERTY_MATH_OPS_H

/// The property math ops: one float, vec3f or matrix4d value revised by an
/// operation and mixed back toward the incoming value by the weight. The
/// arithmetic is rigExecMath/propertyMathKernel.h.

#include "rigExecOps/library.h"
#include "rigExecMath/pathCurveKernel.h"
#include "rigExecMath/propertyMathKernel.h"
#include "rigExecStdTypes/stdTypes.h"

#include <vector>

namespace rigExecOps {

using rigExec::RigExecPropertyOp;

/// What every math op is configured with at compile.
///
/// keys and tangents are the curve operation's (input, output) and
/// (in slope, out slope) pairs; tangents is empty or one per key. path is
/// the path operation's settings; its curve points arrive on PathPoints.
struct MathPayload {
    RigExecPropertyOp op = RigExecPropertyOp::Add;
    std::vector<rigExecStd::Vec2f> keys;
    std::vector<rigExecStd::Vec2f> tangents;
    rigExec::RigExecPathSettings path;
};

/// out = mix(in, op(in), weight), on a float. Every operation.
const rigExec::RigExecOpHandle &FloatMath();
/// The vec3f peer: add, multiply, clamp, remap and blend, per component.
const rigExec::RigExecOpHandle &Vec3fMath();
/// The matrix4d peer: multiply (in * value) and blend.
const rigExec::RigExecOpHandle &MatrixMath();

// Ports. Each op has In, Weight and Out; Value, Min and Max where its
// operations read them, and FloatMath has PathPoints. Value, Min, Max and
// PathPoints are optional: an unfed one reads as zero, identity for
// MatrixMath's Value and empty for PathPoints. Weight is a float on all
// three.
const rigExec::RigExecPortHandle &FloatIn();
const rigExec::RigExecPortHandle &FloatValue();
const rigExec::RigExecPortHandle &FloatMin();
const rigExec::RigExecPortHandle &FloatMax();
const rigExec::RigExecPortHandle &FloatWeight();
const rigExec::RigExecPortHandle &FloatPathPoints();
const rigExec::RigExecPortHandle &FloatOut();

const rigExec::RigExecPortHandle &Vec3fIn();
const rigExec::RigExecPortHandle &Vec3fValue();
const rigExec::RigExecPortHandle &Vec3fMin();
const rigExec::RigExecPortHandle &Vec3fMax();
const rigExec::RigExecPortHandle &Vec3fWeight();
const rigExec::RigExecPortHandle &Vec3fOut();

const rigExec::RigExecPortHandle &MatrixIn();
const rigExec::RigExecPortHandle &MatrixValue();
const rigExec::RigExecPortHandle &MatrixWeight();
const rigExec::RigExecPortHandle &MatrixOut();

} // namespace rigExecOps

#endif
