#ifndef RIGEXEC_SCENE_GEOMETRY_OPS_H
#define RIGEXEC_SCENE_GEOMETRY_OPS_H

/// Host-side bridge from a geometry revision to rigExecOps' deform ops: the
/// packet's settings become the op's payload and its per-frame values its
/// inputs, so each RigExecMoverParameters field has a port or a payload
/// field. Not installed; step 10 moves it to rigExecUsd.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecScene/geometryProgram.h"

#include <string>
#include <vector>

namespace rigExec {

/// One deform op node and its host-written ports. Ports the op lacks keep
/// an invalid id.
struct RigExecGeometryOpNode {
    RigExecNodeId node = 0;
    RigExecRevisionOp op = RigExecRevisionOp::Matrix;
    /// Invalid when an upstream node feeds the points.
    RigExecInputId points;
    RigExecInputId envelope;
    RigExecInputId transform, strength, reference;
    RigExecInputId counts, indices, rest;
    RigExecInputId transforms, skinIndices, skinWeights;
    RigExecOutputId out;
    /// The op's output port, for a downstream node to connect to.
    const RigExecPortHandle *outPort = nullptr;
};

/// True for the revision ops this bridge maps.
bool RigExecGeometryOpSupported(RigExecRevisionOp op);

/// Adds the deform op for \p op to \p builder, its payload taken from
/// \p p. The points come from \p upstream's output when given, else from a
/// host input. False with \p error for an op the bridge does not map.
bool RigExecAddGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          RigExecRigDescBuilder *builder,
                          const RigExecGeometryOpNode *upstream,
                          RigExecGeometryOpNode *node,
                          std::string *error = nullptr);

/// Writes \p p's per-frame values into \p evaluator for \p pointCount
/// points, and \p points when \p node takes them from the host. The
/// envelope is \p p's weights resolved at \p pointCount. False, writing
/// nothing more, where the packet's envelope does not resolve.
bool RigExecWriteGeometryOpInputs(const RigExecMoverParameters &p,
                                  const RigExecGeometryOpNode &node,
                                  const std::vector<GfVec3f> *points,
                                  size_t pointCount,
                                  RigExecEvaluator *evaluator,
                                  std::string *error = nullptr);

} // namespace rigExec

#endif
