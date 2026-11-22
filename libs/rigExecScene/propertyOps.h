#ifndef RIGEXEC_SCENE_PROPERTY_OPS_H
#define RIGEXEC_SCENE_PROPERTY_OPS_H

/// Host-side bridge from a translated property mover to rigExecOps' math
/// ops: the first piece of the stage walk that emits op nodes. Not
/// installed; step 10 moves it to rigExecUsd.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecScene/propertySceneTranslation.h"

#include <string>

namespace rigExec {

/// One math op node and its host-written ports. Ports the record's kind
/// lacks keep an invalid id.
struct RigExecPropertyOpNode {
    RigExecNodeId node = 0;
    /// The descriptor's record kind: 0 float, 1 vec3f, 2 matrix.
    size_t kind = 0;
    RigExecInputId in, value, min, max, weight, pathPoints;
    RigExecOutputId out;
};

/// Adds the math op for \p descriptor's record to \p builder. The op's In
/// is fed by \p upstream's output when given, else bound as a host input.
/// The operation, curve keys and tangents and path settings go into the
/// payload, so animated keys need a rebuild; value, min, max, weight and
/// path points are inputs written per frame. False with \p error for a
/// record the domain would pass through (unknown operation) or a kind
/// mismatch with \p upstream.
bool RigExecAddPropertyOp(const RigExecScenePropertyDescriptor &descriptor,
                          RigExecRigDescBuilder *builder,
                          const RigExecPropertyOpNode *upstream,
                          RigExecPropertyOpNode *node,
                          std::string *error = nullptr);

/// Writes the resolved record's inputs and \p envelope into \p evaluator.
/// False, writing nothing, where RigExecRunProperty would pass through:
/// a non-finite value, min or max.
bool
RigExecWritePropertyOpInputs(const RigExecScenePropertyDescriptor &descriptor,
                             const RigExecPropertyOpNode &node, float envelope,
                             RigExecEvaluator *evaluator,
                             std::string *error = nullptr);

} // namespace rigExec

#endif
