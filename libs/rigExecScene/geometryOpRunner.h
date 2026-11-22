#ifndef RIGEXEC_SCENE_GEOMETRY_OP_RUNNER_H
#define RIGEXEC_SCENE_GEOMETRY_OP_RUNNER_H

/// One scene geometry revision run as a single op node through the op
/// core. Built on first use and rebuilt when a setting baked into it moved,
/// so stale settings never run. Owner-thread state of the scene runtime.

#include "rigExec/api/evaluator.h"
#include "rigExecScene/geometryOps.h"
#include "rigExecScene/geometryProgram.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecGeometryOpRunner {
    std::unique_ptr<RigExecEvaluator> evaluator;
    RigExecGeometryOpNode node;
    RigExecGeometryMatrixNode matrix;
};

/// Loads the built-in op library. False with \p error when it cannot.
bool RigExecLoadGeometryOpCore(std::string *error);

/// Runs \p p over \p points as \p op's node; \p channels for a blend
/// shape, measured against \p base. False with \p error, \p points
/// untouched, where the op refuses.
bool RigExecRunGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          const std::vector<RigExecBlendChannel> *channels,
                          const std::vector<GfVec3f> &base,
                          RigExecGeometryOpRunner &runner,
                          std::vector<GfVec3f> &points,
                          std::string *error = nullptr);

/// The derived matrix op (surface projector, shader dials) for one frame.
/// A refused projection sets \p diagnostic to the line
/// RigExecRunGeometryMatrix would report first, or clears it when that
/// reports none.
bool RigExecRunGeometryMatrixOp(RigExecRevisionOp op,
                                const RigExecRevisionBinding &binding,
                                const RigExecSurfaceProjectorFrames &frames,
                                const RigExecProjectorReads &reads,
                                const std::vector<GfVec3f> &base,
                                const std::vector<GfVec3f> &final,
                                RigExecGeometryOpRunner &runner,
                                GfMatrix4d &matrix, std::string &diagnostic);

} // namespace rigExec

#endif
