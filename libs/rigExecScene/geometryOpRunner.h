#ifndef RIGEXEC_SCENE_GEOMETRY_OP_RUNNER_H
#define RIGEXEC_SCENE_GEOMETRY_OP_RUNNER_H

/// One scene geometry revision run as a single op node through the op
/// core. Built on first use; rebuilt once when a write refuses the
/// settings baked at build. A revision the op core does not take runs
/// natively, and the native run decides success.

#include "rigExec/api/evaluator.h"
#include "rigExecScene/geometryOps.h"
#include "rigExecScene/geometryProgram.h"

#include <memory>
#include <vector>

namespace rigExec {

struct RigExecGeometryOpRunner {
    std::unique_ptr<RigExecEvaluator> evaluator;
    RigExecGeometryOpNode node;
    RigExecGeometryMatrixNode matrix;
    /// Set when a fresh build refused the inputs: native from then on.
    bool native = false;
};

/// Loads the built-in op library. False: run every revision natively, by
/// setting each runner's native flag.
bool RigExecLoadGeometryOpCore();

/// Runs \p p over \p points as \p op's node; \p channels for a blend
/// shape, measured against \p base. False, with \p points untouched, when
/// the op core does not take the revision.
bool RigExecRunGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          const std::vector<RigExecBlendChannel> *channels,
                          const std::vector<GfVec3f> &base,
                          RigExecGeometryOpRunner *runner,
                          std::vector<GfVec3f> *points);

/// The derived matrix op (surface projector, shader dials) for one frame.
/// False, with \p matrix untouched, when the op core does not take it.
bool RigExecRunGeometryMatrixOp(RigExecRevisionOp op,
                                const RigExecRevisionBinding &binding,
                                const RigExecSurfaceProjectorFrames &frames,
                                const RigExecProjectorReads &reads,
                                const std::vector<GfVec3f> &base,
                                const std::vector<GfVec3f> &final,
                                RigExecGeometryOpRunner *runner,
                                GfMatrix4d *matrix);

} // namespace rigExec

#endif
