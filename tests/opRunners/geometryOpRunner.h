#ifndef RIGEXEC_TESTS_GEOMETRY_OP_RUNNER_H
#define RIGEXEC_TESTS_GEOMETRY_OP_RUNNER_H

/// One geometry revision run as a single op node through the op core, for
/// tests that compare an op with its native kernel. Built on first use and
/// rebuilt when a setting baked into it moved, so stale settings never run.

#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExecScene/externalOps.h"
#include "rigExecScene/geometryOps.h"
#include "rigExecScene/geometryProgram.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// The op core's options for geometry: a point op runs as ranges of
/// the grain and from the size the per-point geometry kernels split at.
RigExecCompileOptions RigExecGeometryCompileOptions();

struct RigExecGeometryOpRunner {
    std::unique_ptr<RigExecEvaluator> evaluator;
    RigExecGeometryOpNode node;
    RigExecGeometryMatrixNode matrix;
    RigExecExternalOpNode external;
    RigExecGeometryPayloadScratchPtr payloadScratch;
    RigExecCompileOptions compile = RigExecGeometryCompileOptions();
    RigExecEvaluatorOptions evaluate;
};

/// Initializes the op registry with the built-in libraries and the op
/// plugins (RigExecEnsureSceneOpRegistry). False with \p error when it
/// cannot.
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

/// \p binding's External mover over \p points as its op's node, \p p
/// giving the envelope. A windowed mover reads \p window's ring and leaves
/// its tick's record in \p window's capture, also when the run then
/// refuses. False with \p error, \p points untouched, where the op
/// refuses.
bool RigExecRunExternalOp(const RigExecRevisionBinding &binding,
                          const RigExecMoverParameters &p,
                          const RigExecExternalOpValues &values,
                          RigExecGeometryOpRunner &runner,
                          std::vector<GfVec3f> &points,
                          std::string *error = nullptr,
                          RigExecWindowRun *window = nullptr);

/// The derived matrix op (surface projector, shader dials) for one frame.
/// False with the compile or evaluation error in \p diagnostic where the
/// op fails.
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
