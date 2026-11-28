#include "geometryOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/rangeRunner.h"
#include "rigExecMath/surfaceProjectorKernel.h"
#include "rigExecScene/opPlugins.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExec {

namespace {

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

bool
Compile(RigExecRigDescBuilder &builder, RigExecGeometryOpRunner &runner,
        std::string *error)
{
    const auto desc = builder.Build(error);
    const auto program =
        desc ? RigExecCompile(*desc, runner.compile, error) : nullptr;
    runner.evaluator =
        program ? RigExecEvaluator::Create(program, runner.evaluate, error)
                : nullptr;
    return runner.evaluator != nullptr;
}

/// Writes into the built node; builds first when there is none of \p built's
/// op, and rebuilds once when the write refused settings that moved.
template <class Add, class Write, class Stale>
bool
Ready(RigExecGeometryOpRunner &runner, bool built, const Add &add,
      const Write &write, const Stale &stale, std::string *error)
{
    if (built) {
        if (write())
            return true;
        if (!stale())
            return false;
    }
    runner.evaluator.reset();
    RigExecRigDescBuilder builder;
    return add(builder) && Compile(builder, runner, error) && write();
}

} // namespace

RigExecCompileOptions
RigExecGeometryCompileOptions()
{
    RigExecCompileOptions options;
    options.rangeGrain = RigExecGeometryGrainSize;
    options.rangeThreshold = RigExecGeometryParallelThreshold;
    return options;
}

bool
RigExecLoadGeometryOpCore(std::string *error)
{
    return RigExecEnsureSceneOpRegistry(error);
}

bool
RigExecRunGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                     const std::vector<RigExecBlendChannel> *channels,
                     const std::vector<GfVec3f> &base,
                     RigExecGeometryOpRunner &runner,
                     std::vector<GfVec3f> &points, std::string *error)
{
    if (!runner.payloadScratch)
        runner.payloadScratch = RigExecMakeGeometryPayloadScratch();
    RigExecGeometryPayloadScratch *scratch = runner.payloadScratch.get();
    const bool ready = Ready(
        runner, runner.evaluator && runner.node.op == op,
        [&](RigExecRigDescBuilder &builder) {
            return channels
                       ? RigExecAddGeometryBlendOp(*channels, p, &builder,
                                                   nullptr, &runner.node, error)
                       : RigExecAddGeometryOp(op, p, &builder, nullptr,
                                              &runner.node, error);
        },
        [&] {
            RigExecEvaluator *e = runner.evaluator.get();
            return channels
                       ? RigExecWriteGeometryBlendInputs(
                             *channels, p, runner.node, base, &points,
                             points.size(), e, error, scratch)
                       : RigExecWriteGeometryOpInputs(p, runner.node, &points,
                                                      points.size(), e, error,
                                                      scratch);
        },
        [&] {
            return RigExecGeometryOpNodeStale(op, p, channels, runner.node,
                                              scratch);
        },
        error);
    if (!ready || !runner.evaluator->Evaluate(error))
        return false;
    const auto *out = static_cast<const RigExecArray<Vec3f> *>(
        runner.evaluator->GetOutput(runner.node.out));
    if (!out || out->Size() != points.size()) {
        if (error)
            *error = "geometry op changed the point count";
        return false;
    }
    for (size_t i = 0; i < out->Size(); ++i) {
        const Vec3f &v = (*out)[i];
        points[i] = GfVec3f(v[0], v[1], v[2]);
    }
    return true;
}

bool
RigExecRunExternalOp(const RigExecRevisionBinding &binding,
                     const RigExecMoverParameters &p,
                     const RigExecExternalOpValues &values,
                     RigExecGeometryOpRunner &runner,
                     std::vector<GfVec3f> &points, std::string *error,
                     RigExecWindowRun *window)
{
    const RigExecOpHandle *op = RigExecExternalOpHandle(binding);
    const bool ready =
        op && Ready(
                  runner, runner.evaluator && runner.external.op == op,
                  [&](RigExecRigDescBuilder &builder) {
                      return RigExecAddExternalOp(binding, values, &builder,
                                                  &runner.external, error);
                  },
                  [&] {
                      return RigExecWriteExternalOpInputs(
                          binding, values, p.weights, points, runner.external,
                          runner.evaluator.get(), error);
                  },
                  [&] {
                      return RigExecExternalOpNodeStale(binding, values,
                                                        runner.external);
                  },
                  error);
    if (!ready)
        return false;
    RigExecEvaluator &evaluator = *runner.evaluator;
    if (!runner.external.windowed) {
        if (!evaluator.Evaluate(error))
            return false;
    } else {
        if (!window || !window->ring || !window->capture)
            return Fail(error, "a windowed op runs only with a window");
        if (!evaluator.SetWindowRings({*window->ring}, error))
            return false;
        if (window->captureOnly) {
            // A history tick that only records: the points pass through.
            if (!evaluator.EvaluateHistory(error))
                return false;
            const RigExecWindowRing &pushed = evaluator.GetWindowRings()[0];
            *window->capture = pushed.empty() ? nullptr : pushed.back();
            return true;
        }
        if (!evaluator.Evaluate(error))
            return false;
        *window->capture = evaluator.GetWindowCaptures()[0];
    }
    const auto *out = static_cast<const RigExecArray<Vec3f> *>(
        evaluator.GetOutput(runner.external.out));
    if (!out || out->Size() != points.size()) {
        if (error)
            *error = "external op changed the point count";
        return false;
    }
    // A plugin binder's op answers the full-strength points; the envelope
    // blends them in.
    std::vector<float> weights;
    if (runner.external.planned &&
        !p.weights.ResolveAll(points.size(), &weights))
        return Fail(error, "envelope does not resolve over the points");
    for (size_t i = 0; i < out->Size(); ++i) {
        const Vec3f &v = (*out)[i];
        const GfVec3f full(v[0], v[1], v[2]);
        points[i] = runner.external.planned
                        ? RigExecBlendEnvelope(points[i], full, weights[i])
                        : full;
    }
    return true;
}

bool
RigExecRunGeometryMatrixOp(RigExecRevisionOp op,
                           const RigExecRevisionBinding &binding,
                           const RigExecSurfaceProjectorFrames &frames,
                           const RigExecProjectorReads &reads,
                           const std::vector<GfVec3f> &base,
                           const std::vector<GfVec3f> &final,
                           RigExecGeometryOpRunner &runner, GfMatrix4d &matrix,
                           std::string &diagnostic)
{
    diagnostic.clear();
    std::string error;
    const bool ready = Ready(
        runner, runner.evaluator && runner.matrix.op == op,
        [&](RigExecRigDescBuilder &builder) {
            return RigExecAddGeometryMatrixOp(op, frames, reads, &builder,
                                              &runner.matrix, &error);
        },
        [&] {
            return RigExecWriteGeometryMatrixInputs(
                binding, frames, reads, base, final, runner.matrix,
                runner.evaluator.get(), &error);
        },
        [&] {
            return RigExecGeometryMatrixNodeStale(op, frames, reads,
                                                  runner.matrix);
        },
        &error);
    if (!ready) {
        diagnostic = error;
        return false;
    }
    if (!runner.evaluator->Evaluate(&error)) {
        diagnostic = error;
        return false;
    }
    const auto *out = static_cast<const Mat4d *>(
        runner.evaluator->GetOutput(runner.matrix.out));
    if (!out) {
        diagnostic = "geometry matrix op has no output";
        return false;
    }
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            matrix[r][c] = out->_mtx[r][c];
    return true;
}

} // namespace rigExec
