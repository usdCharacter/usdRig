#include "rigExecScene/geometryOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExec {

namespace {

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

bool
Compile(RigExecRigDescBuilder &builder, RigExecGeometryOpRunner &runner,
        std::string *error)
{
    const auto desc = builder.Build(error);
    const auto program = desc ? RigExecCompile(*desc, {}, error) : nullptr;
    runner.evaluator =
        program ? RigExecEvaluator::Create(program, {}, error) : nullptr;
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

bool
RigExecLoadGeometryOpCore(std::string *error)
{
    return RigExecEnsureBuiltinRegistry(error);
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
                     std::vector<GfVec3f> &points, std::string *error)
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
    if (!ready || !runner.evaluator->Evaluate(error))
        return false;
    const auto *out = static_cast<const RigExecArray<Vec3f> *>(
        runner.evaluator->GetOutput(runner.external.out));
    if (!out || out->Size() != points.size()) {
        if (error)
            *error = "external op changed the point count";
        return false;
    }
    for (size_t i = 0; i < out->Size(); ++i) {
        const Vec3f &v = (*out)[i];
        points[i] = GfVec3f(v[0], v[1], v[2]);
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
        // The op joins the projector kernel's lines with "; " after its own
        // prefix; the native run reports the first. Only the not-a-frame-
        // provider warning can precede the failing line, which is "ray ...".
        const std::string prefix = "SurfaceProjector: ";
        const size_t at = error.find(prefix);
        std::string why =
            at == std::string::npos ? error : error.substr(at + prefix.size());
        const size_t ray = why.rfind("; ray ");
        if (ray != std::string::npos)
            why.resize(ray);
        if (why != "no frame measured")
            diagnostic = "SurfaceProjector " +
                         RigExecGeometryMoverText(binding) + ": " + why;
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
