#include "rigExecScene/geometryOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecStdTypes/stdTypes.h"

#include <string>

namespace rigExec {

namespace {

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

/// Compiles \p builder into \p runner's evaluator.
bool
Finish(RigExecRigDescBuilder *builder, RigExecGeometryOpRunner *runner)
{
    std::string error;
    auto desc = builder->Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    runner->evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    return runner->evaluator != nullptr;
}

/// Builds when needed, then writes; one rebuild when the written settings
/// moved since the build. A fresh build that still refuses latches native.
template <class Build, class Write>
bool
Prepare(RigExecGeometryOpRunner *runner, const Build &build, const Write &write)
{
    if (runner->native)
        return false;
    for (int attempt = 0; attempt < 2; ++attempt) {
        const bool fresh = !runner->evaluator;
        if (fresh) {
            RigExecRigDescBuilder builder;
            if (!build(&builder) || !Finish(&builder, runner)) {
                runner->native = true;
                return false;
            }
        }
        if (write())
            return true;
        runner->evaluator.reset();
        if (fresh) {
            runner->native = true;
            return false;
        }
    }
    return false;
}

} // namespace

bool
RigExecLoadGeometryOpCore()
{
    return RigExecEnsureBuiltinRegistry();
}

bool
RigExecRunGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                     const std::vector<RigExecBlendChannel> *channels,
                     const std::vector<GfVec3f> &base,
                     RigExecGeometryOpRunner *runner,
                     std::vector<GfVec3f> *points)
{
    const bool blend = op == RigExecRevisionOp::BlendShape && channels;
    if (!runner || !points || (!blend && !RigExecGeometryOpSupported(op)))
        return false;
    const bool ready = Prepare(
        runner,
        [&](RigExecRigDescBuilder *builder) {
            return blend ? RigExecAddGeometryBlendOp(*channels, p, builder,
                                                     nullptr, &runner->node)
                         : RigExecAddGeometryOp(op, p, builder, nullptr,
                                                &runner->node);
        },
        [&] {
            RigExecEvaluator *e = runner->evaluator.get();
            return blend ? RigExecWriteGeometryBlendInputs(
                               *channels, p, runner->node, base, points,
                               points->size(), e)
                         : RigExecWriteGeometryOpInputs(p, runner->node, points,
                                                        points->size(), e);
        });
    if (!ready || !runner->evaluator->Evaluate())
        return false;
    const auto *out = static_cast<const RigExecArray<Vec3f> *>(
        runner->evaluator->GetOutput(runner->node.out));
    if (!out)
        return false;
    points->resize(out->Size());
    for (size_t i = 0; i < out->Size(); ++i) {
        const Vec3f &v = (*out)[i];
        (*points)[i] = GfVec3f(v[0], v[1], v[2]);
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
                           RigExecGeometryOpRunner *runner, GfMatrix4d *matrix)
{
    if (!runner || !matrix || !RigExecGeometryMatrixOpSupported(op))
        return false;
    const bool ready = Prepare(
        runner,
        [&](RigExecRigDescBuilder *builder) {
            return RigExecAddGeometryMatrixOp(op, frames, reads, builder,
                                              &runner->matrix);
        },
        [&] {
            return RigExecWriteGeometryMatrixInputs(binding, frames, reads,
                                                    base, final, runner->matrix,
                                                    runner->evaluator.get());
        });
    if (!ready || !runner->evaluator->Evaluate())
        return false;
    const auto *out = static_cast<const Mat4d *>(
        runner->evaluator->GetOutput(runner->matrix.out));
    if (!out)
        return false;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            (*matrix)[r][c] = out->_mtx[r][c];
    return true;
}

} // namespace rigExec
