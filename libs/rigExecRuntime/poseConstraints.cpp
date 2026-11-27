// Runtime constraints and their op nodes' lookups and stores.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace runtimePoseDetail {

bool _RrRunConstraintStep(RrProgram *program, size_t step, std::string *error);

bool
_RrRunConstraintStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    const int walkIndex = program->poses->walkSteps[size_t(wire.object)].index;
    if (walkIndex < 0 ||
        size_t(walkIndex) >= program->poses->constraints.size() ||
        size_t(walkIndex) >= scratch->weightScratch.size() ||
        size_t(walkIndex) >= scratch->weightError.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no constraint";
        }
        return false;
    }
    const size_t ci = size_t(walkIndex);
    const RigExecWireConstraint &c = program->poses->constraints[ci];
    if (ci < store.matchValid.size())
        store.matchValid[ci] = 0;
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), 0);

    const int deltaBase = c.deltaBase;
    if (size_t(wire.object) >= scratch->recordAfter.size() ||
        size_t(wire.object) >= scratch->recordEveryTarget.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    scratch->recordAfter[size_t(wire.object)] = 1;
    scratch->recordEveryTarget[size_t(wire.object)] = 1;
    if (deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no delta path";
            }
            return false;
        }
        scratch->deltaPresent[size_t(deltaBase)] = 0;
        // Published for the geometry fold (the store pair mirrors the
        // baked B.deltaValues/deltaPresent the fold consumes).
        store.deltaPresent[size_t(deltaBase)] = 0;
    }
    if (ci < store.matchShownMask.size())
        store.matchShownMask[ci] = 0;
    RrPoseHost host(program, step);
    eval::StepRef ref;
    ref.kind = uint32_t(wire.kind);
    ref.object = wire.object;
    ref.part = wire.part;
    ref.slot = step;
    const eval::StepResult result = eval::RunValueStep(host, ref);
    if (result.outcome != eval::Outcome::Applied) {
        if (error) {
            const char *what = " names no commit";
            if (result.detail == eval::pose_detail::kPoseDetailNoConstraint) {
                what = " names an incomplete IK/FK match";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoChain) {
                what = " names no chain target";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoRest) {
                what = " names no rest slot";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoPole) {
                what = " names no pole object";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoVersion) {
                what = " names no fin version";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoDeltaPath) {
                what = " names no delta path";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoCandidate) {
                what = " names no candidate slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoTarget) {
                what = " names no target slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoArrays) {
                what = " names no constraint arrays";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoSource) {
                what = " names no constraint source";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSpace) {
                what = " names no space slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoWindow) {
                what = " names no harmonics window";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSlot) {
                what = " names no slot";
            }
            *error = _RrStepHead(program, step) + what;
        }
        return false;
    }
    return true;
}

} // namespace runtimePoseDetail

bool
RrLookUpConstraint(RrProgram *program, eval::ConstraintStepCache &cache,
                   const eval::OpLookup &lookup, const RigExecTypeDesc &type,
                   void *value, std::string *error)
{
    RrPoseHost host(program);
    return eval::LookUpConstraint(host, cache, lookup, type, value, error);
}

bool
RrStoreConstraint(RrProgram *program, size_t step,
                  eval::ConstraintStepCache &cache,
                  const eval::OpHostOutput &output, const void *value,
                  std::string *error)
{
    RrPoseHost host(program, step);
    return eval::StoreConstraint(host, cache, output, value, error);
}

std::string
RrConstraintFailure(const RrProgram *program, uint32_t commit,
                    const std::string &error)
{
    const RrProgram &p = *program;
    if (commit >= p.poses->commits.size())
        return error;
    const std::string mover = p.TextOrEmpty(p.poses->commits[commit].moverPath);
    return mover.empty() ? error : mover + ": " + error;
}

} // namespace rigExec
