// State a rebuilt baked program adopts from the one it replaces.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

void
RigExecBakedProgram::AdoptBlendSampleLayouts(RigExecBlendSampleCache *cache)
{
    if (!cache)
        return;
    auto &B = *_impl;
    const auto &execution = B.opExecution;
    // Only bodies which normalized current leaves may publish after the join.
    const auto adopt = [&](size_t i) {
        if (i >= B.opGraph.ops.size() || i >= execution.ran.size() ||
            !execution.ran[i])
            return;
        const auto &step = B.steps[B.opGraph.ops[i].originalIndex];
        if (step.kind != RigExecBakedStepKind::RevisionStatic ||
            step.object < 0 || size_t(step.object) >= B.revisionIndex.size())
            return;
        const auto &entry = B.revisionIndex[size_t(step.object)];
        auto &revision =
            B.chains[size_t(entry.first)].revisions[size_t(entry.second)];
        if (!revision.parameters.enabled)
            return;
        for (auto &channel : revision.blendChannels)
            for (auto &sample : channel.samples) {
                if (sample.blendShape.IsEmpty() || !sample.layout)
                    continue;
                auto retained = cache->AdoptExclusive(
                    sample.samplePath, sample.layout, sample.layoutRefused);
                if (retained)
                    sample.layout = std::move(retained);
            }
    };
    // The executor's record, ascending: AdoptExclusive is first-wins, so
    // the visit order is that of a sweep over every op.
    if (execution.touchedValid)
        for (const uint32_t i : execution.touched)
            adopt(i);
    else
        for (size_t i = 0; i < B.opGraph.ops.size() && i < execution.ran.size();
             ++i)
            adopt(i);
}

void
RigExecBakedProgram::AdoptGeometryStateFrom(RigExecBakedProgram &previous)
{
    RigExecBakedProgramImpl &B = *_impl;
    RigExecBakedProgramImpl &P = *previous._impl;
    RigExecBakedAdoptSkinOpState(&B, P);
    // One revision's run state, moved from the node the outgoing program
    // held. Everything here is a CACHE of the last run; the compiled
    // description around it comes from the compiled rig the new program was built
    // from and is left alone.
    // \p keepRun says whether the cached RESULT may be kept as well as the
    // node: a revision spliced into or out of a chain changes the point
    // stream every revision after it reads, which is why the dynamic path's
    // VdfNetwork re-executes them, so from the first divergence on the node
    // survives but its result does not.
    const auto adopt = [](RigExecBakedProgramImpl::GeomRevision *destination,
                          RigExecBakedProgramImpl::GeomRevision *source,
                          bool keepRun) {
        destination->created = false;
        // The layout the outgoing SkinTopology op held, which the new op's
        // first build of the same layout hands back: the packet the kept
        // run compares against carries that object, as the evaluator's
        // cache handed it to both programs.
        destination->layoutCandidate = source->layoutHandle
                                           ? source->layoutHandle
                                           : source->layoutCandidate;
        if (!keepRun) {
            return;
        }
        destination->resultStatus = source->resultStatus;
        destination->output = std::move(source->output);
        // Both buffers' roles, the published points' content version and
        // the baseline it is next decided against: a value the outgoing
        // program published is compared with this one's republication.
        destination->stagingOutput = std::move(source->stagingOutput);
        destination->stagingFresh = source->stagingFresh;
        destination->passedPoints = std::move(source->passedPoints);
        destination->doneVersion = source->doneVersion;
        destination->lastParameters = std::move(source->lastParameters);
        // A DERIVED revision's remembered input lives beside the packet
        // rather than inside it -- the chain's 315KB point buffer, held by
        // handle with `lastParameters.auxPoints` left empty -- so it has to
        // travel with the packet it was split from. Leaving it behind gives
        // the adopted node a `ran` that says "compare against what I last
        // saw" and an empty array to compare against, so every derived
        // revision of the rig re-executes on the first generation after any
        // edit, bumps `revisionsExecuted` and emits its diagnostic, while
        // the dynamic path -- whose graphs stood through the same edit --
        // reports none of it. Exactly the failure lastDefaultWeight below
        // describes, one field further along.
        destination->lastAuxPoints = std::move(source->lastAuxPoints);
        destination->lastStatus = source->lastStatus;
        destination->ran = source->ran;
        // The FOLDED influence table, which is the other half of the
        // comparison `ran` promises. A skin revision re-executes when one of
        // its matrices moved, and `influencesChanged` is decided by the fold
        // against the table it last wrote -- so a node that kept its `ran`
        // and lost its table compares this run's matrices against a table
        // that has never held one, reports every entry changed and runs.
        // That is the same divergence the packet fields above describe,
        // reached through the fold instead of through the packet: the
        // dynamic path's VdfNetwork keeps the buffers of the nodes it
        // reconnects, so it reports no such work.
        // Carried only where the two tables are the same shape. keepRun says
        // the revision is the same mover at the same place in the chain; it
        // does not say its binding still names the same joints, and the fold
        // writes one entry per influence slot of the NEW binding.
        if (source->influences.size() == destination->influences.size()) {
            destination->influences = std::move(source->influences);
            // And the fold's verdict on that table, which is a function of
            // the table alone. The fold is a step like any other, so the
            // first run of the new program skips it when nothing it reads
            // moved -- a blend shape's fold reads nothing at all -- and a
            // table that came across without its verdict would leave the
            // fuse reading the `false` a fresh revision starts at.
            destination->influencesValid = source->influencesValid;
        }
        // The last run's envelope scalar belongs to the cached result the
        // same way the packet does: a node whose `ran` survives a rebuild
        // must not then compare this against the zero a fresh revision
        // starts at, or every revision of the rig re-executes on the first
        // generation after any edit.
        destination->lastDefaultWeight = source->lastDefaultWeight;
        // WHICH buffer the chain's running value was in, which is as much a
        // part of the cached result as the buffer itself: a revision that
        // does not execute publishes through this indirection, and a kept
        // result with a lost source would publish the authored base. Valid
        // in the new chain because keepRun is only true below the first
        // divergence, where the two identity sequences agree position by
        // position.
        destination->currentSource = source->currentSource;
    };
    std::map<SdfPath, RigExecBakedProgramImpl::GeomChain *> outgoing;
    for (RigExecBakedProgramImpl::GeomChain &chain : P.chains) {
        outgoing.emplace(chain.target, &chain);
    }
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        const auto found = outgoing.find(chain.target);
        if (found == outgoing.end()) {
            // A target the outgoing program did not drive: every revision of
            // it is genuinely new, which is also what the dynamic path says
            // about a live graph it has just created.
            continue;
        }
        RigExecBakedProgramImpl::GeomChain &old = *found->second;
        std::map<std::pair<SdfPath, RigExecRevisionOp>,
                 RigExecBakedProgramImpl::GeomRevision *>
            retained;
        for (RigExecBakedProgramImpl::GeomRevision &revision : old.revisions) {
            retained.emplace(std::make_pair(revision.moverPath, revision.op),
                             &revision);
        }
        // The first position at which the two identity sequences disagree:
        // everything before it reads the same point stream and
        // everything from it on does not.
        size_t divergence = 0;
        while (divergence < chain.revisions.size() &&
               divergence < old.revisions.size() &&
               old.revisions[divergence].moverPath ==
                   chain.revisions[divergence].moverPath &&
               old.revisions[divergence].op == chain.revisions[divergence].op) {
            ++divergence;
        }
        const bool sameSequence = divergence == chain.revisions.size() &&
                                  divergence == old.revisions.size();
        // A chain cut into groups keeps its groups only where they are the
        // same groups: the bounds of both programs agree.
        const bool grouped =
            chain.groupBounds.size() >= 2 || old.groupBounds.size() >= 2;
        const bool sameBounds = chain.groupBounds.size() >= 2 &&
                                chain.groupBounds == old.groupBounds;
        const int chainIndex = int(&chain - B.chains.data());
        const int oldChainIndex = int(&old - P.chains.data());
        // A content id names its writer's group slot in its own program:
        // the outgoing slot of every carried writer's group, to this
        // program's slot of the same (moverPath, op, group).
        std::map<int64_t, int64_t> slots;
        std::vector<RigExecBakedProgramImpl::GeomRevision *> carried;
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[i];
            const auto node =
                retained.find(std::make_pair(revision.moverPath, revision.op));
            if (node == retained.end()) {
                continue;
            }
            RigExecBakedProgramImpl::GeomRevision &source = *node->second;
            // A grouped revision's run is its groups: kept only with the
            // same role and written groups over the same bounds, else cold.
            const bool carryGroups =
                i < divergence && sameBounds && revision.role == source.role &&
                revision.groupWritten == source.groupWritten;
            adopt(&revision, &source,
                  /* keepRun = */ i < divergence && (!grouped || carryGroups));
            if (carryGroups) {
                const int id =
                    B.chainRevisionBegin[size_t(chainIndex)] + int(i);
                const int oldId = P.chainRevisionBegin[size_t(oldChainIndex)] +
                                  int(&source - old.revisions.data());
                for (size_t g = 0; g + 1 < chain.groupBounds.size(); ++g) {
                    const int from = RigExecBakedGroupSlot(P, oldId, g);
                    const int to = RigExecBakedGroupSlot(B, id, g);
                    if (from >= 0 && to >= 0) {
                        slots[from] = to;
                    }
                }
                revision.groups = std::move(source.groups);
                // A pass-through compares bytes on its next publication:
                // the slot it shared is the outgoing program's.
                for (auto &group : revision.groups) {
                    group.passedFrom = RigExecGroupSource();
                }
                revision.groupIds = std::move(source.groupIds);
                revision.deltasVersion = source.deltasVersion;
                revision.restBaseHeld = source.restBaseHeld;
                revision.restBaseVersion = source.restBaseVersion;
                carried.push_back(&revision);
            }
            // A range-pipelined revision's own buffer holds its version,
            // whatever the outgoing node's indirection named; its first run
            // republishes every range.
            if (revision.rangeRole) {
                revision.currentSource = int(i);
            }
            retained.erase(node);
        }
        // Each carried id to this program's slot of the same writer; an id
        // whose writer was not carried names no source, so the next join
        // or fuse compares it unequal and its version moves. The base's ids
        // (slot -1 - g) need no map: the bounds are the same.
        for (RigExecBakedProgramImpl::GeomRevision *revision : carried) {
            for (RigExecGroupSource &id : revision->groupIds) {
                if (id.slot == kRigExecNoGroupSource || id.slot < 0) {
                    continue;
                }
                const auto mapped = slots.find(id.slot);
                if (mapped == slots.end()) {
                    id = RigExecGroupSource();
                } else {
                    id.slot = mapped->second;
                }
            }
        }
        // Insertion, removal and reordering rebuild the schedule; a rebind
        // only updates packets. Same rule, same words, as the dynamic walk.
        chain.scheduleDirty = !sameSequence;
        chain.lastBase = std::move(old.lastBase);
        chain.result = std::move(old.result);
        chain.baseVersion = old.baseVersion;
        chain.resultVersion = old.resultVersion;
        chain.haveResult = old.haveResult;
        // The base groups describe `lastBase`; carried with it over the same
        // bounds. `resultIds` are not: ChainStatus gathers once and compares
        // the bytes with the carried result.
        if (sameBounds) {
            chain.baseOwner = std::move(old.baseOwner);
            chain.baseGroups = std::move(old.baseGroups);
            for (auto &group : chain.baseGroups) {
                group.passedFrom = RigExecGroupSource();
            }
        }

        std::map<SdfPath, RigExecBakedProgramImpl::GeomChain::Derived *>
            outgoingDerived;
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             old.derived) {
            outgoingDerived.emplace(derived.target, &derived);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            const auto match = outgoingDerived.find(derived.target);
            if (match == outgoingDerived.end() ||
                match->second->revision.moverPath !=
                    derived.revision.moverPath ||
                match->second->revision.op != derived.revision.op) {
                continue;
            }
            adopt(&derived.revision, &match->second->revision,
                  /* keepRun = */ true);
            derived.lastBase = std::move(match->second->lastBase);
            derived.result = std::move(match->second->result);
            derived.resultVersion = match->second->resultVersion;
            derived.haveResult = match->second->haveResult;
            derived.matrix = match->second->matrix;
            derived.haveMatrix = match->second->haveMatrix;
        }
    }
}

} // namespace rigExec
