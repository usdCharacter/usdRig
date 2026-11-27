// The windowed-op bookkeeping every windowed operation shares: a harmonics
// constraint's Harmonics node and a windowed geometry revision.

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecMath/harmonicsKernel.h"

namespace rigExec {

bool
RigExecBakedSettleWindowedOps(RigExecBakedProgramImpl *program,
                              std::string *error)
{
    RigExecBakedProgramImpl &B = *program;
    B.historyMask.clear();
    B.windowCaptures.assign(B.windowedOps.size(), RigExecWindowPacketPtr());
    B.windowRings.assign(B.windowedOps.size(), RigExecWindowRing());
    B.windowRingChanged.assign(B.windowedOps.size(), 0);
    B.windowRingSerial.assign(B.windowedOps.size(), 0);
    B.windowOfOp.assign(B.opGraph.ops.size(), -1);
    B.historyTick = false;
    if (B.windowedOps.empty())
        return true;
    // Each windowed constraint's node step.
    for (size_t i = 0; i < B.steps.size(); ++i) {
        const RigExecBakedStepIdentity s =
            RigExecBakedIdentityOf(B, B.steps[i]);
        if (s.kind != RigExecBakedStepKind::Constraint || s.part >= 0 ||
            s.object < 0 || size_t(s.object) >= B.walkSteps.size()) {
            continue;
        }
        const auto &walk = B.walkSteps[size_t(s.object)];
        if (walk.solverBatch || walk.index < 0 ||
            size_t(walk.index) >= B.constraints.size()) {
            continue;
        }
        const int window = B.constraints[size_t(walk.index)].window;
        if (window >= 0 && size_t(window) < B.windowedOps.size())
            B.windowedOps[size_t(window)].step = int(i);
    }
    // Each windowed geometry revision's chunk step (a whole revision runs
    // one chunk).
    for (size_t i = 0; i < B.steps.size(); ++i) {
        const RigExecBakedStep &s = B.steps[i];
        if (s.kind != RigExecBakedStepKind::RevisionChunk || s.part != 0 ||
            s.object < 0 || size_t(s.object) >= B.revisionIndex.size()) {
            continue;
        }
        const auto &[chainIndex, revisionIndex] =
            B.revisionIndex[size_t(s.object)];
        const auto &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.window >= 0 &&
            size_t(revision.window) < B.windowedOps.size())
            B.windowedOps[size_t(revision.window)].step = int(i);
    }
    const size_t opCount = B.opGraph.ops.size();
    std::vector<uint32_t> opOfStep(B.steps.size(), UINT32_MAX);
    for (uint32_t c = 0; c < opCount; ++c) {
        const uint32_t original = B.opGraph.ops[c].originalIndex;
        if (original < opOfStep.size())
            opOfStep[original] = c;
    }
    std::vector<uint32_t> ops(B.windowedOps.size(), UINT32_MAX);
    for (size_t w = 0; w < B.windowedOps.size(); ++w) {
        auto &op = B.windowedOps[w];
        op.op = op.step >= 0 ? opOfStep[size_t(op.step)] : UINT32_MAX;
        ops[w] = op.op;
        if (op.op < opCount)
            B.windowOfOp[op.op] = int32_t(w);
    }
    // Every op a windowed op depends on, through the compiled predecessors
    // (shared with the runtime).
    RigExecWindowLayout layout;
    RigExecSettleWindowLayout(B.opGraph, ops, &layout);
    B.historyMask = std::move(layout.mask);
    for (size_t w = 0; w < B.windowedOps.size(); ++w) {
        B.windowedOps[w].inner = std::move(layout.inner[w]);
        B.windowedOps[w].feedsWindow = layout.feedsWindow[w] != 0;
    }
    RigExecBakedSelectWindowDevices(&B);
    (void)error;
    return true;
}

void
RigExecBakedPlaceWindowRings(RigExecBakedProgramImpl *program,
                             const std::vector<RigExecWindowRing> &rings)
{
    RigExecBakedProgramImpl &B = *program;
    for (size_t w = 0; w < B.windowedOps.size(); ++w) {
        const RigExecWindowRing &next =
            w < rings.size() ? rings[w] : RigExecWindowRing();
        if (!RigExecWindowRingsEqual(B.windowRings[w], next)) {
            B.windowRings[w] = next;
            B.windowRingChanged[w] = 1;
            if (w < B.windowRingSerial.size())
                ++B.windowRingSerial[w];
        }
    }
}

} // namespace rigExec
