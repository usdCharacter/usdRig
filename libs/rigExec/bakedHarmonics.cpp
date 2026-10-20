// RigExecHarmonicsConstraint in the native program, and the windowed-op
// bookkeeping every windowed operation shares.
//
// The constraint's step captures its target's incoming position, measured
// in rigExec:space, as this tick's packet. When it solves, the earlier
// ticks' packets (placed before the run) and this one feed
// harmonicsKernel.h, shared with the binary runtime, and the offset is added
// to the target's translation under the envelope.

#include "bakedProgramImpl.h"
#include "rigExecMath/harmonicsKernel.h"

#include "pxr/base/gf/matrix4d.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

void
RigExecBakedRunHarmonics(RigExecBakedProgramImpl &B, RigExecBakedStep *step,
                         RigExecBakedProgramImpl::Constraint &c,
                         RigExecBakedCommit &commit)
{
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    commit.recordEveryTarget = false;
    if (c.window < 0 || size_t(c.window) >= B.windowedOps.size() ||
        size_t(c.window) >= B.windowCaptures.size()) {
        step->diagnostics.push_back(c.pathText +
                                    " has no history window; constraint "
                                    "passed through");
        return;
    }
    const RigExecPointFrame input = B.fin[size_t(commit.targetRead)];
    // The measuring space's posed frame, or the rig root's.
    GfMatrix4d space(1.0);
    GfMatrix4d toSpace(1.0);
    bool spaceUsable = true;
    if (c.spaceSlot >= 0) {
        space = B.posedM[size_t(c.spaceSlot)];
        const double det = space.GetDeterminant();
        spaceUsable = std::isfinite(det) && det != 0.0;
        if (spaceUsable) toSpace = space.GetInverse();
    }
    // This tick's packet: the incoming origin in the measuring space. An
    // unusable input captures nothing, which ends the history before it.
    RigExecWindowPacketPtr captured;
    if (spaceUsable && RigExecBakedUsable(input)) {
        const GfVec3d p = c.spaceSlot >= 0 ? toSpace.Transform(input.Origin())
                                           : input.Origin();
        if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            auto packet = std::make_shared<RigExecWindowPacket>();
            packet->scalars = {p[0], p[1], p[2]};
            captured = std::move(packet);
        }
    }
    RigExecWindowPacketPtr &held = B.windowCaptures[size_t(c.window)];
    // Keep the held packet's identity when its bits did not move, so the
    // memo and the readers compare pointers first.
    if (!RigExecWindowPacketsEqual(held, captured)) held = std::move(captured);
    const RigExecBakedProgramImpl::WindowedOp &window =
        B.windowedOps[size_t(c.window)];
    // A history tick needs this op's capture; its result only when another
    // windowed op reads it.
    if (B.historyTick && !window.feedsWindow) return;
    if (!rd(c.enabled)) return;
    double weight = 1.0;
    if (!c.weightObject.IsEmpty()) {
        const auto &field = B.weightFields[size_t(c.weightField)];
        if (!field.ok || field.values.size() != 1) {
            step->diagnostics.push_back(c.pathText + ": " + field.error +
                                        "; constraint passed through");
            return;
        }
        weight = field.values[0];
    } else {
        weight = rd(c.defaultWeight);
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            step->diagnostics.push_back(
                c.pathText +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            return;
        }
    }
    if (weight <= 0.0) return;
    if (!held) {
        step->diagnostics.push_back(
            c.pathText + " could not measure " +
            (*B.pathTexts)[size_t(c.target)] +
            (spaceUsable ? "" : " in a degenerate rigExec:space") +
            "; constraint passed through");
        return;
    }
    // The usable tail of the ring, then this tick, as one position array.
    const RigExecWindowRing &ring = size_t(c.window) < B.windowRings.size()
                                        ? B.windowRings[size_t(c.window)]
                                        : RigExecWindowRing();
    size_t begin = ring.size();
    while (begin > 0 && ring[begin - 1] && ring[begin - 1]->scalars.size() == 3)
        --begin;
    std::vector<double> &positions = c.harmonicsPositions;
    positions.clear();
    for (size_t k = begin; k < ring.size(); ++k)
        positions.insert(positions.end(), ring[k]->scalars.begin(),
                         ring[k]->scalars.end());
    positions.insert(positions.end(), held->scalars.begin(),
                     held->scalars.end());
    RigExecHarmonicsParameters params;
    const auto scalar = [&](size_t k) {
        const auto &s = c.harmonicsScalars[k];
        return s.isFloat ? double(rd(s.narrow)) : rd(s.wide);
    };
    params.amplitude = scalar(0);
    params.decay = scalar(1);
    params.termination = scalar(2);
    const GfVec3d axis = rd(c.offset);
    params.axisAmplitude[0] = axis[0];
    params.axisAmplitude[1] = axis[1];
    params.axisAmplitude[2] = axis[2];
    params.matchVelocity = c.harmonicsMatchVelocity;
    if (!std::isfinite(params.amplitude) || !std::isfinite(params.decay) ||
        !std::isfinite(params.termination) || !std::isfinite(axis[0]) ||
        !std::isfinite(axis[1]) || !std::isfinite(axis[2])) {
        step->diagnostics.push_back(c.pathText +
                                    " has a non-finite input; constraint "
                                    "passed through");
        return;
    }
    double ring3[3];
    if (!RigExecHarmonicsOffset(
            c.harmonicsSettings, params,
            reinterpret_cast<const double(*)[3]>(positions.data()),
            positions.size() / 3, ring3)) {
        // Fewer than two samples: no history yet, nothing rings.
        return;
    }
    GfVec3d offset(ring3[0], ring3[1], ring3[2]);
    if (c.spaceSlot >= 0) offset = space.TransformDir(offset);
    offset *= weight;
    RigExecPointFrame candidate = input;
    for (GfVec3d &q : candidate.points) q += offset;
    if (!RigExecBakedUsable(candidate)) {
        step->diagnostics.push_back(
            c.pathText + " produced an invalid or degenerate frame for " +
            (*B.pathTexts)[size_t(c.target)] + "; constraint passed through");
        return;
    }
    commit.frames[0] = candidate;
    commit.present[0] = 1;
    commit.abandoned = false;
}

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
    if (B.windowedOps.empty()) return true;
    // Each windowed constraint's step.
    for (size_t i = 0; i < B.steps.size(); ++i) {
        const RigExecBakedStep &s = B.steps[i];
        if (s.kind != RigExecBakedStepKind::Constraint || s.object < 0 ||
            size_t(s.object) >= B.walkSteps.size()) {
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
        const auto &[chainIndex, revisionIndex] = B.revisionIndex[size_t(s.object)];
        const auto &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        if (revision.window >= 0 && size_t(revision.window) < B.windowedOps.size())
            B.windowedOps[size_t(revision.window)].step = int(i);
    }
    const size_t opCount = B.opGraph.ops.size();
    std::vector<uint32_t> opOfStep(B.steps.size(), UINT32_MAX);
    for (uint32_t c = 0; c < opCount; ++c) {
        const uint32_t original = B.opGraph.ops[c].originalIndex;
        if (original < opOfStep.size()) opOfStep[original] = c;
    }
    std::vector<uint32_t> ops(B.windowedOps.size(), UINT32_MAX);
    for (size_t w = 0; w < B.windowedOps.size(); ++w) {
        auto &op = B.windowedOps[w];
        op.op = op.step >= 0 ? opOfStep[size_t(op.step)] : UINT32_MAX;
        ops[w] = op.op;
        if (op.op < opCount) B.windowOfOp[op.op] = int32_t(w);
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
            if (w < B.windowRingSerial.size()) ++B.windowRingSerial[w];
        }
    }
}

}  // namespace rigExec
