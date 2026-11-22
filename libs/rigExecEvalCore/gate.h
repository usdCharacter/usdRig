// The group-gate predicate shared by the baked program and playback:
// whether a gated Range packet respects the compiled written-group mask.
// Only the listed points of such an envelope weigh above 0, and each op
// below leaves a point of weight <= 0 at its entering bytes: the matrix
// and wire walks never visit it, and the lattice, linear skin and target
// blend take RigExecBlendEnvelope's `weight <= 0` branch, whatever the
// deformation computed there.
//
// Build uses packet eligibility before the partition exists; after Build/Open,
// both hosts prove each packet index against the immutable compiled mask.
// Borrowed proof views cannot outlive the program's partition.
#ifndef RIGEXEC_EVAL_CORE_GATE_H
#define RIGEXEC_EVAL_CORE_GATE_H

#include <algorithm>
#include <cstddef>

#include "rigExecEvalCore/envelope.h"
#include "rigExecEvalCore/skin.h"

namespace rigExec {
namespace eval {

/// A gated revision's facts as the predicate reads them: the enable, the
/// op as a wire RevisionOp ordinal (kinds.h), the envelope and skin
/// packet facts the adapter classified once, and the compiled written-group
/// proof. Borrowed; the views never
/// outlive the packet or program that owns the arrays.
struct GroupGateFacts {
    bool enabled = false;
    int op = 0;
    EnvelopePacketView envelope;
    int skinMethod = kSkinMethodUnknown;
    bool blendSurfaceFrame = false;
    bool regularGrid = false;
    bool allWritten = false;
    bool hasWrittenProof = false;
    const int *groupBounds = nullptr;
    size_t groupBoundCount = 0;
    const char *groupWritten = nullptr;
    size_t groupWrittenCount = 0;
    size_t pointCount = 0;
};

/// Whether the packet keeps every unwritten group's points at their
/// entering bytes. With a written proof, every listed index must lie in
/// a written group; without one, the answer is the op whitelist alone.
inline bool
GroupGateHolds(const GroupGateFacts &facts)
{
    if (!facts.enabled) {
        return false;
    }
    if (facts.allWritten) {
        return true;
    }
    if (!EnvelopeIsSparseWalk(facts.envelope)) {
        return false;
    }
    if (facts.op == int(fb::RevisionOp::Skin) &&
        facts.skinMethod != kSkinMethodClassicLinear) {
        return false;
    }
    if (facts.op == int(fb::RevisionOp::BlendShape) &&
        facts.blendSurfaceFrame) {
        return false;
    }
    if (facts.op == int(fb::RevisionOp::Lattice) && facts.regularGrid) {
        return false;
    }
    if (facts.hasWrittenProof) {
        if (facts.groupBoundCount < 2 || !facts.groupBounds ||
            !facts.groupWritten ||
            facts.groupWrittenCount != facts.groupBoundCount - 1) {
            return false;
        }
        const size_t groups = facts.groupBoundCount - 1;
        for (size_t k = 0; k < facts.envelope.indexCount; ++k) {
            const int index = facts.envelope.indices[k];
            if (index < 0 || size_t(index) >= facts.pointCount) {
                continue;
            }
            size_t g =
                size_t(std::upper_bound(
                           facts.groupBounds,
                           facts.groupBounds + facts.groupBoundCount, index) -
                       facts.groupBounds);
            g = g == 0 ? 0 : std::min(g - 1, groups - 1);
            if (!facts.groupWritten[g]) {
                return false;
            }
        }
        return true;
    }
    switch (facts.op) {
    case int(fb::RevisionOp::Matrix):
    case int(fb::RevisionOp::Wire):
    case int(fb::RevisionOp::Lattice):
    case int(fb::RevisionOp::Skin):
    case int(fb::RevisionOp::BlendShape):
    case int(fb::RevisionOp::Smooth):
    case int(fb::RevisionOp::DeltaMush):
        return true;
    default:
        return false;
    }
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_GATE_H
