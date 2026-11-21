// The one group-gate predicate the baked program and .rigexec playback
// share: whether a gated Range revision's packet keeps every point of its
// unwritten groups at its entering bytes, so those groups stand as their
// entering groups (docs/specs/eval-core-parity.md, gate-policy-lift).
// Only the listed points of such an envelope weigh above 0, and each op
// below leaves a point of weight <= 0 at its entering bytes: the matrix
// and wire walks never visit it, and the lattice, linear skin and target
// blend take RigExecBlendEnvelope's `weight <= 0` branch, whatever the
// deformation computed there.
//
// The predicate reads explicit enabled / proof / packet facts, in the
// Inputs sense of host.h: borrowed views the adapter already validated,
// never outliving the packet or program that owns them. No engine
// identity selects a path; the two front ends hold different facts, and
// the gate row stays OPEN, so each passes the facts it holds and keeps
// its current answer:
//
//   core (RigExecRevisionGateHolds, and the baked host's consultation)
//        the packet facts alone: the real lattice `regularGrid`, no
//        all-written short-circuit and no weight-index proof -- at Build
//        the written groups are computed from this answer, not before
//        it, and the run-time consultation keeps that shape.
//   playback (RrGeoGroupGateHolds)
//        the decoded written-group flags: the all-written short-circuit
//        and the proof that every listed weight index lies in a written
//        group; it never held a `regularGrid` fact for this gate, so it
//        passes false and the lattice clause cannot fire there.
//
// `enabled` is the adapter's gate enable: the core checks
// RIGEXEC_BAKED_GROUP_GATES (Build's `groupGates`) and its path-leaf
// facts before a group can stand unwritten, and playback has no disable
// knob, so both kernel-half callers pass true and the consultation
// itself stays gated by the chain's gate check. The predicate answers
// holds / does-not-hold only: a Deferred acceptance is never a hold, and
// held-result preservation, counters and the violation note stay with
// the adapters (NoteGateViolation on the core side, the bool result at
// playback).
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
/// packet facts the adapter classified once, and -- playback only -- the
/// written-group proof over the decoded flags. Borrowed; the views never
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
