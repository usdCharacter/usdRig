// The common-envelope rules the baked program and .rigexec playback share:
// the eligibility predicates, the exact blend over a point range, the
// envelope resolve, and the matrix kernel's dense range, sparse walk and
// whole-array dispatch. One definition per rule, so a second front end
// cannot drift into a different answer (docs/specs/eval-core-parity.md,
// envelope-sparse-lift). WHERE a predicate is consulted stays with each
// front end: the core gate and the playback gate keep their own call sites
// until the operator decides the sparse-envelope row.
//
// The bodies read a packet through EnvelopePacketView: representation and
// range policy as the integer codes weights.h already classifies to, the
// arrays borrowed from whichever packet the host owns. The matrix bodies
// are templated on a Math trait:
//
//   Math::Vec3f, Vec3d, Mat4, PartialDecomposition
//        the host's vector, double-row matrix, and radial factorization;
//        Mat4 must be the sixteen doubles the walk memo keys by, construct
//        from 1.0 for the identity, compare with ==, and carry
//        TransformAffine(Vec3d).
//   Math::ToFloat / ToDouble / Clamp
//        the float<->double widening at the kernel boundary.
//   Math::Identity / TransformBytes
//        the identity matrix; the transform's bytes for the memo key.
//   Math::DecomposePartialTransform / ApplyPartialDecomposition
//        the radial arc's per-weight factorization.
//   Math::ApplyWeightedMatrix / ApplyWeightedMatrixSimd
//        the weighted-matrix rule, scalar and SIMD; the SIMD form takes a
//        nullable weights array and applies the transform at full weight
//        where it is null.
//   Math::FloatEnvironmentIsDefault()
//        whether the host computes under round-to-nearest with no
//        flush-to-zero or denormals-are-zero, which licenses the radial
//        identity shortcut. Playback reports false and keeps its exact
//        doubles; the difference is the host's, not the body's.
#ifndef RIGEXEC_EVAL_CORE_ENVELOPE_H
#define RIGEXEC_EVAL_CORE_ENVELOPE_H

#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/weights.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/rangeRunner.h"

namespace rigExec {
namespace eval {

/// A compiled revision's envelope as the predicates read it: the codes a
/// host adapter classified once from its packet, and the packet's arrays
/// borrowed for the call.
struct EnvelopePacketView {
    bool valid = false;
    int representation = kWeightRepresentationUnknown;
    // An absent range policy is accepted by both predicates; a present one
    // the adapter did not name classifies to kWeightPolicyUnknown and is
    // not.
    bool rangePolicyEmpty = false;
    int rangePolicy = kWeightPolicyUnknown;
    float defaultWeight = 0.0f;
    const float *values = nullptr;
    size_t valueCount = 0;
    const int *indices = nullptr;
    size_t indexCount = 0;
};

/// An envelope a kernel can act on: no policy, or one both sides name.
inline bool
EnvelopePolicyIsAccepted(const EnvelopePacketView &v)
{
    return v.rangePolicyEmpty || v.rangePolicy == kWeightPolicyStrict ||
           v.rangePolicy == kWeightPolicyClamp;
}

/// The constant-1.0 packet: the kernel's output IS the revision's output,
/// and no envelope array is resolved or blended anywhere.
inline bool
EnvelopeIsFullStrength(const EnvelopePacketView &v)
{
    return v.valid && v.representation == kWeightConstant &&
           v.valueCount == 0 && v.indexCount == 0 && v.defaultWeight == 1.0f &&
           EnvelopePolicyIsAccepted(v);
}

/// The one eligibility predicate both fronts share: a valid sparse field
/// with a zero default the kernels walk entry by entry instead of
/// resolving densely.
inline bool
EnvelopeIsSparseWalk(const EnvelopePacketView &v)
{
    return v.valid && v.representation == kWeightSparse &&
           v.defaultWeight == 0.0f && v.indexCount == v.valueCount &&
           EnvelopePolicyIsAccepted(v);
}

/// Every entry a sparse walk names over \p count points: in range, strictly
/// ascending, finite and in [0, 1]. Validated exactly as ResolveAll would,
/// so the same packets fail.
inline bool
SparseEnvelopeIsUsable(const EnvelopePacketView &v, size_t count)
{
    for (size_t k = 0; k < v.indexCount; ++k) {
        const int index = v.indices[k];
        const float value = v.values[k];
        if (index < 0 || size_t(index) >= count ||
            (k > 0 && index <= v.indices[k - 1]) || !std::isfinite(value) ||
            value < 0.0f || value > 1.0f) {
            return false;
        }
    }
    return true;
}

/// The ops whose kernel blends its envelope back over the entering points
/// itself, so nothing else may blend them afterwards. \p op is a revision
/// op ordinal (kinds.h's wire enum, the ordinals both fronts use).
inline bool
RevisionTakesSeparateBlend(int op, const EnvelopePacketView &w)
{
    return !(op == int(fb::RevisionOp::Matrix) ||
             op == int(fb::RevisionOp::BlendShape) ||
             op == int(fb::RevisionOp::RecomputeNormals) ||
             op == int(fb::RevisionOp::RecomputeExtent) ||
             (op == int(fb::RevisionOp::Wire) && EnvelopeIsSparseWalk(w)));
}

/// The exact envelope blend over the point range [begin, end): each point
/// blends its candidate back toward the preceding revision through the
/// endpoint-exact rule (rigExecMath/envelope.h). Independent elements: a
/// slice of the range is the same answer however it is split.
template <class T>
void
BlendEnvelopeRange(const T *preceding, const float *envelope, size_t begin,
                   size_t end, T *blended)
{
    for (size_t i = begin; i < end; ++i) {
        blended[i] =
            RigExecBlendEnvelope(preceding[i], blended[i], envelope[i]);
    }
}

/// The blend over all \p count points, spread through the host's \p runner
/// when it passes one; the serial loop is the runner's answer.
template <class T>
void
BlendEnvelopeAll(const T *preceding, const float *envelope, size_t count,
                 T *blended, const RigExecRangeRunner *runner = nullptr)
{
    RigExecRunRanges(
        runner, count, RigExecGeometryGrainSize, [&](size_t begin, size_t end) {
            BlendEnvelopeRange(preceding, envelope, begin, end, blended);
        });
}

/// The envelope resolved at the full \p count into \p scratch, exactly as
/// the packet's own ResolveAll validates it. On success \p noteFloats moves
/// it into the host's field and updates its content version; a failed
/// resolve leaves the field unchanged either side.
template <class Packet, class NoteFloats>
bool
ResolveEnvelopeInto(const Packet &weights, size_t count,
                    std::vector<float> *field, std::vector<float> *scratch,
                    const NoteFloats &noteFloats)
{
    const bool ok = weights.ResolveAll(count, scratch);
    if (ok) {
        noteFloats(field, scratch);
    }
    return ok;
}

/// The matrix kernel over the point range [begin, end): the weighted-matrix
/// rule p' = q + w (T q - q), the envelope read per point, or full weight
/// when it is null. Reads \p in and writes \p pts, which may be the same
/// array: point i is written only after it is read. The radial arm factors
/// the transform per WEIGHT rather than per point: a painted falloff is
/// mostly a handful of distinct values.
template <class Math>
void
ApplyMatrixKernelRange(bool radial, const typename Math::Mat4 &transform,
                       const float *envelope, size_t begin, size_t end,
                       const typename Math::Vec3f *in,
                       typename Math::Vec3f *pts, bool useSimd)
{
    using Vec3f = typename Math::Vec3f;
    using Mat4 = typename Math::Mat4;
    if (radial) {
        // A fraction of the ROTATION, not of the position: the linear form
        // below follows the chord of the arc, and a chord falls inside its
        // arc, toward the axis.
        double cachedWeight = -1.0;
        typename Math::PartialDecomposition decomposition;
        bool haveDecomposition = false;
        Mat4 partial(1.0);
        bool identity = true;
        // The shortcut below writes down what the identity's arithmetic
        // returns under round-to-nearest without flush-to-zero or
        // denormals-are-zero; any other environment takes the transform.
        const bool shortcut = Math::FloatEnvironmentIsDefault();
        for (size_t i = begin; i < end; ++i) {
            const double w = envelope ? envelope[i] : 1.0;
            if (w != cachedWeight) {
                const double clamped = Math::Clamp(w, 0.0, 1.0);
                identity = clamped <= 0.0;
                if (identity)
                    partial = Mat4(1.0);
                else if (clamped >= 1.0)
                    partial = transform;
                else {
                    if (!haveDecomposition) {
                        decomposition =
                            Math::DecomposePartialTransform(transform);
                        haveDecomposition = true;
                    }
                    partial =
                        Math::ApplyPartialDecomposition(decomposition, clamped);
                }
                cachedWeight = w;
            }
            if (identity && shortcut) {
                // What the identity's TransformAffine returns for a finite
                // point, without the double round trip: each coordinate
                // unchanged, except that the "+ 0" of the translation row
                // turns -0 into +0. Non-finite points take the transform
                // (an infinity times 0 is NaN).
                const Vec3f &q = pts[i];
                if (std::isfinite(q[0]) && std::isfinite(q[1]) &&
                    std::isfinite(q[2])) {
                    pts[i] = Vec3f(q[0] == 0.0f ? 0.0f : q[0],
                                   q[1] == 0.0f ? 0.0f : q[1],
                                   q[2] == 0.0f ? 0.0f : q[2]);
                    continue;
                }
            }
            pts[i] =
                Math::ToFloat(partial.TransformAffine(Math::ToDouble(in[i])));
        }
        return;
    }
    if (useSimd) {
        Math::ApplyWeightedMatrixSimd(in + begin, pts + begin,
                                      envelope ? envelope + begin : nullptr,
                                      end - begin, transform);
    } else {
        // Element i is written only after it is read, so in-place is safe.
        for (size_t i = begin; i < end; ++i) {
            pts[i] = Math::ToFloat(Math::ApplyWeightedMatrix(
                Math::ToDouble(in[i]), transform,
                envelope ? double(envelope[i]) : 1.0));
        }
    }
}

/// The matrix kernel's validation over \p count points, the one definition
/// the kernel runs first and an acceptance query decides by: a
/// full-strength envelope, a sparse walk whose entries are usable, or a
/// dense resolve. Given \p dense, the dense resolve writes into it
/// (ResolveAll validates exactly as ResolvesAll does), so the kernel
/// validates once.
template <class Packet>
bool
MatrixKernelAccepts(const EnvelopePacketView &envelope, const Packet &weights,
                    size_t count, std::vector<float> *dense)
{
    if (EnvelopeIsFullStrength(envelope)) {
        return true;
    }
    if (EnvelopeIsSparseWalk(envelope)) {
        return SparseEnvelopeIsUsable(envelope, count);
    }
    // A cardinality mismatch fails atomically.
    return dense ? weights.ResolveAll(count, dense)
                 : weights.ResolvesAll(count);
}

/// The sparse walk's entries [first, last) applied in place to \p data,
/// which holds point \p offset at 0: the per-entry body the whole kernel
/// runs over every entry and a group over the entries whose indices fall
/// in it. Each entry writes only its own point, and the radial arm's
/// partial for a weight is a pure function of that weight and the
/// transform, so a walk over a subset of the entries writes those points'
/// bits.
///
/// An entry's result is therefore a pure function of the blend arm, the
/// transform, its weight and its entering point. Given \p memo (a group's
/// own, rigExecMath/pointBlocks.h), a walk whose arm and transform are the
/// last walk's records its entries, and an entry whose weight and entering
/// point hold the bits recorded takes the recorded result; only the others
/// are computed. The same bits either way.
template <class Math>
void
ApplyMatrixSparseWalk(
    bool radial, const typename Math::Mat4 &transform,
    const EnvelopePacketView &w, size_t first, size_t last,
    typename Math::Vec3f *data, size_t offset = 0,
    RigExecSparseWalkMemo<typename Math::Vec3f> *memo = nullptr)
{
    static_assert(sizeof(typename Math::Mat4) == 16 * sizeof(double),
                  "the transform keys by its sixteen doubles");
    using Vec3f = typename Math::Vec3f;
    using Mat4 = typename Math::Mat4;
    using Mode = RigExecSparseWalkMode;
    Mode mode = Mode::Compute;
    if (memo) {
        // The arm, then the transform's sixteen doubles.
        unsigned char params[1 + sizeof(Mat4)];
        params[0] = radial ? 1 : 0;
        std::memcpy(params + 1, Math::TransformBytes(transform), sizeof(Mat4));
        mode =
            RigExecSparseWalkBegin(memo, params, sizeof(params), last - first);
    }
    // Whether the memo answers for entry k's point; if not and the walk
    // records, its inputs are noted for the result `keep` stores.
    const auto reuse = [&](size_t k, Vec3f *point) {
        if (mode == Mode::Compute) {
            return false;
        }
        const size_t j = k - first;
        if (mode == Mode::Reuse &&
            RigExecSparseWalkHolds(*memo, j, w.indices[k], w.values[k],
                                   *point)) {
            *point = memo->out[j];
            ++memo->reused;
            return true;
        }
        RigExecSparseWalkNote(memo, j, w.indices[k], w.values[k], *point);
        return false;
    };
    const auto keep = [&](size_t k, const Vec3f &point) {
        if (mode != Mode::Compute) {
            memo->out[k - first] = point;
        }
        if (memo) {
            ++memo->computed;
        }
    };
    if (radial) {
        // The sparse walk takes the same arc the dense kernel does; a
        // painted cluster falloff is almost always sparse, so this is the
        // branch every radial cluster on a real rig reaches.
        double cachedWeight = -1.0;
        typename Math::PartialDecomposition decomposition;
        bool haveDecomposition = false;
        Mat4 partial(1.0);
        for (size_t k = first; k < last; ++k) {
            Vec3f &point = data[size_t(w.indices[k]) - offset];
            if (reuse(k, &point)) {
                continue;
            }
            const double value = w.values[k];
            if (value != cachedWeight) {
                const double clamped = Math::Clamp(value, 0.0, 1.0);
                if (clamped <= 0.0)
                    partial = Mat4(1.0);
                else if (clamped >= 1.0)
                    partial = transform;
                else {
                    if (!haveDecomposition) {
                        decomposition =
                            Math::DecomposePartialTransform(transform);
                        haveDecomposition = true;
                    }
                    partial =
                        Math::ApplyPartialDecomposition(decomposition, clamped);
                }
                cachedWeight = value;
            }
            point =
                Math::ToFloat(partial.TransformAffine(Math::ToDouble(point)));
            keep(k, point);
        }
        return;
    }
    for (size_t k = first; k < last; ++k) {
        Vec3f &point = data[size_t(w.indices[k]) - offset];
        if (reuse(k, &point)) {
            continue;
        }
        point = Math::ToFloat(Math::ApplyWeightedMatrix(
            Math::ToDouble(point), transform, w.values[k]));
        keep(k, point);
    }
}

/// The matrix kernel over the whole array: validation, then the sparse walk
/// over the entries, the full-strength range, or the dense range, in that
/// order and by these predicates only.
template <class Math, class Packet>
bool
ApplyMatrixKernel(const EnvelopePacketView &envelope, const Packet &weights,
                  bool radial, const typename Math::Mat4 &transform,
                  std::vector<typename Math::Vec3f> *pts, bool useSimd)
{
    const size_t count = pts->size();
    std::vector<float> dense;
    if (!MatrixKernelAccepts(envelope, weights, count, &dense)) {
        return false;
    }
    if (EnvelopeIsSparseWalk(envelope)) {
        if (transform == Math::Identity()) {
            return true; // at rest every weighted point maps to itself
        }
        ApplyMatrixSparseWalk<Math>(radial, transform, envelope, 0,
                                    envelope.indexCount, pts->data());
        return true;
    }
    if (EnvelopeIsFullStrength(envelope)) {
        ApplyMatrixKernelRange<Math>(radial, transform, nullptr, 0, count,
                                     pts->data(), pts->data(), useSimd);
        return true;
    }
    ApplyMatrixKernelRange<Math>(radial, transform, dense.data(), 0, count,
                                 pts->data(), pts->data(), useSimd);
    return true;
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_ENVELOPE_H
