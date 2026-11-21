// The one skin dispatch the baked program and .rigexec playback share:
// method, layout and transform selection, the usability halves, the
// per-range kernel, the validating whole-array kernel, the acceptance
// tri-state, and the seed-skin-blend range composition, plus the fold,
// chunk gather, layout build and group bodies both front ends run
// (docs/specs/eval-core-parity.md, skin-dispatch-lift). Adapter binding
// stays with each front end: the core classifies its method token and
// reads role facts from its leaves, playback classifies the decoded wire
// rows; neither front end keeps a second decision.
//
// Bodies are templated on the host Math trait of envelope.h's convention:
//   Math::Vec3f, Vec3d, Mat4, ScaledDualQuat
//        the host's point, wide point, matrix and split-influence types;
//        Mat4 is sixteen doubles with [row][col] access and a
//        TransformAffine(Vec3d) member, as both hosts spell it.
//   Math::ToFloat / ToDouble
//        the float<->double widening at the kernel boundary.
//   Math::NarrowSkinRows / ApplyLinearBlendSkinSimd
//        the host's narrowed rows and its float SIMD blend; the scalar
//        blend is spelled here once, in double, so the two hosts cannot
//        accumulate it in a different order. Scalar and SIMD stay
//        separate paths, as the fixtures record them.
// The dual-quaternion gather, palette and apply are the templates in
// rigExecMath/dualQuatKernel.h, instantiated over the same layout view.
#ifndef RIGEXEC_EVAL_CORE_SKIN_H
#define RIGEXEC_EVAL_CORE_SKIN_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <vector>

#include "rigExecEvalCore/envelope.h"
#include "rigExecEvalCore/host.h"
#include "rigExecMath/dualQuatKernel.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/rangeRunner.h"
#include "rigExecMath/skinAcceptance.h"

namespace rigExec {
namespace eval {

/// Floats per influence in a narrowed row table.
inline constexpr size_t kSkinRowStride = 16;

/// The skinning arithmetic a packet names, as the integer code both
/// adapters classify to once (the core from its method token, playback
/// from the decoded wire row).
enum SkinMethodCode : int {
    kSkinMethodUnknown = -1,
    kSkinMethodClassicLinear = 0,
    kSkinMethodDualQuaternion = 1
};

/// A skin revision's apply-or-fail answer: Refuses, Applies, or Deferred
/// ("the run answers"; never a hold). Ordinals match both front ends'
/// acceptance enums so an adapter maps by value.
enum class SkinAcceptanceResult : int {
    Refuses = 0,
    Applies = 1,
    Deferred = 2
};

/// Where a skin's indices, weights and element size live, as the facts
/// both adapters hold: a compiled topology handle's arrays when the rig
/// fixed the layout, else the packet's own arrays. Borrowed; the view
/// never outlives the packet or program that owns the arrays.
struct SkinPacketView {
    int method = kSkinMethodUnknown;
    bool hasTopology = false;
    bool topologyValidated = false;
    size_t topologyInfluenceCount = 0;
    const int *topologyIndices = nullptr;
    const float *topologyWeights = nullptr;
    size_t topologyIndexCount = 0;
    int topologyElementSize = 0;
    const int *indices = nullptr;
    const float *weights = nullptr;
    size_t indexCount = 0;
    size_t weightCount = 0;
    int elementSize = 0;
};

/// The layout one skin range reads: the influence table plus the arrays
/// the packet view selected, over \p pointCount points. Carries the
/// Weight/Transform accessors the dual-quaternion templates read.
template <class Math> struct SkinLayoutViewT {
    const typename Math::Mat4 *transforms = nullptr;
    size_t transformCount = 0;
    const int *indices = nullptr;
    const float *weights = nullptr;
    size_t indexCount = 0;
    size_t elementSize = 0;
    size_t pointCount = 0;

    float Weight(size_t point, size_t slot) const
    {
        return weights[point * elementSize + slot];
    }
    const typename Math::Mat4 &Transform(size_t point, size_t slot) const
    {
        return transforms[size_t(indices[point * elementSize + slot])];
    }
};

/// The influence tables one skin range skins against: the matrices, the
/// narrowed rows the SIMD blend loads (or null), and the split palette
/// (or null, to split on the spot). Both derived forms are pure
/// per-matrix functions of `transforms`.
template <class Math> struct SkinTransformsViewT {
    const typename Math::Mat4 *transforms = nullptr;
    size_t transformCount = 0;
    const float *rows = nullptr;
    const typename Math::ScaledDualQuat *palette = nullptr;
    size_t paletteSize = 0;
};

/// The view of a table alone, which is what an unchunked caller skins
/// against before its fold derives the other forms.
template <class Math>
SkinTransformsViewT<Math>
SkinTransformsOf(const typename Math::Mat4 *transforms, size_t count)
{
    SkinTransformsViewT<Math> view;
    view.transforms = transforms;
    view.transformCount = count;
    return view;
}

/// A revision's own folded table: its rows stand when the fold narrowed
/// them (a scalar fold clears them), its palette when it split.
template <class Math>
SkinTransformsViewT<Math>
SkinWholeTransforms(const typename Math::Mat4 *transforms, size_t count,
                    const float *rows, bool rowsDerived,
                    const typename Math::ScaledDualQuat *palette,
                    size_t paletteSize)
{
    SkinTransformsViewT<Math> view = SkinTransformsOf<Math>(transforms, count);
    if (rowsDerived) {
        view.rows = rows;
    }
    if (palette && paletteSize) {
        view.palette = palette;
        view.paletteSize = paletteSize;
    }
    return view;
}

/// A chunk's own table: its rows stand only on the SIMD path, as the
/// chunk views both front ends already spell them.
template <class Math>
SkinTransformsViewT<Math>
SkinChunkTransforms(const typename Math::Mat4 *transforms, size_t count,
                    const float *rows, bool rowsDerived, bool useSimd,
                    const typename Math::ScaledDualQuat *palette,
                    size_t paletteSize)
{
    return SkinWholeTransforms<Math>(
        transforms, count, rows, rowsDerived && useSimd, palette, paletteSize);
}

/// The layout \p packet and \p transforms describe over \p pointCount
/// points: the topology's arrays when the packet carries a handle, else
/// the packet's own.
template <class Math>
SkinLayoutViewT<Math>
SkinLayoutForPacket(const SkinPacketView &packet,
                    const SkinTransformsViewT<Math> &transforms,
                    size_t pointCount)
{
    SkinLayoutViewT<Math> layout;
    layout.transforms = transforms.transforms;
    layout.transformCount = transforms.transformCount;
    if (packet.hasTopology) {
        layout.indices = packet.topologyIndices;
        layout.weights = packet.topologyWeights;
        layout.indexCount = packet.topologyIndexCount;
        layout.elementSize = packet.topologyElementSize < 1
                                 ? 0
                                 : size_t(packet.topologyElementSize);
    } else {
        layout.indices = packet.indices;
        layout.weights = packet.weights;
        layout.indexCount = packet.indexCount;
        layout.elementSize =
            packet.elementSize < 1 ? 0 : size_t(packet.elementSize);
    }
    layout.pointCount = pointCount;
    return layout;
}

/// The layout half of the whole-array validation: with a handle, the
/// shape the handle proves against this run's point and influence
/// counts; without one, the element shape, index range and weights in
/// full. The matrix half is SkinTransformsAreUsable; ANDing the two is
/// the unsplit check.
inline bool
SkinLayoutIsUsable(const SkinPacketView &packet, size_t pointCount,
                   size_t transformCount)
{
    if (packet.hasTopology) {
        const size_t elementSize = packet.topologyElementSize < 1
                                       ? 0
                                       : size_t(packet.topologyElementSize);
        return packet.topologyValidated &&
               packet.topologyInfluenceCount == transformCount &&
               packet.topologyIndexCount == pointCount * elementSize;
    }
    if (packet.weightCount != packet.indexCount) {
        return false;
    }
    const size_t elementSize =
        packet.elementSize < 1 ? 0 : size_t(packet.elementSize);
    if (elementSize < 1 || transformCount == 0) {
        return false;
    }
    if (packet.indexCount != pointCount * elementSize) {
        return false;
    }
    for (size_t i = 0; i < packet.indexCount; ++i) {
        if (packet.indices[i] < 0 ||
            size_t(packet.indices[i]) >= transformCount) {
            return false;
        }
        if (!std::isfinite(packet.weights[i]) || packet.weights[i] < 0.0f) {
            return false;
        }
    }
    return true;
}

/// The matrix half: every influence finite and affine.
template <class Math>
bool
SkinTransformsAreUsable(const typename Math::Mat4 *transforms, size_t count)
{
    if (!transforms || count == 0) {
        return false;
    }
    for (size_t t = 0; t < count; ++t) {
        const typename Math::Mat4 &m = transforms[t];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(m[r][c])) {
                    return false;
                }
            }
        }
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
            return false;
        }
    }
    return true;
}

/// The validation every chunk guard and the static decision share: the
/// packet, its kind, the layout against the entering count, and the
/// envelope resolve. One definition, over facts the adapter classified.
inline bool
SkinPacketIsUsable(bool valid, bool kindIsSkin, bool layoutUsable,
                   bool envelopeOk)
{
    return valid && kindIsSkin && layoutUsable && envelopeOk;
}

/// Past a usable packet a linear blend cannot fail; a dual-quaternion
/// blend can still be degenerate at a vertex, which a Range revision
/// proves in its fold and a Whole one only by running.
inline SkinAcceptanceResult
SkinAcceptance(bool packetUsable, int method, bool rangeRole)
{
    if (!packetUsable) {
        return SkinAcceptanceResult::Refuses;
    }
    if (method == kSkinMethodClassicLinear) {
        return SkinAcceptanceResult::Applies;
    }
    if (method == kSkinMethodDualQuaternion) {
        return rangeRole ? SkinAcceptanceResult::Applies
                         : SkinAcceptanceResult::Deferred;
    }
    return SkinAcceptanceResult::Refuses;
}

/// The scalar linear blend of one point, spelled once in double:
/// p' = (1 - sum_k w_k) p + sum_k w_k (T_k p), zero weights skipped, the
/// complement held at the rest point. The SIMD form below accumulates in
/// float and stays a separate path.
template <class Math>
typename Math::Vec3d
ApplyLinearBlendSkinPoint(const typename Math::Vec3d &point,
                          const SkinLayoutViewT<Math> &layout, size_t i)
{
    using Vec3d = typename Math::Vec3d;
    Vec3d sum(0.0);
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const double w = layout.Weight(i, k);
        if (w == 0.0) {
            continue;
        }
        sum += layout.Transform(i, k).TransformAffine(point) * w;
        total += w;
    }
    return point * (1.0 - total) + sum;
}

namespace skin_detail {

template <class Math>
SkinLayoutViewT<Math>
SkinLayoutPart(const SkinLayoutViewT<Math> &layout, size_t begin, size_t end)
{
    SkinLayoutViewT<Math> part = layout;
    part.indices = layout.indices + begin * layout.elementSize;
    part.weights = layout.weights + begin * layout.elementSize;
    part.indexCount = (end - begin) * layout.elementSize;
    part.pointCount = end - begin;
    return part;
}

} // namespace skin_detail

/// One vertex range of a skin, written in place over [begin, end) of
/// \p pts: the per-point body and nothing else, no validation and no
/// split, so a chunked caller and an unchunked one deform a vertex the
/// same way. False only for a method neither kernel owns, or a
/// degenerate dual-quaternion blend.
template <class Math>
bool
ApplySkinKernelRange(const SkinPacketView &packet,
                     const SkinTransformsViewT<Math> &transforms, size_t begin,
                     size_t end, std::vector<typename Math::Vec3f> *pts,
                     bool useSimd)
{
    using Vec3f = typename Math::Vec3f;
    const SkinLayoutViewT<Math> layout =
        SkinLayoutForPacket<Math>(packet, transforms, pts->size());
    const SkinLayoutViewT<Math> part =
        skin_detail::SkinLayoutPart<Math>(layout, begin, end);
    Vec3f *const points = pts->data();
    if (packet.method == kSkinMethodClassicLinear) {
        if (useSimd) {
            Math::ApplyLinearBlendSkinSimd(points + begin, points + begin, part,
                                           transforms.rows);
        } else {
            for (size_t i = 0; i < part.pointCount; ++i) {
                points[begin + i] = Math::ToFloat(ApplyLinearBlendSkinPoint(
                    Math::ToDouble(points[begin + i]), part, i));
            }
        }
        return true;
    }
    if (packet.method == kSkinMethodDualQuaternion) {
        using ScaledDualQuat = typename Math::ScaledDualQuat;
        std::vector<ScaledDualQuat> local;
        const ScaledDualQuat *palette = transforms.palette;
        size_t paletteSize = transforms.paletteSize;
        if (!palette) {
            local = RigExecSkinDualQuatPaletteKernel<typename Math::Vec3f,
                                                     typename Math::Vec3d>(
                layout.transforms, layout.transformCount);
            palette = local.data();
            paletteSize = local.size();
        }
        return RigExecApplyDualQuatSkinKernel<typename Math::Vec3f,
                                              typename Math::Vec3d>(
            points + begin, points + begin, part, palette, paletteSize);
    }
    return false;
}

/// The whole-array kernel: the two usability halves first (the matrix
/// half only when \p checkTransforms, the fact each adapter already
/// holds for a layout its rig did not fix), then every range through
/// \p runner with the per-matrix tables derived once. A null runner
/// runs the one range here.
template <class Math>
bool
ApplySkinKernelWithTransforms(const SkinPacketView &packet,
                              const SkinTransformsViewT<Math> &transforms,
                              std::vector<typename Math::Vec3f> *pts,
                              bool useSimd, bool checkTransforms,
                              const RigExecRangeRunner *runner)
{
    using Vec3f = typename Math::Vec3f;
    const size_t count = pts->size();
    if (!SkinLayoutIsUsable(packet, count, transforms.transformCount)) {
        return false;
    }
    if (checkTransforms &&
        !SkinTransformsAreUsable<Math>(transforms.transforms,
                                       transforms.transformCount)) {
        return false;
    }
    const bool splittable = runner && *runner &&
                            packet.method != kSkinMethodUnknown &&
                            count >= RigExecGeometryParallelThreshold;
    if (!splittable) {
        return ApplySkinKernelRange<Math>(packet, transforms, 0, count, pts,
                                          useSimd);
    }
    SkinTransformsViewT<Math> shared = transforms;
    const SkinLayoutViewT<Math> layout =
        SkinLayoutForPacket<Math>(packet, transforms, count);
    std::vector<float> rows;
    std::vector<typename Math::ScaledDualQuat> palette;
    if (packet.method == kSkinMethodClassicLinear && !shared.rows &&
        layout.transforms) {
        rows.resize(layout.transformCount * kSkinRowStride);
        for (size_t t = 0; t < layout.transformCount; ++t) {
            Math::NarrowSkinRows(layout.transforms[t],
                                 &rows[t * kSkinRowStride]);
        }
        shared.rows = rows.data();
    } else if (packet.method == kSkinMethodDualQuaternion && !shared.palette) {
        palette = RigExecSkinDualQuatPaletteKernel<typename Math::Vec3f,
                                                   typename Math::Vec3d>(
            layout.transforms, layout.transformCount);
        shared.palette = palette.data();
        shared.paletteSize = palette.size();
    }
    std::atomic<bool> ok(true);
    RigExecRunRanges(runner, count, RigExecGeometryGrainSize,
                     [&](size_t rangeBegin, size_t rangeEnd) {
                         if (!ApplySkinKernelRange<Math>(packet, shared,
                                                         rangeBegin, rangeEnd,
                                                         pts, useSimd)) {
                             ok.store(false, std::memory_order_relaxed);
                         }
                     });
    return ok.load(std::memory_order_relaxed);
}

/// One vertex range of a skin revision: seed \p target's range from the
/// preceding points, deform in place, blend the envelope back over the
/// same range. \p whole runs the validating full-range kernel instead,
/// which ignores the bounds; every whole-array decision (the packet
/// check, the layout validation, the envelope's resolution) already
/// belongs to the steps that own them.
template <class Math>
bool
SkinRange(const SkinPacketView &packet, bool fullStrength,
          const float *envelope, const typename Math::Vec3f *preceding,
          const SkinTransformsViewT<Math> &view, size_t begin, size_t end,
          bool whole, bool useSimd, bool checkTransforms,
          const RigExecRangeRunner *runner,
          std::vector<typename Math::Vec3f> *target)
{
    std::vector<typename Math::Vec3f> &out = *target;
    if (end > begin && preceding) {
        std::copy(preceding + begin, preceding + end,
                  out.begin() + long(begin));
    }
    if (whole) {
        if (!ApplySkinKernelWithTransforms<Math>(packet, view, &out, useSimd,
                                                 checkTransforms, runner)) {
            return false;
        }
    } else if (!ApplySkinKernelRange<Math>(packet, view, begin, end, &out,
                                           useSimd)) {
        return false;
    }
    if (!fullStrength) {
        if (whole) {
            BlendEnvelopeAll(preceding, envelope, end, out.data(), runner);
        } else {
            BlendEnvelopeRange(preceding, envelope, begin, end, out.data());
        }
    }
    return true;
}

/// A vertex group's skin: the group's part of the layout skinned from
/// \p in into \p out (distinct buffers), then the envelope blended back
/// over the group unless at full strength. The layout's shape is the
/// only check here; the caller validated it whole.
template <class Math>
bool
ApplySkinGroupRange(const SkinPacketView &packet, bool fullStrength,
                    const float *envelope,
                    const SkinTransformsViewT<Math> &view,
                    const typename Math::Vec3f *in, size_t count, size_t begin,
                    size_t end, bool useSimd, typename Math::Vec3f *out)
{
    if (begin > end || end > count || (end > begin && (!in || !out))) {
        return false;
    }
    const size_t n = end - begin;
    const SkinLayoutViewT<Math> layout =
        SkinLayoutForPacket<Math>(packet, view, count);
    if (layout.elementSize < 1 ||
        layout.indexCount != count * layout.elementSize ||
        (!packet.hasTopology && packet.weightCount != packet.indexCount)) {
        return false;
    }
    const SkinLayoutViewT<Math> part =
        skin_detail::SkinLayoutPart<Math>(layout, begin, end);
    if (packet.method == kSkinMethodClassicLinear) {
        if (useSimd) {
            Math::ApplyLinearBlendSkinSimd(in, out, part, view.rows);
        } else {
            for (size_t i = 0; i < n; ++i) {
                out[i] = Math::ToFloat(
                    ApplyLinearBlendSkinPoint(Math::ToDouble(in[i]), part, i));
            }
        }
    } else if (packet.method == kSkinMethodDualQuaternion) {
        using ScaledDualQuat = typename Math::ScaledDualQuat;
        std::vector<ScaledDualQuat> local;
        const ScaledDualQuat *palette = view.palette;
        size_t paletteSize = view.paletteSize;
        if (!palette) {
            local = RigExecSkinDualQuatPaletteKernel<typename Math::Vec3f,
                                                     typename Math::Vec3d>(
                layout.transforms, layout.transformCount);
            palette = local.data();
            paletteSize = local.size();
        }
        if (!RigExecApplyDualQuatSkinKernel<typename Math::Vec3f,
                                            typename Math::Vec3d>(
                in, out, part, palette, paletteSize)) {
            return false;
        }
    } else {
        return false;
    }
    if (!fullStrength) {
        if (!envelope) {
            return false;
        }
        for (size_t k = 0; k < n; ++k) {
            out[k] = RigExecBlendEnvelope(in[k], out[k], envelope[begin + k]);
        }
    }
    return true;
}

/// Narrows and splits a folded table for the packet's method, once per
/// run rather than once per range: the dual-quaternion palette, or the
/// narrowed rows on the SIMD path; the other form is let go.
template <class Math>
void
FoldSkinTransformForms(int method, const typename Math::Mat4 *influences,
                       size_t count, bool useSimd, std::vector<float> *rows,
                       std::vector<typename Math::ScaledDualQuat> *palette)
{
    if (method == kSkinMethodDualQuaternion) {
        rows->clear();
        *palette = RigExecSkinDualQuatPaletteKernel<typename Math::Vec3f,
                                                    typename Math::Vec3d>(
            influences, count);
        return;
    }
    palette->clear();
    if (!useSimd) {
        rows->clear();
        return;
    }
    rows->resize(count * kSkinRowStride);
    for (size_t t = 0; t < count; ++t) {
        Math::NarrowSkinRows(influences[t], &(*rows)[t * kSkinRowStride]);
    }
}

/// Copies a chunk's own influences out of the matrices \p matrixAt
/// resolves per key position (null skips the entry, as a slot with no
/// provider does), keeping the narrowed and split forms beside them.
/// Only the key's entries move; entries outside it stay identity, and a
/// range reaches an entry only through its own vertices' indices.
/// Returns through \p keyChanged whether any entry moved.
template <class Math, class Key, class MatrixAt>
void
GatherSkinChunkTransforms(int method, const Key &key, MatrixAt &&matrixAt,
                          std::vector<typename Math::Mat4> *transforms,
                          std::vector<float> *rows,
                          std::vector<typename Math::ScaledDualQuat> *palette,
                          bool *keyChanged)
{
    using Mat4 = typename Math::Mat4;
    const bool dualQuat = method == kSkinMethodDualQuaternion;
    const size_t count = transforms->size();
    const bool splitAlready = palette->size() == count + 1;
    for (const int position : key) {
        if (position < 0 || size_t(position) >= count) {
            continue;
        }
        const Mat4 *matrix = matrixAt(position);
        if (!matrix) {
            continue;
        }
        if ((*transforms)[size_t(position)] == *matrix) {
            continue;
        }
        (*transforms)[size_t(position)] = *matrix;
        *keyChanged = true;
        if (size_t(position) * kSkinRowStride < rows->size()) {
            Math::NarrowSkinRows(*matrix,
                                 &(*rows)[size_t(position) * kSkinRowStride]);
        }
        if (splitAlready) {
            (*palette)[size_t(position)] =
                RigExecScaledDualQuatFromMatrixKernel<typename Math::Vec3f,
                                                      typename Math::Vec3d>(
                    *matrix);
        }
    }
    if (dualQuat && !splitAlready) {
        *palette = RigExecSkinDualQuatPaletteKernel<typename Math::Vec3f,
                                                    typename Math::Vec3d>(
            transforms->data(), count);
    }
}

/// A layout built from its three authored arrays: its point count when
/// the arrays form rows of the element size, and whether every entry
/// validates (shape, index range, finite non-negative weights, at least
/// one influence). The build both layout operations run; each adapter
/// wraps the result in its own handle and keeps the handle for equal
/// bits, which is binding and stays adapter-local.
struct SkinLayoutBuild {
    size_t pointCount = 0;
    bool validated = false;
};

inline SkinLayoutBuild
BuildSkinLayout(const int *indices, size_t indexCount, const float *weights,
                size_t weightCount, int elementSize, size_t influenceCount)
{
    SkinLayoutBuild build;
    if (elementSize >= 1 && indexCount == weightCount &&
        indexCount % size_t(elementSize) == 0) {
        build.pointCount = indexCount / size_t(elementSize);
    }
    if (elementSize < 1 || weightCount != indexCount ||
        indexCount % size_t(elementSize) != 0 || influenceCount == 0) {
        return build;
    }
    for (size_t i = 0; i < indexCount; ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= influenceCount) {
            return build;
        }
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f) {
            return build;
        }
    }
    build.validated = true;
    return build;
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_SKIN_H
