// A point chain lowered to one per-vertex program: the layout a GPU kernel
// and the CPU judge below both interpret, op by op, for one point at a time.
//
// Every op here reads only its own point (plus per-op tables), so the whole
// chain runs as one pass with one thread per point. The layout holds what an
// epoch fixes (weights, sparse offsets, wire bases, skin layouts) and is
// uploaded once; a frame holds what moves (matrices, palettes, sample
// scales, control-point deltas, enable flags), a few kilobytes.
//
// Header-only and USD-free: the evaluators, the GPU executors and the
// shader generator share this one definition. Float arithmetic throughout,
// which is what a GPU runs; the CPU kernels it is judged against work in
// double in places, so agreement is to a stated tolerance, not to the bit.
#ifndef RIGEXEC_MATH_GPU_DEFORM_PROGRAM_H
#define RIGEXEC_MATH_GPU_DEFORM_PROGRAM_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace rigExec {

/// The op a program entry performs. Values are part of the GLSL contract
/// (shaders/gpuDeform.glsl): append, never renumber.
enum class RigExecGpuOpKind : int32_t {
    /// p' = blend(p, p M, w): one affine matrix, row-vector.
    Matrix = 1,
    /// The fractional-rotation cluster (RigExecPartialTransform).
    MatrixRadial = 2,
    /// Linear blend skinning, complement held at the entering point.
    SkinLinear = 3,
    /// Scale-aware dual quaternion skinning (RigExecApplyDualQuatSkin).
    SkinDualQuat = 4,
    /// Sparse corrective offsets times per-sample scales.
    Blend = 5,
    /// A wire as its linear map from control-point motion to point motion.
    Wire = 6,
};

/// Ints per op entry in RigExecGpuDeformLayout::ops.
inline constexpr int RigExecGpuOpStride = 8;

/// Fields of one op entry, as offsets into the entry.
enum RigExecGpuOpField : int {
    RigExecGpuOpKindField = 0,
    /// -1: full strength. Otherwise the offset of pointCount envelope
    /// weights in staticF.
    RigExecGpuOpEnvelopeField = 1,
    /// Offset of the op's block in the frame. frame[block] is 1 when the op
    /// applies this frame and 0 when it passes its points through.
    RigExecGpuOpFrameField = 2,
    /// Kind-specific offsets and counts (see the layout comment).
    RigExecGpuOpAField = 3,
    RigExecGpuOpBField = 4,
    RigExecGpuOpCField = 5,
    RigExecGpuOpDField = 6,
};

/// Floats per influence in a SkinLinear frame block: a 4x3 row-vector
/// affine matrix, rows 0..3, columns 0..2.
inline constexpr int RigExecGpuAffineFloats = 12;
/// Floats per influence in a SkinDualQuat frame block: real (w, x, y, z),
/// dual (w, x, y, z), the 3x3 row-vector stretch row-major, isRigid.
inline constexpr int RigExecGpuDualQuatFloats = 18;
/// Floats in a MatrixRadial frame block after the enable flag: the full
/// matrix (12), scale (3), axis (3), angle in radians (1), translation
/// (3), along (1), pivot (3), smallAngle (1).
inline constexpr int RigExecGpuRadialFloats = 27;

/// What an epoch fixes for one chain.
///
/// Op entries, kind by kind (a, b, c, d):
///  - Matrix: frame = [enable, affine(12)].
///  - MatrixRadial: frame = [enable, radial(27)]; the envelope picks the
///    fraction per point.
///  - SkinLinear: a = staticI offset of pointCount*c joint indices, b =
///    staticF offset of their weights, c = elementSize. frame = [enable,
///    affine(12) per influence].
///  - SkinDualQuat: a, b, c as SkinLinear, d = influence count T. frame =
///    [enable, dual quaternion(18) per influence, then one identity entry
///    at index T that takes the weight complement].
///  - Blend: a = staticI offset of pointCount+1 row starts, b = staticI
///    offset of sample ids per entry, c = staticF offset of xyz offsets per
///    entry. frame = [enable, scale per sample].
///  - Wire: a = row starts as Blend, b = control-point ids per entry, c =
///    staticF offset of one coefficient per entry. frame = [enable, xyz
///    posed-minus-rest delta per control point].
/// Row starts are absolute entry indices into the b and c tables.
struct RigExecGpuDeformLayout {
    uint32_t pointCount = 0;
    std::vector<int32_t> ops;
    std::vector<float> staticF;
    std::vector<int32_t> staticI;
    /// Floats a frame of this layout holds.
    uint32_t frameFloats = 0;
    /// Distinct per built layout, so an executor keys its uploads by it.
    uint64_t serial = 0;

    size_t OpCount() const { return ops.size() / RigExecGpuOpStride; }
};

/// What moves per frame. `layout` is shared and immutable.
struct RigExecGpuDeformFrame {
    std::shared_ptr<const RigExecGpuDeformLayout> layout;
    std::vector<float> frame;
};

/// One chain's work for a device run (RigExecGpuDeformRunner).
struct RigExecGpuDeformJob {
    /// The caller's stable name for the chain, which keys its buffers.
    uint64_t chain = 0;
    const RigExecGpuDeformFrame *frame = nullptr;
    /// The entering points, pointCount xyz triples, and a version that
    /// moves whenever their bytes do (it keys the upload).
    const float *base = nullptr;
    uint64_t baseVersion = 0;
    /// pointCount xyz triples, written by the run.
    float *out = nullptr;
    /// Set by the run: the program ran and no skin blend was degenerate.
    bool ok = false;
};

namespace gpuDeformDetail {

struct Vec3 {
    float x, y, z;
};

inline Vec3 Add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 Sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 Mul(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 Cross(Vec3 a, Vec3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

/// p M for a row-vector affine stored as 12 floats (rows 0..3, cols 0..2).
inline Vec3 Affine(const float *m, Vec3 p)
{
    return {p.x * m[0] + p.y * m[3] + p.z * m[6] + m[9],
            p.x * m[1] + p.y * m[4] + p.z * m[7] + m[10],
            p.x * m[2] + p.y * m[5] + p.z * m[8] + m[11]};
}

/// RigExecBlendEnvelope: the endpoints exactly, no arithmetic at them.
inline Vec3 Envelope(Vec3 preceding, Vec3 full, float w)
{
    if (w <= 0.0f) return preceding;
    if (w >= 1.0f) return full;
    return Add(preceding, Mul(Sub(full, preceding), w));
}

/// The fractional matrix RigExecApplyPartialDecomposition builds, applied
/// to \p p, for a clamped w in (0, 1).
inline Vec3 Radial(const float *r, float w, Vec3 p)
{
    const float *scale = r + 12;
    const Vec3 axis{r[15], r[16], r[17]};
    const float angle = r[18] * w;
    const Vec3 translation{r[19], r[20], r[21]};
    const float along = r[22];
    const Vec3 pivot{r[23], r[24], r[25]};
    const bool smallAngle = r[26] != 0.0f;
    // Row-vector rotation about `axis`: the transpose of the column form.
    const float h = 0.5f * angle;
    const float qw = std::cos(h), s = std::sin(h);
    const float qx = axis.x * s, qy = axis.y * s, qz = axis.z * s;
    float m[9] = {
        1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + qw * qz), 2 * (qx * qz - qw * qy),
        2 * (qx * qy - qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qw * qx),
        2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)};
    // scale * rotate, row-vector: row r scales by 1 + (s_r - 1) w.
    for (int row = 0; row < 3; ++row) {
        const float k = 1.0f + (scale[row] - 1.0f) * w;
        m[row * 3 + 0] *= k;
        m[row * 3 + 1] *= k;
        m[row * 3 + 2] *= k;
    }
    const auto dir = [&m](Vec3 v) {
        return Vec3{v.x * m[0] + v.y * m[3] + v.z * m[6],
                    v.x * m[1] + v.y * m[4] + v.z * m[7],
                    v.x * m[2] + v.y * m[5] + v.z * m[8]};
    };
    Vec3 t = Mul(translation, w);
    if (!smallAngle) {
        t = Add(Sub(pivot, dir(pivot)), Mul(axis, along * w));
    }
    return Add(dir(p), t);
}

struct Quat {
    float w, x, y, z;
};

inline float Dot(Quat a, Quat b)
{
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

}  // namespace gpuDeformDetail

/// A program's tables by pointer, wherever they are held: a layout and
/// frame, or a renderer's copies of the same arrays.
struct RigExecGpuDeformView {
    const int32_t *ops = nullptr;
    size_t opCount = 0;
    const float *staticF = nullptr;
    const int32_t *staticI = nullptr;
    const float *frame = nullptr;
};

inline RigExecGpuDeformView
RigExecGpuDeformViewOf(const RigExecGpuDeformFrame &frame)
{
    RigExecGpuDeformView view;
    view.ops = frame.layout->ops.data();
    view.opCount = frame.layout->OpCount();
    view.staticF = frame.layout->staticF.data();
    view.staticI = frame.layout->staticI.data();
    view.frame = frame.frame.data();
    return view;
}

/// Runs every op of \p view over point \p i entering at \p xyz, in
/// place. Returns false when a skin blend is degenerate there -- the CPU
/// kernel fails the whole op then, which a per-point pass cannot do, so the
/// caller treats the frame as failed.
inline bool
RigExecRunGpuDeformPoint(const RigExecGpuDeformView &view, size_t i,
                         float *xyz)
{
    using namespace gpuDeformDetail;
    const float *F = view.frame;
    const float *SF = view.staticF;
    const int32_t *SI = view.staticI;
    bool ok = true;
    {
        Vec3 p{xyz[0], xyz[1], xyz[2]};
        for (size_t o = 0; o < view.opCount; ++o) {
            const int32_t *op = view.ops + o * RigExecGpuOpStride;
            const float *block = F + op[RigExecGpuOpFrameField];
            if (block[0] == 0.0f) continue;
            const int32_t env = op[RigExecGpuOpEnvelopeField];
            const float w = env < 0 ? 1.0f : SF[env + i];
            const int32_t a = op[RigExecGpuOpAField];
            const int32_t b = op[RigExecGpuOpBField];
            const int32_t c = op[RigExecGpuOpCField];
            const int32_t d = op[RigExecGpuOpDField];
            switch (RigExecGpuOpKind(op[RigExecGpuOpKindField])) {
            case RigExecGpuOpKind::Matrix:
                if (w > 0.0f) p = Envelope(p, Affine(block + 1, p), w);
                break;
            case RigExecGpuOpKind::MatrixRadial:
                if (w >= 1.0f) p = Affine(block + 1, p);
                else if (w > 0.0f) p = Radial(block + 1, w, p);
                break;
            case RigExecGpuOpKind::SkinLinear: {
                if (w <= 0.0f) break;
                Vec3 sum{0, 0, 0};
                float total = 0.0f;
                for (int32_t k = 0; k < c; ++k) {
                    const float jw = SF[b + i * c + k];
                    if (jw == 0.0f) continue;
                    const float *m = block + 1 +
                        SI[a + i * c + k] * RigExecGpuAffineFloats;
                    sum = Add(sum, Mul(Affine(m, p), jw));
                    total += jw;
                }
                p = Envelope(p, Add(Mul(p, 1.0f - total), sum), w);
                break;
            }
            case RigExecGpuOpKind::SkinDualQuat: {
                if (w <= 0.0f) break;
                // The gather of RigExecApplyDualQuatSkin: the largest slot
                // first (the sign reference), the others in order, the
                // complement last against the identity entry.
                int32_t pivot = 0;
                float pivotWeight = -1.0f, total = 0.0f;
                for (int32_t k = 0; k < c; ++k) {
                    const float jw = SF[b + i * c + k];
                    if (pivotWeight < jw) { pivotWeight = jw; pivot = k; }
                    total += jw;
                }
                Quat real{0, 0, 0, 0}, dual{0, 0, 0, 0}, ref{0, 0, 0, 0};
                bool haveRef = false, allRigid = true;
                float absSum = 0.0f, wSum = 0.0f;
                float stretch[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                const auto take = [&](int32_t index, float jw) {
                    if (jw == 0.0f) return;
                    const float *q = block + 1 + index * RigExecGpuDualQuatFloats;
                    const Quat r{q[0], q[1], q[2], q[3]};
                    float sign = 1.0f;
                    if (!haveRef) { ref = r; haveRef = true; }
                    else if (Dot(r, ref) < 0.0f) sign = -1.0f;
                    const float sw = sign * jw;
                    real = {real.w + r.w * sw, real.x + r.x * sw,
                            real.y + r.y * sw, real.z + r.z * sw};
                    dual = {dual.w + q[4] * sw, dual.x + q[5] * sw,
                            dual.y + q[6] * sw, dual.z + q[7] * sw};
                    absSum += std::fabs(jw);
                    wSum += jw;
                    if (q[17] == 0.0f) allRigid = false;
                    for (int e = 0; e < 9; ++e) stretch[e] += q[8 + e] * jw;
                };
                take(SI[a + i * c + pivot], SF[b + i * c + pivot]);
                for (int32_t k = 0; k < c; ++k)
                    if (k != pivot) take(SI[a + i * c + k], SF[b + i * c + k]);
                const float complement = 1.0f - total;
                if (complement != 0.0f) take(d, complement);
                const float norm = std::sqrt(Dot(real, real));
                const float eps = 1e-9f * (absSum > 1.0f ? absSum : 1.0f);
                if (!haveRef || !(norm > eps)) { ok = false; break; }
                const float inv = 1.0f / norm;
                real = {real.w * inv, real.x * inv, real.y * inv, real.z * inv};
                dual = {dual.w * inv, dual.x * inv, dual.y * inv, dual.z * inv};
                const float rd = Dot(real, dual);
                dual = {dual.w - real.w * rd, dual.x - real.x * rd,
                        dual.y - real.y * rd, dual.z - real.z * rd};
                Vec3 q = p;
                if (!allRigid) {
                    if (!(std::fabs(wSum) > 1e-9f * (absSum > 1.0f ? absSum : 1.0f))) {
                        ok = false;
                        break;
                    }
                    const float k = 1.0f / wSum;
                    q = {p.x * stretch[0] * k + p.y * stretch[3] * k + p.z * stretch[6] * k,
                         p.x * stretch[1] * k + p.y * stretch[4] * k + p.z * stretch[7] * k,
                         p.x * stretch[2] * k + p.y * stretch[5] * k + p.z * stretch[8] * k};
                }
                // Rotate (Rodrigues) and translate t = 2 (w dv - dw v + v x dv).
                const Vec3 u{real.x, real.y, real.z};
                const Vec3 cu = Cross(u, q);
                const Vec3 rotated = Add(q, Mul(Add(Mul(cu, real.w), Cross(u, cu)), 2.0f));
                const Vec3 dv{dual.x, dual.y, dual.z};
                const Vec3 t = Mul(Add(Sub(Mul(dv, real.w), Mul(u, dual.w)), Cross(u, dv)), 2.0f);
                p = Envelope(p, Add(rotated, t), w);
                break;
            }
            case RigExecGpuOpKind::Blend: {
                if (w <= 0.0f) break;
                Vec3 delta{0, 0, 0};
                for (int32_t e = SI[a + i]; e < SI[a + i + 1]; ++e) {
                    const float s = block[1 + SI[b + e]];
                    delta = Add(delta, Mul(Vec3{SF[c + 3 * e], SF[c + 3 * e + 1],
                                                SF[c + 3 * e + 2]}, s));
                }
                p = Envelope(p, Add(p, delta), w);
                break;
            }
            case RigExecGpuOpKind::Wire: {
                if (w <= 0.0f) break;
                Vec3 delta{0, 0, 0};
                for (int32_t e = SI[a + i]; e < SI[a + i + 1]; ++e) {
                    const float *dc = block + 1 + 3 * SI[b + e];
                    delta = Add(delta, Mul(Vec3{dc[0], dc[1], dc[2]}, SF[c + e]));
                }
                p = Envelope(p, Add(p, delta), w);
                break;
            }
            }
        }
        xyz[0] = p.x;
        xyz[1] = p.y;
        xyz[2] = p.z;
    }
    return ok;
}

inline bool
RigExecRunGpuDeformPoint(const RigExecGpuDeformFrame &frame, size_t i,
                         float *xyz)
{
    return RigExecRunGpuDeformPoint(RigExecGpuDeformViewOf(frame), i, xyz);
}

/// RigExecRunGpuDeformPoint over points [begin, end) of \p points
/// (pointCount xyz triples, updated in place).
inline bool
RigExecRunGpuDeformProgram(const RigExecGpuDeformFrame &frame, float *points,
                           size_t begin, size_t end)
{
    const RigExecGpuDeformView view = RigExecGpuDeformViewOf(frame);
    bool ok = true;
    for (size_t i = begin; i < end; ++i)
        ok = RigExecRunGpuDeformPoint(view, i, points + 3 * i) && ok;
    return ok;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_GPU_DEFORM_PROGRAM_H
