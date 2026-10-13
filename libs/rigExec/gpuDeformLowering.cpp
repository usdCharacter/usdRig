#include "rigExec/gpuDeformLowering.h"

#include "rigExec/moverGraph.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/geometryKernels.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>

namespace rigExec {

namespace {

std::atomic<uint64_t> _layoutSerial{0};

bool
_Fail(std::string *why, const char *text)
{
    if (why) *why = text;
    return false;
}

void
_PutAffine(const GfMatrix4d &m, float *out)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c) out[r * 3 + c] = float(m[r][c]);
}

void
_PutDualQuat(const RigExecScaledDualQuat &q, float *out)
{
    const GfQuatd &real = q.rigid.real;
    const GfQuatd &dual = q.rigid.dual;
    out[0] = float(real.GetReal());
    out[1] = float(real.GetImaginary()[0]);
    out[2] = float(real.GetImaginary()[1]);
    out[3] = float(real.GetImaginary()[2]);
    out[4] = float(dual.GetReal());
    out[5] = float(dual.GetImaginary()[0]);
    out[6] = float(dual.GetImaginary()[1]);
    out[7] = float(dual.GetImaginary()[2]);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) out[8 + r * 3 + c] = float(q.stretch[r][c]);
    out[17] = q.isRigid ? 1.0f : 0.0f;
}

const std::vector<GfMatrix4d> &
_Influences(const RigExecGpuRevisionInput &in)
{
    return in.influences ? *in.influences : in.parameters->skinTransforms;
}

/// The blend sum's per-sample scales (RigExecSumBlendChannels' lo/hi pick
/// and lerp, and its structural checks). False where that sum fails.
bool
_BlendScales(const std::vector<RigExecBlendChannel> &channels,
             size_t pointCount, float *scales)
{
    size_t id = 0;
    for (const RigExecBlendChannel &channel : channels) {
        if (channel.samples.empty() || !std::isfinite(channel.weight)) {
            return false;
        }
        for (size_t k = 0; k < channel.samples.size(); ++k) {
            const RigExecBlendSampleData &sample = channel.samples[k];
            const bool shapeOk = sample.layout && sample.layout->valid &&
                                 sample.layout->pointCount == pointCount &&
                                 sample.PointCount() == 0;
            if (!std::isfinite(sample.activation) || sample.activation <= 0 ||
                !shapeOk ||
                (k > 0 && sample.activation == channel.samples[k - 1].activation)) {
                return false;
            }
            scales[id + k] = 0.0f;
        }
        const float w = std::min(std::max(channel.weight, 0.0f),
                                 channel.samples.back().activation);
        if (w != 0.0f) {
            size_t hi = 0;
            while (hi < channel.samples.size() &&
                   channel.samples[hi].activation < w) {
                ++hi;
            }
            if (hi >= channel.samples.size()) hi = channel.samples.size() - 1;
            const float aHi = channel.samples[hi].activation;
            const float aLo = hi > 0 ? channel.samples[hi - 1].activation : 0.0f;
            const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
            if (hi > 0) scales[id + hi - 1] = 1.0f - t;
            scales[id + hi] = t;
        }
        id += channel.samples.size();
    }
    return true;
}

size_t
_SampleCount(const std::vector<RigExecBlendChannel> &channels)
{
    size_t n = 0;
    for (const RigExecBlendChannel &c : channels) n += c.samples.size();
    return n;
}

}  // namespace

/// What one op's layout entry was built from. Equal keys build equal
/// entries, so a frame whose keys all hold reuses the layout.
struct RigExecGpuDeformLowering::_Key {
    RigExecRevisionOp op = RigExecRevisionOp::Matrix;
    RigExecGpuOpKind kind = RigExecGpuOpKind::Matrix;
    RigExecWeightPacket weights;
    int weightsObject = -1;
    uint64_t weightsRevision = 0;
    /// The static half of the entry was buildable: a packet whose envelope,
    /// layout or basis the CPU kernel refuses fails every frame alike.
    bool staticOk = true;
    // Skin.
    std::shared_ptr<const RigExecSkinTopology> topology;
    std::vector<int> skinIndices;
    std::vector<float> skinWeights;
    int elementSize = 0;
    size_t influenceCount = 0;
    // Blend.
    std::vector<std::shared_ptr<const RigExecBlendSampleLayout>> layouts;
    std::vector<size_t> channelSizes;
    // Wire.
    VtArray<GfVec2f> bindCoords;
    std::vector<double> knots;
    int order = 0;
    size_t controlCount = 0;
    double dropoff = 0.0;

    bool operator==(const _Key &o) const {
        return op == o.op && kind == o.kind && weights == o.weights &&
               topology == o.topology && skinIndices == o.skinIndices &&
               skinWeights == o.skinWeights && elementSize == o.elementSize &&
               influenceCount == o.influenceCount && layouts == o.layouts &&
               channelSizes == o.channelSizes &&
               bindCoords == o.bindCoords && knots == o.knots &&
               order == o.order && controlCount == o.controlCount &&
               std::memcmp(&dropoff, &o.dropoff, sizeof(double)) == 0;
    }
};

bool
RigExecGpuDeformSupports(RigExecRevisionOp op, const RigExecMoverParameters &p,
                         const std::vector<RigExecBlendChannel> *channels,
                         std::string *why)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return true;
    case RigExecRevisionOp::Skin:
        if (p.skinningMethod != "classicLinear" &&
            p.skinningMethod != "dualQuaternion") {
            return _Fail(why, "skin method");
        }
        return true;
    case RigExecRevisionOp::BlendShape:
        if (p.blendSurfaceFrame) return _Fail(why, "surface-frame blend");
        if (!channels) return _Fail(why, "blend channels not captured");
        for (const RigExecBlendChannel &c : *channels)
            for (const RigExecBlendSampleData &s : c.samples)
                if (!s.layout) return _Fail(why, "dense blend sample");
        return true;
    case RigExecRevisionOp::Wire:
        return true;
    default:
        return _Fail(why, "op has no GPU program entry");
    }
}

RigExecGpuDeformMode
RigExecGpuDeformModeFromEnv()
{
    const std::string mode = TfGetenv("RIGEXEC_GPU_DEFORM");
    if (mode == "readback" || mode == "gpu") return RigExecGpuDeformMode::Readback;
    if (mode == "shader") return RigExecGpuDeformMode::Shader;
    return RigExecGpuDeformMode::Off;
}

namespace {
std::atomic<RigExecGpuDeformRunner> _runner{nullptr};
}  // namespace

void
RigExecSetGpuDeformRunner(RigExecGpuDeformRunner runner)
{
    _runner.store(runner);
}

RigExecGpuDeformRunner
RigExecGetGpuDeformRunner()
{
    return _runner.load();
}

bool
RigExecBlendChannelsAreUsable(const std::vector<RigExecBlendChannel> &channels,
                              size_t pointCount)
{
    std::vector<float> scales(_SampleCount(channels));
    return _BlendScales(channels, pointCount, scales.data());
}

std::vector<int32_t>
RigExecGpuDeformExtentSample(const RigExecGpuDeformLayout &layout,
                             const GfVec3f *base, size_t count)
{
    std::vector<int32_t> sample;
    if (!base || count != layout.pointCount || !count) return sample;
    // A small mesh is measured whole: its few points cost less than a
    // sample that misses the one a cluster pushed out.
    if (count <= 4096) {
        sample.resize(count);
        for (size_t i = 0; i < count; ++i) sample[i] = int32_t(i);
        return sample;
    }
    // The region of each point: its heaviest influence in the last skin op.
    std::vector<int32_t> region(count, 0);
    int32_t regions = 1;
    for (size_t o = layout.OpCount(); o-- > 0;) {
        const int32_t *op = layout.ops.data() + o * RigExecGpuOpStride;
        const RigExecGpuOpKind kind = RigExecGpuOpKind(op[RigExecGpuOpKindField]);
        if (kind != RigExecGpuOpKind::SkinLinear &&
            kind != RigExecGpuOpKind::SkinDualQuat) continue;
        const int32_t a = op[RigExecGpuOpAField], b = op[RigExecGpuOpBField];
        const int32_t k = op[RigExecGpuOpCField];
        if (k < 1) break;
        regions = op[RigExecGpuOpDField] + 1;
        for (size_t i = 0; i < count; ++i) {
            float best = -1.0f;
            for (int32_t slot = 0; slot < k; ++slot) {
                const float w = layout.staticF[size_t(b) + i * size_t(k) + size_t(slot)];
                if (w > best) {
                    best = w;
                    region[i] = 1 + layout.staticI[size_t(a) + i * size_t(k) + size_t(slot)];
                }
            }
        }
        break;
    }
    // Six extremes per region.
    std::vector<int32_t> extreme(size_t(regions) * 6, -1);
    for (size_t i = 0; i < count; ++i) {
        const int32_t r = region[i] >= 0 && region[i] < regions ? region[i] : 0;
        for (int axis = 0; axis < 3; ++axis) {
            int32_t &lo = extreme[size_t(r) * 6 + size_t(axis) * 2];
            int32_t &hi = extreme[size_t(r) * 6 + size_t(axis) * 2 + 1];
            if (lo < 0 || base[i][axis] < base[size_t(lo)][axis]) lo = int32_t(i);
            if (hi < 0 || base[i][axis] > base[size_t(hi)][axis]) hi = int32_t(i);
        }
    }
    for (const int32_t index : extreme)
        if (index >= 0) sample.push_back(index);
    for (size_t i = 0; i < count; i += 64) sample.push_back(int32_t(i));
    std::sort(sample.begin(), sample.end());
    sample.erase(std::unique(sample.begin(), sample.end()), sample.end());
    return sample;
}

bool
RigExecGpuDeformSampleBounds(const RigExecGpuDeformFrame &frame,
                             const GfVec3f *base,
                             const std::vector<int32_t> &sample,
                             size_t begin, size_t end,
                             GfVec3f *min, GfVec3f *max)
{
    if (!frame.layout || !base || !min || !max) return false;
    end = std::min(end, sample.size());
    for (size_t k = begin; k < end; ++k) {
        const int32_t index = sample[k];
        float xyz[3] = {base[index][0], base[index][1], base[index][2]};
        if (!RigExecRunGpuDeformPoint(frame, size_t(index), xyz)) return false;
        for (int axis = 0; axis < 3; ++axis) {
            (*min)[axis] = std::min((*min)[axis], xyz[axis]);
            (*max)[axis] = std::max((*max)[axis], xyz[axis]);
        }
    }
    return true;
}

void
RigExecGpuDeformPadExtent(GfVec3f *min, GfVec3f *max)
{
    const float pad = 0.02f * (*max - *min).GetLength();
    *min -= GfVec3f(pad);
    *max += GfVec3f(pad);
}

RigExecGpuDeformLowering::RigExecGpuDeformLowering() = default;
RigExecGpuDeformLowering::~RigExecGpuDeformLowering() = default;
RigExecGpuDeformLowering::RigExecGpuDeformLowering(
    const RigExecGpuDeformLowering &) = default;
RigExecGpuDeformLowering &RigExecGpuDeformLowering::operator=(
    const RigExecGpuDeformLowering &) = default;

bool
RigExecGpuDeformLowering::Lower(
    const std::vector<RigExecGpuRevisionInput> &revisions, size_t pointCount,
    RigExecGpuDeformFrame *frame, std::string *why)
{
    if (!frame) return false;
    // The steady state: every stored key still holds, compared in place
    // so a frame copies no packet.
    bool holds = _layout && pointCount == _pointCount &&
                 _keys.size() == revisions.size();
    for (size_t r = 0; holds && r < revisions.size(); ++r) {
        const RigExecGpuRevisionInput &in = revisions[r];
        if (!in.parameters ||
            !RigExecGpuDeformSupports(in.op, *in.parameters, in.channels, why))
            return false;
        holds = _KeyHolds(_keys[r], in);
    }
    if (holds) {
        _FillFrame(revisions, frame);
        return true;
    }
    // Keys first: a changed key anywhere rebuilds the whole layout, since
    // every later op's offsets move with it.
    std::vector<_Key> keys(revisions.size());
    for (size_t r = 0; r < revisions.size(); ++r) {
        const RigExecGpuRevisionInput &in = revisions[r];
        if (!in.parameters) return _Fail(why, "revision without a packet");
        const RigExecMoverParameters &p = *in.parameters;
        if (!RigExecGpuDeformSupports(in.op, p, in.channels, why)) return false;
        _Key &key = keys[r];
        key.op = in.op;
        key.weights = p.weights;
        key.weightsObject = in.weightsObject;
        key.weightsRevision = in.weightsRevision;
        switch (in.op) {
        case RigExecRevisionOp::Matrix:
            key.kind = p.radialWeight ? RigExecGpuOpKind::MatrixRadial
                                      : RigExecGpuOpKind::Matrix;
            break;
        case RigExecRevisionOp::Skin:
            key.kind = p.skinningMethod == "dualQuaternion"
                           ? RigExecGpuOpKind::SkinDualQuat
                           : RigExecGpuOpKind::SkinLinear;
            key.topology = p.skinTopology;
            if (!key.topology) {
                key.skinIndices = p.skinIndices;
                key.skinWeights = p.skinWeights;
                key.elementSize = p.skinElementSize;
            }
            key.influenceCount = _Influences(in).size();
            break;
        case RigExecRevisionOp::BlendShape:
            key.kind = RigExecGpuOpKind::Blend;
            for (const RigExecBlendChannel &c : *in.channels) {
                key.channelSizes.push_back(c.samples.size());
                for (const RigExecBlendSampleData &s : c.samples)
                    key.layouts.push_back(s.layout);
            }
            break;
        case RigExecRevisionOp::Wire:
            key.kind = RigExecGpuOpKind::Wire;
            key.bindCoords = p.wireBindCoords;
            key.knots = p.curveKnots;
            key.order = p.curveOrder;
            key.controlCount = p.restPoints.size();
            key.dropoff = p.dropoffDistance;
            break;
        default:
            return _Fail(why, "op has no GPU program entry");
        }
    }

    if (!_layout || pointCount != _pointCount || keys != _keys) {
        auto layout = std::make_shared<RigExecGpuDeformLayout>();
        layout->pointCount = uint32_t(pointCount);
        layout->ops.assign(revisions.size() * RigExecGpuOpStride, 0);
        uint32_t frameFloats = 0;
        for (size_t r = 0; r < revisions.size(); ++r) {
            const RigExecGpuRevisionInput &in = revisions[r];
            const RigExecMoverParameters &p = *in.parameters;
            _Key &key = keys[r];
            int32_t *op = layout->ops.data() + r * RigExecGpuOpStride;
            op[RigExecGpuOpKindField] = int32_t(key.kind);
            op[RigExecGpuOpFrameField] = int32_t(frameFloats);
            // The envelope, resolved once: -1 at constant full strength.
            std::vector<float> envelope;
            op[RigExecGpuOpEnvelopeField] = -1;
            if (!RigExecEnvelopeIsFullStrength(p.weights)) {
                if (p.weights.ResolveAll(pointCount, &envelope)) {
                    op[RigExecGpuOpEnvelopeField] =
                        int32_t(layout->staticF.size());
                    layout->staticF.insert(layout->staticF.end(),
                                           envelope.begin(), envelope.end());
                } else {
                    key.staticOk = false;
                }
            }
            uint32_t blockFloats = 1;
            switch (key.kind) {
            case RigExecGpuOpKind::Matrix:
                blockFloats += RigExecGpuAffineFloats;
                break;
            case RigExecGpuOpKind::MatrixRadial:
                blockFloats += RigExecGpuRadialFloats;
                break;
            case RigExecGpuOpKind::SkinLinear:
            case RigExecGpuOpKind::SkinDualQuat: {
                const std::vector<int> &indices =
                    key.topology ? key.topology->indices : key.skinIndices;
                const std::vector<float> &weights =
                    key.topology ? key.topology->weights : key.skinWeights;
                const int k =
                    key.topology ? key.topology->elementSize : key.elementSize;
                const bool usable = k >= 1 &&
                    indices.size() == pointCount * size_t(k) &&
                    weights.size() == indices.size() &&
                    RigExecSkinLayoutIsUsable(p, pointCount);
                if (!usable) key.staticOk = false;
                op[RigExecGpuOpAField] = int32_t(layout->staticI.size());
                op[RigExecGpuOpBField] = int32_t(layout->staticF.size());
                op[RigExecGpuOpCField] = usable ? k : 0;
                op[RigExecGpuOpDField] = int32_t(key.influenceCount);
                if (usable) {
                    layout->staticI.insert(layout->staticI.end(),
                                           indices.begin(), indices.end());
                    layout->staticF.insert(layout->staticF.end(),
                                           weights.begin(), weights.end());
                }
                const int per = key.kind == RigExecGpuOpKind::SkinLinear
                                    ? RigExecGpuAffineFloats
                                    : RigExecGpuDualQuatFloats;
                // The dual quaternion block ends with the identity entry
                // the weight complement blends against.
                blockFloats += uint32_t(per) *
                    uint32_t(key.influenceCount +
                             (key.kind == RigExecGpuOpKind::SkinDualQuat));
                break;
            }
            case RigExecGpuOpKind::Blend: {
                // Per point, the entries of every sample in channel order,
                // samples in activation order: the order the CPU sum adds
                // them in at that point.
                std::vector<int32_t> counts(pointCount + 1, 0);
                for (const auto &layoutPtr : key.layouts) {
                    const RigExecBlendSampleLayout &sl = *layoutPtr;
                    if (sl.indices.empty()) {
                        for (size_t i = 0; i < pointCount; ++i) ++counts[i + 1];
                    } else {
                        for (int idx : sl.indices)
                            if (idx >= 0 && size_t(idx) < pointCount)
                                ++counts[size_t(idx) + 1];
                    }
                }
                for (size_t i = 0; i < pointCount; ++i) counts[i + 1] += counts[i];
                const int32_t entries = counts[pointCount];
                const int32_t rows = int32_t(layout->staticI.size());
                const int32_t ids = rows + int32_t(pointCount) + 1;
                const int32_t offs = int32_t(layout->staticF.size());
                // Row starts are absolute entry indices into the id table.
                layout->staticI.resize(size_t(ids) + size_t(entries));
                layout->staticF.resize(size_t(offs) + 3 * size_t(entries));
                for (size_t i = 0; i <= pointCount; ++i)
                    layout->staticI[size_t(rows) + i] = ids + counts[i];
                std::vector<int32_t> fill(counts.begin(), counts.end() - 1);
                const auto put = [&](size_t point, int32_t sample,
                                     const GfVec3f &offset) {
                    const int32_t e = fill[point]++;
                    layout->staticI[size_t(ids + e)] = sample;
                    float *o = layout->staticF.data() + offs + 3 * e;
                    o[0] = offset[0]; o[1] = offset[1]; o[2] = offset[2];
                };
                int32_t sample = 0;
                for (const auto &layoutPtr : key.layouts) {
                    const RigExecBlendSampleLayout &sl = *layoutPtr;
                    if (sl.indices.empty()) {
                        for (size_t i = 0; i < pointCount && i < sl.offsets.size(); ++i)
                            put(i, sample, sl.offsets[i]);
                    } else {
                        for (size_t k = 0; k < sl.indices.size(); ++k) {
                            const int idx = sl.indices[k];
                            if (idx >= 0 && size_t(idx) < pointCount)
                                put(size_t(idx), sample, sl.offsets[k]);
                        }
                    }
                    ++sample;
                }
                // The entry stores ids relative to the id table's start, so
                // the kernel reads staticI[b + e] with e absolute.
                op[RigExecGpuOpAField] = rows;
                op[RigExecGpuOpBField] = 0;
                op[RigExecGpuOpCField] = offs - ids * 3;
                blockFloats += uint32_t(key.layouts.size());
                break;
            }
            case RigExecGpuOpKind::Wire: {
                // The weighted points: every point at full strength or a
                // dense field, the field's non-zero points otherwise.
                // A sparse bind table is parallel to the field's indices.
                std::vector<int> indices;
                if (key.bindCoords.size() != pointCount &&
                    key.bindCoords.size() == p.weights.indices.size()) {
                    indices = p.weights.indices;
                } else {
                    indices.reserve(pointCount);
                    for (size_t i = 0; i < pointCount; ++i)
                        if (envelope.empty() || envelope[i] > 0.0f)
                            indices.push_back(int(i));
                }
                RigExecWireBasis basis;
                const bool built =
                    key.staticOk &&
                    RigExecBuildWireBasis(
                        key.bindCoords.cdata(), key.bindCoords.size(),
                        pointCount, indices, key.order, key.knots,
                        key.controlCount, key.dropoff, &basis);
                if (!built) key.staticOk = false;
                std::vector<int32_t> counts(pointCount + 1, 0);
                if (built) {
                    for (const auto &column : basis.byControlPoint)
                        for (const auto &[k, coeff] : column)
                            ++counts[size_t(indices[k]) + 1];
                }
                for (size_t i = 0; i < pointCount; ++i) counts[i + 1] += counts[i];
                const int32_t entries = counts[pointCount];
                const int32_t rows = int32_t(layout->staticI.size());
                const int32_t ids = rows + int32_t(pointCount) + 1;
                const int32_t coeffs = int32_t(layout->staticF.size());
                layout->staticI.resize(size_t(ids) + size_t(entries));
                layout->staticF.resize(size_t(coeffs) + size_t(entries));
                for (size_t i = 0; i <= pointCount; ++i)
                    layout->staticI[size_t(rows) + i] = ids + counts[i];
                std::vector<int32_t> fill(counts.begin(), counts.end() - 1);
                // Control points ascending per point: the order the basis
                // applies them in.
                for (size_t j = 0; built && j < basis.byControlPoint.size(); ++j) {
                    for (const auto &[k, coeff] : basis.byControlPoint[j]) {
                        const size_t point = size_t(indices[k]);
                        const int32_t e = fill[point]++;
                        layout->staticI[size_t(ids + e)] = int32_t(j);
                        layout->staticF[size_t(coeffs + e)] = coeff;
                    }
                }
                op[RigExecGpuOpAField] = rows;
                op[RigExecGpuOpBField] = 0;
                op[RigExecGpuOpCField] = coeffs - ids;
                blockFloats += 3 * uint32_t(key.controlCount);
                break;
            }
            }
            frameFloats += blockFloats;
        }
        layout->frameFloats = frameFloats;
        layout->serial = ++_layoutSerial;
        _layout = std::move(layout);
        _keys = std::move(keys);
        _pointCount = pointCount;
        ++_layoutBuilds;
    }

    _FillFrame(revisions, frame);
    return true;
}

bool
RigExecGpuDeformLowering::_KeyHolds(const _Key &key,
                                    const RigExecGpuRevisionInput &in)
{
    const RigExecMoverParameters &p = *in.parameters;
    const bool samePacket = in.weightsObject >= 0 &&
                            key.weightsObject == in.weightsObject &&
                            key.weightsRevision == in.weightsRevision;
    if (key.op != in.op || !(samePacket || key.weights == p.weights))
        return false;
    switch (in.op) {
    case RigExecRevisionOp::Matrix:
        return key.kind == (p.radialWeight ? RigExecGpuOpKind::MatrixRadial
                                           : RigExecGpuOpKind::Matrix);
    case RigExecRevisionOp::Skin:
        if (key.kind != (p.skinningMethod == "dualQuaternion"
                             ? RigExecGpuOpKind::SkinDualQuat
                             : RigExecGpuOpKind::SkinLinear) ||
            key.topology != p.skinTopology ||
            key.influenceCount != _Influences(in).size()) {
            return false;
        }
        return key.topology ||
               (key.skinIndices == p.skinIndices &&
                key.skinWeights == p.skinWeights &&
                key.elementSize == p.skinElementSize);
    case RigExecRevisionOp::BlendShape: {
        if (key.channelSizes.size() != in.channels->size()) return false;
        size_t s = 0;
        for (size_t c = 0; c < in.channels->size(); ++c) {
            const RigExecBlendChannel &channel = (*in.channels)[c];
            if (key.channelSizes[c] != channel.samples.size()) return false;
            for (const RigExecBlendSampleData &sample : channel.samples)
                if (s >= key.layouts.size() || key.layouts[s++] != sample.layout)
                    return false;
        }
        return s == key.layouts.size();
    }
    case RigExecRevisionOp::Wire:
        return key.bindCoords == p.wireBindCoords &&
               key.knots == p.curveKnots && key.order == p.curveOrder &&
               key.controlCount == p.restPoints.size() &&
               std::memcmp(&key.dropoff, &p.dropoffDistance,
                           sizeof(double)) == 0;
    default:
        return false;
    }
}

void
RigExecGpuDeformLowering::_FillFrame(
    const std::vector<RigExecGpuRevisionInput> &revisions,
    RigExecGpuDeformFrame *frame) const
{
    // The frame: what moves.
    const RigExecGpuDeformLayout &L = *_layout;
    frame->layout = _layout;
    frame->frame.assign(L.frameFloats, 0.0f);
    for (size_t r = 0; r < revisions.size(); ++r) {
        const RigExecGpuRevisionInput &in = revisions[r];
        const RigExecMoverParameters &p = *in.parameters;
        const _Key &key = _keys[r];
        const int32_t *op = L.ops.data() + r * RigExecGpuOpStride;
        float *block = frame->frame.data() + op[RigExecGpuOpFrameField];
        bool applies = in.applies && key.staticOk;
        switch (key.kind) {
        case RigExecGpuOpKind::Matrix:
            _PutAffine(p.transform, block + 1);
            break;
        case RigExecGpuOpKind::MatrixRadial: {
            _PutAffine(p.transform, block + 1);
            const RigExecPartialDecomposition d =
                RigExecDecomposePartialTransform(p.transform);
            float *r = block + 1 + RigExecGpuAffineFloats;
            for (int k = 0; k < 3; ++k) {
                r[k] = float(d.scale[k]);
                r[3 + k] = float(d.axis[k]);
                r[7 + k] = float(d.translation[k]);
                r[11 + k] = float(d.pivot[k]);
            }
            r[6] = float(GfDegreesToRadians(d.angle));
            r[10] = float(d.along);
            r[14] = d.smallAngle ? 1.0f : 0.0f;
            break;
        }
        case RigExecGpuOpKind::SkinLinear: {
            const std::vector<GfMatrix4d> &table = _Influences(in);
            if (table.size() != key.influenceCount) { applies = false; break; }
            for (size_t t = 0; t < table.size(); ++t)
                _PutAffine(table[t], block + 1 + t * RigExecGpuAffineFloats);
            break;
        }
        case RigExecGpuOpKind::SkinDualQuat: {
            const std::vector<GfMatrix4d> &table = _Influences(in);
            if (table.size() != key.influenceCount) { applies = false; break; }
            for (size_t t = 0; t < table.size(); ++t)
                _PutDualQuat(RigExecScaledDualQuatFromMatrix(table[t]),
                             block + 1 + t * RigExecGpuDualQuatFloats);
            _PutDualQuat(RigExecScaledDualQuat(),
                         block + 1 + table.size() * RigExecGpuDualQuatFloats);
            break;
        }
        case RigExecGpuOpKind::Blend:
            if (_SampleCount(*in.channels) != key.layouts.size() ||
                !_BlendScales(*in.channels, L.pointCount, block + 1)) {
                applies = false;
            }
            break;
        case RigExecGpuOpKind::Wire:
            if (p.restPoints.size() != key.controlCount ||
                p.auxPoints.size() != key.controlCount) {
                applies = false;
                break;
            }
            for (size_t j = 0; j < key.controlCount; ++j) {
                const GfVec3f delta = p.auxPoints[j] - p.restPoints[j];
                block[1 + 3 * j] = delta[0];
                block[2 + 3 * j] = delta[1];
                block[3 + 3 * j] = delta[2];
            }
            break;
        }
        block[0] = applies ? 1.0f : 0.0f;
    }
}

}  // namespace rigExec
