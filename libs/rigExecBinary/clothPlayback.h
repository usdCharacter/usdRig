// RigExecClothMover's epoch, per-tick packet and windowed solve, shared by
// the native mover (libs/rigExec/movers/clothMover.cpp) and the zero-USD
// runtime, which installs RigExecClothPlaybackKernel for the type itself, so
// a .rigexec cloth plays wherever the runtime runs. Header-only and
// USD-free: both sides build the same epoch, pack the same packets and run
// the same solve (rigExecMath/clothKernel.h), so their points agree bit for
// bit.
//
// The epoch is what a compile fixes: the cloth's topology, rest points and
// pins, the solver settings, the time between frames, and each collider's
// topology. A tick's packet is what a frame supplies: the cloth's entering
// points, every collider's points, and the parameters.
#ifndef RIGEXEC_BINARY_CLOTH_PLAYBACK_H
#define RIGEXEC_BINARY_CLOTH_PLAYBACK_H

#include "rigExecBinary/external.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecMath/clothKernel.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// The schema type a runtime installs RigExecClothPlaybackKernel for.
constexpr char RigExecClothMoverType[] = "RigExecClothMover";

/// The declared inputs, in declaration order. The epoch's come first and
/// read at Default (rigExec:frameRate 0 takes the stage's time codes per
/// second, which the binding carries as its compile data); then the
/// per-frame parameters; then four per collider: its points (at the frame,
/// at the relationship's read phase), and its face counts, indices and
/// orientation (at Default).
enum RigExecClothInput : size_t {
    RigExecClothCounts = 0,
    RigExecClothIndices,
    RigExecClothOrientation,
    RigExecClothRest,
    RigExecClothPins,
    RigExecClothWindowFrames,
    RigExecClothSubsteps,
    RigExecClothIterations,
    RigExecClothTethers,
    RigExecClothTetherSlack,
    RigExecClothFrameRate,
    RigExecClothStretchCompliance,
    RigExecClothBendCompliance,
    RigExecClothGravity,
    RigExecClothDamping,
    RigExecClothAttract,
    RigExecClothThickness,
    RigExecClothCollisionDepth,
    RigExecClothFriction,
    RigExecClothFirstCollider,
};
constexpr size_t RigExecClothInputsPerCollider = 4;

/// How many parameters a packet carries, in RigExecClothParameters order:
/// stretch and bend compliance, gravity xyz, damping, attract, thickness,
/// collision depth, friction.
constexpr size_t RigExecClothParameterCount = 10;

struct RigExecClothColliderTopology {
    std::string path;
    uint32_t pointCount = 0;
    std::vector<int32_t> counts, indices;
    bool leftHanded = false;

    bool operator==(const RigExecClothColliderTopology &o) const
    {
        return path == o.path && pointCount == o.pointCount &&
               counts == o.counts && indices == o.indices &&
               leftHanded == o.leftHanded;
    }
};

struct RigExecClothEpoch {
    uint32_t pointCount = 0;
    std::vector<int32_t> counts, indices;
    bool leftHanded = false;
    std::vector<float> rest;
    /// Empty, or one per point.
    std::vector<float> pins;
    RigExecClothSettings settings;
    double frameSeconds = 1.0 / 24.0;
    std::vector<RigExecClothColliderTopology> colliders;

    bool operator==(const RigExecClothEpoch &o) const
    {
        const auto bits = [](const std::vector<float> &a,
                             const std::vector<float> &b) {
            return a.size() == b.size() &&
                   (a.empty() ||
                    std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0);
        };
        return pointCount == o.pointCount && counts == o.counts &&
               indices == o.indices && leftHanded == o.leftHanded &&
               bits(rest, o.rest) && bits(pins, o.pins) &&
               settings == o.settings &&
               std::memcmp(&frameSeconds, &o.frameSeconds, sizeof(double)) == 0 &&
               colliders == o.colliders;
    }
};

/// An epoch built into what a solve reads. \p serial is the builder's own
/// name for it, which a device keys its upload by; zero when unnamed.
struct RigExecClothPrepared {
    RigExecClothEpoch epoch;
    RigExecClothMesh mesh;
    RigExecClothColliders colliders;
    uint64_t serial = 0;
};

namespace cloth_playback_detail {

constexpr char kTag[] = {'R', 'C', 'L', '1'};

inline void PutU32(std::vector<uint8_t> *out, uint32_t v)
{
    const uint8_t *b = reinterpret_cast<const uint8_t *>(&v);
    out->insert(out->end(), b, b + sizeof(v));
}
inline void PutBytes(std::vector<uint8_t> *out, const void *data, size_t size)
{
    const uint8_t *b = static_cast<const uint8_t *>(data);
    out->insert(out->end(), b, b + size);
}
template <class T>
inline void PutVector(std::vector<uint8_t> *out, const std::vector<T> &v)
{
    PutU32(out, uint32_t(v.size()));
    if (!v.empty()) PutBytes(out, v.data(), v.size() * sizeof(T));
}

struct Reader {
    const uint8_t *data = nullptr;
    size_t size = 0, at = 0;
    bool ok = true;
    template <class T> T Get()
    {
        T v{};
        if (!ok || at + sizeof(T) > size) { ok = false; return v; }
        std::memcpy(&v, data + at, sizeof(T));
        at += sizeof(T);
        return v;
    }
    template <class T> std::vector<T> GetVector()
    {
        const uint32_t n = Get<uint32_t>();
        std::vector<T> v;
        if (!ok || at + size_t(n) * sizeof(T) > size) { ok = false; return v; }
        v.resize(n);
        if (n) std::memcpy(v.data(), data + at, size_t(n) * sizeof(T));
        at += size_t(n) * sizeof(T);
        return v;
    }
    std::string GetString()
    {
        const std::vector<char> c = GetVector<char>();
        return std::string(c.begin(), c.end());
    }
};

}  // namespace cloth_playback_detail

/// The epoch's bytes, as a .rigexec file holds them.
inline void
RigExecEncodeClothEpoch(const RigExecClothEpoch &e, std::vector<uint8_t> *out)
{
    using namespace cloth_playback_detail;
    out->assign(kTag, kTag + sizeof(kTag));
    PutU32(out, e.pointCount);
    PutVector(out, e.counts);
    PutVector(out, e.indices);
    PutU32(out, e.leftHanded ? 1u : 0u);
    PutVector(out, e.rest);
    PutVector(out, e.pins);
    PutU32(out, uint32_t(e.settings.windowFrames));
    PutU32(out, uint32_t(e.settings.substeps));
    PutU32(out, uint32_t(e.settings.iterations));
    PutU32(out, e.settings.tethers ? 1u : 0u);
    PutBytes(out, &e.settings.tetherSlack, sizeof(float));
    PutBytes(out, &e.settings.pinThreshold, sizeof(float));
    PutBytes(out, &e.frameSeconds, sizeof(double));
    PutU32(out, uint32_t(e.colliders.size()));
    for (const RigExecClothColliderTopology &c : e.colliders) {
        PutVector(out, std::vector<char>(c.path.begin(), c.path.end()));
        PutU32(out, c.pointCount);
        PutVector(out, c.counts);
        PutVector(out, c.indices);
        PutU32(out, c.leftHanded ? 1u : 0u);
    }
}

/// Decodes RigExecEncodeClothEpoch's bytes. False with \p why on bytes it
/// did not write.
inline bool
RigExecDecodeClothEpoch(const uint8_t *data, size_t size, RigExecClothEpoch *e,
                        std::string *why)
{
    using namespace cloth_playback_detail;
    if (size < sizeof(kTag) || std::memcmp(data, kTag, sizeof(kTag)) != 0) {
        if (why) *why = "not a cloth mover epoch";
        return false;
    }
    Reader r;
    r.data = data;
    r.size = size;
    r.at = sizeof(kTag);
    RigExecClothEpoch out;
    out.pointCount = r.Get<uint32_t>();
    out.counts = r.GetVector<int32_t>();
    out.indices = r.GetVector<int32_t>();
    out.leftHanded = r.Get<uint32_t>() != 0;
    out.rest = r.GetVector<float>();
    out.pins = r.GetVector<float>();
    out.settings.windowFrames = int(r.Get<uint32_t>());
    out.settings.substeps = int(r.Get<uint32_t>());
    out.settings.iterations = int(r.Get<uint32_t>());
    out.settings.tethers = r.Get<uint32_t>() != 0;
    out.settings.tetherSlack = r.Get<float>();
    out.settings.pinThreshold = r.Get<float>();
    out.frameSeconds = r.Get<double>();
    const uint32_t colliders = r.Get<uint32_t>();
    for (uint32_t k = 0; r.ok && k < colliders; ++k) {
        RigExecClothColliderTopology c;
        c.path = r.GetString();
        c.pointCount = r.Get<uint32_t>();
        c.counts = r.GetVector<int32_t>();
        c.indices = r.GetVector<int32_t>();
        c.leftHanded = r.Get<uint32_t>() != 0;
        out.colliders.push_back(std::move(c));
    }
    if (!r.ok || r.at != size) {
        if (why) *why = "corrupt cloth mover epoch";
        return false;
    }
    *e = std::move(out);
    return true;
}

/// Builds \p epoch's mesh and colliders. False with \p why when the cloth
/// topology, rest points or pins, or a collider's topology, are unusable.
inline bool
RigExecPrepareCloth(const RigExecClothEpoch &epoch, RigExecClothPrepared *out,
                    std::string *why)
{
    const auto fail = [why](const std::string &text) {
        if (why) *why = text;
        return false;
    };
    if (epoch.rest.size() != size_t(epoch.pointCount) * 3)
        return fail("the cloth rest points do not match its point count");
    if (!epoch.pins.empty() && epoch.pins.size() != epoch.pointCount)
        return fail("rigExec:pinWeights holds " + std::to_string(epoch.pins.size()) +
                    " weights for " + std::to_string(epoch.pointCount) + " points");
    if (!(epoch.frameSeconds > 0.0) || !std::isfinite(epoch.frameSeconds))
        return fail("the cloth frame rate is not positive");
    RigExecClothPrepared prepared;
    prepared.epoch = epoch;
    std::string reason;
    if (!RigExecBuildClothMesh(epoch.pointCount, epoch.counts.data(),
                               epoch.counts.size(), epoch.indices.data(),
                               epoch.indices.size(), epoch.leftHanded,
                               epoch.rest.data(),
                               epoch.pins.empty() ? nullptr : epoch.pins.data(),
                               epoch.settings, &prepared.mesh, &reason)) {
        return fail(reason);
    }
    for (const RigExecClothColliderTopology &c : epoch.colliders) {
        if (!RigExecAppendClothCollider(c.pointCount, c.counts.data(), c.counts.size(),
                                        c.indices.data(), c.indices.size(),
                                        c.leftHanded, &prepared.colliders)) {
            return fail("collider " + c.path + " has invalid topology");
        }
    }
    *out = std::move(prepared);
    return true;
}

/// One tick's packet: the entering points, then each collider's points in
/// epoch order, and the parameters as doubles. False when a count disagrees
/// with the epoch.
inline bool
RigExecClothPacket(const RigExecClothEpoch &epoch, const float *entering,
                   size_t enteringCount,
                   const std::vector<const float *> &colliderPoints,
                   const std::vector<size_t> &colliderCounts,
                   const RigExecClothParameters &p, RigExecWindowPacket *out)
{
    if (enteringCount != epoch.pointCount || !entering ||
        colliderPoints.size() != epoch.colliders.size() ||
        colliderCounts.size() != epoch.colliders.size()) {
        return false;
    }
    size_t total = 3 * size_t(epoch.pointCount);
    for (size_t k = 0; k < epoch.colliders.size(); ++k) {
        if (colliderCounts[k] != epoch.colliders[k].pointCount || !colliderPoints[k])
            return false;
        total += 3 * colliderCounts[k];
    }
    out->points.resize(total);
    float *at = out->points.data();
    std::memcpy(at, entering, 3 * size_t(epoch.pointCount) * sizeof(float));
    at += 3 * size_t(epoch.pointCount);
    for (size_t k = 0; k < epoch.colliders.size(); ++k) {
        std::memcpy(at, colliderPoints[k], 3 * colliderCounts[k] * sizeof(float));
        at += 3 * colliderCounts[k];
    }
    out->scalars = {double(p.stretchCompliance), double(p.bendCompliance),
                    double(p.gravity[0]),        double(p.gravity[1]),
                    double(p.gravity[2]),        double(p.damping),
                    double(p.attract),           double(p.thickness),
                    double(p.collisionDepth),    double(p.friction)};
    return true;
}

/// The window a solve reads from packets \p ticks (oldest first, the last
/// the current frame, each a RigExecClothPacket): a null tick or one of
/// another shape ends the history before it, and at most windowFrames
/// earlier ticks are kept. The ticks borrow the packets. False when the
/// current packet is unusable.
inline bool
RigExecClothTicksFromPackets(const RigExecClothPrepared &prepared,
                             const RigExecWindowPacket *const *ticks,
                             size_t count, std::vector<RigExecClothTick> *window,
                             std::string *why)
{
    const RigExecClothEpoch &epoch = prepared.epoch;
    const size_t expected = prepared.colliders.pointCount * 3 + 3 * size_t(epoch.pointCount);
    const auto usable = [&](const RigExecWindowPacket *t) {
        return t && t->points.size() == expected &&
               t->scalars.size() == RigExecClothParameterCount;
    };
    window->clear();
    if (!count || !usable(ticks[count - 1])) {
        if (why) *why = "the cloth's current inputs are unusable";
        return false;
    }
    size_t begin = count - 1;
    while (begin > 0 && usable(ticks[begin - 1])) --begin;
    // At most the window's own length: the oldest tick, then windowFrames.
    const size_t keep = size_t(std::max(0, epoch.settings.windowFrames)) + 1;
    if (count - begin > keep) begin = count - keep;
    window->reserve(count - begin);
    for (size_t k = begin; k < count; ++k) {
        const RigExecWindowPacket &t = *ticks[k];
        RigExecClothTick tick;
        tick.entering = t.points.data();
        tick.colliders = prepared.colliders.pointCount
                             ? t.points.data() + 3 * size_t(epoch.pointCount)
                             : nullptr;
        RigExecClothParameters &p = tick.parameters;
        p.stretchCompliance = float(t.scalars[0]);
        p.bendCompliance = float(t.scalars[1]);
        p.gravity[0] = float(t.scalars[2]);
        p.gravity[1] = float(t.scalars[3]);
        p.gravity[2] = float(t.scalars[4]);
        p.damping = float(t.scalars[5]);
        p.attract = float(t.scalars[6]);
        p.thickness = float(t.scalars[7]);
        p.collisionDepth = float(t.scalars[8]);
        p.friction = float(t.scalars[9]);
        window->push_back(tick);
    }
    return true;
}

/// Solves the window of packets \p ticks (RigExecClothTicksFromPackets)
/// into \p out (pointCount float3s) on the CPU kernel. False, with \p out
/// holding the current entering points when they are usable, when the
/// solve fails.
inline bool
RigExecClothSolvePackets(const RigExecClothPrepared &prepared,
                         const RigExecWindowPacket *const *ticks, size_t count,
                         float *out, std::string *why)
{
    const RigExecClothEpoch &epoch = prepared.epoch;
    std::vector<RigExecClothTick> window;
    if (!RigExecClothTicksFromPackets(prepared, ticks, count, &window, why))
        return false;
    RigExecClothWorkspace workspace;
    return RigExecSolveCloth(prepared.mesh, prepared.colliders, epoch.settings,
                             window.data(), window.size(), epoch.frameSeconds,
                             &workspace, out, why);
}

namespace cloth_playback_detail {

// The playback kernel's state: the prepared epoch.
inline std::shared_ptr<const void>
Prepare(const uint8_t *epoch, size_t size, std::string *error)
{
    RigExecClothEpoch decoded;
    if (!RigExecDecodeClothEpoch(epoch, size, &decoded, error)) return nullptr;
    auto prepared = std::make_shared<RigExecClothPrepared>();
    if (!RigExecPrepareCloth(decoded, prepared.get(), error)) return nullptr;
    return prepared;
}

inline const float *
ScalarFloat(const RigExecExternalInputValue *inputs, size_t count, size_t k,
            float *scratch)
{
    if (k >= count || !inputs[k].hasValue || !inputs[k].data) return nullptr;
    if (inputs[k].type == uint8_t(RigExecExternalInputType::Float)) {
        std::memcpy(scratch, inputs[k].data, sizeof(float));
        return scratch;
    }
    if (inputs[k].type == uint8_t(RigExecExternalInputType::Double)) {
        double d;
        std::memcpy(&d, inputs[k].data, sizeof(double));
        *scratch = float(d);
        return scratch;
    }
    return nullptr;
}

// The packet from playback's arguments: the parameters and each collider's
// points from the declared inputs, a phased collider from its evaluated
// points.
inline bool
Capture(const void *state, const RigExecExternalPhasedPoints *phased,
        size_t phasedCount, const RigExecExternalInputValue *inputs,
        size_t inputCount, const float *xyz, size_t pointCount,
        RigExecWindowPacket *packet)
{
    if (!state || !packet) return false;
    const auto &prepared = *static_cast<const RigExecClothPrepared *>(state);
    const RigExecClothEpoch &epoch = prepared.epoch;
    RigExecClothParameters p;
    float v = 0.0f;
    const auto scalar = [&](size_t k, float *out) {
        const float *got = ScalarFloat(inputs, inputCount, k, &v);
        if (got) *out = *got;
    };
    scalar(RigExecClothStretchCompliance, &p.stretchCompliance);
    scalar(RigExecClothBendCompliance, &p.bendCompliance);
    scalar(RigExecClothDamping, &p.damping);
    scalar(RigExecClothAttract, &p.attract);
    scalar(RigExecClothThickness, &p.thickness);
    scalar(RigExecClothCollisionDepth, &p.collisionDepth);
    scalar(RigExecClothFriction, &p.friction);
    if (RigExecClothGravity < inputCount && inputs[RigExecClothGravity].hasValue &&
        inputs[RigExecClothGravity].type == uint8_t(RigExecExternalInputType::Vec3f) &&
        inputs[RigExecClothGravity].data) {
        std::memcpy(p.gravity, inputs[RigExecClothGravity].data, 3 * sizeof(float));
    }
    std::vector<const float *> colliders(epoch.colliders.size(), nullptr);
    std::vector<size_t> counts(epoch.colliders.size(), 0);
    for (size_t k = 0; k < epoch.colliders.size(); ++k) {
        const std::string &path = epoch.colliders[k].path;
        for (size_t j = 0; j < phasedCount; ++j) {
            if (phased[j].path && phased[j].xyz && path == phased[j].path) {
                colliders[k] = phased[j].xyz;
                counts[k] = phased[j].count;
            }
        }
        const size_t at = RigExecClothFirstCollider + k * RigExecClothInputsPerCollider;
        if (!colliders[k] && at < inputCount && inputs[at].hasValue &&
            inputs[at].type == uint8_t(RigExecExternalInputType::Vec3fArray)) {
            colliders[k] = static_cast<const float *>(inputs[at].data);
            counts[k] = inputs[at].count;
        }
    }
    return RigExecClothPacket(epoch, xyz, pointCount, colliders, counts, p, packet);
}

inline bool
ApplyWindow(const void *state, const RigExecWindowPacket *const *ticks,
            size_t tickCount, float *xyz, size_t pointCount)
{
    if (!state) return false;
    const auto &prepared = *static_cast<const RigExecClothPrepared *>(state);
    if (pointCount != prepared.epoch.pointCount) return false;
    std::vector<float> out(3 * pointCount);
    std::string why;
    if (!RigExecClothSolvePackets(prepared, ticks, tickCount, out.data(), &why))
        return false;
    std::memcpy(xyz, out.data(), out.size() * sizeof(float));
    return true;
}

// A cloth never runs outside a window; apply alone solves one tick, which
// is its entering points.
inline bool
Apply(const void *, const uint8_t *, size_t, const RigExecExternalPhasedPoints *,
      size_t, const RigExecExternalInputValue *, size_t, float *, size_t)
{
    return true;
}

}  // namespace cloth_playback_detail

/// The cloth mover's playback kernel. A value, so no static initializer
/// stands between a reader and it.
inline RigExecExternalKernel
RigExecClothPlaybackKernel()
{
    RigExecExternalKernel kernel;
    kernel.prepare = &cloth_playback_detail::Prepare;
    kernel.apply = &cloth_playback_detail::Apply;
    kernel.captureWindow = &cloth_playback_detail::Capture;
    kernel.applyWindow = &cloth_playback_detail::ApplyWindow;
    return kernel;
}

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_CLOTH_PLAYBACK_H
