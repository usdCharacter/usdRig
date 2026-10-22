// RigExecClothMover's compiled rig, per-tick packet and windowed solve, shared by
// the native mover (libs/rigExec/movers/clothMover.cpp) and the zero-USD
// runtime, which installs RigExecClothPlaybackKernel for the type itself, so
// a .rigexec cloth plays wherever the runtime runs. Header-only and
// USD-free: both sides build the same compiled rig, pack the same packets and run
// the same solve (rigExecMath/clothKernel.h), so their points agree bit for
// bit.
//
// The compiled rig is what a compile fixes: the cloth's topology, rest points and
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

/// The declared inputs, in declaration order. The compiled rig's come first and
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
    RigExecClothMaxWindow,
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

struct RigExecClothCompiledData {
    uint32_t pointCount = 0;
    std::vector<int32_t> counts, indices;
    bool leftHanded = false;
    std::vector<float> rest;
    /// Empty, or one per point.
    std::vector<float> pins;
    RigExecClothSettings settings;
    double frameSeconds = 1.0 / 24.0;
    std::vector<RigExecClothColliderTopology> colliders;
    RigExecClothModel model;

    bool operator==(const RigExecClothCompiledData &o) const
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
               colliders == o.colliders && model == o.model;
    }
};

/// A compiled rig built into what a solve reads. \p serial is the builder's own
/// name for it, which a device keys its upload by; zero when unnamed.
struct RigExecClothPrepared {
    RigExecClothCompiledData compiledRig;
    RigExecClothMesh mesh;
    RigExecClothColliders colliders;
    uint64_t serial = 0;
};

namespace cloth_playback_detail {

/// The compiled rig a writer writes: RCL3 (format 26). Format 25 files hold RCL1,
/// which has no maxWindowFrames and no material model, and whose declared
/// inputs have no rigExec:maxWindowFrames slot (RigExecClothInputSlot).
constexpr char kTag[] = {'R', 'C', 'L', '3'};
constexpr char kLegacyRevision = '1';

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

inline void RigExecEncodeClothModel(const RigExecClothModel &model, std::vector<uint8_t> *out)
{
    using namespace cloth_playback_detail;
    PutVector(out, model.triangleUV);
    PutU32(out, uint32_t(model.profiles.size()));
    for (const auto &p : model.profiles) {
        PutU32(out, uint32_t(p.kind)); PutU32(out, p.hard ? 1 : 0);
        PutU32(out, p.twoSided ? 1 : 0);
        PutU32(out, uint32_t(p.targetCollider));
        const float values[] = {p.minStrength, p.maxStrength, p.compliance, p.warp,
            p.weft, p.normal, p.thickness, p.friction, p.restLength};
        PutBytes(out, values, sizeof(values));
        PutVector(out, p.forceCurve);
        for (const auto &map : p.maps) PutVector(out, map);
        for (const auto *samples : {&p.source, &p.target}) {
            PutU32(out, uint32_t(samples->size()));
            for (const auto &sample : *samples) {
                for (int32_t i : sample.vertices) PutU32(out, uint32_t(i));
                PutBytes(out, sample.weights.data(), 3 * sizeof(float));
            }
        }
        PutVector(out, p.sourceMask); PutVector(out, p.targetMask);
    }
}

inline bool RigExecDecodeClothModel(cloth_playback_detail::Reader *reader,
                                    RigExecClothModel *model)
{
    auto &r = *reader;
    model->profiles.clear();
    model->triangleUV = r.GetVector<float>();
    const uint32_t count = r.Get<uint32_t>();
    for (uint32_t k = 0; r.ok && k < count; ++k) {
        RigExecClothProfile p;
        p.kind = RigExecClothProfileKind(r.Get<uint32_t>());
        const uint32_t hard = r.Get<uint32_t>();
        if (hard > 1) r.ok = false;
        p.hard = hard != 0;
        const uint32_t twoSided = r.Get<uint32_t>();
        if (twoSided > 1) r.ok = false;
        p.twoSided = twoSided != 0;
        p.targetCollider = int32_t(r.Get<uint32_t>());
        float *values[] = {&p.minStrength, &p.maxStrength, &p.compliance, &p.warp,
            &p.weft, &p.normal, &p.thickness, &p.friction, &p.restLength};
        for (float *value : values) *value = r.Get<float>();
        p.forceCurve = r.GetVector<float>();
        for (auto &map : p.maps) map = r.GetVector<float>();
        for (auto *samples : {&p.source, &p.target}) {
            const uint32_t n = r.Get<uint32_t>();
            for (uint32_t j = 0; r.ok && j < n; ++j) {
                RigExecClothSample sample;
                for (int32_t &i : sample.vertices) i = int32_t(r.Get<uint32_t>());
                for (float &v : sample.weights) v = r.Get<float>();
                samples->push_back(sample);
            }
        }
        p.sourceMask = r.GetVector<float>(); p.targetMask = r.GetVector<float>();
        model->profiles.push_back(std::move(p));
    }
    return r.ok;
}

/// The compiled rig's bytes, as a .rigexec file holds them.
inline void
RigExecEncodeClothCompiledData(const RigExecClothCompiledData &e, std::vector<uint8_t> *out)
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
    PutU32(out, uint32_t(e.settings.maxWindowFrames));
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
    RigExecEncodeClothModel(e.model, out);
}

/// Decodes RigExecEncodeClothCompiledData's bytes, or a format-25 RCL1 compiled rig,
/// which plays as it was exported: maxWindowFrames equal to windowFrames
/// (no reach-back; the history is the window) and no material model.
/// \p legacy, when set, says whether the bytes were RCL1. False with \p why
/// on bytes neither wrote.
inline bool
RigExecDecodeClothCompiledData(const uint8_t *data, size_t size, RigExecClothCompiledData *e,
                        std::string *why, bool *legacy = nullptr)
{
    using namespace cloth_playback_detail;
    if (size < sizeof(kTag) || std::memcmp(data, kTag, 3) != 0 ||
        (data[3] != kTag[3] && data[3] != kLegacyRevision)) {
        if (why) *why = "not a cloth mover compiled rig";
        return false;
    }
    const bool rcl1 = data[3] == kLegacyRevision;
    Reader r;
    r.data = data;
    r.size = size;
    r.at = sizeof(kTag);
    RigExecClothCompiledData out;
    out.pointCount = r.Get<uint32_t>();
    out.counts = r.GetVector<int32_t>();
    out.indices = r.GetVector<int32_t>();
    out.leftHanded = r.Get<uint32_t>() != 0;
    out.rest = r.GetVector<float>();
    out.pins = r.GetVector<float>();
    out.settings.windowFrames = int(r.Get<uint32_t>());
    out.settings.maxWindowFrames =
        rcl1 ? out.settings.windowFrames : int(r.Get<uint32_t>());
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
    if (!rcl1 && !RigExecDecodeClothModel(&r, &out.model)) {
        if (why) *why = "corrupt cloth material model";
        return false;
    }
    if (!r.ok || r.at != size) {
        if (why) *why = "corrupt cloth mover compiled rig";
        return false;
    }
    *e = std::move(out);
    if (legacy) *legacy = rcl1;
    return true;
}

/// Where a declared input \p k of RigExecClothInput sits in the inputs a
/// file's cloth revision carries: one slot earlier past windowFrames for a
/// format-25 (RCL1) compiled rig, which declared no rigExec:maxWindowFrames.
constexpr size_t
RigExecClothInputSlot(size_t k, bool legacy)
{
    return legacy && k > RigExecClothWindowFrames ? k - 1 : k;
}

/// Builds \p compiledRig's mesh and colliders. False with \p why when the cloth
/// topology, rest points or pins, or a collider's topology, are unusable.
inline bool
RigExecPrepareCloth(const RigExecClothCompiledData &compiledRig, RigExecClothPrepared *out,
                    std::string *why)
{
    const auto fail = [why](const std::string &text) {
        if (why) *why = text;
        return false;
    };
    if (compiledRig.rest.size() != size_t(compiledRig.pointCount) * 3)
        return fail("the cloth rest points do not match its point count");
    if (!compiledRig.pins.empty() && compiledRig.pins.size() != compiledRig.pointCount)
        return fail("rigExec:pinWeights holds " + std::to_string(compiledRig.pins.size()) +
                    " weights for " + std::to_string(compiledRig.pointCount) + " points");
    if (!(compiledRig.frameSeconds > 0.0) || !std::isfinite(compiledRig.frameSeconds))
        return fail("the cloth frame rate is not positive");
    RigExecClothPrepared prepared;
    prepared.compiledRig = compiledRig;
    std::string reason;
    if (!RigExecBuildClothMesh(compiledRig.pointCount, compiledRig.counts.data(),
                               compiledRig.counts.size(), compiledRig.indices.data(),
                               compiledRig.indices.size(), compiledRig.leftHanded,
                               compiledRig.rest.data(),
                               compiledRig.pins.empty() ? nullptr : compiledRig.pins.data(),
                               compiledRig.settings, &prepared.mesh, &reason)) {
        return fail(reason);
    }
    for (const RigExecClothColliderTopology &c : compiledRig.colliders) {
        if (!RigExecAppendClothCollider(c.pointCount, c.counts.data(), c.counts.size(),
                                        c.indices.data(), c.indices.size(),
                                        c.leftHanded, &prepared.colliders)) {
            return fail("collider " + c.path + " has invalid topology");
        }
    }
    std::vector<size_t> colliderCounts;
    for (const auto &c : compiledRig.colliders) colliderCounts.push_back(c.pointCount);
    if (!RigExecPrepareClothModel(&prepared.mesh, compiledRig.model, compiledRig.rest.data(),
                                 colliderCounts, &reason)) return fail(reason);
    *out = std::move(prepared);
    return true;
}

/// One tick's packet: the entering points, then each collider's points in
/// compiled rig order, and the parameters as doubles. False when a count disagrees
/// with the compiled rig.
inline bool
RigExecClothPacket(const RigExecClothCompiledData &compiledRig, const float *entering,
                   size_t enteringCount,
                   const std::vector<const float *> &colliderPoints,
                   const std::vector<size_t> &colliderCounts,
                   const RigExecClothParameters &p, RigExecWindowPacket *out)
{
    if (enteringCount != compiledRig.pointCount || !entering ||
        colliderPoints.size() != compiledRig.colliders.size() ||
        colliderCounts.size() != compiledRig.colliders.size()) {
        return false;
    }
    size_t total = 3 * size_t(compiledRig.pointCount);
    for (size_t k = 0; k < compiledRig.colliders.size(); ++k) {
        if (colliderCounts[k] != compiledRig.colliders[k].pointCount || !colliderPoints[k])
            return false;
        total += 3 * colliderCounts[k];
    }
    out->points.resize(total);
    float *at = out->points.data();
    std::memcpy(at, entering, 3 * size_t(compiledRig.pointCount) * sizeof(float));
    at += 3 * size_t(compiledRig.pointCount);
    for (size_t k = 0; k < compiledRig.colliders.size(); ++k) {
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
/// another shape ends the history before it, at most
/// RigExecClothHistoryFrames earlier ticks are read, and the window starts
/// at RigExecClothWindowStart of those. The ticks borrow the packets. False
/// when the current packet is unusable.
inline bool
RigExecClothTicksFromPackets(const RigExecClothPrepared &prepared,
                             const RigExecWindowPacket *const *ticks,
                             size_t count, std::vector<RigExecClothTick> *window,
                             std::string *why)
{
    const RigExecClothCompiledData &compiledRig = prepared.compiledRig;
    const size_t expected = prepared.colliders.pointCount * 3 + 3 * size_t(compiledRig.pointCount);
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
    // At most the history a window may reach back over, then the start.
    const size_t keep = size_t(RigExecClothHistoryFrames(compiledRig.settings)) + 1;
    if (count - begin > keep) begin = count - keep;
    window->reserve(count - begin);
    for (size_t k = begin; k < count; ++k) {
        const RigExecWindowPacket &t = *ticks[k];
        RigExecClothTick tick;
        tick.entering = t.points.data();
        tick.colliders = prepared.colliders.pointCount
                             ? t.points.data() + 3 * size_t(compiledRig.pointCount)
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
    const size_t start = RigExecClothWindowStart(prepared.mesh, prepared.colliders,
                                                 compiledRig.settings, window->data(),
                                                 window->size());
    window->erase(window->begin(), window->begin() + std::ptrdiff_t(start));
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
    const RigExecClothCompiledData &compiledRig = prepared.compiledRig;
    std::vector<RigExecClothTick> window;
    if (!RigExecClothTicksFromPackets(prepared, ticks, count, &window, why))
        return false;
    RigExecClothWorkspace workspace;
    return RigExecSolveCloth(prepared.mesh, prepared.colliders, compiledRig.settings,
                             window.data(), window.size(), compiledRig.frameSeconds,
                             &workspace, out, why);
}

namespace cloth_playback_detail {

// The playback kernel's state: the prepared compiled rig, and whether it came from
// a format-25 (RCL1) file, whose declared inputs sit at their format-25
// slots (RigExecClothInputSlot).
struct State {
    RigExecClothPrepared prepared;
    bool legacy = false;
};

// inputs:damping's schema fallback in format 25.
constexpr float kLegacyDamping = 0.1f;

inline std::shared_ptr<const void>
Prepare(const uint8_t *compiledRig, size_t size, std::string *error)
{
    RigExecClothCompiledData decoded;
    auto state = std::make_shared<State>();
    if (!RigExecDecodeClothCompiledData(compiledRig, size, &decoded, error, &state->legacy))
        return nullptr;
    if (!RigExecPrepareCloth(decoded, &state->prepared, error)) return nullptr;
    return state;
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
    const auto &held = *static_cast<const State *>(state);
    const RigExecClothCompiledData &compiledRig = held.prepared.compiledRig;
    const auto slot = [&](size_t k) { return RigExecClothInputSlot(k, held.legacy); };
    RigExecClothParameters p;
    if (held.legacy) p.damping = kLegacyDamping;
    float v = 0.0f;
    const auto scalar = [&](size_t k, float *out) {
        const float *got = ScalarFloat(inputs, inputCount, slot(k), &v);
        if (got) *out = *got;
    };
    scalar(RigExecClothStretchCompliance, &p.stretchCompliance);
    scalar(RigExecClothBendCompliance, &p.bendCompliance);
    scalar(RigExecClothDamping, &p.damping);
    scalar(RigExecClothAttract, &p.attract);
    scalar(RigExecClothThickness, &p.thickness);
    scalar(RigExecClothCollisionDepth, &p.collisionDepth);
    scalar(RigExecClothFriction, &p.friction);
    const size_t gravity = slot(RigExecClothGravity);
    if (gravity < inputCount && inputs[gravity].hasValue &&
        inputs[gravity].type == uint8_t(RigExecExternalInputType::Vec3f) &&
        inputs[gravity].data) {
        std::memcpy(p.gravity, inputs[gravity].data, 3 * sizeof(float));
    }
    std::vector<const float *> colliders(compiledRig.colliders.size(), nullptr);
    std::vector<size_t> counts(compiledRig.colliders.size(), 0);
    for (size_t k = 0; k < compiledRig.colliders.size(); ++k) {
        const std::string &path = compiledRig.colliders[k].path;
        for (size_t j = 0; j < phasedCount; ++j) {
            if (phased[j].path && phased[j].xyz && path == phased[j].path) {
                colliders[k] = phased[j].xyz;
                counts[k] = phased[j].count;
            }
        }
        const size_t at = slot(RigExecClothFirstCollider) + k * RigExecClothInputsPerCollider;
        if (!colliders[k] && at < inputCount && inputs[at].hasValue &&
            inputs[at].type == uint8_t(RigExecExternalInputType::Vec3fArray)) {
            colliders[k] = static_cast<const float *>(inputs[at].data);
            counts[k] = inputs[at].count;
        }
    }
    return RigExecClothPacket(compiledRig, xyz, pointCount, colliders, counts, p, packet);
}

inline bool
ApplyWindow(const void *state, const RigExecWindowPacket *const *ticks,
            size_t tickCount, float *xyz, size_t pointCount)
{
    if (!state) return false;
    const auto &prepared = static_cast<const State *>(state)->prepared;
    if (pointCount != prepared.compiledRig.pointCount) return false;
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
