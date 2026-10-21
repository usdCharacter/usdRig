// Cloth material and attachment data. USD-free. Method references
// (docs/references.md): the XPBD constraint force estimate (Macklin, Mueller
// and Chentanez, MIG 2016) drives RigExecClothStrength's force curve;
// samples interpolate with barycentric coordinates (Ericson, Real-Time
// Collision Detection, 3.4). The warp/weft/normal blend in
// RigExecClothStrength is RigExec's own.
#ifndef RIGEXEC_MATH_CLOTH_MODEL_H
#define RIGEXEC_MATH_CLOTH_MODEL_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

enum class RigExecClothProfileKind : uint32_t {
    Stretch, Bend, Damping, Pin, Attachment, Collision, SelfCollision, Tether
};

enum RigExecClothMapChannel : size_t {
    ClothMask, ClothMinStrength, ClothMaxStrength, ClothWarp, ClothWeft,
    ClothNormal, ClothThickness, ClothFriction, ClothMapChannelCount
};

/// A vertex is represented by weights (1,0,0). Barycentric and UV bindings
/// compile to the same three-vertex sample before a solve starts.
struct RigExecClothSample {
    std::array<int32_t, 3> vertices{{0, 0, 0}};
    std::array<float, 3> weights{{1, 0, 0}};
    bool operator==(const RigExecClothSample &b) const
    { return vertices == b.vertices && weights == b.weights; }
};

struct RigExecClothProfile {
    RigExecClothProfileKind kind = RigExecClothProfileKind::Stretch;
    bool hard = false;
    bool twoSided = true;
    float minStrength = 1, maxStrength = 1, compliance = 1e-4f;
    float warp = 1, weft = 1, normal = 1;
    float thickness = 0.5f, friction = 0.3f;
    /// Negative measures attachment distance in the rest shape; zero welds.
    float restLength = -1;
    /// -1 is this cloth. Other values index the captured collider meshes.
    int32_t targetCollider = -1;
    /// (nonnegative force, blend in [0,1]), clamped at either end.
    std::vector<float> forceCurve;
    std::array<std::vector<float>, ClothMapChannelCount> maps;
    std::vector<RigExecClothSample> source, target;
    /// Empty selects the entire cloth. Attachments never alter these masks.
    std::vector<float> sourceMask, targetMask;

    bool operator==(const RigExecClothProfile &b) const
    {
        return kind == b.kind && hard == b.hard && twoSided == b.twoSided && minStrength == b.minStrength &&
            maxStrength == b.maxStrength && compliance == b.compliance &&
            warp == b.warp && weft == b.weft && normal == b.normal &&
            thickness == b.thickness && friction == b.friction &&
            restLength == b.restLength && targetCollider == b.targetCollider &&
            forceCurve == b.forceCurve && maps == b.maps && source == b.source &&
            target == b.target && sourceMask == b.sourceMask && targetMask == b.targetMask;
    }
};

struct RigExecClothModel {
    /// Two UV coordinates per triangle corner, in the cloth's triangulation.
    std::vector<float> triangleUV;
    std::vector<RigExecClothProfile> profiles;
    bool Empty() const { return profiles.empty(); }
    bool operator==(const RigExecClothModel &b) const
    { return triangleUV == b.triangleUV && profiles == b.profiles; }
};

inline float RigExecClothCurve(const std::vector<float> &keys, float force)
{
    if (keys.empty()) return 1;
    if (force <= keys[0]) return keys[1];
    for (size_t k = 2; k < keys.size(); k += 2) {
        if (force <= keys[k]) {
            const float t = (force - keys[k - 2]) / (keys[k] - keys[k - 2]);
            return keys[k - 1] + t * (keys[k + 1] - keys[k - 1]);
        }
    }
    return keys.back();
}

inline float RigExecClothMap(const RigExecClothProfile &p, size_t channel,
                            size_t vertex)
{
    return p.maps[channel].empty() ? 1.0f : p.maps[channel][vertex];
}

inline float RigExecClothSampleMap(const RigExecClothProfile &p, size_t channel,
                                  const RigExecClothSample &sample)
{
    float value = 0;
    for (size_t k = 0; k < 3; ++k)
        value += sample.weights[k] * RigExecClothMap(p, channel, size_t(sample.vertices[k]));
    return value;
}

/// Strength uses the estimated reaction load, independently of the current
/// iteration's compliance. Zero strength disables a constraint; positive
/// strength leaves a hard constraint rigid and scales a soft one's stiffness.
inline float RigExecClothStrength(const RigExecClothProfile &p,
                                  const RigExecClothSample &sample, float force,
                                  const float *direction = nullptr,
                                  const float *frame = nullptr)
{
    const float lo = p.minStrength * RigExecClothSampleMap(p, ClothMinStrength, sample);
    const float hi = p.maxStrength * RigExecClothSampleMap(p, ClothMaxStrength, sample);
    float strength = lo + (hi - lo) * RigExecClothCurve(p.forceCurve, force);
    strength *= RigExecClothSampleMap(p, ClothMask, sample);
    if (direction && frame) {
        float directional = 0;
        const float scale[3] = {p.warp, p.weft, p.normal};
        for (size_t a = 0; a < 3; ++a) {
            float dot = 0;
            for (size_t q = 0; q < 3; ++q) dot += direction[q] * frame[3 * a + q];
            directional += dot * dot * scale[a] *
                RigExecClothSampleMap(p, ClothWarp + a, sample);
        }
        strength *= directional;
    }
    return strength;
}

inline bool RigExecClothValidateModel(const RigExecClothModel &model, size_t points,
                                      size_t triangles,
                                      const std::vector<size_t> &colliderCounts,
                                      std::string *why)
{
    const auto fail = [&](const char *s) { if (why) *why = s; return false; };
    if (!model.triangleUV.empty() && model.triangleUV.size() != triangles * 6)
        return fail("cloth UVs do not match triangle corners");
    for (float v : model.triangleUV)
        if (!std::isfinite(v)) return fail("cloth UVs are not finite");
    bool directional = false;
    for (const auto &p : model.profiles)
        directional = directional || p.warp != p.weft || p.warp != p.normal ||
            !p.maps[ClothWarp].empty() || !p.maps[ClothWeft].empty() || !p.maps[ClothNormal].empty();
    if (directional && model.triangleUV.empty()) return fail("directional cloth response requires a UV set");
    if (directional) for (size_t t = 0; t < triangles; ++t) {
        const float *uv = model.triangleUV.data() + 6 * t;
        const float determinant = (uv[2]-uv[0])*(uv[5]-uv[1]) - (uv[4]-uv[0])*(uv[3]-uv[1]);
        if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-12f)
            return fail("directional cloth response requires nondegenerate UV triangles");
    }
    const auto samplesValid = [](const std::vector<RigExecClothSample> &samples, size_t n) {
        for (const auto &s : samples) {
            float sum = 0;
            for (size_t k = 0; k < 3; ++k) {
                if (s.vertices[k] < 0 || size_t(s.vertices[k]) >= n ||
                    !std::isfinite(s.weights[k]) || s.weights[k] < 0 || s.weights[k] > 1)
                    return false;
                sum += s.weights[k];
            }
            if (std::abs(sum - 1.0f) > 1e-5f) return false;
        }
        return true;
    };
    for (const auto &p : model.profiles) {
        if (uint32_t(p.kind) > uint32_t(RigExecClothProfileKind::Tether))
            return fail("unknown cloth constraint kind");
        const float values[] = {p.minStrength, p.maxStrength, p.compliance,
            p.warp, p.weft, p.normal, p.thickness, p.friction};
        for (float v : values)
            if (!std::isfinite(v) || v < 0) return fail("invalid cloth constraint value");
        if (p.minStrength > p.maxStrength || !std::isfinite(p.restLength) ||
            (p.restLength < 0 && p.restLength != -1))
            return fail("invalid cloth strength range or rest length");
        if (p.forceCurve.size() % 2) return fail("cloth force curve needs pairs");
        for (size_t k = 0; k < p.forceCurve.size(); k += 2)
            if (!std::isfinite(p.forceCurve[k]) || !std::isfinite(p.forceCurve[k + 1]) ||
                p.forceCurve[k] < 0 || p.forceCurve[k + 1] < 0 || p.forceCurve[k + 1] > 1 ||
                (k && p.forceCurve[k] <= p.forceCurve[k - 2]))
                return fail("invalid cloth force curve");
        for (const auto &map : p.maps) {
            if (!map.empty() && map.size() != points) return fail("cloth map size mismatch");
            for (float v : map)
                if (!std::isfinite(v) || v < 0) return fail("invalid cloth map value");
        }
        for (size_t i = 0; i < points; ++i)
            if (p.minStrength * RigExecClothMap(p, ClothMinStrength, i) >
                p.maxStrength * RigExecClothMap(p, ClothMaxStrength, i))
                return fail("painted cloth minimum strength exceeds maximum");
        if (p.targetCollider < -1 ||
            (p.targetCollider >= 0 && size_t(p.targetCollider) >= colliderCounts.size()))
            return fail("invalid cloth target collider");
        const size_t targetCount = p.targetCollider < 0 ? points : colliderCounts[size_t(p.targetCollider)];
        if (!samplesValid(p.source, points) || !samplesValid(p.target, targetCount))
            return fail("invalid cloth binding");
        for (const auto *mask : {&p.sourceMask, &p.targetMask}) {
            const size_t n = mask == &p.targetMask ? targetCount : points;
            if (!mask->empty() && mask->size() != n) return fail("cloth region size mismatch");
            for (float v : *mask)
                if (!std::isfinite(v) || v < 0 || v > 1) return fail("invalid cloth region weight");
        }
        if ((p.kind == RigExecClothProfileKind::Attachment ||
             (p.kind == RigExecClothProfileKind::Tether && !p.target.empty())) &&
            (p.source.empty() || p.source.size() != p.target.size()))
            return fail("cloth attachments need matching nonempty bindings");
        if (p.kind == RigExecClothProfileKind::SelfCollision && p.targetCollider >= 0)
            return fail("self collision regions must belong to the simulated cloth");
    }
    return true;
}

}  // namespace rigExec
#endif
