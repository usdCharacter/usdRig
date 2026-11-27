// The per-frame inputs a constraint mover's op nodes look up, and the
// results they store, through the pose Host both front ends bind (pose.h).
// Whether the mover is on, and its weight, come from its enabled flag and
// weight alone; a value the mover cannot read is an error only where the
// mover uses it. Header-only and free of OpenUSD.
#ifndef RIGEXEC_EVAL_CORE_CONSTRAINT_LOOKUPS_H
#define RIGEXEC_EVAL_CORE_CONSTRAINT_LOOKUPS_H

#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace rigExec {
namespace eval {

/// Whether constraint commit \p w's mover is on, its weight, and whether it
/// solves this frame.
struct ConstraintGate {
    bool enabled = false;
    double weight = 0.0;
    bool open = false;
};

/// Why a constraint's lookups left its mover nothing to solve, for the line
/// its store reports.
enum class ConstraintNote : uint8_t {
    None,
    /// A single-chain IK's effector maps to no transform.
    EffectorDegenerate,
    /// Its pole-vector objects weigh nothing in total.
    PoleWeightless,
};

/// What one op step read of the constraint commit it works on. The step's
/// host owns it, so nothing read here outlives the step.
struct ConstraintStepCache {
    uint32_t object = UINT32_MAX;
    bool commit = false;
    bool gateOk = false;
    PoseConstraintGateFacts facts;
    ConstraintGate gate;
    bool sampled = false;
    bool sampleOk = false;
    ConstraintNote note = ConstraintNote::None;
};

namespace constraint_lookups {

inline bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = why;
    return false;
}

template <class Math>
rigExecStd::Vec3d
ToStd(const typename Math::Vec3 &v)
{
    return rigExecStd::Vec3d(v[0], v[1], v[2]);
}

template <class Math>
rigExecStd::Mat4d
ToStd(const typename Math::Mat4 &m)
{
    static_assert(sizeof(m) == sizeof(rigExecStd::Mat4d),
                  "a pose matrix is sixteen doubles");
    rigExecStd::Mat4d out;
    std::memcpy(static_cast<void *>(&out), &m, sizeof(out));
    return out;
}

template <class Math>
rigExecRig::PointFrame
ToStd(const std::array<typename Math::Vec3, 4> &points, unsigned flags)
{
    rigExecRig::PointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] = ToStd<Math>(points[i]);
    out.flags = flags;
    return out;
}

template <class Math>
std::array<typename Math::Vec3, 4>
FromStd(const rigExecRig::PointFrame &frame)
{
    std::array<typename Math::Vec3, 4> points;
    for (size_t i = 0; i < 4; ++i)
        points[i] = Math::Vec3Of(frame.points[i][0], frame.points[i][1],
                                 frame.points[i][2]);
    return points;
}

template <class T>
bool
Put(const RigExecTypeDesc &type, void *value, const T &v)
{
    if (!type.triviallyCopyable || type.size != sizeof(T))
        return false;
    std::memcpy(value, static_cast<const void *>(&v), sizeof(T));
    return true;
}

template <class T>
bool
Elements(const RigExecTypeDesc &type, void *value, size_t count, T **data)
{
    if (!type.element || !type.resize || !type.data ||
        RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size !=
            sizeof(T))
        return false;
    type.resize(value, count);
    *data = static_cast<T *>(type.data(value));
    return *data || !count;
}

/// The gate \p facts decide; false where the weight of a mover that is on
/// cannot be read.
inline bool
GateOf(const PoseConstraintGateFacts &facts, ConstraintGate *gate)
{
    *gate = ConstraintGate();
    gate->enabled = facts.enabled;
    if (!facts.enabled)
        return true;
    double weight = 1.0;
    if (facts.hasWeightObject && !facts.pointsTarget) {
        if (!facts.fieldOk || facts.fieldCount != 1)
            return false;
        weight = facts.fieldValue;
    } else if (!facts.hasWeightObject) {
        weight = facts.defaultWeight;
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0)
            return false;
    }
    gate->weight = weight;
    gate->open = weight > 0.0;
    return true;
}

/// The reason GateOf refused constraint commit \p w's weight.
template <class Host>
std::string
GateError(Host &host, size_t w, const PoseConstraintGateFacts &facts)
{
    if (facts.hasWeightObject && !facts.pointsTarget) {
        const std::string why = host.inputs().constraintFacts(w).fieldError;
        return why.empty() ? "rigExec:weightObject resolves no single weight"
                           : why;
    }
    return "inputs:defaultWeight is outside finite [0, 1]";
}

/// Constraint commit \p w as this step reads it, its gate resolved once.
template <class Host>
const ConstraintStepCache *
Read(Host &host, ConstraintStepCache &cache, size_t w, std::string *error)
{
    if (cache.object != w) {
        cache = ConstraintStepCache();
        cache.object = uint32_t(w);
        cache.commit = host.inputs().constraintGate(w, &cache.facts);
        cache.gateOk = cache.commit && GateOf(cache.facts, &cache.gate);
    }
    if (!cache.commit) {
        Fail(error,
             "walk step " + std::to_string(w) + " commits no constraint");
        return nullptr;
    }
    if (!cache.gateOk) {
        Fail(error, GateError(host, w, cache.facts));
        return nullptr;
    }
    return &cache;
}

/// Constraint commit \p w's array tables, sampled once per step.
template <class Host>
bool
Sampled(Host &host, ConstraintStepCache &cache, size_t w)
{
    if (!cache.sampled) {
        cache.sampled = true;
        cache.sampleOk = host.state().sampleConstraintArrays(w);
    }
    return cache.sampleOk;
}

/// How source \p k of constraint commit \p w resolved: a valid frame, a
/// frame that maps to no transform (flagged degenerate, handed to the
/// mover, which leaves its target as it entered), or nothing.
enum class SourceState { Valid, Degenerate, Missing };

template <class Host>
SourceState
SourceFrame(Host &host, size_t w, size_t k,
            std::array<typename Host::Math::Vec3, 4> *points, unsigned *flags)
{
    using Math = typename Host::Math;
    int slot = -1;
    uint32_t finRead = 0;
    if (!host.inputs().constraintSourceRead(w, k, &slot, &finRead))
        return SourceState::Missing;
    *points = pose_ops::IdentityLandmarks<Math>();
    *flags = 0;
    const bool resolved =
        slot >= 0 ? host.inputs().finFrame(finRead, points, flags)
                  : host.inputs().resolveNativeSource(w, k, points, flags);
    if (resolved && (*flags & pose_ops::kPoseFrameValid))
        return SourceState::Valid;
    if (*flags & pose_ops::kPoseFrameDegenerate)
        return SourceState::Degenerate;
    *points = pose_ops::IdentityLandmarks<Math>();
    *flags = 0;
    return SourceState::Missing;
}

template <class Host>
std::string
SourceError(Host &host, size_t w, size_t k)
{
    PoseSourceRef ref;
    host.inputs().constraintSourceRef(w, k, &ref);
    return "could not resolve source " + ref.pathText;
}

/// Source \p k's weight as the mover reads it: its table entry, 1 where
/// the tables do not cover the sources (the arrays lookup reports that).
template <class Host>
double
SourceWeight(Host &host, size_t w, size_t k, size_t sources)
{
    const auto a = host.inputs().constraintArrays(w);
    return a.ok && a.weightCount == sources && k < sources ? a.weights[k] : 1.0;
}

/// \p line, a diagnostic line about the mover at \p path, without the path.
inline std::string
WithoutPath(std::string line, const std::string &path)
{
    if (line.size() > path.size() && line.compare(0, path.size(), path) == 0 &&
        line[path.size()] == ' ')
        line.erase(0, path.size() + 1);
    return line;
}

/// Single-chain IK commit \p w's pole point: the weighted mean of its
/// pole-vector objects. An object that does not resolve, or a negative or
/// non-finite weight, is an error; objects weighing nothing in total give
/// the non-finite point, which leaves no solution, noted in \p cache.
template <class Host>
bool
IkPolePoint(Host &host, ConstraintStepCache &cache, size_t w, size_t poles,
            typename Host::Math::Vec3 *pole, std::string *error)
{
    using Math = typename Host::Math;
    if (!Sampled(host, cache, w))
        return Fail(error, "has no constraint arrays");
    const auto arrays = host.inputs().constraintArrays(w);
    if (!arrays.poleOk)
        return Fail(error,
                    arrays.poleDiagnosticCount
                        ? WithoutPath(arrays.poleDiagnostics[0],
                                      host.inputs().constraintFacts(w).pathText)
                        : std::string("has unusable pole-vector "
                                      "weights"));
    typename Math::Vec3 sum = Math::Vec3Of(0, 0, 0);
    double total = 0.0;
    for (size_t k = 0; k < poles; ++k) {
        if (!host.inputs().ikPoleBounds(w, k) || k >= arrays.poleWeightCount)
            return Fail(error,
                        "names no pole-vector object " + std::to_string(k));
        const double weight = arrays.poleWeights[k];
        if (!std::isfinite(weight) || weight < 0)
            return Fail(error, "has a negative or non-finite pole-vector "
                               "weight");
        std::array<typename Math::Vec3, 4> points =
            pose_ops::IdentityLandmarks<Math>();
        unsigned flags = 0;
        if (!host.inputs().resolveIkPole(w, k, &points, &flags))
            return Fail(error, "could not resolve pole-vector object " +
                                   std::to_string(k));
        sum = pose_ops::Add3<Math>(sum,
                                   pose_ops::Scale3<Math>(points[0], weight));
        total += weight;
    }
    if (total <= 0) {
        cache.note = ConstraintNote::PoleWeightless;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        *pole = Math::Vec3Of(nan, nan, nan);
        return true;
    }
    *pole = pose_ops::Scale3<Math>(sum, 1.0 / total);
    return true;
}

/// Whether an aim on at \p gate reads its world up: it does only while some
/// source weighs anything.
template <class Host>
bool
AimUsesWorldUp(Host &host, ConstraintStepCache &cache, size_t w)
{
    if (!cache.gate.open)
        return false;
    const size_t sources = host.inputs().constraintSourceCount(w);
    if (!Sampled(host, cache, w))
        return true;
    for (size_t k = 0; k < sources; ++k)
        if (!(SourceWeight(host, w, k, sources) == 0.0))
            return true;
    return false;
}

/// The world-up direction of mode \p mode for an aim whose target enters
/// at \p in. An object frame that maps to no transform gives the zero
/// direction, which pins no roll, so the aim leaves its target as it
/// entered; an object that does not resolve is false.
template <class Host>
bool
WorldUp(Host &host, size_t w, int32_t mode,
        const std::array<typename Host::Math::Vec3, 4> &in,
        typename Host::Math::Vec3 *up, std::string *error)
{
    using Math = typename Host::Math;
    using V = PoseConstraintVector;
    switch (OpConstraintWorldUp(mode)) {
    case OpConstraintWorldUp::Scene:
        *up = host.inputs().constraintVector(w, V::SceneUp);
        return true;
    case OpConstraintWorldUp::Vector:
        *up = host.inputs().constraintVector(w, V::WorldUpVector);
        return true;
    case OpConstraintWorldUp::Object:
    case OpConstraintWorldUp::ObjectRotation:
        break;
    default:
        return Fail(error, "names no world-up mode " + std::to_string(mode));
    }
    const bool rotation = mode == int32_t(OpConstraintWorldUp::ObjectRotation);
    if (!host.inputs().worldUpObjectNamed(w)) {
        *up = rotation ? host.inputs().constraintVector(w, V::WorldUpVector)
                       : pose_ops::Scale3<Math>(in[0], -1.0);
        return true;
    }
    std::array<typename Math::Vec3, 4> object =
        pose_ops::IdentityLandmarks<Math>();
    unsigned flags = 0;
    if (!host.inputs().resolveWorldUp(w, &object, &flags)) {
        if (!(flags & pose_ops::kPoseFrameDegenerate))
            return Fail(error, "could not resolve its world-up object");
        *up = Math::Vec3Of(0, 0, 0);
        return true;
    }
    if (!rotation) {
        *up = pose_ops::Sub3<Math>(object[0], in[0]);
        return true;
    }
    typename Math::Mat4 matrix = Math::Identity4();
    if (!pose_ops::PointsToMatrix<Math>(pose_ops::IdentityLandmarks<Math>(),
                                        object, &matrix)) {
        *up = Math::Vec3Of(0, 0, 0);
        return true;
    }
    *up = RotationOnlyUp<Math>(
        matrix, host.inputs().worldUpRotationOnly(w),
        host.inputs().constraintVector(w, V::WorldUpVector));
    return true;
}

} // namespace constraint_lookups

/// Resolves constraint lookup \p lookup (OpConstraintLookupSource) of
/// commit \p lookup.object into \p value of \p type, reading the commit
/// through \p cache, which the calling step owns.
template <class Host>
bool
LookUpConstraint(Host &host, ConstraintStepCache &cache, const OpLookup &lookup,
                 const RigExecTypeDesc &type, void *value, std::string *error)
{
    using namespace constraint_lookups;
    using Math = typename Host::Math;
    using L = OpLookupSource;
    using R = OpConstraintRead;
    using V = PoseConstraintVector;
    const size_t w = lookup.object;
    const ConstraintStepCache *read = Read(host, cache, w, error);
    if (!read)
        return false;
    const ConstraintGate &gate = read->gate;
    const PoseConstraintGateFacts &facts = read->facts;
    const auto bad = [&] {
        return Fail(error, "has no input " + std::to_string(lookup.read) +
                               " of lookup source " +
                               std::to_string(lookup.source) + " of that type");
    };
    switch (L(lookup.source)) {
    case L::ConstraintSources: {
        const size_t sources = host.inputs().constraintSourceCount(w);
        std::array<typename Math::Vec3, 4> points =
            pose_ops::IdentityLandmarks<Math>();
        unsigned flags = 0;
        if (lookup.part >= 0) {
            const size_t k = size_t(lookup.part);
            int slot = -1;
            uint32_t finRead = 0;
            if (k >= sources ||
                !host.inputs().constraintSourceRead(w, k, &slot, &finRead))
                return bad();
            if (lookup.read == 1) {
                if (slot < 0 ||
                    !host.inputs().restFrame(size_t(slot), &points, &flags))
                    return bad();
            } else if (gate.open && SourceFrame(host, w, k, &points, &flags) ==
                                        SourceState::Missing) {
                return Fail(error, SourceError(host, w, k));
            }
            return Put(type, value, ToStd<Math>(points, flags)) || bad();
        }
        if (!Sampled(host, cache, w))
            return bad();
        const auto arrays = host.inputs().constraintArrays(w);
        if (gate.open && !arrays.ok) {
            const std::string path = host.inputs().constraintFacts(w).pathText;
            std::string why = arrays.diagnosticCount
                                  ? arrays.diagnostics[0]
                                  : "has unusable constraint inputs";
            if (why.size() > path.size() &&
                why.compare(0, path.size(), path) == 0 &&
                why[path.size()] == ' ')
                why.erase(0, path.size() + 1);
            return Fail(error, why);
        }
        if (type.size != sizeof(rigExecRig::PointFrameArray) ||
            std::strcmp(type.name, "PointFrameArray") != 0)
            return bad();
        auto &frames =
            static_cast<rigExecRig::PointFrameArray *>(value)->frames;
        frames.Resize(sources);
        for (size_t k = 0; k < sources; ++k) {
            points = pose_ops::IdentityLandmarks<Math>();
            flags = 0;
            if (gate.open &&
                SourceFrame(host, w, k, &points, &flags) ==
                    SourceState::Missing &&
                !(SourceWeight(host, w, k, sources) == 0.0))
                return Fail(error, SourceError(host, w, k));
            frames[k] = ToStd<Math>(points, flags);
        }
        return true;
    }
    case L::ConstraintArrays: {
        if (!Sampled(host, cache, w))
            return bad();
        const size_t sources = host.inputs().constraintSourceCount(w);
        const auto a = host.inputs().constraintArrays(w);
        const bool usable =
            a.ok && a.weightCount == sources && a.offsetCount == sources;
        if (lookup.read == uint8_t(R::SourceWeights)) {
            double *out = nullptr;
            if (!Elements(type, value, sources, &out))
                return bad();
            for (size_t k = 0; k < sources; ++k)
                out[k] = usable ? a.weights[k] : 1.0;
            return true;
        }
        if (lookup.read != uint8_t(R::TranslationOffsets) &&
            lookup.read != uint8_t(R::RotationOffsets))
            return bad();
        const auto *offsets = lookup.read == uint8_t(R::TranslationOffsets)
                                  ? a.translationOffsets
                                  : a.rotationOffsets;
        rigExecStd::Vec3d *out = nullptr;
        if (!Elements(type, value, sources, &out))
            return bad();
        for (size_t k = 0; k < sources; ++k)
            out[k] = usable ? ToStd<Math>(offsets[k])
                            : rigExecStd::Vec3d(0.0, 0.0, 0.0);
        return true;
    }
    case L::ConstraintLeaf: {
        const uint8_t r = lookup.read;
        if (r >= uint8_t(R::AffectX) && r <= uint8_t(R::ScaleZ))
            return Put(type, value,
                       uint8_t(host.inputs().constraintFlag(
                                   w, size_t(r - uint8_t(R::AffectX)))
                                   ? 1
                                   : 0)) ||
                   bad();
        const V which = r == uint8_t(R::Offset)           ? V::Offset
                        : r == uint8_t(R::AimVector)      ? V::AimVector
                        : r == uint8_t(R::UpVector)       ? V::UpVector
                        : r == uint8_t(R::RotationOffset) ? V::RotationOffset
                        : r == uint8_t(R::WorldUpVector)  ? V::WorldUpVector
                                                          : V::SceneUp;
        if (which == V::SceneUp)
            return bad();
        return Put(type, value,
                   ToStd<Math>(host.inputs().constraintVector(w, which))) ||
               bad();
    }
    case L::ConstraintEnvelope: {
        if (lookup.read == uint8_t(R::SolveWeight)) {
            const bool full = facts.deltaBase >= 0 && !facts.matrixMover;
            return Put(type, value,
                       gate.open ? (full ? 1.0 : gate.weight) : 0.0) ||
                   bad();
        }
        if (lookup.read == uint8_t(R::DeltaWeight))
            return Put(type, value, gate.enabled ? gate.weight : 0.0) || bad();
        if (lookup.read != uint8_t(R::Present))
            return bad();
        const size_t slots = host.inputs().commitSlotCount(w);
        uint8_t *out = nullptr;
        if (!Elements(type, value, slots, &out))
            return bad();
        for (size_t pos = 0; pos < slots; ++pos)
            out[pos] = gate.open && facts.deltaBase < 0 ? 1 : 0;
        return true;
    }
    case L::ConstraintSpace: {
        typename Math::Mat4 carry = Math::Identity4();
        if (!ComputeSpaceCarry(host, facts.spaceSlot, &carry) && gate.open)
            return Fail(error, "has no rigExec:space carry");
        return Put(type, value, ToStd<Math>(carry)) || bad();
    }
    case L::ConstraintWorldUp: {
        typename Math::Vec3 up = Math::Vec3Of(0, 0, 0);
        if (AimUsesWorldUp(host, cache, w)) {
            std::array<typename Math::Vec3, 4> in =
                pose_ops::IdentityLandmarks<Math>();
            unsigned flags = 0;
            if (!host.inputs().finFrame(facts.targetRead, &in, &flags))
                return bad();
            if (!WorldUp(host, w, lookup.part, in, &up, error))
                return false;
        }
        return Put(type, value, ToStd<Math>(up)) || bad();
    }
    case L::HarmonicsInput: {
        using H = OpHarmonicsRead;
        switch (H(lookup.read)) {
        case H::Space: {
            typename Math::Mat4 space = Math::Identity4();
            if (facts.spaceSlot < 0 ||
                !host.inputs().posedMatrix(size_t(facts.spaceSlot), &space))
                return Fail(error, "has no rigExec:space");
            return Put(type, value, ToStd<Math>(space)) || bad();
        }
        case H::Amplitude:
        case H::Decay:
        case H::Termination:
            return Put(type, value,
                       host.inputs().harmonicsScalar(
                           w, size_t(lookup.read - uint8_t(H::Amplitude)))) ||
                   bad();
        case H::AxisAmplitude:
            return Put(type, value,
                       ToStd<Math>(host.inputs().constraintVector(
                           w, PoseConstraintVector::Offset))) ||
                   bad();
        case H::Open:
            return Put(type, value, uint8_t(gate.open ? 1 : 0)) || bad();
        }
        return bad();
    }
    case L::SingleChainIkInput: {
        using S = OpSingleChainIkRead;
        const PoseIkChainView chain = host.inputs().ikChainView(w);
        const PoseU32List reads = host.inputs().commitTargetReads(w);
        if (!facts.singleChainIk || reads.count != chain.count)
            return Fail(error, "names a chain its commit does not read");
        std::array<typename Math::Vec3, 4> points =
            pose_ops::IdentityLandmarks<Math>();
        unsigned flags = 0;
        switch (S(lookup.read)) {
        case S::Chain:
        case S::Rests: {
            if (type.size != sizeof(rigExecRig::PointFrameArray) ||
                std::strcmp(type.name, "PointFrameArray") != 0)
                return bad();
            auto &frames =
                static_cast<rigExecRig::PointFrameArray *>(value)->frames;
            frames.Resize(chain.count);
            for (size_t k = 0; k < chain.count; ++k) {
                const bool live = lookup.read == uint8_t(S::Chain) ||
                                  host.inputs().ikRestLive(w, k);
                if (live ? !host.inputs().finFrame(reads.data[k], &points,
                                                   &flags)
                         : chain.slots[k] < 0 ||
                               !host.inputs().restFrame(size_t(chain.slots[k]),
                                                        &points, &flags))
                    return bad();
                frames[k] = ToStd<Math>(points, flags);
            }
            return true;
        }
        case S::Effector:
            if (gate.open &&
                !host.inputs().resolveIkEffector(w, &points, &flags)) {
                if (!(flags & pose_ops::kPoseFrameDegenerate))
                    return Fail(error, "could not resolve its effector");
                cache.note = ConstraintNote::EffectorDegenerate;
            }
            return Put(type, value, ToStd<Math>(points, flags)) || bad();
        case S::Pole:
        case S::Twist: {
            typename Math::Vec3 pole = Math::Vec3Of(0.0, 1.0, 0.0);
            double twist = 0.0;
            host.inputs().ikPoleVector(w, &pole, &twist);
            if (lookup.read == uint8_t(S::Twist))
                return Put(type, value, twist) || bad();
            if (chain.poleModeObject && gate.open &&
                !IkPolePoint(host, cache, w, chain.poleObjects, &pole, error))
                return false;
            return Put(type, value, ToStd<Math>(pole)) || bad();
        }
        case S::Stretch:
            return Put(type, value, host.inputs().ikStretch(w)) || bad();
        }
        return bad();
    }
    case L::ConstraintDeltaBase: {
        const int d = facts.deltaBase;
        if (!host.inputs().deltaBaseValid(d))
            return bad();
        if (lookup.read == 1)
            return Put(type, value,
                       uint8_t(host.inputs().deltaBaseOk(d) ? 1 : 0)) ||
                   bad();
        typename Math::Mat4 base = Math::Identity4();
        host.inputs().deltaBaseMatrix(d, &base);
        return Put(type, value, ToStd<Math>(base)) || bad();
    }
    default:
        return bad();
    }
}

namespace constraint_lookups {

/// What constraint commit \p w's node made of its frames (\p status, its
/// kind's op's Status): a solved frame is its commit's candidate, stored
/// next where the node stores its frames; a frame with no solution leaves
/// the target as it entered, with one diagnostic line.
template <class Host>
bool
StoreStatus(Host &host, const ConstraintStepCache &cache, size_t w,
            uint32_t status, std::string *error)
{
    const PoseConstraintGateFacts &facts = cache.facts;
    host.state().setRecordAfter(w, true);
    host.state().setRecordEveryTarget(w, !facts.harmonics);
    if (facts.singleChainIk && cache.gate.enabled && facts.hasWeightObject &&
        !facts.pointsTarget)
        host.state().recordConstraintEnvelope(w);
    PoseCommitState state = host.state().commitState(w);
    std::fill(state.present, state.present + state.presentCount, char(0));
    *state.abandoned = true;
    const auto line = [&](const std::string &why,
                          const char *how = "; constraint passed through") {
        host.state().diagnose(host.inputs().constraintFacts(w).pathText + why +
                              how);
        return true;
    };
    if (status == 0)
        return true;
    if (facts.harmonics) {
        // rigExecOps::HarmonicsResult.
        if (status == 1) {
            if (state.presentCount < 1)
                return Fail(error, "commits no candidate");
            state.present[0] = 1;
            *state.abandoned = false;
            return true;
        }
        if (status == 2 || status == 3)
            return line(" could not measure " +
                        host.inputs().commitPathText(size_t(facts.target)) +
                        (status == 3 ? " in a degenerate rigExec:space" : ""));
    } else if (facts.singleChainIk) {
        // rigExecOps::SingleChainIkResult.
        if (status == 1) {
            *state.abandoned = false;
            return true;
        }
        if (status == 2)
            return line(" could not prepare rest-derived IK inputs");
        if (status == 3)
            return cache.note == ConstraintNote::EffectorDegenerate
                       ? line(" could not resolve its effector")
                   : cache.note == ConstraintNote::PoleWeightless
                       ? line(" has zero total pole-vector weight")
                       : line(" failed to solve its joint chain",
                              "; constraint passed through atomically");
    }
    return Fail(error, "made nothing known of its frames: status " +
                           std::to_string(status));
}

/// The chain single-chain IK commit \p w's node solved, written over the
/// commit's candidates once its status said it solved.
template <class Host>
bool
StoreChain(Host &host, size_t w, const rigExecRig::PointFrameArray &solved,
           std::string *error)
{
    PoseCommitState state = host.state().commitState(w);
    if (*state.abandoned)
        return true;
    const PoseIkChainView chain = host.inputs().ikChainView(w);
    if (solved.frames.Size() != chain.count)
        return Fail(error, "solved " + std::to_string(solved.frames.Size()) +
                               " joints of " + std::to_string(chain.count));
    const size_t slots = host.inputs().commitSlotCount(w);
    for (size_t k = 0; k < chain.count; ++k) {
        for (size_t pos = 0; pos < slots; ++pos) {
            int slot = -1;
            if (!host.inputs().commitSlotAt(w, pos, &slot) ||
                pos >= state.framesCount || pos >= state.presentCount)
                return Fail(error,
                            "names no candidate slot " + std::to_string(pos));
            if (slot != chain.slots[k])
                continue;
            const rigExecRig::PointFrame &frame = solved.frames[k];
            host.state().writeCommitFrame(
                w, pos, FromStd<typename Host::Math>(frame), frame.flags);
            state.present[pos] = 1;
            break;
        }
    }
    return true;
}

} // namespace constraint_lookups

/// Stores a constraint node's output \p value: the frame its mover solved
/// (with the commit's presence and record gates), or the delta its geometry
/// target's node measured and whether it could. A frame flagged degenerate,
/// and a delta that could not be measured, leave the target as it entered,
/// with one diagnostic line through the host.
template <class Host>
bool
StoreConstraint(Host &host, ConstraintStepCache &cache,
                const OpHostOutput &output, const void *value,
                std::string *error)
{
    using namespace constraint_lookups;
    using Math = typename Host::Math;
    const size_t w = output.object;
    const ConstraintStepCache *read = Read(host, cache, w, error);
    if (!read)
        return false;
    const ConstraintGate &gate = read->gate;
    const PoseConstraintGateFacts &facts = read->facts;
    if (output.target == uint8_t(OpOutputTarget::ConstraintDelta) ||
        output.target == uint8_t(OpOutputTarget::ConstraintDeltaMeasured)) {
        if (!host.inputs().deltaBaseValid(facts.deltaBase))
            return Fail(error, "measures no delta");
        const size_t d = size_t(facts.deltaBase);
        if (output.target == uint8_t(OpOutputTarget::ConstraintDelta)) {
            if (!gate.enabled) {
                host.state().writeDeltaAbsent(d);
                return true;
            }
            typename Math::Mat4 delta = Math::Identity4();
            std::memcpy(static_cast<void *>(&delta), value, sizeof(delta));
            host.state().writeDeltaMeasured(d, delta);
            return true;
        }
        if (!gate.enabled || *static_cast<const uint8_t *>(value) != 0)
            return true;
        host.state().writeDeltaAbsent(d);
        host.state().diagnose(host.inputs().constraintFacts(w).pathText +
                              " could not measure its delta against " +
                              host.inputs().deltaBasePath(w) +
                              "; constraint passed through");
        return true;
    }
    if (output.target == uint8_t(OpOutputTarget::ConstraintStatus))
        return StoreStatus(host, *read, w,
                           *static_cast<const uint32_t *>(value), error);
    if (output.target == uint8_t(OpOutputTarget::SingleChainIkFrames))
        return StoreChain(
            host, w, *static_cast<const rigExecRig::PointFrameArray *>(value),
            error);
    if (output.target != uint8_t(OpOutputTarget::ConstraintCandidate))
        return Fail(error,
                    "stores no output target " + std::to_string(output.target));
    if (gate.enabled && facts.hasWeightObject && !facts.pointsTarget)
        host.state().recordConstraintEnvelope(w);
    const bool transform = facts.deltaBase < 0;
    host.state().setRecordEveryTarget(w, !gate.open);
    host.state().setRecordAfter(w, !gate.open || transform);
    PoseCommitState state = host.state().commitState(w);
    std::fill(state.present, state.present + state.presentCount, char(0));
    *state.abandoned = true;
    if (!gate.open)
        return true;
    const auto &frame = *static_cast<const rigExecRig::PointFrame *>(value);
    const std::array<typename Math::Vec3, 4> points = FromStd<Math>(frame);
    host.state().observeCandidate(w, points, frame.flags);
    if (!transform)
        return true;
    if (FrameUsable<Math>(points, frame.flags)) {
        std::fill(state.present, state.present + state.presentCount, char(1));
        *state.abandoned = !state.presentCount;
        for (size_t pos = 0; pos < state.framesCount; ++pos)
            host.state().writeCommitFrame(w, pos, points, frame.flags);
        return true;
    }
    const std::string target =
        facts.target >= 0 ? host.inputs().commitPathText(size_t(facts.target))
                          : std::string("its target");
    const bool finite = FrameUsable<Math>(points, pose_ops::kPoseFrameValid);
    if (!finite || !(frame.flags & pose_ops::kPoseFrameDegenerate))
        return Fail(error, "produced an invalid frame for " + target);
    host.state().diagnose(host.inputs().constraintFacts(w).pathText +
                          " produced an invalid or degenerate frame for " +
                          target + "; constraint passed through");
    return true;
}

} // namespace eval
} // namespace rigExec

#endif
