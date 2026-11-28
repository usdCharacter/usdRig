#include "rigExecOps/deform/latticeOp.h"

#include "rigExecOps/deform/weightPacket.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Lattice)

RIGEXEC_DEFINE_PORT_HANDLE(LatticePoints)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeRestPoints)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeRestCage)
RIGEXEC_DEFINE_PORT_HANDLE(LatticePosedCage)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeTargetToLattice)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeLatticeToTarget)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeCageToLattice)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeOut)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(LatticePacket)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeWritten)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeEvaluation)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeDivisions)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeInterpolation)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeOrigin)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeSpacing)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeStrength)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeMask)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeCageMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeTargetMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(LatticePointSpace)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeCageFrame)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeTargetFrame)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;
using namespace opIo;
using namespace fieldIo;

Mat4d
MatrixOr1(const Mat4d *m)
{
    return m ? *m : Mat4d(1.0);
}

enum LatticeField : uint32_t {
    kDivisions,
    kRegularGrid,
    kInterpolation,
    kOrigin,
    kSpacing,
    kStrength,
    kMaskCount,
    kMask,
    kWhole,
};

/// Instance \p k's settings, its mask read in place.
struct LatticeInstance {
    decltype(LatticePayload::divisions) divisions{};
    bool whole = false;
    rigExec::RigExecLatticeSettingsView settings;
};

LatticeInstance
PayloadAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    LatticeInstance p;
    const auto divisions = ListAt<int32_t>(batch, kDivisions, k);
    const auto interpolation = ListAt<uint32_t>(batch, kInterpolation, k);
    const auto origin = ListAt<float>(batch, kOrigin, k);
    const auto spacing = ListAt<float>(batch, kSpacing, k);
    for (size_t a = 0; a < 3; ++a) {
        p.divisions[a] = divisions[a];
        p.settings.interpolation[a] =
            rigExec::RigExecLatticeInterpolation(interpolation[a]);
        p.settings.origin[a] = origin[a];
        p.settings.spacing[a] = spacing[a];
    }
    p.settings.regularGrid = FlagAt(batch, kRegularGrid, k);
    p.whole = FlagAt(batch, kWhole, k);
    p.settings.strength = ValueAt<float>(batch, kStrength, k);
    const auto mask = ListAt<float>(batch, kMask, k);
    p.settings.mask = {mask.data, mask.Size()};
    return p;
}

/// A node's bind of its rest points into the rest cage, built on the run
/// that first sees those rest values; this frame's pass through, settings,
/// grid maps and the weight at every point when a default weight or a
/// packet gives them.
struct LatticeState {
    std::shared_ptr<const rigExec::RigExecLatticeBind<Vec3f>> bind;
    /// This frame's basis for every range, or null to stream the factors.
    const rigExec::RigExecLatticeBasis *basis = nullptr;
    bool pass = false;
    std::vector<float> dense;
    LatticeInstance p;
    std::vector<float> mask;
    Mat4d targetToLattice = Mat4d(1.0), latticeToTarget = Mat4d(1.0),
          cageToLattice = Mat4d(1.0);
};

/// Instance \p k's settings this frame: its payload's, each overridden by
/// the input that feeds it, and the grid's maps, from the cage and target
/// frames when those are fed. False, with \p error, for settings no
/// evaluation can take.
bool
ResolveInstance(const rigExec::RigExecOpBatch &batch, size_t k,
                const RigExecOpIo &io, LatticeState *state, std::string *error)
{
    LatticeInstance &p = state->p;
    p = PayloadAt(batch, k);
    if (const int32_t *evaluation = In<int32_t>(io, LatticeEvaluation())) {
        if (*evaluation != 0 && *evaluation != 1)
            return Fail(error, "Lattice",
                        "the evaluation names neither a bound lattice nor a "
                        "regular grid");
        p.settings.regularGrid = *evaluation == 1;
    }
    if (const auto *divisions =
            In<RigExecArray<int32_t>>(io, LatticeDivisions())) {
        if (divisions->Size() != 3)
            return Fail(error, "Lattice", "the divisions hold three counts");
        for (size_t a = 0; a < 3; ++a)
            p.divisions[a] = (*divisions)[a];
    }
    const int minimum = p.settings.regularGrid ? 1 : 2;
    if (p.divisions[0] < minimum || p.divisions[1] < minimum ||
        p.divisions[2] < minimum)
        return Fail(error, "Lattice",
                    p.settings.regularGrid
                        ? "a grid's divisions are below 1"
                        : "a lattice's divisions are below 2");
    state->targetToLattice = MatrixOr1(In<Mat4d>(io, LatticeTargetToLattice()));
    state->latticeToTarget = MatrixOr1(In<Mat4d>(io, LatticeLatticeToTarget()));
    state->cageToLattice = MatrixOr1(In<Mat4d>(io, LatticeCageToLattice()));
    if (!p.settings.regularGrid)
        return true;
    if (const auto *interpolation =
            In<RigExecArray<int32_t>>(io, LatticeInterpolation())) {
        if (interpolation->Size() != 3)
            return Fail(error, "Lattice",
                        "the interpolation holds three modes");
        for (size_t a = 0; a < 3; ++a) {
            const int32_t mode = (*interpolation)[a];
            if (mode < 0 ||
                mode >
                    int32_t(rigExec::RigExecLatticeInterpolation::CatmullRom))
                return Fail(error, "Lattice", "an interpolation names no mode");
            p.settings.interpolation[a] =
                rigExec::RigExecLatticeInterpolation(mode);
        }
    }
    for (const auto &[port, to] :
         {std::make_pair(&LatticeOrigin(), &p.settings.origin),
          std::make_pair(&LatticeSpacing(), &p.settings.spacing)})
        if (const auto *axes = In<RigExecArray<float>>(io, *port)) {
            if (axes->Size() != 3)
                return Fail(error, "Lattice",
                            "a grid origin or spacing holds three values");
            for (size_t a = 0; a < 3; ++a)
                (*to)[a] = (*axes)[a];
        }
    if (const float *strength = In<float>(io, LatticeStrength()))
        p.settings.strength = *strength;
    if (const auto *mask = In<RigExecArray<float>>(io, LatticeMask())) {
        state->mask = ToVector(*mask);
        p.settings.mask = {state->mask.data(), state->mask.size()};
    }
    const Mat4d *cageMatrix = In<Mat4d>(io, LatticeCageMatrix());
    if (!cageMatrix)
        return true;
    Mat4d cage = *cageMatrix;
    Mat4d target = MatrixOr1(In<Mat4d>(io, LatticeTargetMatrix()));
    const Mat4d *cageFrame = In<Mat4d>(io, LatticeCageFrame());
    const Mat4d *targetFrame = In<Mat4d>(io, LatticeTargetFrame());
    const int32_t *space = In<int32_t>(io, LatticePointSpace());
    bool usable = rigExec::RigExecSurfaceSnapValidMatrix(cage) &&
                  rigExec::RigExecSurfaceSnapValidMatrix(target) &&
                  !cageFrame == !targetFrame && space &&
                  (*space == 0 || *space == 1);
    if (usable && cageFrame) {
        cage *= *cageFrame;
        target *= *targetFrame;
        usable = rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&cage) &&
                 rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&target);
    }
    if (!usable || !rigExec::RigExecLatticeCoordinateMaps(
                       *space == 0 ? "local" : "common", cage, target,
                       &state->targetToLattice, &state->latticeToTarget,
                       &state->cageToLattice))
        return Fail(error, "Lattice",
                    "the grid's cage, target or point space is unusable");
    return true;
}

/// The bind is keyed by every value its factors derive from (the rest
/// points' bits, the rest cage's box, the divisions). Null when the cage is
/// unusable or the bind is over budget: the kernel then streams the same
/// factors.
const rigExec::RigExecLatticeBasis *
BasisOf(LatticeState &state, const Points &rest, const Points &restCage,
        const Points &posedCage, const LatticeInstance &p)
{
    const int dx = p.divisions[0], dy = p.divisions[1], dz = p.divisions[2];
    Vec3f lo, size;
    if (!rigExec::RigExecLatticeKernelSetup(
            rest.Size(), rest.Data(), rest.Size(), restCage.Data(),
            restCage.Size(), posedCage.Data(), posedCage.Size(), dx, dy, dz,
            &lo, &size, static_cast<std::vector<Vec3f> *>(nullptr)))
        return nullptr;
    if (state.bind &&
        state.bind->Matches(rest.Data(), rest.Size(), lo, size, dx, dy, dz))
        return &state.bind->value;
    state.bind.reset();
    if (rigExec::RigExecLatticeBind<Vec3f>::Bytes(rest.Size(), dx, dy, dz) >
        double(rigExec::RigExecLatticeBindBudgetBytes))
        return nullptr;
    auto built = std::make_shared<rigExec::RigExecLatticeBind<Vec3f>>();
    built->Build(rest.Data(), rest.Size(), lo, size, dx, dy, dz);
    state.bind = std::move(built);
    return &state.bind->value;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &batch, size_t k)
{
    // A regular grid can fail at any point it reads: only the whole run
    // answers.
    return FlagAt(batch, kRegularGrid, k) || FlagAt(batch, kWhole, k)
               ? rigExec::RigExecRangePlan::Whole
               : rigExec::RigExecRangePlan::Ranges;
}

bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const int envelopes = int(Fed(check, LatticeEnvelope())) +
                          int(Fed(check, LatticeDefaultWeight())) +
                          int(Fed(check, LatticePacket()));
    if (envelopes > 1)
        return Fail(error, "Lattice",
                    "one of an envelope, a default weight and a weight "
                    "packet");
    if (!EnvelopeFits(check, LatticePoints(), LatticeEnvelope(), "Lattice",
                      error))
        return false;
    if (FlagAt(check.batch, kRegularGrid, check.k))
        return true;
    if (!Fed(check, LatticeRestPoints()) || !Fed(check, LatticeRestCage()))
        return Fail(error, "Lattice",
                    "a bound lattice needs rest points and a rest cage");
    if (!SameCount(CheckElements(check, LatticeRestPoints()),
                   CheckElements(check, LatticePoints())))
        return Fail(error, "Lattice",
                    "rest points differ in count from the points");
    return true;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
        std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    LatticeState &state =
        *static_cast<LatticeState *>(rigExec::RigExecBatchState(batch, k));
    const uint8_t *enabled = In<uint8_t>(io, LatticeEnabled());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    state.basis = nullptr;
    if (state.pass)
        return true;
    const Points &in = *In<Points>(io, LatticePoints());
    if (!EnvelopeFits(in, In<Envelope>(io, LatticeEnvelope()), "Lattice",
                      error))
        return false;
    if (!ResolveInstance(batch, k, io, &state, error))
        return false;
    const LatticeInstance &p = state.p;
    const size_t count = in.Size();
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    const size_t nodes = size_t(p.divisions[0]) * size_t(p.divisions[1]) *
                         size_t(p.divisions[2]);
    const Points *restCage = nullptr;
    if (p.settings.regularGrid) {
        if (posedCage.Size() != nodes)
            return Fail(error, "Lattice",
                        "a cage does not hold one point per lattice node");
    } else {
        const Points *rest = In<Points>(io, LatticeRestPoints());
        restCage = In<Points>(io, LatticeRestCage());
        if (!rest || !restCage)
            return Fail(error, "Lattice",
                        "a bound lattice needs rest points and a rest cage");
        if (rest->Size() != count)
            return Fail(error, "Lattice",
                        "rest points differ in count from the points");
        if (restCage->Size() != nodes || posedCage.Size() != nodes)
            return Fail(error, "Lattice",
                        "a cage does not hold one point per lattice node");
        for (const Points *cage : {restCage, &posedCage})
            for (const Vec3f &v : *cage)
                if (!std::isfinite(v[0]) || !std::isfinite(v[1]) ||
                    !std::isfinite(v[2]))
                    return Fail(error, "Lattice", "a cage point is not finite");
    }
    const float *weight = In<float>(io, LatticeDefaultWeight());
    const weightPacket::PointWeights *packet =
        In<weightPacket::PointWeights>(io, LatticePacket());
    if (weight && *weight != 1.0f) {
        if (!std::isfinite(*weight) || *weight < 0.0f || *weight > 1.0f)
            return Fail(error, "Lattice",
                        "a default weight is not finite or outside [0, 1]");
        state.dense.assign(count, *weight);
    }
    if (const auto *written = In<RigExecArray<int32_t>>(io, LatticeWritten())) {
        const rigExec::eval::EnvelopePacketView view =
            packet ? weightPacket::PacketView(*packet)
                   : rigExec::eval::EnvelopePacketView();
        bool holds = rigExec::eval::EnvelopeIsSparseWalk(view);
        size_t at = 0;
        for (size_t e = 0; holds && e < view.indexCount; ++e) {
            const int32_t index = view.indices[e];
            while (at + 1 < written->Size() && (*written)[at + 1] <= index)
                at += 2;
            holds = at + 1 < written->Size() && (*written)[at] <= index;
        }
        if (!holds)
            return Fail(error, "Lattice",
                        "the weights reach a vertex group the revision does "
                        "not write");
    }
    if (packet &&
        !rigExec::eval::EnvelopeIsFullStrength(
            weightPacket::PacketView(*packet)) &&
        !weightPacket::ResolvePacket(*packet, count, &state.dense))
        return Fail(error, "Lattice",
                    "the weight packet does not resolve over the points");
    if (!p.settings.regularGrid)
        state.basis = BasisOf(state, *In<Points>(io, LatticeRestPoints()),
                              *restCage, posedCage, p);
    return true;
}

bool
LatticeRange(const rigExec::RigExecOpBatch &batch, size_t k,
             const rigExec::RigExecOpRange &range, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const LatticeState &state = *static_cast<const LatticeState *>(
        rigExec::RigExecBatchState(batch, k));
    const Vec3f *from = RangeIn<Vec3f>(io, range, LatticePoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, LatticeOut());
    if (state.pass) {
        std::copy(from, from + (range.end - range.begin), out);
        return true;
    }
    const LatticeInstance &p = state.p;
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    const float *weights = state.dense.empty()
                               ? RangeIn<float>(io, range, LatticeEnvelope())
                               : state.dense.data() + range.begin;
    if (p.settings.regularGrid) {
        std::vector<Vec3f> points(from, from + (range.end - range.begin));
        if (!rigExec::RigExecApplyLatticeGridKernel<Vec3f, Vec3d>(
                &points, std::vector<Vec3f>(posedCage.begin(), posedCage.end()),
                p.divisions, p.settings, state.targetToLattice,
                state.latticeToTarget, state.cageToLattice))
            return Fail(error, "Lattice",
                        "the grid's settings, cage or spaces are "
                        "unusable");
        std::copy(points.begin(), points.end(), out);
        BlendEnvelopeAt(from, weights, out, range.end - range.begin);
        return true;
    }
    const Points &rest = *In<Points>(io, LatticeRestPoints());
    const Points &restCage = *In<Points>(io, LatticeRestCage());
    rigExec::RigExecApplyLatticeKernelGroup(
        from, out, range.count, range.begin, range.end, rest.Data(),
        rest.Size(), restCage.Data(), restCage.Size(), posedCage.Data(),
        posedCage.Size(), p.divisions[0], p.divisions[1], p.divisions[2],
        state.basis);
    BlendEnvelopeAt(from, weights, out, range.end - range.begin);
    return true;
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const LatticePayload &p = *static_cast<const LatticePayload *>(payload);
    const rigExec::RigExecLatticeSettings &s = p.settings;
    std::array<int32_t, 3> divisions;
    std::array<uint32_t, 3> interpolation;
    for (size_t a = 0; a < 3; ++a) {
        divisions[a] = int32_t(p.divisions[a]);
        interpolation[a] = uint32_t(s.interpolation[a]);
    }
    out.Set(kDivisions, divisions.data(), divisions.size());
    SetFlag(out, kRegularGrid, s.regularGrid);
    out.Set(kInterpolation, interpolation.data(), interpolation.size());
    out.Set(kOrigin, s.origin.data(), s.origin.size());
    out.Set(kSpacing, s.spacing.data(), s.spacing.size());
    out.Set(kStrength, s.strength);
    SetList(out, kMaskCount, s.mask);
    SetFlag(out, kWhole, p.whole);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Fixed(Value("divisions", rigExecStd::IntType()), 3),
    Flag("regularGrid"),
    Fixed(
        Enum("interpolation", rigExec::RigExecLatticeInterpolation::CatmullRom),
        3),
    Fixed(Value("origin", rigExecStd::FloatType()), 3),
    Fixed(Value("spacing", rigExecStd::FloatType()), 3),
    Value("strength", rigExecStd::FloatType()),
    Count("maskCount"),
    Items("mask", rigExecStd::FloatType(), kMaskCount),
    Flag("whole"),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&LatticePoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortPointwise, "points"},
    {&LatticeRestPoints(), &rigExecStd::Vec3fArray(), kOptional, "restPoints"},
    {&LatticeRestCage(), &rigExecStd::Vec3fArray(), kOptional, "restCage"},
    {&LatticePosedCage(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "posedCage"},
    {&LatticeTargetToLattice(), &rigExecStd::Mat4dType(), kOptional,
     "targetToLattice"},
    {&LatticeLatticeToTarget(), &rigExecStd::Mat4dType(), kOptional,
     "latticeToTarget"},
    {&LatticeCageToLattice(), &rigExecStd::Mat4dType(), kOptional,
     "cageToLattice"},
    {&LatticeEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&LatticeOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &LatticePoints()},
    {&LatticeEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&LatticeDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&LatticePacket(), &rigExecRig::PointWeightsType(), kOptional, "packet"},
    {&LatticeWritten(), &rigExecStd::IntArray(), kOptional, "written"},
    {&LatticeEvaluation(), &rigExecStd::IntType(), kOptional, "evaluation"},
    {&LatticeDivisions(), &rigExecStd::IntArray(), kOptional, "divisions"},
    {&LatticeInterpolation(), &rigExecStd::IntArray(), kOptional,
     "interpolation"},
    {&LatticeOrigin(), &rigExecStd::FloatArray(), kOptional, "origin"},
    {&LatticeSpacing(), &rigExecStd::FloatArray(), kOptional, "spacing"},
    {&LatticeStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&LatticeMask(), &rigExecStd::FloatArray(), kOptional, "mask"},
    {&LatticeCageMatrix(), &rigExecStd::Mat4dType(), kOptional, "cageMatrix"},
    {&LatticeTargetMatrix(), &rigExecStd::Mat4dType(), kOptional,
     "targetMatrix"},
    {&LatticePointSpace(), &rigExecStd::IntType(), kOptional, "pointSpace"},
    {&LatticeCageFrame(), &rigExecStd::Mat4dType(), kOptional, "cageFrame"},
    {&LatticeTargetFrame(), &rigExecStd::Mat4dType(), kOptional, "targetFrame"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Lattice";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.validate = &Validate;
    d.range.plan = &Plan;
    d.range.prepare = &Prepare;
    d.range.run = &LatticeRange;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
        rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    d.state = StateOf<LatticeState>();
    return d;
}();

} // namespace

void
RegisterLatticeOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Lattice(), kDesc);
}

} // namespace rigExecOps
