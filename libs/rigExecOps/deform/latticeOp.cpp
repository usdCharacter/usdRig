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
};

/// Instance \p k's settings, its mask read in place.
struct LatticeInstance {
    decltype(LatticePayload::divisions) divisions{};
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
    p.settings.strength = ValueAt<float>(batch, kStrength, k);
    const auto mask = ListAt<float>(batch, kMask, k);
    p.settings.mask = {mask.data, mask.Size()};
    return p;
}

/// A node's bind of its rest points into the rest cage, built on the run
/// that first sees those rest values; this frame's pass through and the
/// weight at every point when a default weight or a packet gives them.
struct LatticeState {
    std::shared_ptr<const rigExec::RigExecLatticeBind<Vec3f>> bind;
    /// This frame's basis for every range, or null to stream the factors.
    const rigExec::RigExecLatticeBasis *basis = nullptr;
    bool pass = false;
    std::vector<float> dense;
};

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
    return FlagAt(batch, kRegularGrid, k) ? rigExec::RigExecRangePlan::Whole
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
        return envelopes == int(Fed(check, LatticeEnvelope())) ||
               Fail(error, "Lattice", "a regular grid weighs by an envelope");
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
    if (state.pass)
        return true;
    const Points &in = *In<Points>(io, LatticePoints());
    if (!EnvelopeFits(in, In<Envelope>(io, LatticeEnvelope()), "Lattice",
                      error))
        return false;
    if (FlagAt(batch, kRegularGrid, k))
        return true;
    const size_t count = in.Size();
    if (In<Points>(io, LatticeRestPoints())->Size() != count)
        return Fail(error, "Lattice",
                    "rest points differ in count from the points");
    const LatticeInstance p = PayloadAt(batch, k);
    const Points &restCage = *In<Points>(io, LatticeRestCage());
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    const size_t nodes = size_t(p.divisions[0]) * size_t(p.divisions[1]) *
                         size_t(p.divisions[2]);
    if (restCage.Size() != nodes || posedCage.Size() != nodes)
        return Fail(error, "Lattice",
                    "a cage does not hold one point per lattice node");
    for (const Points *cage : {&restCage, &posedCage})
        for (const Vec3f &v : *cage)
            if (!std::isfinite(v[0]) || !std::isfinite(v[1]) ||
                !std::isfinite(v[2]))
                return Fail(error, "Lattice", "a cage point is not finite");
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
    state.basis = BasisOf(state, *In<Points>(io, LatticeRestPoints()), restCage,
                          posedCage, p);
    return true;
}

bool
LatticeRange(const rigExec::RigExecOpBatch &batch, size_t k,
             const rigExec::RigExecOpRange &range, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const LatticeState &state = *static_cast<const LatticeState *>(
        rigExec::RigExecBatchState(batch, k));
    if (state.pass) {
        const Vec3f *from = RangeIn<Vec3f>(io, range, LatticePoints());
        std::copy(from, from + (range.end - range.begin),
                  RangeOut<Vec3f>(io, range, LatticeOut()));
        return true;
    }
    const LatticeInstance p = PayloadAt(batch, k);
    const Points &in = *In<Points>(io, LatticePoints());
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    const Envelope *envelope = In<Envelope>(io, LatticeEnvelope());
    if (p.settings.regularGrid) {
        std::vector<Vec3f> points(in.begin(), in.end());
        if (!rigExec::RigExecApplyLatticeGridKernel<Vec3f, Vec3d>(
                &points, std::vector<Vec3f>(posedCage.begin(), posedCage.end()),
                p.divisions, p.settings,
                MatrixOr1(In<Mat4d>(io, LatticeTargetToLattice())),
                MatrixOr1(In<Mat4d>(io, LatticeLatticeToTarget())),
                MatrixOr1(In<Mat4d>(io, LatticeCageToLattice()))))
            return Fail(error, "Lattice",
                        "the grid's settings, cage or spaces are "
                        "unusable");
        return WritePoints(io, LatticeOut(), in, points, envelope, "Lattice",
                           error);
    }
    const Points &rest = *In<Points>(io, LatticeRestPoints());
    const Points &restCage = *In<Points>(io, LatticeRestCage());
    const Vec3f *from = RangeIn<Vec3f>(io, range, LatticePoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, LatticeOut());
    rigExec::RigExecApplyLatticeKernelGroup(
        from, out, range.count, range.begin, range.end, rest.Data(),
        rest.Size(), restCage.Data(), restCage.Size(), posedCage.Data(),
        posedCage.Size(), p.divisions[0], p.divisions[1], p.divisions[2],
        state.basis);
    BlendEnvelopeAt(from,
                    state.dense.empty()
                        ? RangeIn<float>(io, range, LatticeEnvelope())
                        : state.dense.data() + range.begin,
                    out, range.end - range.begin);
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
