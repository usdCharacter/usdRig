// A built-in External mover run as its rigExecOps op through the scene's
// binding gives the bits its native handler gives: across the mover's
// options, under full, constant, dense and narrow envelopes, rebuilt when
// its settings move, and both refuse the same inputs.
#include "rigExec/api/registry.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecOps/deform/patchOp.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecScene/externalOps.h"
#include "rigExecScene/geometryOpRunner.h"
#include "rigExecScene/geometryProgram.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

using Type = RigExecRevisionLeafType;

RigExecWeightPacket
Dense(size_t count)
{
    RigExecWeightPacket w;
    w.representation = TfToken("dense");
    w.rangePolicy = TfToken("strict");
    w.valid = true;
    for (size_t i = 0; i < count; ++i)
        w.values.push_back(i % 5 == 0   ? 0.0f
                           : i % 7 == 0 ? 1.0f
                                        : 0.11f * float(i % 9));
    return w;
}

/// Weight on the first quarter only.
RigExecWeightPacket
Narrow(size_t count)
{
    RigExecWeightPacket w = Dense(count);
    for (size_t i = count / 4; i < count; ++i)
        w.values[i] = 0.0f;
    return w;
}

std::vector<std::pair<const char *, RigExecWeightPacket>>
Envelopes(size_t count)
{
    return {{"full", RigExecWeightPacket::Constant(1.0f)},
            {"constant", RigExecWeightPacket::Constant(0.37f)},
            {"dense", Dense(count)},
            {"narrow", Narrow(count)}};
}

/// One External mover revision: its record and one frame's values.
struct Revision {
    RigExecGeometryRecord record;
    std::vector<VtValue> declared;
    std::vector<GfVec3f> points, base;
    const GfMatrix4d *transform = nullptr;
    const std::vector<GfMatrix4d> *influences = nullptr;

    Revision(const TfToken &schema, std::vector<RigExecRevisionLeafKey> keys,
             std::vector<VtValue> values)
        : declared(std::move(values))
    {
        record.op = RigExecRevisionOp::External;
        record.binding.handler = RigExecFindMoverHandler(schema);
        record.binding.externalSchema = schema;
        record.binding.externalInputs = std::move(keys);
        RigExecDeclareRevisionLeaves(RigExecRevisionOp::External,
                                     SdfPath("/Rig/Mover"), record.binding,
                                     &record.leaves);
    }

    /// Every leaf: the declared inputs where they stand, each other
    /// read's fallback.
    std::vector<VtValue> Leaves() const
    {
        std::vector<VtValue> leaves;
        const auto &keys = record.leaves.keys;
        const int begin = record.leaves.externalBegin;
        for (size_t k = 0; k < keys.size(); ++k) {
            const int i = int(k) - begin;
            leaves.push_back(begin >= 0 && i >= 0 && size_t(i) < declared.size()
                                 ? declared[size_t(i)]
                                 : keys[k].fallback);
        }
        return leaves;
    }

    RigExecGeometryInputs Inputs(const std::vector<VtValue> &leaves,
                                 const RigExecWeightPacket &weights) const
    {
        RigExecGeometryInputs in;
        in.leaves = &leaves;
        in.transform = transform;
        in.influenceTransforms = influences;
        in.weights = &weights;
        in.basePoints = base.empty() ? points : base;
        return in;
    }
};

/// The native handler's run of \p r under \p weights.
bool
RunNative(const Revision &r, const RigExecWeightPacket &weights,
          std::vector<GfVec3f> *out)
{
    const std::vector<VtValue> leaves = r.Leaves();
    const RigExecGeometryInputs in = r.Inputs(leaves, weights);
    RigExecExternalRevision external;
    const RigExecMoverParameters p =
        RigExecAssembleGeometry(r.record, in, &external);
    RigExecGeometryWorkspace workspace;
    *out = r.points;
    return RigExecGeometryPacketMatches(r.record.op, p) &&
           RigExecRunGeometry(r.record, p, &workspace, out, true, &external);
}

/// The op's run of \p r under \p weights through \p runner.
bool
RunOp(const Revision &r, const RigExecWeightPacket &weights,
      RigExecGeometryOpRunner &runner, std::vector<GfVec3f> *out,
      std::string *error = nullptr)
{
    const std::vector<VtValue> leaves = r.Leaves();
    const RigExecGeometryInputs in = r.Inputs(leaves, weights);
    std::vector<VtValue> declared;
    const RigExecMoverParameters p =
        RigExecGatherExternalGeometryInputs(r.record, in, &declared);
    RigExecExternalOpValues values;
    values.inputs = &declared;
    values.transform = in.transform;
    values.influenceTransforms = in.influenceTransforms;
    values.basePoints = &in.basePoints;
    *out = r.points;
    return RigExecGeometryPacketMatches(r.record.op, p) &&
           RigExecRunExternalOp(r.record.binding, p, values, runner, *out,
                                error);
}

bool
SameBits(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(GfVec3f)) == 0);
}

/// \p r under every envelope: the op runs exactly when the native handler
/// does, with its bits. Returns how many envelopes ran.
int
Compare(const char *name, const Revision &r)
{
    int ran = 0;
    RigExecGeometryOpRunner runner;
    for (const auto &[envelope, weights] : Envelopes(r.points.size())) {
        std::vector<GfVec3f> native, op;
        std::string error;
        const bool nativeOk = RunNative(r, weights, &native);
        const bool opOk = RunOp(r, weights, runner, &op, &error);
        if (nativeOk != opOk || (nativeOk && !SameBits(native, op))) {
            std::printf("%s / %s: native %d, op %d (%s)\n", name, envelope,
                        int(nativeOk), int(opOk), error.c_str());
            CHECK(false);
        }
        ran += nativeOk ? 1 : 0;
    }
    return ran;
}

/// \p r refused by both under every envelope.
void
Refused(const char *name, const Revision &r)
{
    CHECK(Compare(name, r) == 0);
}

/// Points of a bumpy grid, as a mover's target.
std::vector<GfVec3f>
GridPoints(int nx, int ny)
{
    std::vector<GfVec3f> points;
    for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x)
            points.emplace_back(float(x) + 0.05f * std::sin(float(y)),
                                float(y) + 0.03f * std::cos(float(x)),
                                0.2f * std::sin(0.7f * float(x)));
    return points;
}

// --- Patch ------------------------------------------------------------------

/// A patch mover's declared inputs (rigExecScene/patchInputs.h).
struct PatchInputs {
    VtVec3fArray controls;
    VtDoubleArray pointWeights;
    int uCount = 4, vCount = 3, uOrder = 3, vOrder = 2;
    VtDoubleArray uKnots{0, 0, 0, 0.5, 1, 1, 1};
    VtDoubleArray vKnots{0, 0, 0.4, 1, 1};
    VtValue uForm = VtValue(TfToken("open"));
    VtValue vForm = VtValue(TfToken("open"));
    VtVec3fArray uv;
    std::vector<VtVec3fArray> shapes;
    /// One Dial per shape when connected, else the authored array.
    bool connected = false;
    std::vector<VtValue> dials;
    VtFloatArray authored;
    VtIntArray lower, upper;

    explicit PatchInputs(size_t pointCount)
    {
        for (int v = 0; v < vCount; ++v)
            for (int u = 0; u < uCount; ++u)
                controls.push_back(GfVec3f(float(u), float(v),
                                           0.3f * float((u * 7 + v * 3) % 5)));
        for (size_t i = 0; i < pointCount; ++i) {
            const float s = float(i) / float(pointCount);
            uv.push_back(GfVec3f(0.9f * s + 0.05f,
                                 0.5f + 0.4f * std::sin(9.0f * s), 0.25f));
        }
        for (int k = 0; k < 2; ++k) {
            VtVec3fArray shape(uv.size());
            for (size_t i = 0; i < uv.size(); ++i)
                shape[i] = uv[i] + GfVec3f(0.07f * float(k + 1),
                                           -0.11f * float((i + k) % 3), 1.0f);
            shapes.push_back(shape);
        }
        dials = {VtValue(0.35), VtValue(0.6f)};
        authored = {0.25f, -0.5f};
    }

    Revision Make(const std::vector<GfVec3f> &points) const
    {
        std::vector<RigExecRevisionLeafKey> keys;
        std::vector<VtValue> values;
        const auto add = [&](Type type, VtValue value) {
            RigExecRevisionLeafKey key;
            key.path = SdfPath("/Rig/Mover.in" + std::to_string(keys.size()));
            key.type = type;
            key.flavour = RigExecRevisionLeafFlavour::Resolved;
            keys.push_back(key);
            values.push_back(std::move(value));
        };
        add(Type::Vec3fArray, VtValue(controls));
        add(Type::DoubleArray, VtValue(pointWeights));
        add(Type::Int, VtValue(uCount));
        add(Type::Int, VtValue(vCount));
        add(Type::Int, VtValue(uOrder));
        add(Type::Int, VtValue(vOrder));
        add(Type::DoubleArray, VtValue(uKnots));
        add(Type::DoubleArray, VtValue(vKnots));
        add(Type::Token, uForm);
        add(Type::Token, vForm);
        add(Type::Vec3fArray, VtValue(uv));
        for (const VtVec3fArray &shape : shapes)
            add(Type::Vec3fArray, VtValue(shape));
        if (connected)
            for (const VtValue &dial : dials)
                add(Type::Dial, dial);
        else
            add(Type::FloatArray, VtValue(authored));
        add(Type::IntArray, VtValue(lower));
        add(Type::IntArray, VtValue(upper));
        Revision r(SchemaType::RigExecPatchMover, std::move(keys),
                   std::move(values));
        r.points = points;
        return r;
    }
};

void
TestPatch()
{
    const std::vector<GfVec3f> points = GridPoints(6, 5);
    const size_t n = points.size();
    const auto ran = [&](const char *name, const PatchInputs &in) {
        CHECK(Compare(name, in.Make(points)) == 4);
    };
    {
        PatchInputs in(n);
        ran("patch", in);
        in.shapes.clear();
        in.authored = {};
        ran("patch, no shapes", in);
    }
    {
        PatchInputs in(n);
        in.connected = true;
        ran("patch, connected weights", in);
        in.dials = {VtValue(0.0), VtValue(1.25)};
        ran("patch, a zero weight", in);
    }
    {
        PatchInputs in(n);
        in.authored = {};
        ran("patch, empty authored weights", in);
    }
    {
        PatchInputs in(n);
        in.uForm = VtValue(TfToken("periodic"));
        in.vForm = VtValue(std::string("open"));
        ran("patch, periodic u", in);
        in.uForm = VtValue(TfToken("open"));
        in.vForm = VtValue(std::string("closed"));
        ran("patch, closed v", in);
    }
    {
        PatchInputs in(n);
        for (size_t i = 0; i < in.controls.size(); ++i)
            in.pointWeights.push_back(0.5 + 0.25 * double(i % 4));
        ran("patch, rational", in);
    }
    {
        PatchInputs in(n);
        in.uv = VtVec3fArray(1, GfVec3f(0.3f, 0.6f, 0.0f));
        for (VtVec3fArray &shape : in.shapes)
            shape = VtVec3fArray(1, GfVec3f(0.5f, 0.2f, 0.0f));
        ran("patch, one UV point for all", in);
    }
    {
        PatchInputs in(n);
        // Pairs whose lower point stands above the upper one meet.
        for (int k = 0; k < 6; ++k) {
            in.lower.push_back(k);
            in.upper.push_back(int(n) - 1 - k);
        }
        in.connected = true;
        ran("patch, contacts", in);
    }

    // Refusals.
    const auto refused = [&](const char *name, const PatchInputs &in) {
        Refused(name, in.Make(points));
    };
    {
        PatchInputs in(n);
        in.uKnots = {0, 0, 0, 1, 1, 1};
        refused("patch, knot count", in);
    }
    {
        PatchInputs in(n);
        in.uKnots[3] = -1.0;
        refused("patch, descending knots", in);
    }
    {
        PatchInputs in(n);
        in.pointWeights = {1.0, 2.0};
        refused("patch, point weight count", in);
    }
    {
        PatchInputs in(n);
        in.uForm = VtValue(TfToken("bent"));
        refused("patch, unknown form", in);
    }
    {
        PatchInputs in(n);
        in.uv.pop_back();
        for (VtVec3fArray &shape : in.shapes)
            shape.pop_back();
        refused("patch, UV point count", in);
    }
    {
        PatchInputs in(n);
        in.shapes[1].pop_back();
        refused("patch, shape point count", in);
    }
    {
        PatchInputs in(n);
        in.authored = {0.5f};
        refused("patch, authored weight count", in);
    }
    {
        PatchInputs in(n);
        in.connected = true;
        in.dials[0] = VtValue(std::numeric_limits<double>::quiet_NaN());
        refused("patch, NaN weight", in);
        in.dials[0] = VtValue(int(1));
        refused("patch, weight of another type", in);
    }
    {
        PatchInputs in(n);
        in.lower = {0, 1};
        in.upper = {2};
        refused("patch, contact counts", in);
        in.upper = {2, 0};
        refused("patch, a point in two contacts", in);
        in.upper = {2, int(n)};
        refused("patch, contact out of range", in);
    }
    {
        PatchInputs in(n);
        in.uv[3][0] = std::numeric_limits<float>::infinity();
        refused("patch, infinite UV", in);
    }

    // One runner across frames whose settings move: rebuilt, never stale.
    {
        RigExecGeometryOpRunner runner;
        PatchInputs in(n);
        const RigExecWeightPacket full = RigExecWeightPacket::Constant(1.0f);
        std::vector<PatchInputs> frames(4, in);
        frames[1].uForm = VtValue(TfToken("periodic"));
        frames[2].shapes.pop_back();
        frames[2].authored = {0.75f};
        frames[3].vOrder = 3;
        frames[3].vKnots = {0, 0, 0, 1, 1, 1};
        for (size_t f = 0; f < frames.size(); ++f) {
            const Revision r = frames[f].Make(points);
            std::vector<GfVec3f> native, op;
            CHECK(RunNative(r, full, &native));
            CHECK(RunOp(r, full, runner, &op));
            CHECK(SameBits(native, op));
        }
    }
}

/// The ops' capabilities mirror the External revision's classification.
void
TestCapabilities()
{
    using Op = RigExecRevisionOp;
    RigExecMoverParameters gated;
    gated.weights.valid = true;
    gated.weights.representation = TfToken("sparse");
    gated.weights.defaultWeight = 0.0f;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const TfToken *schema : {&SchemaType::RigExecPatchMover}) {
        RigExecRevisionBinding binding;
        binding.externalSchema = *schema;
        const RigExecOpHandle *handle = RigExecExternalOpHandle(binding);
        CHECK(handle && handle->IsValid());
        if (!handle)
            continue;
        const RigExecOpDesc &desc = *registry.Op(*handle).desc;
        const auto has = [&](uint32_t bit) {
            return (desc.capabilities & bit) != 0;
        };
        CHECK(desc.threading == RigExecOpThreading::Pure);
        CHECK(RigExecRevisionTakesSeparateBlend(Op::External,
                                                RigExecWeightPacket()) ==
              has(kRigExecOpSeparateBlend));
        CHECK(RigExecRevisionIsRangeOp(Op::External) == has(kRigExecOpRanged));
        CHECK(RigExecRevisionGateHolds(Op::External, gated) ==
              has(kRigExecOpGate));
        CHECK(RigExecRevisionOpAssemblesFromLeaves(Op::External) ==
              has(kRigExecOpFromLeaves));
        CHECK(RigExecRevisionOpBakes(Op::External, false) ==
              has(kRigExecOpBakeable));
        CHECK(RigExecRevisionOpFreezes(Op::External, false) ==
              has(kRigExecOpFreezable));
    }
    RigExecRevisionBinding other;
    other.externalSchema = SchemaType::RigExecMatrixMover;
    CHECK(!RigExecExternalOpHandle(other));
}

} // namespace

int
main()
{
    std::string registryError;
    if (!RigExecEnsureBuiltinRegistry(&registryError)) {
        std::printf("%s\n", registryError.c_str());
        return 1;
    }
    TestPatch();
    TestCapabilities();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
