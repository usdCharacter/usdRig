// A built-in External mover run as its rigExecOps op through the scene's
// binding gives the bits its native handler gives: across the mover's
// options, under full, constant, dense and narrow envelopes, rebuilt when
// its settings move, and both refuse the same inputs.
#include "rigExec/api/registry.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExecBinary/clothPlayback.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecScene/externalOps.h"
#include "opRunners/geometryOpRunner.h"
#include "rigExecScene/geometryProgram.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
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
    /// The provider transform and influence transforms, fed when set.
    bool transformed = false, framed = false;
    GfMatrix4d transform;
    std::vector<GfMatrix4d> influences;

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
        in.transform = transformed ? &transform : nullptr;
        in.influenceTransforms = framed ? &influences : nullptr;
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

// --- Cloth ------------------------------------------------------------------

/// A cloth mover's declared inputs (rigExecBinary/clothPlayback.h's
/// RigExecClothInput) with no constraint-set layout: a 5 x 4 curtain
/// pinned along its top row, and colliders below it.
struct ClothInputs {
    VtIntArray counts, indices;
    VtVec3fArray rest;
    VtFloatArray pins;
    int windowFrames = 5, maxWindowFrames = 5, substeps = 3, iterations = 1;
    bool tethers = true;
    float tetherSlack = 0.05f;
    double frameRate = 24.0;
    float damping = 2.0f, attract = 0.05f;
    GfVec3f gravity = GfVec3f(0.0f, -9.81f, 0.0f);
    struct Collider {
        VtVec3fArray points;
        VtIntArray counts{4}, indices{0, 1, 2, 3};
    };
    std::vector<Collider> colliders;

    ClothInputs()
    {
        for (int y = 0; y < 4; ++y)
            for (int x = 0; x < 5; ++x) {
                rest.push_back(
                    GfVec3f(0.25f * float(x), -0.25f * float(y), 0.0f));
                pins.push_back(y == 0 ? 1.0f : 0.0f);
            }
        for (int y = 0; y + 1 < 4; ++y)
            for (int x = 0; x + 1 < 5; ++x) {
                const int a = y * 5 + x;
                counts.push_back(4);
                for (int i : {a, a + 1, a + 6, a + 5})
                    indices.push_back(i);
            }
        Collider floor;
        floor.points = {GfVec3f(-1, -0.6f, -1), GfVec3f(2, -0.6f, -1),
                        GfVec3f(2, -0.6f, 1), GfVec3f(-1, -0.6f, 1)};
        colliders.push_back(floor);
    }

    /// The cloth at frame \p t: the curtain's entering points swing, the
    /// colliders rise and the damping changes.
    Revision Make(int t) const
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
        add(Type::IntArray, VtValue(counts));
        add(Type::IntArray, VtValue(indices));
        add(Type::Token, VtValue(TfToken("rightHanded")));
        add(Type::Vec3fArray, VtValue(rest));
        add(Type::FloatArray, VtValue(pins));
        add(Type::Int, VtValue(windowFrames));
        add(Type::Int, VtValue(maxWindowFrames));
        add(Type::Int, VtValue(substeps));
        add(Type::Int, VtValue(iterations));
        add(Type::Bool, VtValue(tethers));
        add(Type::Float, VtValue(tetherSlack));
        add(Type::Double, VtValue(frameRate));
        add(Type::Float, VtValue(0.0f));
        add(Type::Float, VtValue(1.0e-3f));
        add(Type::Vec3f, VtValue(gravity));
        add(Type::Float, VtValue(damping + 0.1f * float(t % 4)));
        add(Type::Float, VtValue(attract));
        add(Type::Float, VtValue(0.02f));
        add(Type::Float, VtValue(0.05f));
        add(Type::Float, VtValue(0.3f));
        for (const Collider &c : colliders) {
            VtVec3fArray moved = c.points;
            for (GfVec3f &p : moved)
                p[1] += 0.03f * float(t);
            add(Type::Vec3fArray, VtValue(moved));
            add(Type::IntArray, VtValue(c.counts));
            add(Type::IntArray, VtValue(c.indices));
            add(Type::Token, VtValue(TfToken("rightHanded")));
        }
        Revision r(SchemaType::RigExecClothMover, std::move(keys),
                   std::move(values));
        r.record.binding.externalCompileData = VtValue(24.0);
        const float swing = 0.2f * std::sin(0.7f * float(t));
        for (const GfVec3f &p : rest)
            r.points.push_back(GfVec3f(p[0] + swing, p[1], p[2]));
        return r;
    }
};

/// One evaluator's history of a windowed revision.
struct History {
    RigExecWindowRing ring;
    RigExecWindowPacketPtr capture;
    std::shared_ptr<const void> cache;
    size_t frames = 0;

    RigExecWindowRun Run(bool captureOnly)
    {
        RigExecWindowRun run;
        run.ring = &ring;
        run.capture = &capture;
        run.cache = &cache;
        run.captureOnly = captureOnly;
        return run;
    }
    /// A host's PushHistory for this window.
    void Push()
    {
        ring.push_back(capture);
        while (ring.size() > frames)
            ring.erase(ring.begin());
    }
};

bool
RunNativeWindow(const Revision &r, const RigExecWeightPacket &weights,
                History &history, bool captureOnly, std::vector<GfVec3f> *out)
{
    const std::vector<VtValue> leaves = r.Leaves();
    const RigExecGeometryInputs in = r.Inputs(leaves, weights);
    RigExecExternalRevision external;
    const RigExecMoverParameters p =
        RigExecAssembleGeometry(r.record, in, &external);
    RigExecWindowRun run = history.Run(captureOnly);
    external.window = &run;
    RigExecGeometryWorkspace workspace;
    *out = r.points;
    return RigExecGeometryPacketMatches(r.record.op, p) &&
           RigExecRunGeometry(r.record, p, &workspace, out, true, &external);
}

bool
RunOpWindow(const Revision &r, const RigExecWeightPacket &weights,
            History &history, bool captureOnly, RigExecGeometryOpRunner &runner,
            std::vector<GfVec3f> *out, std::string *error)
{
    const std::vector<VtValue> leaves = r.Leaves();
    const RigExecGeometryInputs in = r.Inputs(leaves, weights);
    std::vector<VtValue> declared;
    const RigExecMoverParameters p =
        RigExecGatherExternalGeometryInputs(r.record, in, &declared);
    RigExecExternalOpValues values;
    values.inputs = &declared;
    values.basePoints = &in.basePoints;
    RigExecWindowRun run = history.Run(captureOnly);
    *out = r.points;
    return RigExecGeometryPacketMatches(r.record.op, p) &&
           RigExecRunExternalOp(r.record.binding, p, values, runner, *out,
                                error, &run);
}

/// The native cloth and its op played side by side: \p ticks lists each
/// tick's frame, negative for a jump to minus it (the history cleared, then
/// rebuilt from the frames before it by record-only ticks), and each tick
/// must give the same points and record. Where the native tick refuses,
/// and on every tick of inputs the op cannot take, the op's evaluation
/// fails with an error holding \p expect; the host then starts both
/// histories over. \p edit, when set, changes the inputs before the tick it
/// names. Returns how many ticks solved.
int
PlayCloth(const char *name, ClothInputs inputs, const std::vector<int> &ticks,
          const RigExecWeightPacket &weights,
          const std::function<void(int, ClothInputs &)> &edit = {},
          const char *expect = nullptr)
{
    History native, op;
    native.frames = op.frames = size_t(RigExecClothHistoryFrames(
        RigExecClothSettings{inputs.windowFrames, inputs.maxWindowFrames}));
    RigExecGeometryOpRunner runner;
    int solved = 0;
    const auto tick = [&](int t, bool captureOnly) {
        if (edit)
            edit(t, inputs);
        const Revision r = inputs.Make(t);
        std::vector<GfVec3f> a, b;
        std::string error;
        const bool nativeOk =
            RunNativeWindow(r, weights, native, captureOnly, &a);
        const bool opOk =
            RunOpWindow(r, weights, op, captureOnly, runner, &b, &error);
        const bool same =
            opOk ? nativeOk && SameBits(a, b) &&
                       RigExecWindowPacketsEqual(native.capture, op.capture)
                 : (!nativeOk || captureOnly) && expect &&
                       error.find(expect) != std::string::npos;
        if (!same) {
            std::printf("%s, t=%d%s: native %d, op %d (%s)\n", name, t,
                        captureOnly ? " (record only)" : "", int(nativeOk),
                        int(opOk), error.c_str());
            CHECK(false);
        }
        solved += opOk && !captureOnly ? 1 : 0;
        if (!opOk) {
            native.ring.clear();
            op.ring.clear();
            native.capture.reset();
            op.capture.reset();
            return;
        }
        native.Push();
        op.Push();
    };
    for (const int t : ticks) {
        if (t >= 0) {
            tick(t, false);
            continue;
        }
        native.ring.clear();
        op.ring.clear();
        for (int k = int(native.frames); k >= 1; --k)
            tick(-t - k, true);
        tick(-t, false);
    }
    return solved;
}

void
TestCloth()
{
    const ClothInputs inputs;
    const std::vector<int> play = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    const size_t n = inputs.rest.size();
    for (const auto &[envelope, weights] : Envelopes(n)) {
        const std::string name = std::string("cloth / ") + envelope;
        CHECK(PlayCloth(name.c_str(), inputs, play, weights) > 0);
    }
    const RigExecWeightPacket full = RigExecWeightPacket::Constant(1.0f);
    CHECK(PlayCloth("cloth, jumps", inputs, {3, 4, -9, 10, -2, 3, 4}, full) >
          0);
    {
        ClothInputs bare = inputs;
        bare.colliders.clear();
        bare.pins = {};
        bare.tethers = false;
        CHECK(PlayCloth("cloth, no colliders or pins", bare, play, full) > 0);
    }
    {
        ClothInputs two = inputs;
        ClothInputs::Collider wall;
        wall.points = {GfVec3f(1.2f, -1, -1), GfVec3f(1.2f, 1, -1),
                       GfVec3f(1.2f, 1, 1), GfVec3f(1.2f, -1, 1)};
        two.colliders.push_back(wall);
        two.windowFrames = 3;
        two.maxWindowFrames = 8;
        CHECK(PlayCloth("cloth, two colliders, reaching back", two, play,
                        full) > 0);
    }
    // Settings that move mid-playback rebuild the op's compiled rig; the
    // history stays the host's.
    CHECK(PlayCloth("cloth, substeps change", inputs, play, full,
                    [](int t, ClothInputs &in) {
                        if (t == 6)
                            in.substeps = 5;
                    }) > 0);
    CHECK(PlayCloth("cloth, pins change", inputs, play, full,
                    [](int t, ClothInputs &in) {
                        if (t == 5)
                            in.pins[7] = 1.0f;
                    }) > 0);
    // A record that cannot be made, an unusable mesh and an envelope that
    // does not resolve fail the op's evaluation with an error.
    CHECK(PlayCloth(
              "cloth, collider points missing", inputs, play, full,
              [](int t, ClothInputs &in) {
                  if (t == 4)
                      in.colliders[0].points = {GfVec3f(0)};
              },
              "Cloth: ") > 0);
    {
        ClothInputs broken = inputs;
        broken.counts[0] = 9;
        CHECK(PlayCloth("cloth, unusable mesh", broken, {0, 1, 2}, full, {},
                        "Cloth: its compiled rig is unusable") == 0);
    }
    {
        RigExecWeightPacket bad = Dense(n);
        bad.values.pop_back();
        CHECK(PlayCloth("cloth, envelope does not resolve", inputs,
                        {0, 1, 2, 3}, bad, {},
                        "envelope does not resolve over the points") == 0);
    }
}

/// The ops' capabilities mirror the External revision's classification.
// --- SurfaceBinding ---------------------------------------------------------

/// A surface binding mover's declared inputs
/// (rigExecScene/surfaceBindingInputs.h): four of eight target points
/// bound by two polygon or limit stencils each on a 4 x 3 grid surface.
struct SurfaceBindingInputs {
    VtIntArray vertices{1, 3, 4, 6}, vertexOffsets{0, 2, 4, 6, 8};
    VtIntArray polygonOffsets, pointIndices, limitOffsets, limitIndices;
    VtFloatArray barycentric, bindingWeights, mask, deltaMultipliers;
    VtFloatArray limitWeights, limitDu, limitDv;
    VtVec3fArray offsets;
    VtValue mode = VtValue(TfToken("polygon"));
    VtValue normal = VtValue(TfToken("geometric"));
    float strength = 1.0f, deltaMultiplier = 1.0f;
    GfMatrix4d surfaceRest, targetRest, surfaceToBinding;
    VtVec3fArray surface;
    VtIntArray counts, indices;
    std::vector<GfMatrix4d> frames;
    bool framed = false;

    SurfaceBindingInputs()
    {
        for (int y = 0; y < 3; ++y)
            for (int x = 0; x < 4; ++x)
                surface.push_back(GfVec3f(float(x), float(y),
                                          0.2f * float((x * 5 + y * 3) % 4)));
        for (int y = 0; y + 1 < 3; ++y)
            for (int x = 0; x + 1 < 4; ++x) {
                const int a = y * 4 + x;
                counts.push_back(4);
                for (int i : {a, a + 1, a + 5, a + 4})
                    indices.push_back(i);
            }
        polygonOffsets.push_back(0);
        limitOffsets.push_back(0);
        for (int s = 0; s < 8; ++s) {
            const int face = (s * 5) % 6;
            for (int k = 0; k < 4; ++k) {
                pointIndices.push_back(indices[size_t(face * 4 + k)]);
                limitIndices.push_back(indices[size_t(face * 4 + (k + s) % 4)]);
                barycentric.push_back(k == 0 ? 0.4f : 0.2f);
                limitWeights.push_back(0.1f + 0.2f * float(k));
                limitDu.push_back(k < 2 ? -0.5f : 0.5f);
                limitDv.push_back(k % 2 ? 0.5f : -0.5f);
            }
            polygonOffsets.push_back(int(pointIndices.size()));
            limitOffsets.push_back(int(limitIndices.size()));
            bindingWeights.push_back(s % 2 ? 0.35f : 0.65f);
            offsets.push_back(GfVec3f(0.1f * float(s), -0.05f, 0.3f));
        }
        surfaceRest.SetTranslate(GfVec3d(0.5, -0.25, 1.0));
        targetRest.SetRotate(GfRotation(GfVec3d(0, 0, 1), 30.0));
        surfaceToBinding.SetScale(GfVec3d(1.0, 2.0, 0.5));
        frames = {GfMatrix4d().SetTranslate(GfVec3d(0.0, 0.5, 0.0)),
                  GfMatrix4d().SetRotate(GfRotation(GfVec3d(1, 0, 0), 15.0))};
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
        for (const VtIntArray *a :
             {&vertices, &vertexOffsets, &polygonOffsets, &pointIndices,
              &limitOffsets, &limitIndices})
            add(Type::IntArray, VtValue(*a));
        for (const VtFloatArray *a :
             {&barycentric, &bindingWeights, &mask, &deltaMultipliers,
              &limitWeights, &limitDu, &limitDv})
            add(Type::FloatArray, VtValue(*a));
        add(Type::Vec3fArray, VtValue(offsets));
        add(Type::Token, mode);
        add(Type::Token, normal);
        add(Type::Float, VtValue(strength));
        add(Type::Float, VtValue(deltaMultiplier));
        add(Type::Matrix4d, VtValue(surfaceRest));
        add(Type::Matrix4d, VtValue(targetRest));
        add(Type::Matrix4d, VtValue(surfaceToBinding));
        add(Type::Vec3fArray, VtValue(surface));
        add(Type::IntArray, VtValue(counts));
        add(Type::IntArray, VtValue(indices));
        Revision r(SchemaType::RigExecSurfaceBindingMover, std::move(keys),
                   std::move(values));
        r.points = points;
        r.framed = framed;
        r.influences = frames;
        return r;
    }
};

void
TestSurfaceBinding()
{
    const std::vector<GfVec3f> points = GridPoints(4, 2);
    const auto ran = [&](const char *name, const SurfaceBindingInputs &in) {
        CHECK(Compare(name, in.Make(points)) == 4);
    };
    const auto refused = [&](const char *name, const SurfaceBindingInputs &in) {
        Refused(name, in.Make(points));
    };
    for (const char *mode : {"polygon", "limit"})
        for (const char *normal : {"geometric", "smooth"}) {
            SurfaceBindingInputs in;
            in.mode = VtValue(TfToken(mode));
            in.normal = VtValue(std::string(normal));
            const std::string name =
                std::string("surface binding, ") + mode + ", " + normal;
            ran(name.c_str(), in);
            in.framed = true;
            ran((name + ", framed").c_str(), in);
        }
    {
        SurfaceBindingInputs in;
        in.mask = VtFloatArray(points.size(), 0.5f);
        in.mask[3] = 0.0f;
        in.mask[4] = 1.0f;
        in.deltaMultipliers = VtFloatArray(points.size(), 1.5f);
        in.strength = 0.75f;
        in.deltaMultiplier = -0.5f;
        ran("surface binding, mask and multipliers", in);
    }
    {
        SurfaceBindingInputs in;
        in.vertices = {};
        in.vertexOffsets = {0};
        in.polygonOffsets = {0};
        in.pointIndices = {};
        in.barycentric = {};
        in.bindingWeights = {};
        in.offsets = {};
        ran("surface binding, nothing bound", in);
    }

    {
        SurfaceBindingInputs in;
        in.vertices[2] = int(points.size());
        refused("surface binding, vertex out of range", in);
        in.vertices[2] = 1;
        refused("surface binding, vertex bound twice", in);
    }
    {
        SurfaceBindingInputs in;
        in.vertexOffsets[1] = 0;
        refused("surface binding, empty stencil row", in);
    }
    {
        SurfaceBindingInputs in;
        in.pointIndices[5] = int(in.surface.size());
        refused("surface binding, surface point out of range", in);
    }
    {
        SurfaceBindingInputs in;
        in.bindingWeights.pop_back();
        refused("surface binding, binding weight count", in);
    }
    {
        SurfaceBindingInputs in;
        in.mask = VtFloatArray(points.size(), 1.5f);
        refused("surface binding, mask above one", in);
        in.mask = VtFloatArray(2, 0.5f);
        refused("surface binding, mask count", in);
    }
    {
        SurfaceBindingInputs in;
        in.offsets[2][1] = std::numeric_limits<float>::quiet_NaN();
        refused("surface binding, NaN offset", in);
    }
    {
        SurfaceBindingInputs in;
        in.barycentric[0] = std::numeric_limits<float>::infinity();
        refused("surface binding, infinite weight", in);
    }
    {
        SurfaceBindingInputs in;
        in.surfaceRest.SetScale(GfVec3d(0.0, 1.0, 1.0));
        refused("surface binding, singular surface rest", in);
    }
    {
        SurfaceBindingInputs in;
        in.framed = true;
        in.frames.pop_back();
        refused("surface binding, one frame", in);
    }
    {
        SurfaceBindingInputs in;
        in.normal = VtValue(TfToken("smooth"));
        in.counts[0] = 2;
        refused("surface binding, two-sided face", in);
    }
    {
        SurfaceBindingInputs in;
        in.mode = VtValue(TfToken("subdivided"));
        refused("surface binding, unknown mode", in);
        in.mode = VtValue(TfToken("limit"));
        in.limitOffsets[1] = 0;
        refused("surface binding, empty limit row", in);
    }
    {
        SurfaceBindingInputs in;
        in.strength = std::numeric_limits<float>::infinity();
        refused("surface binding, infinite strength", in);
    }

    // Settings that move between frames rebuild the node.
    {
        RigExecGeometryOpRunner runner;
        const RigExecWeightPacket full = RigExecWeightPacket::Constant(1.0f);
        std::vector<SurfaceBindingInputs> frames(4);
        frames[1].mode = VtValue(TfToken("limit"));
        frames[2].normal = VtValue(TfToken("smooth"));
        frames[3].framed = true;
        for (const SurfaceBindingInputs &in : frames) {
            const Revision r = in.Make(points);
            std::vector<GfVec3f> native, op;
            CHECK(RunNative(r, full, &native));
            CHECK(RunOp(r, full, runner, &op));
            CHECK(SameBits(native, op));
        }
    }
}

// --- LayeredSkin ------------------------------------------------------------

/// A layered skin mover's declared inputs (rigExecScene/layeredSkinInputs.h)
/// over three influences, two slots per point.
struct LayeredSkinInputs {
    VtIntArray indices;
    VtFloatArray weights, mask;
    int elementSize = 2;
    VtValue method = VtValue(TfToken("classicLinear"));
    bool useBase = false, transformInput = false;
    std::vector<GfMatrix4d> influences;
    GfMatrix4d transform;
    bool framed = true, transformed = true;

    explicit LayeredSkinInputs(size_t pointCount)
    {
        for (size_t i = 0; i < pointCount; ++i) {
            indices.push_back(int(i % 3));
            indices.push_back(int((i + 1) % 3));
            const float w = 0.15f + 0.1f * float(i % 7);
            weights.push_back(w);
            weights.push_back(i % 5 == 0 ? 0.0f : 1.0f - w);
        }
        influences = {
            GfMatrix4d().SetRotate(GfRotation(GfVec3d(0, 0, 1), 25.0)),
            GfMatrix4d().SetTranslate(GfVec3d(0.5, -1.0, 2.0)),
            GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), -40.0), GfVec3d(1, 2, 3))};
        transform = GfMatrix4d(GfRotation(GfVec3d(0, 1, 0), 10.0),
                               GfVec3d(-0.25, 0.0, 0.5));
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
        add(Type::IntArray, VtValue(indices));
        add(Type::FloatArray, VtValue(weights));
        add(Type::Int, VtValue(elementSize));
        add(Type::Token, method);
        add(Type::FloatArray, VtValue(mask));
        add(Type::Bool, VtValue(useBase));
        add(Type::Bool, VtValue(transformInput));
        Revision r(SchemaType::RigExecLayeredSkinMover, std::move(keys),
                   std::move(values));
        r.points = points;
        for (const GfVec3f &p : points)
            r.base.push_back(p * 0.5f + GfVec3f(0.1f, 0.2f, 0.3f));
        r.framed = framed;
        r.influences = influences;
        r.transformed = transformed;
        r.transform = transform;
        return r;
    }
};

void
TestLayeredSkin()
{
    const std::vector<GfVec3f> points = GridPoints(5, 4);
    const size_t n = points.size();
    const auto ran = [&](const char *name, const LayeredSkinInputs &in) {
        CHECK(Compare(name, in.Make(points)) == 4);
    };
    const auto refused = [&](const char *name, const LayeredSkinInputs &in) {
        Refused(name, in.Make(points));
    };
    for (const char *method : {"classicLinear", "dualQuaternion"}) {
        LayeredSkinInputs in(n);
        in.method = VtValue(std::string(method));
        const std::string name = std::string("layered skin, ") + method;
        ran(name.c_str(), in);
        in.useBase = true;
        ran((name + ", from base").c_str(), in);
        in.transformInput = true;
        ran((name + ", from base, input transform").c_str(), in);
        in.mask = VtFloatArray(n, 0.5f);
        in.mask[0] = 0.0f;
        in.mask[1] = 1.0f;
        ran((name + ", masked").c_str(), in);
    }
    {
        LayeredSkinInputs in(n);
        in.elementSize = 1;
        in.indices.resize(n);
        in.weights.resize(n);
        ran("layered skin, one slot", in);
    }
    {
        LayeredSkinInputs in(n);
        in.transformed = false;
        ran("layered skin, no transform", in);
    }

    {
        LayeredSkinInputs in(n);
        in.weights.pop_back();
        refused("layered skin, weight count", in);
    }
    {
        LayeredSkinInputs in(n);
        in.elementSize = 0;
        refused("layered skin, no slots", in);
        in.elementSize = 3;
        refused("layered skin, slot count", in);
    }
    {
        LayeredSkinInputs in(n);
        in.indices[4] = 3;
        refused("layered skin, index out of range", in);
    }
    {
        LayeredSkinInputs in(n);
        in.weights[2] = -0.25f;
        refused("layered skin, negative weight", in);
    }
    {
        LayeredSkinInputs in(n);
        in.mask = VtFloatArray(n, 1.25f);
        refused("layered skin, mask above one", in);
        in.mask = VtFloatArray(3, 0.5f);
        refused("layered skin, mask count", in);
    }
    {
        LayeredSkinInputs in(n);
        in.method = VtValue(TfToken("spline"));
        refused("layered skin, unknown method", in);
    }
    {
        LayeredSkinInputs in(n);
        in.framed = false;
        refused("layered skin, no influences", in);
        in.framed = true;
        in.influences[1][0][3] = 0.5;
        refused("layered skin, projective influence", in);
    }
    {
        LayeredSkinInputs in(n);
        in.transformInput = true;
        in.transformed = false;
        refused("layered skin, input transform missing", in);
    }

    // Settings that move between frames rebuild the node.
    {
        RigExecGeometryOpRunner runner;
        const RigExecWeightPacket full = RigExecWeightPacket::Constant(1.0f);
        std::vector<LayeredSkinInputs> frames(3, LayeredSkinInputs(n));
        frames[1].method = VtValue(TfToken("dualQuaternion"));
        frames[2].elementSize = 1;
        frames[2].indices.resize(n);
        frames[2].weights.resize(n);
        for (const LayeredSkinInputs &in : frames) {
            const Revision r = in.Make(points);
            std::vector<GfVec3f> native, op;
            CHECK(RunNative(r, full, &native));
            CHECK(RunOp(r, full, runner, &op));
            CHECK(SameBits(native, op));
        }
    }
}

void
TestCapabilities()
{
    using Op = RigExecRevisionOp;
    RigExecMoverParameters gated;
    gated.weights.valid = true;
    gated.weights.representation = TfToken("sparse");
    gated.weights.defaultWeight = 0.0f;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const TfToken *schema : {&SchemaType::RigExecPatchMover,
                                  &SchemaType::RigExecSurfaceBindingMover,
                                  &SchemaType::RigExecLayeredSkinMover,
                                  &SchemaType::RigExecClothMover}) {
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
        CHECK(desc.threading == (*schema == SchemaType::RigExecClothMover
                                     ? RigExecOpThreading::Windowed
                                     : RigExecOpThreading::Pure));
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
    TestSurfaceBinding();
    TestLayeredSkin();
    TestCloth();
    TestCapabilities();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
