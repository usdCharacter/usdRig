#include "rigExecMath/geometryKernels.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/moverGraph.h"
#include "rigExec/parallel.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"
#include <cstring>
#include <cstdio>
#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/usdGeom/mesh.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <atomic>
#if defined(_M_X64) || defined(_M_IX86) || defined(__SSE2__)
#include <xmmintrin.h>
#endif
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
#include "rigExecRuntimeDrive.h"
#include "deltaMushReferenceFixtures.h"
static bool Near(const std::vector<GfVec3f> &a,const std::vector<GfVec3f> &b,double e=1e-5) {
    if(a.size()!=b.size())return false;
    for(size_t i=0;i<a.size();++i)if((a[i]-b[i]).GetLength()>e)return false;
    return true;
}
static void CheckReferenceFixtures() {
    for (const auto &fixture : DeltaMushReferenceFixtures()) {
        RigExecDeltaMushSettings settings;
        settings.smoothing = fixture.lengthWeighted ? RigExecDeltaMushSmoothing::LengthWeighted : RigExecDeltaMushSmoothing::Simple;
        settings.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        settings.smoothWeights = fixture.weights;
        settings.edges = fixture.edges;
        settings.onlySmooth = fixture.onlySmooth;
        auto actual = fixture.input;
        CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
            4, fixture.step, fixture.pin, 0, fixture.detail, settings));
        CHECK(Near(actual, fixture.expected, 2e-6));
        // A nonuniform affine owner map must preserve local smoothing behavior.
        GfMatrix4d computationToTarget(1);
        computationToTarget.SetScale(GfVec3d(1.4, 0.73, 1.15));
        GfMatrix4d rotate(1); rotate.SetRotate(GfRotation(GfVec3d(1, 2, 3), 28));
        computationToTarget *= rotate;
        computationToTarget.SetTranslateOnly(GfVec3d(0.3, -0.2, 0.4));
        auto transformedInput = fixture.input, transformedExpected = fixture.expected;
        for (auto &p : transformedInput) p = GfVec3f(computationToTarget.Transform(GfVec3d(p)));
        for (auto &p : transformedExpected) p = GfVec3f(computationToTarget.Transform(GfVec3d(p)));
        actual = transformedInput;
        CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
            4, fixture.step, fixture.pin, 0, fixture.detail, settings, computationToTarget));
        CHECK(Near(actual, transformedExpected, 3e-6));
        auto stage = UsdStage::CreateInMemory();
        auto rig = RigExecRigBuilder::Create(stage, SdfPath("/Rig"));
        auto mesh = UsdGeomMesh::Define(stage, SdfPath("/Rig/Body"));
        mesh.CreatePointsAttr().Set(VtVec3fArray(transformedInput.begin(), transformedInput.end()));
        mesh.CreateFaceVertexCountsAttr().Set(VtIntArray(fixture.counts.begin(), fixture.counts.end()));
        mesh.CreateFaceVertexIndicesAttr().Set(VtIntArray(fixture.indices.begin(), fixture.indices.end()));
        auto mush = rig.NewMoverChain("deform", mesh.GetPointsAttr().GetPath()).AddDeltaMushMover("mush");
        mush.SetRestPoints(fixture.rest); mush.SetIterations(4); mush.SetStep(fixture.step);
        mush.SetPinBorders(fixture.pin);
        auto prim = mush.GetPrim();
        CHECK(prim.GetAttribute(TfToken("inputs:smoothing")).Set(TfToken(fixture.lengthWeighted ? "lengthWeighted" : "simple")));
        CHECK(prim.GetAttribute(TfToken("inputs:frameTransport")).Set(TfToken("corner")));
        CHECK(prim.GetAttribute(TfToken("inputs:displacement")).Set(fixture.detail));
        CHECK(prim.GetAttribute(TfToken("inputs:smoothWeights")).Set(VtFloatArray(fixture.weights.begin(), fixture.weights.end())));
        CHECK(prim.GetAttribute(TfToken("inputs:edges")).Set(VtIntArray(fixture.edges.begin(), fixture.edges.end())));
        CHECK(prim.GetAttribute(TfToken("inputs:onlySmooth")).Set(fixture.onlySmooth));
        CHECK(prim.GetAttribute(TfToken("inputs:computationToTarget")).Set(computationToTarget));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
        // The independent scalar reference judges every evaluation.
        evaluator.cpuReference = true;
        std::vector<std::string> diagnostics;
        CHECK(evaluator.Compile(&diagnostics));
        std::string before; stage->GetRootLayer()->ExportToString(&before);
        auto pose = evaluator.Evaluate(UsdTimeCode(1));
        CHECK(pose.valid); CHECK(pose.referenceMismatches == 0); CHECK(pose.referenceAgreements > 0);
        auto value = pose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
        CHECK(Near({value.begin(), value.end()}, transformedExpected, 3e-6));
        RigExecBakeOpts options; options.time = 1;
        RigExecBakeResult result; std::string error;
        CHECK(RigExecBakeToBinary(evaluator, options, &result, &error));
        auto reader = RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(), &error);
        CHECK(reader); CHECK(reader->Execute(&error));
        bool found = false;
        for (const auto &points : reader->GetPoints()) {
            if (points.path != mesh.GetPointsAttr().GetPath().GetString()) continue;
            found = true; CHECK(points.points.size() == value.size());
            for (size_t i = 0; i < value.size(); ++i)
                CHECK(std::memcmp(points.points[i].data(), value[i].data(), 3*sizeof(float)) == 0);
        }
        CHECK(found);
        std::string after; stage->GetRootLayer()->ExportToString(&after); CHECK(before == after);
        // Live frame folding must use the evaluated provider, including binary
        // avar edits after an export containing only its identity pose.
        auto owner = rig.AddControl("Owner", GfMatrix4d(1));
        CHECK(prim.GetRelationship(TfToken("rigExec:frame")).SetTargets({owner.GetPath()}));
        mush.SetReadPhase(TfToken("rigExec:frame"), "final");
        CHECK(RigExecBakeToBinary(evaluator, options, &result, &error));
        reader = RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(), &error);
        CHECK(reader);
        for (double scale : {1.0, 0.74, 1.31, 1.0}) {
            owner.SetAvarScale(1, scale, 1);
            CHECK(reader->SetInput(owner.GetPath().AppendProperty(TfToken("avars:sy")).GetString(), scale, &error));
            pose = evaluator.Evaluate(UsdTimeCode(1));
            CHECK(pose.valid); CHECK(pose.referenceMismatches == 0); CHECK(pose.referenceAgreements > 0);
            CHECK(reader->Execute(&error));
            value = pose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
            GfMatrix4d scaling(1); scaling.SetScale(GfVec3d(1, scale, 1));
            actual = transformedInput;
            CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
                4, fixture.step, fixture.pin, 0, fixture.detail, settings, computationToTarget * scaling));
            CHECK(Near({value.begin(), value.end()}, actual, 3e-6));
            found = false;
            for (const auto &points : reader->GetPoints()) {
                if (points.path != mesh.GetPointsAttr().GetPath().GetString()) continue;
                found = true; CHECK(points.points.size() == value.size());
                for (size_t i = 0; i < value.size(); ++i)
                    CHECK(std::memcmp(points.points[i].data(), value[i].data(), 3*sizeof(float)) == 0);
            }
            CHECK(found);
        }
        mush.SetReadPhase(TfToken("rigExec:frame"), "/Rig/Movers");
        RigExecRigEvaluator invalidPhase(stage, SdfPath("/Rig"));
        diagnostics.clear(); CHECK(invalidPhase.Compile(&diagnostics));
        CHECK(std::any_of(diagnostics.begin(), diagnostics.end(), [](const std::string &d) {
            return d.find("frame read phase must be base or final") != std::string::npos;
        }));
    }
}
// The envelope-support kernel must equal the full kernel bit for bit on every
// active vertex and leave every other point as it entered, for every
// smoothing mode, frame transport, border pin, distance weight and per-vertex
// smoothing weight. The mesh mixes quads and triangles so quad diagonals (two
// edge rings apart, but one face apart) are exercised.
// Splits every loop it is given into slices of 7 elements dealt round-robin
// to four threads, ignoring the grain, so a kernel whose slices are not
// independent shows up as a mismatch against its serial run.
static const RigExecRangeRunner kInterleavedRanges =
    [](size_t count, size_t, const RigExecRangeBody &body) {
        constexpr size_t slice = 7, threads = 4;
        std::vector<std::thread> pool;
        for (size_t t = 0; t < threads; ++t)
            pool.emplace_back([&body, count, t] {
                for (size_t b = t * slice; b < count; b += threads * slice)
                    body(b, std::min(count, b + slice));
            });
        for (std::thread &thread : pool) thread.join();
    };

static bool SameBytes(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(GfVec3f)) == 0;
}


// The floating-point environment (codex review, round 4). A host may run with
// flush-to-zero, denormals-are-zero or another rounding mode. Split loops
// must then still round as the calling thread does, and the radial matrix
// kernel's zero-weight shortcut must still return what the identity
// transform returns on that thread.
static bool SameBytes(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b);

static void CheckFloatEnvironment() {
#if defined(_M_X64) || defined(_M_IX86) || defined(__SSE2__)
    const unsigned saved = _mm_getcsr();
    CHECK(RigExecFloatEnvironmentIsDefault());
    const unsigned environments[] = {
        saved,
        saved | 0x8040u,                      // flush-to-zero + denormals-are-zero
        saved | 0x8000u,                      // flush-to-zero only
        (saved & ~0x6000u) | 0x2000u,         // round toward negative
    };
    for (unsigned csr : environments) {
        _mm_setcsr(csr);
        CHECK(RigExecFloatEnvironmentIsDefault() == (csr == saved));
        // Every slice of a split loop runs in the caller's environment.
        std::atomic<bool> same(true);
        // Control bits only; the low six are sticky status flags.
        RigExecParallelForInCallerEnvironment(1 << 16, 256, [&](size_t, size_t) {
            if ((_mm_getcsr() & ~0x3Fu) != (csr & ~0x3Fu)) same = false;
        });
        CHECK(same.load());
        CHECK((_mm_getcsr() & ~0x3Fu) == (csr & ~0x3Fu));
        // The radial kernel at weight 0 against the identity's own arithmetic
        // on this thread: subnormals (flushed under FTZ/DAZ), -0 (kept by
        // rounding toward negative), ordinary values.
        RigExecMoverParameters p;
        p.radialWeight = true;
        p.transform = GfMatrix4d().SetTranslate(GfVec3d(0.5, -2, 3));
        const std::vector<GfVec3f> entering{
            {1e-40f, 1.0f, 2.0f}, {-0.0f, -1.0f, 2.0f}, {0.5f, -0.0f, 3.0f},
            {-1e-41f, -0.0f, -0.0f}, {123.25f, 0.0f, -7.5f}};
        const std::vector<float> zero(entering.size(), 0.0f);
        auto shortcut = entering;
        RigExecApplyMatrixKernelRange(p, zero.data(), 0, shortcut.size(), shortcut.data(), false);
        for (size_t i = 0; i < entering.size(); ++i) {
            const GfVec3f identity(GfMatrix4d(1.0).TransformAffine(GfVec3d(entering[i])));
            CHECK(std::memcmp(&identity, &shortcut[i], sizeof(GfVec3f)) == 0);
        }
    }
    _mm_setcsr(saved);
    CHECK(RigExecFloatEnvironmentIsDefault());
    // The masked delta mush memo remembers the control state with its result
    // (codex review, round 5): a result cached under the default environment
    // is not handed back under FTZ/DAZ, where the same inputs flush.
    {
        const std::vector<GfVec3f> restTwo{{0,0,0},{1,0,0},{0,1,0},{3,0,0},{4,0,0},{3,1,0}};
        const std::vector<int> countsTwo{3, 3}, indicesTwo{0,1,2,3,4,5};
        RigExecDeltaMushSettings settings;
        settings.smoothing = RigExecDeltaMushSmoothing::Simple;
        settings.onlySmooth = true;
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> data;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(restTwo, countsTwo, indicesTwo, 1, 0.5,
                                                                false, 0, settings, &data)));
        std::vector<GfVec3f> tiny = restTwo;
        for (GfVec3f &p : tiny) p[0] = 1e-40f;
        std::vector<char> mask(6, 0); mask[0] = 1;
        RigExecDeltaMushMaskMemo<GfVec3f> memo;
        auto first = tiny;
        CHECK(RigExecApplyDeltaMushWithRestDataMasked(&first, data, 1.0, mask, nullptr, &memo));
        _mm_setcsr(saved | 0x8040u);
        auto cached = tiny, fresh = tiny;
        const bool cachedOk = RigExecApplyDeltaMushWithRestDataMasked(&cached, data, 1.0, mask, nullptr, &memo);
        const bool freshOk = RigExecApplyDeltaMushWithRestDataMasked(&fresh, data, 1.0, mask);
        _mm_setcsr(saved);
        CHECK(cachedOk && freshOk);
        CHECK(SameBytes(cached, fresh));
        CHECK(!SameBytes(first, fresh));   // the environment really changes the answer
        auto again = tiny;
        CHECK(RigExecApplyDeltaMushWithRestDataMasked(&again, data, 1.0, mask, nullptr, &memo));
        CHECK(SameBytes(again, first));
    }
#endif
}

static void CheckMaskedMatchesFull() {
    const int W = 24, H = 18;
    std::vector<GfVec3f> rest;
    for (int y = 0; y <= H; ++y)
        for (int x = 0; x <= W; ++x)
            rest.emplace_back(0.1f * x, 0.1f * y, 0.05f * std::sin(0.4f * x) * std::cos(0.3f * y));
    std::vector<int> counts, indices;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const int a = y * (W + 1) + x, b = a + 1, c = a + W + 2, d = a + W + 1;
            if ((x + y) % 3 == 0) { counts.insert(counts.end(), {3, 3}); indices.insert(indices.end(), {a, b, c, a, c, d}); }
            else { counts.push_back(4); indices.insert(indices.end(), {a, b, c, d}); }
        }
    std::vector<GfVec3f> posed = rest;
    for (size_t i = 0; i < posed.size(); ++i)
        posed[i] += GfVec3f(0.03f * std::sin(1.7f * float(i)), 0.02f * std::cos(0.9f * float(i)),
                            0.08f * std::sin(0.11f * float(i)));
    std::vector<char> active(rest.size(), 0);
    for (size_t i = 0; i < rest.size(); ++i) {
        const GfVec3f d = rest[i] - GfVec3f(1.1f, 0.9f, 0);
        active[i] = d[0] * d[0] + d[1] * d[1] < 0.16f;          // a disc of ~50 vertices
    }
    active[0] = 1;                                              // and a border corner
    std::vector<float> smoothWeights(rest.size());
    for (size_t i = 0; i < rest.size(); ++i) smoothWeights[i] = float((i * 7) % 10) / 9.0f;
    size_t cases = 0;
    for (auto smoothing : {RigExecDeltaMushSmoothing::Rest, RigExecDeltaMushSmoothing::Simple,
                           RigExecDeltaMushSmoothing::LengthWeighted})
    for (auto transport : {RigExecDeltaMushFrameTransport::Corner, RigExecDeltaMushFrameTransport::Vertex})
    for (bool onlySmooth : {false, true})
    for (bool pin : {true, false})
    for (bool weighted : {false, true})
    for (double distanceWeight : {0.0, 0.5}) {
        RigExecDeltaMushSettings settings;
        settings.smoothing = smoothing; settings.frameTransport = transport; settings.onlySmooth = onlySmooth;
        if (weighted) settings.smoothWeights = smoothWeights;
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> restData;
        if (!RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(rest, counts, indices, 5, 0.5, pin,
                                                              distanceWeight, settings, &restData))
            continue;
        auto full = posed, masked = posed;
        CHECK(RigExecApplyDeltaMushWithRestData(&full, restData, 0.8));
        CHECK(RigExecApplyDeltaMushWithRestDataMasked(&masked, restData, 0.8, active));
        for (size_t i = 0; i < rest.size(); ++i) {
            const GfVec3f &want = active[i] ? full[i] : posed[i];
            CHECK(std::memcmp(&masked[i], &want, sizeof(GfVec3f)) == 0);
        }
        // Split across threads: the same bytes, and the same verdict.
        auto threaded = posed;
        CHECK(RigExecApplyDeltaMushWithRestDataMasked(&threaded, restData, 0.8, active,
                                                      &kInterleavedRanges));
        CHECK(SameBytes(threaded, masked));
        CHECK(RigExecDeltaMushMaskPreservesVerdict(posed, restData, 0.8) ==
              RigExecDeltaMushMaskPreservesVerdict(posed, restData, 0.8, &kInterleavedRanges));
        // The memo answers every call exactly as a fresh call does: a
        // repeat, an edit no active result reads, an edit one does, a new
        // displacement, a new mask, and the original again.
        {
            RigExecDeltaMushMaskMemo<GfVec3f> memo;
            const auto viaMemo = [&](std::vector<GfVec3f> in, double displacement,
                                     const std::vector<char> &mask) {
                CHECK(RigExecApplyDeltaMushWithRestDataMasked(&in, restData, displacement, mask,
                                                              &kInterleavedRanges, &memo));
                return in;
            };
            const auto fresh = [&](std::vector<GfVec3f> in, double displacement,
                                   const std::vector<char> &mask) {
                CHECK(RigExecApplyDeltaMushWithRestDataMasked(&in, restData, displacement, mask));
                return in;
            };
            auto outside = posed, inside = posed;
            outside.back() += GfVec3f(0.01f, -0.02f, 0.03f);   // the corner opposite the disc
            for (size_t i = 0; i < inside.size(); ++i)
                if (active[i] && i != 0) { inside[i] += GfVec3f(0.005f, 0, 0); break; }
            std::vector<char> smaller = active; smaller[0] = 0;
            CHECK(SameBytes(viaMemo(posed, 0.8, active), masked));
            CHECK(SameBytes(viaMemo(posed, 0.8, active), masked));
            CHECK(SameBytes(viaMemo(outside, 0.8, active), fresh(outside, 0.8, active)));
            CHECK(SameBytes(viaMemo(inside, 0.8, active), fresh(inside, 0.8, active)));
            CHECK(SameBytes(viaMemo(inside, 0.5, active), fresh(inside, 0.5, active)));
            CHECK(SameBytes(viaMemo(inside, 0.5, smaller), fresh(inside, 0.5, smaller)));
            CHECK(SameBytes(viaMemo(posed, 0.8, active), masked));
            // Copy on write: a copy of the memo (what a program clone holds)
            // shares the published plan and result, and a call through the
            // copy replaces the copy's pointers without touching what the
            // original points at.
            {
                using Memo = RigExecDeltaMushMaskMemo<GfVec3f>;
                Memo clone = memo;
                CHECK(clone.plan == memo.plan && clone.result == memo.result);
                const auto plan0 = memo.plan;
                const auto result0 = memo.result;
                const std::vector<GfVec3f> held = result0 ? result0->activeResult
                                                          : std::vector<GfVec3f>();
                auto viaClone = inside;
                CHECK(RigExecApplyDeltaMushWithRestDataMasked(&viaClone, restData, 0.5, smaller,
                                                              &kInterleavedRanges, &clone));
                CHECK(SameBytes(viaClone, fresh(inside, 0.5, smaller)));
                CHECK(memo.plan == plan0 && memo.result == result0);
                CHECK(!result0 || SameBytes(result0->activeResult, held));
                CHECK(clone.plan != plan0);
            }
            // The memo is really consulted: a poisoned result comes back for
            // an input it may reuse, and a recompute replaces it otherwise.
            // Where the control word cannot be read, nothing is ever reused.
            {
                using Memo = RigExecDeltaMushMaskMemo<GfVec3f>;
                auto poisoned = std::make_shared<Memo::Result>(*memo.result);
                for (auto &p : poisoned->activeResult) p = GfVec3f(7);
                memo.result = poisoned;
            }
            const auto reused = viaMemo(outside, 0.8, active);
            if (RigExecFloatControlWord() != kRigExecUnknownFloatControl)
                CHECK(memo.result && reused[memo.plan->activeList.front()] == GfVec3f(7));
            else
                CHECK(SameBytes(reused, fresh(outside, 0.8, active)));
            CHECK(SameBytes(viaMemo(inside, 0.8, active), fresh(inside, 0.8, active)));
        }
        ++cases;
    }
    CHECK(cases >= 40);
    // No active vertex is a pass-through; a full mask is the full kernel.
    RigExecDeltaMushRestData<GfVec3f, GfVec3d> restData;
    CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(rest, counts, indices, 5, 0.5, true, 0,
                                                            RigExecDeltaMushSettings(), &restData)));
    auto none = posed;
    CHECK(RigExecApplyDeltaMushWithRestDataMasked(&none, restData, 1.0, std::vector<char>(rest.size(), 0)));
    CHECK(std::memcmp(none.data(), posed.data(), posed.size() * sizeof(GfVec3f)) == 0);
    auto all = posed, reference = posed;
    CHECK(RigExecApplyDeltaMushWithRestDataMasked(&all, restData, 1.0, std::vector<char>(rest.size(), 1)));
    CHECK(RigExecApplyDeltaMushWithRestData(&reference, restData, 1.0));
    CHECK(std::memcmp(all.data(), reference.data(), posed.size() * sizeof(GfVec3f)) == 0);
    // A mask of the wrong size is refused and leaves the points alone.
    auto refused = posed;
    CHECK(!RigExecApplyDeltaMushWithRestDataMasked(&refused, restData, 1.0, std::vector<char>(3, 1)));
    CHECK(refused == posed);
    // The default frame transport is vertex, which is never substituted.
    CHECK(!RigExecDeltaMushMaskPreservesVerdict(posed, restData, 1.0));

    // Verdict preservation (codex review): two disconnected triangles, only
    // vertex 0 active, the second triangle collapsed to a point. Vertex
    // transport fails the whole full kernel on the collapsed frame, which the
    // masked kernel cannot see, so a host must not substitute it there.
    // Corner transport tolerates the collapse, so the two agree and the
    // substitution stays allowed.
    const std::vector<GfVec3f> twoRest{{0,0,0},{1,0,0},{0,1,0},{3,0,0},{4,0,0},{3,1,0}};
    const std::vector<int> twoCounts{3, 3}, twoIndices{0,1,2,3,4,5};
    auto twoPosed = twoRest;
    for (int i = 0; i < 3; ++i) twoPosed[i] *= 2.0f;
    for (int i = 3; i < 6; ++i) twoPosed[i] = GfVec3f(3, 0, 0);
    std::vector<char> first(6, 0); first[0] = 1;
    for (auto transport : {RigExecDeltaMushFrameTransport::Vertex, RigExecDeltaMushFrameTransport::Corner}) {
        RigExecDeltaMushSettings settings; settings.frameTransport = transport;
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> twoData;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(twoRest, twoCounts, twoIndices, 1, 0.5,
                                                                false, 0, settings, &twoData)));
        auto full = twoPosed, masked = twoPosed;
        const bool fullOk = RigExecApplyDeltaMushWithRestData(&full, twoData, 1.0);
        const bool maskedOk = RigExecApplyDeltaMushWithRestDataMasked(&masked, twoData, 1.0, first);
        if (transport == RigExecDeltaMushFrameTransport::Vertex) {
            CHECK(!fullOk && maskedOk);                     // the verdicts really differ
            CHECK(!RigExecDeltaMushMaskPreservesVerdict(twoPosed, twoData, 1.0));
        } else {
            CHECK(fullOk && maskedOk);
            CHECK(std::memcmp(&full[0], &masked[0], sizeof(GfVec3f)) == 0);
            CHECK(RigExecDeltaMushMaskPreservesVerdict(twoPosed, twoData, 1.0));
        }
    }

    // Atomicity (codex review): finite entering points, vertex 0 computes
    // normally, then vertex 3 overflows (an extrapolating simple step drives
    // it past float max). The call fails and every point is untouched; the
    // full kernel fails too and the magnitudes are never substituted.
    {
        RigExecDeltaMushSettings settings;
        settings.smoothing = RigExecDeltaMushSmoothing::Simple;
        settings.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> data;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(twoRest, twoCounts, twoIndices, 1, 3.0,
                                                                false, 0, settings, &data)));
        auto entering = twoRest;
        entering[3] = GfVec3f(1e38f, 0, 0); entering[4] = GfVec3f(-1e38f, 0, 0);
        entering[5] = GfVec3f(-1e38f, 1, 0);
        std::vector<char> both(6, 0); both[0] = both[3] = 1;
        auto attempt = entering, full = entering;
        CHECK(!RigExecApplyDeltaMushWithRestDataMasked(&attempt, data, 1.0, both));
        CHECK(std::memcmp(attempt.data(), entering.data(), 6 * sizeof(GfVec3f)) == 0);
        CHECK(!RigExecApplyDeltaMushWithRestData(&full, data, 1.0));
        CHECK(!RigExecDeltaMushMaskPreservesVerdict(entering, data, 1.0));
    }
    // Rest-side verdict gaps (codex review), both proved unsafe by the builder:
    // (a) inverse-distance weights that underflow to zero (0/0 in the full
    //     kernel on a triangle outside the support);
    // (b) rest minus smoothed rest that overflows before the displacement
    //     scale (NaN even at displacement 0).
    {
        RigExecDeltaMushSettings onlySmooth; onlySmooth.onlySmooth = true;
        std::vector<GfVec3f> farRest = twoRest;
        for (int i = 3; i < 6; ++i) farRest[i] = twoRest[i] * 5e37f;   // finite; edges ~5e37
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> a;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(farRest, twoCounts, twoIndices, 1, 0.5,
                                                                false, 10, onlySmooth, &a)));
        CHECK(!a.maskRestSafe);
        CHECK(!RigExecDeltaMushMaskPreservesVerdict(twoRest, a, 1.0));

        RigExecDeltaMushSettings corner;
        corner.smoothing = RigExecDeltaMushSmoothing::Simple;
        corner.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        std::vector<GfVec3f> wideRest = twoRest;
        wideRest[3] = GfVec3f(3e38f, 0, 0); wideRest[4] = GfVec3f(-3e38f, 1, 0);
        wideRest[5] = GfVec3f(-3e38f, 0, 1);
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> b;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(wideRest, twoCounts, twoIndices, 1, 1.0,
                                                                false, 0, corner, &b)));
        CHECK(!b.maskRestSafe);
        CHECK(!RigExecDeltaMushMaskPreservesVerdict(twoRest, b, 0.0));

        // An ordinary build is proved safe.
        RigExecDeltaMushRestData<GfVec3f, GfVec3d> c;
        CHECK((RigExecBuildDeltaMushRestData<GfVec3f, GfVec3d>(twoRest, twoCounts, twoIndices, 2, 0.5,
                                                                false, 1, corner, &c)));
        CHECK(c.maskRestSafe);
        CHECK(RigExecDeltaMushMaskPreservesVerdict(twoRest, c, 1.0));
    }
}
int main(int argc,char **argv) {
 try {
    CHECK(argc==2);PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
    CheckReferenceFixtures();
    CheckMaskedMatchesFull();
    CheckFloatEnvironment();
    std::vector<GfVec3f> rest{{1,0,0},{0,1.2f,0},{-1,0,0},{0,-1,0},{0,0,1},{0,0,-1}};
    std::vector<int> counts(8,3),indices{0,1,4,1,2,4,2,3,4,3,0,4,1,0,5,2,1,5,3,2,5,0,3,5};
    auto saved=rest;CHECK(RigExecApplyDeltaMush(&saved,rest,counts,indices));CHECK(Near(saved,rest));
    GfMatrix4d matrix(1);matrix.SetRotate(GfRotation(GfVec3d(0,0,1),37));matrix.SetTranslateOnly(GfVec3d(4,2,-3));
    auto rigid=rest;for(auto &p:rigid)p=GfVec3f(matrix.Transform(GfVec3d(p)));
    auto expected=rigid;CHECK(RigExecApplyDeltaMush(&rigid,rest,counts,indices,3));CHECK(Near(rigid,expected));
    auto posed=rest;posed[4]+=GfVec3f(0.7f,0,0.2f);
    auto full=posed;CHECK(RigExecApplyDeltaMush(&full,rest,counts,indices,3));CHECK(!Near(full,posed,0.01));
    auto zero=posed;CHECK(RigExecApplyDeltaMush(&zero,rest,counts,indices,0));CHECK(zero==posed);
    auto invalid=posed;auto bad=indices;bad.back()=100;
    CHECK(!RigExecApplyDeltaMush(&invalid,rest,counts,bad));CHECK(invalid==posed);
    CHECK(!RigExecApplyDeltaMush(&invalid,rest,counts,indices,-1));CHECK(invalid==posed);
    CHECK(!RigExecApplyDeltaMush(&invalid,{},counts,indices));CHECK(invalid==posed);
    // An open quad has only border vertices, all of which must remain exact.
    std::vector<GfVec3f> quad{{0,0,0},{1,0,0},{1,1,0},{0,1,0}},bent=quad;
    bent[2][2]=1;auto pinned=bent;
    CHECK(RigExecApplyDeltaMush(&pinned,quad,{4},{0,1,2,3}));CHECK(pinned==bent);
    // Author a native operator and exercise runtime edits and serialization.
    auto stage=UsdStage::CreateInMemory();auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto mesh=UsdGeomMesh::Define(stage,SdfPath("/Rig/Body"));
    mesh.CreatePointsAttr().Set(VtVec3fArray(posed.begin(),posed.end()));
    mesh.CreateFaceVertexCountsAttr().Set(VtIntArray(counts.begin(),counts.end()));
    mesh.CreateFaceVertexIndicesAttr().Set(VtIntArray(indices.begin(),indices.end()));
    auto mush=rig.NewMoverChain("deform",mesh.GetPointsAttr().GetPath()).AddDeltaMushMover("mush");
    mush.SetRestPoints(rest);mush.SetIterations(3);
    RigExecRigEvaluator evaluator(stage,SdfPath("/Rig"));std::vector<std::string> diagnostics;
    CHECK(evaluator.Compile(&diagnostics));
    if(!evaluator.IsBakeable(&diagnostics)) {
        for(const auto &d:diagnostics)std::cerr<<d<<'\n';
        CHECK(false);
    }
    auto evaluate=[&](){auto p=evaluator.Evaluate(UsdTimeCode::Default());CHECK(p.valid);
        CHECK(p.comparisonMismatches==0);CHECK(p.referenceMismatches==0);
        CHECK(evaluator.GetBakedProgram()!=nullptr);
        auto v=p.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
        return std::vector<GfVec3f>(v.begin(),v.end());};
    CHECK(Near(evaluate(),full));
    mush.SetDefaultWeight(0.5f);auto halfway=posed;
    for(size_t i=0;i<posed.size();++i)halfway[i]=(posed[i]+full[i])*0.5f;
    CHECK(Near(evaluate(),halfway));
    mush.SetDefaultWeight(0);CHECK(Near(evaluate(),posed));
    mush.SetDefaultWeight(1);mush.SetIterations(0);CHECK(Near(evaluate(),posed));
    mush.SetIterations(3);CHECK(Near(evaluate(),full));
    auto weight=rig.AddStaticWeight("mask",mesh.GetPointsAttr().GetPath(),{0,1,0.5f,1,0,1});
    mush.SetWeightObject(weight.GetPath());auto masked=posed;
    const float mask[6]={0,1,0.5f,1,0,1};
    for(size_t i=0;i<posed.size();++i)masked[i]=posed[i]+mask[i]*(full[i]-posed[i]);
    CHECK(Near(evaluate(),masked));
    mush.GetPrim().GetRelationship(TfToken("rigExec:weightObject")).ClearTargets(true);
    mush.SetDistanceWeight(1);auto weighted=posed;
    CHECK(RigExecApplyDeltaMush(&weighted,rest,counts,indices,3,0.5,true,1));
    CHECK(Near(evaluate(),weighted));CHECK(!Near(weighted,full));
    mush.SetDistanceWeight(0);
    mush.SetDisplacement(0);auto smooth=posed;
    CHECK(RigExecApplyDeltaMush(&smooth,rest,counts,indices,3,0.5,true,0,0));CHECK(Near(evaluate(),smooth));
    std::string text;stage->GetRootLayer()->ExportToString(&text);
    auto layer=SdfLayer::CreateAnonymous("roundtrip.usda");CHECK(layer->ImportFromString(text));
    auto reopened=UsdStage::Open(layer);RigExecRigEvaluator e2(reopened,SdfPath("/Rig"));
    e2.cpuReference=true;CHECK(e2.Compile(&diagnostics));
    auto p=e2.Evaluate(UsdTimeCode::Default());CHECK(p.valid);
    CHECK(p.referenceMismatches==0);CHECK(p.referenceAgreements==1);
    auto value=p.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();CHECK(Near({value.begin(),value.end()},smooth));
    // Binary playback must run the shared kernel, take every parameter from
    // its inputs and publish bitwise-identical points, including revisiting
    // earlier frames: baked at the first frame, played through its inputs.
    auto binaryParity = [&](const std::vector<double> &frames) {
        RigExecBakeOpts options; options.time=frames.front();
        RigExecBakeResult result; std::string error;
        CHECK(RigExecBakeToBinary(evaluator,options,&result,&error));
        CHECK(!result.bytes.empty());
        RigExecTestPlayer reader;
        if(!reader.Open(result.bytes,stage,&error))throw std::runtime_error(error);
        auto playback=frames;playback.insert(playback.end(),frames.rbegin(),frames.rend());
        for(double frame:playback) {
            auto expectedPose=evaluator.Evaluate(UsdTimeCode(frame));
            CHECK(expectedPose.valid);CHECK(expectedPose.comparisonMismatches==0);
            if(!reader.Play(frame,&error))throw std::runtime_error(error);
            bool found=false;
            for(const auto &actual:reader->GetPoints()) {
                if(actual.path!=mesh.GetPointsAttr().GetPath().GetString())continue;
                found=true;
                auto expected=expectedPose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
                CHECK(actual.points.size()==expected.size());
                for(size_t i=0;i<expected.size();++i)
                    CHECK(std::memcmp(actual.points[i].data(),expected[i].data(),3*sizeof(float))==0);
            }
            CHECK(found);
        }
    };
    const auto prim=mush.GetPrim();
    auto sample=[&](const char *name,auto value,int frame) {
        CHECK(prim.GetAttribute(TfToken(name)).Set(value,UsdTimeCode(frame)));
    };
    // Every setting differs from its fallback in at least one frame.
    sample("inputs:iterations",3,1);sample("inputs:iterations",0,2);sample("inputs:iterations",4,3);
    sample("inputs:step",0.3f,1);sample("inputs:step",0.7f,3);
    sample("inputs:distanceWeight",1.0f,1);sample("inputs:distanceWeight",2.0f,3);
    sample("inputs:displacement",0.7f,1);sample("inputs:displacement",0.0f,3);
    sample("inputs:defaultWeight",0.35f,1);sample("inputs:defaultWeight",1.0f,3);
    sample("inputs:enabled",true,1);sample("inputs:enabled",false,4);sample("inputs:enabled",true,5);
    binaryParity({1,2,3,4,5});
    auto driver=stage->GetPrimAtPath(SdfPath("/Rig")).CreateAttribute(
        TfToken("mushStep"),SdfValueTypeNames->Float);
    driver.Set(0.2f,UsdTimeCode(1));driver.Set(0.7f,UsdTimeCode(3));
    prim.GetAttribute(TfToken("inputs:step")).SetConnections({driver.GetPath()});
    mush.SetWeightObject(weight.GetPath());binaryParity({1,2,3,4,5});
    // Open border pinning and changing topology use the same serialized reads.
    mesh.GetPointsAttr().Set(VtVec3fArray(bent.begin(),bent.end()));
    mesh.GetFaceVertexCountsAttr().Set(VtIntArray{4});
    mesh.GetFaceVertexIndicesAttr().Set(VtIntArray{0,1,2,3});
    mush.SetRestPoints(quad);
    prim.GetRelationship(TfToken("rigExec:weightObject")).ClearTargets(true);
    sample("inputs:pinBorders",true,1);sample("inputs:pinBorders",false,3);
    binaryParity({1,2,3,4,5});
    // Empty rest array exercises the authored-base fallback in both hosts.
    mush.SetRestPoints({});binaryParity({1,3});
    // Invalid parameter is an atomic pass-through in both hosts.
    sample("inputs:iterations",-1,6);binaryParity({6});
    {
        // A baked input frame contains only the rest pose. Avar inputs set
        // on the binary must rerun skin -> deltaMush, not replay captured
        // deformed points.
        auto liveStage=UsdStage::CreateInMemory();
        auto builder=RigExecRigBuilder::Create(liveStage,SdfPath("/Rig"));
        auto body=UsdGeomMesh::Define(liveStage,SdfPath("/Rig/Body"));
        body.CreatePointsAttr().Set(VtVec3fArray(rest.begin(),rest.end()));
        body.CreateFaceVertexCountsAttr().Set(VtIntArray(counts.begin(),counts.end()));
        body.CreateFaceVertexIndicesAttr().Set(VtIntArray(indices.begin(),indices.end()));
        auto anchor=builder.AddControl("Anchor",GfMatrix4d(1));
        auto tip=builder.AddControl("Tip",GfMatrix4d(1));
        auto chain=builder.NewMoverChain("Deform",body.GetPointsAttr().GetPath());
        auto detail=chain.AddDeltaMushMover("Detail");
        detail.SetRestPoints(rest);detail.SetIterations(3);
        auto skin=chain.AddSkinMover("Skin",{anchor.GetPath(),tip.GetPath()});
        skin.SetJointInfluences({0,0,0,0,1,0},{1,1,1,1,1,1},1);
        RigExecRigEvaluator live(liveStage,SdfPath("/Rig"));
        RigExecBakeOpts options;options.time=1;
        RigExecBakeResult result;std::string error;
        CHECK(RigExecBakeToBinary(live,options,&result,&error));
        auto reader=RigExecRuntimeReader::Open(result.bytes.data(),result.bytes.size(),&error);
        if(!reader)throw std::runtime_error(error);
        for(double translation:{0.0,0.7,-0.4,0.0}) {
            tip.SetAvarTranslation(translation,0,0);
            CHECK(reader->SetInput(tip.GetPath().AppendProperty(TfToken("avars:tx")).GetString(),translation,&error));
            auto pose=live.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
            CHECK(pose.comparisonMismatches==0);
            if(!reader->Execute(&error))throw std::runtime_error(error);
            auto points=pose.movedProperties.at(body.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
            auto input=rest;input[4][0]+=float(translation);
            auto corrected=input;CHECK(RigExecApplyDeltaMush(&corrected,rest,counts,indices,3));
            CHECK(Near({points.begin(),points.end()},corrected));
            if(translation!=0)CHECK(!Near(corrected,input));
            bool found=false;
            for(const auto &actual:reader->GetPoints()) {
                if(actual.path!="/Rig/Body.points")continue;
                found=true;CHECK(actual.points.size()==points.size());
                for(size_t i=0;i<points.size();++i)
                    CHECK(std::memcmp(actual.points[i].data(),points[i].data(),3*sizeof(float))==0);
            }
            CHECK(found);
        }
    }
    std::cout<<"DeltaMush: shared kernel, USD and binary parity, animated parameters, masks, borders, invalid inputs and reload passed\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
