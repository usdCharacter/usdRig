// RigExecClothMover: position-based cloth on one mesh, colliding with other
// meshes, solved over a window of the frames before the current one
// (rigExecGraph/windowHistory.h, rigExecMath/clothKernel.h).
//
// A built-in external mover. The payload carries the epoch (topology, rest
// points, pins, settings, frame time, collider topology), this frame's
// parameters and collider points; a tick's packet is the cloth's entering
// points, the collider points and the parameters (rigExecBinary/
// clothPlayback.h, shared with the runtime, which plays the type itself).
#include "moverRegistry.h"
#include "externalPlayback.h"
#include "rigExec/gpuCloth.h"
#include "rigExecBinary/clothPlayback.h"
#include "rigExecScene/sceneDescriptors.h"

#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

// Interned once at load: a binder or a validator asks per compile, and
// interning takes the token registry's lock.
const TfToken _colliders("rigExec:colliders");
const TfToken _pinWeights("rigExec:pinWeights");
const TfToken _windowFrames("rigExec:windowFrames");
const TfToken _substeps("rigExec:substeps");
const TfToken _iterations("rigExec:iterations");
const TfToken _tethers("rigExec:tethers");
const TfToken _tetherSlack("rigExec:tetherSlack");
const TfToken _frameRate("rigExec:frameRate");
const TfToken _stretchCompliance("inputs:stretchCompliance");
const TfToken _bendCompliance("inputs:bendCompliance");
const TfToken _gravity("inputs:gravity");
const TfToken _damping("inputs:damping");
const TfToken _attract("inputs:attract");
const TfToken _thickness("inputs:thickness");
const TfToken _collisionDepth("inputs:collisionDepth");
const TfToken _friction("inputs:friction");
const TfToken _points("points");
const TfToken _faceVertexCounts("faceVertexCounts");
const TfToken _faceVertexIndices("faceVertexIndices");
const TfToken _orientation("orientation");
const TfToken _rightHanded("rightHanded");
const TfToken _leftHanded("leftHanded");

// The schema's fallbacks, which the declared inputs repeat.
constexpr int kWindowFrames = 16;
constexpr int kSubsteps = 10;
constexpr int kIterations = 1;
constexpr float kTetherSlack = 0.05f;

SdfPath
MeshOf(const SdfPath &target)
{
    return target.GetPrimPath();
}

std::vector<RigExecRevisionLeafKey>
DeclaredInputs(const SdfPath &mover, const SdfPath &target,
               const SdfPathVector &colliders)
{
    using Type = RigExecRevisionLeafType;
    using Time = RigExecRevisionLeafTime;
    using Flavour = RigExecRevisionLeafFlavour;
    const SdfPath mesh = MeshOf(target);
    std::vector<RigExecRevisionLeafKey> inputs;
    const auto at = [&](const SdfPath &path, Type type, Time time,
                        Flavour flavour, VtValue fallback) {
        inputs.push_back({path, type, time, flavour, std::move(fallback)});
    };
    at(mesh.AppendProperty(_faceVertexCounts), Type::IntArray, Time::AtDefault,
       Flavour::Raw, VtValue(VtIntArray()));
    at(mesh.AppendProperty(_faceVertexIndices), Type::IntArray, Time::AtDefault,
       Flavour::Raw, VtValue(VtIntArray()));
    at(mesh.AppendProperty(_orientation), Type::Token, Time::AtDefault,
       Flavour::Raw, VtValue(_rightHanded));
    at(mesh.AppendProperty(_points), Type::Vec3fArray, Time::AtDefault,
       Flavour::Raw, VtValue(VtVec3fArray()));
    at(mover.AppendProperty(_pinWeights), Type::FloatArray, Time::AtDefault,
       Flavour::Raw, VtValue(VtFloatArray()));
    at(mover.AppendProperty(_windowFrames), Type::Int, Time::AtDefault,
       Flavour::Raw, VtValue(kWindowFrames));
    at(mover.AppendProperty(_substeps), Type::Int, Time::AtDefault,
       Flavour::Raw, VtValue(kSubsteps));
    at(mover.AppendProperty(_iterations), Type::Int, Time::AtDefault,
       Flavour::Raw, VtValue(kIterations));
    at(mover.AppendProperty(_tethers), Type::Bool, Time::AtDefault,
       Flavour::Raw, VtValue(true));
    at(mover.AppendProperty(_tetherSlack), Type::Float, Time::AtDefault,
       Flavour::Raw, VtValue(kTetherSlack));
    at(mover.AppendProperty(_frameRate), Type::Double, Time::AtDefault,
       Flavour::Raw, VtValue(0.0));
    const RigExecClothParameters fallback;
    at(mover.AppendProperty(_stretchCompliance), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.stretchCompliance));
    at(mover.AppendProperty(_bendCompliance), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.bendCompliance));
    at(mover.AppendProperty(_gravity), Type::Vec3f, Time::AtTime,
       Flavour::Resolved,
       VtValue(GfVec3f(fallback.gravity[0], fallback.gravity[1], fallback.gravity[2])));
    at(mover.AppendProperty(_damping), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.damping));
    at(mover.AppendProperty(_attract), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.attract));
    at(mover.AppendProperty(_thickness), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.thickness));
    at(mover.AppendProperty(_collisionDepth), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.collisionDepth));
    at(mover.AppendProperty(_friction), Type::Float, Time::AtTime,
       Flavour::Resolved, VtValue(fallback.friction));
    for (const SdfPath &collider : colliders) {
        const SdfPath prim = MeshOf(collider);
        at(prim.AppendProperty(_points), Type::Vec3fArray, Time::AtTime,
           Flavour::Resolved, VtValue(VtVec3fArray()));
        at(prim.AppendProperty(_faceVertexCounts), Type::IntArray,
           Time::AtDefault, Flavour::Raw, VtValue(VtIntArray()));
        at(prim.AppendProperty(_faceVertexIndices), Type::IntArray,
           Time::AtDefault, Flavour::Raw, VtValue(VtIntArray()));
        at(prim.AppendProperty(_orientation), Type::Token, Time::AtDefault,
           Flavour::Raw, VtValue(_rightHanded));
    }
    return inputs;
}

SdfPathVector
CollidersOf(const UsdPrim &mover)
{
    SdfPathVector out;
    for (const SdfPath &path : RigExecRelationshipTargets(mover, "rigExec:colliders"))
        out.push_back(MeshOf(path));
    return out;
}

template <class T>
bool As(const VtValue &value, T *out)
{
    if (!value.IsHolding<T>()) return false;
    *out = value.UncheckedGet<T>();
    return true;
}

bool LeftHanded(const VtValue &value)
{
    if (value.IsHolding<TfToken>()) return value.UncheckedGet<TfToken>() == _leftHanded;
    if (value.IsHolding<std::string>()) return value.UncheckedGet<std::string>() == "leftHanded";
    return false;
}

float FloatOr(const VtValue &value, float fallback)
{
    if (value.IsHolding<float>()) return value.UncheckedGet<float>();
    if (value.IsHolding<double>()) return float(value.UncheckedGet<double>());
    return fallback;
}

// The stage's time codes per second the binding carries, else 24.
double StageRate(const RigExecRevisionBinding &binding)
{
    if (binding.externalCompileData.IsHolding<double>()) {
        const double rate = binding.externalCompileData.UncheckedGet<double>();
        if (rate > 0.0 && std::isfinite(rate)) return rate;
    }
    return 24.0;
}

std::vector<int32_t> Ints(const VtIntArray &a)
{
    return std::vector<int32_t>(a.begin(), a.end());
}

// The declared inputs as the payload: a dictionary of the epoch's arrays
// and settings, the parameters and the collider points. False when an
// input is missing or of another type.
bool Payload(const std::vector<VtValue> &in, const RigExecRevisionBinding &binding,
             VtValue *out)
{
    if (in.size() < RigExecClothFirstCollider ||
        (in.size() - RigExecClothFirstCollider) % RigExecClothInputsPerCollider != 0) {
        return false;
    }
    VtIntArray counts, indices;
    VtVec3fArray rest;
    VtFloatArray pins;
    int windowFrames = kWindowFrames, substeps = kSubsteps, iterations = kIterations;
    bool tethers = true;
    double frameRate = 0.0;
    if (!As(in[RigExecClothCounts], &counts) || !As(in[RigExecClothIndices], &indices) ||
        !As(in[RigExecClothRest], &rest)) {
        return false;
    }
    As(in[RigExecClothPins], &pins);
    As(in[RigExecClothWindowFrames], &windowFrames);
    As(in[RigExecClothSubsteps], &substeps);
    As(in[RigExecClothIterations], &iterations);
    As(in[RigExecClothTethers], &tethers);
    As(in[RigExecClothFrameRate], &frameRate);
    const float slack = FloatOr(in[RigExecClothTetherSlack], kTetherSlack);
    const double rate = frameRate > 0.0 && std::isfinite(frameRate)
                            ? frameRate : StageRate(binding);
    const RigExecClothParameters fallback;
    VtFloatArray params(RigExecClothParameterCount);
    params[0] = FloatOr(in[RigExecClothStretchCompliance], fallback.stretchCompliance);
    params[1] = FloatOr(in[RigExecClothBendCompliance], fallback.bendCompliance);
    GfVec3f gravity(fallback.gravity[0], fallback.gravity[1], fallback.gravity[2]);
    As(in[RigExecClothGravity], &gravity);
    params[2] = gravity[0];
    params[3] = gravity[1];
    params[4] = gravity[2];
    params[5] = FloatOr(in[RigExecClothDamping], fallback.damping);
    params[6] = FloatOr(in[RigExecClothAttract], fallback.attract);
    params[7] = FloatOr(in[RigExecClothThickness], fallback.thickness);
    params[8] = FloatOr(in[RigExecClothCollisionDepth], fallback.collisionDepth);
    params[9] = FloatOr(in[RigExecClothFriction], fallback.friction);
    VtDictionary d;
    d["counts"] = VtValue(counts);
    d["indices"] = VtValue(indices);
    d["leftHanded"] = VtValue(LeftHanded(in[RigExecClothOrientation]));
    d["rest"] = VtValue(rest);
    d["pins"] = VtValue(pins);
    d["windowFrames"] = VtValue(windowFrames);
    d["substeps"] = VtValue(substeps);
    d["iterations"] = VtValue(iterations);
    d["tethers"] = VtValue(tethers);
    d["tetherSlack"] = VtValue(slack);
    d["frameSeconds"] = VtValue(1.0 / rate);
    d["params"] = VtValue(params);
    const size_t colliders =
        (in.size() - RigExecClothFirstCollider) / RigExecClothInputsPerCollider;
    std::vector<VtValue> points(colliders), cCounts(colliders), cIndices(colliders),
        cLeft(colliders), cPaths(colliders);
    for (size_t k = 0; k < colliders; ++k) {
        const size_t at = RigExecClothFirstCollider + k * RigExecClothInputsPerCollider;
        VtVec3fArray p;
        VtIntArray c, i;
        if (!As(in[at], &p) || !As(in[at + 1], &c) || !As(in[at + 2], &i)) return false;
        points[k] = VtValue(p);
        cCounts[k] = VtValue(c);
        cIndices[k] = VtValue(i);
        cLeft[k] = VtValue(LeftHanded(in[at + 3]));
        // The collider's points property, which playback names its phased
        // points by.
        if (at >= binding.externalInputs.size()) return false;
        cPaths[k] = VtValue(binding.externalInputs[at].path.GetString());
    }
    d["colliderPoints"] = VtValue(points);
    d["colliderCounts"] = VtValue(cCounts);
    d["colliderIndices"] = VtValue(cIndices);
    d["colliderLeftHanded"] = VtValue(cLeft);
    d["colliderPaths"] = VtValue(cPaths);
    *out = VtValue(d);
    return true;
}

template <class T>
const T *Get(const VtDictionary &d, const char *key)
{
    const auto found = d.find(key);
    return found != d.end() && found->second.IsHolding<T>()
               ? &found->second.UncheckedGet<T>()
               : nullptr;
}

// The payload's epoch. False on a payload Payload did not build.
bool EpochOf(const VtValue &data, RigExecClothEpoch *epoch)
{
    if (!data.IsHolding<VtDictionary>()) return false;
    const VtDictionary &d = data.UncheckedGet<VtDictionary>();
    const auto *counts = Get<VtIntArray>(d, "counts");
    const auto *indices = Get<VtIntArray>(d, "indices");
    const auto *left = Get<bool>(d, "leftHanded");
    const auto *rest = Get<VtVec3fArray>(d, "rest");
    const auto *pins = Get<VtFloatArray>(d, "pins");
    const auto *windowFrames = Get<int>(d, "windowFrames");
    const auto *substeps = Get<int>(d, "substeps");
    const auto *iterations = Get<int>(d, "iterations");
    const auto *tethers = Get<bool>(d, "tethers");
    const auto *slack = Get<float>(d, "tetherSlack");
    const auto *frameSeconds = Get<double>(d, "frameSeconds");
    const auto *cCounts = Get<std::vector<VtValue>>(d, "colliderCounts");
    const auto *cIndices = Get<std::vector<VtValue>>(d, "colliderIndices");
    const auto *cLeft = Get<std::vector<VtValue>>(d, "colliderLeftHanded");
    const auto *cPaths = Get<std::vector<VtValue>>(d, "colliderPaths");
    const auto *cPoints = Get<std::vector<VtValue>>(d, "colliderPoints");
    if (!counts || !indices || !left || !rest || !pins || !windowFrames ||
        !substeps || !iterations || !tethers || !slack || !frameSeconds ||
        !cCounts || !cIndices || !cLeft || !cPaths || !cPoints ||
        cCounts->size() != cPoints->size() || cIndices->size() != cPoints->size() ||
        cLeft->size() != cPoints->size() || cPaths->size() != cPoints->size()) {
        return false;
    }
    RigExecClothEpoch e;
    e.pointCount = uint32_t(rest->size());
    e.counts = Ints(*counts);
    e.indices = Ints(*indices);
    e.leftHanded = *left;
    e.rest.resize(3 * rest->size());
    if (!rest->empty()) std::memcpy(e.rest.data(), rest->cdata(), e.rest.size() * sizeof(float));
    e.pins.assign(pins->begin(), pins->end());
    e.settings.windowFrames = *windowFrames;
    e.settings.substeps = *substeps;
    e.settings.iterations = *iterations;
    e.settings.tethers = *tethers;
    e.settings.tetherSlack = *slack;
    e.frameSeconds = *frameSeconds;
    for (size_t k = 0; k < cPoints->size(); ++k) {
        RigExecClothColliderTopology c;
        VtIntArray ci, cc;
        VtVec3fArray cp;
        bool cl = false;
        std::string path;
        if (!As((*cCounts)[k], &cc) || !As((*cIndices)[k], &ci) ||
            !As((*cLeft)[k], &cl) || !As((*cPaths)[k], &path) ||
            !As((*cPoints)[k], &cp)) {
            return false;
        }
        c.path = path;
        c.pointCount = uint32_t(cp.size());
        c.counts = Ints(cc);
        c.indices = Ints(ci);
        c.leftHanded = cl;
        e.colliders.push_back(std::move(c));
    }
    *epoch = std::move(e);
    return true;
}

void Bind(const RigExecMoverBindContext &ctx)
{
    auto &b = *ctx.binding;
    const RigExecReadPhase phase = RigExecPhaseForInput(ctx.moverPrim, "rigExec:colliders");
    if (!phase.IsBase()) {
        for (const SdfPath &collider : CollidersOf(ctx.moverPrim))
            b.phases[collider.AppendProperty(_points)] = phase;
    }
    // The window length is the history the program runs: an edit rebuilds.
    b.externalStructure.push_back(ctx.moverPrim.GetPath().AppendProperty(_windowFrames));
    b.externalCompileData =
        VtValue(double(ctx.moverPrim.GetStage()->GetTimeCodesPerSecond()));
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs)
{
    *inputs = DeclaredInputs(ctx.moverPrim.GetPath(), ctx.target,
                             CollidersOf(ctx.moverPrim));
}

bool CompileScene(const RigExecSceneDescriptors &scene, const SdfPath &mover,
                  const SdfPath &target, RigExecRevisionBinding *binding,
                  std::string *error)
{
    if (!binding) return false;
    SdfPathVector colliders;
    const auto found = scene.relationships.find(mover.AppendProperty(_colliders));
    if (found != scene.relationships.end()) {
        for (const SdfPath &path : found->second.fact.targets) colliders.push_back(MeshOf(path));
        RigExecReadPhase phase;
        if (!RigExecParseReadPhase(found->second.readPhase.GetString(), &phase, error))
            return false;
        if (!phase.IsBase()) {
            for (const SdfPath &collider : colliders)
                binding->phases[collider.AppendProperty(_points)] = phase;
        }
    }
    binding->externalStructure.push_back(mover.AppendProperty(_windowFrames));
    binding->externalCompileData = VtValue(double(scene.timeCodesPerSecond));
    binding->externalInputs = DeclaredInputs(mover, target, colliders);
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *out)
{
    return Payload(ctx.inputs, ctx.binding, out);
}

// Outside a window a cloth solves one tick, which is its entering points.
bool Apply(const VtValue &data, std::vector<GfVec3f> *output)
{
    return output && data.IsHolding<VtDictionary>();
}

bool CaptureWindow(const VtValue &data, const std::vector<GfVec3f> &entering,
                   RigExecWindowPacket *packet)
{
    if (!packet || !data.IsHolding<VtDictionary>()) return false;
    const VtDictionary &d = data.UncheckedGet<VtDictionary>();
    const auto *params = Get<VtFloatArray>(d, "params");
    const auto *colliders = Get<std::vector<VtValue>>(d, "colliderPoints");
    if (!params || params->size() != RigExecClothParameterCount || !colliders)
        return false;
    size_t total = 3 * entering.size();
    for (const VtValue &value : *colliders) {
        if (!value.IsHolding<VtVec3fArray>()) return false;
        total += 3 * value.UncheckedGet<VtVec3fArray>().size();
    }
    packet->points.resize(total);
    float *at = packet->points.data();
    if (!entering.empty())
        std::memcpy(at, entering.data(), 3 * entering.size() * sizeof(float));
    at += 3 * entering.size();
    for (const VtValue &value : *colliders) {
        const VtVec3fArray &p = value.UncheckedGet<VtVec3fArray>();
        if (!p.empty()) std::memcpy(at, p.cdata(), 3 * p.size() * sizeof(float));
        at += 3 * p.size();
    }
    packet->scalars.assign(params->begin(), params->end());
    return true;
}

// Names each prepared epoch, so a device re-uploads a mesh only when it
// moved. Constant-initialized: no guard stands in front of it.
std::atomic<uint64_t> _preparedSerial{0};

// The prepared epoch the revision keeps: reused while the epoch holds the
// same bits, built again when it does not.
std::shared_ptr<const RigExecClothPrepared>
PreparedFor(const VtValue &data, std::shared_ptr<const void> *cache)
{
    RigExecClothEpoch epoch;
    if (!EpochOf(data, &epoch)) return nullptr;
    if (cache && *cache) {
        auto held = std::static_pointer_cast<const RigExecClothPrepared>(*cache);
        if (held->epoch == epoch) return held;
    }
    auto built = std::make_shared<RigExecClothPrepared>();
    std::string why;
    if (!RigExecPrepareCloth(epoch, built.get(), &why)) return nullptr;
    built->serial = ++_preparedSerial;
    if (cache) *cache = built;
    return built;
}

bool ApplyWindow(const VtValue &data, const RigExecWindowPacket *const *ticks,
                 size_t tickCount, std::vector<GfVec3f> *points,
                 std::shared_ptr<const void> *cache)
{
    if (!points) return false;
    const auto prepared = PreparedFor(data, cache);
    if (!prepared || points->size() != prepared->epoch.pointCount) return false;
    return RigExecClothSolvePackets(*prepared, ticks, tickCount,
                                    reinterpret_cast<float *>(points->data()),
                                    nullptr);
}

// The same window on the registered device (rigExec/gpuCloth.h); false when
// none solved it, and the caller solves on the CPU.
bool ApplyWindowDevice(const VtValue &data, const RigExecWindowPacket *const *ticks,
                       size_t tickCount, std::vector<GfVec3f> *points,
                       std::shared_ptr<const void> *cache)
{
    const RigExecGpuClothRunner runner = RigExecGetGpuClothRunner();
    if (!points || !runner) return false;
    const auto prepared = PreparedFor(data, cache);
    if (!prepared || points->size() != prepared->epoch.pointCount) return false;
    std::vector<RigExecClothTick> window;
    if (!RigExecClothTicksFromPackets(*prepared, ticks, tickCount, &window, nullptr) ||
        window.size() < 2 ||
        !RigExecClothWindowUsable(prepared->mesh, prepared->colliders,
                                  window.data(), window.size(), nullptr)) {
        return false;
    }
    std::vector<float> out(3 * points->size());
    std::vector<RigExecGpuClothJob> jobs(1);
    RigExecGpuClothJob &job = jobs[0];
    job.key = uint64_t(reinterpret_cast<uintptr_t>(cache));
    job.epochSerial = prepared->serial;
    job.mesh = &prepared->mesh;
    job.colliders = &prepared->colliders;
    job.settings = prepared->epoch.settings;
    job.frameSeconds = prepared->epoch.frameSeconds;
    job.ticks = window.data();
    job.tickCount = window.size();
    job.out = out.data();
    if (!runner(&jobs) || !job.ok) return false;
    for (const float value : out)
        if (!std::isfinite(value)) return false;
    std::memcpy(points->data(), out.data(), out.size() * sizeof(float));
    return true;
}

uint32_t WindowFrames(const UsdPrim &mover)
{
    int frames = kWindowFrames;
    if (const UsdAttribute a = mover.GetAttribute(_windowFrames)) a.Get(&frames);
    return uint32_t(std::max(1, std::min(frames, RigExecClothMaxWindowFrames)));
}

// The epoch from the bake's payload, and no frame bytes: the parameters and
// collider points reach playback as inputs and phased points.
bool Encode(const VtValue &data, const RigExecRevisionBinding &,
            std::vector<uint8_t> *epoch, std::vector<uint8_t> *frame)
{
    RigExecClothEpoch decoded;
    if (!EpochOf(data, &decoded)) return false;
    RigExecEncodeClothEpoch(decoded, epoch);
    frame->clear();
    return true;
}

bool Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    const std::string who = ctx.prim.GetPath().GetString();
    if (ctx.targets.size() != 1) {
        *error = who + ": cloth mover requires a single target";
        return false;
    }
    const UsdPrim mesh = ctx.stage->GetPrimAtPath(ctx.targets[0].GetPrimPath());
    if (!mesh || !mesh.IsA<UsdGeomMesh>() || ctx.targets[0].GetNameToken() != _points) {
        *error = who + ": cloth mover must move a Mesh's points";
        return false;
    }
    VtIntArray counts;
    VtVec3fArray rest;
    const UsdGeomMesh geom(mesh);
    if (!geom.GetFaceVertexCountsAttr().Get(&counts) || counts.empty() ||
        !geom.GetPointsAttr().Get(&rest) || rest.empty()) {
        *error = who + ": cloth mesh " + mesh.GetPath().GetString() +
                 " needs authored points and faceVertexCounts";
        return false;
    }
    VtFloatArray pins;
    if (const UsdAttribute a = ctx.prim.GetAttribute(_pinWeights)) a.Get(&pins);
    if (!pins.empty() && pins.size() != rest.size()) {
        *error = who + ": rigExec:pinWeights holds " + std::to_string(pins.size()) +
                 " weights; the cloth mesh has " + std::to_string(rest.size()) +
                 " points";
        return false;
    }
    const auto intIn = [&](const TfToken &name, int fallback, int lo, int hi) {
        int value = fallback;
        if (const UsdAttribute a = ctx.prim.GetAttribute(name)) a.Get(&value);
        return value >= lo && value <= hi;
    };
    if (!intIn(_windowFrames, kWindowFrames, 1, RigExecClothMaxWindowFrames) ||
        !intIn(_substeps, kSubsteps, 1, RigExecClothMaxSubsteps) ||
        !intIn(_iterations, kIterations, 1, RigExecClothMaxIterations)) {
        *error = who + ": rigExec:windowFrames, substeps or iterations is out of range";
        return false;
    }
    for (const SdfPath &collider : CollidersOf(ctx.prim)) {
        const UsdPrim prim = ctx.stage->GetPrimAtPath(collider);
        if (!prim || !prim.IsA<UsdGeomMesh>()) {
            *error = who + ": rigExec:colliders target " + collider.GetString() +
                     " is not a Mesh";
            return false;
        }
        if (collider == mesh.GetPath()) {
            *error = who + ": the cloth mesh cannot collide with itself through "
                           "rigExec:colliders";
            return false;
        }
    }
    return true;
}

RigExecMoverHandler Handler()
{
    RigExecMoverHandler h(
        "RigExecClothMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    h.singleTarget = true;
    h.hasScalarOracle = false;
    h.bind = &Bind;
    h.validate = &Validate;
    h.declareExternalInputs = &DeclareInputs;
    h.compileScene = &CompileScene;
    h.assembleExternal = &Assemble;
    h.applyExternal = &Apply;
    h.encodeExternal = &Encode;
    h.windowFrames = &WindowFrames;
    h.captureWindow = &CaptureWindow;
    h.applyWindow = &ApplyWindow;
    h.applyWindowDevice = &ApplyWindowDevice;
    h.runtimeKernel = RigExecClothPlaybackKernel();
    return h;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(Handler());
