// RigExec imaging preview surface: the weight-overlay selection and the
// manipulation preview -- declared slots sampled per tick into evaluator
// overrides and chain xform deltas. Authoring-free: overrides and deltas
// only, never stage writes.
#include "registry.h"

#include "rigExec/parallel.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/hash.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/pcp/mapFunction.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// An overlay selection string, checked before it reaches SdfPath (whose
// constructor is loud about a malformed one). Empty turns the overlay off.
bool
_ParseOverlayPath(const std::string &text, SdfPath *path)
{
    *path = SdfPath();
    if (text.empty()) {
        return true;
    }
    if (!SdfPath::IsValidPathString(text)) {
        return false;
    }
    const SdfPath resolved(text);
    if (!resolved.IsAbsolutePath() || !resolved.IsPrimPath()) {
        return false;
    }
    *path = resolved;
    return true;
}

} // namespace

UsdStageRefPtr
RigExecImagingRegistry::_PreviewStageLocked() const
{
    if (_stage) {
        return _stage;
    }
    return UsdStageRefPtr(_boundStage);
}

bool
RigExecImagingRegistry::SetWeightOverlay(const std::string &weightPrimPath)
{
    UsdTimeCode time;
    // Fenced cancel-THEN-clear like ClearFrameCache: the fence
    // is outermost, so it is taken before the registry lock below and held
    // across every session's cancel plus clear.
    // Held, not borrowed: a Deactivate while the fence is held stops the
    // pool, and the scheduler (and its fence mutex) must outlive the lock.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // An arbitrary caller-supplied string reaches SdfPath here and its
        // constructor is loud about a malformed one. Ask first -- the same
        // guard RigExecImaging_GetGuideBoundsAssetSpaceForStage uses.
        SdfPath resolved;
        if (!_ParseOverlayPath(weightPrimPath, &resolved)) {
            return false;
        }
        if (_weightOverlay == resolved)
            return true;
        _weightOverlay = resolved;
        // The overlay changes what every frame evaluates, so a preview's
        // identity retires with it: nothing keyed to the previous overlay
        // can be matched to a destination under this one.
        _RetirePreviewIdentityLocked();
        if (_sessions.empty()) {
            // Remembered for whenever a rig is activated. Not a failure:
            // a host that sets its viewer mode before opening a stage has
            // done nothing wrong.
            return true;
        }
        // Offered to every active rig: the path selects at most one rig's
        // weight object, and a bridge that does not own it draws no overlay.
        for (RigSession &session : _sessions) {
            if (session.playback) {
                session.playback->SetWeightOverlay(resolved);
            } else {
                // Cancel BEFORE the bridge clears: the overlay
                // flag rides to workers in context.flags and changes pose
                // content without moving the key, so the old clear-then-
                // cancel order let a worker publish a stale-overlay pose
                // into the cleared cache between the two. Retired jobs
                // re-enqueue fresh on the next commit.
                _CancelGenerationLocked(session.rigPath);
                session.bridge->SetWeightOverlay(resolved);
            }
            session.dirty = true;
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock, exactly as _OnObjectsChanged does it:
    // SetTime takes the same plain (non-recursive) std::mutex, so calling
    // it while still holding _mutex is a self-deadlock. This is also what
    // makes the change visible immediately -- the fresh generation's diff
    // reports the overlay's displayColor primvar appearing or disappearing
    // as structural, which dirties the mesh universally.
    return SetTime(time);
}

// Manipulation preview

namespace {

// The largest element count an array slot may declare.
constexpr size_t kMaxPreviewElements = size_t(1) << 20;
const TfToken kPreviewFloatArray("float[]");

// A declaration line's element count: plain decimal digits only, at most
// kMaxPreviewElements.
bool
_ParsePreviewElementCount(const std::string &text, size_t *count)
{
    if (text.empty()) {
        return false;
    }
    size_t value = 0;
    for (const char c : text) {
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + size_t(c - '0');
        if (value > kMaxPreviewElements) {
            return false;
        }
    }
    *count = value;
    return true;
}

// Doubles per element and the builder-kind token for each declared type;
// an empty token means the type cannot be previewed. An element count is
// required for, and only accepted on, an array slot. The rule table keys
// on the codified SdfValueTypeNames tokens; unlisted types are checked
// against their stored representation, which admits custom vec3f-array-
// backed types.
const TfToken kPreviewDouble("double"), kPreviewFloat("float"),
    kPreviewVec3d("vec3d"), kPreviewVec3f("vec3f"),
    kPreviewMatrix4d("matrix4d"), kPreviewFloat2Array("float2[]"),
    kPreviewVec3fArray("vec3f[]");

VtValue
_BuildFloatArrayValue(const double *v, size_t arity)
{
    VtFloatArray array(arity);
    float *out = array.data();
    for (size_t i = 0; i < arity; ++i)
        out[i] = float(v[i]);
    return VtValue(std::move(array));
}

VtValue
_BuildFloat2ArrayValue(const double *v, size_t arity)
{
    VtVec2fArray array(arity / 2);
    GfVec2f *out = array.data();
    for (size_t i = 0; i < array.size(); ++i) {
        out[i] = GfVec2f(float(v[2 * i]), float(v[2 * i + 1]));
    }
    return VtValue(std::move(array));
}

VtValue
_BuildVec3fArrayValue(const double *v, size_t arity)
{
    VtVec3fArray array(arity / 3);
    GfVec3f *out = array.data();
    for (size_t i = 0; i < array.size(); ++i) {
        out[i] =
            GfVec3f(float(v[3 * i]), float(v[3 * i + 1]), float(v[3 * i + 2]));
    }
    return VtValue(std::move(array));
}

VtValue
_BuildDoubleValue(const double *v, size_t)
{
    return VtValue(v[0]);
}

VtValue
_BuildFloatValue(const double *v, size_t)
{
    return VtValue(float(v[0]));
}

VtValue
_BuildVec3dValue(const double *v, size_t)
{
    return VtValue(GfVec3d(v[0], v[1], v[2]));
}

VtValue
_BuildVec3fValue(const double *v, size_t)
{
    return VtValue(GfVec3f(float(v[0]), float(v[1]), float(v[2])));
}

VtValue
_BuildMatrix4dValue(const double *v, size_t)
{
    return VtValue(GfMatrix4d(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
                              v[8], v[9], v[10], v[11], v[12], v[13], v[14],
                              v[15]));
}

// Declared type token -> (value kind, doubles per element, array kind).
struct PreviewTypeRule {
    TfToken kind;
    size_t perElement;
    bool arrayKind;
};

const std::unordered_map<TfToken, PreviewTypeRule, TfHash> kPreviewTypeRules = {
    {SdfValueTypeNames->FloatArray.GetAsToken(), {kPreviewFloatArray, 1, true}},
    {SdfValueTypeNames->Float2Array.GetAsToken(),
     {kPreviewFloat2Array, 2, true}},
    {SdfValueTypeNames->Point3fArray.GetAsToken(),
     {kPreviewVec3fArray, 3, true}},
    {SdfValueTypeNames->Float3Array.GetAsToken(),
     {kPreviewVec3fArray, 3, true}},
    {SdfValueTypeNames->Vector3fArray.GetAsToken(),
     {kPreviewVec3fArray, 3, true}},
    {SdfValueTypeNames->Normal3fArray.GetAsToken(),
     {kPreviewVec3fArray, 3, true}},
    {SdfValueTypeNames->Double.GetAsToken(), {kPreviewDouble, 1, false}},
    {SdfValueTypeNames->Float.GetAsToken(), {kPreviewFloat, 1, false}},
    {SdfValueTypeNames->Double3.GetAsToken(), {kPreviewVec3d, 3, false}},
    {SdfValueTypeNames->Vector3d.GetAsToken(), {kPreviewVec3d, 3, false}},
    {SdfValueTypeNames->Point3d.GetAsToken(), {kPreviewVec3d, 3, false}},
    {SdfValueTypeNames->Normal3d.GetAsToken(), {kPreviewVec3d, 3, false}},
    {SdfValueTypeNames->Float3.GetAsToken(), {kPreviewVec3f, 3, false}},
    {SdfValueTypeNames->Vector3f.GetAsToken(), {kPreviewVec3f, 3, false}},
    {SdfValueTypeNames->Point3f.GetAsToken(), {kPreviewVec3f, 3, false}},
    {SdfValueTypeNames->Normal3f.GetAsToken(), {kPreviewVec3f, 3, false}},
    {SdfValueTypeNames->Matrix4d.GetAsToken(), {kPreviewMatrix4d, 16, false}},
};

// Value kind -> the builder that reconstructs one sample from the slot's
// doubles.
const std::unordered_map<TfToken, VtValue (*)(const double *, size_t), TfHash>
    kPreviewValueBuilders = {
        {kPreviewFloatArray, &_BuildFloatArrayValue},
        {kPreviewFloat2Array, &_BuildFloat2ArrayValue},
        {kPreviewVec3fArray, &_BuildVec3fArrayValue},
        {kPreviewDouble, &_BuildDoubleValue},
        {kPreviewFloat, &_BuildFloatValue},
        {kPreviewVec3d, &_BuildVec3dValue},
        {kPreviewVec3f, &_BuildVec3fValue},
        {kPreviewMatrix4d, &_BuildMatrix4dValue},
};

// A custom type held as a VtVec3fArray previews as a vec3f[].
const PreviewTypeRule kVec3fBackedRule{kPreviewVec3fArray, 3, true};

TfToken
_PreviewValueKind(const SdfValueTypeName &type, bool hasElementCount,
                  size_t elementCount, size_t *arity)
{
    *arity = 0;
    const auto rule = kPreviewTypeRules.find(type.GetAsToken());
    const PreviewTypeRule *matched = nullptr;
    if (rule != kPreviewTypeRules.end()) {
        matched = &rule->second;
    } else if (type.GetType() == TfType::Find<VtVec3fArray>()) {
        matched = &kVec3fBackedRule;
    } else {
        return TfToken();
    }
    if (matched->arrayKind) {
        if (!hasElementCount) {
            return TfToken();
        }
        *arity = matched->perElement * elementCount;
    } else {
        if (hasElementCount) {
            return TfToken();
        }
        *arity = matched->perElement;
    }
    return matched->kind;
}

// \p arity is the slot's double count; a float2[] slot reads it as
// consecutive (x, y) pairs and a vec3f[] slot as (x, y, z) triples.
VtValue
_PreviewValue(const TfToken &kind, const double *v, size_t arity)
{
    const auto builder = kPreviewValueBuilders.find(kind);
    if (builder == kPreviewValueBuilders.end()) {
        return VtValue();
    }
    return builder->second(v, arity);
}

std::vector<std::string>
_SplitLines(const std::string &packed)
{
    std::vector<std::string> out;
    std::string current;
    for (const char c : packed) {
        if (c == '\n' || c == '\r') {
            if (!current.empty()) {
                out.push_back(current);
                current.clear();
            }
            continue;
        }
        current.push_back(c);
    }
    if (!current.empty()) {
        out.push_back(current);
    }
    return out;
}

// Does \p layer author anything for \p path? A default counts only for a
// layer above the edit target: within the layer that authors them, time
// samples or a spline answer every time, so a default in that same layer
// never reaches the composed value.
bool
_LayerAuthorsAt(const SdfLayerHandle &layer, const SdfPath &path,
                bool defaultCounts)
{
    if (!layer || path.IsEmpty()) {
        return false;
    }
    if (defaultCounts && layer->HasField(path, SdfFieldKeys->Default)) {
        return true;
    }
    return layer->HasField(path, SdfFieldKeys->TimeSamples) ||
           layer->HasField(path, SdfFieldKeys->Spline);
}

} // namespace

int
RigExecImagingRegistry::BeginPreview(const std::string &packedAttributePaths)
{
    std::lock_guard<std::mutex> lock(_mutex);
    // The active stage, else the one this context belongs to: a stage with
    // no rig still previews its Xforms, through this context's chains only.
    const UsdStageRefPtr stage = _PreviewStageLocked();
    if (!stage) {
        return -1;
    }
    std::vector<PreviewSlot> slots;
    size_t total = 0;
    for (const std::string &line : _SplitLines(packedAttributePaths)) {
        // `<path>` or `<path>\t<elementCount>`.
        const size_t tab = line.find('\t');
        const bool hasElementCount = tab != std::string::npos;
        size_t elementCount = 0;
        if (hasElementCount &&
            !_ParsePreviewElementCount(line.substr(tab + 1), &elementCount)) {
            return -1;
        }
        const std::string text = line.substr(0, tab);
        // Caller-supplied strings: ask before constructing, the same guard
        // SetWeightOverlay uses.
        if (!SdfPath::IsValidPathString(text)) {
            return -1;
        }
        const SdfPath path(text);
        if (!path.IsAbsolutePath() || !path.IsPropertyPath()) {
            return -1;
        }
        const UsdAttribute attribute = stage->GetAttributeAtPath(path);
        if (!attribute) {
            return -1;
        }
        PreviewSlot slot;
        slot.primPath = path.GetPrimPath();
        slot.attributeName = path.GetNameToken();
        slot.valueKind =
            _PreviewValueKind(attribute.GetTypeName(), hasElementCount,
                              elementCount, &slot.arity);
        if (slot.valueKind.IsEmpty()) {
            return -1;
        }
        // Which lane. An attribute inside an active rig previews through that
        // rig's evaluator; everything else is a transform of its own.
        // A playback rig offers no evaluator lane: its inputs are baked,
        // so an attribute under one falls through to the xform lane (or
        // is refused, when it is not an xform op either).
        for (const RigSession &session : _sessions) {
            if (!session.playback && slot.primPath.HasPrefix(session.rigPath)) {
                slot.rigPath = session.rigPath;
                break;
            }
        }
        if (slot.rigPath.IsEmpty() &&
            (hasElementCount ||
             !UsdGeomXformOp::IsXformOp(slot.attributeName))) {
            // Nothing to preview: the prim has no rig to re-run and this is
            // not part of its transform (an array never is). Refused rather
            // than silently ignored, because the caller would then be
            // sending values that change nothing and wondering why.
            return -1;
        }
        total += slot.arity;
        // The count is returned as an int.
        if (total > size_t(std::numeric_limits<int>::max())) {
            return -1;
        }
        slots.push_back(std::move(slot));
    }
    _previewSlots = std::move(slots);
    _previewStage = stage;
    _previewActive = true;
    // A new session of the identity: the revision counts ticks within it.
    _RetirePreviewIdentityLocked();
    _previewIdentity.active = true;
    return int(total);
}

bool
RigExecImagingRegistry::IsPreviewActive() const
{
    return _previewActive;
}

RigExecPreviewIdentity
RigExecImagingRegistry::GetPreviewIdentity() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _previewIdentity;
}

bool
RigExecImagingRegistry::DeclarePreviewEdit(
    const std::string &packedAttributePaths, RigExecPreviewEditPolicy policy,
    const double *values, size_t count)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_previewActive) {
        return false;
    }
    const UsdStageRefPtr stage = _PreviewStageLocked();
    if (!stage) {
        return false;
    }
    const std::vector<std::string> lines = _SplitLines(packedAttributePaths);
    if (lines.empty()) {
        return false;
    }
    // Resolve every line before touching the description: a declaration
    // naming one bad path declares nothing.
    std::vector<PreviewEdit> parsed;
    parsed.reserve(lines.size());
    for (const std::string &line : lines) {
        // `<path>`, or `<path>\t<elementCount>` for an array slot -- the
        // spelling BeginPreview already takes.
        const size_t tab = line.find('\t');
        const bool hasElementCount = tab != std::string::npos;
        size_t elementCount = 0;
        if (hasElementCount &&
            !_ParsePreviewElementCount(line.substr(tab + 1), &elementCount)) {
            return false;
        }
        const std::string text = line.substr(0, tab);
        if (!SdfPath::IsValidPathString(text)) {
            return false;
        }
        const SdfPath path(text);
        if (!path.IsAbsolutePath() || !path.IsPropertyPath()) {
            return false;
        }
        const UsdAttribute attribute = stage->GetAttributeAtPath(path);
        if (!attribute) {
            return false;
        }
        size_t arity = 0;
        if (_PreviewValueKind(attribute.GetTypeName(), hasElementCount,
                              elementCount, &arity) == TfToken() ||
            arity == 0) {
            return false;
        }
        PreviewEdit edit;
        edit.attributePath = path;
        edit.policy = policy;
        edit.pending.assign(arity, 0.0);
        parsed.push_back(std::move(edit));
    }
    size_t total = 0;
    for (const PreviewEdit &edit : parsed) {
        total += edit.pending.size();
    }
    if (!values || count != total) {
        return false;
    }
    size_t offset = 0;
    for (PreviewEdit &edit : parsed) {
        for (double &pending : edit.pending) {
            pending = values[offset++];
        }
    }
    for (PreviewEdit &edit : parsed) {
        // A path already in the description keeps its place and takes the
        // new pending values: the description is a set of paths, not a log.
        const auto held = std::find_if(
            _previewEdits.begin(), _previewEdits.end(),
            [&edit](const PreviewEdit &existing) {
                return existing.attributePath == edit.attributePath;
            });
        if (held != _previewEdits.end()) {
            held->policy = edit.policy;
            held->pending = std::move(edit.pending);
        } else {
            _previewEdits.push_back(std::move(edit));
        }
    }
    return true;
}

RigExecPreviewEditStats
RigExecImagingRegistry::GetPreviewEditStats() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecPreviewEditStats stats;
    stats.identity = _previewIdentity;
    stats.editCount = _previewEdits.size();
    for (const PreviewEdit &edit : _previewEdits) {
        stats.valueCount += edit.pending.size();
    }
    stats.policy = _previewEdits.empty()
                       ? RigExecPreviewEditPolicy::AttributeDefault
                       : _previewEdits.front().policy;
    return stats;
}

bool
RigExecImagingRegistry::PreviewValueIsIndependent(
    const std::string &attributePathText, UsdTimeCode time,
    std::string *reason) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (!SdfPath::IsValidPathString(attributePathText)) {
        if (reason) {
            *reason = "the path is not a valid Sdf path";
        }
        return false;
    }
    return _PreviewValueIsIndependentLocked(SdfPath(attributePathText), time,
                                            reason);
}

void
RigExecImagingRegistry::_RetirePreviewIdentityLocked()
{
    // A retired session never comes back: the next BeginPreview opens the
    // next one, and nothing can key a frame to a closed session.
    ++_previewIdentity.session;
    _previewIdentity.revision = 0;
    _previewIdentity.active = false;
    _previewEdits.clear();
}

void
RigExecImagingRegistry::_RetirePreviewIdentityForNoticeLocked(
    const UsdNotice::ObjectsChanged &notice)
{
    for (const PreviewEdit &edit : _previewEdits) {
        const SdfPath primPath = edit.attributePath.GetPrimPath();
        // Undo, redo, and any other stage opinion on a declared path: the
        // description no longer describes what the stage will hold.
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (path == edit.attributePath || path == primPath) {
                _RetirePreviewIdentityLocked();
                return;
            }
        }
        for (const SdfPath &path : notice.GetResyncedPaths()) {
            // A resync re-specifies its path and everything below it, and
            // the notice lists the ancestor, not the respecified descendants,
            // so a declared attribute is covered by a resync at or above it.
            if (path == edit.attributePath ||
                edit.attributePath.HasPrefix(path)) {
                _RetirePreviewIdentityLocked();
                return;
            }
        }
    }
}

bool
RigExecImagingRegistry::_PreviewValueIsIndependentLocked(
    const SdfPath &attributePath, UsdTimeCode time, std::string *reason) const
{
    auto refuse = [reason](const std::string &why) {
        if (reason) {
            *reason = why;
        }
        return false;
    };
    auto accept = [reason](const std::string &why) {
        if (reason) {
            *reason = why;
        }
        return true;
    };
    // The same stage the declaration was made against: a preview can stand
    // on a context that has not compiled a rig yet.
    const UsdStageRefPtr stage = _PreviewStageLocked();
    if (!stage) {
        return refuse("this context holds no stage");
    }
    if (!attributePath.IsAbsolutePath() || !attributePath.IsPropertyPath()) {
        return refuse("the path is not an absolute attribute path");
    }
    const auto declared =
        std::find_if(_previewEdits.begin(), _previewEdits.end(),
                     [&attributePath](const PreviewEdit &edit) {
                         return edit.attributePath == attributePath;
                     });
    if (declared == _previewEdits.end()) {
        return refuse("the edit description declares no pending value here");
    }
    if (declared->policy != RigExecPreviewEditPolicy::AttributeDefault) {
        // A pending key or connection is not a default opinion; every rule
        // below is about a default that cannot reach the composed value.
        return refuse("the pending value is not a default opinion");
    }
    if (time.IsDefault()) {
        return refuse("the candidate time is the default time, where the "
                      "pending default is the value");
    }
    const UsdAttribute attribute = stage->GetAttributeAtPath(attributePath);
    if (!attribute) {
        return refuse("the stage has no attribute at this path");
    }
    if (attribute.HasAuthoredConnections()) {
        return refuse("the attribute has authored connections, so its "
                      "composed value is not reasoned about here");
    }
    for (const RigSession &session : _sessions) {
        if (session.bridge &&
            session.bridge->GetEvaluator().IsPropertyChainTarget(
                attributePath)) {
            return refuse("a compiled property chain revises this attribute");
        }
    }
    const UsdEditTarget target = stage->GetEditTarget();
    const SdfLayerHandle targetLayer = target.GetLayer();
    if (!targetLayer) {
        return refuse("this stage has no edit-target layer");
    }
    // Every layer rule below reads the composed path in a layer of this
    // stage's own stack, which is where a local edit target authors it. A
    // mapped target (a reference or a variant) writes another spec path, and
    // a same-spelled spec in that other layer composes nowhere here: the
    // fields read below are not the ones the pending value would reach.
    if (!target.GetMapFunction().IsIdentity() ||
        target.MapToSpecPath(attributePath) != attributePath) {
        return refuse("the edit target maps this composed path to another "
                      "spec path, so the pending value would be authored "
                      "somewhere this proof cannot read");
    }
    const std::vector<SdfLayerHandle> stack = stage->GetLayerStack();
    size_t targetIndex = stack.size();
    for (size_t i = 0; i < stack.size(); ++i) {
        if (stack[i] == targetLayer) {
            targetIndex = i;
            break;
        }
    }
    if (targetIndex == stack.size()) {
        // Not one of the stage's own layers: a non-local target, whose
        // strength against this stack is not something a flat read settles.
        return refuse("the edit-target layer is not in this stage's layer "
                      "stack, so its composition strength is unknown here");
    }
    // Anything authored above the edit target decides the composed answer
    // at every time, and a pending value in the target can never reach it.
    for (size_t i = 0; i < targetIndex; ++i) {
        if (_LayerAuthorsAt(stack[i], attributePath, true)) {
            return accept("a layer stronger than the edit target authors this "
                          "attribute, so the pending default never applies");
        }
    }
    if (_LayerAuthorsAt(targetLayer, attributePath, false)) {
        return accept("authored time samples in the edit target layer answer "
                      "every time, and a same-layer default never hides them");
    }
    return refuse("nothing but the pending default answers this time");
}

bool
RigExecImagingRegistry::_ComposeXformDelta(
    const UsdPrim &prim, const std::map<TfToken, VtValue> &opValues,
    UsdGeomXformCache *cache, GfMatrix4d *delta) const
{
    const UsdGeomXformable xformable(prim);
    if (!xformable || !cache || !delta) {
        return false;
    }
    bool resetsXformStack = false;
    const std::vector<UsdGeomXformOp> ops =
        xformable.GetOrderedXformOps(&resetsXformStack);

    // The authored local transform and the previewed one, composed the same
    // way out of the same ops, so the only difference between them is the
    // overridden values. Composing op by op rather than calling
    // GetLocalTransformation twice is what lets an override stand in for one
    // op's value without being authored anywhere.
    GfMatrix4d authored(1.0), previewed(1.0);
    const UsdTimeCode time = cache->GetTime();
    for (const UsdGeomXformOp &op : ops) {
        VtValue value;
        const bool hasAuthoredValue = op.GetAttr().Get(&value, time);
        if (hasAuthoredValue) {
            authored =
                op.GetOpTransform(op.GetOpType(), value, op.IsInverseOp()) *
                authored;
        }
        // An inverse op shares its attribute with the forward op; the
        // !invert! marker belongs to the order token, not the value key.
        const auto found = opValues.find(op.GetAttr().GetName());
        // A newly created ordered op has no authored value yet. It is an
        // identity in the authored transform but its preview still applies.
        if (found == opValues.end() && !hasAuthoredValue) {
            continue;
        }
        const VtValue &previewValue =
            found == opValues.end() ? value : found->second;
        previewed =
            op.GetOpTransform(op.GetOpType(), previewValue, op.IsInverseOp()) *
            previewed;
    }

    // delta = parentToWorld^-1 . local^-1 . local' . parentToWorld
    // which is exactly xform^-1 . xform', the quantity
    // RigExecXformOverrideSceneIndex post-multiplies. A prim that resets the
    // xform stack has no parent contribution, and then the conjugation
    // collapses to local^-1 . local' on its own.
    const GfMatrix4d parentToWorld =
        resetsXformStack ? GfMatrix4d(1.0)
                         : cache->GetParentToWorldTransform(prim);
    const GfMatrix4d parentInverse = parentToWorld.GetInverse();
    *delta = parentInverse * authored.GetInverse() * previewed * parentToWorld;
    return true;
}

bool
RigExecImagingRegistry::_ResolvePreviewSample(
    const double *values, size_t count,
    std::map<SdfPath, std::vector<RigExecValueOverride>> *byRig,
    std::map<SdfPath, GfMatrix4d> *xformDeltas) const
{
    size_t offset = 0;
    std::map<SdfPath, std::map<TfToken, VtValue>> xformOps;
    for (const PreviewSlot &slot : _previewSlots) {
        if (offset + slot.arity > count) {
            return false;
        }
        const VtValue value =
            _PreviewValue(slot.valueKind, values + offset, slot.arity);
        offset += slot.arity;
        if (value.IsEmpty()) {
            return false;
        }
        if (!slot.rigPath.IsEmpty()) {
            (*byRig)[slot.rigPath].push_back(RigExecValueOverride{
                slot.primPath, TfToken(), slot.attributeName, value});
        } else {
            xformOps[slot.primPath][slot.attributeName] = value;
        }
    }
    if (offset != count) {
        return false;
    }
    if (!xformOps.empty()) {
        UsdGeomXformCache cache(_lastTime);
        for (const auto &[primPath, opValues] : xformOps) {
            const UsdStageRefPtr stage(_previewStage);
            const UsdPrim prim =
                stage ? stage->GetPrimAtPath(primPath) : UsdPrim();
            GfMatrix4d delta(1.0);
            if (!prim || !_ComposeXformDelta(prim, opValues, &cache, &delta)) {
                return false;
            }
            (*xformDeltas)[primPath] = delta;
        }
    }
    return true;
}

void
RigExecImagingRegistry::_SetChainXformDeltas(
    const std::map<SdfPath, GfMatrix4d> &deltas)
{
    _previewXformDeltas = deltas;
    // This context's chains only: a preview on one stage never moves the
    // same path in another stage's viewport. (Sent under _mutex, as
    // before; the xform dirties never intersect the time trigger, so the
    // results index does not re-enter.)
    for (const Chain &chain : _BoundChains()) {
        if (chain.xforms) {
            chain.xforms->SetWorldDeltas(deltas);
        }
    }
}

bool
RigExecImagingRegistry::UpdatePreview(const double *values, size_t count)
{
    UsdTimeCode time;
    bool needsRigPublish = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_previewActive || !_previewStage || (!values && count)) {
            return false;
        }
        std::map<SdfPath, std::vector<RigExecValueOverride>> byRig;
        std::map<SdfPath, GfMatrix4d> xformDeltas;
        if (!_ResolvePreviewSample(values, count, &byRig, &xformDeltas)) {
            return false;
        }
        ++_previewIdentity.revision;
        // The xform lane needs no evaluation at all: the delta goes straight
        // into the chains and the dirty notice it sends is the whole update.
        _SetChainXformDeltas(xformDeltas);

        // Every active rig is told, including the ones with no slots this
        // sample: a rig that was being previewed and no longer is has to stop.
        for (RigSession &session : _sessions) {
            const auto found = byRig.find(session.rigPath);
            if (found == byRig.end()) {
                if (session.bridge) {
                    session.bridge->ClearInteractiveOverrides();
                }
            } else if (session.bridge) {
                session.bridge->SetInteractiveOverrides(found->second);
                needsRigPublish = true;
            }
            // Any new edit bumps the generation and cancels the rig's
            // in-flight jobs. A job sampled before this tick would land
            // under an unreachable key anyway (the digest folds the
            // overrides), but it would still burn a worker on a pose no
            // lookup can reach; the commit trigger re-enqueues fresh.
            _CancelGenerationLocked(session.rigPath);
            session.dirty = true;
            session.settleOverrides.clear();
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock: SetTime takes the same non-recursive mutex
    // (the rule _OnObjectsChanged and SetWeightOverlay both follow).
    // Only when a rig is involved. An xform-lane drag has already dirtied what
    // it changed, and evaluating every active rig to redraw a prim no rig
    // drives would make the cheap lane pay for the expensive one.
    return needsRigPublish ? SetTime(time) : true;
}

bool
RigExecImagingRegistry::EndPreview(bool publish)
{
    UsdTimeCode time;
    bool hadRigOverrides = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _previewSlots.clear();
        _previewActive = false;
        _previewStage = UsdStageWeakPtr();
        _RetirePreviewIdentityLocked();
        _SetChainXformDeltas({});
        for (RigSession &session : _sessions) {
            session.settleOverrides.clear();
            if (session.bridge) {
                // Clean means the last publish evaluated these overrides at
                // _lastTime, so a commit of the same values changes nothing
                // on screen.
                if (!publish && !session.dirty && !session.readRootsDirty &&
                    !_readRootsDirty) {
                    session.settleOverrides =
                        session.bridge->GetInteractiveOverrides();
                }
                session.bridge->ClearInteractiveOverrides();
            }
            session.dirty = true;
            hadRigOverrides = true;
        }
        time = _lastTime;
    }
    // Republished so the authored rig is what is drawn again. The caller
    // authors its committed values separately; when it has already done so,
    // this generation is the committed one and the artist sees no flicker,
    // and when it has not -- an aborted drag -- this is the rig going back.
    // A caller about to commit asks for no republish: the sessions are dirty
    // and the commit's notice evaluates them, once, from the stage.
    return hadRigOverrides && publish ? SetTime(time) : true;
}

bool
RigExecImagingRegistry::_NoticeSettlesPreview(
    const RigSession &session, const UsdNotice::ObjectsChanged &notice) const
{
    if (!_stage || _readRootsDirty || session.readRootsDirty ||
        !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    std::set<SdfPath> stashed;
    for (const RigExecValueOverride &entry : session.settleOverrides) {
        if (entry.attribute.IsEmpty()) {
            return false;
        }
        stashed.insert(entry.prim.AppendProperty(entry.attribute));
    }
    // First opinions create inert ancestor overs, reported without fields.
    // Metadata edits and prim resyncs still require evaluation.
    const auto inertAncestor = [&](const SdfPath &path) {
        return path.IsPrimPath() && notice.GetChangedFields(path).empty() &&
               std::any_of(stashed.begin(), stashed.end(),
                           [&](const SdfPath &property) {
                               return property.HasPrefix(path);
                           });
    };
    const auto valueEdit = [&](const SdfPath &path) {
        if (!stashed.count(path)) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != "default" && field != "timeSamples" &&
                field != "spline" && field != "typeName") {
                return false;
            }
        }
        return true;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!valueEdit(path)) {
            return false;
        }
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!inertAncestor(path) && !valueEdit(path)) {
            return false;
        }
    }
    // And the stage now answers every withdrawn value exactly, including
    // the ones this notice did not touch -- on attributes whose authored
    // value is what the rig reads. A connection overrules an authored
    // value, while the override stood in for the connection's answer. A
    // property chain revises an authored value exactly as it revised the
    // override, which was its base.
    if (!session.bridge) {
        return false;
    }
    for (const RigExecValueOverride &entry : session.settleOverrides) {
        const UsdAttribute attribute = _stage->GetAttributeAtPath(
            entry.prim.AppendProperty(entry.attribute));
        VtValue authored;
        if (!attribute || attribute.HasAuthoredConnections() ||
            !attribute.Get(&authored, _lastTime) || authored != entry.value) {
            return false;
        }
    }
    return true;
}

} // namespace rigExec
