// RigExec imaging C activation surface: the host-facing extern "C" exports
// over the per-stage contexts.
#include "registry.h"
#include "guideBoundsInternal.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using rigExec::RigExecImagingRegistry;

namespace {

using RegistryPtr = RigExecImagingRegistry::Ptr;

// A stage-scoped call's context: the one of the stage the stage cache holds
// under \p id. Never created here except where the call documents it.
RegistryPtr
_ForStage(long long id, bool create = false)
{
    return RigExecImagingRegistry::ForStageCacheId(id, create);
}

long long
_Generation(const RigExecImagingRegistry &registry)
{
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        registry.GetStore()->Get();
    return snapshot ? static_cast<long long>(snapshot->generation) : 0;
}

// Writes \p text into a caller-supplied fixed buffer, truncating to fit and
// always NUL-terminating; does nothing when no buffer was given.
void
_CopyReason(const std::string &text, char *out, int capacity)
{
    if (!out || capacity <= 0) {
        return;
    }
    const size_t keep = text.size() < static_cast<size_t>(capacity - 1)
                            ? text.size()
                            : static_cast<size_t>(capacity - 1);
    if (keep > 0) {
        std::memcpy(out, text.data(), keep);
    }
    out[keep] = '\0';
}

// Reads the current generation's published scalar mover outputs
// (RigExecImagingSnapshot::movedFloats) for caller-supplied property
// paths, newline-packed; an empty, malformed, or unpublished path leaves
// its slot zero. With no generation published nothing is read and the
// return is 0; otherwise the return counts the slots filled.
int
_MovedFloats(const RigExecImagingRegistry &registry, const char *packedPaths,
             float *out, int count)
{
    if (!packedPaths || !out || count <= 0) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        registry.GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    // Every slot is cleared first: a caller that reads past the return
    // value gets zeros rather than whatever was in its buffer.
    for (int i = 0; i < count; ++i) {
        out[i] = 0.0f;
    }
    int found = 0;
    int index = 0;
    const char *cursor = packedPaths;
    while (*cursor && index < count) {
        const char *end = std::strchr(cursor, '\n');
        const std::string text(cursor,
                               end ? end - cursor : std::strlen(cursor));
        // An arbitrary caller-supplied string reaches SdfPath here, whose
        // constructor is loud about a malformed one. Ask first.
        if (!text.empty() && PXR_NS::SdfPath::IsValidPathString(text)) {
            const auto it = snapshot->movedFloats.find(PXR_NS::SdfPath(text));
            if (it != snapshot->movedFloats.end()) {
                out[index] = it->second;
                ++found;
            }
        }
        ++index;
        if (!end) {
            break;
        }
        cursor = end + 1;
    }
    return found;
}

int
_GuideBounds(const RigExecImagingRegistry &registry, const char *primPath,
             double outMinMax[6])
{
    if (!primPath || !primPath[0] || !outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        registry.GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    // An arbitrary caller-supplied string reaches SdfPath here, and its
    // constructor is loud about a malformed one. Ask first.
    if (!PXR_NS::SdfPath::IsValidPathString(primPath)) {
        return 0;
    }
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    if (it == snapshot->prims.end()) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    if (!rigExec::_AccumulateGuideBounds(it->second, &range) ||
        range.IsEmpty()) {
        return 0;
    }
    rigExec::_WriteBounds(range, outMinMax);
    return 1;
}

int
_AllGuideBounds(const RigExecImagingRegistry &registry, double outMinMax[6])
{
    if (!outMinMax) {
        return 0;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        registry.GetStore()->Get();
    if (!snapshot) {
        return 0;
    }
    PXR_NS::GfRange3d range;
    bool any = false;
    PXR_NS::SdfPath commonAssetRoot;
    for (const auto &[path, published] : snapshot->prims) {
        PXR_NS::GfRange3d publishedRange;
        if (!rigExec::_AccumulateGuideBounds(published, &publishedRange)) {
            continue;
        }

        const PXR_NS::SdfPath &assetRoot = published.assetRoot.IsEmpty()
                                               ? snapshot->assetRoot
                                               : published.assetRoot;
        if (!any) {
            commonAssetRoot = assetRoot;
        } else if (commonAssetRoot != assetRoot) {
            // Asset-space bounds from independently placed assets cannot be
            // combined meaningfully.  Callers must query each prim and apply
            // that prim's asset-root transform instead.
            return 0;
        }

        range.UnionWith(publishedRange);
        any = true;
    }
    if (!any || range.IsEmpty()) {
        return 0;
    }
    rigExec::_WriteBounds(range, outMinMax);
    return 1;
}

int
_FrameStates(RigExecImagingRegistry &registry, const char *rigPath,
             const double *frames, int *statesOut, int count)
{
    if (!rigPath || !frames || !statesOut || count < 0) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    std::vector<double> times;
    times.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(frames[i])) {
            return -1;
        }
        times.push_back(frames[i]);
    }
    const std::vector<rigExec::RigExecWarmFrameState> states =
        registry.GetFrameStates(SdfPath(rigPath), times);
    if (states.empty() && count > 0) {
        return -1;
    }
    for (size_t i = 0; i < states.size(); ++i) {
        statesOut[i] = int(states[i]);
    }
    return int(states.size());
}

int
_ClearFrameCache(RigExecImagingRegistry &registry, const char *rigPath)
{
    if (!rigPath) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    registry.ClearFrameCache(SdfPath(rigPath));
    return 0;
}

int
_WarmRange(RigExecImagingRegistry &registry, const char *rigPath,
           const double *frames, int count)
{
    if (!rigPath || count < 0 || (!frames && count > 0)) {
        return -1;
    }
    if (!SdfPath::IsValidPathString(rigPath)) {
        return -1;
    }
    std::vector<double> times;
    times.reserve(size_t(count));
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(frames[i])) {
            return -1;
        }
        times.push_back(frames[i]);
    }
    return registry.SetWarmRange(SdfPath(rigPath), times) ? 0 : -1;
}

int
_UpdatePreview(RigExecImagingRegistry &registry, const double *values,
               int count)
{
    if (count < 0) {
        return 1;
    }
    return registry.UpdatePreview(values, size_t(count)) ? 0 : 1;
}

// RigExecImaging_ActivateForStage: activation of THAT stage's context;
// each failure prints its reason.
int
_ActivateC(long long stageCacheId, const char *rigPath, double initialFrame)
{
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    if (!stage) {
        std::printf("rigExecImaging: no stage for cache id %lld\n",
                    stageCacheId);
        return 1;
    }

    PXR_NS::SdfPath path;
    if (rigPath && rigPath[0]) {
        if (!PXR_NS::SdfPath::IsValidPathString(rigPath)) {
            std::printf("rigExecImaging: invalid rig path '%s'\n", rigPath);
            return 2;
        }
        path = PXR_NS::SdfPath(rigPath);
    }

    // THAT stage's context (created on demand); other stages' contexts are
    // untouched.
    std::vector<std::string> errors;
    const RegistryPtr context =
        RigExecImagingRegistry::ForStage(stage, /* create = */ true);
    const bool activated =
        context && context->Activate(
                       stage, path, PXR_NS::UsdTimeCode(initialFrame), &errors);
    if (!context) {
        errors.push_back("no imaging context for the stage");
    }
    if (!activated) {
        for (const std::string &e : errors) {
            std::printf("rigExecImaging: %s\n", e.c_str());
        }
        return 3;
    }
    if (path.IsEmpty()) {
        std::printf("rigExecImaging: activated all RigExecRoot prims\n");
    } else {
        std::printf("rigExecImaging: activated %s\n", path.GetText());
    }
    return 0;
}

} // namespace

extern "C" {

int
RigExecImaging_ActivateForStage(long long stageCacheId, const char *rigPath,
                                double initialFrame)
{
    return _ActivateC(stageCacheId, rigPath, initialFrame);
}

int
RigExecImaging_GetControlFrameAssetSpace(long long stageCacheId,
                                         const char *primPath, double frame,
                                         int isDefault, double outMatrix[16])
{
    if (!primPath || !outMatrix ||
        !PXR_NS::SdfPath::IsValidPathString(primPath) ||
        (!isDefault && !std::isfinite(frame)))
        return 0;
    const auto stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    if (!stage)
        return 0;
    // THAT stage's own context: a second stage with the same paths never
    // answers for this one.
    const RegistryPtr context =
        RigExecImagingRegistry::ForStage(stage, /* create = */ false);
    const auto snapshot = context ? context->GetStore()->Get() : nullptr;
    const auto time =
        isDefault ? PXR_NS::UsdTimeCode::Default() : PXR_NS::UsdTimeCode(frame);
    if (!snapshot || !snapshot->Describes(stage, time))
        return 0;
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    if (it == snapshot->prims.end())
        return 0;
    // Provider frames first; volume weights carry their guide placement
    // instead (live-only, like the guide elements themselves).
    const PXR_NS::GfMatrix4d *found = nullptr;
    if (it->second.hasControlFrame)
        found = &it->second.controlFrame;
    else if (it->second.hasVolumeGuideFrame)
        found = &it->second.volumeGuideFrame;
    if (!found)
        return 0;
    // GfMatrix4d stores contiguous row-major components (gf/matrix4d.h),
    // exactly the outMatrix contract, so the whole matrix copies at once.
    std::memcpy(outMatrix, found->data(), 16 * sizeof(double));
    return 1;
}

// Stage-scoped surface: each call acts on its stage's context only.

int
RigExecImaging_SetTimeForStage(long long stageCacheId, double frame)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context && context->SetTime(PXR_NS::UsdTimeCode(frame)) ? 0 : 1;
}

int
RigExecImaging_OnEditCommittedForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !context->IsActive()) {
        return 1;
    }
    context->OnEditCommitted();
    return 0;
}

int
RigExecImaging_OnIdleForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !context->IsActive()) {
        return 1;
    }
    context->OnIdle();
    return 0;
}

int
RigExecImaging_ServiceFrameDemandForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !context->IsActive()) {
        return -1;
    }
    return int(context->ServiceFrameDemand());
}

int
RigExecImaging_HasFrameDemandForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !context->IsActive()) {
        return -1;
    }
    return context->HasFrameDemand() ? 1 : 0;
}

int
RigExecImaging_WriteProfileSummaryForStage(long long stageCacheId,
                                           const char *path)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        // Exactly what an inactive registry writes: the header, no rows.
        std::ofstream out(path ? std::string(path) : std::string());
        if (!out) {
            return 1;
        }
        out << "rig\tname\tcategory\tcount\ttotal_ms\tmax_ms\n";
        return 0;
    }
    return context->WriteProfileSummary(path ? std::string(path)
                                             : std::string())
               ? 0
               : 1;
}

void
RigExecImaging_DeactivateForStage(long long stageCacheId)
{
    if (const RegistryPtr context = _ForStage(stageCacheId)) {
        context->Deactivate();
    }
}

long long
RigExecImaging_GetGenerationForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context ? _Generation(*context) : 0;
}

namespace {

// The displayed generation's record of `primPath`, or null.
const rigExec::RigExecPublishedPrim *
_PublishedPrim(const rigExec::RigExecImagingSnapshotConstPtr &snapshot,
               const char *primPath)
{
    if (!snapshot || !primPath ||
        !PXR_NS::SdfPath::IsValidPathString(primPath)) {
        return nullptr;
    }
    const auto it = snapshot->prims.find(PXR_NS::SdfPath(primPath));
    return it == snapshot->prims.end() ? nullptr : &it->second;
}

} // namespace

int
RigExecImaging_GetPublishedPointsForStage(long long stageCacheId,
                                          const char *primPath, float *xyz,
                                          int capacity)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return -2;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        context->GetStore()->Get();
    const rigExec::RigExecPublishedPrim *prim =
        _PublishedPrim(snapshot, primPath);
    if (!prim || !prim->hasPoints) {
        return -1;
    }
    const int count = int(prim->points.size());
    if (xyz && capacity >= count && count > 0) {
        std::memcpy(xyz, prim->points.cdata(),
                    sizeof(float) * 3 * prim->points.size());
    }
    return count;
}

int
RigExecImaging_GetPublishedScalarPrimvarForStage(long long stageCacheId,
                                                 const char *primPath,
                                                 const char *primvarName,
                                                 float *out)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return -2;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        context->GetStore()->Get();
    const rigExec::RigExecPublishedPrim *prim =
        _PublishedPrim(snapshot, primPath);
    if (!prim || !primvarName) {
        return 0;
    }
    const auto it = prim->shaderFloats.find(PXR_NS::TfToken(primvarName));
    if (it == prim->shaderFloats.end()) {
        return 0;
    }
    if (out) {
        *out = it->second;
    }
    return 1;
}

int
RigExecImaging_GetPublishedXformRevisionForStage(long long stageCacheId,
                                                 const char *primPath,
                                                 double *out16)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return -2;
    }
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        context->GetStore()->Get();
    const rigExec::RigExecPublishedPrim *prim =
        _PublishedPrim(snapshot, primPath);
    if (!prim || !prim->hasXform) {
        return 0;
    }
    PXR_NS::GfMatrix4d baseInverse;
    if (!rigExec::RigExecTryInvertXform(prim->xformBase, &baseInverse)) {
        return 0;
    }
    const PXR_NS::GfMatrix4d revision = prim->xform * baseInverse;
    if (out16) {
        std::memcpy(out16, revision.data(), sizeof(double) * 16);
    }
    return 1;
}

int
RigExecImaging_GetMovedFloatsForStage(long long stageCacheId,
                                      const char *packedPaths, float *out,
                                      int count)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return 0;
    }
    return _MovedFloats(*context, packedPaths, out, count);
}

int
RigExecImaging_GetMovedDoublesForStage(long long stageCacheId,
                                       const char *propertyPath, double frame,
                                       int isDefault, double *out, int capacity)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !propertyPath ||
        !PXR_NS::SdfPath::IsValidPathString(propertyPath) ||
        (!isDefault && !std::isfinite(frame))) {
        return -1;
    }
    const auto stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(
            static_cast<long int>(stageCacheId)));
    const rigExec::RigExecImagingSnapshotConstPtr snapshot =
        context->GetStore()->Get();
    // Only a generation of the stage as it stands, at this time: values
    // from an earlier one would match a pose the stage no longer holds.
    const auto time =
        isDefault ? PXR_NS::UsdTimeCode::Default() : PXR_NS::UsdTimeCode(frame);
    if (!stage || !snapshot || !snapshot->Describes(stage, time)) {
        return -1;
    }
    const auto it = snapshot->movedArrays.find(PXR_NS::SdfPath(propertyPath));
    if (it == snapshot->movedArrays.end()) {
        return -1;
    }
    const int size = int(it->second.size());
    for (int i = 0; out && i < size && i < capacity; ++i) {
        out[i] = it->second[size_t(i)];
    }
    return size;
}

int
RigExecImaging_GetGuideBoundsAssetSpaceForStage(long long stageCacheId,
                                                const char *primPath,
                                                double outMinMax[6])
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context ? _GuideBounds(*context, primPath, outMinMax) : 0;
}

int
RigExecImaging_GetAllGuideBoundsAssetSpaceForStage(long long stageCacheId,
                                                   double outMinMax[6])
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context ? _AllGuideBounds(*context, outMinMax) : 0;
}

int
RigExecImaging_SetWeightOverlayForStage(long long stageCacheId,
                                        const char *weightPrimPath)
{
    // Created on demand: a selection made before the stage's rig activates
    // is remembered for its first generation.
    const RegistryPtr context = _ForStage(stageCacheId, /* create = */ true);
    if (!context) {
        return 1;
    }
    return context->SetWeightOverlay(
               weightPrimPath ? std::string(weightPrimPath) : std::string())
               ? 0
               : 1;
}

int
RigExecImaging_GetFrameStatesForStage(long long stageCacheId,
                                      const char *rigPath, const double *frames,
                                      int *statesOut, int count)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return -1;
    }
    return _FrameStates(*context, rigPath, frames, statesOut, count);
}

int
RigExecImaging_ClearFrameCacheForStage(long long stageCacheId,
                                       const char *rigPath)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        // Nothing to drop is still dropped; a malformed path is
        // still refused.
        return rigPath && SdfPath::IsValidPathString(rigPath) ? 0 : -1;
    }
    return _ClearFrameCache(*context, rigPath);
}

int
RigExecImaging_WarmRangeForStage(long long stageCacheId, const char *rigPath,
                                 const double *frames, int count)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return -1;
    }
    return _WarmRange(*context, rigPath, frames, count);
}

long long
RigExecImaging_GetWarmingCompletedCountForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context
               ? static_cast<long long>(context->GetBackgroundStats().completed)
               : 0;
}

long long
RigExecImaging_GetWarmingProgressCountForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context ? static_cast<long long>(context->GetWarmingProgressCount())
                   : 0;
}

int
RigExecImaging_BeginPreviewForStage(long long stageCacheId,
                                    const char *packedAttributePaths)
{
    // Created on demand: a stage with no rig -- so possibly no context yet
    // -- still previews its Xforms, through its own chains.
    const RegistryPtr context = _ForStage(stageCacheId, /* create = */ true);
    if (!context) {
        return -1;
    }
    return context->BeginPreview(packedAttributePaths
                                     ? std::string(packedAttributePaths)
                                     : std::string());
}

int
RigExecImaging_UpdatePreviewForStage(long long stageCacheId,
                                     const double *values, int count)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context ? _UpdatePreview(*context, values, count) : 1;
}

int
RigExecImaging_EndPreviewForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    // Ending a preview that never began is what an inactive registry does:
    // nothing to drop, success.
    return !context || context->EndPreview() ? 0 : 1;
}

int
RigExecImaging_EndPreviewWithoutPublishForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return !context || context->EndPreview(/* publish = */ false) ? 0 : 1;
}

int
RigExecImaging_GetPreviewIdentityForStage(long long stageCacheId,
                                          unsigned long long *session,
                                          unsigned long long *revision,
                                          int *active)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !session || !revision || !active) {
        return 1;
    }
    const rigExec::RigExecPreviewIdentity identity =
        context->GetPreviewIdentity();
    *session = identity.session;
    *revision = identity.revision;
    *active = identity.active ? 1 : 0;
    return 0;
}

int
RigExecImaging_DeclarePreviewEditForStage(long long stageCacheId,
                                          const char *packedAttributePaths,
                                          int policy, const double *values,
                                          int count)
{
    const RegistryPtr context = _ForStage(stageCacheId, /* create = */ true);
    if (!context) {
        return 1;
    }
    rigExec::RigExecPreviewEditPolicy editPolicy;
    switch (policy) {
    case 1:
        editPolicy = rigExec::RigExecPreviewEditPolicy::AttributeTimeSample;
        break;
    case 2:
        editPolicy = rigExec::RigExecPreviewEditPolicy::Connection;
        break;
    default:
        editPolicy = rigExec::RigExecPreviewEditPolicy::AttributeDefault;
        break;
    }
    return context->DeclarePreviewEdit(
               packedAttributePaths ? std::string(packedAttributePaths)
                                    : std::string(),
               editPolicy, values,
               count > 0 ? static_cast<size_t>(count) : size_t(0))
               ? 0
               : 1;
}

int
RigExecImaging_GetPreviewEditStatsForStage(long long stageCacheId,
                                           unsigned long long *session,
                                           unsigned long long *revision,
                                           int *editCount, int *valueCount,
                                           int *policy)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !session || !revision || !editCount || !valueCount ||
        !policy) {
        return 1;
    }
    const rigExec::RigExecPreviewEditStats stats =
        context->GetPreviewEditStats();
    *session = stats.identity.session;
    *revision = stats.identity.revision;
    *editCount = static_cast<int>(stats.editCount);
    *valueCount = static_cast<int>(stats.valueCount);
    *policy = static_cast<int>(stats.policy);
    return 0;
}

int
RigExecImaging_PreviewValueIsIndependentForStage(long long stageCacheId,
                                                 const char *attributePath,
                                                 double time, char *reasonOut,
                                                 int reasonCapacity)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context || !attributePath) {
        return 0;
    }
    std::string reason;
    const bool proven = context->PreviewValueIsIndependent(
        attributePath, UsdTimeCode(time), &reason);
    _CopyReason(reason, reasonOut, reasonCapacity);
    return proven ? 1 : 0;
}

int
RigExecImaging_AdmitWarmRouteForStage(long long stageCacheId, int route,
                                      const char *rigPath, double time,
                                      unsigned long long previewSession,
                                      char *reasonOut, int reasonCapacity)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    if (!context) {
        return 0;
    }
    rigExec::RigExecWarmRouteDescriptor descriptor;
    switch (route) {
    case 1:
        descriptor.route = rigExec::RigExecWarmRoute::CommittedSpeculative;
        break;
    case 2:
        descriptor.route = rigExec::RigExecWarmRoute::PendingPreview;
        break;
    default:
        descriptor.route = rigExec::RigExecWarmRoute::Committed;
        break;
    }
    if (rigPath && SdfPath::IsValidPathString(rigPath)) {
        descriptor.rigPath = SdfPath(rigPath);
    }
    descriptor.time = UsdTimeCode(time);
    descriptor.previewSession = previewSession;
    std::string reason;
    const bool admitted = context->AdmitWarmRoute(descriptor, &reason);
    _CopyReason(reason, reasonOut, reasonCapacity);
    return admitted ? 1 : 0;
}

int
RigExecImaging_IsActiveForStage(long long stageCacheId)
{
    const RegistryPtr context = _ForStage(stageCacheId);
    return context && context->IsActive() ? 1 : 0;
}

int
RigExecImaging_ContextCount()
{
    return static_cast<int>(RigExecImagingRegistry::ContextCount());
}

} // extern "C"
