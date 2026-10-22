// RigExecSurfaceMover: everything about the surface mover.
// A surface mover projects points onto a driver surface. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// rules.
#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/usd/usd/attribute.h"

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;

PXR_NAMESPACE_USING_DIRECTIVE

// Relationship names the binders and validators read, interned once:
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_relTokens,
    ((surface, "rigExec:surface"))
    ((frames, "rigExec:frames")));

// Interned once: callbacks and validators run on parallel lanes, and
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_surfaceTokens,
    (surfaceProject)
    (surfaceFrames)
    ((mask, "rigExec:mask"))
    ((triangles, "rigExec:triangles"))
    (onSurface)
    (local)
    (points)
    (faceVertexCounts)
    (faceVertexIndices)
    ((snapMode, "rigExec:snapMode"))
    ((pointSpace, "rigExec:pointSpace"))
    ((offset, "rigExec:offset"))
    ((surfaceMatrix, "rigExec:surfaceMatrix"))
    ((targetMatrix, "rigExec:targetMatrix"))
    ((frames, "rigExec:frames")));

namespace {
const TfToken _oracleToken0("rigExec:surface");
const TfToken _oracleToken1("points");
const TfToken _oracleToken2("faceVertexCounts");
const TfToken _oracleToken3("faceVertexIndices");
// The settings' fallbacks, built once: an oracle builds no token from text.
const TfToken _onSurfaceToken("onSurface");
const TfToken _insideToken("inside");
const TfToken _outsideToken("outside");
const TfToken _outsideSurfaceToken("outsideSurface");
const TfToken _localToken("local");
const TfToken _commonToken("common");


bool SetSettings(RigExecMoverParameters *p, const TfToken &snap, const TfToken &space,
                 float offset, const std::vector<float> &mask, const std::vector<int> &triangles,
                 const GfMatrix4d &surface, const GfMatrix4d &target)
{
    using namespace rigExec;
    if(snap==_onSurfaceToken)p->surfaceSettings.mode=RigExecSurfaceSnapMode::OnSurface;
    else if(snap==_insideToken)p->surfaceSettings.mode=RigExecSurfaceSnapMode::Inside;
    else if(snap==_outsideToken)p->surfaceSettings.mode=RigExecSurfaceSnapMode::Outside;
    else if(snap==_outsideSurfaceToken)p->surfaceSettings.mode=RigExecSurfaceSnapMode::OutsideSurface;
    else return false;
    p->surfaceSettings.offset=offset;p->surfaceSettings.mask=mask;p->surfaceSettings.triangles=triangles;
    if(!std::isfinite(offset) || !RigExecSurfaceSnapValidMatrix(surface) || !RigExecSurfaceSnapValidMatrix(target))return false;
    if(space==_localToken) {
        p->targetToSurface=target*rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToTarget=surface*rigExec::RigExecSurfaceSnapAffineInverse(target);
    } else if(space==_commonToken) {
        p->targetToSurface=rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToMetric=rigExec::RigExecSurfaceSnapAffineInverse(surface);p->surfaceToTarget=surface;
    } else return false;
    return true;
}

RigExecMoverParameters
_BuildSurfaceMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = _surfaceTokens->surfaceProject;
    const bool *enabled =
        ctx.GetInputValuePtr<bool>(RigExecMoverExecTokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    if (!rigExec::RigExecMoverSetCommonEnvelope(ctx, &params)) {
        return params;
    }
    params.strength = 1.0f;  // v0.1 attach/project maps fully
    params.auxPoints = rigExec::RigExecMoverCollect<GfVec3f>(
        ctx, RigExecMoverExecTokens->surfacePoints);
    params.topologyCounts = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyCounts);
    params.topologyIndices = rigExec::RigExecMoverCollect<int>(
        ctx, RigExecMoverExecTokens->topologyIndices);
    const auto read=[&](const TfToken &name,auto fallback) {
        using T=decltype(fallback);const T *value=ctx.GetInputValuePtr<T>(name);return value?*value:fallback;
    };
    auto surface=read(_surfaceTokens->surfaceMatrix,GfMatrix4d(1.0)),target=read(_surfaceTokens->targetMatrix,GfMatrix4d(1.0));
    if(!rigExec::RigExecSurfaceSnapValidMatrix(surface) || !rigExec::RigExecSurfaceSnapValidMatrix(target))return params;
    const auto frames=rigExec::RigExecMoverCollect<GfMatrix4d>(ctx,_surfaceTokens->surfaceFrames);
    if(!frames.empty()) {
        if(frames.size()!=2)return params;surface*=frames[0];target*=frames[1];
        if(!rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&surface) || !rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&target))return params;
    }
    const auto mask=rigExec::RigExecMoverCollect<float>(ctx,_surfaceTokens->mask);
    const auto triangles=rigExec::RigExecMoverCollect<int>(ctx,_surfaceTokens->triangles);
    params.valid = !params.auxPoints.empty() &&
                   (!params.topologyCounts.empty() || !triangles.empty()) &&
        SetSettings(&params,read(_surfaceTokens->snapMode,_surfaceTokens->onSurface),read(_surfaceTokens->pointSpace,_surfaceTokens->local),
                    read(_surfaceTokens->offset,0.f),mask,triangles,surface,target);
    return params;
}

void
_BindSurfaceMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    binding.base = ctx.target;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const SdfPathVector surfaces = rigExec::RigExecRelationshipTargets(
        moverPrim, _relTokens->surface);
    if (!surfaces.empty()) {
        const SdfPath surfacePrim = surfaces[0].GetPrimPath();
        binding.surfacePoints =
            surfacePrim.AppendProperty(_surfaceTokens->points);
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(moverPrim, _relTokens->surface);
        if (!phase.IsBase()) {
            binding.phases[binding.surfacePoints] = phase;
        }
        binding.topologyCounts =
            surfacePrim.AppendProperty(_surfaceTokens->faceVertexCounts);
        binding.topologyIndices =
            surfacePrim.AppendProperty(_surfaceTokens->faceVertexIndices);
    }
    binding.influences=rigExec::RigExecRelationshipTargets(moverPrim,_relTokens->frames);
    binding.transformPhase=rigExec::RigExecPhaseForInput(moverPrim,_relTokens->frames);
    if(binding.transformPhase.kind==rigExec::RigExecReadPhaseKind::Final)
        for(auto &provider:binding.influences) {
            const auto head=ctx.frameChainHeads.find(provider);if(head!=ctx.frameChainHeads.end())provider=head->second;
        }
}

rigExec::RigExecOracleResult
_OracleSurfaceMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const rigExec::RigExecOracleScene &stage = ctx.stage;
    const rigExec::RigExecOraclePrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const UsdTimeCode time = ctx.time;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    SdfPathVector surfaces;
    if (rigExec::RigExecOracleRelationship rel =
            prim.GetRelationship(_oracleToken0)) {
        rel.GetTargets(&surfaces);
    }
    if (surfaces.empty()) {
        return RigExecOracleResult::PassThrough;
    }
    const SdfPath surfacePrim = surfaces[0].GetPrimPath();
    VtVec3fArray surfacePoints;
    VtIntArray counts, indices;
    rigExec::RigExecReadPhasedPoints(
        ctx, "rigExec:surface",
        surfacePrim.AppendProperty(_oracleToken1), &surfacePoints);
    if (const rigExec::RigExecOraclePrim s = stage->GetPrimAtPath(surfacePrim)) {
        s.GetAttribute(_oracleToken2).Get(&counts, time);
        s.GetAttribute(_oracleToken3)
            .Get(&indices, time);
    }
    VtIntArray explicitTriangles;
    prim.GetAttribute(_surfaceTokens->triangles).Get(&explicitTriangles,time);
    if (surfacePoints.empty() || (counts.empty() && explicitTriangles.empty())) {
        diagnostics->push_back(
            "MoverFailed " + moverPath.GetString() +
            ": surface has no points/topology");
        return RigExecOracleResult::PassThrough;
    }
    std::vector<GfVec3f> scratch(points.begin(), points.end());
    const auto read=[&](const TfToken &name,auto fallback) {
        const auto a=prim.GetAttribute(name);if(a)ctx.resolved.GetAttribute(a,time,&fallback);return fallback;
    };
    auto surface=read(_surfaceTokens->surfaceMatrix,GfMatrix4d(1.0)),target=read(_surfaceTokens->targetMatrix,GfMatrix4d(1.0));
    if(!rigExec::RigExecSurfaceSnapValidMatrix(surface) || !rigExec::RigExecSurfaceSnapValidMatrix(target))return RigExecOracleResult::PassThrough;
    const auto frames=rigExec::RigExecRelationshipTargets(prim,_relTokens->frames);
    if(!frames.empty()) {
        if(frames.size()!=2)return RigExecOracleResult::PassThrough;
        const auto &table=rigExec::RigExecPhaseForInput(prim,_relTokens->frames).kind==rigExec::RigExecReadPhaseKind::Final?
            ctx.finalProviderMatrices:ctx.baseProviderMatrices;
        const auto s=table.find(frames[0]),t=table.find(frames[1]);
        if(s==table.end() || t==table.end())return RigExecOracleResult::PassThrough;
        surface*=s->second;target*=t->second;
        if(!rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&surface) || !rigExec::RigExecSurfaceSnapCanonicalComputedMatrix(&target))return RigExecOracleResult::PassThrough;
    }
    const auto mask=read(_surfaceTokens->mask,VtFloatArray());const auto triangles=read(_surfaceTokens->triangles,VtIntArray());
    RigExecMoverParameters params;
    if(!SetSettings(&params,read(_surfaceTokens->snapMode,_onSurfaceToken),read(_surfaceTokens->pointSpace,_localToken),
                    read(_surfaceTokens->offset,0.f),{mask.begin(),mask.end()},{triangles.begin(),triangles.end()},surface,target))
        return RigExecOracleResult::PassThrough;
    if(rigExec::RigExecSurfaceSnapIsLegacy(params.surfaceSettings,params.targetToSurface,params.surfaceToTarget,params.surfaceToMetric)) {
        rigExec::RigExecApplySurfaceProject(&scratch,{surfacePoints.begin(),surfacePoints.end()},
            {counts.begin(),counts.end()},{indices.begin(),indices.end()},1.0);
    } else if(!rigExec::RigExecApplySurfaceSnapKernel<GfVec3f,GfVec3d>(
        &scratch,
        std::vector<GfVec3f>(surfacePoints.begin(),
                             surfacePoints.end()),
        std::vector<int>(counts.begin(), counts.end()),
        std::vector<int>(indices.begin(), indices.end()),params.surfaceSettings,
        params.targetToSurface,params.surfaceToTarget,params.surfaceToMetric))return RigExecOracleResult::PassThrough;
    std::copy(scratch.begin(), scratch.end(), points.begin());
    return RigExecOracleResult::Blend;
}

bool ValidateSurface(const rigExec::RigExecMoverValidateContext &ctx,std::string *error)
{
    const auto &prim=ctx.prim;
    const auto frames=rigExec::RigExecRelationshipTargets(prim,_relTokens->frames);
    const auto surfaces=rigExec::RigExecRelationshipTargets(prim,_relTokens->surface);
    const auto fail=[&]() {*error="invalid surface snap mode, topology, mask or coordinate frame";return false;};
    if(surfaces.size()!=1 || (!frames.empty() && frames.size()!=2))return fail();
    if(!frames.empty() && rigExec::RigExecPhaseForInput(prim,_relTokens->frames).kind==rigExec::RigExecReadPhaseKind::AtPrim) {
        *error="surface frame providers require base or final read phase";return false;
    }
    const auto read=[&](const TfToken &name,auto fallback) {
        const auto a=prim.GetAttribute(name);if(a)a.Get(&fallback);return fallback;
    };
    const auto mask=read(_surfaceTokens->mask,VtFloatArray());const auto triangles=read(_surfaceTokens->triangles,VtIntArray());
    RigExecMoverParameters params;
    if(!SetSettings(&params,read(_surfaceTokens->snapMode,_surfaceTokens->onSurface),read(_surfaceTokens->pointSpace,_surfaceTokens->local),
                    read(_surfaceTokens->offset,0.f),{mask.begin(),mask.end()},{triangles.begin(),triangles.end()},
                    read(_surfaceTokens->surfaceMatrix,GfMatrix4d(1.0)),read(_surfaceTokens->targetMatrix,GfMatrix4d(1.0))))return fail();
    const auto surface=ctx.stage->GetPrimAtPath(surfaces[0].GetPrimPath());
    if(!surface || !surface.GetAttribute(_surfaceTokens->points))return fail();
    for(float w:mask)if(!std::isfinite(w) || w<0 || w>1)return fail();
    if(triangles.size()%3)return fail();
    for(int i:triangles)if(i<0)return fail();
    // Shape-dependent checks belong to the evaluated frame. Default points
    // may be absent or degenerate in an otherwise valid animated surface.
    for(const auto &target:ctx.targets) {
        VtVec3fArray points;
        if(!mask.empty() && ctx.stage->GetAttributeAtPath(target).Get(&points) && mask.size()!=points.size())return fail();
    }
    return true;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecSurfaceMover",
        &rigExec::RigExecFixedMoverOp<
            rigExec::RigExecRevisionOp::SurfaceProject>,
        rigExec::RigExecMoverDomain::Points);
    handler.bind = &_BindSurfaceMover;
    handler.oracle = &_OracleSurfaceMover;
    handler.validate = &ValidateSurface;
    handler.frameRelationships={"rigExec:frames"};handler.transformRelationship="rigExec:frames";
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSurfaceMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildSurfaceMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<TfToken>(_surfaceTokens->snapMode),
            AttributeValue<TfToken>(_surfaceTokens->pointSpace),
            AttributeValue<float>(_surfaceTokens->offset),
            AttributeValue<float>(_surfaceTokens->mask),
            AttributeValue<int>(_surfaceTokens->triangles),
            AttributeValue<GfMatrix4d>(_surfaceTokens->surfaceMatrix),
            AttributeValue<GfMatrix4d>(_surfaceTokens->targetMatrix),
            Relationship(_surfaceTokens->frames).TargetedObjects<GfMatrix4d>(RigExecMoverExecTokens->computeMatrix)
                .InputName(_surfaceTokens->surfaceFrames),
            Relationship(RigExecMoverExecTokens->resolvedSurfacePoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->surfacePoints),
            Relationship(RigExecMoverExecTokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyCounts),
            Relationship(RigExecMoverExecTokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyIndices));

    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(
            &rigExec::RigExecMoverBuildStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                RigExecMoverExecTokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(RigExecMoverExecTokens->moverPath));
}

RIGEXEC_REGISTER_MOVER(_MakeHandler());
