#include "rigExecSchema/schemaNames.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/moverPacket.h"
#include "rigExecMath/geometryKernels.h"
#include "pxr/usd/usdGeom/mesh.h"

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

// Relationship names the binders and validators read, interned once:
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_relTokens,
    ((frame, rigExec::SchemaAttrText::rigExecFrame)));

// Interned once: callbacks and validators run on parallel lanes, and
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_deltaMushTokens,
    (deltaMush)
    (faceVertexCounts)
    (faceVertexIndices));

TF_DEFINE_PRIVATE_TOKENS(_mush,
    ((rest,rigExec::SchemaAttrText::inputsRestPoints)) ((iterations,rigExec::SchemaAttrText::inputsIterations))
    ((step,rigExec::SchemaAttrText::inputsStep)) ((pin,rigExec::SchemaAttrText::inputsPinBorders))
    ((distance,rigExec::SchemaAttrText::inputsDistanceWeight)) ((detail,rigExec::SchemaAttrText::inputsDisplacement))
    ((smoothing,rigExec::SchemaAttrText::inputsSmoothing)) ((transport,rigExec::SchemaAttrText::inputsFrameTransport))
    ((smoothWeights,rigExec::SchemaAttrText::inputsSmoothWeights)) ((edges,rigExec::SchemaAttrText::inputsEdges))
    ((onlySmooth,rigExec::SchemaAttrText::inputsOnlySmooth)) ((computationToTarget,rigExec::SchemaAttrText::inputsComputationToTarget))
    ((frame,rigExec::SchemaAttrText::rigExecFrame)) (frameTransforms));
namespace {
const TfToken _oracleToken0("faceVertexCounts");
const TfToken _oracleToken1("faceVertexIndices");
// The modes' fallbacks, built once: an oracle builds no token from text.
const TfToken _restToken("rest");
const TfToken _vertexToken("vertex");

void
Bind(const RigExecMoverBindContext &ctx)
{
    ctx.binding->base = ctx.target;
    ctx.binding->topologyCounts =
        ctx.ownerPath.AppendProperty(_deltaMushTokens->faceVertexCounts);
    ctx.binding->topologyIndices =
        ctx.ownerPath.AppendProperty(_deltaMushTokens->faceVertexIndices);
    ctx.binding->influences =
        RigExecRelationshipTargets(ctx.moverPrim, _relTokens->frame);
    ctx.binding->transformPhase =
        RigExecPhaseForInput(ctx.moverPrim, _relTokens->frame);
    if (ctx.binding->transformPhase.kind == RigExecReadPhaseKind::Final)
        for (auto &path : ctx.binding->influences) {
            const auto it = ctx.frameChainHeads.find(path);
            if (it != ctx.frameChainHeads.end())
                path = it->second;
        }
}
bool
Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    if (!UsdGeomMesh(
            ctx.stage->GetPrimAtPath(ctx.targets.front().GetPrimPath()))) {
        *error = "DeltaMush requires polygon mesh topology";
        return false;
    }
    auto attr = ctx.prim.GetAttribute(_mush->rest);
    if (attr.GetNumTimeSamples() != 0 || attr.HasAuthoredConnections()) {
        *error = "DeltaMush restPoints must be static";
        return false;
    }
    const auto frames = RigExecRelationshipTargets(ctx.prim, _relTokens->frame);
    if (frames.size() > 1) {
        *error = "DeltaMush frame takes at most one provider";
        return false;
    }
    if (RigExecPhaseForInput(ctx.prim, _relTokens->frame).kind ==
        RigExecReadPhaseKind::AtPrim) {
        *error = "DeltaMush frame read phase must be base or final";
        return false;
    }
    for (const auto &name :
         {_mush->smoothing, _mush->transport, _mush->edges}) {
        const auto a = ctx.prim.GetAttribute(name);
        if (a.GetNumTimeSamples() != 0 || a.HasAuthoredConnections()) {
            *error = "DeltaMush topology and mode inputs must be static";
            return false;
        }
    }
    VtVec3fArray rest;
    attr.Get(&rest);
    GfMatrix4d matrix(1);
    ctx.prim.GetAttribute(_mush->computationToTarget).Get(&matrix);
    bool only = false;
    ctx.prim.GetAttribute(_mush->onlySmooth).Get(&only);
    if (rest.empty() && (!frames.empty() || matrix != GfMatrix4d(1)) && !only) {
        *error = "DeltaMush computation transform requires explicit restPoints";
        return false;
    }
    return true;
}
RigExecOracleResult
Oracle(const RigExecMoverOracleContext &ctx)
{
    VtVec3fArray rest;
    ctx.prim.GetAttribute(_mush->rest).Get(&rest);

    VtIntArray counts, indices;
    auto owner = ctx.stage->GetPrimAtPath(ctx.target.GetPrimPath());
    owner.GetAttribute(_oracleToken0).Get(&counts, ctx.time);
    owner.GetAttribute(_oracleToken1).Get(&indices, ctx.time);
    int iterations = 10;
    float step = 0.5f, distance = 0, detail = 1;
    bool pin = true;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->iterations),
                              ctx.time, &iterations);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->step), ctx.time,
                              &step);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->pin), ctx.time,
                              &pin);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->distance), ctx.time,
                              &distance);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->detail), ctx.time,
                              &detail);
    RigExecDeltaMushSettings settings;
    TfToken smoothing = _restToken, transport = _vertexToken;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->smoothing),
                              UsdTimeCode::Default(), &smoothing);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->transport),
                              UsdTimeCode::Default(), &transport);
    if (!RigExecParseDeltaMushSmoothing(smoothing.GetText(),
                                        &settings.smoothing) ||
        !RigExecParseDeltaMushFrameTransport(transport.GetText(),
                                             &settings.frameTransport))
        return RigExecOracleResult::PassThrough;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->onlySmooth),
                              ctx.time, &settings.onlySmooth);
    VtFloatArray weights;
    VtIntArray edges;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->smoothWeights),
                              ctx.time, &weights);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->edges),
                              UsdTimeCode::Default(), &edges);
    settings.smoothWeights.assign(weights.begin(), weights.end());
    settings.edges.assign(edges.begin(), edges.end());
    GfMatrix4d computationToTarget(1);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_mush->computationToTarget),
                              ctx.time, &computationToTarget);
    const auto frames = RigExecRelationshipTargets(ctx.prim, _relTokens->frame);
    if (rest.empty() &&
        (!frames.empty() || computationToTarget != GfMatrix4d(1)) &&
        !settings.onlySmooth)
        return RigExecOracleResult::PassThrough;
    if (rest.empty())
        rest = ctx.basePoints;
    if (frames.size() > 1)
        return RigExecOracleResult::PassThrough;
    if (!frames.empty()) {
        const bool final =
            RigExecPhaseForInput(ctx.prim, _relTokens->frame).kind ==
            RigExecReadPhaseKind::Final;
        const auto &matrices =
            final ? ctx.finalProviderMatrices : ctx.baseProviderMatrices;
        const auto it = matrices.find(frames.front());
        if (it == matrices.end())
            return RigExecOracleResult::PassThrough;
        computationToTarget *= it->second;
    }
    std::vector<GfVec3f> points(ctx.points->begin(), ctx.points->end());
    if (!RigExecApplyDeltaMush(
            &points, {rest.begin(), rest.end()}, {counts.begin(), counts.end()},
            {indices.begin(), indices.end()}, iterations, step, pin, distance,
            detail, settings, computationToTarget)) {
        ctx.diagnostics->push_back("MoverFailed " + ctx.moverPath.GetString() +
                                   ": invalid delta mush inputs");
        return RigExecOracleResult::PassThrough;
    }
    std::copy(points.begin(), points.end(), ctx.points->begin());
    return RigExecOracleResult::Blend;
}
RigExecMoverHandler
Handler()
{
    RigExecMoverHandler h(rigExec::SchemaTypeName::RigExecDeltaMushMover,
                          &RigExecFixedMoverOp<RigExecRevisionOp::DeltaMush>,
                          RigExecMoverDomain::Points);
    h.frameRelationships = {rigExec::SchemaAttrText::rigExecFrame};
    h.transformRelationship = rigExec::SchemaAttrText::rigExecFrame;
    h.singleTarget = true;
    h.bind = &Bind;
    h.validate = &Validate;
    h.oracle = &Oracle;
    return h;
}
} // namespace
RIGEXEC_REGISTER_MOVER(Handler());
