#include "rigExecSchema/schemaNames.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/moverPacket.h"
#include "rigExecMath/geometryKernels.h"
#include "pxr/usd/usdGeom/mesh.h"

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

// Interned once: callbacks and validators run on parallel lanes, and
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_wrinkleTokens,
    (wrinkle)
    (faceVertexCounts)
    (faceVertexIndices));

TF_DEFINE_PRIVATE_TOKENS(_wrinkle,
    ((rest, rigExec::SchemaAttrText::inputsRestPoints))
    ((iterations, rigExec::SchemaAttrText::inputsIterations))
    ((topology, rigExec::SchemaAttrText::inputsTopology))
    ((neighborDistance, rigExec::SchemaAttrText::inputsNeighborDistance))
    ((restLengthScale, rigExec::SchemaAttrText::inputsRestLengthScale))
    ((stretchStiffness, rigExec::SchemaAttrText::inputsStretchStiffness))
    ((compressionStiffness, rigExec::SchemaAttrText::inputsCompressionStiffness))
    ((bendStiffness, rigExec::SchemaAttrText::inputsBendStiffness))
    ((maxDisplacement, rigExec::SchemaAttrText::inputsMaxDisplacement))
    ((pinBorders, rigExec::SchemaAttrText::inputsPinBorders))
    ((pinPoints, rigExec::SchemaAttrText::inputsPinPoints))
    ((tangentPlaneCollisions, rigExec::SchemaAttrText::inputsTangentPlaneCollisions))
    ((tangentPlaneInset, rigExec::SchemaAttrText::inputsTangentPlaneInset))
    ((wrinkleScale, rigExec::SchemaAttrText::inputsWrinkleScale))
    ((smoothingIterations, rigExec::SchemaAttrText::inputsSmoothingIterations))
    (cloth) (surfaceStruts));

namespace {
const TfToken _oracleToken0("faceVertexCounts");
const TfToken _oracleToken1("faceVertexIndices");

void
Bind(const RigExecMoverBindContext &ctx)
{
    ctx.binding->base = ctx.target;
    ctx.binding->topologyCounts =
        ctx.ownerPath.AppendProperty(_wrinkleTokens->faceVertexCounts);
    ctx.binding->topologyIndices =
        ctx.ownerPath.AppendProperty(_wrinkleTokens->faceVertexIndices);
}

bool
Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    if (!UsdGeomMesh(
            ctx.stage->GetPrimAtPath(ctx.targets.front().GetPrimPath()))) {
        *error = "Wrinkle requires polygon mesh topology";
        return false;
    }
    for (const TfToken &name :
         {_wrinkle->rest, _wrinkle->topology, _wrinkle->pinPoints}) {
        const UsdAttribute attr = ctx.prim.GetAttribute(name);
        if (attr.GetNumTimeSamples() != 0 || attr.HasAuthoredConnections()) {
            *error = "Wrinkle " + name.GetString() +
                     " must be static and unconnected";
            return false;
        }
    }
    TfToken topology;
    ctx.prim.GetAttribute(_wrinkle->topology).Get(&topology);
    if (topology != _wrinkle->cloth && topology != _wrinkle->surfaceStruts) {
        *error = "Wrinkle topology must be cloth or surfaceStruts";
        return false;
    }
    return true;
}

RigExecOracleResult
Oracle(const RigExecMoverOracleContext &ctx)
{
    VtVec3fArray rest;
    ctx.prim.GetAttribute(_wrinkle->rest).Get(&rest);
    if (rest.empty())
        rest = ctx.basePoints;
    const RigExecOraclePrim owner =
        ctx.stage->GetPrimAtPath(ctx.target.GetPrimPath());
    VtIntArray counts, indices;
    owner.GetAttribute(_oracleToken0).Get(&counts, ctx.time);
    owner.GetAttribute(_oracleToken1).Get(&indices, ctx.time);
    RigExecWrinkleSettings s;
    TfToken topology;
    ctx.prim.GetAttribute(_wrinkle->topology).Get(&topology);
    s.topology = topology == _wrinkle->cloth
                     ? RigExecWrinkleTopology::Cloth
                     : RigExecWrinkleTopology::SurfaceStruts;
    VtIntArray pins;
    ctx.prim.GetAttribute(_wrinkle->pinPoints).Get(&pins);
    s.pinPoints.assign(pins.begin(), pins.end());
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->iterations),
                              ctx.time, &s.iterations);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->neighborDistance),
                              ctx.time, &s.neighborDistance);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->restLengthScale),
                              ctx.time, &s.restLengthScale);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->stretchStiffness),
                              ctx.time, &s.stretchStiffness);
    ctx.resolved.GetAttribute(
        ctx.prim.GetAttribute(_wrinkle->compressionStiffness), ctx.time,
        &s.compressionStiffness);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->bendStiffness),
                              ctx.time, &s.bendStiffness);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->maxDisplacement),
                              ctx.time, &s.maxDisplacement);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->pinBorders),
                              ctx.time, &s.pinBorders);
    ctx.resolved.GetAttribute(
        ctx.prim.GetAttribute(_wrinkle->tangentPlaneCollisions), ctx.time,
        &s.tangentPlaneCollisions);
    ctx.resolved.GetAttribute(
        ctx.prim.GetAttribute(_wrinkle->tangentPlaneInset), ctx.time,
        &s.tangentPlaneInset);
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(_wrinkle->wrinkleScale),
                              ctx.time, &s.wrinkleScale);
    ctx.resolved.GetAttribute(
        ctx.prim.GetAttribute(_wrinkle->smoothingIterations), ctx.time,
        &s.smoothingIterations);
    std::vector<GfVec3f> points(ctx.points->begin(), ctx.points->end());
    if (!RigExecApplyWrinkle(&points, {rest.begin(), rest.end()},
                             {counts.begin(), counts.end()},
                             {indices.begin(), indices.end()}, s)) {
        ctx.diagnostics->push_back("MoverFailed " + ctx.moverPath.GetString() +
                                   ": invalid wrinkle inputs");
        return RigExecOracleResult::PassThrough;
    }
    std::copy(points.begin(), points.end(), ctx.points->begin());
    return RigExecOracleResult::Blend;
}

RigExecMoverHandler
Handler()
{
    RigExecMoverHandler h(rigExec::SchemaTypeName::RigExecWrinkleMover,
                          &RigExecFixedMoverOp<RigExecRevisionOp::Wrinkle>,
                          RigExecMoverDomain::Points);
    h.singleTarget = true;
    h.bind = &Bind;
    h.validate = &Validate;
    h.oracle = &Oracle;
    return h;
}

} // namespace

RIGEXEC_REGISTER_MOVER(Handler());
