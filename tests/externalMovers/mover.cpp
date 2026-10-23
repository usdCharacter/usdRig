#include "rigExec/movers/moverRegistry.h"
#include "rigExecScene/sceneDescriptors.h"
#include "kernel.h"
#include "testExternalMoverSchema.h"

#include <cmath>
#include <cstring>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

const TfToken kGain("inputs:gain");
const TfToken kReference("rigExec:reference");

// The payload: (gain, reference y, whether a reference was read). A
// reference is any points property named by rigExec:reference, read at the
// phase that relationship declares; its first point's y is added to every
// point, so a phased read that playback re-evaluates shows in the result.
using Payload = GfVec3f;

void Bind(const RigExecMoverBindContext &ctx)
{
    const SdfPathVector references =
        RigExecRelationshipTargets(ctx.moverPrim, kReference);
    if (references.size() != 1) return;
    const RigExecReadPhase phase =
        RigExecPhaseForInput(ctx.moverPrim, kReference);
    if (!phase.IsBase()) {
        ctx.binding->phases[references[0]] = phase;
    }
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs)
{
    inputs->push_back({ctx.moverPrim.GetPath().AppendProperty(kGain),
        RigExecRevisionLeafType::Float, RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved, VtValue(testExternalMoverSchema::inputs_gain)});
    const auto references = RigExecRelationshipTargets(ctx.moverPrim, kReference);
    if (references.size() == 1) {
        inputs->push_back({references[0], RigExecRevisionLeafType::Vec3fArray,
            RigExecRevisionLeafTime::AtTime, RigExecRevisionLeafFlavour::Resolved,
            VtValue(VtVec3fArray())});
    }
}

bool CompileScene(const RigExecSceneDescriptors &scene,const SdfPath &mover,
                  const SdfPath &,RigExecRevisionBinding *binding,std::string *error)
{
    if(!binding)return false;
    binding->externalInputs={{mover.AppendProperty(kGain),
        RigExecRevisionLeafType::Float,RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved,VtValue(testExternalMoverSchema::inputs_gain)}};
    const auto reference=scene.relationships.find(mover.AppendProperty(kReference));
    if(reference!=scene.relationships.end() && reference->second.fact.targets.size()==1) {
        const auto &path=reference->second.fact.targets.front();RigExecReadPhase phase;
        if(!RigExecParseReadPhase(reference->second.readPhase.GetText(),&phase,error))return false;
        if(!phase.IsBase())binding->phases[path]=phase;
        binding->externalInputs.push_back({path,RigExecRevisionLeafType::Vec3fArray,
            RigExecRevisionLeafTime::AtTime,RigExecRevisionLeafFlavour::Resolved,VtValue(VtVec3fArray())});
    }
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *data)
{
    if (ctx.values.basePoints.empty() || ctx.inputs.empty() ||
        !ctx.inputs[0].IsHolding<float>()) return false;
    const float gain = ctx.inputs[0].UncheckedGet<float>();
    if (!std::isfinite(gain)) return false;
    Payload payload(gain, 0.0f, 0.0f);
    if (ctx.inputs.size() == 2) {
        if (!ctx.inputs[1].IsHolding<VtVec3fArray>()) return false;
        const auto &points = ctx.inputs[1].UncheckedGet<VtVec3fArray>();
        if (points.empty()) return false;
        payload[1] = points[0][1];
        payload[2] = 1.0f;
    }
    *data = VtValue(payload);
    return true;
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *points)
{
    if (!data.IsHolding<Payload>()) return false;
    const Payload &payload = data.UncheckedGet<Payload>();
    if (payload[0] == -2.0f) {
        points->push_back(GfVec3f(0));
        return true;
    }
    return TestQuadraticDefinition::Deform(payload[0], payload[1],
                  points->empty() ? nullptr : points->front().data(),
                  points->size());
}

RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx)
{
    float gain = 1.0f;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(kGain),
                              ctx.time, &gain);
    if (!std::isfinite(gain) || gain < 0.0f) {
        return RigExecOracleResult::PassThrough;
    }
    for (GfVec3f &point : *ctx.points) {
        point[2] += gain * point[0] * point[0];
    }
    return RigExecOracleResult::Blend;
}

// .rigexec export: compiled data bytes are a format tag the kernel checks; frame
// bytes are the payload's three floats.
constexpr auto &kCompiledDataTag = TestQuadraticDefinition::CompiledTag;

bool EncodeCompiledData(const RigExecRevisionBinding &,std::vector<uint8_t> *compiledRig) {
    compiledRig->assign(kCompiledDataTag,kCompiledDataTag+sizeof(kCompiledDataTag));return true;
}

bool Encode(const VtValue &data, const RigExecRevisionBinding &,
            std::vector<uint8_t> *compiledRig, std::vector<uint8_t> *frame)
{
    if (!data.IsHolding<Payload>()) return false;
    const Payload &payload = data.UncheckedGet<Payload>();
    compiledRig->assign(kCompiledDataTag, kCompiledDataTag + sizeof(kCompiledDataTag));
    frame->resize(sizeof(float) * 3);
    std::memcpy(frame->data(), payload.data(), frame->size());
    return true;
}

RigExecMoverHandler Handler(const char *schemaType)
{
    RigExecMoverHandler handler(schemaType,
        &RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.bind = &Bind;
    handler.declareExternalInputs = &DeclareInputs;
    handler.compileScene = &CompileScene;
    handler.assembleExternal = &Assemble;
    handler.applyExternal = &Apply;
    handler.oracle = &Oracle;
    handler.encodeExternalCompiledData = &EncodeCompiledData;
    handler.encodeExternal = &Encode;
    return handler;
}

// The same mover from a plugin that never adopted .rigexec export.
RigExecMoverHandler UnexportableHandler()
{
    RigExecMoverHandler handler = Handler("ExternalUnexportableMover");
    handler.encodeExternalCompiledData = nullptr;
    handler.encodeExternal = nullptr;
    handler.runtimeKernel = RigExecExternalKernel();
    return handler;
}

}  // namespace

namespace rigExec {
RigExecMoverHandler RigExecQuadraticUsdAdapter(const char *schemaType)
{
    return Handler(schemaType);
}
}
RIGEXEC_REGISTER_MOVER(UnexportableHandler());
