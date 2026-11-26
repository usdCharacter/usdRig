// Op nodes injected into a real rig's point chain run as steps of the baked
// schedule: a head per range chain and one step per (segment, vertex
// group), on the chain's own vertex groups. Native, frame-cache and binary
// runtime evaluation give the same bits, and an edit that moves one group
// reruns only the op steps of that group and the groups reading it.
#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/program/program.h"
#include "rigExecBinary/format.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/property/mathOps.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

namespace {

// --- A gated chain ---------------------------------------------------------

constexpr size_t kPoints = 10000;
const char *const kRig = "/Asset/Rig";
const char *const kRaise = "/Asset/Rig/BlendInputs/Raise";
const char *const kLift = "/Asset/Rig/BlendInputs/Lift";
const char *const kMoving = "/Asset/Rig/Moving";

/// The gated mover's support: every tenth point of [500, 2500), so three of
/// the chain's ten groups hold supported points beside unsupported ones.
bool
Supported(size_t i)
{
    return i >= 500 && i < 2500 && i % 10 == 3;
}

/// One 10000-point target under a blend shape and then a gated radial
/// Matrix mover. `Raise` moves points outside the mover's support in the
/// groups it writes; `Lift` moves ten supported points of one group;
/// `Moving` is the mover's transform. \p raise, \p lift and \p moving are
/// the authored values.
UsdStageRefPtr
MakeGatedStage(float raise, float lift, double moving, size_t count = kPoints)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath(kRig), TfToken("RigExecRoot"));
    const UsdPrim control =
        stage->DefinePrim(SdfPath(kMoving), TfToken("RigExecControl"));
    control.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(moving);
    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray base(count), raised(count), lifted(count);
    VtIntArray support;
    VtFloatArray weights;
    for (size_t i = 0; i < count; ++i) {
        base[i] = GfVec3f(float(i % 100), float(i / 100), 0.0f);
        raised[i] = base[i];
        lifted[i] = base[i];
        if (i < 4000 && i % 10 == 7) {
            raised[i][2] += 1.0f;
        }
        if (i >= 1000 && i < 1100 && Supported(i)) {
            lifted[i][1] += 2.0f;
        }
        if (count == 8 ? i == 1 : Supported(i)) {
            support.push_back(int(i));
            // A falloff of a few distinct values, some at the ends.
            weights.push_back(count == 8 ? 1.0f : float((i / 10) % 5) / 4.0f);
        }
    }
    const auto points = [&](const char *path, const VtVec3fArray &values) {
        stage->DefinePrim(SdfPath(path), TfToken("Points"))
            .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
            .Set(values);
    };
    points("/Asset/Shape", base);
    points("/Asset/Raised", raised);
    points("/Asset/Lifted", lifted);
    const auto input = [&](const char *path, const char *shape, float value) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecBlendInput"));
        prim.CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float)
            .Set(value);
        const UsdPrim sample =
            stage->DefinePrim(SdfPath(path).AppendChild(TfToken("Full")),
                              TfToken("RigExecBlendSample"));
        sample
            .CreateAttribute(TfToken("rigExec:activation"),
                             SdfValueTypeNames->Float)
            .Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:targetPoints"))
            .SetTargets({SdfPath(shape).AppendProperty(TfToken("points"))});
        prim.CreateRelationship(TfToken("rigExec:samples"))
            .SetTargets({sample.GetPath()});
    };
    input(kRaise, "/Asset/Raised", raise);
    input(kLift, "/Asset/Lifted", lift);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sparse"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight
        .CreateAttribute(TfToken("rigExec:representation"),
                         SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight
        .CreateAttribute(TfToken("rigExec:indices"),
                         SdfValueTypeNames->IntArray, false)
        .Set(support);
    weight
        .CreateAttribute(TfToken("rigExec:values"),
                         SdfValueTypeNames->FloatArray, false)
        .Set(weights);
    weight
        .CreateAttribute(TfToken("rigExec:defaultWeight"),
                         SdfValueTypeNames->Float, false)
        .Set(0.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto mover = [&](const char *name, const char *type) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name), TfToken(type));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        return prim;
    };
    // Movers run last sibling first: the blend, then the gated mover.
    const UsdPrim gated = mover("Gated", "RigExecMatrixMover");
    gated.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({control.GetPath()});
    gated.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});
    gated
        .CreateAttribute(TfToken("rigExec:weightBlend"),
                         SdfValueTypeNames->Token)
        .Set(TfToken("radial"));
    const UsdPrim blend = mover("Blend", "RigExecBlendShapeMover");
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({SdfPath(kRaise), SdfPath(kLift)});
    blend
        .CreateAttribute(TfToken("inputs:defaultWeight"),
                         SdfValueTypeNames->Float)
        .Set(1.0f);
    return stage;
}

RigExecValueOverride
Override(const char *prim, const char *attribute, const VtValue &value)
{
    return RigExecValueOverride{SdfPath(prim), TfToken(), TfToken(attribute),
                                value};
}

/// The stage's points as quads, 100 to a row.
void
GridFaces(size_t count, VtIntArray *counts, VtIntArray *indices)
{
    const int w = 100;
    const int rows = int(count) / w;
    for (int r = 0; r + 1 < rows; ++r)
        for (int c = 0; c + 1 < w; ++c) {
            counts->push_back(4);
            indices->push_back(r * w + c);
            indices->push_back(r * w + c + 1);
            indices->push_back((r + 1) * w + c + 1);
            indices->push_back((r + 1) * w + c);
        }
}

GfMatrix4d
Offset()
{
    GfMatrix4d m(1.0);
    m.SetScale(GfVec3d(1.0, 1.5, 1.0));
    m.SetTranslateOnly(GfVec3d(0.25, 0.0, 0.5));
    return m;
}

/// The injected op nodes: the moving control's final matrix times a fixed
/// offset (a node of its own) moves the chain's final points, which are
/// then smoothed over the grid.
struct OpRig {
    std::shared_ptr<const RigExecRigDesc> desc;
    RigExecNodeId math = 0, deform = 0, smooth = 0;
    RigExecInputId matrix, value, weight, points, counts, indices, strength;
    RigExecOutputId out;
};

/// The ops, added to \p into when given, else built into their own
/// description.
OpRig
MakeOps(bool envelope = false, RigExecRigDescBuilder *into = nullptr)
{
    using namespace rigExecOps;
    OpRig rig;
    RigExecRigDescBuilder own;
    RigExecRigDescBuilder &b = into ? *into : own;
    MathPayload math;
    math.op = RigExecPropertyOp::Multiply;
    MatrixDeformPayload deform;
    rig.math = b.AddOp(MatrixMath(), &math);
    rig.deform = b.AddOp(MatrixDeform(), &deform);
    rig.smooth = b.AddOp(Smooth());
    b.Connect(rig.math, MatrixOut(), rig.deform, MatrixDeformTransform());
    b.Connect(rig.deform, MatrixDeformOut(), rig.smooth, SmoothPoints());
    rig.matrix = b.BindInput(rig.math, MatrixIn());
    rig.value = b.BindInput(rig.math, MatrixValue());
    rig.weight = b.BindInput(rig.math, MatrixWeight());
    rig.points = b.BindInput(rig.deform, MatrixDeformPoints());
    rig.counts = b.BindInput(rig.smooth, SmoothCounts());
    rig.indices = b.BindInput(rig.smooth, SmoothIndices());
    rig.strength = b.BindInput(rig.smooth, SmoothStrength());
    if (envelope)
        b.BindInput(rig.deform, MatrixDeformEnvelope());
    rig.out = b.BindOutput(rig.smooth, SmoothOut());
    if (into)
        return rig;
    std::string error;
    rig.desc = b.Build(&error);
    if (!rig.desc)
        std::printf("op description: %s\n", error.c_str());
    return rig;
}

/// The base-matrix slot the gated mover's transform reads, or -1.
int
SlotOf(const RigExecBakedProgramImpl &B, const char *)
{
    int slot = -1;
    for (const auto &chain : B.chains)
        for (const auto &revision : chain.revisions)
            if (revision.moverPath.GetName() == "Gated")
                RigExecBakedForEachMatrixRead(
                    revision, [&](RigExecBakedSlotDomain domain, int s) {
                        if (domain == RigExecBakedSlotDomain::BaseMatrix)
                            slot = s;
                    });
    return slot;
}

int
ChainOf(const RigExecBakedProgramImpl &B, const char *target)
{
    for (size_t c = 0; c < B.chains.size(); ++c)
        if (B.chains[c].target == SdfPath(target))
            return int(c);
    return -1;
}

/// Injects MakeOps over \p B's chain of /Asset/Shape.points; \p faces
/// replaces the grid's face indices when not empty.
bool
Inject(const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out,
       const VtFloatArray &envelope)
{
    using namespace rigExecOps;
    const int chain = ChainOf(B, "/Asset/Shape.points");
    const int slot = SlotOf(B, kMoving);
    if (chain < 0 || slot < 0)
        return false;
    const OpRig rig = MakeOps(!envelope.empty(), out->builder);
    out->hostInputs = {{rig.math, MatrixIn().Index(),
                        eval::OpHostSource::BaseMatrix, uint32_t(slot)},
                       {rig.deform, MatrixDeformPoints().Index(),
                        eval::OpHostSource::ChainPoints, uint32_t(chain)}};
    VtIntArray counts, indices;
    GridFaces(B.chains[size_t(chain)].groupPointCount, &counts, &indices);
    out->constants = {{rig.math, MatrixValue().Index(), VtValue(Offset())},
                      {rig.math, MatrixWeight().Index(), VtValue(1.0f)},
                      {rig.smooth, SmoothCounts().Index(), VtValue(counts)},
                      {rig.smooth, SmoothIndices().Index(), VtValue(indices)},
                      {rig.smooth, SmoothStrength().Index(), VtValue(0.5f)}};
    if (!envelope.empty())
        out->constants.push_back(
            {rig.deform, MatrixDeformEnvelope().Index(), VtValue(envelope)});
    out->outputs = {{"smoothed", rig.out}};
    return true;
}

bool
SameBits(const VtVec3fArray &a, const VtVec3fArray &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.cdata(), b.cdata(), a.size() * sizeof(GfVec3f)) == 0);
}

const VtVec3fArray *
Smoothed(const RigExecRigPose &pose)
{
    const auto found = pose.opOutputs.find("smoothed");
    return found != pose.opOutputs.end() &&
                   found->second.IsHolding<VtVec3fArray>()
               ? &found->second.UncheckedGet<VtVec3fArray>()
               : nullptr;
}

/// MakeOps run by a plain evaluator over \p B's final values.
VtVec3fArray
Reference(const RigExecBakedProgramImpl &B, const RigExecRigPose &pose)
{
    const OpRig rig = MakeOps();
    auto program = RigExecCompile(*rig.desc);
    auto evaluator = RigExecEvaluator::Create(program);
    rigExecStd::Mat4d matrix, offset;
    std::memcpy(&matrix, B.baseMatrix[size_t(SlotOf(B, kMoving))].data(),
                sizeof(matrix));
    std::memcpy(&offset, Offset().data(), sizeof(offset));
    const float one = 1.0f, half = 0.5f;
    const VtVec3fArray &points =
        pose.movedProperties.at(SdfPath("/Asset/Shape.points"))
            .UncheckedGet<VtVec3fArray>();
    const rigExecStd::RigExecArray<rigExecStd::Vec3f> in(
        reinterpret_cast<const rigExecStd::Vec3f *>(points.cdata()),
        points.size());
    VtIntArray counts, faces;
    GridFaces(points.size(), &counts, &faces);
    const rigExecStd::RigExecArray<int32_t> c(counts.cdata(), counts.size());
    const rigExecStd::RigExecArray<int32_t> f(faces.cdata(), faces.size());
    evaluator->SetInput(rig.matrix, &matrix);
    evaluator->SetInput(rig.value, &offset);
    evaluator->SetInput(rig.weight, &one);
    evaluator->SetInput(rig.points, &in);
    evaluator->SetInput(rig.counts, &c);
    evaluator->SetInput(rig.indices, &f);
    evaluator->SetInput(rig.strength, &half);
    std::string error;
    VtVec3fArray result;
    if (!evaluator->Evaluate(&error)) {
        std::printf("reference: %s\n", error.c_str());
        return result;
    }
    const auto *out =
        static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
            evaluator->GetOutput(rig.out));
    result.assign(reinterpret_cast<const GfVec3f *>(out->Data()),
                  reinterpret_cast<const GfVec3f *>(out->Data()) + out->Size());
    return result;
}

/// The (segment, group) op range steps the last run ran.
std::set<std::pair<int, int>>
RanRanges(const RigExecBakedProgramImpl &B, int groups)
{
    std::set<std::pair<int, int>> ran;
    for (size_t i = 0; i < B.steps.size(); ++i)
        if (B.steps[i].kind == RigExecBakedStepKind::OpChainRange &&
            i < B.opExecution.ran.size() && B.opExecution.ran[i])
            ran.insert({B.steps[i].part / groups, B.steps[i].part % groups});
    return ran;
}

/// A frame-cache worker's pose at \p time.
RigExecRigPose
Frozen(RigExecRigEvaluator &evaluator, UsdTimeCode time)
{
    std::string error;
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    RigExecFrameInputs inputs;
    if (!RigExecFreezeProgram(evaluator, &frozen, &error) || !frozen ||
        !RigExecSampleFrameInputs(evaluator, time, {}, &inputs, &error)) {
        std::printf("frozen: %s\n", error.c_str());
        return RigExecRigPose();
    }
    RigExecCancellationSource cancellation;
    RigExecFrozenEvalContext context;
    context.compiledRigSignature = evaluator.GetCompiledRigSignature();
    context.generation = cancellation.Serial();
    context.slotCount = evaluator.GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    context.frozen = frozen.get();
    return RigExecEvaluateFrozen(context, inputs,
                                 RigExecMakeProductionStepRunner(),
                                 cancellation.Token(context.generation));
}

void
TestInjectedChain()
{
    const UsdTimeCode time(1.0);
    RigExecBakedProgramTesting::SetOpInjector(
        [](const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out) {
            return Inject(B, out, VtFloatArray());
        });
    const UsdStageRefPtr stage = MakeGatedStage(0.5f, 0.0f, 1.0);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    for (const std::string &line : errors)
        std::printf("  %s\n", line.c_str());
    if (!compiled)
        return;
    const RigExecRigPose first = evaluator.Evaluate(time);
    CHECK(first.valid && evaluator.GetBakedProgram());
    if (!evaluator.GetBakedProgram())
        return;
    const RigExecBakedProgramImpl &B =
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram());
    CHECK(B.opProgram && B.opState);
    if (!B.opProgram)
        return;
    const int chain = ChainOf(B, "/Asset/Shape.points");

    const int groups = int(B.chains[size_t(chain)].groupBounds.size()) - 1;
    CHECK(groups == 10);
    size_t nodes = 0, heads = 0, ranges = 0;
    for (const RigExecBakedStep &step : B.steps) {
        nodes += step.kind == RigExecBakedStepKind::OpNode &&
                 !RigExecBakedOriginOf(B, step);
        heads += step.kind == RigExecBakedStepKind::OpChainHead;
        ranges += step.kind == RigExecBakedStepKind::OpChainRange;
    }
    // MatrixMath alone; one chain; its two segments (the smooth reads a
    // neighbourhood) over every group.
    CHECK(nodes == 1 && heads == 1 && ranges == 2 * size_t(groups));
    const VtVec3fArray *native = Smoothed(first);
    CHECK(native && native->size() == kPoints);
    if (!native)
        return;
    CHECK(SameBits(*native, Reference(B, first)));
    CHECK(RanRanges(B, groups).size() == 2 * size_t(groups));

    // One group's points move: the matrix-deform segment reruns that group
    // only, the smooth the groups whose halo holds it.
    const auto edit = [&](float lift, double moving) {
        evaluator.SetInteractiveOverrides(
            {Override(kLift, "inputs:weight", VtValue(lift)),
             Override(kMoving, "avars:tx", VtValue(moving))});
        return evaluator.Evaluate(time);
    };
    CHECK(edit(0.0f, 1.0).valid);
    const RigExecRigPose lifted = edit(0.5f, 1.0);
    CHECK(lifted.valid);
    const std::set<std::pair<int, int>> ran = RanRanges(B, groups);
    const std::set<std::pair<int, int>> wanted = {
        {0, 1}, {1, 0}, {1, 1}, {1, 2}};
    if (ran != wanted) {
        std::printf("lift reran:");
        for (const auto &[segment, g] : ran)
            std::printf(" (%d, %d)", segment, g);
        std::printf("\n");
    }
    CHECK(ran == wanted);
    const VtVec3fArray *after = Smoothed(lifted);
    CHECK(after && SameBits(*after, Reference(B, lifted)));
    CHECK(after && !SameBits(*after, *native));
    // The control the head reads: every group.
    const RigExecRigPose moved = edit(0.5f, 2.0);
    CHECK(moved.valid && RanRanges(B, groups).size() == 2 * size_t(groups));
    CHECK(Smoothed(moved) && SameBits(*Smoothed(moved), Reference(B, moved)));

    // A frame-cache worker's clone runs its own run of the same nodes.
    evaluator.SetInteractiveOverrides({});
    const RigExecRigPose live = evaluator.Evaluate(time);
    const RigExecRigPose warmed = Frozen(evaluator, time);
    CHECK(warmed.valid);
    CHECK(Smoothed(warmed) && Smoothed(live));
    CHECK(Smoothed(warmed) && Smoothed(live) &&
          SameBits(*Smoothed(warmed), *Smoothed(live)));
    RigExecBakedProgramTesting::SetOpInjector(nullptr);
}

/// The runtime's named output \p name as points; empty when it has none.
VtVec3fArray
RuntimeOutput(const RigExecRuntimeReader &reader, const char *name)
{
    size_t index = 0;
    RigExecRuntimeOpOutput output;
    VtVec3fArray out;
    if (!reader.FindOpOutput(name, &index) ||
        !reader.GetOpOutput(index, &output) || !output.value)
        return out;
    const auto &points =
        *static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
            output.value);
    out.assign(reinterpret_cast<const GfVec3f *>(points.Data()),
               reinterpret_cast<const GfVec3f *>(points.Data()) +
                   points.Size());
    return out;
}

bool
Contains(const std::string &text, const char *part)
{
    const bool found = text.find(part) != std::string::npos;
    if (!found)
        std::printf("  expected \"%s\" in: %s\n", part, text.c_str());
    return found;
}

std::vector<uint8_t>
Pack(const fb::RigExecWireFile &file)
{
    flatbuffers::FlatBufferBuilder builder;
    fb::FinishFileBuffer(builder, fb::File::Pack(builder, &file));
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() + builder.GetSize());
}

/// Open refuses \p file, with \p why in the reason.
void
Refused(const fb::RigExecWireFile &file, const char *why)
{
    const std::vector<uint8_t> bytes = Pack(file);
    std::string error;
    CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error));
    CHECK(Contains(error, why));
}

/// The baked rig in the runtime: the op steps run there as in the native
/// program, bit for bit, edits included; files whose op graph is damaged
/// are refused at Open.
void
TestRuntime(const char *writeTo)
{
    RigExecBakedProgramTesting::SetOpInjector(
        [](const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out) {
            return Inject(B, out, VtFloatArray());
        });
    const UsdStageRefPtr stage = MakeGatedStage(0.5f, 0.0f, 1.0);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    std::vector<uint8_t> bytes;
    std::string error;
    RigExecTestPlayer player;
    const bool opened = RigExecTestBakeAt(evaluator, 1.0, &bytes, &error) &&
                        player.Open(bytes, stage, &error);
    if (!opened)
        std::printf("bake or open: %s\n", error.c_str());
    CHECK(opened);
    if (!opened)
        return;
    // Native, frozen and the runtime pick the same kernel paths:
    // RIGEXEC_ENABLE_SIMD decides for all three.
    const RigExecBakedProgramImpl &B =
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram());
    CHECK(B.opProgram && B.opProgram->layout.program &&
          B.opProgram->layout.program->useSimd == B.useSimd);
    CHECK(player.Reader().GetSimdEnabledForTesting() == B.useSimd);
    if (writeTo) {
        std::ofstream out(writeTo, std::ios::binary);
        out.write(reinterpret_cast<const char *>(bytes.data()),
                  std::streamsize(bytes.size()));
        CHECK(bool(out));
    }
    struct Edit {
        float lift;
        double moving;
    };
    for (const Edit edit :
         {Edit{0.0f, 1.0}, Edit{0.5f, 1.0}, Edit{0.5f, 2.0}}) {
        evaluator.SetInteractiveOverrides(
            {Override(kLift, "inputs:weight", VtValue(edit.lift)),
             Override(kMoving, "avars:tx", VtValue(edit.moving))});
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1.0));
        CHECK(player.Hold(std::string(kLift) + ".inputs:weight", edit.lift,
                          &error));
        CHECK(player.Hold(std::string(kMoving) + ".avars:tx", edit.moving,
                          &error));
        CHECK(player.Play(1.0, &error));
        const VtVec3fArray played = RuntimeOutput(player.Reader(), "smoothed");
        CHECK(pose.valid && Smoothed(pose) && !played.empty() &&
              SameBits(*Smoothed(pose), played));
    }

    std::unique_ptr<fb::RigExecWireFile> file;
    CHECK(RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error));
    if (!file || !file->opGraph)
        return;
    const auto damaged = [&](auto &&change, const char *why) {
        std::unique_ptr<fb::RigExecWireFile> copy;
        RigExecFormatOpen(bytes.data(), bytes.size(), &copy, &error);
        change(*copy);
        Refused(*copy, why);
    };
    damaged(
        [](fb::RigExecWireFile &f) {
            f.manifest->coreOps[0]->name = "Missing";
        },
        "rig:Missing v1 is in no op library");
    damaged(
        [](fb::RigExecWireFile &f) {
            f.manifest->coreTypes[0]->name = "Missing";
        },
        "Missing v1 is in no type library");
    // The injected nodes come after the rig's weight packet nodes.
    const size_t injected = file->opGraph->nodes.size() - 3;
    damaged(
        [&](fb::RigExecWireFile &f) {
            f.opGraph->nodes[injected]->payload.resize(3);
        },
        "settings do not decode");
    damaged(
        [](fb::RigExecWireFile &f) {
            for (auto &node : f.opGraph->nodes)
                for (auto &input : node->inputs)
                    if (input->node >= 0)
                        input->node = 9;
        },
        "node 9 out of range");
    damaged(
        [&](fb::RigExecWireFile &f) {
            f.opGraph->nodes[injected]->inputs[0]->port = 40;
        },
        "has no input port 40");
    damaged([&](fb::RigExecWireFile &f) { f.opGraph->nodes[injected]->op = 7; },
            "past manifest core_ops");
    damaged(
        [&](fb::RigExecWireFile &f) {
            f.opGraph->order.emplace_back(uint32_t(injected + 2),
                                          uint32_t(injected));
        },
        "cycle");
    damaged(
        [&](fb::RigExecWireFile &f) {
            for (auto &input : f.opGraph->nodes[injected + 2]->inputs)
                if (input->host == 3)
                    input->constant.resize(3);
        },
        "is not of its type");
    damaged(
        [](fb::RigExecWireFile &f) { f.opGraph->chains[0]->bounds[1] = -5; },
        "its bounds are no cut into groups");
    damaged([](fb::RigExecWireFile &f) { f.formatVersion = 35; },
            "unsupported .rigexec format version 35");
    RigExecBakedProgramTesting::SetOpInjector(nullptr);
}

/// An envelope shorter than the chain's points: the frame stops with the
/// matrix deform's error and publishes nothing.
void
TestFailingNode()
{
    RigExecBakedProgramTesting::SetOpInjector(
        [](const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out) {
            return Inject(B, out, VtFloatArray(5, 0.5f));
        });
    const UsdStageRefPtr stage = MakeGatedStage(0.5f, 0.0f, 1.0);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    std::string reported;
    for (const std::string &line : errors)
        reported += line + "\n";
    if (compiled) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(1.0));
        CHECK(!pose.valid);
        for (const std::string &line : pose.diagnostics)
            reported += line + "\n";
    }
    std::printf("failing node: %s", reported.c_str());
    CHECK(compiled && reported.find("MatrixDeform") != std::string::npos &&
          reported.find("envelope") != std::string::npos);
    RigExecBakedProgramTesting::SetOpInjector(nullptr);
}

/// The file \p path holds op steps: Open refuses it while no op or type
/// library is registered, and opens it given the built-in libraries.
void
TestUnregistered(const char *path)
{
    std::ifstream in(path, std::ios::binary);
    const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
    CHECK(!bytes.empty());
    CHECK(!RigExecRegistry::IsInitialized());
    std::string error;
    CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error));
    CHECK(Contains(error, "open it with the op and type libraries"));
    CHECK(!RigExecRegistry::IsInitialized());
    const bool opened =
        bool(RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr, 0,
                                        RigExecBuiltinRegistrations(), &error));
    if (!opened)
        std::printf("open with libraries: %s\n", error.c_str());
    CHECK(opened && RigExecRegistry::IsInitialized());
}

/// A host that resolves op steps' lookups and stores their group outputs
/// on its own: a chain member's lookup is resolved by the chain's head, and
/// each range step hands every group it wrote to StoreGroup.
struct GroupHost {
    GfMatrix4d matrix;
    std::vector<GfMatrix4d> bases;
    std::vector<int> bounds;
    std::vector<VtVec3fArray> groups;
    /// Per chain version, its groups.
    std::vector<std::vector<VtVec3fArray>> versions;
    float strength = 0.5f;
    int lookups = 0;
    std::vector<VtVec3fArray> stored;
    std::vector<int> stores;
    /// Per frame record and constraint delta, its matrix when present.
    std::vector<std::pair<GfMatrix4d, bool>> records, deltas;

    const void *Matrix(eval::OpHostSource source, uint32_t slot) const
    {
        if (source == eval::OpHostSource::BaseMatrix)
            return slot < bases.size() ? bases[slot].data() : nullptr;
        return matrix.data();
    }
    const void *PresentMatrix(const eval::OpLookup &lookup) const
    {
        const auto &table =
            lookup.source == uint8_t(eval::OpLookupSource::FrameRecord)
                ? records
                : deltas;
        return lookup.object < table.size() && table[lookup.object].second
                   ? table[lookup.object].first.data()
                   : nullptr;
    }
    const void *ChainGroupPoints(uint32_t, uint32_t g) const
    {
        return groups[g].cdata();
    }
    const void *ChainVersionGroupPoints(const eval::OpLookup &lookup,
                                        uint32_t g) const
    {
        return versions[size_t(lookup.part)][g].cdata();
    }
    bool LookUpProperty(uint32_t, const eval::OpLookup &,
                        const RigExecTypeDesc &, void *value, std::string *)
    {
        ++lookups;
        *static_cast<float *>(value) = strength;
        return true;
    }
    bool LookUpPoseVersion(uint32_t, const eval::OpLookup &,
                           const RigExecTypeDesc &, void *, std::string *error)
    {
        *error = "no pose versions";
        return false;
    }
    bool StorePropertyVersion(const eval::OpHostOutput &, const void *,
                              std::string *error)
    {
        *error = "no property versions";
        return false;
    }
    bool StoreVolumePlacement(const eval::OpHostOutput &, const void *,
                              std::string *error)
    {
        *error = "no volume placements";
        return false;
    }
    bool LookUpWeight(uint32_t, const eval::OpLookup &, const RigExecTypeDesc &,
                      void *, std::string *error)
    {
        *error = "no weight objects";
        return false;
    }
    bool StoreWeightPacket(const eval::OpHostOutput &, const void *,
                           std::string *error)
    {
        *error = "no weight packets";
        return false;
    }
    bool LookUpChainVersion(uint32_t, const eval::OpLookup &,
                            const RigExecTypeDesc &, void *, std::string *error)
    {
        *error = "no point chains";
        return false;
    }
    bool LookUpWeightField(uint32_t, const eval::OpLookup &,
                           const RigExecTypeDesc &, void *, std::string *error)
    {
        *error = "no weight fields";
        return false;
    }
    bool StoreWeightField(const eval::OpHostOutput &, const void *,
                          std::string *error)
    {
        *error = "no weight fields";
        return false;
    }
    bool StoreGroup(const eval::OpHostOutput &output, uint32_t g,
                    const void *elements, size_t count, std::string *error)
    {
        if (count != size_t(bounds[g + 1] - bounds[g])) {
            *error = "group " + std::to_string(g) + " holds " +
                     std::to_string(count) + " points";
            return false;
        }
        const auto *points = static_cast<const GfVec3f *>(elements);
        std::copy(points, points + count,
                  stored[output.object].begin() + bounds[g]);
        ++stores[output.object];
        return true;
    }
};

template <class T>
eval::OpConstant
ConstantOf(uint32_t node, uint32_t port, const T *data, size_t count)
{
    eval::OpConstant constant;
    constant.node = node;
    constant.port = port;
    const auto *bytes = reinterpret_cast<const uint8_t *>(data);
    constant.bytes.assign(bytes, bytes + count * sizeof(T));
    return constant;
}

void
TestHostLookupsAndGroupOutputs()
{
    using namespace rigExecOps;
    RigExecRigDescBuilder builder;
    const OpRig rig = MakeOps(false, &builder);
    const RigExecOutputId deformOut =
        builder.BindOutput(rig.deform, MatrixDeformOut());
    std::string why;
    const auto desc = builder.Build(&why);
    CHECK(desc);
    if (!desc)
        return;

    VtVec3fArray points(kPoints);
    for (size_t i = 0; i < kPoints; ++i)
        points[i] = GfVec3f(float(i % 100), float(i / 100), float(i % 7));
    VtIntArray counts, faces;
    GridFaces(kPoints, &counts, &faces);
    const GfMatrix4d offset = Offset();
    const float one = 1.0f;

    GroupHost host;
    host.matrix.SetTranslate(GfVec3d(0.5, -1.0, 2.0));
    host.bounds = {0, 2500, 5000, 7500, int(kPoints)};
    for (size_t g = 0; g + 1 < host.bounds.size(); ++g)
        host.groups.emplace_back(points.cbegin() + host.bounds[g],
                                 points.cbegin() + host.bounds[g + 1]);
    host.stored.assign(2, VtVec3fArray(kPoints));
    host.stores.assign(2, 0);

    const auto layOut = [&](eval::OpStepLayout *layout) {
        RigExecCompileOptions options;
        options.hostOrder = true;
        layout->program = RigExecCompile(*desc, options, &why);
        layout->hostInputs = {
            {rig.math, MatrixIn().Index(), eval::OpHostSource::FinalMatrix, 0},
            {rig.deform, MatrixDeformPoints().Index(),
             eval::OpHostSource::ChainPoints, 0},
            {rig.smooth, SmoothStrength().Index(), eval::OpHostSource::Lookup,
             0}};
        layout->lookups = {{uint8_t(eval::OpLookupSource::Property), 0, -1, 0}};
        layout->hostOutputs = {{rig.smooth, SmoothOut().Index(),
                                uint8_t(eval::OpOutputTarget::ChainGroup), 0},
                               {rig.deform, MatrixDeformOut().Index(),
                                uint8_t(eval::OpOutputTarget::ChainGroup), 1}};
        layout->constants = {
            ConstantOf(rig.math, MatrixValue().Index(), offset.data(), 16),
            ConstantOf(rig.math, MatrixWeight().Index(), &one, 1),
            ConstantOf(rig.smooth, SmoothCounts().Index(), counts.cdata(),
                       counts.size()),
            ConstantOf(rig.smooth, SmoothIndices().Index(), faces.cdata(),
                       faces.size())};
        layout->chains.assign(RigExecProgramChainCount(*layout->program),
                              eval::OpChainLayout());
        for (eval::OpChainLayout &chain : layout->chains)
            chain.bounds = host.bounds;
    };

    eval::OpStepLayout layout;
    layOut(&layout);
    CHECK(eval::LayOutOpSteps(&layout, &why));
    CHECK(layout.chains.size() == 1);
    eval::OpStepState state;
    state.run = RigExecRunState::Create(layout.program, {}, &why);
    CHECK(state.run && eval::BindOpStepState(layout, host, &state, &why));
    if (!state.run || layout.chains.size() != 1)
        return;
    bool ran = true;
    for (size_t n = 0; n < layout.nodes.size(); ++n)
        if (layout.chainOf[n] == kRigExecInvalidIndex)
            ran = ran &&
                  eval::RunOpStep(layout, host, &state, eval::OpStepKind::Node,
                                  int(n), -1, &why);
    ran = ran && eval::RunOpStep(layout, host, &state,
                                 eval::OpStepKind::ChainHead, 0, -1, &why);
    CHECK(host.lookups == 1);
    const RigExecChainDesc &chain = layout.chainDescs[0];
    const uint32_t groups = layout.chains[0].Groups();
    for (uint32_t s = 0; s < chain.members.size(); ++s)
        if (chain.segments[s] == s)
            for (uint32_t g = 0; g < (chain.whole[s] ? 1 : groups); ++g)
                ran = ran && eval::RunOpStep(layout, host, &state,
                                             eval::OpStepKind::ChainRange, 0,
                                             int(s * groups + g), &why);
    if (!ran)
        std::printf("host steps: %s\n", why.c_str());
    CHECK(ran);
    CHECK(host.stores[0] == int(groups) && host.stores[1] == int(groups));

    auto evaluator = RigExecEvaluator::Create(RigExecCompile(*desc));
    const rigExecStd::RigExecArray<rigExecStd::Vec3f> in(
        reinterpret_cast<const rigExecStd::Vec3f *>(points.cdata()),
        points.size());
    const rigExecStd::RigExecArray<int32_t> c(counts.cdata(), counts.size());
    const rigExecStd::RigExecArray<int32_t> f(faces.cdata(), faces.size());
    rigExecStd::Mat4d matrix, value;
    std::memcpy(&matrix, host.matrix.data(), sizeof(matrix));
    std::memcpy(&value, offset.data(), sizeof(value));
    evaluator->SetInput(rig.matrix, &matrix);
    evaluator->SetInput(rig.value, &value);
    evaluator->SetInput(rig.weight, &one);
    evaluator->SetInput(rig.points, &in);
    evaluator->SetInput(rig.counts, &c);
    evaluator->SetInput(rig.indices, &f);
    evaluator->SetInput(rig.strength, &host.strength);
    CHECK(evaluator->Evaluate(&why));
    const auto same = [](const void *output, const VtVec3fArray &stored) {
        const auto *array =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                output);
        return array && array->Size() == stored.size() &&
               std::memcmp(array->Data(), stored.cdata(),
                           stored.size() * sizeof(GfVec3f)) == 0;
    };
    CHECK(same(evaluator->GetOutput(rig.out), host.stored[0]));
    CHECK(same(evaluator->GetOutput(deformOut), host.stored[1]));

    const auto refused = [&](auto &&change, const char *expected) {
        eval::OpStepLayout bad;
        layOut(&bad);
        change(&bad);
        std::string reason;
        const bool laid = eval::LayOutOpSteps(&bad, &reason);
        if (laid || reason.find(expected) == std::string::npos)
            std::printf("expected \"%s\", got \"%s\"\n", expected,
                        reason.c_str());
        CHECK(!laid && reason.find(expected) != std::string::npos);
    };
    refused([](eval::OpStepLayout *bad) { bad->lookups[0].source = 10; },
            "reads lookup source 10, which no host resolves");
    refused(
        [&](eval::OpStepLayout *bad) {
            bad->hostInputs[1].source = eval::OpHostSource::Lookup;
        },
        "a lookup feeds point input");
    refused(
        [](eval::OpStepLayout *bad) {
            bad->hostOutputs[0].target =
                uint8_t(eval::OpOutputTarget::PropertyVersion);
        },
        "in output target 0, which no host stores");
    refused(
        [&](eval::OpStepLayout *bad) {
            bad->hostOutputs.push_back(
                {rig.math, MatrixOut().Index(),
                 uint8_t(eval::OpOutputTarget::ChainGroup), 2});
        },
        "in output target 4, which no host stores");
}

/// The groups of \p points at \p bounds, each a buffer of its own.
std::vector<VtVec3fArray>
GroupsOf(const VtVec3fArray &points, const std::vector<int> &bounds)
{
    std::vector<VtVec3fArray> groups;
    for (size_t g = 0; g + 1 < bounds.size(); ++g)
        groups.emplace_back(points.cbegin() + bounds[g],
                            points.cbegin() + bounds[g + 1]);
    return groups;
}

/// A chain whose first member, a smooth, reads a host chain version or the
/// chain's points through its neighbourhood input and whose skin reads a
/// list of looked-up base matrices: every range step hands its smooth the
/// halo groups as the host holds them now, so a frame whose groups moved
/// to new buffers, with no head between, matches the plain evaluator.
void
TestHaloVersionsAndLists()
{
    using namespace rigExecOps;
    RigExecRigDescBuilder b;
    SkinPayload skin;
    const RigExecNodeId smooth = b.AddOp(Smooth());
    const RigExecNodeId skinned = b.AddOp(Skin(), &skin);
    b.Connect(smooth, SmoothOut(), skinned, SkinPoints());
    const RigExecInputId in = b.BindInput(smooth, SmoothPoints());
    const RigExecInputId countsIn = b.BindInput(smooth, SmoothCounts());
    const RigExecInputId facesIn = b.BindInput(smooth, SmoothIndices());
    const RigExecInputId strengthIn = b.BindInput(smooth, SmoothStrength());
    const RigExecInputId transformsIn = b.BindInput(skinned, SkinTransforms());
    const RigExecInputId jointsIn = b.BindInput(skinned, SkinIndices());
    const RigExecInputId weightsIn = b.BindInput(skinned, SkinWeights());
    const RigExecOutputId out = b.BindOutput(skinned, SkinOut());
    std::string why;
    const auto desc = b.Build(&why);
    CHECK(desc);
    if (!desc)
        return;

    VtIntArray counts, faces;
    GridFaces(kPoints, &counts, &faces);
    VtIntArray joints(kPoints);
    VtFloatArray weights(kPoints, 1.0f);
    for (size_t i = 0; i < kPoints; ++i)
        joints[i] = int(i % 2);
    const float half = 0.5f;
    const auto pointsAt = [](float scale) {
        VtVec3fArray points(kPoints);
        for (size_t i = 0; i < kPoints; ++i)
            points[i] =
                GfVec3f(float(i % 100), float(i / 100), scale * float(i % 7));
        return points;
    };
    GfMatrix4d second;
    second.SetTranslate(GfVec3d(0.0, 2.0, -1.0));

    const auto reference = [&](const VtVec3fArray &points) {
        auto evaluator = RigExecEvaluator::Create(RigExecCompile(*desc));
        const rigExecStd::RigExecArray<rigExecStd::Vec3f> p(
            reinterpret_cast<const rigExecStd::Vec3f *>(points.cdata()),
            points.size());
        const rigExecStd::RigExecArray<int32_t> c(counts.cdata(),
                                                  counts.size());
        const rigExecStd::RigExecArray<int32_t> f(faces.cdata(), faces.size());
        const rigExecStd::RigExecArray<int32_t> j(joints.cdata(),
                                                  joints.size());
        const rigExecStd::RigExecArray<float> w(weights.cdata(),
                                                weights.size());
        std::vector<rigExecStd::Mat4d> matrices(2);
        std::memcpy(&matrices[0], GfMatrix4d(1.0).data(), sizeof(matrices[0]));
        std::memcpy(&matrices[1], second.data(), sizeof(matrices[1]));
        const rigExecStd::RigExecArray<rigExecStd::Mat4d> t(matrices.data(),
                                                            matrices.size());
        evaluator->SetInput(in, &p);
        evaluator->SetInput(countsIn, &c);
        evaluator->SetInput(facesIn, &f);
        evaluator->SetInput(strengthIn, &half);
        evaluator->SetInput(transformsIn, &t);
        evaluator->SetInput(jointsIn, &j);
        evaluator->SetInput(weightsIn, &w);
        VtVec3fArray result;
        std::string error;
        if (!evaluator->Evaluate(&error)) {
            std::printf("reference: %s\n", error.c_str());
            return result;
        }
        const auto *points3 =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                evaluator->GetOutput(out));
        result.assign(reinterpret_cast<const GfVec3f *>(points3->Data()),
                      reinterpret_cast<const GfVec3f *>(points3->Data()) +
                          points3->Size());
        return result;
    };

    for (const bool version : {false, true}) {
        GroupHost host;
        host.bounds = {0, 2500, 5000, 7500, int(kPoints)};
        host.bases = {GfMatrix4d(1.0), second};
        host.versions = {GroupsOf(pointsAt(9.0f), host.bounds),
                         GroupsOf(pointsAt(1.0f), host.bounds)};
        host.groups = host.versions[1];
        host.stored.assign(1, VtVec3fArray(kPoints));
        host.stores.assign(1, 0);

        eval::OpStepLayout layout;
        RigExecCompileOptions options;
        options.hostOrder = true;
        layout.program = RigExecCompile(*desc, options, &why);
        layout.lookups = {
            {uint8_t(eval::OpLookupSource::List), 1, 2, 0},
            {uint8_t(eval::OpLookupSource::Matrix), 0, 1, 0},
            {uint8_t(eval::OpLookupSource::Matrix), 1, 1, 0},
            {uint8_t(eval::OpLookupSource::ChainVersion), 0, 1, 0}};
        layout.hostInputs = {
            version ? eval::OpHostInput{smooth, SmoothPoints().Index(),
                                        eval::OpHostSource::Lookup, 3}
                    : eval::OpHostInput{smooth, SmoothPoints().Index(),
                                        eval::OpHostSource::ChainPoints, 0},
            {skinned, SkinTransforms().Index(), eval::OpHostSource::Lookup, 0}};
        layout.hostOutputs = {{skinned, SkinOut().Index(),
                               uint8_t(eval::OpOutputTarget::ChainGroup), 0}};
        layout.constants = {
            ConstantOf(smooth, SmoothCounts().Index(), counts.cdata(),
                       counts.size()),
            ConstantOf(smooth, SmoothIndices().Index(), faces.cdata(),
                       faces.size()),
            ConstantOf(smooth, SmoothStrength().Index(), &half, 1),
            ConstantOf(skinned, SkinIndices().Index(), joints.cdata(),
                       joints.size()),
            ConstantOf(skinned, SkinWeights().Index(), weights.cdata(),
                       weights.size())};
        layout.chains.assign(RigExecProgramChainCount(*layout.program),
                             eval::OpChainLayout());
        for (eval::OpChainLayout &chain : layout.chains)
            chain.bounds = host.bounds;
        const bool laid = eval::LayOutOpSteps(&layout, &why);
        if (!laid)
            std::printf("halo layout: %s\n", why.c_str());
        CHECK(laid && layout.chains.size() == 1 && layout.grouped[0] == 2);
        if (!laid || layout.chains.size() != 1)
            return;
        eval::OpStepState state;
        state.run = RigExecRunState::Create(layout.program, {}, &why);
        const bool bound =
            state.run && eval::BindOpStepState(layout, host, &state, &why);
        if (!bound)
            std::printf("halo bind: %s\n", why.c_str());
        CHECK(bound);
        if (!bound)
            return;
        const RigExecChainDesc &chain = layout.chainDescs[0];
        const uint32_t groups = layout.chains[0].Groups();
        const auto ranges = [&]() {
            bool ran = true;
            for (uint32_t s = 0; s < chain.members.size(); ++s)
                if (chain.segments[s] == s)
                    for (uint32_t g = 0; g < (chain.whole[s] ? 1 : groups); ++g)
                        ran = ran &&
                              eval::RunOpStep(layout, host, &state,
                                              eval::OpStepKind::ChainRange, 0,
                                              int(s * groups + g), &why);
            if (!ran)
                std::printf("halo ranges: %s\n", why.c_str());
            return ran;
        };
        CHECK(eval::RunOpStep(layout, host, &state, eval::OpStepKind::ChainHead,
                              0, -1, &why));
        CHECK(state.haloReads[0].size() == groups &&
              state.haloReads[0][1].size() == 3);
        CHECK(ranges());
        CHECK(SameBits(host.stored[0], reference(pointsAt(1.0f))));

        const std::vector<std::vector<VtVec3fArray>> held = host.versions;
        host.versions[1] = GroupsOf(pointsAt(3.0f), host.bounds);
        host.groups = host.versions[1];
        CHECK(ranges());
        CHECK(SameBits(host.stored[0], reference(pointsAt(3.0f))));
        CHECK(!SameBits(host.stored[0], reference(pointsAt(1.0f))));

        eval::OpStepLayout bad = layout;
        bad.lookups[0].part = 4;
        std::string reason;
        CHECK(!eval::LayOutOpSteps(&bad, &reason) &&
              Contains(reason, "reads lookup list 0, whose elements"));
        bad = layout;
        bad.chains[0].writes.assign(3, 1);
        CHECK(!eval::LayOutOpSteps(&bad, &reason) &&
              Contains(reason, "names the groups its members write in 3"));
        bad = layout;
        bad.lookups[0].source = uint8_t(eval::OpLookupSource::Selected);
        bad.lookups[0].object = 3;
        bad.lookups[0].part = 1;
        CHECK(!eval::LayOutOpSteps(&bad, &reason) &&
              Contains(reason, "which answer no matrix"));
    }
}

/// A Selected lookup answers its first present candidate: a valid frame
/// record, else a measured constraint delta, else the matrix table.
void
TestSelectedMatrix()
{
    GroupHost host;
    GfMatrix4d record, delta, table;
    record.SetTranslate(GfVec3d(1.0, 0.0, 0.0));
    delta.SetTranslate(GfVec3d(0.0, 2.0, 0.0));
    table.SetTranslate(GfVec3d(0.0, 0.0, 3.0));
    host.bases = {GfMatrix4d(1.0), table};
    host.records = {{GfMatrix4d(1.0), false}, {record, true}};
    host.deltas = {{delta, true}};
    eval::OpStepLayout layout;
    layout.lookups = {
        {uint8_t(eval::OpLookupSource::Selected), 1, 4, 0},
        {uint8_t(eval::OpLookupSource::FrameRecord), 0, -1, 0},
        {uint8_t(eval::OpLookupSource::FrameRecord), 1, -1, 0},
        {uint8_t(eval::OpLookupSource::ConstraintDelta), 0, -1, 0},
        {uint8_t(eval::OpLookupSource::Matrix), 1, 1, 0},
        {uint8_t(eval::OpLookupSource::Selected), 3, 2, 0},
        {uint8_t(eval::OpLookupSource::Selected), 1, 1, 0}};
    const RigExecTypeDesc &type =
        *RigExecRegistry::Get().TypeAt(rigExecStd::Mat4dType().Index()).desc;
    const auto select = [&](uint32_t index, GfMatrix4d *out, std::string *why) {
        return eval::OpLookUpAt(layout, host, index, type, out->data(), why);
    };
    GfMatrix4d out;
    std::string why;
    CHECK(select(0, &out, &why) && out == record);
    host.records[1].second = false;
    CHECK(select(0, &out, &why) && out == delta);
    host.deltas[0].second = false;
    CHECK(select(0, &out, &why) && out == table);
    CHECK(select(5, &out, &why) && out == table);
    CHECK(!select(6, &out, &why) && Contains(why, "none of which is present"));
    CHECK(!select(1, &out, &why) &&
          Contains(why, "frame record 0 holds no matrix"));
}

/// An op chain storing the gated mover's groups: it runs and stores only
/// the groups the mover writes, the version of every other group stays the
/// entering one, and a drag reruns only written groups it reaches.
void
TestInjectedGates()
{
    using namespace rigExecOps;
    RigExecNodeId deform = 0;
    RigExecBakedProgramTesting::SetOpInjector(
        [&](const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out) {
            const int chain = ChainOf(B, "/Asset/Shape.points");
            const int slot = SlotOf(B, kMoving);
            if (chain < 0 || slot < 0)
                return false;
            MatrixDeformPayload payload;
            deform = out->builder->AddOp(MatrixDeform(), &payload);
            out->hostInputs = {{deform, MatrixDeformPoints().Index(),
                                eval::OpHostSource::Lookup, 0},
                               {deform, MatrixDeformTransform().Index(),
                                eval::OpHostSource::Lookup, 1}};
            out->lookups = {
                {uint8_t(eval::OpLookupSource::ChainVersion), uint32_t(chain),
                 1, 0},
                {uint8_t(eval::OpLookupSource::Selected), 2, 1, 0},
                {uint8_t(eval::OpLookupSource::Matrix), uint32_t(slot), 1, 0}};
            out->hostOutputs = {
                {deform, MatrixDeformOut().Index(),
                 uint8_t(eval::OpOutputTarget::ChainGroup),
                 uint32_t(B.chainRevisionBegin[size_t(chain)] + 1)}};
            return true;
        });
    const UsdTimeCode time(1.0);
    const UsdStageRefPtr stage = MakeGatedStage(0.5f, 0.0f, 1.0);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    for (const std::string &line : errors)
        std::printf("  %s\n", line.c_str());
    if (!compiled || !evaluator.GetBakedProgram()) {
        RigExecBakedProgramTesting::SetOpInjector(nullptr);
        return;
    }
    const RigExecBakedProgramImpl &B =
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram());
    const int chain = ChainOf(B, "/Asset/Shape.points");
    const int id = B.chainRevisionBegin[size_t(chain)] + 1;
    const std::vector<char> &written =
        B.chains[size_t(chain)].revisions[1].groupWritten;
    const int groups = int(written.size());
    std::set<std::pair<int, int>> writes;
    for (int g = 0; g < groups; ++g)
        if (written[size_t(g)])
            writes.insert({0, g});
    CHECK(!writes.empty() && int(writes.size()) < groups);
    size_t ranges = 0, chunks = 0;
    for (const RigExecBakedStep &step : B.steps) {
        ranges += step.kind == RigExecBakedStepKind::OpChainRange;
        chunks += step.kind == RigExecBakedStepKind::RevisionChunk &&
                  step.object == id;
    }
    CHECK(ranges == writes.size() && chunks == 0);

    const auto reference = [&]() {
        std::vector<GfVec3f> scratch;
        const GfVec3f *points = nullptr;
        size_t count = 0;
        RigExecBakedVersionPoints(B.chains[size_t(chain)], 1, &scratch, &points,
                                  &count);
        VtVec3fArray result(points, points + count);
        RigExecRigDescBuilder b;
        MatrixDeformPayload payload;
        const RigExecNodeId node = b.AddOp(MatrixDeform(), &payload);
        const RigExecInputId in = b.BindInput(node, MatrixDeformPoints());
        const RigExecInputId transform =
            b.BindInput(node, MatrixDeformTransform());
        const RigExecOutputId out = b.BindOutput(node, MatrixDeformOut());
        std::string why;
        auto evaluator =
            RigExecEvaluator::Create(RigExecCompile(*b.Build(&why)));
        const rigExecStd::RigExecArray<rigExecStd::Vec3f> p(
            reinterpret_cast<const rigExecStd::Vec3f *>(points), count);
        rigExecStd::Mat4d matrix;
        std::memcpy(&matrix, B.baseMatrix[size_t(SlotOf(B, kMoving))].data(),
                    sizeof(matrix));
        evaluator->SetInput(in, &p);
        evaluator->SetInput(transform, &matrix);
        std::string error;
        if (!evaluator->Evaluate(&error)) {
            std::printf("reference: %s\n", error.c_str());
            return VtVec3fArray();
        }
        const auto *moved =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                evaluator->GetOutput(out));
        const auto &bounds = B.chains[size_t(chain)].groupBounds;
        for (int g = 0; g < groups; ++g)
            if (written[size_t(g)])
                std::memcpy(result.data() + bounds[size_t(g)],
                            moved->Data() + bounds[size_t(g)],
                            size_t(bounds[size_t(g) + 1] - bounds[size_t(g)]) *
                                sizeof(GfVec3f));
        return result;
    };
    const auto shape = [](const RigExecRigPose &pose) {
        const auto found =
            pose.movedProperties.find(SdfPath("/Asset/Shape.points"));
        return found != pose.movedProperties.end() &&
                       found->second.IsHolding<VtVec3fArray>()
                   ? found->second.UncheckedGet<VtVec3fArray>()
                   : VtVec3fArray();
    };
    const auto edit = [&](float raise, float lift, double moving) {
        evaluator.SetInteractiveOverrides(
            {Override(kRaise, "inputs:weight", VtValue(raise)),
             Override(kLift, "inputs:weight", VtValue(lift)),
             Override(kMoving, "avars:tx", VtValue(moving))});
        return evaluator.Evaluate(time);
    };
    const auto report = [&](const char *what,
                            const std::set<std::pair<int, int>> &ran) {
        std::printf("%s reran:", what);
        for (const auto &[segment, g] : ran)
            std::printf(" (%d, %d)", segment, g);
        std::printf("\n");
    };
    const RigExecRigPose first = edit(0.5f, 0.0f, 1.0);
    CHECK(first.valid && SameBits(shape(first), reference()));
    CHECK(RanRanges(B, groups) == writes);
    for (int g = 0; g < groups; ++g) {
        const RigExecGroupSource at =
            RigExecBakedGroupSourceAt(B, chain, 2, size_t(g));
        const RigExecGroupSource entering =
            RigExecBakedGroupSourceAt(B, chain, 1, size_t(g));
        CHECK(written[size_t(g)]
                  ? at.slot == RigExecBakedGroupSlot(B, id, size_t(g))
                  : at.slot == entering.slot && at.version == entering.version);
    }

    const RigExecRigPose moved = edit(0.5f, 0.0f, 2.0);
    if (RanRanges(B, groups) != writes)
        report("move", RanRanges(B, groups));
    CHECK(RanRanges(B, groups) == writes);
    CHECK(moved.valid && SameBits(shape(moved), reference()) &&
          !SameBits(shape(moved), shape(first)));
    const RigExecRigPose raised = edit(1.0f, 0.0f, 2.0);
    if (RanRanges(B, groups) != writes)
        report("raise", RanRanges(B, groups));
    CHECK(RanRanges(B, groups) == writes);
    CHECK(raised.valid && SameBits(shape(raised), reference()));
    const RigExecRigPose lifted = edit(1.0f, 0.5f, 2.0);
    const std::set<std::pair<int, int>> one = {{0, 1}};
    if (RanRanges(B, groups) != one)
        report("lift", RanRanges(B, groups));
    CHECK(RanRanges(B, groups) == one);
    CHECK(lifted.valid && SameBits(shape(lifted), reference()));

    evaluator.SetInteractiveOverrides({});
    const RigExecRigPose live = evaluator.Evaluate(time);
    const RigExecRigPose warmed = Frozen(evaluator, time);
    CHECK(live.valid && warmed.valid && SameBits(shape(warmed), shape(live)) &&
          SameBits(shape(live), reference()));
    RigExecBakedProgramTesting::SetOpInjector(nullptr);
}

/// The halo chain on a real rig: two smooths, the first reading the points
/// entering the gated mover (version 1) group by group through its
/// neighbourhood input. Lifting one group reruns that group's halo only;
/// moving the gated mover, which that version precedes, reruns nothing;
/// native, frame-cache and runtime evaluation agree. A group stored into
/// the mover's own state passes the entering group through when it holds
/// its bits and moves its version only when its bits move.
void
TestInjectedHalo()
{
    using namespace rigExecOps;
    struct Halo {
        RigExecInputId points, counts[2], faces[2], strengths[2];
        RigExecOutputId out;
    };
    const auto make = [](RigExecRigDescBuilder &b, Halo *halo) {
        const RigExecNodeId first = b.AddOp(Smooth());
        const RigExecNodeId next = b.AddOp(Smooth());
        b.Connect(first, SmoothOut(), next, SmoothPoints());
        halo->points = b.BindInput(first, SmoothPoints());
        const RigExecNodeId nodes[2] = {first, next};
        for (int i = 0; i < 2; ++i) {
            halo->counts[i] = b.BindInput(nodes[i], SmoothCounts());
            halo->faces[i] = b.BindInput(nodes[i], SmoothIndices());
            halo->strengths[i] = b.BindInput(nodes[i], SmoothStrength());
        }
        halo->out = b.BindOutput(next, SmoothOut());
        return std::pair<RigExecNodeId, RigExecNodeId>(first, next);
    };
    RigExecBakedProgramTesting::SetOpInjector(
        [&](const RigExecBakedProgramImpl &B, RigExecBakedOpInjection *out) {
            const int chain = ChainOf(B, "/Asset/Shape.points");
            if (chain < 0)
                return false;
            Halo halo;
            const auto [first, next] = make(*out->builder, &halo);
            out->hostInputs = {
                {first, SmoothPoints().Index(), eval::OpHostSource::Lookup, 0}};
            out->lookups = {{uint8_t(eval::OpLookupSource::ChainVersion),
                             uint32_t(chain), 1, 0}};
            VtIntArray counts, faces;
            GridFaces(B.chains[size_t(chain)].groupPointCount, &counts, &faces);
            for (const RigExecNodeId node : {first, next}) {
                out->constants.push_back(
                    {node, SmoothCounts().Index(), VtValue(counts)});
                out->constants.push_back(
                    {node, SmoothIndices().Index(), VtValue(faces)});
                out->constants.push_back(
                    {node, SmoothStrength().Index(), VtValue(0.5f)});
            }
            out->outputs = {{"smoothed", halo.out}};
            return true;
        });
    const UsdTimeCode time(1.0);
    const UsdStageRefPtr stage = MakeGatedStage(0.5f, 0.0f, 1.0);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    for (const std::string &line : errors)
        std::printf("  %s\n", line.c_str());
    if (!compiled || !evaluator.GetBakedProgram()) {
        RigExecBakedProgramTesting::SetOpInjector(nullptr);
        return;
    }
    const RigExecBakedProgramImpl &B =
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram());
    const int chain = ChainOf(B, "/Asset/Shape.points");
    const int groups = int(B.chains[size_t(chain)].groupBounds.size()) - 1;
    const auto reference = [&]() {
        std::vector<GfVec3f> scratch;
        const GfVec3f *points = nullptr;
        size_t count = 0;
        RigExecBakedVersionPoints(B.chains[size_t(chain)], 1, &scratch, &points,
                                  &count);
        Halo halo;
        RigExecRigDescBuilder b;
        make(b, &halo);
        std::string why;
        auto evaluator =
            RigExecEvaluator::Create(RigExecCompile(*b.Build(&why)));
        const rigExecStd::RigExecArray<rigExecStd::Vec3f> p(
            reinterpret_cast<const rigExecStd::Vec3f *>(points), count);
        VtIntArray counts, faces;
        GridFaces(count, &counts, &faces);
        const rigExecStd::RigExecArray<int32_t> c(counts.cdata(),
                                                  counts.size());
        const rigExecStd::RigExecArray<int32_t> f(faces.cdata(), faces.size());
        const float half = 0.5f;
        evaluator->SetInput(halo.points, &p);
        for (int i = 0; i < 2; ++i) {
            evaluator->SetInput(halo.counts[i], &c);
            evaluator->SetInput(halo.faces[i], &f);
            evaluator->SetInput(halo.strengths[i], &half);
        }
        VtVec3fArray result;
        std::string error;
        if (!evaluator->Evaluate(&error)) {
            std::printf("reference: %s\n", error.c_str());
            return result;
        }
        const auto *out =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                evaluator->GetOutput(halo.out));
        result.assign(reinterpret_cast<const GfVec3f *>(out->Data()),
                      reinterpret_cast<const GfVec3f *>(out->Data()) +
                          out->Size());
        return result;
    };
    const auto edit = [&](float lift, double moving) {
        evaluator.SetInteractiveOverrides(
            {Override(kLift, "inputs:weight", VtValue(lift)),
             Override(kMoving, "avars:tx", VtValue(moving))});
        return evaluator.Evaluate(time);
    };
    const RigExecRigPose first = edit(0.0f, 1.0);
    CHECK(first.valid && Smoothed(first) &&
          SameBits(*Smoothed(first), reference()));
    const RigExecRigPose lifted = edit(0.5f, 1.0);
    const std::set<std::pair<int, int>> ran = RanRanges(B, groups);
    const std::set<std::pair<int, int>> wanted = {
        {0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {1, 3}};
    if (ran != wanted) {
        std::printf("halo lift reran:");
        for (const auto &[segment, g] : ran)
            std::printf(" (%d, %d)", segment, g);
        std::printf("\n");
    }
    CHECK(ran == wanted);
    CHECK(lifted.valid && Smoothed(lifted) &&
          SameBits(*Smoothed(lifted), reference()) &&
          !SameBits(*Smoothed(lifted), *Smoothed(first)));
    const RigExecRigPose moved = edit(0.5f, 2.0);
    CHECK(moved.valid && RanRanges(B, groups).empty());
    CHECK(Smoothed(moved) && SameBits(*Smoothed(moved), *Smoothed(lifted)));

    evaluator.SetInteractiveOverrides({});
    const RigExecRigPose live = evaluator.Evaluate(time);
    const RigExecRigPose warmed = Frozen(evaluator, time);
    CHECK(warmed.valid && Smoothed(warmed) && Smoothed(live) &&
          SameBits(*Smoothed(warmed), *Smoothed(live)));

    std::vector<uint8_t> bytes;
    std::string error;
    RigExecTestPlayer player;
    const bool opened = RigExecTestBakeAt(evaluator, 1.0, &bytes, &error) &&
                        player.Open(bytes, stage, &error);
    if (!opened)
        std::printf("halo bake or open: %s\n", error.c_str());
    CHECK(opened);
    for (const float lift : {0.0f, 0.5f}) {
        if (!opened)
            break;
        const RigExecRigPose pose = edit(lift, 2.0);
        CHECK(player.Hold(std::string(kLift) + ".inputs:weight", lift, &error));
        CHECK(player.Hold(std::string(kMoving) + ".avars:tx", 2.0, &error));
        CHECK(player.Play(1.0, &error));
        const VtVec3fArray played = RuntimeOutput(player.Reader(), "smoothed");
        CHECK(pose.valid && Smoothed(pose) && !played.empty() &&
              SameBits(*Smoothed(pose), played));
    }

    // The gated mover's own groups, stored as an op chain stores them.
    auto &program = const_cast<RigExecBakedProgramImpl &>(
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram()));
    const int id = program.chainRevisionBegin[size_t(chain)] + 1;
    auto &revision = program.chains[size_t(chain)].revisions[1];
    size_t g = 0;
    while (g < revision.groupWritten.size() && !revision.groupWritten[g])
        ++g;
    CHECK(g < revision.groupWritten.size());
    size_t unwritten = 0;
    while (unwritten < revision.groupWritten.size() &&
           revision.groupWritten[unwritten])
        ++unwritten;
    if (g < revision.groupWritten.size()) {
        const eval::OpHostOutput output{
            0, 0, uint8_t(eval::OpOutputTarget::ChainGroup), uint32_t(id)};
        const RigExecPointsRef<GfVec3f> entering =
            RigExecBakedGroupAt(program.chains[size_t(chain)], 1, g);
        CHECK(entering.count > 0);
        std::string why;
        CHECK(RigExecBakedStoreChainGroup(program, output, uint32_t(g),
                                          entering.data, entering.count, &why));
        CHECK(revision.groups[g].published.data == entering.data);
        std::vector<GfVec3f> moved3(entering.data,
                                    entering.data + entering.count);
        moved3.resize(std::max<size_t>(moved3.size(), 1));
        moved3[0][0] += 1.0f;
        CHECK(RigExecBakedStoreChainGroup(program, output, uint32_t(g),
                                          moved3.data(), moved3.size(), &why));
        const uint64_t version = revision.groups[g].version;
        CHECK(revision.groups[g].published.data != entering.data &&
              revision.groups[g].published.data != moved3.data() &&
              revision.groups[g].published.count == moved3.size() &&
              std::memcmp(revision.groups[g].published.data, moved3.data(),
                          moved3.size() * sizeof(GfVec3f)) == 0);
        CHECK(RigExecBakedStoreChainGroup(program, output, uint32_t(g),
                                          moved3.data(), moved3.size(), &why));
        CHECK(revision.groups[g].version == version);
        CHECK(!RigExecBakedStoreChainGroup(program, output, uint32_t(g),
                                           moved3.data(), 3, &why) &&
              Contains(why, "is given 3 points for group"));
        if (unwritten < revision.groupWritten.size())
            CHECK(!RigExecBakedStoreChainGroup(
                      program, output, uint32_t(unwritten), moved3.data(),
                      moved3.size(), &why) &&
                  Contains(why, "writes no group"));
    }
    RigExecBakedProgramTesting::SetOpInjector(nullptr);
}

} // namespace

/// With --write FILE the baked file is also written to FILE; with
/// --unregistered FILE only that file is opened, before any registry exists.
int
main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const char *writeTo = nullptr;
    if (argc == 3 && std::strcmp(argv[1], "--unregistered") == 0) {
        TestUnregistered(argv[2]);
        std::printf(failures ? "%d failure(s)\n" : "ok\n", failures);
        return failures ? 1 : 0;
    }
    if (argc == 3 && std::strcmp(argv[1], "--write") == 0)
        writeTo = argv[2];
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    std::string error;
    if (!RigExecEnsureBuiltinRegistry(&error)) {
        std::printf("registry: %s\n", error.c_str());
        return 1;
    }
    TestInjectedChain();
    TestRuntime(writeTo);
    TestFailingNode();
    TestHostLookupsAndGroupOutputs();
    TestHaloVersionsAndLists();
    TestInjectedHalo();
    TestSelectedMatrix();
    TestInjectedGates();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
