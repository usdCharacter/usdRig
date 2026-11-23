// Weight objects publish one shared packet per generation. Ordinary mover
// packets use the BASE pose placement and packet validity policy. Current
// point fields and scalar envelopes use the diagnostic field policy and their
// explicitly bound placement phase. Both scheduled adapters use EvalCore.
#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedSlotDomains.h"

#include "rigExec/api/types.h"
#include "rigExec/internal/weightPackets.h"
#include "weightPackets.h"
#include "parallel.h"

#include "rigExecEvalCore/weights.h"
#include "rigExecMath/affineFrameKernels.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <string>
#include <vector>

// The weight object types, spelled once. The per-frame dispatch runs a
// step body down this list, and TfToken(const char *) takes the token
// registry's spin lock on every construction -- which a step body may not
// take (libs/rigExec/internal/bodyPurity.h) -- so the comparison is against
// interned tokens rather than against literals.
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((staticWeight, rigExec::SchemaTypeName::RigExecStaticWeight))
    ((dynamicWeight, rigExec::SchemaTypeName::RigExecDynamicWeight))
    ((combineWeight, rigExec::SchemaTypeName::RigExecCombineWeight))
    ((sphereWeight, rigExec::SchemaTypeName::RigExecSphereWeight))
    ((planeWeight, rigExec::SchemaTypeName::RigExecPlaneWeight))
    ((curveWeight, rigExec::SchemaTypeName::RigExecCurveWeight))
    ((falloffSamples, rigExec::SchemaAttrText::inputsFalloffSamples))
    ((rigExecValues, rigExec::SchemaAttrText::rigExecValues))
    ((rigExecIndices, rigExec::SchemaAttrText::rigExecIndices))
    ((rigExecFalloffProfile, rigExec::SchemaAttrText::rigExecFalloffProfile))
    ((rigExecFalloffCurve, rigExec::SchemaAttrText::rigExecFalloffCurve))
    ((rigExecDefaultWeight, rigExec::SchemaAttrText::rigExecDefaultWeight))
    ((inputsDriver, rigExec::SchemaAttrText::inputsDriver))
    ((inputsScale, rigExec::SchemaAttrText::inputsScale))
    ((inputsBias, rigExec::SchemaAttrText::inputsBias))
    ((inputsStrength, rigExec::SchemaAttrText::inputsStrength))
    ((inputsInvert, rigExec::SchemaAttrText::inputsInvert))
    ((inputsFalloffMin, rigExec::SchemaAttrText::inputsFalloffMin))
    ((inputsFalloffMax, rigExec::SchemaAttrText::inputsFalloffMax))
    ((inputsScaleXPos, rigExec::SchemaAttrText::inputsScaleXPos))
    ((inputsScaleYPos, rigExec::SchemaAttrText::inputsScaleYPos))
    ((inputsScaleZPos, rigExec::SchemaAttrText::inputsScaleZPos))
    ((inputsScaleXNeg, rigExec::SchemaAttrText::inputsScaleXNeg))
    ((inputsScaleYNeg, rigExec::SchemaAttrText::inputsScaleYNeg))
    ((inputsScaleZNeg, rigExec::SchemaAttrText::inputsScaleZNeg))
    ((inputsScaleX, rigExec::SchemaAttrText::inputsScaleX))
    ((inputsScaleY, rigExec::SchemaAttrText::inputsScaleY))
    ((inputsScaleZ, rigExec::SchemaAttrText::inputsScaleZ))
    ((inputsExtentU, rigExec::SchemaAttrText::inputsExtentU))
    ((inputsExtentV, rigExec::SchemaAttrText::inputsExtentV))
    ((rigExecBaseWeight, rigExec::SchemaAttrText::rigExecBaseWeight))
    ((rigExecInputWeights, rigExec::SchemaAttrText::rigExecInputWeights))
    ((rigExecRepresentation, rigExec::SchemaAttrText::rigExecRepresentation))
    (constant)
    ((rigExecRangePolicy, rigExec::SchemaAttrText::rigExecRangePolicy))
    (strict)
    ((rigExecCombineMode, rigExec::SchemaAttrText::rigExecCombineMode))
    ((rigExecPlaneAxis, rigExec::SchemaAttrText::rigExecPlaneAxis))
    (x)
    (y)
    (z)
    ((rigExecPlaneBounds, rigExec::SchemaAttrText::rigExecPlaneBounds))
    (bounded)
    (unbounded)
    ((rigExecWeightTarget, rigExec::SchemaAttrText::rigExecWeightTarget))
    ((rigExecSampleSource, rigExec::SchemaAttrText::rigExecSampleSource))
    ((rigExecCurve, rigExec::SchemaAttrText::rigExecCurve))
);

namespace rigExec {

void
RigExecBakedWeightsTouchTokens()
{
    (void)_tokens.Get();
}

int
RigExecBakedBakeWeightObject(RigExecBakedBuildContext *ctx, const SdfPath &path)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    if (path.IsEmpty()) {
        return -1;
    }
    // Allocate identity on entry so authored back-edges survive discovery.
    // Canonical graph compilation owns SCC diagnosis and exclusion.
    const auto seen = B.weightIndex.find(path);
    if (seen != B.weightIndex.end())
        return seen->second;
    const UsdPrim prim = B.stage->GetPrimAtPath(path);
    if (!prim) {
        // No marker left behind: there is nothing to recurse into, so a
        // second bind of the same missing prim should say so again rather
        // than be reported as a cycle.
        ctx->Refuse("weight object prim is missing", path);
        return -1;
    }
    const int index = int(B.weightObjects.size());
    B.weightIndex[path] = index;
    B.weightObjects.emplace_back();

    RigExecBakedProgramImpl::WeightObject object;
    object.path = path;
    object.type = prim.GetTypeName();

    // Composition first, and depth first: the table is in dependency order,
    // so everything this object reads has to be in it already. The authored
    // order of rigExec:inputWeights is kept -- subtract and overlay are
    // order dependent.
    object.base = RigExecBakedBakeWeightObject(ctx, [&]() {
        const SdfPathVector targets =
            ctx->Targets(prim, _tokens->rigExecBaseWeight);
        return targets.empty() ? SdfPath() : targets[0];
    }());
    for (const SdfPath &input :
         ctx->Targets(prim, _tokens->rigExecInputWeights)) {
        const int index = RigExecBakedBakeWeightObject(ctx, input);
        if (index >= 0) {
            object.inputs.push_back(index);
        }
    }

    // The structural tokens are read the way the dynamic path reads them --
    // plainly, with no time and no resolution walk -- and land in `folded`,
    // so an edit to one rebuilds the program rather than being missed.
    object.representation =
        ctx->ReadToken(prim, _tokens->rigExecRepresentation, _tokens->constant);
    object.rangePolicy =
        ctx->ReadToken(prim, _tokens->rigExecRangePolicy, _tokens->strict);
    object.combineMode =
        ctx->ReadToken(prim, _tokens->rigExecCombineMode, TfToken());

    // The painted arrays. Uniform by schema, so they fold; still named, so a
    // resync that authors one is seen.
    ctx->Fold(prim, _tokens->rigExecValues);
    if (const UsdAttribute a = prim.GetAttribute(_tokens->rigExecValues)) {
        VtFloatArray values;
        a.Get(&values);
        object.values.assign(values.begin(), values.end());
    }
    ctx->Fold(prim, _tokens->rigExecIndices);
    if (const UsdAttribute a = prim.GetAttribute(_tokens->rigExecIndices)) {
        VtIntArray indices;
        a.Get(&indices);
        object.indices.assign(indices.begin(), indices.end());
    }

    // The per-frame inputs. Every one goes through Bind, which registers it
    // for interactive-override placement and records what a notice would
    // have to touch -- that is what makes IsInvalidatedBy and SetOverrides
    // correct here with no code of their own.
    object.defaultWeight = ctx->Bind(prim, _tokens->rigExecDefaultWeight, 0.0f);
    object.driver = ctx->Bind(prim, _tokens->inputsDriver, 1.0f);
    object.scale = ctx->Bind(prim, _tokens->inputsScale, 1.0f);
    object.bias = ctx->Bind(prim, _tokens->inputsBias, 0.0f);
    object.strength = ctx->Bind(prim, _tokens->inputsStrength, 1.0f);
    object.invert = ctx->Bind(prim, _tokens->inputsInvert, 0.0f);

    // The volumetric three, which are the only weight objects with a
    // PLACEMENT: they are RigExecXformables, so the pose walk composes them
    // into a provider slot like a joint, and the field is generated about
    // that frame.
    // The array-bearing relationships, as the properties exec would have
    // reached: authored target order, nothing inferred. See
    // WeightObject::targetPoints for why authored and not canonicalized.
    const auto attributesOf = [&](const TfToken &name) {
        std::vector<UsdAttribute> out;
        for (const SdfPath &target : ctx->Targets(prim, name)) {
            if (!target.IsPropertyPath()) {
                continue;
            }
            B.named.insert(target);
            B.prims.insert(target.GetPrimPath());
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
                // A terminal opinion of another type cannot contribute to a
                // typed point gather. Keep connected walks and missing values
                // on actual point-array attributes; raw oracle relationship
                // facts retain the target and its unavailable typed value.
                if (a.GetTypeName().GetType() !=
                        SdfValueTypeNames->Float3Array.GetType() &&
                    !a.HasAuthoredConnections()) {
                    continue;
                }
                out.push_back(a);
            }
        }
        return out;
    };

    const bool volumetric = object.type == _tokens->sphereWeight ||
                            object.type == _tokens->planeWeight ||
                            object.type == _tokens->curveWeight;
    if (volumetric) {
        object.providerSlot = ctx->SlotOf(path);
        if (object.providerSlot < 0) {
            ctx->Refuse("volume weight is not a pose provider", path);
        }
        object.falloffMin = ctx->Bind(prim, _tokens->inputsFalloffMin, 0.0f);
        object.falloffMax = ctx->Bind(prim, _tokens->inputsFalloffMax, 1.0f);
        object.scaleXPos = ctx->Bind(prim, _tokens->inputsScaleXPos, 1.0f);
        object.scaleYPos = ctx->Bind(prim, _tokens->inputsScaleYPos, 1.0f);
        object.scaleZPos = ctx->Bind(prim, _tokens->inputsScaleZPos, 1.0f);
        object.scaleXNeg = ctx->Bind(prim, _tokens->inputsScaleXNeg, 1.0f);
        object.scaleYNeg = ctx->Bind(prim, _tokens->inputsScaleYNeg, 1.0f);
        object.scaleZNeg = ctx->Bind(prim, _tokens->inputsScaleZNeg, 1.0f);
        object.scaleX = ctx->Bind(prim, _tokens->inputsScaleX, 1.0f);
        object.scaleY = ctx->Bind(prim, _tokens->inputsScaleY, 1.0f);
        object.scaleZ = ctx->Bind(prim, _tokens->inputsScaleZ, 1.0f);
        object.extentU = ctx->Bind(prim, _tokens->inputsExtentU, 1.0f);
        object.extentV = ctx->Bind(prim, _tokens->inputsExtentV, 1.0f);
        object.planeAxis =
            ctx->ReadToken(prim, _tokens->rigExecPlaneAxis, _tokens->y);
        object.planeBounds = ctx->ReadToken(prim, _tokens->rigExecPlaneBounds,
                                            _tokens->unbounded);
        // The authored profile supplies the default table. A declared sample
        // array can replace it without recompiling the profile during a drag.
        ctx->Fold(prim, _tokens->rigExecFalloffProfile);
        ctx->Fold(prim, _tokens->rigExecFalloffCurve);
        const auto lut = B.falloffLuts.find(path);
        if (lut != B.falloffLuts.end()) {
            object.falloffCurve = lut->second;
        }
        object.targetPoints = attributesOf(_tokens->rigExecWeightTarget);
        object.samplePoints = attributesOf(_tokens->rigExecSampleSource);
        object.curvePoints = attributesOf(_tokens->rigExecCurve);
    } else if (object.type == _tokens->combineWeight) {
        // Read for its SIZE and never for its points: the combine consults
        // its own weight target only when no input is dense enough to carry
        // the cardinality itself.
        for (const SdfPath &target :
             ctx->Targets(prim, _tokens->rigExecWeightTarget)) {
            if (!target.IsPropertyPath()) {
                continue;
            }
            B.named.insert(target);
            B.prims.insert(target.GetPrimPath());
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
                object.combineTargetPoints.push_back(a);
            }
        }
    }

    // What the schedule sizes this step by. The points if the object reads
    // any, else the painted table, else one element.
    object.costElements = std::max<size_t>(object.values.size(), 1);
    for (const std::vector<UsdAttribute> *points :
         {&object.targetPoints, &object.combineTargetPoints}) {
        for (const UsdAttribute &a : *points) {
            VtVec3fArray value;
            if (a.Get(&value, UsdTimeCode::EarliestTime())) {
                object.costElements =
                    std::max(object.costElements, value.size());
            }
        }
    }

    // The point gathers' reads as path leaves, one per attribute in gather
    // order, sampled in the prologue through the generation's resolved
    // inputs: the packet step reads them instead of the stage.
    const auto declare = [&object](const std::vector<UsdAttribute> &points) {
        const size_t begin = object.pointLeaves.decl.keys.size();
        for (const UsdAttribute &a : points) {
            object.pointLeaves.decl.Add(
                {a.GetPath(), RigExecRevisionLeafType::Vec3fArray,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly,
                 VtValue(VtVec3fArray())});
        }
        return begin;
    };
    object.targetLeaves = declare(object.targetPoints);
    object.sampleLeaves = declare(object.samplePoints);
    object.curveLeaves = declare(object.curvePoints);
    object.combineLeaves = declare(object.combineTargetPoints);
    if (volumetric) {
        const SdfPath samples = path.AppendProperty(_tokens->falloffSamples);
        object.falloffSamplesLeaf = int(object.pointLeaves.decl.keys.size());
        object.pointLeaves.decl.Add({samples,
                                     RigExecRevisionLeafType::FloatArray,
                                     RigExecRevisionLeafTime::AtTime,
                                     RigExecRevisionLeafFlavour::ResolvedOnly,
                                     VtValue(VtFloatArray())});
        B.named.insert(samples);
        B.resolvedRoutedPrims.insert(path);
    }
    RigExecBakedBindPathLeaves(B.stage, &object.pointLeaves);

    B.weightObjects[size_t(index)] = std::move(object);
    return index;
}

namespace {
using Baked = RigExecBakedProgramImpl;
struct WeightMath : RigExecGfAffineMath {
    static Mat4 RemoveScaleShear(const Mat4 &m)
    {
        return m.RemoveScaleShear();
    }
    static bool PointsToMatrix(const std::array<Vec3d, 4> &a,
                               const std::array<Vec3d, 4> &b, Mat4 *m)
    {
        return RigExecPointsToMatrix(a, b, m);
    }
};
struct PacketInputs {
    const Baked &b;
    Baked::WeightObject *object;
    VtFloatArray samples;
    eval::WeightPacketFacts<WeightMath> facts;
    PacketInputs(const Baked &program, Baked::WeightObject *o)
        : b(program), object(o)
    {
        if (!o)
            return;
        const auto &w = *o;
        auto &f = facts;
        f.kind = w.oracleKind;
        f.representation = w.oracleRepresentation;
        f.rangePolicy = w.oracleClamp ? eval::kWeightPolicyClamp
                        : w.rangePolicy == SchemaValueText::strict
                            ? eval::kWeightPolicyStrict
                            : eval::kWeightPolicyUnknown;
        f.strict = !w.oracleClamp;
        f.clamp = w.oracleClamp;
        f.combineMode = w.oracleCombineMode;
        f.base = w.base;
        f.inputCount = w.inputs.size();
        f.inputIndices = w.inputs.data();
        const auto read = [&](size_t member, const auto &input) {
            f.scalars[member] = RigExecBakedLeafRead(b, input);
        };
        if (f.kind == eval::kWeightKindStatic) {
            read(0, w.defaultWeight);
            f.paintedValues = w.values.data();
            f.paintedValueCount = w.values.size();
            f.paintedIndices = w.indices.data();
            f.paintedIndexCount = w.indices.size();
        } else if (f.kind == eval::kWeightKindDynamic) {
            read(1, w.driver);
            read(2, w.scale);
            read(3, w.bias);
        } else if (f.kind == eval::kWeightKindCombine) {
            for (size_t i = 0; i < w.combineTargetPoints.size(); ++i)
                f.combineTargetCount +=
                    w.pointLeaves
                        .Value<VtVec3fArray>(int(w.combineLeaves + i), {})
                        .size();
            read(4, w.strength);
            read(5, w.invert);
        } else if (f.kind >= eval::kWeightKindSphere) {
            if (w.providerSlot >= 0) {
                const auto &frame =
                    b.base[size_t(b.baseLast[size_t(w.providerSlot)])];
                f.framePoints = &frame.points;
                f.frameValid = frame.IsValid();
                f.frameDegenerate = frame.IsDegenerate();
            }
            read(4, w.strength);
            read(5, w.invert);
            read(6, w.falloffMin);
            read(7, w.falloffMax);
            if (f.kind == eval::kWeightKindPlane) {
                // Interned comparison: GetText on an empty token reads a
                // guarded static, which a step body may not touch.
                f.planeAxis =
                    w.planeAxis == _tokens->x   ? eval::kWeightPlaneAxisX
                    : w.planeAxis == _tokens->y ? eval::kWeightPlaneAxisY
                    : w.planeAxis == _tokens->z ? eval::kWeightPlaneAxisZ
                                                : eval::kWeightPlaneAxisUnknown;
                f.planeBounds = w.planeBounds == _tokens->bounded
                                    ? eval::kWeightBoundsBounded
                                : w.planeBounds == _tokens->unbounded
                                    ? eval::kWeightBoundsUnbounded
                                    : eval::kWeightBoundsUnknown;
                if (f.planeBounds == eval::kWeightBoundsBounded) {
                    read(17, w.extentU);
                    read(18, w.extentV);
                }
            } else {
                read(8, w.scaleX);
                read(9, w.scaleY);
                read(10, w.scaleZ);
                read(11, w.scaleXPos);
                read(12, w.scaleYPos);
                read(13, w.scaleZPos);
                read(14, w.scaleXNeg);
                read(15, w.scaleYNeg);
                read(16, w.scaleZNeg);
            }
            samples =
                w.pointLeaves.Value<VtFloatArray>(w.falloffSamplesLeaf, {});
            f.falloffCurve =
                samples.empty() ? w.falloffCurve.data() : samples.cdata();
            f.falloffCurveCount =
                samples.empty() ? w.falloffCurve.size() : samples.size();
        }
    }
    const auto &packetFacts(size_t) const
    {
        return facts;
    }
    eval::WeightPointViewT<WeightMath> volumePoints(size_t, size_t key) const
    {
        auto &w = *object;
        const size_t begin[] = {w.sampleLeaves, w.targetLeaves, w.curveLeaves};
        const size_t counts[] = {w.samplePoints.size(), w.targetPoints.size(),
                                 w.curvePoints.size()};
        auto &held = w.packetPointViews[key];
        auto &scratch = w.packetPointScratch[key];
        held.clear();
        scratch.clear();
        if (counts[key] == 1) {
            held = w.pointLeaves.Value<VtVec3fArray>(int(begin[key]), {});
            return {held.cdata(), held.size()};
        }
        for (size_t i = 0; i < counts[key]; ++i) {
            const auto points =
                w.pointLeaves.Value<VtVec3fArray>(int(begin[key] + i), {});
            scratch.insert(scratch.end(), points.begin(), points.end());
        }
        return {scratch.data(), scratch.size()};
    }
    eval::WeightFrameView<WeightMath> volumePlacementFacts(size_t slot,
                                                           bool base) const
    {
        const auto &frame = base ? b.base[size_t(b.baseLast[slot])]
                                 : b.fin[size_t(b.finLast[slot])];
        return {&frame.points, frame.IsValid(), frame.IsDegenerate()};
    }
};
struct PacketState {
    Baked *b;
    const std::vector<RigExecWeightPacket> &packets;
    Baked::WeightObject *object;
    RigExecWeightPacket *result;
    size_t packetCount() const
    {
        return packets.size();
    }
    auto *packetScratch(size_t) const
    {
        return &object->packetWorkspace;
    }
    auto *packetCurveScratch(size_t) const
    {
        return &object->packetLocalCurve;
    }
    eval::WeightPacketView packetView(size_t i) const
    {
        const auto &p = packets[i];
        const int rep =
            p.representation == SchemaValueText::dense    ? eval::kWeightDense
            : p.representation == SchemaValueText::sparse ? eval::kWeightSparse
            : p.representation == SchemaValueText::constant
                ? eval::kWeightConstant
                : eval::kWeightRepresentationUnknown;
        return {p.valid,         rep,
                p.defaultWeight, p.values.data(),
                p.values.size(), p.indices.data(),
                p.indices.size()};
    }
    void storePacket(size_t, eval::WeightPacketResult p) const
    {
        RigExecWeightPacket out;
        if (!p.bare) {
            out.representation = object->representation;
            out.rangePolicy = object->rangePolicy;
            out.defaultWeight = p.defaultWeight;
            out.valid = p.valid;
            out.values = std::move(p.values);
            out.indices = std::move(p.indices);
        }
        *result = std::move(out);
    }
    void setVolumePlacement(size_t slot, bool base,
                            const GfMatrix4d &placement) const
    {
        (base ? b->volumePlacementBase : b->volumePlacement)[slot] = placement;
    }
};
struct PacketHost {
    using Math = WeightMath;
    using Inputs = PacketInputs;
    using State = PacketState;
    using EvalValueFamily = eval::WeightValueFamily;
    Inputs in;
    State out;
    const Inputs &inputs() const
    {
        return in;
    }
    State &state()
    {
        return out;
    }
    const RigExecRangeRunner *ranges() const
    {
        return RigExecGeometryRanges();
    }
};
} // namespace

RigExecWeightPacket
RigExecBakedWeightPacket(const RigExecBakedProgramImpl &program,
                         RigExecBakedProgramImpl::WeightObject *object,
                         const std::vector<RigExecWeightPacket> &packets,
                         UsdTimeCode)
{
    RigExecWeightPacket result;
    PacketHost host{{program, object}, {nullptr, packets, object, &result}};
    const int32_t index = int32_t(object - program.weightObjects.data());
    eval::RunValueStep(host, {uint32_t(eval::StepKind::WeightPacket), index, -1,
                              size_t(index)});
    return result;
}

void
RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedDependencySink *sink)
{
    // This object's OWN inputs only. Every object it composes has a step of
    // its own, and that step declares its own inputs; a packet moves when
    // any of them does, and the cone carries it forward along the
    // WeightPacket edges between them.
    RigExecBakedNoteInput(weight.defaultWeight, sink);
    RigExecBakedNoteInput(weight.driver, sink);
    RigExecBakedNoteInput(weight.scale, sink);
    RigExecBakedNoteInput(weight.bias, sink);
    RigExecBakedNoteInput(weight.strength, sink);
    RigExecBakedNoteInput(weight.invert, sink);
    RigExecBakedNoteInput(weight.falloffMin, sink);
    RigExecBakedNoteInput(weight.falloffMax, sink);
    RigExecBakedNoteInput(weight.scaleXPos, sink);
    RigExecBakedNoteInput(weight.scaleYPos, sink);
    RigExecBakedNoteInput(weight.scaleZPos, sink);
    RigExecBakedNoteInput(weight.scaleXNeg, sink);
    RigExecBakedNoteInput(weight.scaleYNeg, sink);
    RigExecBakedNoteInput(weight.scaleZNeg, sink);
    RigExecBakedNoteInput(weight.scaleX, sink);
    RigExecBakedNoteInput(weight.scaleY, sink);
    RigExecBakedNoteInput(weight.scaleZ, sink);
    RigExecBakedNoteInput(weight.extentU, sink);
    RigExecBakedNoteInput(weight.extentV, sink);
    // The point arrays a volume measures, which no RigExecBakedInput covers:
    // they are read through the generation's resolved inputs every frame, so
    // a property chain or a drag on the weighted mesh reaches this step the
    // same way it reaches a geometry mover.
    for (const std::vector<UsdAttribute> *points :
         {&weight.targetPoints, &weight.samplePoints, &weight.curvePoints,
          &weight.combineTargetPoints}) {
        if (!points->empty()) {
            sink->resolvedReads = true;
            for (const UsdAttribute &a : *points) {
                sink->step->varyingInputs =
                    sink->step->varyingInputs || a.ValueMightBeTimeVarying();
            }
        }
    }
}

void
RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // Where each volume weight ended up: one step per volume slot, writing
    // that slot of the program's volumePlacement table (WeightFrames[slot]).
    // The table has two readers. pose.weightFrames is one (the epilogue
    // publishes it); the other is the ORACLE, handed the table as a
    // RigExecVolumePlacementView, which a current-phase field resolves
    // through and which declares WeightFrames for exactly the volumes of its
    // object's closure. The geometry walk runs after every pose commit, so a
    // placement from the slot's last pose version is what every reader
    // sees. Every volume slot gets a step, bound to a mover or not: an
    // unbound volume still publishes a weightFrames entry.
    // part = 1 marks the per-volume form (object = provider slot); the
    // whole-map form (object 0, part -1) is not emitted.
    for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
        if (B.noScaleAvars[i] == 0) {
            continue;
        }
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::VolumePlacements;
        step.object = int(i);
        step.part = 1;
        step.maxDiagnostics = 0;
        step.reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, B.finLast[i]));
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, int(i)));
        B.steps.push_back(std::move(step));
        RigExecBakedStep base;
        base.kind = RigExecBakedStepKind::VolumePlacements;
        base.object = int(i);
        base.part = 2;
        base.maxDiagnostics = 0;
        base.reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseBase, B.baseLast[i]));
        base.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFramesBase, int(i)));
        B.steps.push_back(std::move(base));
    }
    // The slot storage, sized once. A consumer holds a pointer into it for
    // the whole region, so it is never resized inside one.
    B.weightPackets.assign(B.weightObjects.size(), RigExecWeightPacket());
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[i];
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::WeightPacket;
        step.object = int(i);
        step.maxDiagnostics = 0;
        // The composition, as edges. The table is in dependency order, so
        // every one of these names a lower index and the edge sweep sees a
        // forward edge like any other.
        if (weight.base >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::WeightPacket, weight.base));
        }
        for (const int input : weight.inputs) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket, input));
        }
        // A volume's placement is a pose frame, which makes its step a
        // reader of the pose half and so neither a source nor a pure
        // function of the stage.
        if (weight.providerSlot >= 0) {
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,
                                B.baseLast[size_t(weight.providerSlot)]));
        }
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket, int(i)));
        B.steps.push_back(std::move(step));
    }
}

void
RigExecBakedRunWeightStep(RigExecBakedProgramImpl *program,
                          RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::VolumePlacements) {
        PacketHost host{{B, nullptr}, {&B, B.weightPackets, nullptr, nullptr}};
        eval::RunValueStep(host, {uint32_t(step->kind), step->object,
                                  step->part, size_t(step->object)});
        return;
    }
    RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(step->object)];
    // Rebuilt every frame; one packet per object per frame.
    B.weightPackets[size_t(step->object)] =
        RigExecBakedWeightPacket(B, &weight, B.weightPackets, time);
}

void
RigExecBakedPublishVolumePlacements(const RigExecBakedProgramImpl &program,
                                    std::map<SdfPath, GfMatrix4d> *frames)
{
    frames->clear();
    const std::vector<char> &slots = program.placedVolumes;
    for (size_t i = 0; i < slots.size() && i < program.paths.size() &&
                       i < program.volumePlacement.size();
         ++i) {
        if (slots[i]) {
            // Slots are in path order, so each entry lands at the end.
            frames->emplace_hint(frames->end(), program.paths[i],
                                 program.volumePlacement[i]);
        }
    }
}

} // namespace rigExec

namespace rigExec {

void
RigExecBakedResetWeightPacket(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.weightPackets[slot] = {};
}

void
RigExecBakedResetWeightFrames(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.volumePlacement[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetWeightFramesBase(RigExecBakedProgramImpl &B, uint32_t slot)
{
    B.volumePlacementBase[slot] = RigExecBakedSetAsideMatrix();
}

void
RigExecBakedResetWeightFieldSlot(RigExecBakedProgramImpl &B, uint32_t slot)
{
    RigExecBakedResetWeightField(&B, int(slot));
}

} // namespace rigExec
