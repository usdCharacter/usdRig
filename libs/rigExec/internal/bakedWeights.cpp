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

#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/weights.h"
#include "rigExecOps/weight/weightOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/affineFrameKernels.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <cstring>
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

    // The structural tokens are read plainly, with no time and no resolution
    // walk, and land in `folded`, so an edit to one rebuilds the program.
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
                // on actual point-array attributes; raw field relationship
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

using rigExecOps::VolumeWeightShape;

/// A float input of a weight object, by its WeightArrays read.
RigExecBakedInput<float> RigExecBakedWeightObject::*const kFloatReads[] = {
    &RigExecBakedWeightObject::defaultWeight,
    &RigExecBakedWeightObject::driver,
    &RigExecBakedWeightObject::scale,
    &RigExecBakedWeightObject::bias,
    &RigExecBakedWeightObject::strength,
    &RigExecBakedWeightObject::invert,
    &RigExecBakedWeightObject::falloffMin,
    &RigExecBakedWeightObject::falloffMax,
};

/// The three float inputs a Vec3f WeightArrays read packs.
RigExecBakedInput<float> RigExecBakedWeightObject::*const kAxisReads[3][3] = {
    {&RigExecBakedWeightObject::scaleX, &RigExecBakedWeightObject::scaleY,
     &RigExecBakedWeightObject::scaleZ},
    {&RigExecBakedWeightObject::scaleXPos, &RigExecBakedWeightObject::scaleYPos,
     &RigExecBakedWeightObject::scaleZPos},
    {&RigExecBakedWeightObject::scaleXNeg, &RigExecBakedWeightObject::scaleYNeg,
     &RigExecBakedWeightObject::scaleZNeg},
};

bool
WeightFail(std::string *error, const std::string &why)
{
    if (error)
        *error = why;
    return false;
}

template <class T>
bool
PutWeightValue(const T &value, const RigExecTypeDesc &type, void *out)
{
    if (type.size != sizeof(T) || !type.triviallyCopyable)
        return false;
    std::memcpy(out, &value, sizeof(T));
    return true;
}

/// \p count elements at \p data lent to an array port value of \p type.
bool
LendWeightArray(const void *data, size_t count, const RigExecTypeDesc &type,
                void *out)
{
    if (!type.borrow)
        return false;
    type.borrow(out, data, count);
    return true;
}

/// The points of attribute list \p key (0 samples, 1 target, 2 curve) of
/// \p w, held by \p w until its next read of the same list.
std::pair<const GfVec3f *, size_t>
WeightPoints(RigExecBakedWeightObject &w, size_t key)
{
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

/// The range policy of \p w a packet op runs under, or false when it has
/// none.
bool
PacketClamp(const RigExecBakedWeightObject &w, bool *clamp)
{
    *clamp = w.fieldClamp;
    return w.fieldClamp || w.rangePolicy == SchemaValueText::strict;
}

/// Whether \p input holds one value for every frame and every edit.
bool
FixedInput(const RigExecBakedInput<float> &input)
{
    return !input.varying && input.overrideIndex < 0 && input.walk < 0;
}

} // namespace

bool
RigExecBakedLookUpWeight(RigExecBakedProgramImpl &B,
                         const eval::OpLookup &lookup,
                         const RigExecTypeDesc &type, void *out,
                         std::string *error)
{
    if (lookup.object >= B.weightObjects.size())
        return WeightFail(error, "weight object " +
                                     std::to_string(lookup.object) +
                                     " is no weight object of the program");
    RigExecBakedWeightObject &w = B.weightObjects[lookup.object];
    using R = eval::OpWeightRead;
    const R read = R(lookup.read);
    bool ok = false;
    if (lookup.read <= uint8_t(R::FalloffMax)) {
        ok = PutWeightValue(
            RigExecBakedLeafRead(B, w.*kFloatReads[lookup.read]), type, out);
    } else if (read == R::ExtentU || read == R::ExtentV) {
        ok = PutWeightValue(
            RigExecBakedLeafRead(B, read == R::ExtentU ? w.extentU : w.extentV),
            type, out);
    } else if (read == R::ScaleAxes || read == R::PositiveScales ||
               read == R::NegativeScales) {
        const auto &axes = kAxisReads[lookup.read - uint8_t(R::ScaleAxes)];
        const GfVec3f value(RigExecBakedLeafRead(B, w.*axes[0]),
                            RigExecBakedLeafRead(B, w.*axes[1]),
                            RigExecBakedLeafRead(B, w.*axes[2]));
        ok = PutWeightValue(value, type, out);
    } else if (read == R::Values) {
        ok = LendWeightArray(w.values.data(), w.values.size(), type, out);
    } else if (read == R::Indices) {
        static_assert(sizeof(int) == sizeof(int32_t), "indices are int32");
        ok = LendWeightArray(w.indices.data(), w.indices.size(), type, out);
    } else if (read == R::SamplePoints || read == R::TargetPoints ||
               read == R::CurvePoints) {
        const auto points =
            WeightPoints(w, size_t(lookup.read - uint8_t(R::SamplePoints)));
        ok = LendWeightArray(points.first, points.second, type, out);
    } else if (read == R::FalloffSamples) {
        w.packetFalloffSamples =
            w.pointLeaves.Value<VtFloatArray>(w.falloffSamplesLeaf, {});
        ok = w.packetFalloffSamples.empty()
                 ? LendWeightArray(w.falloffCurve.data(), w.falloffCurve.size(),
                                   type, out)
                 : LendWeightArray(w.packetFalloffSamples.cdata(),
                                   w.packetFalloffSamples.size(), type, out);
    } else if (read == R::CombineTargetCount) {
        uint32_t count = 0;
        for (size_t i = 0; i < w.combineTargetPoints.size(); ++i)
            count += uint32_t(
                w.pointLeaves.Value<VtVec3fArray>(int(w.combineLeaves + i), {})
                    .size());
        ok = PutWeightValue(count, type, out);
    }
    if (!ok)
        return WeightFail(error,
                          "weight object " + std::to_string(lookup.object) +
                              " has no input " + std::to_string(lookup.read) +
                              " of the looked-up type");
    return true;
}

bool
RigExecBakedStoreWeightPacket(RigExecBakedProgramImpl &B,
                              const eval::OpHostOutput &output,
                              const void *value, std::string *error)
{
    if (output.object >= B.weightPackets.size())
        return WeightFail(error, "weight object " +
                                     std::to_string(output.object) +
                                     " holds no packet");
    const auto &in = *static_cast<const rigExecRig::PointWeights *>(value);
    const RigExecBakedWeightObject &w = B.weightObjects[output.object];
    RigExecWeightPacket &packet = B.weightPackets[output.object];
    packet.representation = w.representation;
    packet.rangePolicy = w.rangePolicy;
    packet.values.assign(in.values.Data(), in.values.Data() + in.values.Size());
    packet.indices.assign(in.indices.Data(),
                          in.indices.Data() + in.indices.Size());
    packet.defaultWeight = in.defaultWeight;
    packet.valid = in.valid != 0;
    return true;
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
    // publishes it); the other is the weight field, handed the table as a
    // RigExecVolumePlacementView, which a current-phase field resolves
    // through and which declares WeightFrames for exactly the volumes of its
    // object's closure. The geometry walk runs after every pose commit, so a
    // placement from the slot's last pose version is what every reader
    // sees. Every volume slot gets a step, bound to a mover or not: an
    // unbound volume still publishes a weightFrames entry.
    // Each runs as a VolumePlacement op node identified as VolumePlacements:
    // object the provider slot, part 1 the final placement and 2 the base
    // one.
    for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
        if (B.noScaleAvars[i] == 0) {
            continue;
        }
        for (const int part : {1, 2}) {
            const bool base = part == 2;
            RigExecBakedOpOrigin origin;
            origin.family = RigExecBakedOpFamily::VolumePlacement;
            origin.kind = RigExecBakedStepKind::VolumePlacements;
            origin.object = int32_t(i);
            origin.part = part;
            RigExecBakedStep step;
            step.kind = RigExecBakedStepKind::OpNode;
            step.object = int(B.opNodeOrigins.size());
            step.part = -1;
            step.maxDiagnostics = 1;
            step.reads.push_back(
                RigExecBakedOne(base ? RigExecBakedSlotDomain::PoseBase
                                     : RigExecBakedSlotDomain::PoseFin,
                                base ? B.baseLast[i] : B.finLast[i]));
            step.writes.push_back(
                RigExecBakedOne(base ? RigExecBakedSlotDomain::WeightFramesBase
                                     : RigExecBakedSlotDomain::WeightFrames,
                                int(i)));
            B.opNodeOrigins.push_back(origin);
            B.steps.push_back(std::move(step));
        }
    }
    // The slot storage, sized once. A consumer holds a pointer into it for
    // the whole region, so it is never resized inside one.
    B.weightPackets.assign(B.weightObjects.size(), RigExecWeightPacket());
    // One op node per weight object a revision's envelope reads, and per
    // object it composes, keeping the WeightPacket identity, numbered after
    // the objects it composes; the steps stay in table order. A weight read
    // only as a field makes no packet.
    std::vector<char> enveloping(B.weightObjects.size(), 0);
    const auto envelopes = [&](int object) {
        if (object >= 0 && size_t(object) < enveloping.size())
            enveloping[size_t(object)] = 1;
    };
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions)
            envelopes(revision.weightObject);
        for (const auto &derived : chain.derived)
            envelopes(derived.revision.weightObject);
    }
    std::vector<int> nodeOf(B.weightObjects.size(), -1);
    std::vector<std::pair<size_t, size_t>> stack;
    for (size_t root = 0; root < B.weightObjects.size(); ++root) {
        if (nodeOf[root] >= 0 || !enveloping[root])
            continue;
        stack.assign(1, {root, 0});
        nodeOf[root] = -2;
        while (!stack.empty()) {
            const size_t object = stack.back().first;
            const auto &weight = B.weightObjects[object];
            const size_t next = stack.back().second++;
            const int composed = next == 0 ? weight.base
                                 : next <= weight.inputs.size()
                                     ? weight.inputs[next - 1]
                                     : -3;
            if (composed == -3) {
                stack.pop_back();
                nodeOf[object] = int(B.opNodeOrigins.size());
                RigExecBakedOpOrigin origin;
                origin.family = RigExecBakedOpFamily::WeightPacket;
                origin.kind = RigExecBakedStepKind::WeightPacket;
                origin.object = int32_t(object);
                B.opNodeOrigins.push_back(origin);
            } else if (composed >= 0 &&
                       size_t(composed) < B.weightObjects.size() &&
                       nodeOf[size_t(composed)] == -1) {
                nodeOf[size_t(composed)] = -2;
                stack.push_back({size_t(composed), 0});
            }
        }
    }
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        if (nodeOf[i] < 0)
            continue;
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[i];
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::OpNode;
        step.object = nodeOf[i];
        step.maxDiagnostics = 1;
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
RigExecBakedAddVolumePlacementOps(const RigExecBakedProgramImpl &B,
                                  RigExecRigDescBuilder *builder,
                                  eval::OpStepLayout *layout)
{
    for (const RigExecBakedOpOrigin &origin : B.opNodeOrigins) {
        if (origin.family != RigExecBakedOpFamily::VolumePlacement)
            continue;
        const bool base = origin.part == 2;
        const size_t slot = size_t(origin.object);
        const RigExecNodeId node =
            builder->AddOp(rigExecOps::VolumePlacement());
        builder->BindInput(node, rigExecOps::VolumePlacementFrame());
        builder->BindOutput(node, rigExecOps::VolumePlacementOut());
        layout->hostInputs.push_back(
            {node, rigExecOps::VolumePlacementFrame().Index(),
             eval::OpHostSource::Lookup, uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::PoseVersion),
             uint32_t(base ? B.baseLast[slot] : B.finLast[slot]), base ? 1 : 0,
             0});
        layout->hostOutputs.push_back(
            {node, rigExecOps::VolumePlacementOut().Index(),
             uint8_t(base ? eval::OpOutputTarget::VolumePlacementBase
                          : eval::OpOutputTarget::VolumePlacement),
             uint32_t(slot)});
    }
}

namespace {

// What a packet object's authored structure gets wrong, or empty: checked for
// every packet object the rig binds, read by an envelope or not.
std::string
PacketStructureError(const RigExecBakedProgramImpl &B,
                     const RigExecBakedWeightObject &w)
{
    bool clamp = false;
    if (!PacketClamp(w, &clamp))
        return "unknown rigExec:rangePolicy '" + w.rangePolicy.GetString() +
               "'";
    const bool known = w.fieldRepresentation >= eval::kWeightConstant &&
                       w.fieldRepresentation <= eval::kWeightSparse;
    switch (w.fieldKind) {
    case eval::kWeightKindStatic: {
        if (!known)
            return "unknown rigExec:representation '" +
                   w.representation.GetString() + "'";
        static_assert(sizeof(int) == sizeof(int32_t), "indices are int32");
        RigExecStaticWeightScratch scratch;
        RigExecWeightPacketBuild built;
        // A default weight a frame can set is checked when it runs.
        if (const char *why = RigExecStaticWeightPacketKernel(
                w.fieldRepresentation, clamp,
                FixedInput(w.defaultWeight)
                    ? RigExecBakedLeafRead(B, w.defaultWeight)
                    : 0.0f,
                w.values.data(), w.values.size(),
                reinterpret_cast<const int32_t *>(w.indices.data()),
                w.indices.size(), &scratch, &built))
            return why;
        return {};
    }
    case eval::kWeightKindDynamic:
        if (!known)
            return "unknown rigExec:representation '" +
                   w.representation.GetString() + "'";
        if (w.base >= 0) {
            if (B.weightObjects[size_t(w.base)].fieldRepresentation !=
                w.fieldRepresentation)
                return "its base packet is of another representation";
        } else if (w.fieldRepresentation != eval::kWeightConstant) {
            return "a packet that is not constant needs a base weight";
        }
        return {};
    case eval::kWeightKindCombine:
        if (w.fieldRepresentation != eval::kWeightDense)
            return "a combined packet is dense";
        if (w.fieldCombineMode < 0 ||
            w.fieldCombineMode > eval::kWeightCombineOverlay)
            return "unknown rigExec:combineMode '" + w.combineMode.GetString() +
                   "'";
        return {};
    case eval::kWeightKindSphere:
    case eval::kWeightKindPlane:
    case eval::kWeightKindCurve:
        if (w.fieldRepresentation != eval::kWeightDense)
            return "a volume's packet is dense";
        if (w.providerSlot < 0 || size_t(w.providerSlot) >= B.baseLast.size())
            return "no pose frame places the volume";
        if (w.targetPoints.empty())
            return "the volume weighs no target points";
        if (w.fieldKind == eval::kWeightKindPlane) {
            if (w.planeAxis != _tokens->x && w.planeAxis != _tokens->y &&
                w.planeAxis != _tokens->z)
                return "unknown rigExec:planeAxis '" + w.planeAxis.GetString() +
                       "'";
            if (w.planeBounds != _tokens->bounded &&
                w.planeBounds != _tokens->unbounded)
                return "unknown rigExec:planeBounds '" +
                       w.planeBounds.GetString() + "'";
        }
        if (w.fieldKind == eval::kWeightKindCurve && w.curvePoints.empty())
            return "the curve weight has no curve";
        return {};
    default:
        return "a weight object of type '" + w.type.GetString() +
               "' makes no packet";
    }
}

} // namespace

bool
RigExecBakedAddWeightPacketOps(const RigExecBakedProgramImpl &B,
                               RigExecRigDescBuilder *builder,
                               eval::OpStepLayout *layout, std::string *error)
{
    using R = eval::OpWeightRead;
    std::vector<RigExecNodeId> nodeOf(B.weightObjects.size(),
                                      kRigExecInvalidIndex);
    std::vector<const RigExecPortHandle *> outOf(B.weightObjects.size(),
                                                 nullptr);
    struct Feed {
        size_t from;
        RigExecNodeId node;
        const RigExecPortHandle *port;
    };
    std::vector<Feed> feeds;
    for (const RigExecBakedWeightObject &w : B.weightObjects) {
        if (w.envelopeOnly)
            continue;
        const std::string why = PacketStructureError(B, w);
        if (!why.empty())
            return WeightFail(error, w.path.GetString() + ": " + why);
    }
    for (const RigExecBakedOpOrigin &origin : B.opNodeOrigins) {
        if (origin.family != RigExecBakedOpFamily::WeightPacket)
            continue;
        const size_t i = size_t(origin.object);
        const RigExecBakedWeightObject &w = B.weightObjects[i];
        bool clamp = false;
        PacketClamp(w, &clamp);
        const auto look = [&](RigExecNodeId node, const RigExecPortHandle &port,
                              R read) {
            builder->BindInput(node, port);
            layout->hostInputs.push_back({node, port.Index(),
                                          eval::OpHostSource::Lookup,
                                          uint32_t(layout->lookups.size())});
            layout->lookups.push_back(
                {uint8_t(eval::OpLookupSource::WeightArrays), uint32_t(i), -1,
                 uint8_t(read)});
        };
        const auto feed = [&](int from, RigExecNodeId node,
                              const RigExecPortHandle &port) {
            feeds.push_back({size_t(from), node, &port});
        };
        const auto representation =
            rigExecRig::WeightRepresentation(w.fieldRepresentation);
        RigExecNodeId node = kRigExecInvalidIndex;
        const RigExecPortHandle *out = nullptr;
        switch (w.fieldKind) {
        case eval::kWeightKindStatic: {
            const rigExecOps::WeightPacketPayload payload{representation,
                                                          clamp};
            node = builder->AddOp(rigExecOps::StaticWeights(), &payload);
            look(node, rigExecOps::StaticWeightsValues(), R::Values);
            look(node, rigExecOps::StaticWeightsIndices(), R::Indices);
            look(node, rigExecOps::StaticWeightsDefaultWeight(),
                 R::DefaultWeight);
            out = &rigExecOps::StaticWeightsOut();
            break;
        }
        case eval::kWeightKindDynamic: {
            const rigExecOps::WeightPacketPayload payload{representation,
                                                          clamp};
            node = builder->AddOp(rigExecOps::DynamicWeights(), &payload);
            if (w.base >= 0)
                feed(w.base, node, rigExecOps::DynamicWeightsBase());
            look(node, rigExecOps::DynamicWeightsDriver(), R::Driver);
            look(node, rigExecOps::DynamicWeightsScale(), R::Scale);
            look(node, rigExecOps::DynamicWeightsBias(), R::Bias);
            out = &rigExecOps::DynamicWeightsOut();
            break;
        }
        case eval::kWeightKindCombine: {
            rigExecOps::CombineWeightsPayload payload;
            payload.mode = rigExecOps::WeightCombineMode(w.fieldCombineMode);
            payload.inputs = rigExecOps::WeightCombineInputs::Packets;
            payload.clamp = clamp;
            node = builder->AddOp(rigExecOps::CombineWeights(), &payload);
            for (const int input : w.inputs)
                feed(input, node, rigExecOps::CombineWeightsInputs());
            look(node, rigExecOps::CombineWeightsTargetCount(),
                 R::CombineTargetCount);
            look(node, rigExecOps::CombineWeightsStrength(), R::Strength);
            look(node, rigExecOps::CombineWeightsInvert(), R::Invert);
            out = &rigExecOps::CombineWeightsOut();
            break;
        }
        case eval::kWeightKindSphere:
        case eval::kWeightKindPlane:
        case eval::kWeightKindCurve: {
            rigExecOps::VolumeWeightsPayload payload;
            payload.clamp = clamp;
            payload.shape = w.fieldKind == eval::kWeightKindSphere
                                ? VolumeWeightShape::Sphere
                            : w.fieldKind == eval::kWeightKindPlane
                                ? VolumeWeightShape::Plane
                                : VolumeWeightShape::Curve;
            if (payload.shape == VolumeWeightShape::Plane) {
                payload.planeAxis = w.planeAxis == _tokens->x
                                        ? rigExecOps::WeightFieldAxis::X
                                    : w.planeAxis == _tokens->y
                                        ? rigExecOps::WeightFieldAxis::Y
                                        : rigExecOps::WeightFieldAxis::Z;
                payload.planeBounded = w.planeBounds == _tokens->bounded;
            }
            node = builder->AddOp(rigExecOps::VolumeWeights(), &payload);
            builder->BindInput(node, rigExecOps::VolumeWeightsFrame());
            layout->hostInputs.push_back(
                {node, rigExecOps::VolumeWeightsFrame().Index(),
                 eval::OpHostSource::Lookup, uint32_t(layout->lookups.size())});
            layout->lookups.push_back(
                {uint8_t(eval::OpLookupSource::PoseVersion),
                 uint32_t(B.baseLast[size_t(w.providerSlot)]), 1, 0});
            look(node, rigExecOps::VolumeWeightsPoints(), R::TargetPoints);
            if (!w.samplePoints.empty())
                look(node, rigExecOps::VolumeWeightsSamples(), R::SamplePoints);
            if (payload.shape == VolumeWeightShape::Curve)
                look(node, rigExecOps::VolumeWeightsCurve(), R::CurvePoints);
            look(node, rigExecOps::VolumeWeightsStrength(), R::Strength);
            look(node, rigExecOps::VolumeWeightsInvert(), R::Invert);
            look(node, rigExecOps::VolumeWeightsFalloffMin(), R::FalloffMin);
            look(node, rigExecOps::VolumeWeightsFalloffMax(), R::FalloffMax);
            look(node, rigExecOps::VolumeWeightsFalloffSamples(),
                 R::FalloffSamples);
            if (payload.shape != VolumeWeightShape::Plane)
                look(node, rigExecOps::VolumeWeightsScale(), R::ScaleAxes);
            if (payload.shape == VolumeWeightShape::Sphere) {
                look(node, rigExecOps::VolumeWeightsPositiveScale(),
                     R::PositiveScales);
                look(node, rigExecOps::VolumeWeightsNegativeScale(),
                     R::NegativeScales);
            }
            if (payload.planeBounded) {
                look(node, rigExecOps::VolumeWeightsExtentU(), R::ExtentU);
                look(node, rigExecOps::VolumeWeightsExtentV(), R::ExtentV);
            }
            out = &rigExecOps::VolumeWeightsOut();
            break;
        }
        default:
            return WeightFail(error, w.path.GetString() + ": makes no packet");
        }
        builder->BindOutput(node, *out);
        layout->hostOutputs.push_back(
            {node, out->Index(), uint8_t(eval::OpOutputTarget::WeightPacket),
             uint32_t(i)});
        nodeOf[i] = node;
        outOf[i] = out;
    }
    for (const Feed &feed : feeds) {
        if (nodeOf[feed.from] == kRigExecInvalidIndex)
            return WeightFail(error,
                              B.weightObjects[feed.from].path.GetString() +
                                  ": composed into a packet, makes none");
        if (nodeOf[feed.from] >= feed.node)
            return WeightFail(error,
                              "operation cycle: " +
                                  B.weightObjects[feed.from].path.GetString() +
                                  " composes a weight that composes it");
        builder->Connect(nodeOf[feed.from], *outOf[feed.from], feed.node,
                         *feed.port);
    }
    return true;
}

bool
RigExecBakedLookUpPoseVersion(const RigExecBakedProgramImpl &B,
                              const eval::OpLookup &lookup,
                              const RigExecTypeDesc &type, void *out,
                              std::string *error)
{
    const std::vector<RigExecPointFrame> &frames = lookup.part ? B.base : B.fin;
    if (type.size != sizeof(rigExecRig::PointFrame) ||
        lookup.object >= frames.size()) {
        if (error)
            *error = "pose version " + std::to_string(lookup.object) +
                     " is no pose frame of the program";
        return false;
    }
    const RigExecPointFrame &frame = frames[lookup.object];
    auto &value = *static_cast<rigExecRig::PointFrame *>(out);
    for (size_t i = 0; i < 4; ++i)
        value.points[i] = rigExecStd::Vec3d(
            frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    value.flags = frame.flags;
    return true;
}

bool
RigExecBakedStoreVolumePlacement(RigExecBakedProgramImpl &B,
                                 const eval::OpHostOutput &output,
                                 const void *value, std::string *error)
{
    const bool base =
        output.target == uint8_t(eval::OpOutputTarget::VolumePlacementBase);
    std::vector<GfMatrix4d> &placements =
        base ? B.volumePlacementBase : B.volumePlacement;
    if (output.object >= placements.size()) {
        if (error)
            *error =
                "slot " + std::to_string(output.object) + " places no volume";
        return false;
    }
    std::memcpy(placements[output.object].data(), value, sizeof(GfMatrix4d));
    return true;
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
