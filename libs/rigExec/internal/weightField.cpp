#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/weightField.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExecScene/weightProgram.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/weights.h"
#include "rigExecMath/affineFrameKernels.h"
#include "rigExecOps/weight/weightOps.h"
#include "rigExecStdTypes/array.h"
#include "rigExecStdTypes/stdTypes.h"
#include "rigExec/api/rigDesc.h"
#include <pxr/base/tf/staticTokens.h>
#include <pxr/usd/usdGeom/pointBased.h>
#include <pxr/usd/sdf/types.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE

// Interned once: Build runs on parallel lanes, and building a token from
// text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_weightFieldTokens,
    (points)
    (constant)
    (dense)
    (sparse)
    (clamp)
    ((planeAxis, rigExec::SchemaAttrText::rigExecPlaneAxis))
    ((planeBounds, rigExec::SchemaAttrText::rigExecPlaneBounds))
    ((sampleSource, rigExec::SchemaAttrText::rigExecSampleSource))
    ((weightTarget, rigExec::SchemaAttrText::rigExecWeightTarget))
    ((curve, rigExec::SchemaAttrText::rigExecCurve))
    ((falloffSamples, rigExec::SchemaAttrText::inputsFalloffSamples))
    ((weightObject, rigExec::SchemaAttrText::rigExecWeightObject))
    ((readPhase, rigExec::RigExecReadPhaseMetadataName))
    ((rigExecDefaultWeight, rigExec::SchemaAttrText::rigExecDefaultWeight))
    ((inputsDriver, rigExec::SchemaAttrText::inputsDriver))
    ((inputsScale, rigExec::SchemaAttrText::inputsScale))
    ((inputsBias, rigExec::SchemaAttrText::inputsBias))
    ((inputsStrength, rigExec::SchemaAttrText::inputsStrength))
    ((inputsInvert, rigExec::SchemaAttrText::inputsInvert))
    ((inputsFalloffMin, rigExec::SchemaAttrText::inputsFalloffMin))
    ((inputsFalloffMax, rigExec::SchemaAttrText::inputsFalloffMax))
    ((inputsScaleX, rigExec::SchemaAttrText::inputsScaleX))
    ((inputsScaleY, rigExec::SchemaAttrText::inputsScaleY))
    ((inputsScaleZ, rigExec::SchemaAttrText::inputsScaleZ))
    ((inputsScaleXPos, rigExec::SchemaAttrText::inputsScaleXPos))
    ((inputsScaleYPos, rigExec::SchemaAttrText::inputsScaleYPos))
    ((inputsScaleZPos, rigExec::SchemaAttrText::inputsScaleZPos))
    ((inputsScaleXNeg, rigExec::SchemaAttrText::inputsScaleXNeg))
    ((inputsScaleYNeg, rigExec::SchemaAttrText::inputsScaleYNeg))
    ((inputsScaleZNeg, rigExec::SchemaAttrText::inputsScaleZNeg))
    ((inputsExtentU, rigExec::SchemaAttrText::inputsExtentU))
    ((inputsExtentV, rigExec::SchemaAttrText::inputsExtentV))
    ((frozenWeightFieldLeaf0, "frozenWeightField:leaf0"))
    ((frozenWeightFieldLeaf1, "frozenWeightField:leaf1"))
    ((frozenWeightFieldLeaf2, "frozenWeightField:leaf2"))
    ((frozenWeightFieldLeaf3, "frozenWeightField:leaf3"))
    ((frozenWeightFieldLeaf4, "frozenWeightField:leaf4"))
    ((frozenWeightFieldLeaf5, "frozenWeightField:leaf5")));

namespace rigExec {
namespace {
using Baked = RigExecBakedProgramImpl;
using Field = Baked::WeightField;
const TfToken ScalarNames[] = {_weightFieldTokens->rigExecDefaultWeight,
                               _weightFieldTokens->inputsDriver,
                               _weightFieldTokens->inputsScale,
                               _weightFieldTokens->inputsBias,
                               _weightFieldTokens->inputsStrength,
                               _weightFieldTokens->inputsInvert,
                               _weightFieldTokens->inputsFalloffMin,
                               _weightFieldTokens->inputsFalloffMax,
                               _weightFieldTokens->inputsScaleX,
                               _weightFieldTokens->inputsScaleY,
                               _weightFieldTokens->inputsScaleZ,
                               _weightFieldTokens->inputsScaleXPos,
                               _weightFieldTokens->inputsScaleYPos,
                               _weightFieldTokens->inputsScaleZPos,
                               _weightFieldTokens->inputsScaleXNeg,
                               _weightFieldTokens->inputsScaleYNeg,
                               _weightFieldTokens->inputsScaleZNeg,
                               _weightFieldTokens->inputsExtentU,
                               _weightFieldTokens->inputsExtentV};
SdfPath
PointsPath(const UsdStageRefPtr &stage, const UsdPrim &prim,
           const TfToken &name)
{
    SdfPathVector paths;
    const auto rel = prim.GetRelationship(name);
    if (!rel || !rel.GetTargets(&paths) || paths.size() != 1)
        return SdfPath();
    if (paths[0].IsPrimPath()) {
        const auto target = stage->GetPrimAtPath(paths[0]);
        return UsdGeomPointBased(target)
                   ? paths[0].AppendProperty(_weightFieldTokens->points)
                   : SdfPath();
    }
    return paths[0];
}
// The frozen run's per-leaf sample keys, interned once at load: one per
// field leaf, the three point sets, the two plane tokens and a volume's
// sampled falloff.
const std::array<TfToken, 6> _fieldLeafKeys = {
    _weightFieldTokens->frozenWeightFieldLeaf0,
    _weightFieldTokens->frozenWeightFieldLeaf1,
    _weightFieldTokens->frozenWeightFieldLeaf2,
    _weightFieldTokens->frozenWeightFieldLeaf3,
    _weightFieldTokens->frozenWeightFieldLeaf4,
    _weightFieldTokens->frozenWeightFieldLeaf5};
float
Scalar(const Baked &B, const Field &field, int object, int member,
       float fallback)
{
    const size_t key = size_t(object) * 19 + size_t(member);
    if (object >= 0 && member >= 0 && member < 19 &&
        key < field.scalarReadIndex.size()) {
        const int read = field.scalarReadIndex[key];
        if (read >= 0)
            return RigExecBakedReadWeightScalar(
                B, field.scalarReads[size_t(read)], field.availableChains,
                fallback);
    }
    return fallback;
}

} // namespace
bool
RigExecBakedBuildWeightFields(const RigExecRigEvaluator &evaluator,
                              Baked *program, std::string *error)
{
    auto &B = *program;
    B.weightFields.clear();
    B.weightProgram.clear();
    // GetString() interns a path and reads a static on an empty token.
    RigExecPathText pathText;
    for (auto &object : B.weightObjects) {
        RigExecBakedDescribeWeightFieldFacts(evaluator, object.path,
                                             RigExecBakedProbeTime(B.stage),
                                             &object.fieldFacts);
        object.fieldName = pathText(object.path);
        const std::string type = object.type.GetText();
        object.fieldKind = type == SchemaTypeName::RigExecStaticWeight    ? 0
                           : type == SchemaTypeName::RigExecDynamicWeight ? 1
                           : type == SchemaTypeName::RigExecCombineWeight ? 2
                           : type == SchemaTypeName::RigExecSphereWeight  ? 3
                           : type == SchemaTypeName::RigExecPlaneWeight   ? 4
                           : type == SchemaTypeName::RigExecCurveWeight   ? 5
                                                                          : -1;
        object.fieldRepresentation =
            object.representation == _weightFieldTokens->constant ? 0
            : object.representation == _weightFieldTokens->dense  ? 1
            : object.representation == _weightFieldTokens->sparse ? 2
                                                                  : -1;
        object.fieldClamp = object.rangePolicy == _weightFieldTokens->clamp;
        const std::string modes[] = {
            SchemaValueText::multiply, SchemaValueText::add,
            SchemaValueText::subtract, SchemaValueText::max,
            SchemaValueText::min,      SchemaValueText::average,
            SchemaValueText::overlay};
        const std::string combine = object.combineMode.GetText();
        for (int i = 0; i < 7; ++i)
            if (combine == modes[i])
                object.fieldCombineMode = i;
        object.fieldAxisName = object.fieldFacts.planeAxis.GetText();
        object.fieldBoundsName = object.fieldFacts.planeBounds.GetText();
        object.fieldAxis = object.fieldAxisName == SchemaValueText::x   ? 0
                           : object.fieldAxisName == SchemaValueText::y ? 1
                           : object.fieldAxisName == SchemaValueText::z ? 2
                                                                        : -1;
        object.fieldBounded =
            object.fieldBoundsName == SchemaValueText::bounded;
        object.fieldUnbounded =
            object.fieldBoundsName == SchemaValueText::unbounded;
        const auto prim = B.stage->GetPrimAtPath(object.path);
        object.fieldAxisAttribute =
            bool(prim.GetAttribute(_weightFieldTokens->planeAxis));
        object.fieldBoundsAttribute =
            bool(prim.GetAttribute(_weightFieldTokens->planeBounds));
        object.fieldLeaves.decl.keys.clear();
        for (const TfToken *name :
             {&_weightFieldTokens->sampleSource,
              &_weightFieldTokens->weightTarget, &_weightFieldTokens->curve}) {
            object.fieldLeaves.decl.Add({PointsPath(B.stage, prim, *name),
                                         RigExecRevisionLeafType::Vec3fArray,
                                         RigExecRevisionLeafTime::AtTime,
                                         RigExecRevisionLeafFlavour::Raw,
                                         VtValue()});
        }
        for (const TfToken *name : {&_weightFieldTokens->planeAxis,
                                    &_weightFieldTokens->planeBounds}) {
            object.fieldLeaves.decl.Add({object.path.AppendProperty(*name),
                                         RigExecRevisionLeafType::Token,
                                         RigExecRevisionLeafTime::AtTime,
                                         RigExecRevisionLeafFlavour::Raw,
                                         VtValue()});
        }
        object.fieldFalloffSamplesLeaf = -1;
        if (object.fieldKind >= 3 && object.fieldKind <= 5) {
            object.fieldFalloffSamplesLeaf =
                int(object.fieldLeaves.decl.keys.size());
            object.fieldLeaves.decl.Add(
                {object.path.AppendProperty(_weightFieldTokens->falloffSamples),
                 RigExecRevisionLeafType::FloatArray,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly,
                 VtValue(VtFloatArray())});
        }
        object.fieldFrozenKeys.clear();
        for (size_t k = 0; k < object.fieldLeaves.decl.keys.size(); ++k)
            object.fieldFrozenKeys.push_back(
                object.path.AppendProperty(_fieldLeafKeys[k]));
        RigExecBakedBindPathLeaves(B.stage, &object.fieldLeaves);
        RigExecWeightRecord record;
        record.name = object.fieldName;
        record.kind = object.fieldKind;
        record.representation = object.fieldRepresentation;
        record.combineMode = object.fieldCombineMode;
        record.type = object.type;
        record.representationToken = object.representation;
        record.rangePolicy = object.rangePolicy;
        record.combineToken = object.combineMode;
        record.planeAxis = object.planeAxis;
        record.planeBounds = object.planeBounds;
        record.clamp = object.fieldClamp;
        record.base = object.base;
        record.inputs = object.inputs;
        record.values = object.values;
        record.indices = object.indices;
        record.falloffCurve = object.falloffCurve;
        record.staticError = object.fieldFacts.staticError;
        record.phaseError = object.fieldFacts.phaseError;
        record.samplesInFlight = object.fieldFacts.samplesInFlight;
        B.weightProgram.push_back(std::move(record));
    }
    std::string placementError;
    RigExecBakedWeightScalarReadIndex readIndex;
    const auto add = [&](const SdfPath &path, Field::Form form, int consumer,
                         int part) {
        const auto found = B.weightIndex.find(path);
        if (found == B.weightIndex.end() || found->second < 0)
            return -1;
        Field field;
        field.object = found->second;
        field.form = form;
        field.scalarReadIndex.assign(B.weightObjects.size() * 19, -1);
        field.consumer = consumer;
        field.part = part;
        SdfPath reader;
        if (form == Field::Form::Revision) {
            const auto [chain, revision] = B.revisionIndex[size_t(consumer)];
            reader =
                B.chains[size_t(chain)].revisions[size_t(revision)].moverPath;
        } else if (form == Field::Form::EnvelopeProperty)
            reader = B.propertyChains[size_t(consumer)]
                         .revisions[size_t(part - 1)]
                         .mover;
        else
            reader =
                B.constraints[size_t(B.walkSteps[size_t(consumer)].index)].path;
        RigExecReadPhase placement;
        std::string why;
        const auto relation = B.stage->GetPrimAtPath(reader).GetRelationship(
            _weightFieldTokens->weightObject);
        if (!RigExecResolveReadPhase(relation, &placement, &why) ||
            (placement.kind != RigExecReadPhaseKind::Base &&
             placement.kind != RigExecReadPhaseKind::Final)) {
            placementError = pathText(reader) +
                             ": weight placement must read base or final" +
                             (why.empty() ? std::string() : ": " + why);
            return -1;
        }
        field.placementPhase = placement.kind == RigExecReadPhaseKind::Final
                                   ? Field::PlacementPhase::Final
                                   : Field::PlacementPhase::Base;
        // Weight-field reads observe the generation publication boundary,
        // not every retained final in the canonical graph. Property envelopes
        // run before their own chain finishes; only earlier semantic chains
        // have published. Rest-tier consumers observe the completed head tier.
        const size_t publishedEnd = form == Field::Form::EnvelopeProperty
                                        ? size_t(consumer)
                                        : B.propertyChains.size();
        for (size_t c = 0; c < publishedEnd; ++c)
            field.availableChains.push_back(int(c));
        std::set<int> objects;
        const auto visit = [&](const auto &self, int object) -> void {
            if (object < 0 || !objects.insert(object).second)
                return;
            const auto &weight = B.weightObjects[size_t(object)];
            self(self, weight.base);
            for (int input : weight.inputs)
                self(self, input);
            if (weight.providerSlot >= 0)
                field.volumes.push_back(weight.providerSlot);
            const auto prim = B.stage->GetPrimAtPath(weight.path);
            for (int member = 0; member < 19; ++member) {
                const bool used =
                    member == 0 ||
                    (weight.fieldKind == 1 && member >= 1 && member <= 3) ||
                    (weight.fieldKind == 2 && (member == 4 || member == 5)) ||
                    (weight.fieldKind == 3 && member >= 4 && member <= 16) ||
                    (weight.fieldKind == 4 &&
                     ((member >= 4 && member <= 7) || member >= 17)) ||
                    (weight.fieldKind == 5 && member >= 4 && member <= 10);
                if (!used)
                    continue;
                field.scalarReadIndex[size_t(object) * 19 + size_t(member)] =
                    int(field.scalarReads.size());
                field.scalarObjects.push_back(object);
                field.scalarMembers.push_back(member);
                field.scalarReads.push_back(RigExecBakedBuildWeightScalarRead(
                    &B, prim.GetAttribute(ScalarNames[member]),
                    field.availableChains, &readIndex));
            }
        };
        visit(visit, field.object);
        field.objects.assign(objects.begin(), objects.end());
        std::vector<char> mark(B.weightObjects.size(), 0);
        const auto order = [&](const auto &self, int object) -> void {
            if (object < 0 || size_t(object) >= mark.size() ||
                mark[size_t(object)] == 2)
                return;
            if (mark[size_t(object)] == 1) {
                field.cyclic = true;
                return;
            }
            mark[size_t(object)] = 1;
            const auto &weight = B.weightObjects[size_t(object)];
            if (weight.fieldKind == eval::kWeightKindDynamic)
                self(self, weight.base);
            if (weight.fieldKind == eval::kWeightKindCombine)
                for (int input : weight.inputs)
                    self(self, input);
            mark[size_t(object)] = 2;
            field.nodeObjects.push_back(object);
        };
        order(order, field.object);
        for (auto &read : field.scalarReads)
            for (auto &hop : read.walk.hops) {
                const auto pose = B.poseWeightIndex.find(hop.path);
                if (pose != B.poseWeightIndex.end())
                    hop.poseWeight = pose->second;
            }
        std::sort(field.volumes.begin(), field.volumes.end());
        field.volumes.erase(
            std::unique(field.volumes.begin(), field.volumes.end()),
            field.volumes.end());
        B.weightFields.push_back(std::move(field));
        return int(B.weightFields.size()) - 1;
    };
    for (size_t c = 0; c < B.propertyChains.size(); ++c)
        for (size_t k = 0; k < B.propertyChains[c].revisions.size(); ++k) {
            auto &revision = B.propertyChains[c].revisions[k];
            if (!revision.weightObject.IsEmpty())
                revision.weightField =
                    add(revision.weightObject, Field::Form::EnvelopeProperty,
                        int(c), int(k + 1));
        }
    for (size_t w = 0; w < B.walkSteps.size(); ++w) {
        const auto &walk = B.walkSteps[w];
        if (walk.solverBatch || walk.index < 0)
            continue;
        auto &constraint = B.constraints[size_t(walk.index)];
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty())
            constraint.weightField =
                add(constraint.weightObject, Field::Form::EnvelopeConstraint,
                    int(w), 0);
    }
    for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
        const auto [c, k] = B.revisionIndex[r];
        auto &revision = B.chains[size_t(c)].revisions[size_t(k)];
        if (revision.weightObject >= 0 && revision.weightCurrentPhase) {
            revision.weightField =
                add(B.weightObjects[size_t(revision.weightObject)].path,
                    Field::Form::Revision, int(r), 0);
        }
    }
    if (!placementError.empty()) {
        if (error)
            *error = placementError;
        return false;
    }
    for (const auto &field : B.weightFields) {
        if (field.scalarReads.size() != field.scalarObjects.size() ||
            field.scalarReads.size() != field.scalarMembers.size()) {
            if (error)
                *error = "weight field scalar provenance is incomplete";
            return false;
        }
    }
    B.headOverrides.resize(B.headOverrideSlots.size());
    B.lastHeadOverrides.resize(B.headOverrideSlots.size());
    B.headOverrideMoved.resize(B.headOverrideSlots.size(), 0);
    B.volumePlacementBase.assign(B.paths.size(), GfMatrix4d(1.0));
    for (size_t f = 0; f < B.weightFields.size(); ++f)
        if (B.weightFields[f].form != Field::Form::Revision)
            RigExecBakedAddWeightFieldSteps(&B, int(f), true);
    for (auto &step : B.steps) {
        size_t chain = 0, part = 0;
        if (!RigExecBakedPropertyPartOf(B, step, &chain, &part))
            continue;
        auto &revision = B.propertyChains[chain].revisions[part - 1];
        if (revision.weightField >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::WeightField, revision.weightField));
            step.alwaysRuns = false;
        }
    }
    return true;
}
int
RigExecBakedWeightFieldOf(const Baked &B, const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::WeightField
               ? origin->object
               : -1;
}
void
RigExecBakedAddWeightFieldSteps(Baked *program, int index, bool head)
{
    auto &B = *program;
    const size_t nodes = B.weightFields[size_t(index)].nodeObjects.size();
    for (size_t i = 0; i < nodes; ++i) {
        RigExecBakedOpOrigin origin;
        origin.family = RigExecBakedOpFamily::WeightField;
        origin.object = index;
        origin.part = int32_t(i);
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::OpNode;
        step.object = int(B.opNodeOrigins.size());
        step.maxDiagnostics = 1;
        if (head) {
            step.isHead = true;
            step.alwaysRuns = false;
        }
        B.opNodeOrigins.push_back(origin);
        RigExecBakedDeclareWeightField(&B, &step);
        B.steps.push_back(std::move(step));
    }
}
void
RigExecBakedDeclareWeightField(Baked *program, RigExecBakedStep *step)
{
    const int index = RigExecBakedWeightFieldOf(*program, *step);
    if (index < 0)
        return;
    const auto &field = program->weightFields[size_t(index)];
    const int part = program->opNodeOrigins[size_t(step->object)].part;
    if (size_t(part) + 1 == field.nodeObjects.size())
        step->writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightField, index));
    const std::vector<char> objects =
        RigExecBakedWeightFieldNodeObjects(*program, index, part);
    for (size_t r = 0; r < field.scalarReads.size(); ++r) {
        if (!objects[size_t(field.scalarObjects[r])])
            continue;
        const auto &read = field.scalarReads[r];
        for (uint32_t version : read.versions)
            step->reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PropertyResult, int(version)));
        step->leaves.insert(step->leaves.end(), read.leaves.begin(),
                            read.leaves.end());
        step->overrideSlots.insert(step->overrideSlots.end(),
                                   read.slots.begin(), read.slots.end());
        for (const auto &hop : read.walk.hops)
            if (hop.poseWeight >= 0)
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseWeight, hop.poseWeight));
        for (uint32_t leaf : read.leaves)
            if (program->headLeaves[leaf].varying)
                step->varyingInputs = true;
    }
    // Raw field arrays participate in the same numbered leaf census as
    // revision inputs, including private sampled arrays and failed reads.
    for (size_t i = 0; i < program->pathLeafRefs.size(); ++i) {
        const auto &ref = program->pathLeafRefs[i];
        if (ref.owner != RigExecBakedPathLeafOwner::WeightField ||
            ref.a >= objects.size() || !objects[ref.a])
            continue;
        const auto &leaves = program->weightObjects[ref.a].fieldLeaves;
        step->bindingLeaves.push_back(uint32_t(program->leafRefs.size() + i));
        if (ref.key < leaves.varying.size() && leaves.varying[ref.key]) {
            step->varyingInputs = true;
            step->varyingLeaves = true;
        }
    }
    std::sort(step->bindingLeaves.begin(), step->bindingLeaves.end());
    step->bindingLeaves.erase(
        std::unique(step->bindingLeaves.begin(), step->bindingLeaves.end()),
        step->bindingLeaves.end());
    if (field.form == Field::Form::Revision) {
        const auto [c, k] = program->revisionIndex[size_t(field.consumer)];
        if (k > 0 &&
            RigExecBakedWeightFieldNodeReadsChain(*program, index, objects)) {
            const int previous = field.consumer - 1;
            step->reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::RevisionDone, previous));
            step->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, previous));
            for (int earlier = 0; earlier < k; ++earlier) {
                const int revision =
                    program->chainRevisionBegin[size_t(c)] + earlier;
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone, revision));
            }
            step->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
        } else
            step->reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
    }
    for (int object : field.objects) {
        const int slot = program->weightObjects[size_t(object)].providerSlot;
        if (slot >= 0 && objects[size_t(object)])
            step->reads.push_back(RigExecBakedOne(
                field.placementPhase == Field::PlacementPhase::Base
                    ? RigExecBakedSlotDomain::WeightFramesBase
                    : RigExecBakedSlotDomain::WeightFrames,
                slot));
    }
    // Declaration is repeated after input numbering; keep producer ranges unique.
    std::sort(step->reads.begin(), step->reads.end());
    step->reads.erase(std::unique(step->reads.begin(), step->reads.end()),
                      step->reads.end());
    std::sort(step->writes.begin(), step->writes.end());
    step->writes.erase(std::unique(step->writes.begin(), step->writes.end()),
                       step->writes.end());
    std::sort(step->leaves.begin(), step->leaves.end());
    step->leaves.erase(std::unique(step->leaves.begin(), step->leaves.end()),
                       step->leaves.end());
    std::sort(step->overrideSlots.begin(), step->overrideSlots.end());
    step->overrideSlots.erase(
        std::unique(step->overrideSlots.begin(), step->overrideSlots.end()),
        step->overrideSlots.end());
}
std::vector<char>
RigExecBakedWeightFieldNodeObjects(const Baked &B, int index, int part)
{
    const auto &field = B.weightFields[size_t(index)];
    std::vector<char> objects(B.weightObjects.size(), 0);
    if (size_t(part) + 1 == field.nodeObjects.size()) {
        for (int object : field.objects)
            objects[size_t(object)] = 1;
        return objects;
    }
    const auto visit = [&](const auto &self, int object) -> void {
        if (object < 0 || size_t(object) >= objects.size() ||
            objects[size_t(object)])
            return;
        objects[size_t(object)] = 1;
        const auto &weight = B.weightObjects[size_t(object)];
        if (weight.fieldKind == eval::kWeightKindDynamic)
            self(self, weight.base);
        if (weight.fieldKind == eval::kWeightKindCombine)
            for (int input : weight.inputs)
                self(self, input);
    };
    visit(visit, field.nodeObjects[size_t(part)]);
    return objects;
}
bool
RigExecBakedWeightFieldNodeReadsChain(const Baked &B, int index,
                                      const std::vector<char> &objects)
{
    if (B.weightFields[size_t(index)].form != Field::Form::Revision)
        return false;
    for (size_t object = 0; object < objects.size(); ++object)
        if (objects[object] &&
            B.weightObjects[object].fieldFacts.samplesInFlight)
            return true;
    return false;
}
void
RigExecBakedWeightFieldCoveredVersions(const Baked &B, int index,
                                       std::vector<uint32_t> *versions)
{
    if (!versions || index < 0 || size_t(index) >= B.weightFields.size())
        return;
    for (const auto &read : B.weightFields[size_t(index)].scalarReads)
        versions->insert(versions->end(), read.versions.begin(),
                         read.versions.end());
}
namespace {
bool
FieldFail(std::string *error, const std::string &why)
{
    if (error)
        *error = why;
    return false;
}

template <class T>
bool
PutFieldValue(const T &value, const RigExecTypeDesc &type, void *out)
{
    if (type.size != sizeof(T) || !type.triviallyCopyable)
        return false;
    std::memcpy(out, &value, sizeof(T));
    return true;
}

bool
LendFieldArray(const void *data, size_t count, const RigExecTypeDesc &type,
               void *out)
{
    if (!type.borrow)
        return false;
    type.borrow(out, data, count);
    return true;
}

/// How many points field \p field weighs this frame: one for an envelope,
/// the entering points of its revision's chain otherwise.
bool
FieldCount(const Baked &B, const Field &field, size_t *count,
           std::string *error)
{
    *count = 1;
    if (field.form != Field::Form::Revision)
        return true;
    const auto [c, k] = B.revisionIndex[size_t(field.consumer)];
    const auto &chain = B.chains[size_t(c)];
    if (!chain.haveBase)
        return FieldFail(error, "point chain " + std::to_string(c) +
                                    " has no points this frame");
    *count = chain.lastBase.size();
    return true;
}

/// Whether \p w's weight target is read at a phase other than base.
bool
PhasedTarget(const Baked &B, const Baked::WeightObject &w)
{
    const auto relation = B.stage->GetPrimAtPath(w.path).GetRelationship(
        _weightFieldTokens->weightTarget);
    if (!relation ||
        !relation.HasAuthoredMetadata(_weightFieldTokens->readPhase))
        return false;
    RigExecReadPhase phase;
    std::string why;
    return RigExecResolveReadPhase(relation, &phase, &why) && !phase.IsBase();
}

/// What makes weight object \p object of \p field unable to weigh any
/// frame, or empty.
std::string
FieldStructureError(const Baked &B, const Field &field, size_t object)
{
    const auto &w = B.weightObjects[object];
    const auto &record = B.weightProgram[object];
    const std::string &who = record.name;
    if (field.cyclic)
        return "operation cycle: " + who +
               " composes a weight that composes it";
    if (!record.staticError.empty())
        return record.staticError;
    switch (record.kind) {
    case eval::kWeightKindStatic: {
        if (record.representation < eval::kWeightConstant ||
            record.representation > eval::kWeightSparse)
            return "unknown weight representation on " + who;
        if (record.representation == eval::kWeightConstant &&
            !record.values.empty())
            return "constant weight must not author values on " + who;
        if (record.representation != eval::kWeightSparse)
            return {};
        if (record.indices.size() != record.values.size())
            return "sparse index/value size mismatch on " + who;
        std::set<int> seen;
        for (const int index : record.indices) {
            if (index < 0)
                return "sparse index out of range on " + who;
            if (!seen.insert(index).second)
                return "duplicate sparse index on " + who;
        }
        return {};
    }
    case eval::kWeightKindDynamic:
        return {};
    case eval::kWeightKindCombine:
        if (record.combineMode < 0 ||
            record.combineMode > eval::kWeightCombineOverlay)
            return who + ": unknown rigExec:combineMode " +
                   record.combineToken.GetString();
        return {};
    case eval::kWeightKindSphere:
    case eval::kWeightKindPlane:
    case eval::kWeightKindCurve: {
        const int slot = w.providerSlot;
        if (slot < 0 || size_t(slot) >= B.placedVolumes.size() ||
            !B.placedVolumes[size_t(slot)])
            return who + ": no resolved placement for this volume weight";
        if (!record.phaseError.empty())
            return record.phaseError;
        if (w.fieldFacts.samplesInFlight && !PhasedTarget(B, w) &&
            field.form != Field::Form::Revision)
            return who + ": rigExec:weightTarget reads `preceding` but no "
                         "in-flight points were supplied";
        return {};
    }
    default:
        return "unknown weight object type " + record.type.GetString() +
               " on " + who;
    }
}

/// The points volume \p object of \p field samples this frame: its phased
/// target, else its sample source, else its raw target.
bool
SamplePoints(const Baked &B, int index, const Field &field, int object,
             RigExecBakedPointInputValue *points, std::string *error)
{
    const auto &w = B.weightObjects[size_t(object)];
    const std::string &who = B.weightProgram[size_t(object)].name;
    const auto raw = [&](int key) {
        RigExecBakedPointInputValue value;
        const auto &values = w.fieldLeaves.values;
        if (size_t(key) < values.size() &&
            values[size_t(key)].IsHolding<VtVec3fArray>()) {
            const auto &array =
                values[size_t(key)].UncheckedGet<VtVec3fArray>();
            value.available = true;
            value.data = array.cdata();
            value.count = array.size();
        }
        return value;
    };
    *points = RigExecBakedReadWeightPointInput(B, index, object, 1);
    if (!points->declared) {
        if (w.fieldFacts.samplesInFlight)
            return FieldFail(error, who + ": rigExec:weightTarget reads "
                                          "`preceding` but no in-flight "
                                          "points were supplied");
        *points = RigExecBakedReadWeightPointInput(B, index, object, 0);
        if (!points->declared)
            *points = raw(0);
        if (!points->available)
            *points = raw(1);
    }
    if (!points->available)
        return FieldFail(error, who + ": could not read the points to sample");
    size_t count = 0;
    if (!FieldCount(B, field, &count, error))
        return false;
    if (points->count != count)
        return FieldFail(
            error, who + ": sampled point count does not match the target");
    return true;
}

} // namespace

bool
RigExecBakedLookUpWeightField(const Baked &B, const eval::OpLookup &lookup,
                              const RigExecTypeDesc &type, void *out,
                              std::string *error)
{
    using R = eval::OpFieldRead;
    if (lookup.object >= B.weightFields.size() || lookup.part < 0 ||
        size_t(lookup.part) >= B.weightObjects.size())
        return FieldFail(error, "weight field " +
                                    std::to_string(lookup.object) +
                                    " reads no weight object " +
                                    std::to_string(lookup.part));
    const int index = int(lookup.object);
    const Field &field = B.weightFields[lookup.object];
    const int object = lookup.part;
    const auto &w = B.weightObjects[size_t(object)];
    const auto &record = B.weightProgram[size_t(object)];
    const auto scalar = [&](int member) {
        return Scalar(B, field, object, member,
                      eval::kOpFieldScalarDefaults[size_t(member)]);
    };
    bool ok = false;
    if (lookup.read <= uint8_t(R::LastScalar)) {
        ok = PutFieldValue(scalar(lookup.read), type, out);
    } else if (lookup.read <= uint8_t(R::NegativeScales)) {
        const int first = 8 + 3 * (lookup.read - uint8_t(R::ScaleAxes));
        ok = PutFieldValue(
            GfVec3f(scalar(first), scalar(first + 1), scalar(first + 2)), type,
            out);
    } else if (lookup.read == uint8_t(R::SamplePoints)) {
        RigExecBakedPointInputValue points;
        if (!SamplePoints(B, index, field, object, &points, error))
            return false;
        ok = LendFieldArray(points.data, points.count, type, out);
    } else if (lookup.read == uint8_t(R::CurvePoints)) {
        RigExecBakedPointInputValue curve =
            RigExecBakedReadWeightPointInput(B, index, object, 2);
        if (!curve.declared) {
            const auto &values = w.fieldLeaves.values;
            curve = {};
            if (values.size() > 2 && values[2].IsHolding<VtVec3fArray>()) {
                const auto &array = values[2].UncheckedGet<VtVec3fArray>();
                curve.available = true;
                curve.data = array.cdata();
                curve.count = array.size();
            }
        }
        ok = LendFieldArray(curve.available ? curve.data : nullptr,
                            curve.available ? curve.count : 0, type, out);
    } else if (lookup.read == uint8_t(R::FalloffSamples)) {
        const int leaf = w.fieldFalloffSamplesLeaf;
        const auto &values = w.fieldLeaves.values;
        const VtFloatArray *samples =
            leaf >= 0 && size_t(leaf) < values.size() &&
                    values[size_t(leaf)].IsHolding<VtFloatArray>()
                ? &values[size_t(leaf)].UncheckedGet<VtFloatArray>()
                : nullptr;
        ok = samples && !samples->empty()
                 ? LendFieldArray(samples->cdata(), samples->size(), type, out)
                 : LendFieldArray(record.falloffCurve.data(),
                                  record.falloffCurve.size(), type, out);
    } else if (lookup.read == uint8_t(R::Placement)) {
        const auto &placements =
            field.placementPhase == Field::PlacementPhase::Base
                ? B.volumePlacementBase
                : B.volumePlacement;
        const size_t slot = size_t(w.providerSlot);
        if (w.providerSlot < 0 || slot >= placements.size())
            return FieldFail(error, record.name +
                                        ": no resolved placement for this "
                                        "volume weight");
        ok = type.size == sizeof(GfMatrix4d) && type.triviallyCopyable;
        if (ok)
            std::memcpy(out, placements[slot].data(), sizeof(GfMatrix4d));
    } else if (lookup.read == uint8_t(R::TargetCount)) {
        size_t count = 0;
        if (!FieldCount(B, field, &count, error))
            return false;
        ok = PutFieldValue(uint32_t(count), type, out);
    } else if (lookup.read == uint8_t(R::PlaneAxis) ||
               lookup.read == uint8_t(R::PlaneBounded)) {
        const bool bounds = lookup.read == uint8_t(R::PlaneBounded);
        const auto &values = w.fieldLeaves.values;
        const size_t key = bounds ? 4 : 3;
        std::string name;
        if (key < values.size() && values[key].IsHolding<TfToken>()) {
            const TfToken &token = values[key].UncheckedGet<TfToken>();
            if (!token.IsEmpty())
                name = token.GetString();
        } else if (!(bounds ? w.fieldBoundsAttribute : w.fieldAxisAttribute)) {
            name = bounds ? SchemaValueText::unbounded : SchemaValueText::x;
        }
        uint32_t code = 0;
        if (bounds) {
            if (name != SchemaValueText::bounded &&
                name != SchemaValueText::unbounded)
                return FieldFail(
                    error, RigExecUnknownPlaneBoundsMessage(record.name, name));
            code = name == SchemaValueText::bounded ? 1 : 0;
        } else {
            const int axis = eval::WeightAxis(name);
            if (axis == eval::kWeightPlaneAxisUnknown)
                return FieldFail(
                    error, record.name + ": unknown rigExec:planeAxis " + name);
            code = uint32_t(axis);
        }
        ok = PutFieldValue(code, type, out);
    } else if (lookup.read == uint8_t(R::Values)) {
        ok = LendFieldArray(record.values.data(), record.values.size(), type,
                            out);
    } else if (lookup.read == uint8_t(R::Indices)) {
        static_assert(sizeof(int) == sizeof(int32_t), "indices are int32");
        ok = LendFieldArray(record.indices.data(), record.indices.size(), type,
                            out);
    }
    if (!ok)
        return FieldFail(error,
                         "weight field " + std::to_string(lookup.object) +
                             " has no input " + std::to_string(lookup.read) +
                             " of the looked-up type");
    return true;
}

bool
RigExecBakedLookUpChainVersion(Baked &B, uint32_t index,
                               const eval::OpLookup &lookup,
                               const RigExecTypeDesc &type, void *out,
                               std::string *error)
{
    if (lookup.object >= B.chains.size() || lookup.part < 0 ||
        size_t(lookup.part) > B.chains[lookup.object].revisions.size() ||
        index >= B.chainVersionGather.size())
        return FieldFail(error, "point chain " + std::to_string(lookup.object) +
                                    " has no version " +
                                    std::to_string(lookup.part));
    const auto &chain = B.chains[lookup.object];
    if (!chain.haveBase)
        return FieldFail(error, "point chain " + std::to_string(lookup.object) +
                                    " has no points this frame");
    const GfVec3f *points = nullptr;
    size_t count = 0;
    RigExecBakedVersionPoints(chain, size_t(lookup.part),
                              &B.chainVersionGather[index], &points, &count);
    if (!LendFieldArray(points, count, type, out))
        return FieldFail(error, "point chain " + std::to_string(lookup.object) +
                                    " is looked up as no point array");
    return true;
}

bool
RigExecBakedStoreWeightField(Baked &B, const eval::OpHostOutput &output,
                             const void *value, std::string *error)
{
    if (output.object >= B.weightFields.size())
        return FieldFail(error,
                         "no weight field " + std::to_string(output.object));
    const auto &in =
        *static_cast<const rigExecStd::RigExecArray<float> *>(value);
    Field &field = B.weightFields[output.object];
    const size_t count = in.Size();
    // Equality preserves signed-zero bits.
    const bool same = field.ok && field.error.empty() && field.count == count &&
                      field.values.size() == count &&
                      (count == 0 || std::memcmp(field.values.data(), in.Data(),
                                                 count * sizeof(float)) == 0);
    field.changed = !same;
    field.values.assign(in.Data(), in.Data() + count);
    field.count = count;
    field.ok = true;
    field.error.clear();
    return true;
}

bool
RigExecBakedAddWeightFieldOps(const Baked &B, RigExecRigDescBuilder *builder,
                              eval::OpStepLayout *layout, bool revisions,
                              std::string *error)
{
    using R = eval::OpFieldRead;
    using namespace rigExecOps;
    std::vector<RigExecNodeId> nodeOf;
    std::vector<const RigExecPortHandle *> outOf;
    int current = -1;
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o) {
        const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
        if (origin.family != RigExecBakedOpFamily::WeightField)
            continue;
        const int f = origin.object;
        const Field &field = B.weightFields[size_t(f)];
        if ((field.form == Field::Form::Revision) != revisions)
            continue;
        if (f != current) {
            current = f;
            nodeOf.assign(field.nodeObjects.size(), kRigExecInvalidIndex);
            outOf.assign(field.nodeObjects.size(), nullptr);
        }
        const size_t part = size_t(origin.part);
        const int object = field.nodeObjects[part];
        const auto &w = B.weightObjects[size_t(object)];
        const auto &record = B.weightProgram[size_t(object)];
        const std::string why = FieldStructureError(B, field, size_t(object));
        if (!why.empty())
            return FieldFail(error, why);
        const auto lookAt = [&](RigExecNodeId node,
                                const RigExecPortHandle &port, int from,
                                R read) {
            builder->BindInput(node, port);
            layout->hostInputs.push_back({node, port.Index(),
                                          eval::OpHostSource::Lookup,
                                          uint32_t(layout->lookups.size())});
            layout->lookups.push_back(
                {uint8_t(eval::OpLookupSource::WeightFieldInputs), uint32_t(f),
                 int32_t(from), uint8_t(read)});
        };
        const auto look = [&](RigExecNodeId node, const RigExecPortHandle &port,
                              R read) { lookAt(node, port, object, read); };
        const auto scalar = [&](RigExecNodeId node,
                                const RigExecPortHandle &port,
                                int member) { look(node, port, R(member)); };
        const auto feed = [&](int from, RigExecNodeId node,
                              const RigExecPortHandle &port) {
            const auto at = std::find(field.nodeObjects.begin(),
                                      field.nodeObjects.end(), from);
            const size_t i = size_t(at - field.nodeObjects.begin());
            if (at == field.nodeObjects.end() ||
                nodeOf[i] == kRigExecInvalidIndex)
                return false;
            builder->Connect(nodeOf[i], *outOf[i], node, port);
            return true;
        };
        const bool clamp = record.clamp;
        RigExecNodeId node = kRigExecInvalidIndex;
        const RigExecPortHandle *out = nullptr;
        bool fed = true;
        switch (record.kind) {
        case eval::kWeightKindStatic: {
            const WeightPacketPayload payload{
                rigExecRig::WeightRepresentation(record.representation), clamp};
            node = builder->AddOp(StaticWeightField(), &payload);
            look(node, StaticWeightFieldValues(), R::Values);
            if (record.representation == eval::kWeightSparse)
                look(node, StaticWeightFieldIndices(), R::Indices);
            scalar(node, StaticWeightFieldDefaultWeight(), 0);
            look(node, StaticWeightFieldTargetCount(), R::TargetCount);
            out = &StaticWeightFieldOut();
            break;
        }
        case eval::kWeightKindDynamic: {
            const WeightPacketPayload payload{
                rigExecRig::WeightRepresentation::Dense, clamp};
            node = builder->AddOp(DynamicWeightField(), &payload);
            if (w.base >= 0) {
                fed = feed(w.base, node, DynamicWeightFieldBase());
                if (record.representation == eval::kWeightSparse) {
                    look(node, DynamicWeightFieldIndices(), R::Indices);
                    lookAt(node, DynamicWeightFieldBaseIndices(), w.base,
                           R::Indices);
                }
            } else
                look(node, DynamicWeightFieldTargetCount(), R::TargetCount);
            scalar(node, DynamicWeightFieldDriver(), 1);
            scalar(node, DynamicWeightFieldScale(), 2);
            scalar(node, DynamicWeightFieldBias(), 3);
            out = &DynamicWeightFieldOut();
            break;
        }
        case eval::kWeightKindCombine: {
            CombineWeightFieldsPayload payload;
            payload.mode = WeightCombineMode(record.combineMode);
            payload.clamp = clamp;
            node = builder->AddOp(CombineWeightFields(), &payload);
            for (const int input : w.inputs)
                fed = fed && feed(input, node, CombineWeightFieldsInputs());
            look(node, CombineWeightFieldsTargetCount(), R::TargetCount);
            scalar(node, CombineWeightFieldsStrength(), 4);
            scalar(node, CombineWeightFieldsInvert(), 5);
            out = &CombineWeightFieldsOut();
            break;
        }
        default: {
            WeightFieldPayload payload;
            payload.clamp = clamp;
            const RigExecOpHandle &op =
                record.kind == eval::kWeightKindSphere  ? SphereWeight()
                : record.kind == eval::kWeightKindPlane ? PlaneWeight()
                                                        : CurveWeight();
            node = builder->AddOp(op, &payload);
            const bool sphere = record.kind == eval::kWeightKindSphere;
            const bool plane = record.kind == eval::kWeightKindPlane;
            const RigExecPortHandle &points = sphere  ? SphereWeightPoints()
                                              : plane ? PlaneWeightPoints()
                                                      : CurveWeightPoints();
            if (w.fieldFacts.samplesInFlight && !PhasedTarget(B, w)) {
                const auto [c, k] = B.revisionIndex[size_t(field.consumer)];
                builder->BindInput(node, points);
                layout->hostInputs.push_back(
                    {node, points.Index(), eval::OpHostSource::Lookup,
                     uint32_t(layout->lookups.size())});
                layout->lookups.push_back(
                    {uint8_t(eval::OpLookupSource::ChainVersion), uint32_t(c),
                     int32_t(k), 0});
            } else {
                look(node, points, R::SamplePoints);
            }
            look(node,
                 sphere  ? SphereWeightPlacement()
                 : plane ? PlaneWeightPlacement()
                         : CurveWeightPlacement(),
                 R::Placement);
            scalar(node,
                   sphere  ? SphereWeightStrength()
                   : plane ? PlaneWeightStrength()
                           : CurveWeightStrength(),
                   4);
            scalar(node,
                   sphere  ? SphereWeightInvert()
                   : plane ? PlaneWeightInvert()
                           : CurveWeightInvert(),
                   5);
            scalar(node,
                   sphere  ? SphereWeightFalloffMin()
                   : plane ? PlaneWeightFalloffMin()
                           : CurveWeightFalloffMin(),
                   6);
            scalar(node,
                   sphere  ? SphereWeightFalloffMax()
                   : plane ? PlaneWeightFalloffMax()
                           : CurveWeightFalloffMax(),
                   7);
            look(node,
                 sphere  ? SphereWeightFalloffSamples()
                 : plane ? PlaneWeightFalloffSamples()
                         : CurveWeightFalloffSamples(),
                 R::FalloffSamples);
            if (sphere) {
                look(node, SphereWeightScale(), R::ScaleAxes);
                look(node, SphereWeightPositiveScale(), R::PositiveScales);
                look(node, SphereWeightNegativeScale(), R::NegativeScales);
                out = &SphereWeightOut();
            } else if (plane) {
                look(node, PlaneWeightAxis(), R::PlaneAxis);
                look(node, PlaneWeightBounded(), R::PlaneBounded);
                scalar(node, PlaneWeightExtentU(), 17);
                scalar(node, PlaneWeightExtentV(), 18);
                out = &PlaneWeightOut();
            } else {
                look(node, CurveWeightScale(), R::ScaleAxes);
                look(node, CurveWeightCurve(), R::CurvePoints);
                out = &CurveWeightOut();
            }
            break;
        }
        }
        if (!fed)
            return FieldFail(error, record.name +
                                        ": composes a weight its field "
                                        "does not weigh");
        if (node != RigExecNodeId(o))
            return FieldFail(error, "weight field " + std::to_string(f) +
                                        ": node " + std::to_string(node) +
                                        " is out of origin order");
        nodeOf[part] = node;
        outOf[part] = out;
        if (part + 1 != field.nodeObjects.size())
            continue;
        builder->BindOutput(node, *out);
        layout->hostOutputs.push_back(
            {node, out->Index(), uint8_t(eval::OpOutputTarget::WeightField),
             uint32_t(f)});
    }
    return true;
}

void
RigExecBakedResetWeightField(Baked *program, int index)
{
    if (index < 0 || size_t(index) >= program->weightFields.size())
        return;
    auto &field = program->weightFields[size_t(index)];
    field.values.clear();
    field.count = 0;
    field.ok = false;
    field.error = "operation cycle";
    field.changed = true;
}
void
RigExecBakedBindWeightCycleState(Baked *program)
{
    auto &B = *program;
    B.weightCycleBlocked.assign(B.weightObjects.size(), 0);
    for (auto id : B.opAdapter.excludedValues) {
        if (id >= B.opAdapter.values.size())
            continue;
        const auto &value = B.opAdapter.values[size_t(id)];
        if (value.domain == uint32_t(RigExecBakedSlotDomain::WeightPacket) &&
            value.slot < B.weightCycleBlocked.size())
            B.weightCycleBlocked[value.slot] = 1;
    }
}
} // namespace rigExec
