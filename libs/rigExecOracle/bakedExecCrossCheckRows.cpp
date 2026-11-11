#include "rigExecSchema/schemaTokens.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/goldenPose.h"
#include "rigExec/solverKernels.h"
#include "rigExec/weightPackets.h"
#include "rigExec/independentConstraintCheck.h"
#include "rigExecScene/startFrameDerivation.h"
#include "pxr/base/vt/array.h"
#include <algorithm>
#include <limits>
#include <set>

namespace rigExec {
namespace {
using B = RigExecBakedProgramImpl;
using Kind = RigExecBakedStepKind;
using Input = RigExecExecCheckInput;
// Constructed before any worker can enter a capture callback.
const std::map<std::string, TfToken, std::less<>> _tokens = [] {
    std::map<std::string, TfToken, std::less<>> result;
    const char *names[] = {SchemaComputationText::computeBasePointFrame,
                           SchemaComputationText::computePointFrame,
                           SchemaComputationText::computeRestFrame,
                           SchemaComputationText::computePointFrameArray,
                           SchemaComputationText::computeWeightPacket,
                           SchemaComputationText::computeFalloffLut,
                           SchemaAttrText::posedSpace,
                           SchemaAttrText::posedDefaultSpace,
                           SchemaAttrText::parentDefaultSpace,
                           SchemaAttrText::parentSpace,
                           SchemaAttrText::avarsTx,
                           SchemaAttrText::avarsTy,
                           SchemaAttrText::avarsTz,
                           SchemaAttrText::avarsSx,
                           SchemaAttrText::avarsSy,
                           SchemaAttrText::avarsSz,
                           SchemaAttrText::avarsRx,
                           SchemaAttrText::avarsRy,
                           SchemaAttrText::avarsRz,
                           SchemaAttrText::avarsRspin,
                           SchemaAttrText::avarsUnitScaleFactor,
                           SchemaAttrText::avarsRotationSign,
                           SchemaAttrText::avarsRotationOrder,
                           SchemaAttrText::restSpace,
                           SchemaAttrText::restTx,
                           SchemaAttrText::restTy,
                           SchemaAttrText::restTz,
                           SchemaAttrText::restRx,
                           SchemaAttrText::restRy,
                           SchemaAttrText::restRz,
                           SchemaAttrText::defaultTx,
                           SchemaAttrText::defaultTy,
                           SchemaAttrText::defaultTz,
                           SchemaAttrText::defaultRx,
                           SchemaAttrText::defaultRy,
                           SchemaAttrText::defaultRz,
                           SchemaAttrText::rigExecControlSpace,
                           SchemaValueText::world,
                           SchemaValueText::parentRelative,
                           SchemaAttrText::rigExecSpaceMatrix,
                           SchemaAttrText::rigExecUpperLengthOffset,
                           SchemaAttrText::rigExecLowerLengthOffset,
                           SchemaAttrText::rigExecPreferredBendRadians,
                           SchemaAttrText::inputsStretch,
                           SchemaAttrText::inputsSoftness,
                           SchemaAttrText::inputsPin,
                           SchemaAttrText::inputsUpperScale,
                           SchemaAttrText::inputsLowerScale,
                           SchemaAttrText::inputsSoftDistance,
                           SchemaAttrText::inputsTwist,
                           SchemaAttrText::rigExecStretchPolicy,
                           SchemaAttrText::rigExecSegmentScale,
                           SchemaAttrText::rigExecScaleCalibration,
                           SchemaValueText::softDistance,
                           SchemaValueText::uniformSegments,
                           SchemaValueText::toChild,
                           SchemaValueText::none,
                           SchemaAttrText::inputsWeight,
                           SchemaAttrText::rigExecScaleBlend,
                           SchemaValueText::log,
                           SchemaValueText::linear,
                           SchemaAttrText::rigExecRotationBlend,
                           SchemaValueText::shortestArc,
                           SchemaAttrText::inputsTwistTurns,
                           SchemaAttrText::rigExecCount,
                           SchemaAttrText::rigExecWeights,
                           SchemaAttrText::rigExecJointElements,
                           SchemaAttrText::inputsPreserveVolume,
                           SchemaAttrText::inputsMidFollowWeight,
                           SchemaAttrText::inputsRoll,
                           SchemaAttrText::inputsTwist,
                           SchemaAttrText::inputsMinLengthRatio,
                           SchemaAttrText::rigExecRootTangent,
                           SchemaValueText::aim,
                           SchemaValueText::rigid,
                           SchemaAttrText::rigExecVolumeWeights,
                           SchemaAttrText::rigExecRestLength,
                           SchemaValueText::curve,
                           SchemaValueText::chain,
                           SchemaAttrText::rigExecRepresentation,
                           SchemaAttrText::rigExecRangePolicy,
                           SchemaAttrText::rigExecDefaultWeight,
                           SchemaAttrText::rigExecValues,
                           SchemaAttrText::rigExecIndices,
                           SchemaAttrText::inputsDriver,
                           SchemaAttrText::inputsScale,
                           SchemaAttrText::inputsBias,
                           SchemaAttrText::rigExecCombineMode,
                           SchemaAttrText::inputsStrength,
                           SchemaAttrText::inputsInvert,
                           SchemaAttrText::inputsFalloffMin,
                           SchemaAttrText::inputsFalloffMax,
                           SchemaAttrText::rigExecPlaneAxis,
                           SchemaAttrText::rigExecPlaneBounds,
                           SchemaAttrText::inputsExtentU,
                           SchemaAttrText::inputsExtentV,
                           SchemaAttrText::inputsScaleX,
                           SchemaAttrText::inputsScaleY,
                           SchemaAttrText::inputsScaleZ,
                           SchemaAttrText::inputsScaleXPos,
                           SchemaAttrText::inputsScaleYPos,
                           SchemaAttrText::inputsScaleZPos,
                           SchemaAttrText::inputsScaleXNeg,
                           SchemaAttrText::inputsScaleYNeg,
                           SchemaAttrText::inputsScaleZNeg,
                           SchemaAttrText::inputsEnabled,
                           SchemaAttrText::inputsDefaultWeight,
                           SchemaAttrText::inputsRotationOffset,
                           SchemaAttrText::inputsAimVector,
                           SchemaAttrText::inputsUpVector,
                           SchemaAttrText::inputsWorldUpVector,
                           SchemaAttrText::inputsAffectTranslationX,
                           SchemaAttrText::inputsAffectTranslationY,
                           SchemaAttrText::inputsAffectTranslationZ,
                           SchemaAttrText::inputsAffectRotationX,
                           SchemaAttrText::inputsAffectRotationY,
                           SchemaAttrText::inputsAffectRotationZ,
                           SchemaAttrText::inputsAffectScaleX,
                           SchemaAttrText::inputsAffectScaleY,
                           SchemaAttrText::inputsAffectScaleZ};
    for (const char *name : names)
        result.emplace(name, TfToken(name));
    return result;
}();
const TfToken &
Token(const char *name)
{
    return _tokens.find(name)->second;
}
// Step kinds that have a row-level witness here are keyed by the schedule's
// shared kind name; the rest are keyed as having no equivalent.
const char *
KindName(Kind kind)
{
    switch (kind) {
    case Kind::ComposeSubtree:
    case Kind::RestCompose:
    case Kind::LadderCompose:
    case Kind::Solve:
    case Kind::ProviderMatrix:
    case Kind::FrameMatrix:
    case Kind::VolumePlacements:
    case Kind::WeightPacket:
        return RigExecBakedStepKindName(kind);
    default:
        return RigExecExecCheckSkipText::noEquivalent;
    }
}

RigExecValueAddress
Address(const RigExecValueOverride &v)
{
    return v.attribute.IsEmpty()
               ? RigExecValueAddress::Prim(v.prim, v.computation)
               : RigExecValueAddress::Property(
                     v.prim.AppendProperty(v.attribute));
}
void
Put(Input &in, const SdfPath &path, const char *computation, VtValue value)
{
    RigExecValueOverride v;
    v.prim = path;
    v.computation = Token(computation);
    v.value = std::move(value);
    const auto found = std::find_if(
        in.overrides.begin(), in.overrides.end(), [&](const auto &old) {
            return old.prim == v.prim && old.computation == v.computation &&
                   old.attribute.IsEmpty();
        });
    if (found == in.overrides.end())
        in.overrides.push_back(std::move(v));
    else if (found->value != v.value)
        in.skip = RigExecExecCheckSkip::NoEquivalent;
}
void
Attr(Input &in, const SdfPath &path, const TfToken &name, VtValue value)
{
    RigExecValueOverride v;
    v.prim = path;
    v.attribute = name;
    v.value = std::move(value);
    const auto found = std::find_if(
        in.overrides.begin(), in.overrides.end(), [&](const auto &old) {
            return old.prim == v.prim && old.attribute == v.attribute;
        });
    if (found == in.overrides.end())
        in.overrides.push_back(std::move(v));
    else if (found->value != v.value)
        in.skip = RigExecExecCheckSkip::NoEquivalent;
}
void
Attr(Input &in, const SdfPath &path, const char *name, VtValue value)
{
    Attr(in, path, Token(name), std::move(value));
}
template <class T>
void
Leaf(Input &in, const B &b, const SdfPath &path, const char *name,
     const RigExecBakedInput<T> &input)
{
    Attr(in, path, name, VtValue(RigExecBakedLeafRead(b, input)));
}
bool
Authoritative(const B &b, const RigExecBakedInput<GfMatrix4d> &input,
              bool connected)
{
    const int k = input.overrideIndex;
    return connected ||
           (k >= 0 &&
            ((size_t(k) < b.overridden.size() && b.overridden[size_t(k)]) ||
             (size_t(k) < b.upstreamOn.size() && b.upstreamOn[size_t(k)])));
}
Input
ComposeInputs(const B &b, int slot)
{
    Input in;
    const size_t s = size_t(slot);
    const auto &path = b.paths[s];
    if ((!b.spaceSwitchBySlot.empty() && b.spaceSwitchBySlot[s] >= 0) ||
        b.slotKind[s] != RigExecBakedSlotKind::FirstFramePose) {
        in.skip = RigExecExecCheckSkip::NoEquivalent;
        return in;
    }
    // These are the effective bound inputs of the point-frame computation.
    // None is the point-frame output being checked.
    Attr(in, path, SchemaAttrText::posedSpace, VtValue(b.posedAuthoredM[s]));
    Attr(in, path, SchemaAttrText::posedDefaultSpace, VtValue(b.posedD[s]));
    const int parent = b.parent[s];
    const auto &l = b.ladders[s];
    const auto rawParentDefault = RigExecBakedLeafRead(b, l.parentDefaultSpace);
    const auto parentDefault =
        Authoritative(b, l.parentDefaultSpace, l.parentDefaultSpaceConnected) ||
                rawParentDefault != GfMatrix4d(1.0)
            ? rawParentDefault
        : parent >= 0 ? b.defaultRoundTrip[size_t(parent)]
                      : GfMatrix4d(1.0);
    Attr(in, path, SchemaAttrText::parentDefaultSpace, VtValue(parentDefault));
    Attr(in, path, SchemaAttrText::parentSpace,
         VtValue(b.parentSpaceAuthored[s] ? b.parentSpaceM[s]
                 : parent >= 0            ? b.posedM[size_t(parent)]
                                          : GfMatrix4d(1.0)));
    const double *a = &b.avars[s * 11];
    const char *names[] = {SchemaAttrText::avarsTx,
                           SchemaAttrText::avarsTy,
                           SchemaAttrText::avarsTz,
                           SchemaAttrText::avarsSx,
                           SchemaAttrText::avarsSy,
                           SchemaAttrText::avarsSz,
                           SchemaAttrText::avarsRx,
                           SchemaAttrText::avarsRy,
                           SchemaAttrText::avarsRz,
                           SchemaAttrText::avarsRspin,
                           SchemaAttrText::avarsUnitScaleFactor};
    for (size_t i = 0; i != 11; ++i) {
        if (b.noScaleAvars[s] && i >= 3 && i <= 5)
            continue;
        Attr(in, path, names[i], VtValue(a[i]));
    }
    Leaf(in, b, path, SchemaAttrText::avarsRotationSign,
         b.ladders[s].rotationSign);
    Attr(in, path, SchemaAttrText::avarsRotationOrder, VtValue(b.rotOrder[s]));
    // Point-frame registers a Connections input in addition to AttributeValue.
    // The bound walk's source paths must receive the copied effective value as
    // well, including an authoritative connected identity.
    if (b.ladders[s].posedSpaceConnected) {
        bool source = false;
        const int binding = b.ladders[s].posedSpace.overrideIndex;
        for (const auto &[property, bindings] : b.overridableInputs) {
            if ((property.GetPrimPath() == path &&
                 property.GetNameToken() ==
                     Token(SchemaAttrText::posedSpace)) ||
                std::find(bindings.begin(), bindings.end(), binding) ==
                    bindings.end())
                continue;
            Attr(in, property.GetPrimPath(), property.GetNameToken(),
                 VtValue(b.posedAuthoredM[s]));
            source = true;
        }
        if (!source)
            in.skip = RigExecExecCheckSkip::NoEquivalent;
    }
    return in;
}
Input
RestInputs(const B &b, int slot)
{
    Input in;
    const size_t s = size_t(slot);
    const auto &path = b.paths[s];
    const auto &l = b.ladders[s];
    Leaf(in, b, path, SchemaAttrText::restSpace, l.restSpace);
    const char *names[] = {SchemaAttrText::restTx, SchemaAttrText::restTy,
                           SchemaAttrText::restTz, SchemaAttrText::restRx,
                           SchemaAttrText::restRy, SchemaAttrText::restRz};
    for (size_t i = 0; i != 6; ++i)
        Leaf(in, b, path, names[i], l.restAvars[i]);
    const int parent = b.parent[s];
    if (parent >= 0)
        Put(in, b.paths[size_t(parent)],
            SchemaComputationText::computeRestFrame,
            VtValue(b.restFrames[size_t(parent)]));
    return in;
}
Input
LadderInputs(const B &b, int slot)
{
    Input in;
    const size_t s = size_t(slot);
    const auto &path = b.paths[s];
    const auto &l = b.ladders[s];
    // computedDefaultSpace is the native default ladder's fallback arm.
    // An authored/connected default is instead a SpaceExpression operation.
    if (Authoritative(b, l.defaultSpace, l.defaultSpaceConnected) ||
        RigExecBakedLeafRead(b, l.defaultSpace) != GfMatrix4d(1.0))
        in.skip = RigExecExecCheckSkip::NoEquivalent;
    Put(in, path, SchemaComputationText::computeRestFrame,
        VtValue(b.restFrames[s]));
    const int parent = b.parent[s];
    if (parent >= 0)
        Put(in, b.paths[size_t(parent)],
            SchemaComputationText::computeRestFrame,
            VtValue(b.restFrames[size_t(parent)]));
    const auto raw = RigExecBakedLeafRead(b, l.parentDefaultSpace);
    const auto *expression = l.spaceValues[4] >= 0
                                 ? b.providerValues.Read<GfMatrix4d>(
                                       RigExecValueId(l.spaceValues[4]))
                                 : nullptr;
    const auto parentDefault = expression ? *expression
                               : Authoritative(b, l.parentDefaultSpace,
                                               l.parentDefaultSpaceConnected) ||
                                       raw != GfMatrix4d(1.0)
                                   ? raw
                               : parent >= 0
                                   ? b.defaultRoundTrip[size_t(parent)]
                                   : GfMatrix4d(1.0);
    Attr(in, path, SchemaAttrText::parentDefaultSpace, VtValue(parentDefault));
    const char *names[] = {
        SchemaAttrText::defaultTx, SchemaAttrText::defaultTy,
        SchemaAttrText::defaultTz, SchemaAttrText::defaultRx,
        SchemaAttrText::defaultRy, SchemaAttrText::defaultRz};
    for (size_t i = 0; i != 6; ++i)
        Leaf(in, b, path, names[i], l.defaultAvars[i]);
    return in;
}
Input
MatrixInputs(const B &b, int slot, uint32_t version, bool base)
{
    Input in;
    const size_t s = size_t(slot);
    const auto &frame = base ? b.base[version] : b.fin[version];
    if (b.slotKind[s] != RigExecBakedSlotKind::FirstFramePose)
        in.skip = RigExecExecCheckSkip::NoEquivalent;
    else if (!RigExecBakedUsable(frame) || !RigExecBakedUsable(b.restFrames[s]))
        in.skip = RigExecExecCheckSkip::Unusable;
    Put(in, b.paths[s], SchemaComputationText::computePointFrame,
        VtValue(frame));
    Put(in, b.paths[s], SchemaComputationText::computeRestFrame,
        VtValue(b.restFrames[s]));
    return in;
}
void
SolverFrame(Input &in, const B &b, int slot, uint32_t version)
{
    if (slot >= 0) {
        Put(in, b.paths[size_t(slot)], SchemaComputationText::computePointFrame,
            VtValue(b.fin[version]));
        Put(in, b.paths[size_t(slot)], SchemaComputationText::computeRestFrame,
            VtValue(b.restFrames[size_t(slot)]));
    }
}
Input
SolveInputs(const B &b, int index)
{
    Input in;
    const auto &s = b.solvers[size_t(index)];
    const auto &path = s.path;
    for (size_t i = 0; i < s.restRefs.size(); ++i) {
        const size_t slot = size_t(s.restRefs[i].first);
        auto rest = b.restFrames[slot];
        if (i < s.restIsLive.size() && s.restIsLive[i]) {
            rest = b.fin[s.restReads[i]];
            rest.flags |= RigExecPointFrameLiveRest;
        }
        Put(in, b.paths[slot], SchemaComputationText::computeRestFrame,
            VtValue(rest));
    }
    if (s.type == SchemaTypeName::RigExecFkChain) {
        for (size_t i = 0; i < s.controls.size(); ++i)
            SolverFrame(in, b, s.controls[i], s.controlReads[i]);
        SolverFrame(in, b, s.start, s.startRead);
        Attr(in, path, SchemaAttrText::rigExecSegmentScale,
             VtValue(Token(s.scaleSegments ? SchemaValueText::toChild
                                           : SchemaValueText::none)));
        Attr(in, path, SchemaAttrText::rigExecControlSpace,
             VtValue(Token(s.parentRelative ? SchemaValueText::parentRelative
                                            : SchemaValueText::world)));
        // The exec FK computation reads no joint groups.
        if (!s.jointGroups.empty())
            in.skip = RigExecExecCheckSkip::NoEquivalent;
    } else if (s.type == SchemaTypeName::RigExecTwoBoneIk) {
        SolverFrame(in, b, s.root, s.rootRead);
        SolverFrame(in, b, s.end, s.endRead);
        SolverFrame(in, b, s.pole, s.poleRead);
        if (s.spaceSlot >= 0)
            SolverFrame(in, b, s.spaceSlot, uint32_t(s.spaceRead));
        Leaf(in, b, path, SchemaAttrText::rigExecSpaceMatrix, s.ikSpace);
        Leaf(in, b, path, SchemaAttrText::rigExecUpperLengthOffset,
             s.upperOffset);
        Leaf(in, b, path, SchemaAttrText::rigExecLowerLengthOffset,
             s.lowerOffset);
        Leaf(in, b, path, SchemaAttrText::rigExecPreferredBendRadians, s.bend);
        Leaf(in, b, path, SchemaAttrText::inputsStretch, s.stretch);
        Leaf(in, b, path, SchemaAttrText::inputsSoftness, s.softness);
        Leaf(in, b, path, SchemaAttrText::inputsPin, s.pin);
        Leaf(in, b, path, SchemaAttrText::inputsUpperScale, s.upperScale);
        Leaf(in, b, path, SchemaAttrText::inputsLowerScale, s.lowerScale);
        Leaf(in, b, path, SchemaAttrText::inputsSoftDistance, s.softDistance);
        Leaf(in, b, path, SchemaAttrText::inputsTwist, s.limbTwist);
        Attr(in, path, SchemaAttrText::rigExecStretchPolicy,
             VtValue(Token(s.ikParams.softDistancePolicy
                               ? SchemaValueText::softDistance
                               : SchemaValueText::uniformSegments)));
        Attr(in, path, SchemaAttrText::rigExecSegmentScale,
             VtValue(Token(s.ikParams.scaleSegments ? SchemaValueText::toChild
                                                    : SchemaValueText::none)));
        Attr(in, path, SchemaAttrText::rigExecScaleCalibration,
             VtValue(s.ikParams.limb.scaleCalibration));
    } else if (s.type == SchemaTypeName::RigExecBlendPointFrames) {
        if (s.inA >= 0)
            Put(in, b.solvers[size_t(s.inA)].path,
                SchemaComputationText::computePointFrameArray,
                VtValue(b.aggregates[size_t(s.inA)]));
        if (s.inB >= 0)
            Put(in, b.solvers[size_t(s.inB)].path,
                SchemaComputationText::computePointFrameArray,
                VtValue(b.aggregates[size_t(s.inB)]));
        Leaf(in, b, path, SchemaAttrText::inputsWeight, s.blendWeight);
        Attr(in, path, SchemaAttrText::rigExecScaleBlend,
             VtValue(Token(s.scaleMode == RigExecScaleBlend::Log
                               ? SchemaValueText::log
                               : SchemaValueText::linear)));
        if (s.blendRotationRejected)
            in.skip = RigExecExecCheckSkip::Unusable;
        else
            Attr(in, path, SchemaAttrText::rigExecRotationBlend,
                 VtValue(Token(SchemaValueText::shortestArc)));
    } else if (s.type == SchemaTypeName::RigExecTwistDistribution) {
        SolverFrame(in, b, s.root, s.rootRead);
        SolverFrame(in, b, s.end, s.endRead);
        Leaf(in, b, path, SchemaAttrText::inputsTwistTurns, s.twistTurns);
        Attr(in, path, SchemaAttrText::rigExecCount, VtValue(s.execTwistCount));
    } else if (s.type == SchemaTypeName::RigExecRibbon) {
        // The rest/live curve packets lack an equivalent bound base array
        // protocol in this row. They are never reconstructed from the result.
        in.skip = RigExecExecCheckSkip::ArrayInput;
    } else if (s.type == SchemaTypeName::RigExecSplineIk) {
        SolverFrame(in, b, s.root, s.rootRead);
        SolverFrame(in, b, s.mid, s.midRead);
        SolverFrame(in, b, s.end, s.endRead);
        if (s.spaceSlot >= 0)
            SolverFrame(in, b, s.spaceSlot, uint32_t(s.spaceRead));
        Leaf(in, b, path, SchemaAttrText::inputsPreserveVolume,
             s.preserveVolume);
        Leaf(in, b, path, SchemaAttrText::inputsMidFollowWeight,
             s.midFollowWeight);
        Leaf(in, b, path, SchemaAttrText::inputsRoll, s.roll);
        Leaf(in, b, path, SchemaAttrText::inputsTwist, s.twist);
        Leaf(in, b, path, SchemaAttrText::inputsMinLengthRatio,
             s.minLengthRatio);
        Attr(in, path, SchemaAttrText::rigExecRootTangent,
             VtValue(Token(s.splineParams.aimRootTangent
                               ? SchemaValueText::aim
                               : SchemaValueText::rigid)));
        Attr(in, path, SchemaAttrText::rigExecRestLength,
             VtValue(Token(s.splineRestMode == RigExecSplineIkRestLength::Curve
                               ? SchemaValueText::curve
                               : SchemaValueText::chain)));
    } else
        in.skip = RigExecExecCheckSkip::NoEquivalent;
    const auto arrayIsReplaced = [&](const char *name) {
        return b.upstream.count(path.AppendProperty(Token(name))) != 0;
    };
    if ((s.type == SchemaTypeName::RigExecTwoBoneIk ||
         s.type == SchemaTypeName::RigExecSplineIk) &&
        arrayIsReplaced(SchemaAttrText::rigExecJointElements))
        in.skip = RigExecExecCheckSkip::ArrayInput;
    if (s.type == SchemaTypeName::RigExecSplineIk &&
        arrayIsReplaced(SchemaAttrText::rigExecVolumeWeights))
        in.skip = RigExecExecCheckSkip::ArrayInput;
    if (s.type == SchemaTypeName::RigExecTwistDistribution &&
        arrayIsReplaced(SchemaAttrText::rigExecWeights))
        in.skip = RigExecExecCheckSkip::ArrayInput;
    return in;
}
std::vector<RigExecExecArrayWitness>
SolveArrays(const B &b, int index, const SdfPath &elementsPath,
            const SdfPath &weightsPath, const SdfPath &volumeWeightsPath)
{
    const auto &s = b.solvers[size_t(index)];
    std::vector<RigExecExecArrayWitness> result;
    if (s.type == SchemaTypeName::RigExecTwoBoneIk ||
        s.type == SchemaTypeName::RigExecSplineIk)
        result.push_back({elementsPath, VtValue(s.execJointElements), true});
    if (s.type == SchemaTypeName::RigExecSplineIk)
        result.push_back(
            {volumeWeightsPath, VtValue(s.execSplineWeights), true});
    if (s.type == SchemaTypeName::RigExecTwistDistribution)
        result.push_back({weightsPath, VtValue(s.execTwistWeights), true});
    return result;
}

independentConstraintCheck::Binding
ConstraintReferenceBinding(const B &b, int walk)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    independentConstraintCheck::Binding binding;
    binding.owner = c.path;
    binding.type = c.type;
    binding.assetRoot = b.assetRootPath;
    binding.geometry = c.deltaBase >= 0;
    binding.target =
        binding.geometry ? c.deltaBasePath : b.paths[size_t(c.target)];
    if (c.spaceSlot >= 0)
        binding.carry = b.paths[size_t(c.spaceSlot)];
    const auto source =
        [&](const SdfPath &path, int slot,
            const std::vector<RigExecBakedCommit::AncestorRead> &ancestors) {
            independentConstraintCheck::Source input;
            input.path = path;
            input.boundFrame = slot >= 0;
            for (const auto &a : ancestors)
                input.ancestors.push_back(b.paths[size_t(a.slot)]);
            return input;
        };
    for (size_t k = 0; k < c.sources.size(); ++k)
        binding.sources.push_back(
            source(c.sourcePaths[k], c.sources[k], commit.sourceAncestors[k]));
    if (c.worldUpObjectNamed)
        binding.worldUp =
            source(c.worldUpPath, c.worldUpObject, commit.worldUpAncestors);
    return binding;
}
Input
ConstraintInputs(const B &b, int walk,
                 const independentConstraintCheck::Binding &binding)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    Input in;
    Put(in, binding.target, SchemaComputationText::computePointFrame,
        VtValue(b.fin[size_t(commit.targetRead)]));
    const auto source =
        [&](const independentConstraintCheck::Source &input, int slot,
            uint32_t version,
            const std::vector<RigExecBakedCommit::AncestorRead> &ancestors) {
            if (slot >= 0)
                Put(in, input.path, SchemaComputationText::computePointFrame,
                    VtValue(b.fin[size_t(version)]));
            else
                for (const auto &a : ancestors) {
                    Put(in, b.paths[size_t(a.slot)],
                        SchemaComputationText::computeBasePointFrame,
                        VtValue(b.base[size_t(a.base)]));
                    Put(in, b.paths[size_t(a.slot)],
                        SchemaComputationText::computePointFrame,
                        VtValue(b.fin[size_t(a.fin)]));
                }
        };
    for (size_t k = 0; k < c.sources.size(); ++k)
        source(binding.sources[k], c.sources[k], commit.sourceReads[k],
               commit.sourceAncestors[k]);
    if (c.worldUpObjectNamed)
        source(binding.worldUp, c.worldUpObject, commit.worldUpRead,
               commit.worldUpAncestors);
    if (c.spaceSlot >= 0) {
        Attr(in, binding.carry, SchemaAttrText::posedSpace,
             VtValue(b.posedM[size_t(c.spaceSlot)]));
        Attr(in, binding.carry, SchemaAttrText::posedDefaultSpace,
             VtValue(b.defaultRoundTrip[size_t(c.spaceSlot)]));
    }
    Leaf(in, b, c.path, SchemaAttrText::inputsEnabled, c.enabled);
    Leaf(in, b, c.path, SchemaAttrText::inputsDefaultWeight, c.defaultWeight);
    if (c.type == SchemaTypeName::RigExecParentConstraint) {
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectTranslationX, c.tX);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectTranslationY, c.tY);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectTranslationZ, c.tZ);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationX, c.rX);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationY, c.rY);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationZ, c.rZ);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectScaleX, c.sX);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectScaleY, c.sY);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectScaleZ, c.sZ);
    } else {
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationX, c.affectX);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationY, c.affectY);
        Leaf(in, b, c.path, SchemaAttrText::inputsAffectRotationZ, c.affectZ);
        if (c.type == SchemaTypeName::RigExecRotationConstraint)
            Leaf(in, b, c.path, SchemaAttrText::inputsRotationOffset, c.offset);
        else {
            Leaf(in, b, c.path, SchemaAttrText::inputsAimVector, c.aimVector);
            Leaf(in, b, c.path, SchemaAttrText::inputsUpVector, c.upVector);
            Leaf(in, b, c.path, SchemaAttrText::inputsRotationOffset,
                 c.rotationOffset);
            Leaf(in, b, c.path, SchemaAttrText::inputsWorldUpVector,
                 c.worldUpVector);
        }
    }
    return in;
}
VtValue
ConstraintResult(const B &b, int walk, const RigExecPointFrame *candidate)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    const bool geometry = c.deltaBase >= 0;
    const bool present = !commit.present.empty() && commit.present[0];
    const bool deltaPresent = geometry && b.deltaPresent[size_t(c.deltaBase)];
    VtValue result;
    independentConstraintCheck::Encode(
        present, commit.abandoned, geometry, deltaPresent, candidate,
        geometry ? nullptr : &b.fin[size_t(commit.slotWrites[0])],
        deltaPresent ? &b.deltaValues[size_t(c.deltaBase)] : nullptr, &result);
    return result;
}

Input
WeightInputs(const B &b, int index)
{
    Input in;
    const auto &w = b.weightObjects[size_t(index)];
    const auto &path = w.path;
    Attr(in, path, SchemaAttrText::rigExecRepresentation,
         VtValue(w.representation));
    Attr(in, path, SchemaAttrText::rigExecRangePolicy, VtValue(w.rangePolicy));
    if (w.type == SchemaTypeName::RigExecStaticWeight) {
        Leaf(in, b, path, SchemaAttrText::rigExecDefaultWeight,
             w.defaultWeight);
    } else if (w.type == SchemaTypeName::RigExecDynamicWeight) {
        Leaf(in, b, path, SchemaAttrText::inputsDriver, w.driver);
        Leaf(in, b, path, SchemaAttrText::inputsScale, w.scale);
        Leaf(in, b, path, SchemaAttrText::inputsBias, w.bias);
        if (w.base >= 0)
            Put(in, b.weightObjects[size_t(w.base)].path,
                SchemaComputationText::computeWeightPacket,
                VtValue(b.weightPackets[size_t(w.base)]));
    } else if (w.type == SchemaTypeName::RigExecCombineWeight) {
        Attr(in, path, SchemaAttrText::rigExecCombineMode,
             VtValue(w.combineMode));
        Leaf(in, b, path, SchemaAttrText::inputsStrength, w.strength);
        Leaf(in, b, path, SchemaAttrText::inputsInvert, w.invert);
        for (const int input : w.inputs)
            Put(in, b.weightObjects[size_t(input)].path,
                SchemaComputationText::computeWeightPacket,
                VtValue(b.weightPackets[size_t(input)]));
    } else if (w.type == SchemaTypeName::RigExecSphereWeight ||
               w.type == SchemaTypeName::RigExecPlaneWeight ||
               w.type == SchemaTypeName::RigExecCurveWeight) {
        if (w.providerSlot >= 0)
            Put(in, path, SchemaComputationText::computePointFrame,
                VtValue(b.base[b.baseLast[size_t(w.providerSlot)]]));
        RigExecFalloffLut lut;
        lut.samples = w.falloffCurve;
        if (w.falloffSamplesLeaf >= 0) {
            const VtFloatArray samples = w.pointLeaves.Value<VtFloatArray>(
                size_t(w.falloffSamplesLeaf), {});
            if (!samples.empty())
                lut.samples.assign(samples.cbegin(), samples.cend());
        }
        Put(in, path, SchemaComputationText::computeFalloffLut, VtValue(lut));
        Leaf(in, b, path, SchemaAttrText::inputsFalloffMin, w.falloffMin);
        Leaf(in, b, path, SchemaAttrText::inputsFalloffMax, w.falloffMax);
        Leaf(in, b, path, SchemaAttrText::inputsStrength, w.strength);
        Leaf(in, b, path, SchemaAttrText::inputsInvert, w.invert);
        if (w.type == SchemaTypeName::RigExecPlaneWeight) {
            Attr(in, path, SchemaAttrText::rigExecPlaneAxis,
                 VtValue(w.planeAxis));
            Attr(in, path, SchemaAttrText::rigExecPlaneBounds,
                 VtValue(w.planeBounds));
            Leaf(in, b, path, SchemaAttrText::inputsExtentU, w.extentU);
            Leaf(in, b, path, SchemaAttrText::inputsExtentV, w.extentV);
        } else {
            Leaf(in, b, path, SchemaAttrText::inputsScaleX, w.scaleX);
            Leaf(in, b, path, SchemaAttrText::inputsScaleY, w.scaleY);
            Leaf(in, b, path, SchemaAttrText::inputsScaleZ, w.scaleZ);
            if (w.type == SchemaTypeName::RigExecSphereWeight) {
                Leaf(in, b, path, SchemaAttrText::inputsScaleXPos, w.scaleXPos);
                Leaf(in, b, path, SchemaAttrText::inputsScaleYPos, w.scaleYPos);
                Leaf(in, b, path, SchemaAttrText::inputsScaleZPos, w.scaleZPos);
                Leaf(in, b, path, SchemaAttrText::inputsScaleXNeg, w.scaleXNeg);
                Leaf(in, b, path, SchemaAttrText::inputsScaleYNeg, w.scaleYNeg);
                Leaf(in, b, path, SchemaAttrText::inputsScaleZNeg, w.scaleZNeg);
            }
        }
    } else
        in.skip = RigExecExecCheckSkip::NoEquivalent;
    // Only the sampled path-leaf copies are passed to exec. A chain-routed or
    // upstream replacement array is an explicitly unjudged array-input row.
    const auto arrays = [&](const std::vector<UsdAttribute> &attrs,
                            size_t begin) {
        for (size_t i = 0; i < attrs.size(); ++i) {
            const size_t k = begin + i;
            if (k >= w.pointLeaves.values.size()) {
                in.skip = RigExecExecCheckSkip::ArrayInput;
                continue;
            }
            if ((k < w.pointLeaves.walks.size() &&
                 w.pointLeaves.walks[k] >= 0) ||
                b.upstream.count(w.pointLeaves.decl.keys[k].path))
                in.skip = RigExecExecCheckSkip::ArrayInput;
        }
    };
    arrays(w.targetPoints, w.targetLeaves);
    arrays(w.samplePoints, w.sampleLeaves);
    arrays(w.curvePoints, w.curveLeaves);
    arrays(w.combineTargetPoints, w.combineLeaves);
    return in;
}
std::vector<RigExecExecArrayWitness>
WeightArrays(const B &b, int index, const SdfPath &valuesPath,
             const SdfPath &indicesPath)
{
    std::vector<RigExecExecArrayWitness> result;
    const auto &w = b.weightObjects[size_t(index)];
    if (w.type == SchemaTypeName::RigExecStaticWeight) {
        result.push_back(
            {valuesPath,
             VtValue(VtFloatArray(w.values.begin(), w.values.end())), true});
        result.push_back(
            {indicesPath,
             VtValue(VtIntArray(w.indices.begin(), w.indices.end())), true});
    }
    const auto append = [&](const std::vector<UsdAttribute> &attrs,
                            size_t begin) {
        for (size_t i = 0; i < attrs.size(); ++i)
            if (begin + i < w.pointLeaves.values.size() &&
                begin + i < w.pointLeaves.decl.keys.size()) {
                const auto &key = w.pointLeaves.decl.keys[begin + i];
                result.push_back(
                    {key.path, w.pointLeaves.values[begin + i],
                     key.time == RigExecRevisionLeafTime::AtDefault});
            }
    };
    append(w.targetPoints, w.targetLeaves);
    append(w.samplePoints, w.sampleLeaves);
    append(w.curvePoints, w.curveLeaves);
    append(w.combineTargetPoints, w.combineLeaves);
    return result;
}
} // namespace

bool
RigExecBakedExecCheckRows::Add(RigExecExecCheckDescriptor d, Binding binding,
                               const B &b, std::string *error)
{
    Input initial = binding.capture(b);
    for (const auto &v : initial.overrides)
        d.requiredOverrides.push_back(Address(v));
    if (!_check.Add(d, error))
        return false;
    const size_t row = _bindings.size();
    _byStep[binding.step].push_back(row);
    if (binding.perSlot)
        _bySlot[{uint32_t(b.steps[binding.step].kind), binding.slot}].push_back(
            row);
    _descriptors.push_back(std::move(d));
    _bindings.push_back(std::move(binding));
    // A constraint result callback reads its row-local candidate storage.
    // Establish that row before invoking the initial result callback.
    _candidateProduced.push_back(0);
    _constraintCandidates.emplace_back();
    initial.expected = _bindings.back().result(b);
    _inputs.push_back(std::move(initial));
    _captured.push_back(0);
    _stageArrays.push_back(_bindings.back().arrays
                               ? _bindings.back().arrays(b)
                               : std::vector<RigExecExecArrayWitness>{});
    return true;
}

RigExecBakedExecCheckRows::~RigExecBakedExecCheckRows() = default;

std::shared_ptr<RigExecBakedExecCheckRows>
RigExecBakedExecCheckRows::Build(const B &b, std::string *error)
{
    auto rows = std::make_shared<RigExecBakedExecCheckRows>(_Tag(0), b.stage);
    rows->_byStep.resize(b.steps.size());
    for (size_t i = 0; i < b.steps.size(); ++i) {
        const auto &step = b.steps[i];
        const auto add =
            [&](const SdfPath &path, const char *key, int slot, bool perSlot,
                std::function<Input(const B &)> capture,
                std::function<VtValue(const B &)> result,
                std::function<bool(const VtValue &, VtValue *)> project = {},
                std::function<std::vector<RigExecExecArrayWitness>(const B &)>
                    arrays = {}) {
                RigExecExecCheckDescriptor d;
                d.key = std::string(KindName(step.kind)) + " " +
                        path.GetString() + ":" + key;
                d.key += " op " + step.descriptorKey;
                if (step.kind == Kind::ProviderMatrix ||
                    step.kind == Kind::VolumePlacements)
                    d.key +=
                        step.part == 0 || step.part == 2 ? " base" : " final";
                if (step.kind == Kind::FrameMatrix)
                    d.key +=
                        " after " +
                        b.frameRecords[size_t(step.object)].mover.GetString();
                d.kind = uint32_t(step.kind);
                d.address = RigExecValueAddress::Prim(path, Token(key));
                d.project = std::move(project);
                if (slot >= 0 && b.slotKind[size_t(slot)] !=
                                     RigExecBakedSlotKind::FirstFramePose) {
                    d.skipOnly = true;
                    d.address = {};
                    d.project = {};
                    capture = [](const B &) {
                        Input input;
                        input.skip = RigExecExecCheckSkip::NoEquivalent;
                        return input;
                    };
                }
                return rows->Add(std::move(d),
                                 {i, slot, perSlot, std::move(capture),
                                  std::move(result), std::move(arrays)},
                                 b, error);
            };
        if (step.kind == Kind::ComposeSubtree ||
            step.kind == Kind::RestCompose ||
            step.kind == Kind::LadderCompose) {
            const auto &g = b.composeGroups[size_t(step.object)];
            for (int s = g.begin; s < g.end; ++s) {
                if (step.kind == Kind::ComposeSubtree) {
                    if (!add(
                            b.paths[size_t(s)],
                            SchemaComputationText::computePointFrame, s, true,
                            [s](const B &p) { return ComposeInputs(p, s); },
                            [s](const B &p) {
                                return VtValue(p.base[size_t(s)]);
                            }))
                        return {};
                } else if (step.kind == Kind::RestCompose) {
                    if (!add(
                            b.paths[size_t(s)],
                            SchemaComputationText::computeRestFrame, s, true,
                            [s](const B &p) { return RestInputs(p, s); },
                            [s](const B &p) {
                                return VtValue(p.restFrames[size_t(s)]);
                            }))
                        return {};
                } else {
                    if (!add(
                            b.paths[size_t(s)],
                            SchemaComputationText::computedDefaultSpace, s,
                            true,
                            [s](const B &p) { return LadderInputs(p, s); },
                            [s](const B &p) {
                                return VtValue(p.selfD[size_t(s)]);
                            }))
                        return {};
                }
            }
        } else if (step.kind == Kind::ProviderMatrix) {
            const int s = step.object;
            const bool final = step.part != 0;
            if (!add(
                    b.paths[size_t(s)], SchemaComputationText::computeMatrix, s,
                    false,
                    [s, final](const B &p) {
                        return MatrixInputs(p, s,
                                            final ? p.finLast[size_t(s)]
                                                  : p.baseLast[size_t(s)],
                                            !final);
                    },
                    [s, final](const B &p) {
                        return VtValue(final ? p.finalMatrix[size_t(s)]
                                             : p.baseMatrix[size_t(s)]);
                    }))
                return {};
        } else if (step.kind == Kind::FrameMatrix) {
            const int r = step.object;
            const auto record = b.frameRecords[size_t(r)];
            if (!add(
                    b.paths[size_t(record.slot)],
                    SchemaComputationText::computeMatrix, record.slot, false,
                    [record](const B &p) {
                        return MatrixInputs(p, record.slot, record.version,
                                            false);
                    },
                    [r](const B &p) {
                        return VtValue(p.frameMatrix[size_t(r)]);
                    }))
                return {};
        } else if (step.kind == Kind::Solve) {
            const int s = step.object;
            const auto elementsPath = b.solvers[size_t(s)].path.AppendProperty(
                Token(SchemaAttrText::rigExecJointElements));
            const auto weightsPath = b.solvers[size_t(s)].path.AppendProperty(
                Token(SchemaAttrText::rigExecWeights));
            const auto volumeWeightsPath =
                b.solvers[size_t(s)].path.AppendProperty(
                    Token(SchemaAttrText::rigExecVolumeWeights));
            // A policy-derived start frame has no OpenExec equivalent: exec
            // binds rigExec:startFrame only from authored targets. Decided
            // here, once, by the rule every evaluator shares.
            const auto &solver = b.solvers[size_t(s)];
            SdfPath derivedStart;
            const bool startDerived =
                solver.type == SchemaTypeName::RigExecFkChain &&
                solver.start >= 0 && b.sceneDescriptors &&
                RigExecDeriveSceneStartFrame(*b.sceneDescriptors, solver.path,
                                             &derivedStart) ==
                    RigExecStartFrameDerivation::Derived;
            if (!add(
                    b.solvers[size_t(s)].path,
                    SchemaComputationText::computePointFrameArray, -1, false,
                    [s, startDerived](const B &p) {
                        Input in = SolveInputs(p, s);
                        if (startDerived)
                            in.skip = RigExecExecCheckSkip::NoEquivalent;
                        return in;
                    },
                    [s](const B &p) {
                        return VtValue(p.aggregates[size_t(s)]);
                    },
                    {},
                    [s, elementsPath, weightsPath,
                     volumeWeightsPath](const B &p) {
                        return SolveArrays(p, s, elementsPath, weightsPath,
                                           volumeWeightsPath);
                    }))
                return {};
        } else if (step.kind == Kind::Constraint) {
            const int walk = step.object;
            const auto &c =
                b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
            const auto &commit = b.commits[size_t(walk)];
            const bool supported =
                (c.type == SchemaTypeName::RigExecAimConstraint ||
                 c.type == SchemaTypeName::RigExecRotationConstraint ||
                 c.type == SchemaTypeName::RigExecParentConstraint) &&
                c.target >= 0 && !commit.split && commit.propagate.empty() &&
                (c.deltaBase >= 0 ? commit.slots.empty()
                                  : commit.slots.size() == 1) &&
                (c.weightObject.IsEmpty() || c.deltaBase >= 0);
            RigExecExecCheckDescriptor d;
            d.key = std::string(supported
                                    ? "OriginalConstraint "
                                    : RigExecExecCheckSkipText::noEquivalent) +
                    c.path.GetString() + " op " + step.descriptorKey;
            d.kind = uint32_t(step.kind);
            if (supported) {
                const auto binding = ConstraintReferenceBinding(b, walk);
                d.address = RigExecValueAddress::Prim(
                    c.path, SchemaComputation::computeConstraintWitness);
                d.acquireReference =
                    [binding](const UsdStageRefPtr &stage, UsdTimeCode time,
                              const std::vector<RigExecValueOverride> &inputs,
                              RigExecExecCheckDescriptor::Reference *reference,
                              std::string *error) {
                        independentConstraintCheck::Prepared prepared;
                        if (!independentConstraintCheck::Prepare(
                                binding, stage, time, inputs, &prepared, error))
                            return false;
                        *reference = [prepared = std::move(prepared)](
                                         VtValue *out, std::string *error) {
                            return independentConstraintCheck::Run(prepared,
                                                                   out, error);
                        };
                        return true;
                    };
                const size_t row = rows->_bindings.size();
                const auto *owner = rows.get();
                if (!rows->Add(std::move(d),
                               {i, -1, false,
                                [walk, binding](const B &p) {
                                    return ConstraintInputs(p, walk, binding);
                                },
                                [walk, row, owner](const B &p) {
                                    return ConstraintResult(
                                        p, walk,
                                        owner->_candidateProduced[row]
                                            ? &owner->_constraintCandidates[row]
                                            : nullptr);
                                }},
                               b, error))
                    return {};
                rows->_constraintRows.emplace(walk, row);
            } else {
                d.skipOnly = true;
                if (!rows->Add(std::move(d),
                               {i, -1, false,
                                [](const B &) {
                                    Input input;
                                    input.skip =
                                        RigExecExecCheckSkip::NoEquivalent;
                                    return input;
                                },
                                [](const B &) { return VtValue(); }},
                               b, error))
                    return {};
            }
        } else if (step.kind == Kind::WeightPacket) {
            const int w = step.object;
            const auto valuesPath =
                b.weightObjects[size_t(w)].path.AppendProperty(
                    Token(SchemaAttrText::rigExecValues));
            const auto indicesPath =
                b.weightObjects[size_t(w)].path.AppendProperty(
                    Token(SchemaAttrText::rigExecIndices));
            if (!add(
                    b.weightObjects[size_t(w)].path,
                    SchemaComputationText::computeWeightPacket, -1, false,
                    [w](const B &p) { return WeightInputs(p, w); },
                    [w](const B &p) {
                        return VtValue(p.weightPackets[size_t(w)]);
                    },
                    {},
                    [w, valuesPath, indicesPath](const B &p) {
                        return WeightArrays(p, w, valuesPath, indicesPath);
                    }))
                return {};
        } else if (step.kind == Kind::VolumePlacements) {
            const int s = step.object;
            const bool base = step.part == 2;
            size_t composeRow = rows->_bindings.size();
            for (size_t r = 0; r < rows->_bindings.size(); ++r)
                if (rows->_bindings[r].slot == s &&
                    b.steps[rows->_bindings[r].step].kind ==
                        Kind::ComposeSubtree)
                    composeRow = r;
            const auto *owner = rows.get();
            if (!add(
                    b.paths[size_t(s)],
                    SchemaComputationText::computePointFrame, s, false,
                    [s, base, composeRow, owner](const B &p) {
                        Input input;
                        if (composeRow < owner->_inputs.size())
                            input = owner->_inputs[composeRow];
                        else
                            input.skip = RigExecExecCheckSkip::NoEquivalent;
                        input.expected = VtValue();
                        const auto version =
                            base ? p.baseLast[size_t(s)] : p.finLast[size_t(s)];
                        if (version != uint32_t(s))
                            input.skip = RigExecExecCheckSkip::NoEquivalent;
                        return input;
                    },
                    [s, base](const B &p) {
                        return VtValue(base ? p.volumePlacementBase[size_t(s)]
                                            : p.volumePlacement[size_t(s)]);
                    },
                    [](const VtValue &value, VtValue *out) {
                        if (!value.IsHolding<RigExecPointFrame>())
                            return false;
                        *out = VtValue(RigExecVolumePlacement(
                            value.UncheckedGet<RigExecPointFrame>()));
                        return true;
                    }))
                return {};
        } else {
            RigExecExecCheckDescriptor d;
            d.key = std::string(RigExecExecCheckSkipText::noEquivalent) + " " +
                    step.label;
            if (step.kind == Kind::PropertyRevision)
                d.key +=
                    " target " +
                    b.propertyChains[size_t(step.object)].target.GetString();
            if (step.kind == Kind::RevisionStatic ||
                step.kind == Kind::RevisionChunk ||
                step.kind == Kind::RevisionFuse ||
                step.kind == Kind::InfluenceFold) {
                const auto &id = b.revisionIndex[size_t(step.object)];
                d.key +=
                    " target " + b.chains[size_t(id.first)].target.GetString();
            }
            d.key += " part " + std::to_string(step.part);
            d.key += " op " + step.descriptorKey;
            // The compiled label is the authored identity. Some mechanical
            // rows carry no label, so state their stable graph identity.
            if (step.label.empty())
                d.key += std::to_string(uint32_t(step.kind)) + ":" +
                         std::to_string(step.object) + ":" +
                         std::to_string(step.part);
            d.kind = uint32_t(step.kind);
            d.skipOnly = true;
            if (!rows->Add(std::move(d),
                           {i, -1, false,
                            [](const B &) {
                                Input input;
                                input.skip = RigExecExecCheckSkip::NoEquivalent;
                                return input;
                            },
                            [](const B &) { return VtValue(); }},
                           b, error))
                return {};
        }
    }
    // The native blend assembler produces a mover parameter packet, while
    // these public computations produce descriptor objects. Their presence
    // is recorded explicitly; neither descriptor is used as a result oracle.
    std::set<std::string> blendKeys;
    for (const auto &chain : b.chains)
        for (const auto &revision : chain.revisions)
            for (const auto &channel : revision.blendChannels) {
                const auto skip = [&](const std::string &key) {
                    if (b.steps.empty() || !blendKeys.insert(key).second)
                        return true;
                    RigExecExecCheckDescriptor d;
                    d.key = key;
                    d.kind = uint32_t(Kind::RevisionStatic);
                    d.skipOnly = true;
                    return rows->Add(
                        std::move(d),
                        {0, -1, false,
                         [](const B &) {
                             Input input;
                             input.skip = RigExecExecCheckSkip::NoEquivalent;
                             return input;
                         },
                         [](const B &) { return VtValue(); }},
                        b, error);
                };
                if (!skip(std::string(
                              SchemaComputationText::computeBlendChannel) +
                          " " + channel.weightPath.GetPrimPath().GetString()))
                    return {};
                for (const auto &sample : channel.samples)
                    if (!skip(
                            std::string(
                                SchemaComputationText::computeBlendSampleData) +
                            " " + sample.samplePath.GetString()))
                        return {};
            }
    return rows;
}

void
RigExecBakedExecCheckRows::ObserveConstraintCandidate(
    int walk, const RigExecPointFrame &candidate)
{
    const auto found = _constraintRows.find(walk);
    if (found == _constraintRows.end())
        return;
    _constraintCandidates[found->second] = candidate;
    _candidateProduced[found->second] = 1;
}
void
RigExecBakedExecCheckRows::BeforeStep(const B &b, size_t step)
{
    for (const size_t row : _byStep.at(step)) {
        if (!_bindings[row].perSlot) {
            _candidateProduced[row] = 0;
            _inputs[row] = _bindings[row].capture(b);
            _captured[row] = 1;
            if (_bindings[row].arrays)
                _stageArrays[row] = _bindings[row].arrays(b);
        }
    }
}
void
RigExecBakedExecCheckRows::BeforeSlot(const B &b, Kind kind, int groupBegin,
                                      int slot)
{
    (void)groupBegin;
    const auto found = _bySlot.find({uint32_t(kind), slot});
    if (found != _bySlot.end())
        for (const size_t row : found->second) {
            _inputs[row] = _bindings[row].capture(b);
            _captured[row] = 1;
        }
}
void
RigExecBakedExecCheckRows::AfterStep(const B &b, size_t step)
{
    for (const size_t row : _byStep.at(step)) {
        _inputs[row].expected = _bindings[row].result(b);
        if (b.steps[step].kind == Kind::FrameMatrix &&
            !b.frameMatrixValid[size_t(b.steps[step].object)])
            _inputs[row].skip = RigExecExecCheckSkip::Unusable;
        if (b.steps[step].kind == Kind::Solve &&
            b.aggregates[size_t(b.steps[step].object)].frames.empty() &&
            _inputs[row].skip == RigExecExecCheckSkip::None)
            _inputs[row].skip = RigExecExecCheckSkip::Unusable;
    }
}
RigExecExecCrossCheckReport
RigExecBakedExecCheckRows::Evaluate(UsdTimeCode time)
{
    auto verified = _inputs;
    _lastChecked.assign(verified.size(), 0);
    for (size_t row = 0; row < verified.size(); ++row) {
        if (verified[row].skip != RigExecExecCheckSkip::None)
            continue;
        for (const auto &witness : _stageArrays[row]) {
            const auto &path = witness.path;
            const auto &expected = witness.value;
            VtValue actual;
            const auto attribute = _stage->GetAttributeAtPath(path);
            if (attribute)
                attribute.Get(
                    &actual, witness.atDefault ? UsdTimeCode::Default() : time);
            std::string expectedBits, actualBits;
            if (!RigExecEncodeGoldenValue(expected, &expectedBits) ||
                !RigExecEncodeGoldenValue(actual, &actualBits) ||
                expectedBits != actualBits) {
                verified[row].skip = RigExecExecCheckSkip::ArrayInput;
                break;
            }
            if (witness.atDefault && !time.IsDefault()) {
                actual = VtValue();
                if (attribute)
                    attribute.Get(&actual, time);
                if (!RigExecEncodeGoldenValue(actual, &actualBits) ||
                    expectedBits != actualBits) {
                    verified[row].skip = RigExecExecCheckSkip::ArrayInput;
                    break;
                }
            }
        }
    }
    for (size_t row = 0; row < verified.size(); ++row)
        _lastChecked[row] = verified[row].skip == RigExecExecCheckSkip::None;
    auto report = _check.Evaluate(time, verified);
    for (size_t row = 0; row < _captured.size(); ++row) {
        if (!_captured[row] && !_descriptors[row].skipOnly) {
            ++report.failed;
            report.diagnostics.push_back(_descriptors[row].key +
                                         ": no pre-body input capture");
        }
    }
    return report;
}
bool
RigExecBakedProgramTesting::EnableExecCrossCheck(
    const RigExecBakedProgram &program, std::string *error)
{
    program._impl->execCheckRows =
        RigExecBakedExecCheckRows::Build(*program._impl, error);
    return bool(program._impl->execCheckRows);
}
std::shared_ptr<RigExecBakedExecCheckRows>
RigExecBakedProgramTesting::ExecCrossCheckRows(
    const RigExecBakedProgram &program)
{
    return program._impl->execCheckRows;
}
} // namespace rigExec
