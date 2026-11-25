#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/weightPackets.h"
#include "rigExec/internal/parallel.h"

#include "rigExec/internal/frameExtraction.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/weightFieldKernel.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>

// The shape types and plane bounds the packet builders dispatch on. A table
// rather than function-local statics: these run inside step bodies and the
// weight field on workers, and Build touches the table on the owning thread
// (RigExecWeightPacketsTouchTokens).
TF_DEFINE_PRIVATE_TOKENS(
    _weightTokens,
    ((sphere, rigExec::SchemaTypeName::RigExecSphereWeight))
    ((plane, rigExec::SchemaTypeName::RigExecPlaneWeight))
    ((curve, rigExec::SchemaTypeName::RigExecCurveWeight))
    ((unbounded, rigExec::SchemaValueText::unbounded))
    ((bounded, rigExec::SchemaValueText::bounded))
);

namespace rigExec {

void
RigExecWeightPacketsTouchTokens()
{
    (void)_weightTokens.Get();
}

bool
RigExecApplyWeightRangePolicy(const TfToken &policy, float *w)
{
    return RigExecApplyWeightRangeKernel(policy == SchemaValueText::clamp, w);
}

namespace {

int
_RepresentationKind(const TfToken &representation)
{
    return representation == SchemaValueText::constant ? kRigExecWeightConstant
           : representation == SchemaValueText::dense  ? kRigExecWeightDense
           : representation == SchemaValueText::sparse ? kRigExecWeightSparse
                                                       : -1;
}

bool
_KnownPolicy(const TfToken &rangePolicy)
{
    return rangePolicy == SchemaValueText::strict ||
           rangePolicy == SchemaValueText::clamp;
}

RigExecWeightPacketView
_View(const RigExecWeightPacket &packet)
{
    RigExecWeightPacketView view;
    view.valid = packet.valid;
    view.representation = _RepresentationKind(packet.representation);
    view.defaultWeight = packet.defaultWeight;
    view.values = packet.values.data();
    view.valueCount = packet.values.size();
    view.indices = packet.indices.data();
    view.indexCount = packet.indices.size();
    return view;
}

bool
_Publish(const char *why, RigExecWeightPacketBuild *built,
         RigExecWeightPacket *packet)
{
    packet->defaultWeight = built->defaultWeight;
    packet->values = std::move(built->values);
    packet->indices = std::move(built->indices);
    packet->valid = !why;
    return packet->valid;
}

} // namespace

static_assert(std::is_same<int, int32_t>::value,
              "weight packet indices are the kernel's int32 indices");

static RigExecWeightPacket
_BuildStaticWeightPacket(const TfToken &representation,
                         const TfToken &rangePolicy, const float *values,
                         size_t valuesSize, const int *indices,
                         size_t indicesSize, float defaultWeight,
                         RigExecWeightPacketWorkspace &workspace)
{
    RigExecWeightPacket packet;
    packet.representation = representation;
    packet.rangePolicy = rangePolicy;
    const int kind = _RepresentationKind(representation);
    if (kind < 0 || !_KnownPolicy(rangePolicy))
        return packet;
    packet.defaultWeight = defaultWeight;
    if ((valuesSize && !values) || (indicesSize && !indices))
        return packet;
    RigExecWeightPacketBuild built;
    _Publish(RigExecStaticWeightPacketKernel(
                 kind, rangePolicy == SchemaValueText::clamp, defaultWeight,
                 values, valuesSize, indices, indicesSize,
                 &workspace.staticScratch, &built),
             &built, &packet);
    return packet;
}

RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputs &inputs)
{
    const RigExecStaticWeightInputsView view{
        inputs.representation, inputs.rangePolicy,    inputs.values.data(),
        inputs.values.size(),  inputs.indices.data(), inputs.indices.size(),
        inputs.defaultWeight};
    return RigExecBuildStaticWeightPacket(view);
}

RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputsView &inputs)
{
    RigExecWeightPacketWorkspace workspace;
    return RigExecBuildStaticWeightPacket(inputs, &workspace);
}
RigExecWeightPacket
RigExecBuildStaticWeightPacket(const RigExecStaticWeightInputsView &inputs,
                               RigExecWeightPacketWorkspace *workspace)
{
    if (!workspace)
        return RigExecBuildStaticWeightPacket(inputs);
    return _BuildStaticWeightPacket(inputs.representation, inputs.rangePolicy,
                                    inputs.values, inputs.valuesSize,
                                    inputs.indices, inputs.indicesSize,
                                    inputs.defaultWeight, *workspace);
}

RigExecWeightPacket
RigExecBuildDynamicWeightPacket(const RigExecDynamicWeightInputs &inputs,
                                const RigExecWeightPacket *base)
{
    RigExecWeightPacket packet;
    packet.representation = inputs.representation;
    packet.rangePolicy = inputs.rangePolicy;
    const int kind = _RepresentationKind(inputs.representation);
    if (kind < 0 || !_KnownPolicy(inputs.rangePolicy))
        return packet;
    const RigExecWeightPacketView view =
        base ? _View(*base) : RigExecWeightPacketView();
    RigExecWeightPacketBuild built;
    _Publish(RigExecDynamicWeightPacketKernel(
                 kind, inputs.rangePolicy == SchemaValueText::clamp,
                 inputs.driver, inputs.scale, inputs.bias,
                 base ? &view : nullptr, &built),
             &built, &packet);
    return packet;
}

// Volumetric weight objects.
// One shared body: read the placement and the band, pick a distance
// function, remap, and publish a dense packet. Only the distance function
// and the extra inputs it needs differ between sphere, plane, and curve.

bool
RigExecRigidWorldToLocal(const RigExecPointFrame &posed, GfMatrix4d *result)
{
    return RigExecRigidWorldToLocalKernel(
        posed.points, posed.IsValid(), posed.IsDegenerate(),
        [](const std::array<GfVec3d, 4> &rest,
           const std::array<GfVec3d, 4> &pose, GfMatrix4d *matrix) {
            return RigExecPointsToMatrix(rest, pose, matrix);
        },
        [](const GfMatrix4d &m) { return m.RemoveScaleShear(); }, result);
}

GfMatrix4d
RigExecVolumePlacement(const RigExecPointFrame &final)
{
    GfMatrix4d placement(1.0);
    RigExecVolumePlacement(
        final.points, final.IsValid(), final.IsDegenerate(),
        [](const std::array<GfVec3d, 4> &rest,
           const std::array<GfVec3d, 4> &pose, GfMatrix4d *matrix) {
            return RigExecPointsToMatrix(rest, pose, matrix);
        },
        &placement);
    return placement;
}

namespace {

// Per-axis divisors have to describe a volume: see _CheckVolumePrologue,
// which is the only caller.
bool
_ValidateScales(const GfVec3f &scales)
{
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(scales[a]) || scales[a] <= 0.0f) {
            return false;
        }
    }
    return true;
}

// True for the shapes whose distance function divides local coordinates
// by inputs.scales. The plane ignores them, so a plane whose scaleX is
// zero is still a perfectly good plane.
bool
_UsesAxisScales(const TfToken &typeName)
{
    return typeName == _weightTokens->sphere ||
           typeName == _weightTokens->curve;
}

// Everything a volumetric weight can settle before it looks at a single
// point: the structural tokens, the placement, and the divisors the shape
// is about to divide by. Kept separate from the rest of the prologue so a
// caller that has to gather a whole mesh to fill in targetPoints can ask
// first -- see RigExecVolumeWeightCanBuild.
bool
_CheckVolumePrologue(bool usesScales, const RigExecVolumeWeightInputs &inputs,
                     GfMatrix4d *worldToLocal)
{
    // A generated field has a value at every element, so dense is the only
    // representation it can honestly publish. Rejected, never coerced.
    if (inputs.representation != SchemaValueText::dense) {
        return false;
    }
    if (inputs.rangePolicy != SchemaValueText::strict &&
        inputs.rangePolicy != SchemaValueText::clamp) {
        return false;
    }
    if (!inputs.hasPlacement ||
        !RigExecRigidWorldToLocal(inputs.placement, worldToLocal)) {
        return false;
    }
    // Non-positive or non-finite collapses the volume, so it invalidates
    // the packet rather than dividing by zero.
    if (usesScales && !_ValidateScales(inputs.scales)) {
        return false;
    }
    return true;
}

// The shared prologue: structural tokens, placement, band, and the points
// the field is measured over. Returns false when the packet is invalid.
bool
_BeginVolumeWeight(bool usesScales, const RigExecVolumeWeightInputs &inputs,
                   RigExecWeightPacket *packet, GfMatrix4d *worldToLocal,
                   RigExecWeightPointView *points)
{
    packet->representation = inputs.representation;
    packet->rangePolicy = inputs.rangePolicy;
    if (!_CheckVolumePrologue(usesScales, inputs, worldToLocal)) {
        return false;
    }

    // rigExec:sampleSource, when authored, replaces the weightTarget for
    // SAMPLING only -- the weighted domain stays the target, so the two
    // must still agree in cardinality.
    const RigExecWeightPointView target =
        inputs.usePointViews
            ? inputs.targetView
            : RigExecWeightPointView{inputs.targetPoints.data(),
                                     inputs.targetPoints.size()};
    const RigExecWeightPointView samples =
        inputs.usePointViews
            ? inputs.sampleView
            : RigExecWeightPointView{inputs.samplePoints.data(),
                                     inputs.samplePoints.size()};
    const size_t targetCount =
        inputs.targetPointCount != 0 ? inputs.targetPointCount : target.count;
    if (targetCount == 0)
        return false;
    if (samples.count != 0 && samples.count != targetCount)
        return false;
    if (samples.count == 0 && target.count != targetCount)
        return false;
    *points = samples.count == 0 ? target : samples;
    return true;
}

// The shared epilogue: range policy over the generated field.
bool
_FinishVolumeWeight(RigExecWeightPacket *packet, std::vector<float> *weights)
{
    for (float &w : *weights) {
        if (!RigExecApplyWeightRangePolicy(packet->rangePolicy, &w)) {
            return false;
        }
    }
    packet->values = std::move(*weights);
    packet->defaultWeight = 0.0f; // canonical dense default
    packet->valid = true;
    return true;
}

// Divides local coordinates by the per-axis scales, turning the sphere's
// iso-surfaces into ellipsoids and the curve's tube into an elliptical
// one. Folded into the matrix so the hot loop stays a single transform.
GfMatrix4d
_ApplyAxisScales(const GfMatrix4d &worldToLocal, const GfVec3f &scales)
{
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(scales[0]), 1.0 / double(scales[1]),
                            1.0 / double(scales[2])));
    return worldToLocal * divide;
}

RigExecWeightPacket
_BuildSphereWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal, &points)) {
        return packet;
    }
    if (!_ValidateScales(inputs.positiveScales) ||
        !_ValidateScales(inputs.negativeScales))
        return packet;
    std::vector<float> weights;
    RigExecSphereWeightField(points,
                             _ApplyAxisScales(worldToLocal, inputs.scales),
                             inputs.params, &weights, inputs.positiveScales,
                             inputs.negativeScales, RigExecGeometryRanges());
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildPlaneWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ false, inputs, &packet, &worldToLocal,
            &points)) {
        return packet;
    }
    int axisIndex;
    if (inputs.planeAxis == SchemaValueText::x) {
        axisIndex = 0;
    } else if (inputs.planeAxis == SchemaValueText::y) {
        axisIndex = 1;
    } else if (inputs.planeAxis == SchemaValueText::z) {
        axisIndex = 2;
    } else {
        return packet; // unknown structural token: rejected, not coerced
    }

    // Bounded clips the field to the in-plane rectangle; unbounded is
    // the infinite half-space gradient. The extents are consulted ONLY in
    // the bounded arm: unbounded does not use them for the field (they
    // still size the drawn guide), so a bad extent there is a legibility
    // problem, not a reason to invalidate the whole rig.
    RigExecPlaneBounds extent;
    const RigExecPlaneBounds *extentPtr = nullptr;
    if (inputs.planeBounds == _weightTokens->bounded) {
        extent.extentU = inputs.extentU;
        extent.extentV = inputs.extentV;
        for (const float e : {extent.extentU, extent.extentV}) {
            if (!std::isfinite(e) || e <= 0.0f) {
                return packet; // no such rectangle; same rule as scales
            }
        }
        extentPtr = &extent;
    } else if (inputs.planeBounds != _weightTokens->unbounded) {
        return packet; // unknown structural token: rejected, not coerced
    }

    std::vector<float> weights;
    RigExecPlaneWeightField(points, worldToLocal, axisIndex, inputs.params,
                            &weights, extentPtr, RigExecGeometryRanges());
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildCurveWeightPacket(const RigExecVolumeWeightInputs &inputs)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    RigExecWeightPointView points;
    if (!_BeginVolumeWeight(
            /* usesScales = */ true, inputs, &packet, &worldToLocal, &points)) {
        return packet;
    }
    const RigExecWeightPointView curve =
        inputs.usePointViews
            ? inputs.curveView
            : RigExecWeightPointView{inputs.curvePoints.data(),
                                     inputs.curvePoints.size()};
    if (curve.count == 0) {
        return packet; // a curve weight with no curve is not a field
    }
    std::vector<float> weights;
    std::vector<GfVec3f> localCurve;
    RigExecCurveWeightField(
        points, curve, _ApplyAxisScales(worldToLocal, inputs.scales),
        inputs.params, &weights,
        inputs.localCurveScratch ? inputs.localCurveScratch : &localCurve,
        RigExecGeometryRanges());
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

} // namespace

bool
RigExecVolumeWeightCanBuild(const TfToken &typeName,
                            const RigExecVolumeWeightInputs &inputs)
{
    GfMatrix4d worldToLocal(1.0);
    if (typeName == SchemaTypeName::RigExecSphereWeight &&
        (!_ValidateScales(inputs.positiveScales) ||
         !_ValidateScales(inputs.negativeScales)))
        return false;
    return _CheckVolumePrologue(_UsesAxisScales(typeName), inputs,
                                &worldToLocal);
}

RigExecWeightPacket
RigExecBuildVolumeWeightPacket(const TfToken &typeName,
                               const RigExecVolumeWeightInputs &inputs)
{
    if (typeName == _weightTokens->sphere) {
        return _BuildSphereWeightPacket(inputs);
    }
    if (typeName == _weightTokens->plane) {
        return _BuildPlaneWeightPacket(inputs);
    }
    if (typeName == _weightTokens->curve) {
        return _BuildCurveWeightPacket(inputs);
    }
    // Shaped like every other rejection rather than a bare packet, so a
    // caller that mistypes the token sees the same failure object a
    // rejected representation produces instead of a subtly different one.
    RigExecWeightPacket packet;
    packet.representation = inputs.representation;
    packet.rangePolicy = inputs.rangePolicy;
    return packet;
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount, float strength, float invert)
{
    RigExecWeightPacketWorkspace workspace;
    return RigExecBuildCombineWeightPacket(
        representation, rangePolicy, combineMode, inputs, weightTargetCount,
        strength, invert, &workspace);
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount, float strength, float invert,
    RigExecWeightPacketWorkspace *workspace)
{
    if (!workspace)
        return RigExecBuildCombineWeightPacket(
            representation, rangePolicy, combineMode, inputs, weightTargetCount,
            strength, invert);
    RigExecWeightPacket packet;
    packet.representation = representation;
    packet.rangePolicy = rangePolicy;
    if (packet.representation != SchemaValueText::dense) {
        return packet;
    }
    if (packet.rangePolicy != SchemaValueText::strict &&
        packet.rangePolicy != SchemaValueText::clamp) {
        return packet;
    }

    RigExecWeightCombine mode;
    if (combineMode == SchemaValueText::multiply) {
        mode = RigExecWeightCombine::Multiply;
    } else if (combineMode == SchemaValueText::add) {
        mode = RigExecWeightCombine::Add;
    } else if (combineMode == SchemaValueText::subtract) {
        mode = RigExecWeightCombine::Subtract;
    } else if (combineMode == SchemaValueText::max) {
        mode = RigExecWeightCombine::Max;
    } else if (combineMode == SchemaValueText::min) {
        mode = RigExecWeightCombine::Min;
    } else if (combineMode == SchemaValueText::average) {
        mode = RigExecWeightCombine::Average;
    } else if (combineMode == SchemaValueText::overlay) {
        mode = RigExecWeightCombine::Overlay;
    } else {
        return packet;
    }

    // A constant-only combine takes its element count from its own
    // rigExec:weightTarget, as the CPU oracle does.
    std::vector<RigExecWeightPacketView> views;
    views.reserve(inputs.size());
    for (const RigExecWeightPacket *in : inputs) {
        if (!in)
            return packet;
        views.push_back(_View(*in));
    }
    RigExecWeightPacketBuild built;
    if (RigExecCombineWeightPacketKernel(
            mode, packet.rangePolicy == SchemaValueText::clamp, strength,
            invert, views.size(),
            [&](size_t k) -> const RigExecWeightPacketView & {
                return views[k];
            },
            weightTargetCount, &workspace->combineField, &built))
        return packet;
    packet.values = std::move(built.values);
    packet.defaultWeight = 0.0f;
    packet.valid = true;
    return packet;
}

RigExecWeightPacket
RigExecBuildCombineWeightPacket(const TfToken &representation,
                                const TfToken &rangePolicy,
                                const TfToken &combineMode,
                                const std::vector<RigExecWeightPacket> &inputs,
                                size_t weightTargetCount, float strength,
                                float invert)
{
    std::vector<const RigExecWeightPacket *> borrowed;
    borrowed.reserve(inputs.size());
    for (const RigExecWeightPacket &in : inputs) {
        borrowed.push_back(&in);
    }
    return RigExecBuildCombineWeightPacket(representation, rangePolicy,
                                           combineMode, borrowed,
                                           weightTargetCount, strength, invert);
}

} // namespace rigExec
