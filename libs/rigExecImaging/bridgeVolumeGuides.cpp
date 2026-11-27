// Guide rendering, part 2: the influence-volume iso-surfaces and the
// weight overlay. Same component as bridgeGuides.cpp (bridgeGuides.h).
#include "rigExecSchema/schemaNames.h"
#include "rigExecSchema/schemaTokens.h"
#include "bridgeGuides.h"

#include "bridgeGuideGeometry.h"
#include "rigExec/api/frameReader.h"
#include "rigExec/api/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <cmath>
#include <utility>
#include <vector>

namespace rigExec {

// Every placed influence volume draws its falloffMin and falloffMax
// iso-surfaces.
// Driven by the generation's weight frames rather than by a second stage traversal: the
// evaluator discovers every placed volume beneath the rig and resolves its
// frame whether or not a mover consumes the field. That makes the schema's
// selectable/framable guide contract useful during authoring, before the
// volume is wired, while keeping drawing on the exact same placement path the
// field uses once it is consumed.
void
RigExecBridgeGuides::_FillVolumeGuides(const RigExecGuideCall &call,
                                       const RigExecFrameReader &reader,
                                       RigExecImagingSnapshot *snapshot)
{
    // Set here too: a rig may publish these and no joint or control
    // guides at all, in which case this is the only place the asset-space
    // anchor gets recorded (see _FillGuides).
    snapshot->assetRoot = _rigPath.GetParentPath();

    const size_t volumeCount = reader.GetCount(RigExecFrameDomain::WeightFrame);
    for (size_t volume = 0; volume < volumeCount; ++volume) {
        const SdfPath &weightPath =
            reader.GetPath(RigExecFrameDomain::WeightFrame, volume);
        const GfMatrix4d placement =
            reader.GetMatrix(RigExecFrameDomain::WeightFrame, volume);
        const UsdPrim prim = call.stage->GetPrimAtPath(weightPath);
        if (!prim) {
            continue;
        }
        // A stage composed without the codeless schema
        // plugin registered has no fallback to resolve, and a volume would
        // silently stop drawing rather than draw the documented default.
        TfToken drawMode(_tFalloff);
        if (UsdAttribute a = prim.GetAttribute(_tGuideDrawMode)) {
            a.Get(&drawMode, reader.GetTime());
        }
        const bool clean = drawMode == _tCage || drawMode == _tBrackets ||
                           drawMode == _tShell || drawMode == _tFalloff;
        const bool shell = drawMode == _tShell;
        const bool falloff = drawMode == _tFalloff;
        if (drawMode != _tWire && drawMode != _tGeometry && !clean) {
            // `none` -- and any unrecognized token, which has to mean
            // something definite and means the same thing here as an
            // unknown control shape does: nothing is drawn.
            continue;
        }
        const bool wire = drawMode != _tGeometry;

        GfMatrix4d rigid(1.0);
        if (!_RigidVolumeMatrix(placement, &rigid)) {
            continue;
        }

        auto readFloat = [this, &prim, &reader, &call,
                          &weightPath](const TfToken &name, float fallback) {
            float value = fallback;
            if (UsdAttribute a = prim.GetAttribute(name)) {
                // Property movers can drive volume dimensions. The field
                // consumes these evaluated values, so its iso-surfaces must
                // use the same values instead of the authored source.
                float moved = 0.0f;
                if (reader.FindMovedValue(a.GetPath(), &moved)) {
                    return moved;
                }
                for (const auto &override : call.displayOverrides) {
                    if (override.prim == weightPath &&
                        override.attribute == a.GetName()) {
                        if (override.value.IsHolding<float>())
                            return override.value.UncheckedGet<float>();
                        if (override.value.IsHolding<double>())
                            return float(override.value.UncheckedGet<double>());
                    }
                }
                VtValue authored;
                if (a.Get(&authored, reader.GetTime())) {
                    if (authored.IsHolding<float>())
                        value = authored.UncheckedGet<float>();
                    else if (authored.IsHolding<double>())
                        value = float(authored.UncheckedGet<double>());
                }
            }
            return value;
        };
        const double falloffMin = readFloat(_volumeInputsFalloffMin, 0.0f);
        const double falloffMax = readFloat(_volumeInputsFalloffMax, 1.0f);
        if (!std::isfinite(falloffMin) || !std::isfinite(falloffMax)) {
            continue;
        }
        double wireWidth = readFloat(_tGuideWireWidth, 0.05f);
        // A non-positive width is not a reason to draw nothing: it
        // selects the hairline fallback, exactly as it does for a control.
        if (!wire || !std::isfinite(wireWidth) || wireWidth <= 0.0) {
            wireWidth = 0.0;
        }

        const TfToken typeName = prim.GetTypeName();
        std::vector<RigExecVolumeGuideElement> elements;
        const float guideOpacity = readFloat(_tGuideDisplayOpacity, 0.35f);
        auto finishBoundary = [&](size_t first, bool outer) {
            if (!clean)
                return;
            for (size_t i = first; i < elements.size(); ++i) {
                auto &element = elements[i];
                if (element.primType == HdPrimTypeTokens->basisCurves) {
                    element.wireWidth *= 0.3;
                    element.opacity = guideOpacity * (outer ? 0.75f : 1.0f);
                    if (drawMode == _tBrackets || (outer && !shell))
                        _PatternVolumeCurves(&element, drawMode == _tBrackets);
                } else {
                    element.opacity = guideOpacity * 0.25f;
                }
            }
        };

        if (typeName == SchemaTypeName::RigExecPlaneWeight) {
            TfToken axisToken(SchemaFallback::rigExecPlaneAxis);
            if (UsdAttribute a = prim.GetAttribute(_tRigExecPlaneAxis)) {
                a.Get(&axisToken, reader.GetTime());
            }
            const int axis =
                axisToken == SchemaValueText::x
                    ? 0
                    : (axisToken == SchemaValueText::y
                           ? 1
                           : (axisToken == SchemaValueText::z ? 2 : -1));
            if (axis < 0) {
                continue; // the kernel rejects it too
            }
            // Size from the authored extents ONLY. The band moves the two
            // surfaces apart along the axis; it must not change their
            // size (a max(1, |min|, |max|) half-size would, on every scrub).
            const double extentU = readFloat(_volumeInputsExtentU, 1.0f);
            const double extentV = readFloat(_volumeInputsExtentV, 1.0f);
            TfToken boundsToken(SchemaFallback::rigExecPlaneBounds);
            if (UsdAttribute a = prim.GetAttribute(_tRigExecPlaneBounds)) {
                a.Get(&boundsToken, reader.GetTime());
            }
            if (boundsToken != SchemaValueText::unbounded &&
                boundsToken != SchemaValueText::bounded) {
                continue; // the kernel rejects it too
            }
            const bool bounded = boundsToken == SchemaValueText::bounded;
            for (const double distance : {falloffMin, falloffMax}) {
                const size_t first = elements.size();
                if ((shell && distance == falloffMax) ||
                    (falloff && distance == falloffMin))
                    _AppendPlaneVolumeGuide(distance, axis, extentU, extentV,
                                            bounded, false, rigid, wireWidth,
                                            &elements);
                _AppendPlaneVolumeGuide(distance, axis, extentU, extentV,
                                        bounded, wire, rigid, wireWidth,
                                        &elements);
                finishBoundary(first, distance == falloffMax);
            }
        } else {
            // Sphere and curve share the per-axis divisors, and reject the
            // same values the kernel rejects -- a volume whose packet is
            // invalid must not draw a shape suggesting it is fine.
            const GfVec3d axisScale(readFloat(_volumeInputsScaleX, 1.0f),
                                    readFloat(_volumeInputsScaleY, 1.0f),
                                    readFloat(_volumeInputsScaleZ, 1.0f));
            bool scaleOk = true;
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(axisScale[i]) || axisScale[i] <= 0.0) {
                    scaleOk = false;
                }
            }
            if (!scaleOk) {
                continue;
            }
            if (typeName == SchemaTypeName::RigExecSphereWeight) {
                const GfVec3d positiveScales(
                    readFloat(_volumeInputsScaleXPos, 1.0f),
                    readFloat(_volumeInputsScaleYPos, 1.0f),
                    readFloat(_volumeInputsScaleZPos, 1.0f));
                const GfVec3d negativeScales(
                    readFloat(_volumeInputsScaleXNeg, 1.0f),
                    readFloat(_volumeInputsScaleYNeg, 1.0f),
                    readFloat(_volumeInputsScaleZNeg, 1.0f));
                for (int axis = 0; axis < 3; ++axis) {
                    if (!std::isfinite(positiveScales[axis]) ||
                        positiveScales[axis] <= 0 ||
                        !std::isfinite(negativeScales[axis]) ||
                        negativeScales[axis] <= 0)
                        scaleOk = false;
                }
                if (!scaleOk)
                    continue;
                // falloffMin is fully on. In wire mode that interior is a
                // half-translucent fill plus the same dotted great circles
                // the exterior uses. falloffMax stays dotted wire only.
                // geometry mode still draws both surfaces solid. A
                // non-positive radius is skipped inside the append, so
                // falloffMin = 0 draws the dotted exterior alone.
                const double radii[2] = {falloffMin, falloffMax};
                for (int surface = 0; surface < 2; ++surface) {
                    const size_t first = elements.size();
                    if ((shell && surface == 1) || (falloff && surface == 0))
                        _AppendSphereVolumeGuide(radii[surface], axisScale,
                                                 positiveScales, negativeScales,
                                                 false, rigid, wireWidth, -1.0f,
                                                 &elements, clean);
                    const bool interior = wire && !clean && surface == 0;
                    if (interior) {
                        _AppendSphereVolumeGuide(
                            radii[surface], axisScale, positiveScales,
                            negativeScales, false, rigid, wireWidth,
                            _kSphereInteriorOpacity, &elements);
                        _AppendSphereVolumeGuide(radii[surface], axisScale,
                                                 positiveScales, negativeScales,
                                                 true, rigid, wireWidth, -1.0f,
                                                 &elements);
                    } else {
                        _AppendSphereVolumeGuide(radii[surface], axisScale,
                                                 positiveScales, negativeScales,
                                                 wire, rigid, wireWidth, -1.0f,
                                                 &elements, clean);
                    }
                    finishBoundary(first, surface == 1);
                }
            } else if (typeName == SchemaTypeName::RigExecCurveWeight) {
                VtVec3fArray curvePoints;
                if (!_ReadCurveGuidePoints(prim, reader.GetTime(),
                                           &curvePoints)) {
                    continue;
                }
                // Into the DIVIDED local space the kernel measures in:
                // rigid^-1 then the per-axis divisors, so the drawn tube
                // is round exactly where the field's is.
                GfMatrix4d divide(1.0);
                divide.SetScale(GfVec3d(1.0 / axisScale[0], 1.0 / axisScale[1],
                                        1.0 / axisScale[2]));
                const GfMatrix4d toLocal = rigid.GetInverse() * divide;
                VtVec3fArray localCurve(curvePoints.size());
                for (size_t k = 0; k < curvePoints.size(); ++k) {
                    localCurve[k] = GfVec3f(
                        toLocal.Transform(GfVec3d(curvePoints.cdata()[k])));
                }
                GfMatrix4d scale(1.0);
                scale.SetScale(axisScale);
                const GfMatrix4d toAsset = scale * rigid;
                for (const double radius : {falloffMin, falloffMax}) {
                    const size_t first = elements.size();
                    if ((shell && radius == falloffMax) ||
                        (falloff && radius == falloffMin))
                        _AppendCurveVolumeGuide(radius, localCurve, toAsset,
                                                false, wireWidth, &elements,
                                                clean);
                    _AppendCurveVolumeGuide(radius, localCurve, toAsset, wire,
                                            wireWidth, &elements, clean);
                    finishBoundary(first, radius == falloffMax);
                }
            } else {
                continue; // not a shape this version draws
            }
        }

        if (elements.empty()) {
            continue;
        }
        // Composed with the asset root by the scene index, like every
        // other guide: these frames carry no stage placement.
        RigExecPublishedPrim &published = snapshot->prims[weightPath];
        published.assetRoot = _rigPath.GetParentPath();
        published.hasVolumeGuides = true;
        published.hasVolumeGuideFrame = true;
        published.volumeGuideFrame = rigid;
        published.volumeGuides = std::move(elements);
        _ReadGuideStyle(prim, reader, &published);
    }
}

// Paints ONE selected weight object's resolved field onto the geometry it
// weights.
// The field is taken from the generation's weight fields -- the packet a mover actually
// consumed -- rather than recomputed, so what a rigger sees is exactly what
// deformed the geometry. A re-derivation could drift, and it would drift
// silently: a plausible-looking gradient over the wrong region is
// indistinguishable from a correct one by eye, which is the whole reason
// the visualisation exists.
void
RigExecBridgeGuides::_FillWeightOverlay(const RigExecGuideCall &call,
                                        const RigExecFrameReader &reader,
                                        RigExecImagingSnapshot *snapshot)
{
    const SdfPath &weightOverlay = call.weightOverlay;
    if (weightOverlay.IsEmpty()) {
        return; // off: an ordinary render is untouched
    }
    size_t field = 0;
    if (!reader.FindIndex(RigExecFrameDomain::WeightField, weightOverlay,
                          &field)) {
        return;
    }
    VtFloatArray weights = reader.GetWeightFieldWeights(field);
    if (weights.empty()) {
        return;
    }
    // The field carries its own geometry target: the selection names a
    // WEIGHT OBJECT, and which prim that paints is the evaluator's answer,
    // not the caller's.
    const SdfPath geomPath = reader.GetWeightFieldTarget(field).GetPrimPath();
    if (geomPath.IsEmpty() || geomPath.IsAbsoluteRootPath()) {
        return;
    }
    RigExecPublishedPrim &published = snapshot->prims[geomPath];
    published.hasWeightOverlay = true;
    published.weightOverlay = std::move(weights);
    TfToken display;
    const UsdPrim weight = call.stage->GetPrimAtPath(weightOverlay);
    if (weight) {
        if (const UsdAttribute attr = weight.GetAttribute(_tWeightDisplay))
            attr.Get(&display, reader.GetTime());
    }
    published.weightDisplay = display == _tHeatmap
                                  ? RigExecWeightDisplay::Heatmap
                              : display == _tBands ? RigExecWeightDisplay::Bands
                                                   : RigExecWeightDisplay::Tint;
}

} // namespace rigExec
