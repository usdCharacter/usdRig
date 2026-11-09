// Codified schema names for C++ consumers of the codeless RigExec schema:
// type names, attribute names, and authored value tokens, declared once so
// no use site spells them as strings. schema.usda is the authority; keep
// these lists in step with it. Header-only: the schema ships as a codeless
// resource plugin with no library of its own, so the constants live as
// inline namespace-scope tokens (built once at load; copying, comparing,
// and hashing them is lock-free).
#ifndef RIGEXEC_SCHEMA_TOKENS_H
#define RIGEXEC_SCHEMA_TOKENS_H

#include "pxr/base/tf/token.h"

namespace rigExec {

// Schema type names (the class names in schema.usda).
// Text constants first: they need no dynamic initialization, so they are
// safe inside TF_REGISTRY_FUNCTION bodies, which can run while the
// consuming dylib is still loading -- before inline TfToken globals in
// any translation unit have been constructed.
namespace SchemaTypeName {
inline constexpr const char *const RigExecRoot = "RigExecRoot";
inline constexpr const char *const RigExecXformable = "RigExecXformable";
inline constexpr const char *const RigExecControl = "RigExecControl";
inline constexpr const char *const RigExecJoint = "RigExecJoint";
inline constexpr const char *const RigExecVolumeWeight = "RigExecVolumeWeight";
inline constexpr const char *const RigExecSphereWeight = "RigExecSphereWeight";
inline constexpr const char *const RigExecPlaneWeight = "RigExecPlaneWeight";
inline constexpr const char *const RigExecCurveWeight = "RigExecCurveWeight";
inline constexpr const char *const RigExecClothConstraintSet =
    "RigExecClothConstraintSet";
inline constexpr const char *const RigExecFkChain = "RigExecFkChain";
inline constexpr const char *const RigExecTwoBoneIk = "RigExecTwoBoneIk";
inline constexpr const char *const RigExecBlendPointFrames =
    "RigExecBlendPointFrames";
inline constexpr const char *const RigExecTwistDistribution =
    "RigExecTwistDistribution";
inline constexpr const char *const RigExecRibbon = "RigExecRibbon";
inline constexpr const char *const RigExecSplineIk = "RigExecSplineIk";
} // namespace SchemaTypeName

namespace SchemaType {
inline const PXR_NS::TfToken RigExecRoot(SchemaTypeName::RigExecRoot);
inline const PXR_NS::TfToken RigExecXformable(SchemaTypeName::RigExecXformable);
inline const PXR_NS::TfToken RigExecControl(SchemaTypeName::RigExecControl);
inline const PXR_NS::TfToken RigExecJoint(SchemaTypeName::RigExecJoint);
inline const PXR_NS::TfToken
    RigExecVolumeWeight(SchemaTypeName::RigExecVolumeWeight);
inline const PXR_NS::TfToken
    RigExecSphereWeight(SchemaTypeName::RigExecSphereWeight);
inline const PXR_NS::TfToken
    RigExecPlaneWeight(SchemaTypeName::RigExecPlaneWeight);
inline const PXR_NS::TfToken
    RigExecCurveWeight(SchemaTypeName::RigExecCurveWeight);
inline const PXR_NS::TfToken
    RigExecClothConstraintSet(SchemaTypeName::RigExecClothConstraintSet);
inline const PXR_NS::TfToken RigExecFkChain(SchemaTypeName::RigExecFkChain);
inline const PXR_NS::TfToken RigExecTwoBoneIk(SchemaTypeName::RigExecTwoBoneIk);
inline const PXR_NS::TfToken
    RigExecBlendPointFrames(SchemaTypeName::RigExecBlendPointFrames);
inline const PXR_NS::TfToken
    RigExecTwistDistribution(SchemaTypeName::RigExecTwistDistribution);
inline const PXR_NS::TfToken RigExecRibbon(SchemaTypeName::RigExecRibbon);
inline const PXR_NS::TfToken RigExecSplineIk(SchemaTypeName::RigExecSplineIk);
} // namespace SchemaType

// Authored attributes and relationships read without the evaluator.
namespace SchemaAttr {
inline const PXR_NS::TfToken guideCurveVertexCounts("guide:curveVertexCounts");
inline const PXR_NS::TfToken guideDisplayOpacity("guide:displayOpacity");
inline const PXR_NS::TfToken guideDrawMode("guide:drawMode");
inline const PXR_NS::TfToken guideOffset("guide:offset");
inline const PXR_NS::TfToken guidePoints("guide:points");
inline const PXR_NS::TfToken guideRadius("guide:radius");
inline const PXR_NS::TfToken guideScaleX("guide:scaleX");
inline const PXR_NS::TfToken guideScaleY("guide:scaleY");
inline const PXR_NS::TfToken guideScaleZ("guide:scaleZ");
inline const PXR_NS::TfToken guideShape("guide:shape");
inline const PXR_NS::TfToken guideWireWidth("guide:wireWidth");
inline const PXR_NS::TfToken avarsSx("avars:sx");
inline const PXR_NS::TfToken avarsSy("avars:sy");
inline const PXR_NS::TfToken avarsSz("avars:sz");
inline const PXR_NS::TfToken restSpace("rest:space");
inline const PXR_NS::TfToken rigExecCurve("rigExec:curve");
inline const PXR_NS::TfToken rigExecPlaneAxis("rigExec:planeAxis");
inline const PXR_NS::TfToken rigExecPlaneBounds("rigExec:planeBounds");
inline const PXR_NS::TfToken inputsExtentU("inputs:extentU");
inline const PXR_NS::TfToken inputsExtentV("inputs:extentV");
inline const PXR_NS::TfToken inputsFalloffMax("inputs:falloffMax");
inline const PXR_NS::TfToken inputsFalloffMin("inputs:falloffMin");
inline const PXR_NS::TfToken inputsScaleX("inputs:scaleX");
inline const PXR_NS::TfToken inputsScaleY("inputs:scaleY");
inline const PXR_NS::TfToken inputsScaleZ("inputs:scaleZ");
inline const PXR_NS::TfToken inputsScaleXNeg("inputs:scaleXNeg");
inline const PXR_NS::TfToken inputsScaleYNeg("inputs:scaleYNeg");
inline const PXR_NS::TfToken inputsScaleZNeg("inputs:scaleZNeg");
inline const PXR_NS::TfToken inputsScaleXPos("inputs:scaleXPos");
inline const PXR_NS::TfToken inputsScaleYPos("inputs:scaleYPos");
inline const PXR_NS::TfToken inputsScaleZPos("inputs:scaleZPos");
} // namespace SchemaAttr

// Authored value tokens for those attributes.
namespace SchemaValue {
inline const PXR_NS::TfToken x("x");
inline const PXR_NS::TfToken y("y");
inline const PXR_NS::TfToken z("z");
inline const PXR_NS::TfToken bounded("bounded");
inline const PXR_NS::TfToken unbounded("unbounded");
inline const PXR_NS::TfToken shapeCircle("circle");
inline const PXR_NS::TfToken shapeCustom("custom");
inline const PXR_NS::TfToken drawModeBrackets("brackets");
inline const PXR_NS::TfToken drawModeCage("cage");
inline const PXR_NS::TfToken drawModeFalloff("falloff");
inline const PXR_NS::TfToken drawModeGeometry("geometry");
inline const PXR_NS::TfToken drawModeShell("shell");
inline const PXR_NS::TfToken drawModeWire("wire");
} // namespace SchemaValue

} // namespace rigExec

#endif
