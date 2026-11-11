// Codified schema names for stage-side C++ consumers of the codeless RigExec
// schema: the text constants in schemaNames.h, interned as tokens. Header-only:
// the schema ships as a codeless resource plugin with no library of its own, so
// the constants live as inline namespace-scope tokens (built once at load;
// copying, comparing, and hashing them is lock-free). The text constants need
// no dynamic initialization, so they stay usable inside TF_REGISTRY_FUNCTION
// bodies, which can run while the consuming dylib is still loading.
#ifndef RIGEXEC_SCHEMA_TOKENS_H
#define RIGEXEC_SCHEMA_TOKENS_H

#include "pxr/base/tf/token.h"
#include "rigExecSchema/schemaNames.h"

namespace rigExec {

// Type, attribute, and value names are spelled once in schemaNames.h; this
// header interns them for code that reads a stage.

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
inline const PXR_NS::TfToken
    RigExecSkeleton2d(SchemaTypeName::RigExecSkeleton2d);
inline const PXR_NS::TfToken
    RigExecPositionConstraint(SchemaTypeName::RigExecPositionConstraint);
inline const PXR_NS::TfToken
    RigExecRotationConstraint(SchemaTypeName::RigExecRotationConstraint);
inline const PXR_NS::TfToken
    RigExecScaleConstraint(SchemaTypeName::RigExecScaleConstraint);
inline const PXR_NS::TfToken
    RigExecParentConstraint(SchemaTypeName::RigExecParentConstraint);
inline const PXR_NS::TfToken
    RigExecAimConstraint(SchemaTypeName::RigExecAimConstraint);
inline const PXR_NS::TfToken RigExecSingleChainIkConstraint(
    SchemaTypeName::RigExecSingleChainIkConstraint);
inline const PXR_NS::TfToken
    RigExecIkFkMatchConstraint(SchemaTypeName::RigExecIkFkMatchConstraint);
inline const PXR_NS::TfToken
    RigExecHarmonicsConstraint(SchemaTypeName::RigExecHarmonicsConstraint);
inline const PXR_NS::TfToken
    RigExecControlAPI(SchemaTypeName::RigExecControlAPI);
inline const PXR_NS::TfToken RigExecMoverAPI(SchemaTypeName::RigExecMoverAPI);
inline const PXR_NS::TfToken
    RigExecJointGroupAPI(SchemaTypeName::RigExecJointGroupAPI);
inline const PXR_NS::TfToken RigExecBone2dAPI(SchemaTypeName::RigExecBone2dAPI);
inline const PXR_NS::TfToken
    RigExecBlendInput(SchemaTypeName::RigExecBlendInput);
inline const PXR_NS::TfToken
    RigExecClothBinding(SchemaTypeName::RigExecClothBinding);
inline const PXR_NS::TfToken
    RigExecFloatMathMover(SchemaTypeName::RigExecFloatMathMover);
inline const PXR_NS::TfToken
    RigExecDeltaMushMover(SchemaTypeName::RigExecDeltaMushMover);
inline const PXR_NS::TfToken
    RigExecLayeredSkinMover(SchemaTypeName::RigExecLayeredSkinMover);
inline const PXR_NS::TfToken
    RigExecBlendShapeMover(SchemaTypeName::RigExecBlendShapeMover);
inline const PXR_NS::TfToken
    RigExecCombineWeight(SchemaTypeName::RigExecCombineWeight);
inline const PXR_NS::TfToken
    RigExecDynamicWeight(SchemaTypeName::RigExecDynamicWeight);
inline const PXR_NS::TfToken
    RigExecLatticeMover(SchemaTypeName::RigExecLatticeMover);
inline const PXR_NS::TfToken
    RigExecClothMover(SchemaTypeName::RigExecClothMover);
inline const PXR_NS::TfToken
    RigExecCurveMover(SchemaTypeName::RigExecCurveMover);
inline const PXR_NS::TfToken
    RigExecAnimatedMap(SchemaTypeName::RigExecAnimatedMap);
inline const PXR_NS::TfToken
    RigExecAnimatedMaps(SchemaTypeName::RigExecAnimatedMaps);
inline const PXR_NS::TfToken
    RigExecArmatureParent(SchemaTypeName::RigExecArmatureParent);
inline const PXR_NS::TfToken
    RigExecAutoClavicle(SchemaTypeName::RigExecAutoClavicle);
inline const PXR_NS::TfToken
    RigExecBlendSample(SchemaTypeName::RigExecBlendSample);
inline const PXR_NS::TfToken RigExecBoneFrame(SchemaTypeName::RigExecBoneFrame);
inline const PXR_NS::TfToken RigExecClothMap(SchemaTypeName::RigExecClothMap);
inline const PXR_NS::TfToken
    RigExecConstraintFrame(SchemaTypeName::RigExecConstraintFrame);
inline const PXR_NS::TfToken RigExecCopyFrame(SchemaTypeName::RigExecCopyFrame);
inline const PXR_NS::TfToken
    RigExecMappedFrame(SchemaTypeName::RigExecMappedFrame);
inline const PXR_NS::TfToken
    RigExecMatrixMathMover(SchemaTypeName::RigExecMatrixMathMover);
inline const PXR_NS::TfToken
    RigExecMatrixMover(SchemaTypeName::RigExecMatrixMover);
inline const PXR_NS::TfToken RigExecMlMover(SchemaTypeName::RigExecMlMover);
inline const PXR_NS::TfToken RigExecMlOutput(SchemaTypeName::RigExecMlOutput);
inline const PXR_NS::TfToken
    RigExecPatchMover(SchemaTypeName::RigExecPatchMover);
inline const PXR_NS::TfToken RigExecPicker(SchemaTypeName::RigExecPicker);
inline const PXR_NS::TfToken
    RigExecPickerButton(SchemaTypeName::RigExecPickerButton);
inline const PXR_NS::TfToken
    RigExecPickerPanel(SchemaTypeName::RigExecPickerPanel);
inline const PXR_NS::TfToken RigExecPose(SchemaTypeName::RigExecPose);
inline const PXR_NS::TfToken
    RigExecPoseInterpolator(SchemaTypeName::RigExecPoseInterpolator);
inline const PXR_NS::TfToken
    RigExecSkinInfluence(SchemaTypeName::RigExecSkinInfluence);
inline const PXR_NS::TfToken RigExecSkinMover(SchemaTypeName::RigExecSkinMover);
inline const PXR_NS::TfToken
    RigExecSmoothMover(SchemaTypeName::RigExecSmoothMover);
inline const PXR_NS::TfToken
    RigExecSpaceSwitch(SchemaTypeName::RigExecSpaceSwitch);
inline const PXR_NS::TfToken
    RigExecStaticWeight(SchemaTypeName::RigExecStaticWeight);
inline const PXR_NS::TfToken
    RigExecSurfaceBindingMover(SchemaTypeName::RigExecSurfaceBindingMover);
inline const PXR_NS::TfToken
    RigExecSurfaceMover(SchemaTypeName::RigExecSurfaceMover);
inline const PXR_NS::TfToken
    RigExecSurfaceProjector(SchemaTypeName::RigExecSurfaceProjector);
inline const PXR_NS::TfToken
    RigExecTouchOverlay(SchemaTypeName::RigExecTouchOverlay);
inline const PXR_NS::TfToken
    RigExecTouchRegion(SchemaTypeName::RigExecTouchRegion);
inline const PXR_NS::TfToken
    RigExecTouchRegions(SchemaTypeName::RigExecTouchRegions);
inline const PXR_NS::TfToken
    RigExecVec3fMathMover(SchemaTypeName::RigExecVec3fMathMover);
inline const PXR_NS::TfToken
    RigExecVolumeCorrectMover(SchemaTypeName::RigExecVolumeCorrectMover);
inline const PXR_NS::TfToken
    RigExecWrinkleMover(SchemaTypeName::RigExecWrinkleMover);
} // namespace SchemaType

// Authored attributes and relationships read without the evaluator.
namespace SchemaAttr {
inline const PXR_NS::TfToken
    guideCurveVertexCounts(SchemaAttrText::guideCurveVertexCounts);
inline const PXR_NS::TfToken
    guideDisplayOpacity(SchemaAttrText::guideDisplayOpacity);
inline const PXR_NS::TfToken guideDrawMode(SchemaAttrText::guideDrawMode);
inline const PXR_NS::TfToken guideOffset(SchemaAttrText::guideOffset);
inline const PXR_NS::TfToken guidePoints(SchemaAttrText::guidePoints);
inline const PXR_NS::TfToken guideRadius(SchemaAttrText::guideRadius);
inline const PXR_NS::TfToken guideScaleX(SchemaAttrText::guideScaleX);
inline const PXR_NS::TfToken guideScaleY(SchemaAttrText::guideScaleY);
inline const PXR_NS::TfToken guideScaleZ(SchemaAttrText::guideScaleZ);
inline const PXR_NS::TfToken guideShape(SchemaAttrText::guideShape);
inline const PXR_NS::TfToken guideWireWidth(SchemaAttrText::guideWireWidth);
inline const PXR_NS::TfToken avarsSx(SchemaAttrText::avarsSx);
inline const PXR_NS::TfToken avarsSy(SchemaAttrText::avarsSy);
inline const PXR_NS::TfToken avarsSz(SchemaAttrText::avarsSz);
inline const PXR_NS::TfToken restSpace(SchemaAttrText::restSpace);
inline const PXR_NS::TfToken defaultSpace(SchemaAttrText::defaultSpace);
inline const PXR_NS::TfToken
    avarsDefaultSpace(SchemaAttrText::avarsDefaultSpace);
inline const PXR_NS::TfToken posedSpace(SchemaAttrText::posedSpace);
inline const PXR_NS::TfToken
    posedDefaultSpace(SchemaAttrText::posedDefaultSpace);
inline const PXR_NS::TfToken parentSpace(SchemaAttrText::parentSpace);
inline const PXR_NS::TfToken
    parentDefaultSpace(SchemaAttrText::parentDefaultSpace);
inline const PXR_NS::TfToken rigExecCurve(SchemaAttrText::rigExecCurve);
inline const PXR_NS::TfToken restTx(SchemaAttrText::restTx);
inline const PXR_NS::TfToken restTy(SchemaAttrText::restTy);
inline const PXR_NS::TfToken restTz(SchemaAttrText::restTz);
inline const PXR_NS::TfToken restRx(SchemaAttrText::restRx);
inline const PXR_NS::TfToken restRy(SchemaAttrText::restRy);
inline const PXR_NS::TfToken restRz(SchemaAttrText::restRz);
inline const PXR_NS::TfToken defaultTx(SchemaAttrText::defaultTx);
inline const PXR_NS::TfToken defaultTy(SchemaAttrText::defaultTy);
inline const PXR_NS::TfToken defaultTz(SchemaAttrText::defaultTz);
inline const PXR_NS::TfToken defaultRx(SchemaAttrText::defaultRx);
inline const PXR_NS::TfToken defaultRy(SchemaAttrText::defaultRy);
inline const PXR_NS::TfToken defaultRz(SchemaAttrText::defaultRz);
inline const PXR_NS::TfToken avarsTx(SchemaAttrText::avarsTx);
inline const PXR_NS::TfToken avarsTy(SchemaAttrText::avarsTy);
inline const PXR_NS::TfToken avarsTz(SchemaAttrText::avarsTz);
inline const PXR_NS::TfToken avarsRx(SchemaAttrText::avarsRx);
inline const PXR_NS::TfToken avarsRy(SchemaAttrText::avarsRy);
inline const PXR_NS::TfToken avarsRz(SchemaAttrText::avarsRz);
inline const PXR_NS::TfToken avarsRspin(SchemaAttrText::avarsRspin);
inline const PXR_NS::TfToken
    avarsUnitScaleFactor(SchemaAttrText::avarsUnitScaleFactor);
inline const PXR_NS::TfToken
    avarsRotationOrder(SchemaAttrText::avarsRotationOrder);
inline const PXR_NS::TfToken
    avarsRotationSign(SchemaAttrText::avarsRotationSign);
inline const PXR_NS::TfToken rigExecPlaneAxis(SchemaAttrText::rigExecPlaneAxis);
inline const PXR_NS::TfToken
    rigExecPlaneBounds(SchemaAttrText::rigExecPlaneBounds);
inline const PXR_NS::TfToken inputsExtentU(SchemaAttrText::inputsExtentU);
inline const PXR_NS::TfToken inputsExtentV(SchemaAttrText::inputsExtentV);
inline const PXR_NS::TfToken inputsFalloffMax(SchemaAttrText::inputsFalloffMax);
inline const PXR_NS::TfToken inputsFalloffMin(SchemaAttrText::inputsFalloffMin);
inline const PXR_NS::TfToken inputsScaleX(SchemaAttrText::inputsScaleX);
inline const PXR_NS::TfToken inputsScaleY(SchemaAttrText::inputsScaleY);
inline const PXR_NS::TfToken inputsScaleZ(SchemaAttrText::inputsScaleZ);
inline const PXR_NS::TfToken inputsScaleXNeg(SchemaAttrText::inputsScaleXNeg);
inline const PXR_NS::TfToken inputsScaleYNeg(SchemaAttrText::inputsScaleYNeg);
inline const PXR_NS::TfToken inputsScaleZNeg(SchemaAttrText::inputsScaleZNeg);
inline const PXR_NS::TfToken inputsScaleXPos(SchemaAttrText::inputsScaleXPos);
inline const PXR_NS::TfToken inputsScaleYPos(SchemaAttrText::inputsScaleYPos);
inline const PXR_NS::TfToken inputsScaleZPos(SchemaAttrText::inputsScaleZPos);
} // namespace SchemaAttr

// Provider computation identities, interned once for stage-side lookups.
namespace SchemaComputation {
inline const PXR_NS::TfToken
    computeRestFrame(SchemaComputationText::computeRestFrame);
inline const PXR_NS::TfToken
    computeDefaultFrame(SchemaComputationText::computeDefaultFrame);
inline const PXR_NS::TfToken
    computedDefaultSpace(SchemaComputationText::computedDefaultSpace);
inline const PXR_NS::TfToken computedParentDefaultSpace(
    SchemaComputationText::computedParentDefaultSpace);
inline const PXR_NS::TfToken
    computedParentSpace(SchemaComputationText::computedParentSpace);
inline const PXR_NS::TfToken
    computePointFrame(SchemaComputationText::computePointFrame);
inline const PXR_NS::TfToken
    computeBasePointFrame(SchemaComputationText::computeBasePointFrame);
inline const PXR_NS::TfToken
    computePointFrameArray(SchemaComputationText::computePointFrameArray);
inline const PXR_NS::TfToken
    computeMatrix(SchemaComputationText::computeMatrix);
inline const PXR_NS::TfToken
    computeAvarMatrix(SchemaComputationText::computeAvarMatrix);
inline const PXR_NS::TfToken
    computeWeightPacket(SchemaComputationText::computeWeightPacket);
inline const PXR_NS::TfToken
    computeFalloffLut(SchemaComputationText::computeFalloffLut);
inline const PXR_NS::TfToken
    computeConstraintWitness(SchemaComputationText::computeConstraintWitness);
inline const PXR_NS::TfToken
    computeBlendChannel(SchemaComputationText::computeBlendChannel);
inline const PXR_NS::TfToken
    computeResolvedValue(SchemaComputationText::computeResolvedValue);
inline const PXR_NS::TfToken computeValue(SchemaComputationText::computeValue);
inline const PXR_NS::TfToken
    interveningSpace(SchemaComputationText::interveningSpace);
inline const PXR_NS::TfToken localXform(SchemaComputationText::localXform);
} // namespace SchemaComputation

// Authored value tokens for those attributes.
namespace SchemaValue {
inline const PXR_NS::TfToken aim(SchemaValueText::aim);
inline const PXR_NS::TfToken chain(SchemaValueText::chain);
inline const PXR_NS::TfToken curve(SchemaValueText::curve);
inline const PXR_NS::TfToken linear(SchemaValueText::linear);
inline const PXR_NS::TfToken log(SchemaValueText::log);
inline const PXR_NS::TfToken none(SchemaValueText::none);
inline const PXR_NS::TfToken parentRelative(SchemaValueText::parentRelative);
inline const PXR_NS::TfToken rigid(SchemaValueText::rigid);
inline const PXR_NS::TfToken shortestArc(SchemaValueText::shortestArc);
inline const PXR_NS::TfToken softDistance(SchemaValueText::softDistance);
inline const PXR_NS::TfToken toChild(SchemaValueText::toChild);
inline const PXR_NS::TfToken uniformSegments(SchemaValueText::uniformSegments);
inline const PXR_NS::TfToken world(SchemaValueText::world);
inline const PXR_NS::TfToken x(SchemaValueText::x);
inline const PXR_NS::TfToken y(SchemaValueText::y);
inline const PXR_NS::TfToken z(SchemaValueText::z);
inline const PXR_NS::TfToken bounded(SchemaValueText::bounded);
inline const PXR_NS::TfToken unbounded(SchemaValueText::unbounded);
inline const PXR_NS::TfToken shapeCircle(SchemaValueText::circle);
inline const PXR_NS::TfToken shapeCustom(SchemaValueText::custom);
inline const PXR_NS::TfToken drawModeBrackets(SchemaValueText::brackets);
inline const PXR_NS::TfToken drawModeCage(SchemaValueText::cage);
inline const PXR_NS::TfToken drawModeFalloff(SchemaValueText::falloff);
inline const PXR_NS::TfToken drawModeGeometry(SchemaValueText::geometry);
inline const PXR_NS::TfToken drawModeShell(SchemaValueText::shell);
inline const PXR_NS::TfToken drawModeWire(SchemaValueText::wire);
} // namespace SchemaValue

} // namespace rigExec

#endif
