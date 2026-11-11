// Schema names as plain text: type names, attribute names, and authored value
// tokens, spelled once. schema.usda is the authority; keep these lists in step
// with it.
//
// Text, not TfToken, so the binary runtime -- which compares interned wire
// text and links no OpenUSD library -- can use them too. schemaTokens.h
// interns these constants for stage-side code, so a name is never declared
// twice.
#ifndef RIGEXEC_SCHEMA_NAMES_H
#define RIGEXEC_SCHEMA_NAMES_H

namespace rigExec {

// Prefixes shared by every RigExec schema name. A stage-side check that a
// type or primvar belongs to the rig tests this prefix rather than a literal.
namespace SchemaNamePrefix {
inline constexpr const char *const type = "RigExec";
inline constexpr const char *const touchType = "RigExecTouch";
inline constexpr const char *const attribute = "rigExec:";
inline constexpr const char *const avars = "avars:";
inline constexpr const char *const rest = "rest:";
inline constexpr const char *const guide = "guide:";
} // namespace SchemaNamePrefix

// CollectionAPI member property names. These come from OpenUSD's
// UsdCollectionAPI, not the RigExec schema, but they are read by the same
// scene lookups, so they are spelled once here.
namespace CollectionMemberText {
inline constexpr const char *const includes = "collection:members:includes";
inline constexpr const char *const excludes = "collection:members:excludes";
inline constexpr const char *const expansionRule =
    "collection:members:expansionRule";
inline constexpr const char *const membershipExpression =
    "collection:members:membershipExpression";
} // namespace CollectionMemberText

// Provider computation names: the identities a provider value is read
// through ("computePointFrame", "computeRestFrame", ...). The scene provider
// program declares them, and the evaluator, baked program, binary format,
// runtime and oracle all compare against the same spelling.
namespace SchemaComputationText {
inline constexpr const char *const computeRestFrame = "computeRestFrame";
inline constexpr const char *const computeDefaultFrame = "computeDefaultFrame";
inline constexpr const char *const computedDefaultSpace =
    "computedDefaultSpace";
inline constexpr const char *const computedParentDefaultSpace =
    "computedParentDefaultSpace";
inline constexpr const char *const computedParentSpace = "computedParentSpace";
inline constexpr const char *const computePointFrame = "computePointFrame";
inline constexpr const char *const computeBasePointFrame =
    "computeBasePointFrame";
inline constexpr const char *const computePointFrameArray =
    "computePointFrameArray";
inline constexpr const char *const computeMatrix = "computeMatrix";
inline constexpr const char *const computeAvarMatrix = "computeAvarMatrix";
inline constexpr const char *const computeWeightPacket = "computeWeightPacket";
inline constexpr const char *const computeFalloffLut = "computeFalloffLut";
inline constexpr const char *const computeConstraintWitness =
    "computeConstraintWitness";
inline constexpr const char *const computeBlendChannel = "computeBlendChannel";
inline constexpr const char *const computeBlendSampleData =
    "computeBlendSampleData";
inline constexpr const char *const computeResolvedValue =
    "computeResolvedValue";
inline constexpr const char *const computeValue = "computeValue";
inline constexpr const char *const interveningSpace = "interveningSpace";
inline constexpr const char *const localXform = "localXform";
} // namespace SchemaComputationText

// Schema type names (the class names in schema.usda).
namespace SchemaTypeName {
inline constexpr const char *const RigExecAimConstraint =
    "RigExecAimConstraint";
inline constexpr const char *const RigExecAnimatedMap = "RigExecAnimatedMap";
inline constexpr const char *const RigExecAnimatedMaps = "RigExecAnimatedMaps";
inline constexpr const char *const RigExecArmatureParent =
    "RigExecArmatureParent";
inline constexpr const char *const RigExecAutoClavicle = "RigExecAutoClavicle";
inline constexpr const char *const RigExecBlendInput = "RigExecBlendInput";
inline constexpr const char *const RigExecBlendPointFrames =
    "RigExecBlendPointFrames";
inline constexpr const char *const RigExecBlendSample = "RigExecBlendSample";
inline constexpr const char *const RigExecBlendShapeMover =
    "RigExecBlendShapeMover";
inline constexpr const char *const RigExecBone2dAPI = "RigExecBone2dAPI";
inline constexpr const char *const RigExecBoneFrame = "RigExecBoneFrame";
inline constexpr const char *const RigExecClothBinding = "RigExecClothBinding";
inline constexpr const char *const RigExecClothConstraintSet =
    "RigExecClothConstraintSet";
inline constexpr const char *const RigExecClothMap = "RigExecClothMap";
inline constexpr const char *const RigExecClothMover = "RigExecClothMover";
inline constexpr const char *const RigExecCombineWeight =
    "RigExecCombineWeight";
inline constexpr const char *const RigExecConstraintFrame =
    "RigExecConstraintFrame";
inline constexpr const char *const RigExecControl = "RigExecControl";
inline constexpr const char *const RigExecControlAPI = "RigExecControlAPI";
inline constexpr const char *const RigExecCopyFrame = "RigExecCopyFrame";
inline constexpr const char *const RigExecCurveMover = "RigExecCurveMover";
inline constexpr const char *const RigExecCurveWeight = "RigExecCurveWeight";
inline constexpr const char *const RigExecDeltaMushMover =
    "RigExecDeltaMushMover";
inline constexpr const char *const RigExecDynamicWeight =
    "RigExecDynamicWeight";
inline constexpr const char *const RigExecFkChain = "RigExecFkChain";
inline constexpr const char *const RigExecFloatMathMover =
    "RigExecFloatMathMover";
inline constexpr const char *const RigExecHarmonicsConstraint =
    "RigExecHarmonicsConstraint";
inline constexpr const char *const RigExecIkFkMatchConstraint =
    "RigExecIkFkMatchConstraint";
inline constexpr const char *const RigExecJoint = "RigExecJoint";
inline constexpr const char *const RigExecJointGroupAPI =
    "RigExecJointGroupAPI";
inline constexpr const char *const RigExecLatticeMover = "RigExecLatticeMover";
inline constexpr const char *const RigExecLayeredSkinMover =
    "RigExecLayeredSkinMover";
inline constexpr const char *const RigExecMappedFrame = "RigExecMappedFrame";
inline constexpr const char *const RigExecMatrixMathMover =
    "RigExecMatrixMathMover";
inline constexpr const char *const RigExecMatrixMover = "RigExecMatrixMover";
inline constexpr const char *const RigExecMlMover = "RigExecMlMover";
inline constexpr const char *const RigExecMlOutput = "RigExecMlOutput";
inline constexpr const char *const RigExecMoverAPI = "RigExecMoverAPI";
inline constexpr const char *const RigExecParentConstraint =
    "RigExecParentConstraint";
inline constexpr const char *const RigExecPatchMover = "RigExecPatchMover";
inline constexpr const char *const RigExecPicker = "RigExecPicker";
inline constexpr const char *const RigExecPickerButton = "RigExecPickerButton";
inline constexpr const char *const RigExecPickerPanel = "RigExecPickerPanel";
inline constexpr const char *const RigExecPlaneWeight = "RigExecPlaneWeight";
inline constexpr const char *const RigExecPose = "RigExecPose";
inline constexpr const char *const RigExecPoseInterpolator =
    "RigExecPoseInterpolator";
inline constexpr const char *const RigExecPositionConstraint =
    "RigExecPositionConstraint";
inline constexpr const char *const RigExecRibbon = "RigExecRibbon";
inline constexpr const char *const RigExecRoot = "RigExecRoot";
inline constexpr const char *const RigExecRotationConstraint =
    "RigExecRotationConstraint";
inline constexpr const char *const RigExecScaleConstraint =
    "RigExecScaleConstraint";
inline constexpr const char *const RigExecSingleChainIkConstraint =
    "RigExecSingleChainIkConstraint";
inline constexpr const char *const RigExecSkeleton2d = "RigExecSkeleton2d";
inline constexpr const char *const RigExecSkinInfluence =
    "RigExecSkinInfluence";
inline constexpr const char *const RigExecSkinMover = "RigExecSkinMover";
inline constexpr const char *const RigExecSmoothMover = "RigExecSmoothMover";
inline constexpr const char *const RigExecSpaceSwitch = "RigExecSpaceSwitch";
inline constexpr const char *const RigExecSphereWeight = "RigExecSphereWeight";
inline constexpr const char *const RigExecSplineIk = "RigExecSplineIk";
inline constexpr const char *const RigExecStaticWeight = "RigExecStaticWeight";
inline constexpr const char *const RigExecSurfaceBindingMover =
    "RigExecSurfaceBindingMover";
inline constexpr const char *const RigExecSurfaceMover = "RigExecSurfaceMover";
inline constexpr const char *const RigExecSurfaceProjector =
    "RigExecSurfaceProjector";
inline constexpr const char *const RigExecTouchOverlay = "RigExecTouchOverlay";
inline constexpr const char *const RigExecTouchRegion = "RigExecTouchRegion";
inline constexpr const char *const RigExecTouchRegions = "RigExecTouchRegions";
inline constexpr const char *const RigExecTwistDistribution =
    "RigExecTwistDistribution";
inline constexpr const char *const RigExecTwoBoneIk = "RigExecTwoBoneIk";
inline constexpr const char *const RigExecVec3fMathMover =
    "RigExecVec3fMathMover";
inline constexpr const char *const RigExecVolumeCorrectMover =
    "RigExecVolumeCorrectMover";
inline constexpr const char *const RigExecVolumeWeight = "RigExecVolumeWeight";
inline constexpr const char *const RigExecWrinkleMover = "RigExecWrinkleMover";
inline constexpr const char *const RigExecXformable = "RigExecXformable";
} // namespace SchemaTypeName

// Authored attribute names, including the provider spaces and the avar
// channels the baked program binds by name.
namespace SchemaAttrText {
inline constexpr const char *const avarsDefaultSpace = "avars:defaultSpace";
inline constexpr const char *const avarsRotationOrder = "avars:rotationOrder";
inline constexpr const char *const avarsRotationSign = "avars:rotationSign";
inline constexpr const char *const avarsRspin = "avars:rspin";
inline constexpr const char *const avarsRx = "avars:rx";
inline constexpr const char *const avarsRy = "avars:ry";
inline constexpr const char *const avarsRz = "avars:rz";
inline constexpr const char *const avarsSx = "avars:sx";
inline constexpr const char *const avarsSy = "avars:sy";
inline constexpr const char *const avarsSz = "avars:sz";
inline constexpr const char *const avarsTx = "avars:tx";
inline constexpr const char *const avarsTy = "avars:ty";
inline constexpr const char *const avarsTz = "avars:tz";
inline constexpr const char *const avarsUnitScaleFactor =
    "avars:unitScaleFactor";
inline constexpr const char *const bone2dAngleControl = "bone2d:angleControl";
inline constexpr const char *const bone2dAngleControlFactor =
    "bone2d:angleControlFactor";
inline constexpr const char *const bone2dAngleLimits = "bone2d:angleLimits";
inline constexpr const char *const bone2dColorTag = "bone2d:colorTag";
inline constexpr const char *const bone2dEndFlip = "bone2d:endFlip";
inline constexpr const char *const bone2dIgnoredByIk = "bone2d:ignoredByIk";
inline constexpr const char *const bone2dIndependentAngle =
    "bone2d:independentAngle";
inline constexpr const char *const bone2dLock = "bone2d:lock";
inline constexpr const char *const bone2dLockAngle = "bone2d:lockAngle";
inline constexpr const char *const bone2dLockX = "bone2d:lockX";
inline constexpr const char *const bone2dLockY = "bone2d:lockY";
inline constexpr const char *const bone2dMaxAngle = "bone2d:maxAngle";
inline constexpr const char *const bone2dMaxStretch = "bone2d:maxStretch";
inline constexpr const char *const bone2dMinAngle = "bone2d:minAngle";
inline constexpr const char *const bone2dOffsetX = "bone2d:offsetX";
inline constexpr const char *const bone2dOffsetY = "bone2d:offsetY";
inline constexpr const char *const bone2dPosControl = "bone2d:posControl";
inline constexpr const char *const bone2dPosControlFactorX =
    "bone2d:posControlFactorX";
inline constexpr const char *const bone2dPosControlFactorY =
    "bone2d:posControlFactorY";
inline constexpr const char *const bone2dScaleControl = "bone2d:scaleControl";
inline constexpr const char *const bone2dScaleControlFactor =
    "bone2d:scaleControlFactor";
inline constexpr const char *const bone2dScalingMode = "bone2d:scalingMode";
inline constexpr const char *const bone2dShowLabel = "bone2d:showLabel";
inline constexpr const char *const bone2dShy = "bone2d:shy";
inline constexpr const char *const bone2dSideFlip = "bone2d:sideFlip";
inline constexpr const char *const bone2dSquashFactor = "bone2d:squashFactor";
inline constexpr const char *const bone2dTargetCandidates =
    "bone2d:targetCandidates";
inline constexpr const char *const bone2dTargetIndex = "bone2d:targetIndex";
inline constexpr const char *const defaultRx = "default:rx";
inline constexpr const char *const defaultRy = "default:ry";
inline constexpr const char *const defaultRz = "default:rz";
inline constexpr const char *const defaultSpace = "default:space";
inline constexpr const char *const defaultTx = "default:tx";
inline constexpr const char *const defaultTy = "default:ty";
inline constexpr const char *const defaultTz = "default:tz";
inline constexpr const char *const guideCurveVertexCounts =
    "guide:curveVertexCounts";
inline constexpr const char *const guideDisplayColor = "guide:displayColor";
inline constexpr const char *const guideDisplayOpacity = "guide:displayOpacity";
inline constexpr const char *const guideDisplayOpacityInvert =
    "guide:displayOpacityInvert";
inline constexpr const char *const guideDisplayOpacityMin =
    "guide:displayOpacityMin";
inline constexpr const char *const guideDrawMode = "guide:drawMode";
inline constexpr const char *const guideOffset = "guide:offset";
inline constexpr const char *const guideOrient = "guide:orient";
inline constexpr const char *const guidePlaneNormal = "guide:planeNormal";
inline constexpr const char *const guidePoints = "guide:points";
inline constexpr const char *const guideRadius = "guide:radius";
inline constexpr const char *const guideScaleX = "guide:scaleX";
inline constexpr const char *const guideScaleY = "guide:scaleY";
inline constexpr const char *const guideScaleZ = "guide:scaleZ";
inline constexpr const char *const guideShape = "guide:shape";
inline constexpr const char *const guideSource = "guide:source";
inline constexpr const char *const guideWeightDisplay = "guide:weightDisplay";
inline constexpr const char *const guideWireWidth = "guide:wireWidth";
inline constexpr const char *const inputsActiveSpace = "inputs:activeSpace";
inline constexpr const char *const inputsAffect = "inputs:affect";
inline constexpr const char *const inputsAffectRotationX =
    "inputs:affectRotationX";
inline constexpr const char *const inputsAffectRotationY =
    "inputs:affectRotationY";
inline constexpr const char *const inputsAffectRotationZ =
    "inputs:affectRotationZ";
inline constexpr const char *const inputsAffectScaleX = "inputs:affectScaleX";
inline constexpr const char *const inputsAffectScaleY = "inputs:affectScaleY";
inline constexpr const char *const inputsAffectScaleZ = "inputs:affectScaleZ";
inline constexpr const char *const inputsAffectTranslationX =
    "inputs:affectTranslationX";
inline constexpr const char *const inputsAffectTranslationY =
    "inputs:affectTranslationY";
inline constexpr const char *const inputsAffectTranslationZ =
    "inputs:affectTranslationZ";
inline constexpr const char *const inputsAffectX = "inputs:affectX";
inline constexpr const char *const inputsAffectY = "inputs:affectY";
inline constexpr const char *const inputsAffectZ = "inputs:affectZ";
inline constexpr const char *const inputsAimVector = "inputs:aimVector";
inline constexpr const char *const inputsAmplitude = "inputs:amplitude";
inline constexpr const char *const inputsAttract = "inputs:attract";
inline constexpr const char *const inputsAxisAmplitude = "inputs:axisAmplitude";
inline constexpr const char *const inputsAxisMask = "inputs:axisMask";
inline constexpr const char *const inputsBendCompliance =
    "inputs:bendCompliance";
inline constexpr const char *const inputsBendStiffness = "inputs:bendStiffness";
inline constexpr const char *const inputsBias = "inputs:bias";
inline constexpr const char *const inputsBulge = "inputs:bulge";
inline constexpr const char *const inputsBulgeMax = "inputs:bulgeMax";
inline constexpr const char *const inputsBulgeMin = "inputs:bulgeMin";
inline constexpr const char *const inputsBulgeSmooth = "inputs:bulgeSmooth";
inline constexpr const char *const inputsCollisionDepth =
    "inputs:collisionDepth";
inline constexpr const char *const inputsCompliance = "inputs:compliance";
inline constexpr const char *const inputsCompressionStiffness =
    "inputs:compressionStiffness";
inline constexpr const char *const inputsComputationToTarget =
    "inputs:computationToTarget";
inline constexpr const char *const inputsConnected = "inputs:connected";
inline constexpr const char *const inputsCurrentPivot = "inputs:currentPivot";
inline constexpr const char *const inputsDamping = "inputs:damping";
inline constexpr const char *const inputsDecay = "inputs:decay";
inline constexpr const char *const inputsDefaultWeight = "inputs:defaultWeight";
inline constexpr const char *const inputsDisplacement = "inputs:displacement";
inline constexpr const char *const inputsDistanceWeight =
    "inputs:distanceWeight";
inline constexpr const char *const inputsDriver = "inputs:driver";
inline constexpr const char *const inputsDriverBaseWeights =
    "inputs:driverBaseWeights";
inline constexpr const char *const inputsDriverWeights = "inputs:driverWeights";
inline constexpr const char *const inputsDropoffDistance =
    "inputs:dropoffDistance";
inline constexpr const char *const inputsDualQuaternion =
    "inputs:dualQuaternion";
inline constexpr const char *const inputsEdges = "inputs:edges";
inline constexpr const char *const inputsEnabled = "inputs:enabled";
inline constexpr const char *const inputsExtentU = "inputs:extentU";
inline constexpr const char *const inputsExtentV = "inputs:extentV";
inline constexpr const char *const inputsFalloffMax = "inputs:falloffMax";
inline constexpr const char *const inputsFalloffMin = "inputs:falloffMin";
inline constexpr const char *const inputsFalloffSamples =
    "inputs:falloffSamples";
inline constexpr const char *const inputsFollowOnly = "inputs:followOnly";
inline constexpr const char *const inputsFrameTransport =
    "inputs:frameTransport";
inline constexpr const char *const inputsFriction = "inputs:friction";
inline constexpr const char *const inputsFromBind = "inputs:fromBind";
inline constexpr const char *const inputsGain = "inputs:gain";
inline constexpr const char *const inputsGravity = "inputs:gravity";
inline constexpr const char *const inputsHasParent = "inputs:hasParent";
inline constexpr const char *const inputsIncoming = "inputs:incoming";
inline constexpr const char *const inputsInfluence = "inputs:influence";
inline constexpr const char *const inputsInheritRotation =
    "inputs:inheritRotation";
inline constexpr const char *const inputsInheritScale = "inputs:inheritScale";
inline constexpr const char *const inputsInverseBind = "inputs:inverseBind";
inline constexpr const char *const inputsInverseMesh = "inputs:inverseMesh";
inline constexpr const char *const inputsInvert = "inputs:invert";
inline constexpr const char *const inputsInvertMask = "inputs:invertMask";
inline constexpr const char *const inputsIterations = "inputs:iterations";
inline constexpr const char *const inputsKeepAxis = "inputs:keepAxis";
inline constexpr const char *const inputsKeys = "inputs:keys";
inline constexpr const char *const inputsLocal = "inputs:local";
inline constexpr const char *const inputsLocalLocation = "inputs:localLocation";
inline constexpr const char *const inputsLowerScale = "inputs:lowerScale";
inline constexpr const char *const inputsMapAxes = "inputs:mapAxes";
inline constexpr const char *const inputsMapExtrapolate =
    "inputs:mapExtrapolate";
inline constexpr const char *const inputsMapFrom = "inputs:mapFrom";
inline constexpr const char *const inputsMapFromMax = "inputs:mapFromMax";
inline constexpr const char *const inputsMapFromMin = "inputs:mapFromMin";
inline constexpr const char *const inputsMapMix = "inputs:mapMix";
inline constexpr const char *const inputsMapToMax = "inputs:mapToMax";
inline constexpr const char *const inputsMapToMin = "inputs:mapToMin";
inline constexpr const char *const inputsMask = "inputs:mask";
inline constexpr const char *const inputsMax = "inputs:max";
inline constexpr const char *const inputsMaxDisplacement =
    "inputs:maxDisplacement";
inline constexpr const char *const inputsMaxStrength = "inputs:maxStrength";
inline constexpr const char *const inputsMidFollowWeight =
    "inputs:midFollowWeight";
inline constexpr const char *const inputsMin = "inputs:min";
inline constexpr const char *const inputsMinLengthRatio =
    "inputs:minLengthRatio";
inline constexpr const char *const inputsMinStrength = "inputs:minStrength";
inline constexpr const char *const inputsNeighborDistance =
    "inputs:neighborDistance";
inline constexpr const char *const inputsNormal = "inputs:normal";
inline constexpr const char *const inputsObjectIndices = "inputs:objectIndices";
inline constexpr const char *const inputsOffset = "inputs:offset";
inline constexpr const char *const inputsOnlySmooth = "inputs:onlySmooth";
inline constexpr const char *const inputsOperation = "inputs:operation";
inline constexpr const char *const inputsOrigin = "inputs:origin";
inline constexpr const char *const inputsOwnerHasParent =
    "inputs:ownerHasParent";
inline constexpr const char *const inputsOwnerInheritRotation =
    "inputs:ownerInheritRotation";
inline constexpr const char *const inputsOwnerInheritScale =
    "inputs:ownerInheritScale";
inline constexpr const char *const inputsOwnerLocal = "inputs:ownerLocal";
inline constexpr const char *const inputsOwnerLocalLocation =
    "inputs:ownerLocalLocation";
inline constexpr const char *const inputsOwnerParentRest =
    "inputs:ownerParentRest";
inline constexpr const char *const inputsOwnerRest = "inputs:ownerRest";
inline constexpr const char *const inputsOwnerSpace = "inputs:ownerSpace";
inline constexpr const char *const inputsParentRest = "inputs:parentRest";
inline constexpr const char *const inputsPin = "inputs:pin";
inline constexpr const char *const inputsPinBorders = "inputs:pinBorders";
inline constexpr const char *const inputsPinPoints = "inputs:pinPoints";
inline constexpr const char *const inputsPivot = "inputs:pivot";
inline constexpr const char *const inputsPoleVector = "inputs:poleVector";
inline constexpr const char *const inputsPoleVectorWeights =
    "inputs:poleVectorWeights";
inline constexpr const char *const inputsPower = "inputs:power";
inline constexpr const char *const inputsPreserveLocation =
    "inputs:preserveLocation";
inline constexpr const char *const inputsPreserveVolume =
    "inputs:preserveVolume";
inline constexpr const char *const inputsRemoveTargetShear =
    "inputs:removeTargetShear";
inline constexpr const char *const inputsRestLength = "inputs:restLength";
inline constexpr const char *const inputsRestLengthScale =
    "inputs:restLengthScale";
inline constexpr const char *const inputsRestPoints = "inputs:restPoints";
inline constexpr const char *const inputsRoll = "inputs:roll";
inline constexpr const char *const inputsRotationMix = "inputs:rotationMix";
inline constexpr const char *const inputsRotationOffset =
    "inputs:rotationOffset";
inline constexpr const char *const inputsRotationOffsets =
    "inputs:rotationOffsets";
inline constexpr const char *const inputsRx = "inputs:rx";
inline constexpr const char *const inputsRy = "inputs:ry";
inline constexpr const char *const inputsRz = "inputs:rz";
inline constexpr const char *const inputsScale = "inputs:scale";
inline constexpr const char *const inputsScaleAdd = "inputs:scaleAdd";
inline constexpr const char *const inputsScaleOffset = "inputs:scaleOffset";
inline constexpr const char *const inputsScaleX = "inputs:scaleX";
inline constexpr const char *const inputsScaleXNeg = "inputs:scaleXNeg";
inline constexpr const char *const inputsScaleXPos = "inputs:scaleXPos";
inline constexpr const char *const inputsScaleY = "inputs:scaleY";
inline constexpr const char *const inputsScaleYNeg = "inputs:scaleYNeg";
inline constexpr const char *const inputsScaleYPos = "inputs:scaleYPos";
inline constexpr const char *const inputsScaleZ = "inputs:scaleZ";
inline constexpr const char *const inputsScaleZNeg = "inputs:scaleZNeg";
inline constexpr const char *const inputsScaleZPos = "inputs:scaleZPos";
inline constexpr const char *const inputsSmoothWeights = "inputs:smoothWeights";
inline constexpr const char *const inputsSmoothing = "inputs:smoothing";
inline constexpr const char *const inputsSmoothingIterations =
    "inputs:smoothingIterations";
inline constexpr const char *const inputsSoftDistance = "inputs:softDistance";
inline constexpr const char *const inputsSoftness = "inputs:softness";
inline constexpr const char *const inputsSourceHasParent =
    "inputs:sourceHasParent";
inline constexpr const char *const inputsSourceInheritRotation =
    "inputs:sourceInheritRotation";
inline constexpr const char *const inputsSourceInheritScale =
    "inputs:sourceInheritScale";
inline constexpr const char *const inputsSourceLocal = "inputs:sourceLocal";
inline constexpr const char *const inputsSourceLocalLocation =
    "inputs:sourceLocalLocation";
inline constexpr const char *const inputsSourceParentRest =
    "inputs:sourceParentRest";
inline constexpr const char *const inputsSourceRest = "inputs:sourceRest";
inline constexpr const char *const inputsSourceWeights = "inputs:sourceWeights";
inline constexpr const char *const inputsSpaceKind = "inputs:spaceKind";
inline constexpr const char *const inputsStep = "inputs:step";
inline constexpr const char *const inputsStrength = "inputs:strength";
inline constexpr const char *const inputsStretch = "inputs:stretch";
inline constexpr const char *const inputsStretchCompliance =
    "inputs:stretchCompliance";
inline constexpr const char *const inputsStretchStiffness =
    "inputs:stretchStiffness";
inline constexpr const char *const inputsSwitch = "inputs:switch";
inline constexpr const char *const inputsSx = "inputs:sx";
inline constexpr const char *const inputsSy = "inputs:sy";
inline constexpr const char *const inputsSz = "inputs:sz";
inline constexpr const char *const inputsTangentPlaneCollisions =
    "inputs:tangentPlaneCollisions";
inline constexpr const char *const inputsTangentPlaneInset =
    "inputs:tangentPlaneInset";
inline constexpr const char *const inputsTangents = "inputs:tangents";
inline constexpr const char *const inputsTargetBinds = "inputs:targetBinds";
inline constexpr const char *const inputsTargetIndices = "inputs:targetIndices";
inline constexpr const char *const inputsTargetOffset = "inputs:targetOffset";
inline constexpr const char *const inputsTargetRest = "inputs:targetRest";
inline constexpr const char *const inputsTargetSpace = "inputs:targetSpace";
inline constexpr const char *const inputsTargetWeights = "inputs:targetWeights";
inline constexpr const char *const inputsTermination = "inputs:termination";
inline constexpr const char *const inputsThickness = "inputs:thickness";
inline constexpr const char *const inputsTopology = "inputs:topology";
inline constexpr const char *const inputsTrackAxis = "inputs:trackAxis";
inline constexpr const char *const inputsTransformInput =
    "inputs:transformInput";
inline constexpr const char *const inputsTranslationOffset =
    "inputs:translationOffset";
inline constexpr const char *const inputsTranslationOffsets =
    "inputs:translationOffsets";
inline constexpr const char *const inputsTwist = "inputs:twist";
inline constexpr const char *const inputsTwistDegrees = "inputs:twistDegrees";
inline constexpr const char *const inputsTwistTurns = "inputs:twistTurns";
inline constexpr const char *const inputsTx = "inputs:tx";
inline constexpr const char *const inputsTy = "inputs:ty";
inline constexpr const char *const inputsTz = "inputs:tz";
inline constexpr const char *const inputsUniformScale = "inputs:uniformScale";
inline constexpr const char *const inputsUpVector = "inputs:upVector";
inline constexpr const char *const inputsUpperScale = "inputs:upperScale";
inline constexpr const char *const inputsUseBaseInput = "inputs:useBaseInput";
inline constexpr const char *const inputsUseBulgeMax = "inputs:useBulgeMax";
inline constexpr const char *const inputsUseBulgeMin = "inputs:useBulgeMin";
inline constexpr const char *const inputsUseIncoming = "inputs:useIncoming";
inline constexpr const char *const inputsUvWeights = "inputs:uvWeights";
inline constexpr const char *const inputsValue = "inputs:value";
inline constexpr const char *const inputsValues = "inputs:values";
inline constexpr const char *const inputsVolume = "inputs:volume";
inline constexpr const char *const inputsWarp = "inputs:warp";
inline constexpr const char *const inputsWeft = "inputs:weft";
inline constexpr const char *const inputsWeight = "inputs:weight";
inline constexpr const char *const inputsWeights = "inputs:weights";
inline constexpr const char *const inputsWorldUpVector = "inputs:worldUpVector";
inline constexpr const char *const inputsWrinkleScale = "inputs:wrinkleScale";
inline constexpr const char *const outputsExpression = "outputs:expression";
inline constexpr const char *const outputsFkValues = "outputs:fkValues";
inline constexpr const char *const outputsIkValues = "outputs:ikValues";
inline constexpr const char *const outputsMatrix = "outputs:matrix";
inline constexpr const char *const outputsValue = "outputs:value";
inline constexpr const char *const outputsWeight = "outputs:weight";
inline constexpr const char *const parentDefaultSpace = "parent:defaultSpace";
inline constexpr const char *const parentSpace = "parent:space";
inline constexpr const char *const posedDefaultSpace = "posed:defaultSpace";
inline constexpr const char *const posedSpace = "posed:space";
inline constexpr const char *const restRx = "rest:rx";
inline constexpr const char *const restRy = "rest:ry";
inline constexpr const char *const restRz = "rest:rz";
inline constexpr const char *const restSpace = "rest:space";
inline constexpr const char *const spacesMatchValues = "spaces:matchValues";
inline constexpr const char *const restTx = "rest:tx";
inline constexpr const char *const restTy = "rest:ty";
inline constexpr const char *const restTz = "rest:tz";
inline constexpr const char *const rigExecActivation = "rigExec:activation";
inline constexpr const char *const rigExecActiveMember = "rigExec:activeMember";
inline constexpr const char *const rigExecActiveSpaceAttribute =
    "rigExec:activeSpaceAttribute";
inline constexpr const char *const rigExecAimAxis = "rigExec:aimAxis";
inline constexpr const char *const rigExecAimTarget = "rigExec:aimTarget";
inline constexpr const char *const rigExecAllowNegativeWeights =
    "rigExec:allowNegativeWeights";
inline constexpr const char *const rigExecAmountAttribute =
    "rigExec:amountAttribute";
inline constexpr const char *const rigExecAnchor = "rigExec:anchor";
inline constexpr const char *const rigExecAsset = "rigExec:asset";
inline constexpr const char *const rigExecAutoSmooth = "rigExec:autoSmooth";
inline constexpr const char *const rigExecAutomaticRadius =
    "rigExec:automaticRadius";
inline constexpr const char *const rigExecBarycentricWeights =
    "rigExec:barycentricWeights";
inline constexpr const char *const rigExecBaseWeight = "rigExec:baseWeight";
inline constexpr const char *const rigExecBasis = "rigExec:basis";
inline constexpr const char *const rigExecBiases = "rigExec:biases";
inline constexpr const char *const rigExecBindCoordinates =
    "rigExec:bindCoordinates";
inline constexpr const char *const rigExecBinding = "rigExec:binding";
inline constexpr const char *const rigExecBindingWeights =
    "rigExec:bindingWeights";
inline constexpr const char *const rigExecBlendInputs = "rigExec:blendInputs";
inline constexpr const char *const rigExecBlendShape = "rigExec:blendShape";
inline constexpr const char *const rigExecBlendShear = "rigExec:blendShear";
inline constexpr const char *const rigExecCage = "rigExec:cage";
inline constexpr const char *const rigExecCageMatrix = "rigExec:cageMatrix";
inline constexpr const char *const rigExecCageReadPhase =
    "rigExec:cageReadPhase";
inline constexpr const char *const rigExecChannelRole = "rigExec:channelRole";
inline constexpr const char *const rigExecChannelSpaces =
    "rigExec:channelSpaces";
inline constexpr const char *const rigExecChannels = "rigExec:channels";
inline constexpr const char *const rigExecClampOutput = "rigExec:clampOutput";
inline constexpr const char *const rigExecColliders = "rigExec:colliders";
inline constexpr const char *const rigExecCombineMode = "rigExec:combineMode";
inline constexpr const char *const rigExecComputeDriverPoints =
    "rigExec:computeDriverPoints";
inline constexpr const char *const rigExecComputeRestDriverPoints =
    "rigExec:computeRestDriverPoints";
inline constexpr const char *const rigExecConstraints = "rigExec:constraints";
inline constexpr const char *const rigExecControlSpace = "rigExec:controlSpace";
inline constexpr const char *const rigExecControls = "rigExec:controls";
inline constexpr const char *const rigExecCoordinates = "rigExec:coordinates";
inline constexpr const char *const rigExecCount = "rigExec:count";
inline constexpr const char *const rigExecCurve = "rigExec:curve";
inline constexpr const char *const rigExecCustomSpace = "rigExec:customSpace";
inline constexpr const char *const rigExecDefaultWeight =
    "rigExec:defaultWeight";
inline constexpr const char *const rigExecDeltaMultiplier =
    "rigExec:deltaMultiplier";
inline constexpr const char *const rigExecDeltaMultipliers =
    "rigExec:deltaMultipliers";
inline constexpr const char *const rigExecDeltaSpace = "rigExec:deltaSpace";
inline constexpr const char *const rigExecDistanceMethod =
    "rigExec:distanceMethod";
inline constexpr const char *const rigExecDistribution = "rigExec:distribution";
inline constexpr const char *const rigExecDivisions = "rigExec:divisions";
inline constexpr const char *const rigExecDriver = "rigExec:driver";
inline constexpr const char *const rigExecDriverAttributes =
    "rigExec:driverAttributes";
inline constexpr const char *const rigExecDriverBaseTransformSpaces =
    "rigExec:driverBaseTransformSpaces";
inline constexpr const char *const rigExecDriverBaseTransforms =
    "rigExec:driverBaseTransforms";
inline constexpr const char *const rigExecDriverChannels =
    "rigExec:driverChannels";
inline constexpr const char *const rigExecDriverCurve = "rigExec:driverCurve";
inline constexpr const char *const rigExecDriverCurveReadPhase =
    "rigExec:driverCurveReadPhase";
inline constexpr const char *const rigExecDriverDeltaFrame =
    "rigExec:driverDeltaFrame";
inline constexpr const char *const rigExecDriverFrames = "rigExec:driverFrames";
inline constexpr const char *const rigExecDriverTransformSpaces =
    "rigExec:driverTransformSpaces";
inline constexpr const char *const rigExecDriverTransforms =
    "rigExec:driverTransforms";
inline constexpr const char *const rigExecEffector = "rigExec:effector";
inline constexpr const char *const rigExecEffectorControl =
    "rigExec:effectorControl";
inline constexpr const char *const rigExecElementSize = "rigExec:elementSize";
inline constexpr const char *const rigExecEnableRotation =
    "rigExec:enableRotation";
inline constexpr const char *const rigExecEnableTranslation =
    "rigExec:enableTranslation";
inline constexpr const char *const rigExecEnabled = "rigExec:enabled";
inline constexpr const char *const rigExecEnd = "rigExec:end";
inline constexpr const char *const rigExecEndControl = "rigExec:endControl";
inline constexpr const char *const rigExecEndFrame = "rigExec:endFrame";
inline constexpr const char *const rigExecEndJoint = "rigExec:endJoint";
inline constexpr const char *const rigExecEvaluation = "rigExec:evaluation";
inline constexpr const char *const rigExecEvaluationMode =
    "rigExec:evaluationMode";
inline constexpr const char *const rigExecFalloff = "rigExec:falloff";
inline constexpr const char *const rigExecFalloffCurve = "rigExec:falloffCurve";
inline constexpr const char *const rigExecFalloffProfile =
    "rigExec:falloffProfile";
inline constexpr const char *const rigExecFirstJoint = "rigExec:firstJoint";
inline constexpr const char *const rigExecFkControls = "rigExec:fkControls";
inline constexpr const char *const rigExecForceCurve = "rigExec:forceCurve";
inline constexpr const char *const rigExecFrame = "rigExec:frame";
inline constexpr const char *const rigExecFrameTransport =
    "rigExec:frameTransport";
inline constexpr const char *const rigExecFrameRate = "rigExec:frameRate";
inline constexpr const char *const rigExecFrames = "rigExec:frames";
inline constexpr const char *const rigExecFrequency = "rigExec:frequency";
inline constexpr const char *const rigExecFrictionMap = "rigExec:frictionMap";
inline constexpr const char *const rigExecIgnoreFirstFrame =
    "rigExec:ignoreFirstFrame";
inline constexpr const char *const rigExecIkBlendAttribute =
    "rigExec:ikBlendAttribute";
inline constexpr const char *const rigExecIkControl = "rigExec:ikControl";
inline constexpr const char *const rigExecIkTarget = "rigExec:ikTarget";
inline constexpr const char *const rigExecIkValue = "rigExec:ikValue";
inline constexpr const char *const rigExecInbetweenWeights =
    "rigExec:inbetweenWeights";
inline constexpr const char *const rigExecIndices = "rigExec:indices";
inline constexpr const char *const rigExecInfluences = "rigExec:influences";
inline constexpr const char *const rigExecInputA = "rigExec:inputA";
inline constexpr const char *const rigExecInputB = "rigExec:inputB";
inline constexpr const char *const rigExecInputMean = "rigExec:inputMean";
inline constexpr const char *const rigExecInputScale = "rigExec:inputScale";
inline constexpr const char *const rigExecInputWeights = "rigExec:inputWeights";
inline constexpr const char *const rigExecInterpolationU =
    "rigExec:interpolationU";
inline constexpr const char *const rigExecInterpolationV =
    "rigExec:interpolationV";
inline constexpr const char *const rigExecInterpolationW =
    "rigExec:interpolationW";
inline constexpr const char *const rigExecIterations = "rigExec:iterations";
inline constexpr const char *const rigExecJointElements =
    "rigExec:jointElements";
inline constexpr const char *const rigExecJointGroups = "rigExec:jointGroups";
inline constexpr const char *const rigExecJointIndices = "rigExec:jointIndices";
inline constexpr const char *const rigExecJointPaths = "rigExec:jointPaths";
inline constexpr const char *const rigExecJointTransforms =
    "rigExec:jointTransforms";
inline constexpr const char *const rigExecJointWeights = "rigExec:jointWeights";
inline constexpr const char *const rigExecJoints = "rigExec:joints";
inline constexpr const char *const rigExecKernel = "rigExec:kernel";
inline constexpr const char *const rigExecKind = "rigExec:kind";
inline constexpr const char *const rigExecLayerActivations =
    "rigExec:layerActivations";
inline constexpr const char *const rigExecLayerAlphas = "rigExec:layerAlphas";
inline constexpr const char *const rigExecLayerSizes = "rigExec:layerSizes";
inline constexpr const char *const rigExecLimbJoints = "rigExec:limbJoints";
inline constexpr const char *const rigExecLimitDuWeights =
    "rigExec:limitDuWeights";
inline constexpr const char *const rigExecLimitDvWeights =
    "rigExec:limitDvWeights";
inline constexpr const char *const rigExecLimitIndices = "rigExec:limitIndices";
inline constexpr const char *const rigExecLimitOffsets = "rigExec:limitOffsets";
inline constexpr const char *const rigExecLimitWeights = "rigExec:limitWeights";
inline constexpr const char *const rigExecLocked = "rigExec:locked";
inline constexpr const char *const rigExecLowerLength = "rigExec:lowerLength";
inline constexpr const char *const rigExecLowerLengthOffset =
    "rigExec:lowerLengthOffset";
inline constexpr const char *const rigExecMask = "rigExec:mask";
inline constexpr const char *const rigExecMaskMap = "rigExec:maskMap";
inline constexpr const char *const rigExecMatchVelocity =
    "rigExec:matchVelocity";
inline constexpr const char *const rigExecMaxStrengthMap =
    "rigExec:maxStrengthMap";
inline constexpr const char *const rigExecMaxWindowFrames =
    "rigExec:maxWindowFrames";
inline constexpr const char *const rigExecMeshes = "rigExec:meshes";
inline constexpr const char *const rigExecMethod = "rigExec:method";
inline constexpr const char *const rigExecMidControl = "rigExec:midControl";
inline constexpr const char *const rigExecMinStrengthMap =
    "rigExec:minStrengthMap";
inline constexpr const char *const rigExecMode = "rigExec:mode";
inline constexpr const char *const rigExecMover = "rigExec:mover";
inline constexpr const char *const rigExecMoves = "rigExec:moves";
inline constexpr const char *const rigExecNormalMap = "rigExec:normalMap";
inline constexpr const char *const rigExecNormalMode = "rigExec:normalMode";
inline constexpr const char *const rigExecNormalize = "rigExec:normalize";
inline constexpr const char *const rigExecNormalizeMethod =
    "rigExec:normalizeMethod";
inline constexpr const char *const rigExecOffset = "rigExec:offset";
inline constexpr const char *const rigExecOffsets = "rigExec:offsets";
inline constexpr const char *const rigExecOperation = "rigExec:operation";
inline constexpr const char *const rigExecOrientationMode =
    "rigExec:orientationMode";
inline constexpr const char *const rigExecOrigin = "rigExec:origin";
inline constexpr const char *const rigExecOutputDefaults =
    "rigExec:outputDefaults";
inline constexpr const char *const rigExecOwner = "rigExec:owner";
inline constexpr const char *const rigExecOwnerObject = "rigExec:ownerObject";
inline constexpr const char *const rigExecOwnerParent = "rigExec:ownerParent";
inline constexpr const char *const rigExecParameterization =
    "rigExec:parameterization";
inline constexpr const char *const rigExecParent = "rigExec:parent";
inline constexpr const char *const rigExecPartition = "rigExec:partition";
inline constexpr const char *const rigExecPatch = "rigExec:patch";
inline constexpr const char *const rigExecPath = "rigExec:path";
inline constexpr const char *const rigExecPathComponent =
    "rigExec:pathComponent";
inline constexpr const char *const rigExecPathHeadingOffset =
    "rigExec:pathHeadingOffset";
inline constexpr const char *const rigExecPathMeasure = "rigExec:pathMeasure";
inline constexpr const char *const rigExecPathOffset = "rigExec:pathOffset";
inline constexpr const char *const rigExecPinWeights = "rigExec:pinWeights";
inline constexpr const char *const rigExecPivot = "rigExec:pivot";
inline constexpr const char *const rigExecPlaneAxis = "rigExec:planeAxis";
inline constexpr const char *const rigExecPlaneBounds = "rigExec:planeBounds";
inline constexpr const char *const rigExecPointFrame = "rigExec:pointFrame";
inline constexpr const char *const rigExecPointIndices = "rigExec:pointIndices";
inline constexpr const char *const rigExecPointSpace = "rigExec:pointSpace";
inline constexpr const char *const rigExecPointsReadPhase =
    "rigExec:pointsReadPhase";
inline constexpr const char *const rigExecPoleControl = "rigExec:poleControl";
inline constexpr const char *const rigExecPoleVectorMode =
    "rigExec:poleVectorMode";
inline constexpr const char *const rigExecPoleVectorObjects =
    "rigExec:poleVectorObjects";
inline constexpr const char *const rigExecPolygonOffsets =
    "rigExec:polygonOffsets";
inline constexpr const char *const rigExecPoseControlValues =
    "rigExec:poseControlValues";
inline constexpr const char *const rigExecPoseControls = "rigExec:poseControls";
inline constexpr const char *const rigExecPoseFalloffs = "rigExec:poseFalloffs";
inline constexpr const char *const rigExecPoseGains = "rigExec:poseGains";
inline constexpr const char *const rigExecPoseInputs = "rigExec:poseInputs";
inline constexpr const char *const rigExecPoseRotations =
    "rigExec:poseRotations";
inline constexpr const char *const rigExecPoseType = "rigExec:poseType";
inline constexpr const char *const rigExecPreferredBendRadians =
    "rigExec:preferredBendRadians";
inline constexpr const char *const rigExecPreserve = "rigExec:preserve";
inline constexpr const char *const rigExecPrimvarName = "rigExec:primvarName";
inline constexpr const char *const rigExecProjectionMode =
    "rigExec:projectionMode";
inline constexpr const char *const rigExecRadius = "rigExec:radius";
inline constexpr const char *const rigExecRangePolicy = "rigExec:rangePolicy";
inline constexpr const char *const rigExecRayDirection = "rigExec:rayDirection";
inline constexpr const char *const rigExecRayOrigin = "rigExec:rayOrigin";
inline constexpr const char *const rigExecRayUp = "rigExec:rayUp";
inline constexpr const char *const rigExecReferenceTransform =
    "rigExec:referenceTransform";
inline constexpr const char *const rigExecReferenceTransformSpace =
    "rigExec:referenceTransformSpace";
inline constexpr const char *const rigExecRegularization =
    "rigExec:regularization";
inline constexpr const char *const rigExecRepresentation =
    "rigExec:representation";
inline constexpr const char *const rigExecResolvedBase = "rigExec:resolvedBase";
inline constexpr const char *const rigExecResolvedBindCoords =
    "rigExec:resolvedBindCoords";
inline constexpr const char *const rigExecResolvedBlendInputs =
    "rigExec:resolvedBlendInputs";
inline constexpr const char *const rigExecResolvedCagePoints =
    "rigExec:resolvedCagePoints";
inline constexpr const char *const rigExecResolvedDriverFrames =
    "rigExec:resolvedDriverFrames";
inline constexpr const char *const rigExecResolvedFinalPoints =
    "rigExec:resolvedFinalPoints";
inline constexpr const char *const rigExecResolvedSurfacePoints =
    "rigExec:resolvedSurfacePoints";
inline constexpr const char *const rigExecResolvedTopologyCounts =
    "rigExec:resolvedTopologyCounts";
inline constexpr const char *const rigExecResolvedTopologyIndices =
    "rigExec:resolvedTopologyIndices";
inline constexpr const char *const rigExecResolvedTransform =
    "rigExec:resolvedTransform";
inline constexpr const char *const rigExecResolvedWidths =
    "rigExec:resolvedWidths";
inline constexpr const char *const rigExecRestCagePoints =
    "rigExec:restCagePoints";
inline constexpr const char *const rigExecRestLength = "rigExec:restLength";
inline constexpr const char *const rigExecRestPoints = "rigExec:restPoints";
inline constexpr const char *const rigExecRestShape = "rigExec:restShape";
inline constexpr const char *const rigExecRootControl = "rigExec:rootControl";
inline constexpr const char *const rigExecRootTangent = "rigExec:rootTangent";
inline constexpr const char *const rigExecRotation = "rigExec:rotation";
inline constexpr const char *const rigExecRotationBlend =
    "rigExec:rotationBlend";
inline constexpr const char *const rigExecRotationFilters =
    "rigExec:rotationFilters";
inline constexpr const char *const rigExecRotationOrder =
    "rigExec:rotationOrder";
inline constexpr const char *const rigExecRotationRadius =
    "rigExec:rotationRadius";
inline constexpr const char *const rigExecRotations = "rigExec:rotations";
inline constexpr const char *const rigExecRowCut = "rigExec:rowCut";
inline constexpr const char *const rigExecRowFrom = "rigExec:rowFrom";
inline constexpr const char *const rigExecRowInputs = "rigExec:rowInputs";
inline constexpr const char *const rigExecRowSlope = "rigExec:rowSlope";
inline constexpr const char *const rigExecRowTo = "rigExec:rowTo";
inline constexpr const char *const rigExecSampleCount = "rigExec:sampleCount";
inline constexpr const char *const rigExecSamplePhase = "rigExec:samplePhase";
inline constexpr const char *const rigExecSampleSource = "rigExec:sampleSource";
inline constexpr const char *const rigExecSamples = "rigExec:samples";
inline constexpr const char *const rigExecSamplesPerSpline =
    "rigExec:samplesPerSpline";
inline constexpr const char *const rigExecScale = "rigExec:scale";
inline constexpr const char *const rigExecScaleBlend = "rigExec:scaleBlend";
inline constexpr const char *const rigExecScaleCalibration =
    "rigExec:scaleCalibration";
inline constexpr const char *const rigExecSegmentScale = "rigExec:segmentScale";
inline constexpr const char *const rigExecShaderDialPrimvar =
    "rigExec:shaderDialPrimvar";
inline constexpr const char *const rigExecShaderDialSources =
    "rigExec:shaderDialSources";
inline constexpr const char *const rigExecShaderOffset = "rigExec:shaderOffset";
inline constexpr const char *const rigExecShaderPrimvar =
    "rigExec:shaderPrimvar";
inline constexpr const char *const rigExecSkinMethod = "rigExec:skinMethod";
inline constexpr const char *const rigExecSkinningMethod =
    "rigExec:skinningMethod";
inline constexpr const char *const rigExecSnapMode = "rigExec:snapMode";
inline constexpr const char *const rigExecSolver = "rigExec:solver";
inline constexpr const char *const rigExecSolverMode = "rigExec:solverMode";
inline constexpr const char *const rigExecSource = "rigExec:source";
inline constexpr const char *const rigExecSourceBinding =
    "rigExec:sourceBinding";
inline constexpr const char *const rigExecSourceMesh = "rigExec:sourceMesh";
inline constexpr const char *const rigExecSourceObject = "rigExec:sourceObject";
inline constexpr const char *const rigExecSourceParent = "rigExec:sourceParent";
inline constexpr const char *const rigExecSourceSpace = "rigExec:sourceSpace";
inline constexpr const char *const rigExecSources = "rigExec:sources";
inline constexpr const char *const rigExecSpace = "rigExec:space";
inline constexpr const char *const rigExecSpaceLabels = "rigExec:spaceLabels";
inline constexpr const char *const rigExecSpaceMatrix = "rigExec:spaceMatrix";
inline constexpr const char *const rigExecSpacing = "rigExec:spacing";
inline constexpr const char *const rigExecStart = "rigExec:start";
inline constexpr const char *const rigExecStartFrame = "rigExec:startFrame";
inline constexpr const char *const rigExecStartFramePolicy =
    "rigExec:startFramePolicy";
inline constexpr const char *const rigExecStrength = "rigExec:strength";
inline constexpr const char *const rigExecStretchPolicy =
    "rigExec:stretchPolicy";
inline constexpr const char *const rigExecSubsteps = "rigExec:substeps";
inline constexpr const char *const rigExecSurface = "rigExec:surface";
inline constexpr const char *const rigExecSurfaceMatrix =
    "rigExec:surfaceMatrix";
inline constexpr const char *const rigExecSurfaceMode = "rigExec:surfaceMode";
inline constexpr const char *const rigExecSurfaceReadPhase =
    "rigExec:surfaceReadPhase";
inline constexpr const char *const rigExecSurfaceRestMatrix =
    "rigExec:surfaceRestMatrix";
inline constexpr const char *const rigExecSurfaceToBinding =
    "rigExec:surfaceToBinding";
inline constexpr const char *const rigExecTarget = "rigExec:target";
inline constexpr const char *const rigExecTargetBinding =
    "rigExec:targetBinding";
inline constexpr const char *const rigExecTargetMatrix = "rigExec:targetMatrix";
inline constexpr const char *const rigExecTargetMesh = "rigExec:targetMesh";
inline constexpr const char *const rigExecTargetObjects =
    "rigExec:targetObjects";
inline constexpr const char *const rigExecTargetPoints = "rigExec:targetPoints";
inline constexpr const char *const rigExecTargetRestMatrix =
    "rigExec:targetRestMatrix";
inline constexpr const char *const rigExecTargets = "rigExec:targets";
inline constexpr const char *const rigExecTetherSlack = "rigExec:tetherSlack";
inline constexpr const char *const rigExecTethers = "rigExec:tethers";
inline constexpr const char *const rigExecThicknessMap = "rigExec:thicknessMap";
inline constexpr const char *const rigExecTransform = "rigExec:transform";
inline constexpr const char *const rigExecTransformReadPhase =
    "rigExec:transformReadPhase";
inline constexpr const char *const rigExecTransformSpace =
    "rigExec:transformSpace";
inline constexpr const char *const rigExecTranslationEnabled =
    "rigExec:translationEnabled";
inline constexpr const char *const rigExecTranslation = "rigExec:translation";
inline constexpr const char *const rigExecTranslationRadius =
    "rigExec:translationRadius";
inline constexpr const char *const rigExecTriangles = "rigExec:triangles";
inline constexpr const char *const rigExecTwistAxis = "rigExec:twistAxis";
inline constexpr const char *const rigExecTwistFrames = "rigExec:twistFrames";
inline constexpr const char *const rigExecTwoSided = "rigExec:twoSided";
inline constexpr const char *const rigExecUnreachablePolicy =
    "rigExec:unreachablePolicy";
inline constexpr const char *const rigExecUnreachedValue =
    "rigExec:unreachedValue";
inline constexpr const char *const rigExecUpPolicy = "rigExec:upPolicy";
inline constexpr const char *const rigExecUpperLength = "rigExec:upperLength";
inline constexpr const char *const rigExecUpperLengthOffset =
    "rigExec:upperLengthOffset";
inline constexpr const char *const rigExecUvContactLower =
    "rigExec:uvContactLower";
inline constexpr const char *const rigExecUvContactUpper =
    "rigExec:uvContactUpper";
inline constexpr const char *const rigExecUvPoints = "rigExec:uvPoints";
inline constexpr const char *const rigExecUvSet = "rigExec:uvSet";
inline constexpr const char *const rigExecUvShapes = "rigExec:uvShapes";
inline constexpr const char *const rigExecValues = "rigExec:values";
inline constexpr const char *const rigExecVertexOffsets =
    "rigExec:vertexOffsets";
inline constexpr const char *const rigExecVertices = "rigExec:vertices";
inline constexpr const char *const rigExecVolumeWeights =
    "rigExec:volumeWeights";
inline constexpr const char *const rigExecWarpMap = "rigExec:warpMap";
inline constexpr const char *const rigExecWaveLength = "rigExec:waveLength";
inline constexpr const char *const rigExecWaves = "rigExec:waves";
inline constexpr const char *const rigExecWeftMap = "rigExec:weftMap";
inline constexpr const char *const rigExecWeightBlend = "rigExec:weightBlend";
inline constexpr const char *const rigExecWeightObject = "rigExec:weightObject";
inline constexpr const char *const rigExecWeightTarget = "rigExec:weightTarget";
inline constexpr const char *const rigExecWeightThreshold =
    "rigExec:weightThreshold";
inline constexpr const char *const rigExecWeights = "rigExec:weights";
inline constexpr const char *const rigExecWindowFrames = "rigExec:windowFrames";
inline constexpr const char *const rigExecWorldUpObject =
    "rigExec:worldUpObject";
inline constexpr const char *const rigExecWorldUpRotationOnly =
    "rigExec:worldUpRotationOnly";
inline constexpr const char *const rigExecWorldUpType = "rigExec:worldUpType";
inline constexpr const char *const uiAlternate = "ui:alternate";
inline constexpr const char *const uiBackground = "ui:background";
inline constexpr const char *const uiBackgroundImage = "ui:backgroundImage";
inline constexpr const char *const uiBezier = "ui:bezier";
inline constexpr const char *const uiBold = "ui:bold";
inline constexpr const char *const uiDepth = "ui:depth";
inline constexpr const char *const uiDirection = "ui:direction";
inline constexpr const char *const uiFill = "ui:fill";
inline constexpr const char *const uiFontSize = "ui:fontSize";
inline constexpr const char *const uiLabel = "ui:label";
inline constexpr const char *const uiOrder = "ui:order";
inline constexpr const char *const uiPolygon = "ui:polygon";
inline constexpr const char *const uiPosition = "ui:position";
inline constexpr const char *const uiRotation = "ui:rotation";
inline constexpr const char *const uiRoundness = "ui:roundness";
inline constexpr const char *const uiShape = "ui:shape";
inline constexpr const char *const uiSize = "ui:size";
inline constexpr const char *const uiSlopeLeft = "ui:slopeLeft";
inline constexpr const char *const uiSlopeRight = "ui:slopeRight";
inline constexpr const char *const uiStroke = "ui:stroke";
inline constexpr const char *const uiStrokeWidth = "ui:strokeWidth";
inline constexpr const char *const uiText = "ui:text";
inline constexpr const char *const uiTextAlign = "ui:textAlign";
inline constexpr const char *const uiTextColor = "ui:textColor";
inline constexpr const char *const uiValueColor = "ui:valueColor";
} // namespace SchemaAttrText

// Authored value tokens.
namespace SchemaValueText {
inline constexpr const char *const add = "add";
inline constexpr const char *const average = "average";
inline constexpr const char *const bounded = "bounded";
inline constexpr const char *const brackets = "brackets";
inline constexpr const char *const cage = "cage";
inline constexpr const char *const circle = "circle";
inline constexpr const char *const clamp = "clamp";
inline constexpr const char *const constant = "constant";
inline constexpr const char *const custom = "custom";
inline constexpr const char *const dense = "dense";
inline constexpr const char *const falloff = "falloff";
inline constexpr const char *const geometry = "geometry";
inline constexpr const char *const max = "max";
inline constexpr const char *const min = "min";
inline constexpr const char *const multiply = "multiply";
inline constexpr const char *const overlay = "overlay";
inline constexpr const char *const shell = "shell";
inline constexpr const char *const sparse = "sparse";
inline constexpr const char *const strict = "strict";
inline constexpr const char *const subtract = "subtract";
inline constexpr const char *const aim = "aim";
inline constexpr const char *const chain = "chain";
inline constexpr const char *const curve = "curve";
inline constexpr const char *const linear = "linear";
inline constexpr const char *const log = "log";
inline constexpr const char *const none = "none";
inline constexpr const char *const parentRelative = "parentRelative";
inline constexpr const char *const rigid = "rigid";
inline constexpr const char *const shortestArc = "shortestArc";
inline constexpr const char *const softDistance = "softDistance";
inline constexpr const char *const toChild = "toChild";
inline constexpr const char *const uniformSegments = "uniformSegments";
inline constexpr const char *const unbounded = "unbounded";
inline constexpr const char *const wire = "wire";
inline constexpr const char *const world = "world";
inline constexpr const char *const xyz = "XYZ";
inline constexpr const char *const x = "x";
inline constexpr const char *const y = "y";
inline constexpr const char *const z = "z";
} // namespace SchemaValueText

} // namespace rigExec

#endif
