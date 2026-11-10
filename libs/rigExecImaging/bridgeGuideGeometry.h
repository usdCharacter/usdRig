// Guide tokens, static geometry tables and placement helpers shared by the
// imaging bridge's guide domains. Every token and unit table is interned
// or built exactly once, at load, in bridgeGuideGeometry.cpp; the tables
// and their initialization dependencies stay in that one translation
// unit. Nothing declared here builds a token or spells a path per call.
#ifndef RIGEXEC_IMAGING_BRIDGE_GUIDE_GEOMETRY_H
#define RIGEXEC_IMAGING_BRIDGE_GUIDE_GEOMETRY_H

#include "rigExecMath/pointFrame.h"
#include "snapshotStore.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <vector>

namespace rigExec {

struct RigExecRigPose;

// Guide names are interned before frame publication.
extern const TfToken _tControlType;
extern const TfToken _tJointType;
extern const TfToken _tSphereWeightType;
extern const TfToken _tPlaneWeightType;
extern const TfToken _tCurveWeightType;
extern const TfToken _tY;
extern const TfToken _tRigExecGenerated;
extern const TfToken _tCircle;
extern const TfToken _tConstraintsRel;
extern const TfToken _tGuideDisplayColor;
extern const TfToken _tGuideDisplayOpacity;
extern const TfToken _tGuideDisplayOpacityInvert;
extern const TfToken _tGuideDisplayOpacityMin;
extern const TfToken _tGuideDrawMode;
extern const TfToken _tGuideOffset;
extern const TfToken _tGuidePlaneNormal;
extern const TfToken _tGuideRadius;
extern const TfToken _tGuideScaleX;
extern const TfToken _tGuideScaleY;
extern const TfToken _tGuideScaleZ;
extern const TfToken _tGuideShape;
extern const TfToken _tGuideWireWidth;
extern const TfToken _tPoints;
extern const TfToken _tRigExecPlaneAxis;
extern const TfToken _tRigExecPlaneBounds;
extern const TfToken _tWire;
extern const TfToken _tGeometry;
extern const TfToken _tCage;
extern const TfToken _tBrackets;
extern const TfToken _tShell;
extern const TfToken _tFalloff;
extern const TfToken _tWeightDisplay;
extern const TfToken _tHeatmap;
extern const TfToken _tBands;

extern const TfToken _volumeInputsExtentU;
extern const TfToken _volumeInputsExtentV;
extern const TfToken _volumeInputsFalloffMax;
extern const TfToken _volumeInputsFalloffMin;
extern const TfToken _volumeInputsScaleX;
extern const TfToken _volumeInputsScaleXNeg;
extern const TfToken _volumeInputsScaleXPos;
extern const TfToken _volumeInputsScaleY;
extern const TfToken _volumeInputsScaleYNeg;
extern const TfToken _volumeInputsScaleYPos;
extern const TfToken _volumeInputsScaleZ;
extern const TfToken _volumeInputsScaleZNeg;
extern const TfToken _volumeInputsScaleZPos;

// The schema fallback for guide:displayOpacityMin, restated for the same
// reason _FillControlGuides restates its own: a stage composed without the
// codeless schema plugin has no fallback to resolve.
constexpr double _kDefaultGuideOpacityMin = 0.15;

// Opacity of the max-weight interior fill. Half translucent, independent
// of guide:displayOpacity, which the dotted rings still use.
constexpr float _kSphereInteriorOpacity = 0.5f;

// The rigid ASSET-space placement matrix of one posed frame, or nothing
// when the frame cannot supply one. \p axisMagnitudes receives the removed
// basis magnitudes when requested.
bool _RigidGuideMatrix(const RigExecPointFrame &frame, GfMatrix4d *result,
                       GfVec3d *axisMagnitudes = nullptr);

// One guide element from a posed frame: rig-space placement matrix, cone
// length, primitive radius, and whether it owns an origin sphere.
bool _AppendGuideFrame(const RigExecPointFrame &frame, double length,
                       double authoredRadius, bool useAimFallback,
                       bool drawSphere, RigExecPublishedPrim *published);

// Builds the rigid frame and length of the link from parent to child.
bool _JointLinkFrame(const RigExecPointFrame &parent,
                     const RigExecPointFrame &child, GfMatrix4d *frame,
                     double *length);

// The prim's resolved render purpose, which is what BBoxCache classifies
// its extent by.
TfToken _ReadGuidePurpose(const UsdPrim &prim);

// The value the FIRST connection source of \p attr holds at the pose's
// time, or nothing when the attribute has no connection or the source
// cannot supply one. A property-mover result for the source wins over its
// authored value; a value the rig published on the attribute itself wins
// over both.
bool _ReadConnectedValue(const UsdAttribute &attr, const RigExecRigPose &pose,
                         VtValue *held);

// The scalar a float OR double VtValue holds, or nothing when it holds
// neither or the value is not finite.
bool _HeldScalar(const VtValue &held, double *value);

// The prim's authored guide styling (purpose, color, opacity) into
// \p published at the pose's time, resolving connected values.
void _ReadGuideStyle(const UsdPrim &prim, const RigExecRigPose &pose,
                     RigExecPublishedPrim *published);

// The volume's rigid ASSET-space placement, or nothing when it cannot
// supply one.
bool _RigidVolumeMatrix(const GfMatrix4d &placement, GfMatrix4d *result);

// The authored points of a curve weight's rigExec:curve target.
bool _ReadCurveGuidePoints(const UsdPrim &prim, UsdTimeCode time,
                           VtVec3fArray *points);

// Clip existing polylines into evenly spaced dashes or cardinal brackets.
void _PatternVolumeCurves(RigExecVolumeGuideElement *element, bool brackets);

// One sphere iso-surface at local radius \p radius, as a mesh, implicit
// sphere, or dotted basis-curve shell.
void _AppendSphereVolumeGuide(double radius, const GfVec3d &axisScale,
                              const GfVec3d &positiveScales,
                              const GfVec3d &negativeScales, bool wire,
                              const GfMatrix4d &rigidToAsset, double wireWidth,
                              float opacity,
                              std::vector<RigExecVolumeGuideElement> *elements,
                              bool clean = false);

// One plane iso-surface: the rectangle at signed offset \p distance along
// the local rigExec:planeAxis, sized by the in-plane half-extents.
void _AppendPlaneVolumeGuide(double distance, int axis, double extentU,
                             double extentV, bool bounded, bool wire,
                             const GfMatrix4d &rigidToAsset, double wireWidth,
                             std::vector<RigExecVolumeGuideElement> *elements);

// One curve iso-surface: the tube of local radius \p radius about the
// polyline through \p localCurve.
void _AppendCurveVolumeGuide(double radius, const VtVec3fArray &localCurve,
                             const GfMatrix4d &toAsset, bool wire,
                             double wireWidth,
                             std::vector<RigExecVolumeGuideElement> *elements,
                             bool clean = false);

// Can this attribute's value differ between two generations with no stage
// notice in between? Connected (a dial drives it) or time-varying.
bool _GuideAttrIsLive(const UsdAttribute &attribute);

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_GUIDE_GEOMETRY_H
