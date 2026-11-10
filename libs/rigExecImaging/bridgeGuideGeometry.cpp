// Guide tokens, static geometry tables and placement helpers for the
// imaging bridge's guides. Tokens and unit tables are interned and built
// once at load, in this TU; the tables and their initialization
// dependencies never split across TUs.
#include "bridgeGuideGeometry.h"

#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

namespace rigExec {

// Guide names are interned before frame publication.
const TfToken _tControlType("RigExecControl");
const TfToken _tJointType("RigExecJoint");
const TfToken _tSphereWeightType("RigExecSphereWeight");
const TfToken _tPlaneWeightType("RigExecPlaneWeight");
const TfToken _tCurveWeightType("RigExecCurveWeight");
const TfToken _tY("Y");
const TfToken _tRigExecGenerated("__RigExecGenerated");
const TfToken _tCircle("circle");
const TfToken _tConstraintsRel("rigExec:constraints");
const TfToken _tGuideDisplayColor("guide:displayColor");
const TfToken _tGuideDisplayOpacity("guide:displayOpacity");
const TfToken _tGuideDisplayOpacityInvert("guide:displayOpacityInvert");
const TfToken _tGuideDisplayOpacityMin("guide:displayOpacityMin");
const TfToken _tGuideDrawMode("guide:drawMode");
const TfToken _tGuideOffset("guide:offset");
const TfToken _tGuidePlaneNormal("guide:planeNormal");
const TfToken _tGuideRadius("guide:radius");
const TfToken _tGuideScaleX("guide:scaleX");
const TfToken _tGuideScaleY("guide:scaleY");
const TfToken _tGuideScaleZ("guide:scaleZ");
const TfToken _tGuideShape("guide:shape");
const TfToken _tGuideWireWidth("guide:wireWidth");
const TfToken _tPoints("points");
const TfToken _tRigExecPlaneAxis("rigExec:planeAxis");
const TfToken _tRigExecPlaneBounds("rigExec:planeBounds");
const TfToken _tWire("wire");
const TfToken _tGeometry("geometry"), _tCage("cage");
const TfToken _tBrackets("brackets"), _tShell("shell");
const TfToken _tFalloff("falloff");
const TfToken _tWeightDisplay("guide:weightDisplay");
const TfToken _tHeatmap("heatmap"), _tBands("bands");

const TfToken _volumeInputsExtentU("inputs:extentU");
const TfToken _volumeInputsExtentV("inputs:extentV");
const TfToken _volumeInputsFalloffMax("inputs:falloffMax");
const TfToken _volumeInputsFalloffMin("inputs:falloffMin");
const TfToken _volumeInputsScaleX("inputs:scaleX");
const TfToken _volumeInputsScaleXNeg("inputs:scaleXNeg");
const TfToken _volumeInputsScaleXPos("inputs:scaleXPos");
const TfToken _volumeInputsScaleY("inputs:scaleY");
const TfToken _volumeInputsScaleYNeg("inputs:scaleYNeg");
const TfToken _volumeInputsScaleYPos("inputs:scaleYPos");
const TfToken _volumeInputsScaleZ("inputs:scaleZ");
const TfToken _volumeInputsScaleZNeg("inputs:scaleZNeg");
const TfToken _volumeInputsScaleZPos("inputs:scaleZPos");

namespace {

// Tokens only this TU draws with; no other guide TU reads them.
const TfToken _tFaceVarying("faceVarying");
const TfToken _tRigExecCurve("rigExec:curve");
const TfToken _tVertex("vertex");

// Influence volume guides.
// A weight volume is otherwise invisible: it owns no points, and its effect
// is only legible once geometry has already moved -- by which time the
// artist is reading a deformation, not the region that caused it. What is
// actually being PLACED is the pair of surfaces the falloff band is defined
// by, so those are what is drawn: the falloffMin iso-surface (fully ON) and
// the falloffMax iso-surface (fully OFF).
// Everything here is built in the volume's own rigid local space, the SAME
// space the field is measured in (RigExecRigPose::weightFrames is the very
// matrix the kernels invert). That is what makes the drawing and the field
// agree by construction rather than by review -- and it is why the schema
// insists shape comes from floats and never from the transform: a scale
// baked into the placement would deform the field without deforming this
// guide, and the artist would be painting with a shape they cannot see.

// std::acos(-1) rather than M_PI, which needs _USE_MATH_DEFINES on MSVC.
const double _kTwoPi = 2.0 * std::acos(-1.0);

// Ring resolution for the round guides, matching the control guides' 32 so
// a sphere weight and a sphere control read as the same drawing.
constexpr int _kVolumeRingSegments = 32;

// Dots on one great circle of a sphere weight's exterior iso-surface.
// Basis curves have no stipple, so each mark is its own short curve and
// the gap is the absence of the next one. 48 marks, each covering 18% of
// its period: on the unit sphere that arc is shorter than the default
// guide:wireWidth, so a widened curve reads as a dot. The first mark
// starts at angle 0, which keeps a sample on the ring's axisA cardinal.
constexpr int _kSphereWireDots = 48;
constexpr double _kSphereWireDotDuty = 0.18;

// Segments around a curve weight's tube. Deliberately coarser: a curve
// guide draws one of these per polyline vertex, so the vertex count is the
// product of the two, and eight already reads as round at guide scale.
constexpr int _kVolumeTubeSegments = 8;

// Appends one closed ring about \p center in the plane spanned by \p axisA
// and \p axisB, whose LENGTHS are the ring's semi-axes.
// The first point is repeated at the end: linear nonperiodic curves have no
// wrap, so restating the start vertex is what closes the ring. That costs
// one vertex and avoids the linear-periodic path entirely -- the same trade
// the control guides make.
void
_AppendVolumeRing(const GfVec3f &center, const GfVec3f &axisA,
                  const GfVec3f &axisB, int segments, VtVec3fArray *points)
{
    for (int i = 0; i <= segments; ++i) {
        const double theta = _kTwoPi * double(i % segments) / double(segments);
        points->push_back(center + float(std::cos(theta)) * axisA +
                          float(std::sin(theta)) * axisB);
    }
}

// One dotted ring in the same plane: _kSphereWireDots separate two-point
// marks. Counts are appended beside the points so each mark is its own curve.
void
_AppendDottedVolumeRing(const GfVec3f &center, const GfVec3f &axisA,
                        const GfVec3f &axisB, VtVec3fArray *points,
                        VtIntArray *counts)
{
    constexpr int samples = 2;
    for (int dot = 0; dot < _kSphereWireDots; ++dot) {
        const double start = _kTwoPi * double(dot) / double(_kSphereWireDots);
        const double span =
            _kTwoPi * _kSphereWireDotDuty / double(_kSphereWireDots);
        for (int sample = 0; sample < samples; ++sample) {
            const double theta =
                start + span * (double(sample) / double(samples - 1));
            points->push_back(center + float(std::cos(theta)) * axisA +
                              float(std::sin(theta)) * axisB);
        }
        counts->push_back(samples);
    }
}

// Some unit vector perpendicular to \p t, chosen to stay well conditioned:
// crossing with the world axis LEAST aligned with t keeps the result far
// from zero length for every input.
GfVec3f
_PerpendicularTo(const GfVec3f &t)
{
    const GfVec3f seed =
        std::abs(t[0]) < 0.9f ? GfVec3f(1, 0, 0) : GfVec3f(0, 1, 0);
    GfVec3f u = GfCross(t, seed);
    u.Normalize();
    return u;
}

// Immutable unit geometry is prepared before frame publication.
// Each guide shapes these samples into its owned output arrays.
struct _UnitSphereMesh {
    std::vector<GfVec3f> units;
    std::vector<int> counts;
    std::vector<int> indices;
};

const _UnitSphereMesh _unitSphereMesh = []() {
    _UnitSphereMesh mesh;
    constexpr int longitude = 64, latitude = 32;
    constexpr double pi = 3.14159265358979323846;
    mesh.units.reserve(size_t((latitude - 1) * longitude) + 2);
    mesh.units.push_back(GfVec3f(0, 1, 0));
    for (int j = 1; j < latitude; ++j) {
        const double theta = pi * j / latitude;
        for (int i = 0; i < longitude; ++i) {
            const double phi = 2 * pi * i / longitude;
            mesh.units.push_back(GfVec3f(
                float(std::sin(theta) * std::cos(phi)), float(std::cos(theta)),
                float(std::sin(theta) * std::sin(phi))));
        }
    }
    mesh.units.push_back(GfVec3f(0, -1, 0));
    mesh.counts.reserve(size_t(longitude) * size_t(latitude - 1));
    mesh.indices.reserve(mesh.counts.capacity() * 4);
    const int bottom = int(mesh.units.size()) - 1;
    auto face = [&](std::initializer_list<int> ids) {
        mesh.counts.push_back(int(ids.size()));
        for (int id : ids)
            mesh.indices.push_back(id);
    };
    for (int i = 0; i < longitude; ++i) {
        const int next = (i + 1) % longitude;
        face({0, 1 + next, 1 + i});
        for (int j = 0; j < latitude - 2; ++j) {
            const int row = 1 + j * longitude;
            face({row + i, row + next, row + longitude + next,
                  row + longitude + i});
        }
        const int row = 1 + (latitude - 2) * longitude;
        face({row + i, row + next, bottom});
    }
    return mesh;
}();

struct _UnitSphereRings {
    VtVec3fArray points;
    VtIntArray counts;
};

const _UnitSphereRings _unitSphereRings = []() {
    const GfVec3f kX(1, 0, 0), kY(0, 1, 0), kZ(0, 0, 1);
    const GfVec3f kOrigin(0, 0, 0);
    _UnitSphereRings built;
    _AppendDottedVolumeRing(kOrigin, kX, kZ, &built.points, &built.counts);
    _AppendDottedVolumeRing(kOrigin, kX, kY, &built.points, &built.counts);
    _AppendDottedVolumeRing(kOrigin, kY, kZ, &built.points, &built.counts);
    return built;
}();

const _UnitSphereRings _cleanSphereRings = []() {
    _UnitSphereRings built;
    const GfVec3f axes[3] = {GfVec3f(1, 0, 0), GfVec3f(0, 1, 0),
                             GfVec3f(0, 0, 1)};
    for (int axis = 0; axis < 3; ++axis) {
        _AppendVolumeRing(GfVec3f(0), axes[axis], axes[(axis + 1) % 3], 64,
                          &built.points);
        built.counts.push_back(65);
    }
    return built;
}();

} // namespace

// The rigid ASSET-space placement matrix of one posed frame, or nothing
// when the frame cannot supply one.
// Guides are placed by a rigid matrix: scale/shear never leaks into the
// placement itself. Joint link lengths come from child origins, solver
// dimensions come from their frame landmarks, and controls ask for the
// removed basis magnitudes separately and fold those into their positive
// authored guide:scaleX/Y/Z multipliers.
// Shared by the joint/solver payload and the control guides so the two
// cannot drift apart: a guide drawn from a frame one of them rejects and
// the other accepts would be a difference nobody could see coming.
bool
_RigidGuideMatrix(const RigExecPointFrame &frame, GfMatrix4d *result,
                  GfVec3d *axisMagnitudes)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    static const RigExecPointFrame identity;
    GfMatrix4d m(1.0);
    if (!RigExecPointsToMatrix(identity.points, frame.points, &m)) {
        return false;
    }
    if (axisMagnitudes) {
        for (int axis = 0; axis < 3; ++axis) {
            const double magnitude = m.GetRow3(axis).GetLength();
            if (!std::isfinite(magnitude) || magnitude <= 0.0) {
                return false;
            }
            (*axisMagnitudes)[axis] = magnitude;
        }
    }
    // A basis that cannot be orthonormalized (collapsed/collinear axes)
    // draws nothing rather than publishing a broken placement.
    if (!m.Orthonormalize(/* issueWarning = */ false)) {
        return false;
    }
    // Proper rigid rotation: a reflected frame keeps its +X aim but has
    // its Z basis flipped so the determinant is positive.
    if (m.GetDeterminant3() < 0) {
        for (int c = 0; c < 3; ++c) {
            m[2][c] = -m[2][c];
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    *result = m;
    return true;
}

// One guide element from a posed frame: rig-space placement matrix, cone
// length, primitive radius, and whether it owns an origin sphere. Solver
// elements derive length from the frame's aim landmark distance. Joint
// elements pass child-derived link frames and lengths directly.
// A non-positive radius draws nothing at all. Unlike length, where zero
// legitimately means "sphere but no bone", a zero-radius guide is invisible
// either way, so publishing it would only cost the viewer geometry it
// cannot see.
bool
_AppendGuideFrame(const RigExecPointFrame &frame, double length,
                  double authoredRadius, bool useAimFallback, bool drawSphere,
                  RigExecPublishedPrim *published)
{
    GfMatrix4d m(1.0);
    if (!_RigidGuideMatrix(frame, &m)) {
        return false;
    }
    if (length <= 0.0) {
        length =
            useAimFallback ? (frame.X() - frame.Origin()).GetLength() : 0.0;
    }
    if (!std::isfinite(length)) {
        return false;
    }
    if (!std::isfinite(authoredRadius) || authoredRadius <= 0.0) {
        return false;
    }
    published->guideFrames.push_back(m);
    published->guideLengths.push_back(length);
    published->guideRadii.push_back(authoredRadius);
    published->guideDrawSpheres.push_back(drawSphere);
    return true;
}

// Builds the rigid frame and length of the link from parent to child. The
// cone is rotationally symmetric about X, so preserving the parent's Y axis
// where possible gives a stable frame without assigning semantic roll to the
// hierarchy edge.
bool
_JointLinkFrame(const RigExecPointFrame &parent, const RigExecPointFrame &child,
                GfMatrix4d *frame, double *length)
{
    GfMatrix4d parentMatrix(1.0), childMatrix(1.0);
    if (!_RigidGuideMatrix(parent, &parentMatrix) ||
        !_RigidGuideMatrix(child, &childMatrix)) {
        return false;
    }
    GfVec3d x =
        childMatrix.ExtractTranslation() - parentMatrix.ExtractTranslation();
    *length = x.GetLength();
    if (!std::isfinite(*length) || *length <= 1e-12) {
        return false;
    }
    x /= *length;

    GfVec3d y = parentMatrix.GetRow3(1);
    y -= GfDot(y, x) * x;
    if (y.GetLength() <= 1e-12) {
        y = parentMatrix.GetRow3(2);
        y -= GfDot(y, x) * x;
    }
    if (y.GetLength() <= 1e-12) {
        const GfVec3d seed =
            std::abs(x[0]) < 0.9 ? GfVec3d(1, 0, 0) : GfVec3d(0, 1, 0);
        y = GfCross(seed, x);
    }
    y.Normalize();
    GfVec3d z = GfCross(x, y);
    z.Normalize();
    y = GfCross(z, x);
    y.Normalize();

    *frame = parentMatrix;
    for (int column = 0; column < 3; ++column) {
        (*frame)[0][column] = x[column];
        (*frame)[1][column] = y[column];
        (*frame)[2][column] = z[column];
    }
    return true;
}

// The prim's resolved render purpose, which is what BBoxCache classifies
// its extent by -- so the guide it draws is stamped with the same value.
TfToken
_ReadGuidePurpose(const UsdPrim &prim)
{
    if (const UsdGeomImageable imageable = UsdGeomImageable(prim)) {
        const TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return UsdGeomTokens->default_;
}

// The value the FIRST connection source of \p attr holds at the pose's
// time, or nothing when the attribute has no connection or the source
// cannot supply one (dangling path, a relationship, no value).
// UsdAttribute::Get() does NOT follow connections: they are a
// shading-graph concept, not value resolution. So a rig that authored
// `guide:displayOpacity.connect = </limb_params.avars:ikfk>` compiled,
// drew nothing different, and gave no hint why. This is the read-through.
// A property-mover result for the source wins over its authored value,
// exactly as the volume guides read their driven dimensions: a dial that
// is itself computed must fade the guide by what it computed. A value the
// rig published on the attribute itself wins over both: it is the
// connection read at its phase (RigExecPhasedConnection), the base of a
// computed dial unless the attribute declares another.
bool
_ReadConnectedValue(const UsdAttribute &attr, const RigExecRigPose &pose,
                    VtValue *held)
{
    SdfPathVector connections;
    if (!attr.GetConnections(&connections) || connections.empty()) {
        return false;
    }
    const auto phased = pose.movedProperties.find(attr.GetPath());
    if (phased != pose.movedProperties.end() && !phased->second.IsEmpty()) {
        *held = phased->second;
        return true;
    }
    const SdfPath &sourcePath = connections.front();
    if (!sourcePath.IsPropertyPath()) {
        return false;
    }
    const UsdAttribute source = attr.GetStage()->GetAttributeAtPath(sourcePath);
    if (!source) {
        return false;
    }
    const auto moved = pose.movedProperties.find(sourcePath);
    if (moved != pose.movedProperties.end() && !moved->second.IsEmpty()) {
        *held = moved->second;
        return true;
    }
    return source.Get(held, pose.time);
}

// The scalar a float OR double VtValue holds -- avars are double, the
// biped's ikfk dial is float, and a guide must accept either -- or
// nothing when it holds neither or the value is not finite.
bool
_HeldScalar(const VtValue &held, double *value)
{
    if (held.IsHolding<float>()) {
        *value = held.UncheckedGet<float>();
    } else if (held.IsHolding<double>()) {
        *value = held.UncheckedGet<double>();
    } else {
        return false;
    }
    return std::isfinite(*value);
}

void
_ReadGuideStyle(const UsdPrim &prim, const RigExecRigPose &pose,
                RigExecPublishedPrim *published)
{
    if (!prim) {
        return;
    }
    const UsdTimeCode time = pose.time;
    published->guidePurpose = _ReadGuidePurpose(prim);
    if (UsdAttribute a = prim.GetAttribute(_tGuideDisplayColor)) {
        VtValue held;
        if (_ReadConnectedValue(a, pose, &held) && held.IsHolding<GfVec3f>()) {
            published->guideColor = held.UncheckedGet<GfVec3f>();
        } else {
            a.Get(&published->guideColor, time);
        }
    }
    if (UsdAttribute a = prim.GetAttribute(_tGuideDisplayOpacity)) {
        VtValue held;
        double driven = 0.0;
        if (_ReadConnectedValue(a, pose, &held) && _HeldScalar(held, &driven)) {
            // A driven opacity is a dial's value, and a dial parks at its
            // end stops: without the complement one switch could not fade
            // two control sets in opposite directions, and without the
            // floor the inactive set would vanish -- unfindable, so
            // unswitchable-back. Both apply ONLY to a connected value; an
            // unconnected attribute draws exactly what it says.
            bool invert = false;
            if (UsdAttribute i =
                    prim.GetAttribute(_tGuideDisplayOpacityInvert)) {
                i.Get(&invert, time);
            }
            double floor = _kDefaultGuideOpacityMin;
            if (UsdAttribute m = prim.GetAttribute(_tGuideDisplayOpacityMin)) {
                VtValue heldMin;
                double authoredMin = 0.0;
                if (m.Get(&heldMin, time) &&
                    _HeldScalar(heldMin, &authoredMin)) {
                    floor = authoredMin;
                }
            }
            floor = std::min(1.0, std::max(0.0, floor));
            double opacity = invert ? 1.0 - driven : driven;
            opacity = std::min(1.0, std::max(floor, opacity));
            published->guideOpacity = static_cast<float>(opacity);
        } else {
            a.Get(&published->guideOpacity, time);
        }
    }
}

// The volume's rigid ASSET-space placement, or nothing when it cannot
// supply one.
// RemoveScaleShear() for exactly the reason _ResolveVolumeWeights applies
// it to the same matrix: inputs:scaleX/Y/Z is the sole authority on
// anisotropy, so a scale that leaked in through the transform would size
// the field and this guide differently.
bool
_RigidVolumeMatrix(const GfMatrix4d &placement, GfMatrix4d *result)
{
    GfMatrix4d rigid = placement.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return false;
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(rigid[r][c])) {
                return false;
            }
        }
    }
    *result = rigid;
    return true;
}

// The authored points of a curve weight's rigExec:curve target, accepting
// either an exact point3f[] property or a prim canonicalizing to .points --
// the same two spellings _ReadTargetPoints accepts in the evaluator, so a
// curve that drives a field is a curve this can draw.
bool
_ReadCurveGuidePoints(const UsdPrim &prim, UsdTimeCode time,
                      VtVec3fArray *points)
{
    const UsdRelationship rel = prim.GetRelationship(_tRigExecCurve);
    if (!rel) {
        return false;
    }
    SdfPathVector targets;
    rel.GetTargets(&targets);
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath &target = targets.front();
    const UsdPrim curvePrim =
        prim.GetStage()->GetPrimAtPath(target.GetPrimPath());
    if (!curvePrim) {
        return false;
    }
    const TfToken name =
        target.IsPropertyPath() ? target.GetNameToken() : _tPoints;
    const UsdAttribute attr = curvePrim.GetAttribute(name);
    return attr && attr.Get(points, time) && points->size() >= 2;
}

// Clip existing polylines into evenly spaced dashes or cardinal brackets.
// Positions stay on the original guide, including asymmetric sphere halves.
void
_PatternVolumeCurves(RigExecVolumeGuideElement *element, bool brackets)
{
    const auto &source = std::as_const(element->points);
    VtVec3fArray points;
    VtIntArray counts;
    size_t start = 0;
    for (const int count : std::as_const(element->counts)) {
        if (count < 2) {
            start += count;
            continue;
        }
        double length = 0;
        for (int i = 1; i < count; ++i)
            length += (source[start + i] - source[start + i - 1]).GetLength();
        if (length <= 1e-9) {
            start += count;
            continue;
        }
        const bool closed = source[start] == source[start + count - 1];
        const int marks = brackets ? (closed ? 4 : 2) : 16;
        const double period = length / marks;
        const double duty = brackets ? 0.22 : 0.62;
        double distance = 0;
        for (int i = 1; i < count; ++i) {
            const GfVec3f a = source[start + i - 1];
            const GfVec3f b = source[start + i];
            const double segment = (b - a).GetLength();
            const double end = distance + segment;
            if (segment > 1e-9) {
                // Center bracket marks at cardinal points, wrapping at zero.
                for (int mark = -1; mark <= marks; ++mark) {
                    const double lo = std::max(
                        distance,
                        period * (mark - (brackets ? duty * 0.5 : 0.0)));
                    const double hi = std::min(
                        end, period * (mark + (brackets ? duty * 0.5 : duty)));
                    if (hi <= lo)
                        continue;
                    points.push_back(a + float((lo - distance) / segment) *
                                             (b - a));
                    points.push_back(a + float((hi - distance) / segment) *
                                             (b - a));
                    counts.push_back(2);
                }
            }
            distance = end;
        }
        start += count;
    }
    element->points = std::move(points);
    element->counts = std::move(counts);
}

// One sphere iso-surface at local radius \p radius.
// The radius and shared axis scales ride in the transform. Directional
// scales reshape the unit points; asymmetric geometry uses a closed mesh
// because an implicit sphere cannot represent unequal half-axes.
// \p wire selects the dotted great circles. The solid branch is the
// max-weight interior fill, and what geometry draw mode uses for both
// surfaces. \p opacity replaces guide:displayOpacity when non-negative.
void
_AppendSphereVolumeGuide(double radius, const GfVec3d &axisScale,
                         const GfVec3d &positiveScales,
                         const GfVec3d &negativeScales, bool wire,
                         const GfMatrix4d &rigidToAsset, double wireWidth,
                         float opacity,
                         std::vector<RigExecVolumeGuideElement> *elements,
                         bool clean)
{
    if (!std::isfinite(radius) || radius <= 0.0) {
        // A zero- or negative-radius iso-surface is a point or nothing at
        // all. falloffMin = 0 is the schema DEFAULT, so this is the
        // ordinary case for the inner surface, not an error: the volume
        // simply draws its outer boundary alone.
        return;
    }
    GfMatrix4d scale(1.0);
    scale.SetScale(GfVec3d(radius * axisScale[0], radius * axisScale[1],
                           radius * axisScale[2]));

    RigExecVolumeGuideElement element;
    element.xform = scale * rigidToAsset;
    element.opacity = opacity;
    const bool asymmetric = positiveScales != negativeScales;
    if (!wire && !asymmetric && !clean) {
        GfMatrix4d directional(1.0);
        directional.SetScale(positiveScales);
        element.xform = directional * element.xform;
        // Hydra's own implicit, exactly as the joint and control guides
        // use it. Unit radius, set explicitly by the scene index.
        element.primType = HdPrimTypeTokens->sphere;
        elements->push_back(std::move(element));
        return;
    }
    auto shapePoint = [&](GfVec3f p) {
        for (int axis = 0; axis < 3; ++axis)
            p[axis] *= float(p[axis] < 0 ? negativeScales[axis]
                                         : positiveScales[axis]);
        return p;
    };
    if (!wire) {
        element.primType = HdPrimTypeTokens->mesh;
        element.normalsInterpolation = _tVertex;
        const _UnitSphereMesh &unit = _unitSphereMesh;
        element.points.resize(unit.units.size());
        element.normals.resize(unit.units.size());
        for (size_t v = 0; v < unit.units.size(); ++v) {
            const GfVec3f &sample = unit.units[v];
            element.points[v] = shapePoint(sample);
            GfVec3f normal = sample;
            for (int a = 0; a < 3; ++a)
                normal[a] /= float(sample[a] < 0 ? negativeScales[a]
                                                 : positiveScales[a]);
            normal.Normalize();
            element.normals[v] = normal;
        }
        element.counts.assign(unit.counts.begin(), unit.counts.end());
        element.indices.assign(unit.indices.begin(), unit.indices.end());
        elements->push_back(std::move(element));
        return;
    }
    element.primType = HdPrimTypeTokens->basisCurves;
    element.wireWidth = wireWidth;
    // Three orthogonal great circles, dotted. Wire mode draws this both
    // on the exterior falloff and over the translucent interior fill.
    const _UnitSphereRings &rings =
        clean ? _cleanSphereRings : _unitSphereRings;
    element.points.resize(rings.points.size());
    for (size_t v = 0; v < rings.points.size(); ++v) {
        element.points[v] = shapePoint(rings.points[v]);
    }
    element.counts.assign(rings.counts.begin(), rings.counts.end());
    elements->push_back(std::move(element));
}

// One plane iso-surface: the rectangle at signed offset \p distance along
// the local rigExec:planeAxis, sized by the in-plane half-extents
// \p extentU and \p extentV.
// The size comes from inputs:extentU/extentV and from NOTHING ELSE. A
// half-size derived from the falloff band would resize the drawn square as
// well as slide it when the band is scrubbed. The band and the extents are different
// directions and different quantities: the band is a distance ALONG the
// axis and is the only thing \p distance carries, the extents are the size
// ACROSS it. Keeping them apart is bug (2) of this change.
// BOUNDED vs UNBOUNDED has to be readable at a glance, because it decides
// whether the rectangle is the field's edge or just a label. Bounded draws
// the closed rectangle alone: what you see is exactly what the field
// covers. Unbounded adds four short outward ticks at the edge midpoints --
// the arrowless "continues this way" mark, four extra segments and no
// extra draw decisions, and it survives being drawn at any scale because
// the ticks are a fraction of the extent rather than a fixed length.
// Filled corner flares or a fade would need a second material and a
// gradient the guide protocol does not carry.
// Note the per-axis divisors are deliberately absent: the plane kernel
// returns BEFORE inputs:scaleX/Y/Z is folded into its matrix, so a scaled
// plane guide would draw a band the field does not have.
void
_AppendPlaneVolumeGuide(double distance, int axis, double extentU,
                        double extentV, bool bounded, bool wire,
                        const GfMatrix4d &rigidToAsset, double wireWidth,
                        std::vector<RigExecVolumeGuideElement> *elements)
{
    if (!std::isfinite(distance) || !std::isfinite(extentU) ||
        !std::isfinite(extentV) || extentU <= 0.0 || extentV <= 0.0) {
        // The same values the bounded kernel rejects, and for the same
        // reason: there is no such rectangle. An unbounded plane with a
        // degenerate extent still HAS a field, it just has no square worth
        // drawing, so this draws nothing rather than guessing a size.
        return;
    }
    GfVec3f center(0.0f);
    center[axis] = float(distance);
    GfVec3f u(0.0f), v(0.0f);
    u[(axis + 1) % 3] = float(extentU);
    v[(axis + 2) % 3] = float(extentV);

    RigExecVolumeGuideElement element;
    // The dimensions live in the points here, not in the transform: a
    // plane has no implicit Hydra form whose schema could carry them.
    element.xform = rigidToAsset;
    const GfVec3f corners[4] = {center - u - v, center - u + v, center + u + v,
                                center + u - v};
    if (wire) {
        element.primType = HdPrimTypeTokens->basisCurves;
        element.wireWidth = wireWidth;
        for (int i = 0; i < 5; ++i) {
            element.points.push_back(corners[i % 4]);
        }
        element.counts = VtIntArray{5};
    } else {
        element.primType = HdPrimTypeTokens->mesh;
        // A rectangle has no thickness. Make both sides visible, unlike the
        // closed curve tube whose outward winding is sufficient.
        element.doubleSided = true;
        for (const GfVec3f &corner : corners) {
            element.points.push_back(corner);
        }
        element.counts = VtIntArray{4};
        element.indices = VtIntArray{0, 1, 2, 3};
    }
    elements->push_back(std::move(element));

    if (bounded) {
        return;
    }
    // The ticks are their own element rather than extra curves on the
    // rectangle: in `geometry` draw mode the rectangle is a mesh, and a
    // mesh cannot carry them. One element type per element keeps both
    // draw modes on the same code.
    RigExecVolumeGuideElement ticks;
    ticks.xform = rigidToAsset;
    ticks.primType = HdPrimTypeTokens->basisCurves;
    ticks.wireWidth = wireWidth;
    // Each half-extent vector reaches the midpoint of one edge AND points
    // outward from it, so one array does both jobs.
    const GfVec3f outward[4] = {u, -u, v, -v};
    for (const GfVec3f &out : outward) {
        const GfVec3f base = center + out;
        ticks.points.push_back(base);
        ticks.points.push_back(base + out * 0.25f);
        ticks.counts.push_back(2);
    }
    elements->push_back(std::move(ticks));
}

// One curve iso-surface: the tube of local radius \p radius about the
// polyline through \p localCurve.
// Wire mode draws rings at every polyline vertex plus four longitudinal
// rails. Geometry mode uses the same transported rings as shared vertices
// of a closed manifold: outward-wound side quads and oppositely-wound end
// caps. Its normals are face-varying so the caps remain flat without
// splitting the ring vertices and turning the topology back into an open
// surface.
// The polyline is already in the DIVIDED local space, so the tube is round
// here and the per-axis divisors turn it elliptical through the transform
// -- the same order the kernel folds them in.
void
_AppendCurveVolumeGuide(double radius, const VtVec3fArray &localCurve,
                        const GfMatrix4d &toAsset, bool wire, double wireWidth,
                        std::vector<RigExecVolumeGuideElement> *elements,
                        bool clean)
{
    if (!std::isfinite(radius) || radius <= 0.0 || localCurve.size() < 2) {
        return;
    }

    // Consecutive coincident points define no segment and would create a
    // strip of zero-area quads. Removing only those points preserves the
    // polyline locus (and therefore the field) while keeping the guide a
    // valid surface. A non-finite curve is not drawable at all.
    std::vector<GfVec3f> curve;
    curve.reserve(localCurve.size());
    for (const GfVec3f &p : localCurve) {
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2])) {
            return;
        }
        if (curve.empty() || (p - curve.back()).GetLength() > 1e-6f) {
            curve.push_back(p);
        }
    }
    if (curve.size() < 2) {
        return;
    }

    const int tubeSegments = clean ? 32 : _kVolumeTubeSegments;
    const size_t vertices = curve.size();
    std::vector<GfVec3f> segmentTangents(vertices - 1);
    for (size_t k = 0; k + 1 < vertices; ++k) {
        segmentTangents[k] = curve[k + 1] - curve[k];
        segmentTangents[k].Normalize();
    }

    // Rotation-minimizing per-vertex frames. Projecting the preceding U
    // axis onto the new tangent plane transports it continuously around a
    // bend instead of independently choosing a world axis at every knot,
    // which can flip a mesh strip by 180 degrees.
    std::vector<GfVec3f> tangents(vertices), ringU(vertices), ringV(vertices);
    for (size_t k = 0; k < vertices; ++k) {
        if (k == 0) {
            tangents[k] = segmentTangents.front();
        } else if (k + 1 == vertices) {
            tangents[k] = segmentTangents.back();
        } else {
            tangents[k] = segmentTangents[k - 1] + segmentTangents[k];
            if (tangents[k].GetLength() < 1e-6f) {
                // A 180-degree turn has no unique bisector. The outgoing
                // segment is at least a valid ring plane.
                tangents[k] = segmentTangents[k];
            }
            tangents[k].Normalize();
        }
        GfVec3f u = k == 0 ? _PerpendicularTo(tangents[k])
                           : ringU[k - 1] -
                                 GfDot(ringU[k - 1], tangents[k]) * tangents[k];
        if (u.GetLength() < 1e-6f) {
            u = _PerpendicularTo(tangents[k]);
        } else {
            u.Normalize();
        }
        if (k > 0 && GfDot(u, ringU[k - 1]) < 0.0f) {
            u = -u;
        }
        GfVec3f v = GfCross(tangents[k], u);
        v.Normalize();
        ringU[k] = u;
        ringV[k] = v;
    }

    RigExecVolumeGuideElement element;
    element.xform = toAsset;
    if (wire) {
        element.primType = HdPrimTypeTokens->basisCurves;
        element.wireWidth = wireWidth;
        for (size_t k = 0; k < vertices; ++k) {
            // Keep at most five section rings; rails still follow every bend.
            if (clean && k != vertices - 1 &&
                k % std::max(size_t(1), (vertices - 1 + 3) / 4) != 0)
                continue;
            _AppendVolumeRing(curve[k], float(radius) * ringU[k],
                              float(radius) * ringV[k], tubeSegments,
                              &element.points);
            element.counts.push_back(tubeSegments + 1);
        }
        // Four rails, at the quarter turns of each ring.
        for (int quarter = 0; quarter < 4; ++quarter) {
            const double theta = _kTwoPi * double(quarter) / 4.0;
            const float c = float(std::cos(theta));
            const float s = float(std::sin(theta));
            for (size_t k = 0; k < vertices; ++k) {
                element.points.push_back(
                    curve[k] + float(radius) * (c * ringU[k] + s * ringV[k]));
            }
            element.counts.push_back(int(vertices));
        }
        elements->push_back(std::move(element));
        return;
    }

    element.primType = HdPrimTypeTokens->mesh;
    element.normalsInterpolation = _tFaceVarying;
    element.points.reserve(vertices * tubeSegments);
    for (size_t k = 0; k < vertices; ++k) {
        for (int j = 0; j < tubeSegments; ++j) {
            const double theta = _kTwoPi * double(j) / double(tubeSegments);
            const GfVec3f radial = float(std::cos(theta)) * ringU[k] +
                                   float(std::sin(theta)) * ringV[k];
            element.points.push_back(curve[k] + float(radius) * radial);
        }
    }

    // Append one polygon plus a flat, outward normal at every face corner.
    // Face-varying normals keep the octagonal side facets honest and leave
    // a hard edge where each cap meets the tube.
    auto appendFace = [&element](const int *face, size_t count) {
        element.counts.push_back(static_cast<int>(count));
        element.indices.insert(element.indices.end(), face, face + count);
        GfVec3f normal(0.0f);
        const GfVec3f origin = element.points[face[0]];
        for (size_t i = 1; i + 1 < count; ++i) {
            normal += GfCross(element.points[face[i]] - origin,
                              element.points[face[i + 1]] - origin);
        }
        if (normal.GetLength() > 1e-9f) {
            normal.Normalize();
        }
        for (size_t i = 0; i < count; ++i) {
            element.normals.push_back(normal);
        }
    };

    // Side quads. With V = tangent x U, increasing the ring index and then
    // advancing along the curve winds the face outward.
    for (size_t k = 0; k + 1 < vertices; ++k) {
        for (int j = 0; j < tubeSegments; ++j) {
            const int next = (j + 1) % tubeSegments;
            const int a = int(k * tubeSegments) + j;
            const int b = int(k * tubeSegments) + next;
            const int c = int((k + 1) * tubeSegments) + next;
            const int d = int((k + 1) * tubeSegments) + j;
            const int quad[] = {a, b, c, d};
            appendFace(quad, 4);
        }
    }

    // The start cap runs opposite the ring; the end cap follows it. Their
    // boundary edges therefore oppose the adjacent side edges, making the
    // complete face set a consistently oriented closed manifold.
    std::vector<int> startCap, endCap;
    startCap.reserve(tubeSegments);
    endCap.reserve(tubeSegments);
    for (int j = 0; j < tubeSegments; ++j) {
        startCap.push_back(tubeSegments - 1 - j);
        endCap.push_back(int((vertices - 1) * tubeSegments) + j);
    }
    appendFace(startCap.data(), startCap.size());
    appendFace(endCap.data(), endCap.size());
    elements->push_back(std::move(element));
}

// Can this attribute's value differ between two generations with no stage
// notice in between? Connected (a dial drives it) or time-varying.
bool
_GuideAttrIsLive(const UsdAttribute &attribute)
{
    return attribute && (attribute.HasAuthoredConnections() ||
                         attribute.ValueMightBeTimeVarying());
}

} // namespace rigExec
