// Guide rendering, part 1: the styling caches, joint-hierarchy cache, cloth
// constraint bindings, and the joint/control guide fills. The volume guides
// and the weight overlay are the same class in bridgeVolumeGuides.cpp.
#include "rigExecSchema/schemaNames.h"
#include "bridgeGuides.h"

#include "bridgeGuideGeometry.h"

#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/clothPlayback.h"
#include "rigExec/movers/clothInputs.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rigExec {

namespace {
// Namespace-scope so the token registry is interned once, not under a
// function-local static guard on every forwarded notice.
const TfToken kPurpose("purpose");
} // namespace

RigExecBridgeGuides::RigExecBridgeGuides(const SdfPath &rigPath)
    : _rigPath(rigPath)
{
}

void
RigExecBridgeGuides::InvalidateGuideCaches()
{
    _clothGuideSerial = ~uint64_t(0);
    _guideInputs.clear();
    _jointChildren.clear();
    _jointChildrenValid = false;
}

void
RigExecBridgeGuides::NoteGuideEdits(const UsdNotice::ObjectsChanged &notice)
{
    _guideNoticesForwarded = true;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath()) {
            // A prim added, removed, retyped or re-parented can move the
            // joint hierarchy and which joints sit in control space, as well
            // as any styling: everything goes.
            InvalidateGuideCaches();
            return;
        }
    }
    // Property resyncs (a spec created or removed -- the first time an avar
    // is authored in a session) are judged exactly like value edits.
    std::vector<SdfPath> edited;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        edited.push_back(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        edited.push_back(path);
    }
    for (const SdfPath &path : edited) {
        const SdfPath prim = path.GetPrimPath();
        if (!path.IsPropertyPath()) {
            // Prim metadata: be conservative for the prim and below.
            for (auto it = _guideInputs.begin(); it != _guideInputs.end();) {
                it = it->first.HasPrefix(prim) ? _guideInputs.erase(it)
                                               : std::next(it);
            }
            continue;
        }
        const TfToken &name = path.GetNameToken();
        if (name == kPurpose) {
            // Purpose inherits: every cached descendant read it.
            for (auto it = _guideInputs.begin(); it != _guideInputs.end();) {
                it = it->first.HasPrefix(prim) ? _guideInputs.erase(it)
                                               : std::next(it);
            }
        } else if (TfStringStartsWith(name.GetString(),
                                      SchemaNamePrefix::guide)) {
            _guideInputs.erase(prim);
        }
    }
}

void
RigExecBridgeGuides::_SyncGuideCaches(const RigExecGuideCall &call)
{
    // A bridge the registry forwards notices to has had every edit applied
    // selectively by NoteGuideEdits; clearing on the serial as well would
    // throw that away on every pose edit.
    if (_guideNoticesForwarded) {
        return;
    }
    // Every stage edit the evaluator saw -- through the registry or not;
    // a bridge driven directly (tests, headless tools) gets no registry
    // notice forwarding, and must not publish stale styling for it.
    const uint64_t serial = call.stageEditSerial;
    if (serial != _guideCacheSerial) {
        _guideInputs.clear();
        _jointChildren.clear();
        _jointChildrenValid = false;
        _guideCacheSerial = serial;
    }
}

RigExecBridgeGuides::_GuideInputs &
RigExecBridgeGuides::_GuideInputsFor(const RigExecGuideCall &call,
                                     const SdfPath &path)
{
    auto found = _guideInputs.find(path);
    if (found == _guideInputs.end()) {
        found = _guideInputs.emplace(path, _GuideInputs()).first;
        found->second.prim = call.stage->GetPrimAtPath(path);
    }
    return found->second;
}

double
RigExecBridgeGuides::_GuideRadius(_GuideInputs &inputs, UsdTimeCode time)
{
    // Schema default, so an unauthored joint keeps Hydra's own fallback
    // radius and every existing rig looks exactly as it did.
    if (!inputs.prim) {
        return 1.0;
    }
    if (!inputs.radiusReady || inputs.radiusLive) {
        double radius = 1.0;
        const UsdAttribute a = inputs.prim.GetAttribute(_tGuideRadius);
        if (a) {
            a.Get(&radius, time);
        }
        inputs.radius = radius;
        inputs.radiusLive = _GuideAttrIsLive(a);
        inputs.radiusReady = true;
    }
    return inputs.radius;
}

// The static half of _ReadGuideStyle's connected-opacity rule, into
// \p inputs: held only for a single property connection whose invert and
// floor cannot move without a notice on this prim, which drops the entry
// (NoteGuideEdits, _SyncGuideCaches).
void
RigExecBridgeGuides::_ReadOpacityRule(_GuideInputs &inputs, UsdTimeCode time)
{
    inputs.opacityRuleReady = true;
    inputs.opacityRuleHeld = false;
    const UsdPrim &prim = inputs.prim;
    SdfPathVector connections;
    if (!inputs.opacityAttr.GetConnections(&connections) ||
        connections.empty() || !connections.front().IsPropertyPath()) {
        return;
    }
    const UsdAttribute invert = prim.GetAttribute(_tGuideDisplayOpacityInvert);
    const UsdAttribute floor = prim.GetAttribute(_tGuideDisplayOpacityMin);
    if (_GuideAttrIsLive(invert) || _GuideAttrIsLive(floor)) {
        return;
    }
    inputs.opacityInvert = false;
    if (invert) {
        invert.Get(&inputs.opacityInvert, time);
    }
    double value = _kDefaultGuideOpacityMin;
    if (floor) {
        VtValue held;
        double authored = 0.0;
        if (floor.Get(&held, time) && _HeldScalar(held, &authored)) {
            value = authored;
        }
    }
    inputs.opacityFloor = std::min(1.0, std::max(0.0, value));
    inputs.opacitySource =
        prim.GetStage()->GetAttributeAtPath(connections.front());
    inputs.opacityRuleHeld = true;
}

// _ReadGuideStyle's connected-opacity rule over the held static half: the
// dial read as _ReadConnectedValue reads it, else the attribute's own value
// (\p fallback where it holds none).
float
RigExecBridgeGuides::_HeldOpacity(const _GuideInputs &inputs,
                                  const RigExecRigPose &pose, float fallback)
{
    VtValue held;
    bool read = false;
    const auto phased = pose.movedProperties.find(inputs.opacityAttr.GetPath());
    if (phased != pose.movedProperties.end() && !phased->second.IsEmpty()) {
        held = phased->second;
        read = true;
    } else if (inputs.opacitySource) {
        const auto moved =
            pose.movedProperties.find(inputs.opacitySource.GetPath());
        if (moved != pose.movedProperties.end() && !moved->second.IsEmpty()) {
            held = moved->second;
            read = true;
        } else {
            read = inputs.opacitySource.Get(&held, pose.time);
        }
    }
    double driven = 0.0;
    if (read && _HeldScalar(held, &driven)) {
        const double opacity = inputs.opacityInvert ? 1.0 - driven : driven;
        return static_cast<float>(
            std::min(1.0, std::max(inputs.opacityFloor, opacity)));
    }
    float opacity = fallback;
    inputs.opacityAttr.Get(&opacity, pose.time);
    return opacity;
}

void
RigExecBridgeGuides::_ReadGuideStyleCached(_GuideInputs &inputs,
                                           const RigExecRigPose &pose,
                                           RigExecPublishedPrim *published)
{
    const UsdPrim &prim = inputs.prim;
    if (!prim) {
        return;
    }
    const UsdTimeCode time = pose.time;
    if (!inputs.styleReady) {
        inputs.styleReady = true;
        inputs.purpose = _ReadGuidePurpose(prim);
        inputs.colorAttr = prim.GetAttribute(_tGuideDisplayColor);
        inputs.colorLive = _GuideAttrIsLive(inputs.colorAttr);
        if (inputs.colorAttr && !inputs.colorLive) {
            inputs.hasColor = inputs.colorAttr.Get(&inputs.color, time);
        }
        inputs.opacityAttr = prim.GetAttribute(_tGuideDisplayOpacity);
        inputs.opacityLive = _GuideAttrIsLive(inputs.opacityAttr);
        if (inputs.opacityAttr && !inputs.opacityLive) {
            inputs.hasOpacity = inputs.opacityAttr.Get(&inputs.opacity, time);
        }
    }
    published->guidePurpose = inputs.purpose;
    if (inputs.colorAttr) {
        if (inputs.colorLive) {
            VtValue held;
            if (_ReadConnectedValue(inputs.colorAttr, pose, &held) &&
                held.IsHolding<GfVec3f>()) {
                published->guideColor = held.UncheckedGet<GfVec3f>();
            } else {
                inputs.colorAttr.Get(&published->guideColor, time);
            }
        } else if (inputs.hasColor) {
            published->guideColor = inputs.color;
        }
    }
    if (inputs.opacityAttr) {
        if (inputs.opacityLive) {
            if (!inputs.opacityRuleReady) {
                _ReadOpacityRule(inputs, time);
            }
            if (inputs.opacityRuleHeld) {
                // A fresh prim's opacity, as _ReadGuideStyle's scratch.
                static const float unread = RigExecPublishedPrim().guideOpacity;
                published->guideOpacity = _HeldOpacity(inputs, pose, unread);
            } else {
                // _ReadGuideStyle's opacity rule verbatim (connected dial,
                // invert, floor), every part of it read again.
                RigExecPublishedPrim scratch;
                _ReadGuideStyle(prim, pose, &scratch);
                published->guideOpacity = scratch.guideOpacity;
            }
        } else if (inputs.hasOpacity) {
            published->guideOpacity = inputs.opacity;
        }
    }
}

const std::map<SdfPath, std::vector<SdfPath>> &
RigExecBridgeGuides::_JointChildren(const RigExecGuideCall &call,
                                    const RigExecRigPose &pose)
{
    // Namespace nesting is joint hierarchy, which only a recompile can
    // change -- so the map is rebuilt when the compiled rig or the joint
    // count moves, not on every generation.
    const size_t key = call.compiledRigSignature ^
                       (pose.jointFramesFinal.size() * 0x9E3779B97F4A7C15ull);
    if (_jointChildrenValid && key == _jointChildrenKey) {
        return _jointChildren;
    }
    _jointChildren.clear();
    for (const auto &[childPath, childFrame] : pose.jointFramesFinal) {
        for (SdfPath ancestor = childPath.GetParentPath();
             !ancestor.IsEmpty() && ancestor != _rigPath;
             ancestor = ancestor.GetParentPath()) {
            if (pose.jointFramesFinal.count(ancestor)) {
                _jointChildren[ancestor].push_back(childPath);
                break;
            }
        }
    }
    _jointChildrenKey = key;
    _jointChildrenValid = true;
    return _jointChildren;
}

void
RigExecBridgeGuides::_FillClothConstraintGuides(
    const RigExecGuideCall &call, const RigExecRigPose &pose,
    RigExecImagingSnapshot *snapshot)
{
    // Resolve the same weighted samples as the mover, through its own
    // declaration and leaf reads, only when authored inputs change.
    // Publication runs on the stage's owning thread.
    const uint64_t serial = call.stageEditSerial;
    if (_clothGuideSerial != serial) {
        _clothGuideBindings.clear();
        for (const auto &mover : call.moverOrder) {
            if (mover.schemaType != SchemaTypeName::RigExecClothMover)
                continue;
            const UsdPrim moverPrim =
                call.stage->GetPrimAtPath(mover.moverPath);
            if (!moverPrim)
                continue;
            for (const auto &constraint :
                 RigExecRelationshipTargets(moverPrim, _tConstraintsRel)) {
                for (const auto &destination : mover.targets) {
                    if (destination.GetNameToken() != _tPoints)
                        continue;
                    _ClothGuideBinding guide;
                    guide.constraint = constraint;
                    guide.sourceMesh = destination.GetPrimPath();
                    std::vector<RigExecRevisionLeafKey> leaves;
                    RigExecRevisionBinding binding;
                    auto declaration = cloth_inputs::StageDeclaration(
                        call.stage, &leaves, &binding);
                    if (declaration.type(constraint) !=
                        SchemaTypeName::RigExecClothConstraintSet)
                        continue;
                    const auto &T = *cloth_inputs::_clothInputsTokens;
                    VtDictionary kind;
                    declaration.Attribute(
                        &kind, "kind", constraint, T.rigExecKind,
                        RigExecRevisionLeafType::Token, VtValue(T.stretch));
                    guide.targetMesh =
                        declaration.Target(constraint, T.rigExecTargetMesh)
                            .GetPrimPath();
                    if (guide.targetMesh.IsEmpty())
                        guide.targetMesh = guide.sourceMesh;
                    const auto source = declaration.Binding(
                        declaration.Target(constraint, T.rigExecSourceBinding),
                        guide.sourceMesh);
                    const auto target = declaration.Binding(
                        declaration.Target(constraint, T.rigExecTargetBinding),
                        guide.targetMesh);
                    const auto values = cloth_inputs::SampleLeaves(
                        call.stage, leaves, UsdTimeCode::Default());
                    const auto *attachment =
                        cloth_inputs::Value<TfToken>(values, kind, "kind");
                    if (declaration.valid && attachment &&
                        *attachment == "attachment" &&
                        cloth_inputs::Samples(values, source, &guide.source,
                                              nullptr) &&
                        cloth_inputs::Samples(values, target, &guide.target,
                                              nullptr) &&
                        !guide.source.empty() &&
                        guide.source.size() == guide.target.size()) {
                        _clothGuideBindings.push_back(std::move(guide));
                    }
                }
            }
        }
        _clothGuideSerial = serial;
    }
    if (_clothGuideBindings.empty())
        return;

    UsdGeomXformCache xforms(pose.time);
    const SdfPath asset = _rigPath.GetParentPath();
    const GfMatrix4d worldToAsset =
        xforms.GetLocalToWorldTransform(call.stage->GetPrimAtPath(asset))
            .GetInverse();
    const auto points = [&](const SdfPath &mesh, VtVec3fArray *result) {
        // GPU readback, when present, has already joined this generation.
        const auto published = snapshot->prims.find(mesh);
        if (published != snapshot->prims.end() && published->second.hasPoints) {
            *result = published->second.points;
            return true;
        }
        const SdfPath path = mesh.AppendProperty(_tPoints);
        const auto moved = pose.movedProperties.find(path);
        if (moved != pose.movedProperties.end()) {
            if (!moved->second.IsHolding<VtVec3fArray>())
                return false;
            *result = moved->second.UncheckedGet<VtVec3fArray>();
            return true;
        }
        const auto attr = call.stage->GetAttributeAtPath(path);
        return attr && attr.Get(result, pose.time);
    };
    const auto placement = [&](const SdfPath &mesh) {
        const GfMatrix4d base =
            xforms.GetLocalToWorldTransform(call.stage->GetPrimAtPath(mesh)) *
            worldToAsset;
        // Apply the nearest driven ancestor, respecting resetXformStack.
        for (SdfPath path = mesh; !path.IsAbsoluteRootPath();
             path = path.GetParentPath()) {
            const auto driven = pose.providerXforms.find(path);
            if (driven != pose.providerXforms.end()) {
                const GfMatrix4d ancestor =
                    xforms.GetLocalToWorldTransform(
                        call.stage->GetPrimAtPath(path)) *
                    worldToAsset;
                return base * ancestor.GetInverse() * driven->second;
            }
            const UsdGeomXformable xform(call.stage->GetPrimAtPath(path));
            if (xform && xform.GetResetXformStack())
                break;
        }
        return base;
    };
    const auto sample = [](const RigExecClothSample &binding,
                           const VtVec3fArray &vertices,
                           const GfMatrix4d &matrix, GfVec3f *result) {
        GfVec3d value(0);
        double sum = 0;
        for (size_t k = 0; k < 3; ++k) {
            const int32_t index = binding.vertices[k];
            const float weight = binding.weights[k];
            if (index < 0 || size_t(index) >= vertices.size() ||
                !std::isfinite(weight) || weight < 0)
                return false;
            value += GfVec3d(vertices[size_t(index)]) * weight;
            sum += weight;
        }
        if (std::abs(sum - 1) > 1e-5)
            return false;
        *result = GfVec3f(matrix.Transform(value));
        return std::isfinite((*result)[0]) && std::isfinite((*result)[1]) &&
               std::isfinite((*result)[2]);
    };
    for (const auto &guide : _clothGuideBindings) {
        VtVec3fArray source, target;
        if (!points(guide.sourceMesh, &source) ||
            !points(guide.targetMesh, &target))
            continue;
        const GfMatrix4d sourceFrame = placement(guide.sourceMesh);
        const GfMatrix4d targetFrame = placement(guide.targetMesh);
        auto &published = snapshot->prims[guide.constraint];
        published.assetRoot = asset;
        published.controlGuideShape = TfToken(SchemaValueText::custom);
        published.controlGuideDrawMode = _tWire;
        published.controlGuidePlaneNormal = _tY;
        _ReadGuideStyleCached(_GuideInputsFor(call, guide.constraint), pose,
                              &published);
        const auto width = call.stage->GetPrimAtPath(guide.constraint)
                               .GetAttribute(_tGuideWireWidth);
        if (width)
            width.Get(&published.controlGuideWireWidth, pose.time);
        for (size_t i = 0; i < guide.source.size(); ++i) {
            GfVec3f a, b;
            if (!sample(guide.source[i], source, sourceFrame, &a) ||
                !sample(guide.target[i], target, targetFrame, &b))
                continue;
            published.controlGuidePoints.push_back(a);
            published.controlGuidePoints.push_back(b);
            published.controlGuideCounts.push_back(2);
        }
        published.hasControlGuide = !published.controlGuideCounts.empty();
    }
}

void
RigExecBridgeGuides::_FillGuides(const RigExecGuideCall &call,
                                 const RigExecRigPose &pose,
                                 RigExecImagingSnapshot *snapshot)
{
    _SyncGuideCaches(call);
    // Guide frames are asset-space; the consumer needs to know which prim
    // that space is anchored to in order to place them.
    snapshot->assetRoot = _rigPath.GetParentPath();

    // Namespace nesting is joint hierarchy. Map every joint to its nearest
    // joint ancestor; scopes or other grouping prims between them do not
    // interrupt the same NamespaceAncestor relationship evaluation uses.
    // Cached per compiled rig (see _JointChildren).
    const std::map<SdfPath, std::vector<SdfPath>> &jointChildren =
        _JointChildren(call, pose);

    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        _GuideInputs &inputs = _GuideInputsFor(call, jointPath);
        const double authoredRadius = _GuideRadius(inputs, pose.time);
        RigExecPublishedPrim &published = snapshot->prims[jointPath];
        bool any = false;
        const auto children = jointChildren.find(jointPath);
        if (children != jointChildren.end()) {
            for (const SdfPath &childPath : children->second) {
                const auto child = pose.jointFramesFinal.find(childPath);
                if (child == pose.jointFramesFinal.end()) {
                    continue;
                }
                GfMatrix4d linkMatrix(1.0);
                double linkLength = 0.0;
                if (!_JointLinkFrame(frame, child->second, &linkMatrix,
                                     &linkLength)) {
                    continue;
                }
                RigExecPointFrame linkFrame;
                static const std::array<GfVec3d, 4> identity = {
                    GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                    GfVec3d(0, 0, 1)};
                for (size_t i = 0; i < identity.size(); ++i) {
                    linkFrame.points[i] = linkMatrix.Transform(identity[i]);
                }
                any = _AppendGuideFrame(linkFrame, linkLength, authoredRadius,
                                        /* useAimFallback = */ false,
                                        /* drawSphere = */ !any, &published) ||
                      any;
            }
        }
        // A leaf, or a joint whose children are all coincident/invalid, still
        // owns its selectable origin sphere.
        if (!any) {
            any = _AppendGuideFrame(frame, 0.0, authoredRadius,
                                    /* useAimFallback = */ false,
                                    /* drawSphere = */ true, &published);
        }
        if (any) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyleCached(inputs, pose, &published);
        }
    }
    for (const auto &[solverPath, frames] : pose.solverFrames) {
        RigExecPublishedPrim &published = snapshot->prims[solverPath];
        // An element has no prim of its own, but the SOLVER does, and it
        // is already where the guide colour and opacity are read from --
        // so the radius comes off it too, exactly as a joint's does. The
        // schema default is 1.0, so a rig that authors nothing draws what
        // it always drew.
        _GuideInputs &solverInputs = _GuideInputsFor(call, solverPath);
        const double authoredRadius = _GuideRadius(solverInputs, pose.time);
        bool any = false;
        for (const RigExecPointFrame &frame : frames) {
            any = _AppendGuideFrame(frame, 0.0, authoredRadius,
                                    /* useAimFallback = */ true,
                                    /* drawSphere = */ true, &published) ||
                  any;
        }
        if (any) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyleCached(solverInputs, pose, &published);
        } else if (!published.hasPoints && !published.hasNormals &&
                   !published.hasExtent && !published.hasXform) {
            snapshot->prims.erase(solverPath);
        }
    }
}

// Controls draw one synthesized shape each at their posed frame:
// shape, draw mode, and a positive per-axis draw
// multiplier are authored on the control. Evaluated frame-axis magnitudes
// multiply that authored size while the results scene index places the shape
// with the rigidized frame.
void
RigExecBridgeGuides::_FillControlGuides(const RigExecGuideCall &call,
                                        const RigExecRigPose &pose,
                                        RigExecImagingSnapshot *snapshot)
{
    _SyncGuideCaches(call);
    // Set here as well as in _FillGuides: control guide frames are
    // asset-space too, and a rig can publish these and no joint guides at
    // all (every joint frame degenerate, say), in which case this is the
    // only place the anchor gets recorded.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[controlPath, frame] : pose.controlFrames) {
        GfMatrix4d evaluated(1.0);
        const RigExecPointFrame identity;
        if (frame.IsValid() && !frame.IsDegenerate() &&
            RigExecPointsToMatrix(identity.points, frame.points, &evaluated)) {
            auto &published = snapshot->prims[controlPath];
            published.assetRoot = _rigPath.GetParentPath();
            published.hasControlFrame = true;
            published.controlFrame = evaluated;
        }
        GfMatrix4d placement(1.0);
        GfVec3d evaluatedScale(1.0);
        if (!_RigidGuideMatrix(frame, &placement, &evaluatedScale)) {
            continue;
        }
        _GuideInputs &inputs = _GuideInputsFor(call, controlPath);
        const UsdPrim &prim = inputs.prim;
        if (!inputs.controlReady || inputs.controlLive) {
            // The schema fallbacks, restated. Normally GetAttribute resolves
            // them for us, but a stage composed without the codeless schema
            // plugin registered has no fallback to find, and an empty shape
            // token names no shape at all -- so a rig would silently stop
            // drawing control guides rather than draw the documented default.
            inputs.shape = _tCircle;
            inputs.drawMode = _tWire;
            inputs.planeNormal = _tY;
            inputs.scale = GfVec3d(1.0, 1.0, 1.0);
            inputs.wireWidth = 0.05;
            inputs.offset = GfVec3d(0.0);
            inputs.orient = GfQuatf(1.0f);
            inputs.guideSource = SdfPath();
            inputs.customPoints.clear();
            inputs.customCounts.clear();
            bool live = false;
            if (prim) {
                if (UsdAttribute a = prim.GetAttribute(_tGuideShape)) {
                    a.Get(&inputs.shape, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a = prim.GetAttribute(_tGuideDrawMode)) {
                    a.Get(&inputs.drawMode, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a = prim.GetAttribute(_tGuidePlaneNormal)) {
                    a.Get(&inputs.planeNormal, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                const TfToken scaleAttrs[3] = {_tGuideScaleX, _tGuideScaleY,
                                               _tGuideScaleZ};
                for (int axis = 0; axis < 3; ++axis) {
                    if (UsdAttribute a = prim.GetAttribute(scaleAttrs[axis])) {
                        a.Get(&inputs.scale[axis], pose.time);
                        live = live || _GuideAttrIsLive(a);
                    }
                }
                if (UsdAttribute a = prim.GetAttribute(_tGuideWireWidth)) {
                    a.Get(&inputs.wireWidth, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a = prim.GetAttribute(_tGuideOffset)) {
                    a.Get(&inputs.offset, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a = prim.GetAttribute(
                        TfToken(SchemaAttrText::guideOrient))) {
                    a.Get(&inputs.orient, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (inputs.shape == TfToken(SchemaValueText::custom)) {
                    if (UsdAttribute a = prim.GetAttribute(
                            TfToken(SchemaAttrText::guidePoints))) {
                        a.Get(&inputs.customPoints, pose.time);
                        live = live || _GuideAttrIsLive(a);
                    }
                    if (UsdAttribute a = prim.GetAttribute(
                            TfToken(SchemaAttrText::guideCurveVertexCounts))) {
                        a.Get(&inputs.customCounts, pose.time);
                        live = live || _GuideAttrIsLive(a);
                    }
                }
            }
            if (prim) {
                SdfPathVector targets;
                prim.GetRelationship(TfToken(SchemaAttrText::guideSource))
                    .GetTargets(&targets);
                if (targets.size() == 1)
                    inputs.guideSource = targets.front();
            }
            inputs.controlLive = live;
            inputs.controlReady = true;
        }
        if (!inputs.guideSource.IsEmpty()) {
            const RigExecPointFrame *source = nullptr;
            const auto control = pose.controlFrames.find(inputs.guideSource);
            const auto joint = pose.jointFramesFinal.find(inputs.guideSource);
            if (control != pose.controlFrames.end())
                source = &control->second;
            else if (joint != pose.jointFramesFinal.end())
                source = &joint->second;
            if (!source ||
                !RigExecPointsToMatrix(identity.points, source->points,
                                       &evaluated) ||
                !_RigidGuideMatrix(*source, &placement, &evaluatedScale))
                continue;
        }
        TfToken shape = inputs.shape;
        TfToken drawMode = inputs.drawMode;
        GfVec3d authoredScale = inputs.scale;
        double wireWidth = inputs.wireWidth;
        // Unlike the scale, a non-positive width is NOT a reason to draw
        // nothing: it selects the hairline fallback, which is what wire
        // guides did before the width existed. Non-finite collapses to the
        // same thing rather than reaching the curves as a NaN.
        if (!std::isfinite(wireWidth) || wireWidth <= 0.0) {
            wireWidth = 0.0;
        }
        // A non-finite or non-positive AUTHORED multiplier on ANY axis draws
        // nothing,
        // mirroring the joint guide:radius rule: a flattened shape is
        // invisible from most angles and degenerate from the rest, so
        // publishing it would only cost the viewer geometry it cannot
        // meaningfully see.
        GfVec3d effectiveScale(1.0);
        bool scaleOk = true;
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(authoredScale[axis]) ||
                authoredScale[axis] <= 0.0) {
                scaleOk = false;
                break;
            }
            effectiveScale[axis] = authoredScale[axis] * evaluatedScale[axis];
            if (!std::isfinite(effectiveScale[axis]) ||
                effectiveScale[axis] <= 0.0) {
                scaleOk = false;
                break;
            }
        }
        if (!scaleOk) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[controlPath];
        published.assetRoot = _rigPath.GetParentPath();
        if (shape == TfToken(SchemaValueText::custom)) {
            size_t count = 0;
            bool valid = !inputs.customCounts.empty();
            for (int n : inputs.customCounts) {
                valid = valid && n >= 2;
                if (n > 0)
                    count += size_t(n);
            }
            valid = valid && count == inputs.customPoints.size();
            for (const GfVec3f &p : inputs.customPoints) {
                for (int axis = 0; axis < 3; ++axis)
                    valid = valid && std::isfinite(p[axis]);
            }
            if (!valid)
                continue;
            published.controlGuidePoints = inputs.customPoints;
            published.controlGuideCounts = inputs.customCounts;
        }
        published.hasControlGuide = true;
        // guide:offset moves the drawn shape in the control's local frame,
        // carried by the evaluated scale; the pivot stays where it is.
        GfMatrix4d guideFrame = placement;
        if (inputs.offset != GfVec3d(0.0) && std::isfinite(inputs.offset[0]) &&
            std::isfinite(inputs.offset[1]) &&
            std::isfinite(inputs.offset[2])) {
            const GfVec3d local(inputs.offset[0] * evaluatedScale[0],
                                inputs.offset[1] * evaluatedScale[1],
                                inputs.offset[2] * evaluatedScale[2]);
            guideFrame = GfMatrix4d(1.0).SetTranslate(local) * placement;
        }
        if (shape == TfToken(SchemaValueText::custom)) {
            // Custom shapes may be asymmetric. Preserve reflections and
            // affine axes instead of rebuilding them from axis magnitudes.
            const GfVec3d offset = std::isfinite(inputs.offset[0]) &&
                                           std::isfinite(inputs.offset[1]) &&
                                           std::isfinite(inputs.offset[2])
                                       ? inputs.offset
                                       : GfVec3d(0.0);
            guideFrame = GfMatrix4d(1.0).SetTranslate(offset) * evaluated;
            effectiveScale = authoredScale;
        }
        // Shape scale precedes local orientation, offset and full placement.
        const GfQuatd orient(inputs.orient);
        if (orient != GfQuatd(1.0) && std::isfinite(orient.GetReal()) &&
            orient.GetLength() > 1e-12) {
            GfMatrix4d rotate(1.0);
            rotate.SetRotate(GfRotation(orient.GetNormalized()));
            guideFrame = rotate * guideFrame;
        }
        published.controlGuideFrame = guideFrame;
        published.controlGuideShape = shape;
        published.controlGuideDrawMode = drawMode;
        published.controlGuidePlaneNormal = inputs.planeNormal;
        published.controlGuideScale = effectiveScale;
        published.controlGuideWireWidth = wireWidth;
        _ReadGuideStyleCached(inputs, pose, &published);
        // Static hidden helpers need no synthetic geometry. Keep live opacity
        // guides present so an animated/connected fade can update in place.
        published.hasControlGuide =
            published.guideOpacity > 0.0f || inputs.opacityLive;
    }

    // Pose readers and manipulators consume final joint frames wherever the
    // joint sits. Preserve full evaluated matrices before guide rigidization.
    const RigExecPointFrame identityFrame;
    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        if (!frame.IsValid() || frame.IsDegenerate()) {
            continue;
        }
        GfMatrix4d evaluated(1.0);
        if (RigExecPointsToMatrix(identityFrame.points, frame.points,
                                  &evaluated)) {
            auto &published = snapshot->prims[jointPath];
            published.assetRoot = _rigPath.GetParentPath();
            published.hasControlFrame = true;
            published.controlFrame = evaluated;
        }
    }
}

} // namespace rigExec
