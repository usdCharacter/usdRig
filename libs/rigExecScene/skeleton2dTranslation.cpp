#include "rigExecSchema/schemaNames.h"
#include "skeleton2dTranslation.h"
#include "poseSceneTranslation.h"
#include "sceneCompileInputs.h"
#include "rigExec/pathText.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (RigExecJoint) (RigExecBone2dAPI) (RigExecSpaceSwitch) (normal) (squashStretch) (uniform)
    ((rigExecJoints, SchemaAttrText::rigExecJoints))
    ((rigExecJointElements, SchemaAttrText::rigExecJointElements))
    ((rigExecTarget, SchemaAttrText::rigExecTarget))
    ((rigExecSpace, SchemaAttrText::rigExecSpace))
    ((restSpace, SchemaAttrText::restSpace))
    ((restTx, SchemaAttrText::restTx)) ((restTy, SchemaAttrText::restTy)) ((restTz, SchemaAttrText::restTz))
    ((restRx, SchemaAttrText::restRx)) ((restRy, SchemaAttrText::restRy)) ((restRz, SchemaAttrText::restRz))
    ((boneAngleLimits, SchemaAttrText::bone2dAngleLimits))
    ((boneMinAngle, SchemaAttrText::bone2dMinAngle))
    ((boneMaxAngle, SchemaAttrText::bone2dMaxAngle))
    ((boneIndependentAngle, SchemaAttrText::bone2dIndependentAngle))
    ((boneScalingMode, SchemaAttrText::bone2dScalingMode))
    ((boneSquashFactor, SchemaAttrText::bone2dSquashFactor))
    ((boneIgnoredByIk, SchemaAttrText::bone2dIgnoredByIk))
    ((boneMaxStretch, SchemaAttrText::bone2dMaxStretch))
    ((boneTargetCandidates, SchemaAttrText::bone2dTargetCandidates))
    ((boneAngleControl, SchemaAttrText::bone2dAngleControl))
    ((boneAngleControlFactor, SchemaAttrText::bone2dAngleControlFactor))
    ((bonePosControl, SchemaAttrText::bone2dPosControl))
    ((bonePosControlFactorX, SchemaAttrText::bone2dPosControlFactorX))
    ((bonePosControlFactorY, SchemaAttrText::bone2dPosControlFactorY))
    ((boneScaleControl, SchemaAttrText::bone2dScaleControl))
    ((boneScaleControlFactor, SchemaAttrText::bone2dScaleControlFactor))
);

namespace {

std::string
Text(const SdfPath &path)
{
    std::string text;
    if (!RigExecSpellPathText(path, &text))
        text = path.GetAsString();
    return text;
}

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

// Refuses \p property when it is time-sampled, a spline or may vary with
// time; \p consequence says what an edit to it does.
bool
RefuseAnimated(const RigExecSceneCompileInputs &source, const SdfPath &property,
               const char *consequence, std::string *error)
{
    const auto *attribute = source.Attribute(property);
    if (attribute && (!attribute->fact.sampleTimes.empty() ||
                      !attribute->fact.spline.IsEmpty() ||
                      attribute->fact.mightBeTimeVarying)) {
        return Fail(error, Text(property) +
                               " is a setup value and must not be animated (" +
                               consequence + ")");
    }
    return true;
}

// A structural attribute: its value at Default, refused when it is
// time-sampled or a spline (an edit to it recompiles; it cannot animate).
template <class T>
bool
Uniform(const RigExecSceneCompileInputs &source, const SdfPath &property,
        const T &fallback, T *value, std::string *error)
{
    *value = fallback;
    const auto *attribute = source.Attribute(property);
    if (!attribute)
        return true;
    if (!RefuseAnimated(source, property, "an edit recompiles the skeleton",
                        error)) {
        return false;
    }
    RigExecSceneBoundInput binding;
    VtValue boxed;
    if (!source.Bind(property, RigExecSceneReadRoute::Raw, &binding, error) ||
        !source.Read(binding, UsdTimeCode::Default(), &boxed, nullptr, error)) {
        return false;
    }
    if (boxed.IsHolding<T>())
        *value = boxed.UncheckedGet<T>();
    return true;
}

bool
HasApplied(const RigExecSceneNodeDescriptor &node, const TfToken &api)
{
    const auto &schemas = node.fact.appliedSchemas;
    return std::find(schemas.begin(), schemas.end(), api) != schemas.end();
}

bool
HasApi(const RigExecSceneNodeDescriptor &node)
{
    return HasApplied(node, _tokens->RigExecBone2dAPI);
}

// The nearest RigExecJoint namespace ancestor of \p joint below the rig
// root, or empty. Ancestry is structural only, never a name.
SdfPath
NearestJointAncestor(const RigExecSceneCompileInputs &source,
                     const SdfPath &rigRoot, const SdfPath &joint)
{
    for (SdfPath a = joint.GetParentPath();
         !a.IsEmpty() && a != SdfPath::AbsoluteRootPath() && a != rigRoot &&
         a.HasPrefix(rigRoot);
         a = a.GetParentPath()) {
        const auto *node = source.Node(a);
        if (node && node->fact.type == _tokens->RigExecJoint)
            return a;
    }
    return SdfPath();
}

// The first active RigExecJoint child of \p joint in namespace order, or
// empty: the bone's tip.
SdfPath
FirstJointChild(const RigExecSceneCompileInputs &source, const SdfPath &joint)
{
    const auto *node = source.Node(joint);
    if (!node)
        return SdfPath();
    for (const SdfPath &child : node->fact.children) {
        const auto *c = source.Node(child);
        if (c && c->fact.active && c->fact.type == _tokens->RigExecJoint)
            return child;
    }
    return SdfPath();
}

// A parent space while the bones are still in rigExec:joints order: a bone
// (>= 0, joints index), the base, or outside space e (-2 - e).
constexpr int32_t kBase = kRigExecBone2dSpaceBase;

} // namespace

bool
RigExecTranslateSceneSkeleton2d(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const SdfPath &start, const std::map<SdfPath, RigExecPointFrame> &rests,
    RigExecSceneSkeleton2dDescriptor *output, std::string *error)
{
    if (!output)
        return Fail(error, "null 2D skeleton descriptor");
    *output = RigExecSceneSkeleton2dDescriptor();
    const RigExecSceneCompileInputs source(scene);
    const auto *node = source.Node(path);
    if (!node || !node->fact.active ||
        node->fact.type != SchemaTypeName::RigExecSkeleton2d) {
        return Fail(error, "missing active 2D skeleton: " + Text(path));
    }
    const std::string who = Text(path);
    RigExecSceneSkeleton2dDescriptor result;
    result.path = path;
    result.joints = source.Targets(path.AppendProperty(_tokens->rigExecJoints));
    if (const auto *elements = source.Attribute(
            path.AppendProperty(_tokens->rigExecJointElements))) {
        if (elements->fact.hasAuthoredValue) {
            return Fail(
                error, who + ": rigExec:jointElements is not read by a 2D "
                             "skeleton; a bone's list position is its element");
        }
    }
    const size_t count = result.joints.size();
    std::map<SdfPath, size_t> index;
    for (size_t k = 0; k < count; ++k) {
        const SdfPath &joint = result.joints[k];
        if (!index.emplace(joint, k).second) {
            return Fail(error, who + ": rigExec:joints lists " + Text(joint) +
                                   " twice");
        }
    }
    // Under a bone: \p p is a bone or sits beneath one, so its frame moves
    // with the skeleton's own result.
    const auto underBone = [&](const SdfPath &p, SdfPath *bone) {
        for (const SdfPath &joint : result.joints) {
            if (p.HasPrefix(joint)) {
                *bone = joint;
                return true;
            }
        }
        return false;
    };

    // The plane: the start frame's rest frame (asset space without one).
    RigExecSkeleton2dPlane plane;
    if (!start.IsEmpty()) {
        SdfPath bone;
        if (underBone(start, &bone)) {
            return Fail(error,
                        who + ": rigExec:startFrame " + Text(start) +
                            " is or sits under the bone " + Text(bone) +
                            "; a skeleton cannot hang from its own bones");
        }
        const auto found = rests.find(start);
        if (found == rests.end() || !found->second.IsValid() ||
            !plane.Set(&found->second.points, &found->second.points)) {
            return Fail(error, who + ": rigExec:startFrame " + Text(start) +
                                   " has no usable rest frame");
        }
    }

    // Each bone: a RigExecJoint carrying RigExecBone2dAPI whose only rest
    // attribute is a static rest:space and whose composed rest is planar.
    std::vector<RigExecBone2dSetup> byJoint(count);
    std::vector<RigExecBone2dRest> planar(count);
    std::vector<int32_t> parentOf(count, -1), tipOf(count, -1);
    std::vector<std::vector<int32_t>> targetCandOf(count);
    std::vector<std::array<int32_t, 3>> controllerOf(count, {-1, -1, -1});
    for (size_t k = 0; k < count; ++k) {
        const SdfPath &joint = result.joints[k];
        const std::string bone = Text(joint);
        const auto *jointNode = source.Node(joint);
        if (!jointNode || !jointNode->fact.active ||
            jointNode->fact.type != _tokens->RigExecJoint ||
            !HasApi(*jointNode)) {
            return Fail(error,
                        who + ": rigExec:joints target " + bone +
                            " is not a RigExecJoint with RigExecBone2dAPI");
        }
        for (const TfToken *name :
             {&_tokens->restTx, &_tokens->restTy, &_tokens->restTz,
              &_tokens->restRx, &_tokens->restRy, &_tokens->restRz}) {
            const auto *attribute =
                source.Attribute(joint.AppendProperty(*name));
            if (attribute && (attribute->fact.hasAuthoredValue ||
                              attribute->fact.hasAuthoredConnections)) {
                return Fail(error,
                            who + ": bone " + bone + " authors " +
                                name->GetString() +
                                "; a 2D bone's setup is its rest:space alone");
            }
        }
        if (const auto *space =
                source.Attribute(joint.AppendProperty(_tokens->restSpace))) {
            if (!space->fact.sampleTimes.empty() ||
                !space->fact.spline.IsEmpty() ||
                space->fact.mightBeTimeVarying ||
                space->fact.hasAuthoredConnections) {
                return Fail(error,
                            who + ": bone " + bone +
                                " has an animated or connected "
                                "rest:space; the setup frame is not animated");
            }
        }
        const auto rest = rests.find(joint);
        if (rest == rests.end() || !rest->second.IsValid() ||
            !plane.Rest(rest->second.points, &planar[k])) {
            return Fail(
                error,
                who + ": bone " + bone +
                    " has a rest that is not a "
                    "rotation about +Z plus a translation in the "
                    "skeleton's plane (2D bones lie in the start frame's "
                    "XY plane, the asset space's without one)");
        }
        // The parent: the nearest RigExecJoint ancestor when it is a bone of
        // this skeleton; otherwise the bone is a root.
        const SdfPath ancestor =
            NearestJointAncestor(source, scene.rigRoot, joint);
        if (!ancestor.IsEmpty()) {
            const auto found = index.find(ancestor);
            if (found != index.end())
                parentOf[k] = int32_t(found->second);
        }
        const auto property = [&](const TfToken &name) {
            return joint.AppendProperty(name);
        };
        // Uniform settings: structural, digested.
        RigExecBone2dSetup &b = byJoint[k];
        bool limits = false, independent = false, ignored = false;
        TfToken scaling;
        if (!Uniform(source, property(_tokens->boneAngleLimits), false, &limits,
                     error) ||
            !Uniform(source, property(_tokens->boneMinAngle), 0.0, &b.minAngle,
                     error) ||
            !Uniform(source, property(_tokens->boneMaxAngle), 0.0, &b.maxAngle,
                     error) ||
            !Uniform(source, property(_tokens->boneIndependentAngle), false,
                     &independent, error) ||
            !Uniform(source, property(_tokens->boneScalingMode),
                     _tokens->normal, &scaling, error) ||
            !Uniform(source, property(_tokens->boneSquashFactor), 1.0,
                     &b.squashFactor, error) ||
            !Uniform(source, property(_tokens->boneIgnoredByIk), false,
                     &ignored, error) ||
            !Uniform(source, property(_tokens->boneMaxStretch), 1.0,
                     &b.maxStretch, error)) {
            return false;
        }
        b.flags = uint8_t((independent ? kRigExecBone2dIndependentAngle : 0) |
                          (limits ? kRigExecBone2dAngleLimits : 0) |
                          (ignored ? kRigExecBone2dIgnoredByIk : 0));
        // The tip: the first RigExecJoint child. A child bone's base is
        // read from the rests every solve; an End joint's offset in this
        // bone's setup frame is compiled (an edit to it recompiles).
        const SdfPath tip = FirstJointChild(source, joint);
        if (!tip.IsEmpty()) {
            b.flags |= kRigExecBone2dTip;
            const auto tipBone = index.find(tip);
            if (tipBone != index.end()) {
                tipOf[k] = int32_t(tipBone->second);
            } else {
                const auto tipRest = rests.find(tip);
                if (tipRest == rests.end() || !tipRest->second.IsValid()) {
                    return Fail(error, who + ": bone " + bone +
                                           " has the tip joint " + Text(tip) +
                                           " with no rest frame");
                }
                double at[3];
                plane.PointIn(tipRest->second.points[0], at);
                const double a =
                    planar[k].angle * (3.14159265358979323846 / 180.0);
                const double cs = std::cos(a), sn = std::sin(a);
                const double dx = at[0] - planar[k].x, dy = at[1] - planar[k].y;
                b.tipX = dx * cs + dy * sn;
                b.tipY = -dx * sn + dy * cs;
                if (!std::isfinite(b.tipX) || !std::isfinite(b.tipY)) {
                    return Fail(error, who + ": bone " + bone +
                                           " has a non-finite tip");
                }
            }
        }
        if (scaling == _tokens->squashStretch) {
            b.scaling = RigExecBone2dScaling::SquashStretch;
        } else if (scaling == _tokens->uniform) {
            b.scaling = RigExecBone2dScaling::Uniform;
        } else if (scaling == _tokens->normal || scaling.IsEmpty()) {
            b.scaling = RigExecBone2dScaling::Normal;
        } else {
            return Fail(error, who + ": bone " + bone +
                                   " has an unknown bone2d:scalingMode '" +
                                   scaling.GetString() + "'");
        }
        if (!std::isfinite(b.minAngle) || !std::isfinite(b.maxAngle) ||
            !std::isfinite(b.squashFactor) || !std::isfinite(b.maxStretch)) {
            return Fail(error,
                        who + ": bone " + bone + " has a non-finite setting");
        }
        // Squash factor is clamped to [0.01, 100].
        if (b.squashFactor < 0.01 || b.squashFactor > 100) {
            result.notes.push_back(
                who + ": bone " + bone + " bone2d:squashFactor " +
                std::to_string(b.squashFactor) + " is clamped to [0.01, 100]");
            b.squashFactor = std::min(std::max(b.squashFactor, 0.01), 100.0);
        }
        // maxStretch is clamped to >= 1.
        b.maxStretch = std::max(1.0, b.maxStretch);
        // Target candidates: authored IK goals, all bones but itself.
        const SdfPathVector targetOptions =
            source.Targets(joint.AppendProperty(_tokens->boneTargetCandidates));
        for (const SdfPath &option : targetOptions) {
            const auto found = index.find(option);
            if (found == index.end()) {
                return Fail(error,
                            who + ": bone " + bone +
                                " has bone2d:targetCandidates " + Text(option) +
                                ", which is not a bone of this skeleton");
            }
            if (found->second == k) {
                return Fail(error, who + ": bone " + bone +
                                       " targets itself in " +
                                       SchemaAttrText::bone2dTargetCandidates);
            }
            targetCandOf[k].push_back(int32_t(found->second));
        }
        // Control bones: at most one controller per kind, never itself.
        const auto controlOf = [&](const TfToken &name, int32_t *controller) {
            const SdfPathVector controllers =
                source.Targets(joint.AppendProperty(name));
            if (controllers.size() > 1) {
                return Fail(error, who + ": bone " + bone + " names " +
                                       std::to_string(controllers.size()) +
                                       " " + name.GetString() +
                                       " targets; at most one");
            }
            if (controllers.empty()) {
                return true;
            }
            const auto found = index.find(controllers[0]);
            if (found == index.end()) {
                return Fail(error,
                            who + ": bone " + bone + " has " +
                                name.GetString() + " " + Text(controllers[0]) +
                                ", which is not a bone of this skeleton");
            }
            if (found->second == k) {
                return Fail(error, who + ": bone " + bone +
                                       " controls itself with " +
                                       name.GetString());
            }
            *controller = int32_t(found->second);
            return true;
        };
        int32_t angleController = -1, posController = -1, scaleController = -1;
        if (!controlOf(_tokens->boneAngleControl, &angleController) ||
            !controlOf(_tokens->bonePosControl, &posController) ||
            !controlOf(_tokens->boneScaleControl, &scaleController) ||
            !Uniform(source, property(_tokens->boneAngleControlFactor), 0.0,
                     &b.angleFactor, error) ||
            !Uniform(source, property(_tokens->bonePosControlFactorX), 0.0,
                     &b.posFactorX, error) ||
            !Uniform(source, property(_tokens->bonePosControlFactorY), 0.0,
                     &b.posFactorY, error) ||
            !Uniform(source, property(_tokens->boneScaleControlFactor), 0.0,
                     &b.scaleFactor, error)) {
            return false;
        }
        if (!std::isfinite(b.angleFactor) || !std::isfinite(b.posFactorX) ||
            !std::isfinite(b.posFactorY) || !std::isfinite(b.scaleFactor)) {
            return Fail(error, who + ": bone " + bone +
                                   " has a non-finite control factor");
        }
        controllerOf[k][0] = angleController;
        controllerOf[k][1] = posController;
        controllerOf[k][2] = scaleController;
    }

    // Space switches on the bones: the skeleton reads them, and they never
    // run as pose steps of their own. Their sources are the bone's parent
    // spaces in order: a bone, the base (a prim that is not a frame
    // provider, the RigExecRoot), or an outside provider read per frame.
    std::vector<SdfPath> switchOf(count);
    for (const auto &[nodePath, switchNode] : scene.nodes) {
        if (!switchNode.fact.active ||
            switchNode.fact.type != _tokens->RigExecSpaceSwitch) {
            continue;
        }
        const SdfPathVector targets =
            source.Targets(nodePath.AppendProperty(_tokens->rigExecTarget));
        if (targets.size() != 1)
            continue;
        const auto bone = index.find(targets[0]);
        if (bone == index.end())
            continue;
        if (!switchOf[bone->second].IsEmpty()) {
            return Fail(error,
                        Text(targets[0]) +
                            " is the target of more than one space switch");
        }
        switchOf[bone->second] = nodePath;
    }
    std::vector<std::vector<int32_t>> spacesOf(count);
    std::vector<RigExecSceneTypedRead> activeOf(count);
    std::vector<double> fallbackOf(count, 0.0);
    for (size_t k = 0; k < count; ++k) {
        if (switchOf[k].IsEmpty())
            continue;
        const std::string bone = Text(result.joints[k]);
        const std::string name = Text(switchOf[k]);
        RigExecSceneSpaceSwitchDescriptor sw;
        if (!RigExecTranslateSceneSpaceSwitch(scene, switchOf[k], &sw, error)) {
            return false;
        }
        const std::string head =
            who + ": " + name + " switches the 2D bone " + bone;
        if (!source.Targets(switchOf[k].AppendProperty(_tokens->rigExecSpace))
                 .empty()) {
            return Fail(
                error, head +
                           " and authors rigExec:space; the skeleton reads its "
                           "sources in its own plane");
        }
        for (const int filter : sw.record.filters) {
            if (filter != 0) {
                return Fail(
                    error,
                    head + " and authors rigExec:rotationFilters; a 2D bone "
                           "takes each space whole");
            }
        }
        for (size_t axis = 0; axis < 3; ++axis) {
            if (!sw.record.affectTranslation[axis] ||
                !sw.record.affectRotation[axis] ||
                !sw.record.affectScale[axis]) {
                return Fail(
                    error, head +
                               " and clears an axis mask; a 2D bone takes each "
                               "space whole");
            }
        }
        if (sw.tokenIndex) {
            return Fail(
                error,
                head + " and selects its space by label; the skeleton reads "
                       "a numeric index");
        }
        for (const SdfPath &space : sw.sources) {
            if (space.IsEmpty()) {
                spacesOf[k].push_back(kBase);
                continue;
            }
            const auto found = index.find(space);
            if (found != index.end()) {
                if (found->second == k) {
                    return Fail(
                        error,
                        head + " and names the bone as its own parent space "
                               "(a cycle)");
                }
                spacesOf[k].push_back(int32_t(found->second));
                continue;
            }
            SdfPath under;
            if (underBone(space, &under)) {
                return Fail(error, head + " and reads " + Text(space) +
                                       ", which sits under "
                                       "the bone " +
                                       Text(under) +
                                       " and so moves with the "
                                       "skeleton's own result (a cycle)");
            }
            const auto spaceRest = rests.find(space);
            RigExecBone2dRest placement;
            if (spaceRest == rests.end() || !spaceRest->second.IsValid() ||
                !plane.Placement(spaceRest->second.points, false, &placement)) {
                return Fail(error,
                            head + " and reads " + Text(space) +
                                ", whose rest is not "
                                "a rotation about +Z in the skeleton's plane");
            }
            const auto at = std::find(result.externalSources.begin(),
                                      result.externalSources.end(), space);
            const size_t e = size_t(at - result.externalSources.begin());
            if (at == result.externalSources.end())
                result.externalSources.push_back(space);
            spacesOf[k].push_back(RigExecBone2dSpaceExternal(e));
        }
        activeOf[k] = sw.active;
        fallbackOf[k] = sw.activeFallback;
    }

    // A lone pin has nothing to turn toward a target.
    for (size_t k = 0; k < count; ++k) {
        if ((byJoint[k].flags & kRigExecBone2dTip) || targetCandOf[k].empty() ||
            parentOf[k] >= 0) {
            continue;
        }
        const bool boneSpace =
            std::any_of(spacesOf[k].begin(), spacesOf[k].end(),
                        [](int32_t s) { return s >= 0; });
        if (!boneSpace) {
            return Fail(
                error,
                who + ": bone " + Text(result.joints[k]) +
                    " is a pin (no joint "
                    "child) with an IK target and no parent bone; a lone pin "
                    "has nothing to turn");
        }
    }

    // Solve order over the union of the namespace parents and every switch
    // source: parents first, otherwise rigExec:joints order (a stable
    // topological sort), so a reorder of independent bones changes nothing.
    // A cyclic union is refused: no static order could serve every frame.
    std::vector<int32_t> solveIndex(count, -1);
    while (result.solveOrder.size() < count) {
        const size_t before = result.solveOrder.size();
        for (size_t k = 0; k < count; ++k) {
            if (solveIndex[k] >= 0)
                continue;
            const int32_t p = parentOf[k];
            if (p >= 0 && solveIndex[size_t(p)] < 0)
                continue;
            bool waiting = false;
            for (const int32_t space : spacesOf[k]) {
                if (space >= 0 && solveIndex[size_t(space)] < 0)
                    waiting = true;
            }
            if (waiting)
                continue;
            solveIndex[k] = int32_t(result.solveOrder.size());
            result.solveOrder.push_back(uint32_t(k));
        }
        if (result.solveOrder.size() == before) {
            for (size_t k = 0; k < count; ++k) {
                if (solveIndex[k] < 0) {
                    return Fail(error,
                                who +
                                    ": the namespace parents and space-switch "
                                    "sources form a cycle through " +
                                    Text(result.joints[k]));
                }
            }
        }
    }
    const auto solved = [&](int32_t space) {
        return space >= 0 ? solveIndex[size_t(space)] : space;
    };
    result.setup.resize(count);
    for (size_t e = 0; e < count; ++e) {
        const size_t k = result.solveOrder[e];
        result.setup[e] = byJoint[k];
        RigExecBone2dSetup &setup = result.setup[e];
        setup.parent = parentOf[k] < 0 ? -1 : solveIndex[size_t(parentOf[k])];
        setup.tipBone = tipOf[k] < 0 ? -1 : solveIndex[size_t(tipOf[k])];
        if (!spacesOf[k].empty()) {
            setup.spaceBegin = uint32_t(result.spaceSources.size());
            setup.spaceCount = uint32_t(spacesOf[k].size());
            for (const int32_t space : spacesOf[k]) {
                result.spaceSources.push_back(solved(space));
            }
        }
        if (!targetCandOf[k].empty()) {
            setup.targetBegin = uint32_t(result.targetCandidates.size());
            setup.targetCount = uint32_t(targetCandOf[k].size());
            for (const int32_t option : targetCandOf[k]) {
                result.targetCandidates.push_back(solveIndex[size_t(option)]);
            }
        }
        setup.angleController = controllerOf[k][0] < 0
                                    ? -1
                                    : solveIndex[size_t(controllerOf[k][0])];
        setup.posController = controllerOf[k][1] < 0
                                  ? -1
                                  : solveIndex[size_t(controllerOf[k][1])];
        setup.scaleController = controllerOf[k][2] < 0
                                    ? -1
                                    : solveIndex[size_t(controllerOf[k][2])];
    }

    // One active-index selector per switch, in solve order.
    for (size_t e = 0; e < count; ++e) {
        const size_t k = result.solveOrder[e];
        if (switchOf[k].IsEmpty())
            continue;
        result.setup[e].spaceSelector = int32_t(result.selectorReads.size());
        result.selectorReads.push_back(std::move(activeOf[k]));
        result.selectorFallbacks.push_back(fallbackOf[k]);
        result.switches.push_back(switchOf[k]);
    }

    // Value reads: thirteen channels per bone, rigExec:joints order.
    result.channelReads.resize(count * kRigExecBone2dChannelCount);
    for (size_t k = 0; k < count; ++k) {
        for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
            const SdfPath property =
                result.joints[k].AppendProperty(RigExecBone2dChannelNames[c]);
            if (!RigExecBindSceneTypedRead(
                    scene, property,
                    RigExecBone2dChannelIsBool(c) ? SdfValueTypeNames->Bool
                                                  : SdfValueTypeNames->Double,
                    &result.channelReads[k * kRigExecBone2dChannelCount + c],
                    error)) {
                return false;
            }
        }
    }
    *output = std::move(result);
    return true;
}

} // namespace rigExec
