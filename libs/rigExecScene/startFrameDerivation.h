#ifndef RIGEXEC_SCENE_START_FRAME_DERIVATION_H
#define RIGEXEC_SCENE_START_FRAME_DERIVATION_H
#include "sceneDescriptors.h"
#include <cstring>
#include <string>

namespace rigExec {
/// Property names the rule reads, built once at load, never per call.
inline const TfToken RigExecStartFrameRelName("rigExec:startFrame");
inline const TfToken RigExecStartFramePolicyName("rigExec:startFramePolicy");
inline const TfToken RigExecStartFrameJointsName("rigExec:joints");

/// What rigExec:startFramePolicy decided for one RigExecFkChain.
enum class RigExecStartFrameDerivation {
    None,                 ///< nothing derived: policy none, authored targets, or no joints
    Derived,              ///< `parent` named a provider
    UnknownPolicy,        ///< neither none nor parent; the chain solves absolute
    NoProviderAncestor,   ///< `parent` found no joint or control ancestor
    JointsSpanProviders,  ///< `parent`: the joints do not share that ancestor
};

/// The rigExec:startFramePolicy rule. Every evaluator calls this one function
/// at compile time, and the result is a declared start-frame input of the
/// compiled chain; nothing is written back to the stage.
///
/// With policy `parent` and no authored rigExec:startFrame target, the start
/// provider is the nearest namespace ancestor of the first joint, below
/// \p rigRoot, that is a RigExecJoint or RigExecControl, provided every joint
/// lies strictly beneath it. Ancestry is structural only (GetParentPath,
/// never a name).
///
/// \p primType(path, &type) returns whether a composed prim exists at path
/// and, if so, its type name. The rule reads nothing else, so a stage and a
/// captured scene give the same answer for the same composed facts.
template <class PrimType>
RigExecStartFrameDerivation RigExecDeriveStartFrame(
    const SdfPath &rigRoot, const TfToken &policy, bool authored,
    const SdfPathVector &joints, const PrimType &primType, SdfPath *provider)
{
    *provider = SdfPath();
    const char *text = policy.GetText();
    if (!text[0] || std::strcmp(text, "none") == 0) {
        return RigExecStartFrameDerivation::None;
    }
    if (std::strcmp(text, "parent") != 0) {
        return RigExecStartFrameDerivation::UnknownPolicy;
    }
    if (authored || joints.empty()) {
        // Authored targets always win; a jointless chain hangs from nothing.
        return RigExecStartFrameDerivation::None;
    }
    TfToken type;
    const SdfPath first = joints[0].GetPrimPath();
    SdfPath found;
    if (primType(first, &type)) {
        for (SdfPath a = first.GetParentPath();
             !a.IsEmpty() && !a.IsAbsoluteRootPath();
             a = a.GetParentPath()) {
            if (a == rigRoot) {
                break;
            }
            if (!primType(a, &type)) {
                continue;
            }
            const char *name = type.GetText();
            if (std::strcmp(name, "RigExecJoint") == 0 ||
                std::strcmp(name, "RigExecControl") == 0) {
                found = a;
                break;
            }
        }
    }
    if (found.IsEmpty()) {
        return RigExecStartFrameDerivation::NoProviderAncestor;
    }
    for (const SdfPath &joint : joints) {
        const SdfPath jointPrim = joint.GetPrimPath();
        if (jointPrim == found || !jointPrim.HasPrefix(found)) {
            return RigExecStartFrameDerivation::JointsSpanProviders;
        }
    }
    *provider = found;
    return RigExecStartFrameDerivation::Derived;
}

/// The rule over captured scene facts for the chain at \p chain: its policy at
/// the Default identity (a uniform token), its composed rigExec:startFrame and
/// rigExec:joints targets, and the captured prim types.
inline RigExecStartFrameDerivation RigExecDeriveSceneStartFrame(
    const RigExecSceneDescriptors &scene, const SdfPath &chain,
    SdfPath *provider)
{
    TfToken policy;
    const auto attr =
        scene.attributes.find(chain.AppendProperty(RigExecStartFramePolicyName));
    if (attr != scene.attributes.end() && !attr->second.inputs.empty()) {
        size_t identity = 0;
        for (size_t i = 0; i < scene.identities.size(); ++i) {
            if (scene.identities[i] == UsdTimeCode::Default()) {
                identity = i;
                break;
            }
        }
        if (identity < attr->second.inputs.size()) {
            const VtValue &raw = attr->second.inputs[identity].raw;
            if (raw.IsHolding<TfToken>()) {
                policy = raw.UncheckedGet<TfToken>();
            }
        }
    }
    const auto start =
        scene.relationships.find(chain.AppendProperty(RigExecStartFrameRelName));
    const bool authored =
        start != scene.relationships.end() && !start->second.fact.targets.empty();
    const auto joints =
        scene.relationships.find(chain.AppendProperty(RigExecStartFrameJointsName));
    const SdfPathVector noJoints;
    return RigExecDeriveStartFrame(
        scene.rigRoot, policy, authored,
        joints == scene.relationships.end() ? noJoints : joints->second.fact.targets,
        [&scene](const SdfPath &path, TfToken *type) {
            const auto node = scene.nodes.find(path);
            if (node == scene.nodes.end()) {
                return false;
            }
            *type = node->second.fact.type;
            return true;
        },
        provider);
}

/// The compile warning for a derivation the rule refused, or empty.
inline std::string RigExecStartFrameDerivationMessage(
    RigExecStartFrameDerivation derivation, const std::string &chain,
    const TfToken &policy)
{
    switch (derivation) {
    case RigExecStartFrameDerivation::UnknownPolicy:
        return chain + " has rigExec:startFramePolicy '" + policy.GetText() +
               "', expected 'none' or 'parent'; solving absolute.";
    case RigExecStartFrameDerivation::NoProviderAncestor:
        return chain + " has rigExec:startFramePolicy 'parent' but no "
               "RigExecJoint/RigExecControl ancestor; solving absolute.";
    case RigExecStartFrameDerivation::JointsSpanProviders:
        return chain + " has rigExec:startFramePolicy 'parent' but its "
               "joints span providers; solving absolute.";
    case RigExecStartFrameDerivation::None:
    case RigExecStartFrameDerivation::Derived:
        break;
    }
    return std::string();
}
}
#endif
