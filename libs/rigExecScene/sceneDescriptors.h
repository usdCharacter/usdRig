#ifndef RIGEXEC_GRAPH_SCENE_DESCRIPTORS_H
#define RIGEXEC_GRAPH_SCENE_DESCRIPTORS_H
#include "sceneAccess.h"
#include "rigExecGraph/opGraph.h"
#include <map>

namespace rigExec {
/// Read phases named by scene translation; built once at load, never per read.
inline const TfToken RigExecSceneBasePhase("base");
inline const TfToken RigExecScenePrecedingPhase("preceding");
inline const TfToken RigExecSceneFinalPhase("final");
enum class RigExecSceneDomain {
    Data, Provider, Solver, PropertyMover, GeometryMover, Weight,
    Constraint, SpaceSwitch, PoseInterpolator, UnknownRigExec, AutoClavicle
};
enum class RigExecSceneInputState { Raw, Connected, Computed, Empty, Unavailable, Cycle };
struct RigExecSceneInput {
    VtValue raw, resolved;
    bool rawBlocked = false, resolvedBlocked = false;
    RigExecSceneInputState state = RigExecSceneInputState::Unavailable;
    SdfPath source;
    SdfPathVector hops;
};
struct RigExecSceneAttributeDescriptor {
    RigExecSceneAttribute fact;
    TfToken readPhase = RigExecSceneBasePhase;
    std::vector<RigExecSceneInput> inputs;
};
struct RigExecSceneRelationshipDescriptor {
    RigExecSceneRelationship fact;
    TfToken readPhase = RigExecSceneBasePhase;
    /// Original forwarded order and cardinality, including nonexistent targets.
    SdfPathVector forwardedTargets;
    std::vector<char> targetExists;
};
struct RigExecSceneNodeDescriptor {
    RigExecScenePrim fact;
    RigExecSceneDomain domain = RigExecSceneDomain::Data;
    bool transformProvider = false;
    bool pointBased = false;
    SdfPathVector attributes, relationships;
    /// Reverse-sibling post-order within the selected rig, -1 outside it.
    int stackOrdinal = -1;
};
struct RigExecSceneApplicationDescriptor {
    SdfPath owner, target;
    RigExecSceneDomain domain;
    int stackOrdinal = -1;
    /// For point-based geometry targets, the canonical .points property.
    SdfPath canonicalTarget;
    size_t targetIndex = 0;
};
struct RigExecSceneJointBindingDescriptor {
    SdfPath solver, joint;
    size_t element = 0;
};
/// Detached compiler input; source access ends when capture returns.
struct RigExecSceneDescriptors {
    SdfPath rigRoot;
    SdfPathVector compilerTargets;
    std::vector<UsdTimeCode> identities;
    std::map<SdfPath, RigExecSceneNodeDescriptor> nodes;
    std::map<SdfPath, RigExecSceneAttributeDescriptor> attributes;
    std::map<SdfPath, RigExecSceneRelationshipDescriptor> relationships;
    std::vector<RigExecSceneApplicationDescriptor> applications;
    std::vector<RigExecSceneJointBindingDescriptor> jointBindings;
    double timeCodesPerSecond = 24, framesPerSecond = 24;
    TfToken interpolation;
    TfToken upAxis;
    /// Primvar property names translation publishes to ("primvars:" and an
    /// authored or prim name). No captured property carries them, so capture
    /// interns them on the owner thread and translation builds no token.
    std::map<std::string, TfToken> primvarNames;
};
/// The interned primvar property name for "primvars:" + \p name, or null when
/// capture did not see it.
inline const TfToken *RigExecFindScenePrimvarName(const RigExecSceneDescriptors &scene,
    const std::string &name)
{
    const auto found = scene.primvarNames.find(name);
    return found == scene.primvarNames.end() ? nullptr : &found->second;
}
bool RigExecCaptureSceneDescriptors(const RigExecSceneAccess &scene,
    const SdfPath &rigRoot, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error = nullptr);

/// Copy every detached fact while selecting only exact captured identities.
/// Missing identities or incomplete input rows fail without changing output.
bool RigExecSelectSceneDescriptorIdentities(const RigExecSceneDescriptors &scene,
    const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error = nullptr);

/// Typed production kernel translation supplies operation/value identities. This
/// boundary never substitutes raw input for a computed provider expression.
using RigExecSceneKernelTranslation = std::function<bool(
    const RigExecSceneDescriptors &, std::vector<RigExecOpDescriptor> *,
    std::vector<RigExecValueId> *, std::string *)>;
bool RigExecCompileSceneDescriptors(const RigExecSceneDescriptors &scene,
    const RigExecSceneKernelTranslation &translate, RigExecCyclePolicy cyclePolicy,
    RigExecCompiledGraph *graph, std::string *error = nullptr);
}
#endif
