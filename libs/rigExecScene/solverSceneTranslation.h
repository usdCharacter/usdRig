#ifndef RIGEXEC_GRAPH_SOLVER_SCENE_TRANSLATION_H
#define RIGEXEC_GRAPH_SOLVER_SCENE_TRANSLATION_H
#include "sceneCompileInputs.h"
#include <functional>
#include "sceneTypedReads.h"
#include "solverProgram.h"
namespace rigExec {
struct RigExecSceneSolverFrameBinding {
    TfToken phase;
    std::map<SdfPath, SdfPath> keys;
};
struct RigExecSceneSolverDescriptor {
    SdfPath path;
    RigExecSolverRecord record;
    SdfPathVector controls, joints;
    std::vector<int> jointElements;
    SdfPath start, root, mid, end, pole, space, blendA, blendB, ribbonPoints;
    TfToken ribbonReadPhase;
    RigExecSolverRest spaceRest{};
    /// Typed input graph bindings. Keys are the exact schema property names.
    std::map<std::string, RigExecSceneBoundInput> inputs;
    std::map<std::string, RigExecSceneTypedRead> typedReads;
    std::map<std::string, RigExecSceneSolverFrameBinding> frameBindings;
    /// Skeleton2d: kRigExecBone2dChannelCount typed reads per bone, in
    /// rigExec:joints order (RigExecBone2dChannelNames order within a bone),
    /// and one active-index read per space switch the skeleton reads, each
    /// with its unauthored value. Its outside parent spaces ride on
    /// `controls`, read like FK controls.
    std::vector<RigExecSceneTypedRead> bone2dChannelReads;
    std::vector<RigExecSceneTypedRead> bone2dSelectorReads;
    std::vector<double> bone2dSelectorFallbacks;
    /// FkChain and Skeleton2d: one rigExec:activeMember read per joint
    /// group (rigExec:jointGroups order); unauthored reads 0.
    std::vector<RigExecSceneTypedRead> jointGroupSelectorReads;
};
/// Translate structural solver facts using already compiled provider rest values.
/// The caller binds frame versions and scalar producer IDs from these paths.
/// Missing required providers yield the production empty aggregate guard.
bool RigExecTranslateSceneSolver(
    const RigExecSceneDescriptors &, const SdfPath &,
    const std::map<SdfPath, RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *, std::string *error = nullptr);

} // namespace rigExec
#endif
