#ifndef RIGEXEC_GRAPH_POSE_SCENE_TRANSLATION_H
#define RIGEXEC_GRAPH_POSE_SCENE_TRANSLATION_H
#include "poseProgram.h"
#include "sceneCompileInputs.h"
#include "sceneTypedReads.h"
namespace rigExec {
struct RigExecSceneSpaceSwitchDescriptor {
    SdfPath path,target,space,activeAttribute;
    SdfPathVector sources;
    std::vector<TfToken> labels;
    bool tokenIndex=false;
    double activeFallback=0;
    RigExecSceneTypedRead active;
    RigExecSpaceSwitchRecord record;
};
struct RigExecScenePoseInterpolatorDescriptor {
    SdfPath path,driver,parent;
    /// Frame drivers after the first and their parents (empty where none),
    /// in rigExec:driver order; only a RigLogic solve has any.
    SdfPathVector extraDrivers,extraParents;
    SdfPathVector driverAttributes,poseWeights,disabledPoseWeights;
    RigExecSceneTypedRead enabled;
    /// A RigExecMlMover's inputs:mask; unbound for an interpolator.
    RigExecSceneTypedRead mask;
    bool ml=false;
    /// A RigExecAnimatedMaps node.
    bool maps=false;
    std::vector<RigExecSceneTypedRead> numericReads;
    /// Per numeric driver, whether it is a float property (ML movers and
    /// animated maps bind each with its own type; interpolators read
    /// doubles).
    std::vector<char> numericIsFloat;
    /// Animated maps: each map's constant primvar on each bound mesh
    /// (<mesh>.primvars:<name>) and the map it republishes, parallel.
    SdfPathVector primvarPaths;
    std::vector<size_t> primvarSources;
    RigExecPoseInterpolatorRecord record;
    std::vector<std::string> notes;
};
bool RigExecTranslateSceneSpaceSwitch(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecSceneSpaceSwitchDescriptor *,std::string *error=nullptr);
bool RigExecResolveSceneSpaceSwitch(const RigExecSceneDescriptors &,
    const RigExecSceneSpaceSwitchDescriptor &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,double *active,std::string *error=nullptr);
bool RigExecTranslateScenePoseInterpolator(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecScenePoseInterpolatorDescriptor *,std::string *error=nullptr);
bool RigExecResolveScenePoseInterpolator(const RigExecSceneDescriptors &,
    const RigExecScenePoseInterpolatorDescriptor &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,RigExecPoseInterpolatorInputs *,
    std::string *error=nullptr);
}
#endif
