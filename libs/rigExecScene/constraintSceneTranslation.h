#ifndef RIGEXEC_GRAPH_CONSTRAINT_SCENE_TRANSLATION_H
#define RIGEXEC_GRAPH_CONSTRAINT_SCENE_TRANSLATION_H
#include "sceneCompileInputs.h"
#include <functional>
#include "sceneTypedReads.h"
#include "constraintProgram.h"
namespace rigExec {
/// RigExecIkFkMatchConstraint's relationships as authored and its uniform
/// settings. The scene program checks their cardinality as the compile does.
struct RigExecSceneIkFkMatchFacts {
    SdfPathVector ikControl,poleControl,rootControl,effector,fkControls,limbJoints,moves,space;
    double ikValue=1,scaleCalibration=0,upperLengthOffset=0,lowerLengthOffset=0;
    /// rigExec:stretchPolicy == "softDistance".
    bool stretchLimb=true;
};
struct RigExecSceneConstraintDescriptor {
    SdfPath path,space,worldUpObject,effector;
    SdfPathVector targets,sources,poleObjects,ikChain;
    std::map<std::string,RigExecSceneBoundInput> inputs;
    std::map<std::string,RigExecSceneTypedRead> typedReads;
    RigExecConstraintRecord record;
    RigExecEulerOrder order=RigExecEulerOrder::XYZ;
    bool aimVectorAuthored=false,preserveInputUp=true,poleModeObject=false,useAnimatedTs=false;
    bool blendShear=false,worldUpRotationOnly=false,radialBlend=false;
    TfToken worldUpType;
    /// RigExecHarmonicsConstraint's uniform window settings (harmonicsKernel.h).
    double harmonicsWaves=5,harmonicsWaveLength=5,harmonicsFrequency=1;
    bool harmonicsIgnoreFirstFrame=true,harmonicsMatchVelocity=true;
    RigExecSceneIkFkMatchFacts match;
    GfVec3d aimAxisFallback{1,0,0},sceneUp{0,1,0};
};
/// Translates schema reads into detached ordered relationship and typed input facts.
/// The graph compiler selects frame versions and target snapshots separately.
bool RigExecTranslateSceneConstraint(const RigExecSceneDescriptors &,const SdfPath &,
    const SdfPathVector &targets,RigExecSceneConstraintDescriptor *,std::string *error=nullptr);
/// Resolves parameter/table values without source access. Source frames,
/// effective weight and world-up/carry graph values are delivered by compiler.
bool RigExecResolveSceneConstraint(const RigExecSceneDescriptors &,
    RigExecSceneConstraintDescriptor *,UsdTimeCode,const std::map<SdfPath,VtValue> &delivered,
    RigExecConstraintInputs *,bool *enabled,float *defaultWeight,std::string *error=nullptr);
/// Shared parameter/table preparation from delivered current graph values.
bool RigExecRefreshConstraintParameters(RigExecSceneConstraintDescriptor *,
    const std::function<bool(const char *,VtValue *)> &,RigExecConstraintInputs *,
    bool *enabled,float *defaultWeight,std::string *error=nullptr);

}
#endif
