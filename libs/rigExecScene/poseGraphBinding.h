#ifndef RIGEXEC_GRAPH_POSE_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_POSE_GRAPH_BINDING_H
#include "poseSceneTranslation.h"
#include "sceneGraphTypedRead.h"
#include <array>
namespace rigExec {
struct RigExecBoundSpaceSwitch {
    RigExecSpaceSwitchRecord record;
    RigExecSpaceSwitchInputs inputs;
    RigExecGraphTypedRead active;
    std::vector<TfToken> labels;
    bool tokenIndex=false;
    double activeFallback=0;
    RigExecValueId avars=UINT64_MAX,posedDefault=UINT64_MAX,parentDefault=UINT64_MAX;
    RigExecValueId parentPosed=UINT64_MAX,parentExpression=UINT64_MAX;
    RigExecValueId spaceDefault=UINT64_MAX,spacePosed=UINT64_MAX;
    std::vector<RigExecValueId> sourceDefault,sourcePosed,reads;
    std::string unavailable;
};
struct RigExecBoundPoseInterpolator {
    RigExecPoseInterpolatorRecord record;
    RigExecPoseInterpolatorInputs inputs;
    RigExecGraphTypedRead enabled;
    /// A RigExecMlMover's inputs:mask; unbound otherwise.
    RigExecGraphTypedRead mask;
    bool ml=false;
    std::vector<RigExecGraphTypedRead> numeric;
    RigExecValueId driverFinal=UINT64_MAX,driverRest=UINT64_MAX;
    RigExecValueId parentFinal=UINT64_MAX,parentRest=UINT64_MAX;
    /// A RigLogic solve's further drivers: final, rest, parent final and
    /// parent rest per driver (UINT64_MAX for a missing parent).
    std::vector<std::array<RigExecValueId,4>> extraDrivers;
    std::vector<RigExecValueId> reads,weights,disabledWeights;
    /// Animated maps' mesh primvars and the weight each republishes.
    std::vector<RigExecValueId> primvars;
    std::vector<size_t> primvarSources;
    std::vector<double> scratch;
    std::string unusableRotation,unusableTranslation,countMismatch;
};
bool RigExecBindSceneSpaceSwitch(const RigExecSceneDescriptors &,
    const RigExecSceneSpaceSwitchDescriptor &,const RigExecSceneGraphBindingContext &,
    RigExecBoundSpaceSwitch *,std::string *error=nullptr,
    RigExecValueId parentContext=UINT64_MAX,RigExecValueId spaceContext=UINT64_MAX);
bool RigExecBindScenePoseInterpolator(const RigExecScenePoseInterpolatorDescriptor &,
    const RigExecSceneGraphBindingContext &,const std::vector<RigExecValueId> &weights,
    const std::vector<RigExecValueId> &disabledWeights,RigExecBoundPoseInterpolator *,
    std::string *error=nullptr,const std::vector<RigExecValueId> &primvars={});
/// Runs only delivered current typed values; no source paths are resolved here.
bool RigExecRunBoundSpaceSwitch(RigExecBoundSpaceSwitch *,RigExecTypedValueStore *,
    RigExecValueId output,std::string *error=nullptr);
bool RigExecRunBoundPoseInterpolator(RigExecBoundPoseInterpolator *,
    RigExecTypedValueStore *,std::string *error=nullptr);
}
#endif
