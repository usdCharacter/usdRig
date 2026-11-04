#ifndef RIGEXEC_GRAPH_SOLVER_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_SOLVER_GRAPH_BINDING_H
#include "solverSceneTranslation.h"
#include "sceneGraphTypedRead.h"
namespace rigExec {
struct RigExecBoundGraphInput {
    bool typed=false;
    RigExecGraphTypedRead traversal;
    RigExecValueId direct=UINT64_MAX;
};
using RigExecBoundGraphInputs=std::map<std::string,RigExecBoundGraphInput>;
bool RigExecBindGraphInputs(const std::map<std::string,RigExecSceneBoundInput> &,
    const std::map<std::string,RigExecSceneTypedRead> &,const SdfPath &reader,
    const RigExecSceneGraphBindingContext &,RigExecBoundGraphInputs *,
    std::vector<RigExecValueId> *reads,std::string *error=nullptr);
bool RigExecReadBoundGraphInput(const RigExecBoundGraphInputs &,const char *,
    const RigExecTypedValueStore &,VtValue *,std::string *error=nullptr);
struct RigExecSolverGraphBinding {
    RigExecSolverRecord record;
    RigExecBoundGraphInputs inputs;
    std::vector<RigExecValueId> controls,controlRests,jointRests,reads;
    std::vector<int> jointElements;
    std::vector<char> jointRestLive;
    RigExecValueId startRest=UINT64_MAX,rootRest=UINT64_MAX,midRest=UINT64_MAX,
        endRest=UINT64_MAX,spaceRestId=UINT64_MAX,ribbonRestPoints=UINT64_MAX;
    RigExecValueId start=UINT64_MAX,root=UINT64_MAX,mid=UINT64_MAX,end=UINT64_MAX,
        pole=UINT64_MAX,space=UINT64_MAX,blendA=UINT64_MAX,blendB=UINT64_MAX,output=UINT64_MAX;
    bool hasSpace=false,blendARequired=false,blendBRequired=false;
    RigExecSolverRest spaceRest{};
    /// Skeleton2d: one traversal per bone channel, rigExec:joints order,
    /// then one per space switch's active index with the values they read
    /// when unauthored.
    std::vector<RigExecGraphTypedRead> bone2dChannels;
    std::vector<RigExecGraphTypedRead> bone2dSelectors;
    std::vector<double> bone2dSelectorFallbacks;
    /// One traversal per joint group's rigExec:activeMember.
    std::vector<RigExecGraphTypedRead> jointGroupSelectors;
};
bool RigExecBindSolverGraph(const RigExecSceneSolverDescriptor &,
    const RigExecSceneGraphBindingContext &,RigExecValueId output,
    RigExecSolverGraphBinding *,std::string *error=nullptr);
bool RigExecRunSolverGraph(const RigExecSolverGraphBinding &,RigExecTypedValueStore *,
    RigExecSolverWorkspace *,std::string *error=nullptr);
}
#endif
