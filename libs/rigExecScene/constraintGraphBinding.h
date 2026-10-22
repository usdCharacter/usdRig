#ifndef RIGEXEC_GRAPH_CONSTRAINT_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_CONSTRAINT_GRAPH_BINDING_H
#include "constraintSceneTranslation.h"
#include "solverGraphBinding.h"
namespace rigExec {
struct RigExecWindowRun;
/// Exact entering/carry/weight/rest versions selected by the central compiler.
struct RigExecConstraintGraphTarget {
    RigExecValueId incoming=UINT64_MAX,carry=UINT64_MAX,weight=UINT64_MAX;
    std::vector<RigExecValueId> chainRests,chainOutputs;
    std::vector<char> restLive;
    RigExecValueId output=UINT64_MAX;
};
struct RigExecConstraintGraphBinding {
    RigExecSceneConstraintDescriptor structural;
    RigExecBoundGraphInputs inputs;
    RigExecConstraintGraphTarget target;
    std::vector<RigExecValueId> sources,chain,poles,reads;
    RigExecValueId effector=UINT64_MAX,worldUp=UINT64_MAX;
    /// Earlier frames a windowed constraint (harmonics) reads; 0 otherwise.
    uint32_t windowFrames=0;
};
bool RigExecBindConstraintGraph(const RigExecSceneDescriptors &,
    const RigExecSceneConstraintDescriptor &,const RigExecSceneGraphBindingContext &,
    const RigExecConstraintGraphTarget &,RigExecConstraintGraphBinding *,std::string *error=nullptr);
/// \p window is the run of a windowed constraint (rigExecGraph/windowHistory.h)
/// and null for every other one.
bool RigExecRunConstraintGraph(const RigExecConstraintGraphBinding &,
    RigExecTypedValueStore *,std::string *error=nullptr,RigExecWindowRun *window=nullptr);
}
#endif
