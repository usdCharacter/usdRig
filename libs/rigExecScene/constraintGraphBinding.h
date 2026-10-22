#ifndef RIGEXEC_GRAPH_CONSTRAINT_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_CONSTRAINT_GRAPH_BINDING_H
#include "constraintSceneTranslation.h"
#include "solverGraphBinding.h"
#include "poseGraphBinding.h"
#include <array>
#include <memory>
namespace rigExec {
struct RigExecWindowRun;
/// RigExecIkFkMatchConstraint (bakedIkFkMatch.cpp) over typed values. It
/// revises no frame a reader sees: it publishes outputs:ikValues and
/// outputs:fkValues, and the presented frames of its five controls and of
/// every control riding one of them. No operation reads a presented frame.
struct RigExecIkFkMatchGraphBinding {
    /// Joints 0-2, IK root 3, effector 4, IK control 5, pole 6, FK controls 7-9.
    std::array<RigExecValueId,10> sources;
    /// Rest frames: the joints at 0-2, then the five controls at 3-7.
    std::array<RigExecValueId,8> rests;
    /// Per control, its avar inputs in RigExecProviderReadAvarMatrix order:
    /// unit scale, t3, s3, r3, spin, order, sign.
    std::array<std::array<RigExecValueId,13>,5> channels;
    std::array<char,5> scaleAvars{{1,1,1,1,1}};
    /// rigExec:space's rest frame; its posed matrix is the target's carry.
    RigExecValueId spaceRest=UINT64_MAX;
    /// The pole's own space switch, re-run with the IK control moved, and per
    /// switch source whether it rides the IK control. Without one, whether
    /// the pole's namespace parent rides it.
    bool poleSwitch=false;
    RigExecBoundSpaceSwitch poleSwitchBinding;
    std::vector<char> poleSourceRides;
    bool poleParentRides=false;
    /// The pole's posed:space: authored, it is the pose and no switch re-runs.
    RigExecValueId polePosedSpace=UINT64_MAX;
    double ikValue=1,scaleCalibration=0,upperLengthOffset=0,lowerLengthOffset=0;
    bool stretchLimb=true;
    /// Each control's published frame before this presentation, and its
    /// presented frame.
    std::array<RigExecValueId,5> published,presented;
    /// Every other control under one of the five, presented carried by the
    /// revision of the nearest of them above it.
    struct Rider { RigExecValueId published=UINT64_MAX,presented=UINT64_MAX;int rides=0; };
    std::vector<Rider> riders;
    RigExecValueId ikValues=UINT64_MAX,fkValues=UINT64_MAX;
    /// Diagnostics spelled at bind: an op body does not spell paths.
    std::string unreadable,unmatched;
};
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
    /// An IK/FK match's reads and outputs; null for every other constraint.
    std::shared_ptr<const RigExecIkFkMatchGraphBinding> match;
    /// Outputs beyond target.output and the chain outputs.
    std::vector<RigExecValueId> writes;
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
