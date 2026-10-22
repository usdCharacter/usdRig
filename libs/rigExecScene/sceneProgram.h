#ifndef RIGEXEC_GRAPH_SCENE_PROGRAM_H
#define RIGEXEC_GRAPH_SCENE_PROGRAM_H
#include "sceneProgramLayout.h"
#include "propertyGraphBinding.h"
#include "solverGraphBinding.h"
#include "constraintGraphBinding.h"
#include "weightGraphBinding.h"
#include "geometryGraphBinding.h"
#include "poseGraphBinding.h"
#include "poseCommit.h"
#include "autoClavicleGraph.h"
#include "rigExecGraph/windowHistory.h"
#include <memory>
#include <variant>
namespace rigExec {
struct RigExecSceneGeometryOp {
    RigExecGeometryGraphBinding binding;
    RigExecValueId incoming=UINT64_MAX,output=UINT64_MAX,field=UINT64_MAX;
    RigExecValueId pointsBase=UINT64_MAX,pointsFinal=UINT64_MAX;
    RigExecWeightPacket fieldPacket;
    bool posedPoints=false,useSimd=true;
    /// Earlier frames a windowed mover reads; 0 for every other mover.
    uint32_t windowFrames=0;
};
struct RigExecSceneBlendChannelOp { RigExecGeometryGraphBinding binding;RigExecValueId output=UINT64_MAX; };
struct RigExecSceneWeightOp { RigExecWeightGraphBinding binding;int object=-1;RigExecValueId output=UINT64_MAX; };
struct RigExecSceneFieldOp { RigExecWeightGraphBinding binding;RigExecGraphTypedRead defaultWeight;RigExecValueId points=UINT64_MAX,output=UINT64_MAX;size_t count=1;std::string staticError; };
struct RigExecSceneExtractOp { RigExecValueId aggregate=UINT64_MAX,fallback=UINT64_MAX,output=UINT64_MAX;size_t element=0; };
struct RigExecSceneProviderOp {uint32_t originalIndex=0;RigExecValueId output=UINT64_MAX;std::vector<RigExecValueId> reads;};
struct RigExecSceneEffectiveReadOp {RigExecGraphTypedRead read;RigExecValueId output=UINT64_MAX;std::vector<RigExecValueId> reads;};
struct RigExecSceneUnavailableFrameOp { RigExecValueId output=UINT64_MAX; };
struct RigExecSceneCopyOp { RigExecValueId input=UINT64_MAX,output=UINT64_MAX;bool authoritative=false; };
struct RigExecSceneSwitchOp { RigExecBoundSpaceSwitch binding;RigExecValueId output=UINT64_MAX; };
using RigExecSceneOperation=std::variant<RigExecPropertyGraphBinding,RigExecSolverGraphBinding,
    RigExecSceneWeightOp,RigExecSceneGeometryOp,RigExecSceneExtractOp,RigExecSceneSwitchOp,RigExecBoundPoseInterpolator,RigExecConstraintGraphBinding,RigExecSceneCopyOp,RigExecSceneFieldOp,RigExecPoseCommitBinding,RigExecSceneProviderOp,RigExecSceneEffectiveReadOp,RigExecSceneBlendChannelOp,RigExecSceneUnavailableFrameOp,RigExecBoundAutoClavicle>;
/// Shared typed production operation table. Native capture and SceneDb compile
/// into this table; the same producer compiler/executor owns every operation.
class RigExecSceneProgram {
public:
    RigExecSceneProgramLayout layout;
    std::vector<RigExecSceneOperation> operations;
    std::map<SdfPath,RigExecValueId> publicValues;
    // Compile-only owner admission, independent of the serialized value graph.
    std::map<SdfPath,std::string> skippedOperations;
    RigExecCompiledGraph graph;
    /// Original compiler SCC authority retained across explicit recompiles.
    RigExecOpExclusionProof retainedExclusions;
    std::shared_ptr<const char> identity;
    /// Windowed operations (rigExecGraph/windowHistory.h), settled by Compile
    /// from the compiled predecessors, as every evaluator settles them.
    struct WindowedOp {
        size_t operation=0;
        uint32_t frames=0,canonical=UINT32_MAX;
        std::vector<uint32_t> inner;
        bool feedsWindow=false;
    };
    std::vector<WindowedOp> windowedOps;
    /// Per canonical op: 1 for the windowed ops and every op one depends on.
    std::vector<char> historyMask;
    /// How many earlier ticks a complete history reaches back.
    size_t GetHistoryDepth() const;
    bool Append(const std::string &key,RigExecSceneOperation,std::string *error=nullptr);
    bool Compile(RigExecCyclePolicy,std::string *error=nullptr);
    bool Compile(RigExecCyclePolicy,const RigExecOpExclusionProof *,std::string *error);
};
struct RigExecSceneOperationWorkspace {
    RigExecSolverWorkspace solver;
    RigExecPoseCommitWorkspace poseCommit;
    RigExecWeightGraphWorkspace weight;
    RigExecGeometryGraphWorkspace geometryInputs;
    RigExecGeometryWorkspace geometry;
    std::vector<GfVec3f> points,basePoints,finalPoints;
    std::vector<std::string> geometryDiagnostics;
    std::vector<RigExecBlendChannel> blendChannels;
};
/// Retained values, domain scratch, and readiness workspace. No source access,
/// metadata decisions or path-based version lookup occurs in operation bodies.
class RigExecSceneProgramRuntime {
public:
    RigExecTypedValueStore values;
    RigExecOpExecution execution;
    bool Prepare(const RigExecSceneProgram &,std::string *error=nullptr);
    bool Evaluate(const RigExecSceneProgram &,size_t identity,
        const std::map<SdfPath,VtValue> &overlays={},std::string *error=nullptr);
    /// History for windowed operations, kept as RigExecRuntimeReader keeps
    /// it: a host playing in order calls PushHistory after each Evaluate that
    /// advanced a frame; a host that jumps calls ClearHistory, then
    /// EvaluateHistory at each of GetHistoryDepth earlier identities, oldest
    /// first. A drag is an Evaluate with overlays and no push.
    void PushHistory();
    void ClearHistory();
    /// Runs the windowed operations and what they depend on at \p identity,
    /// without overlays, on a runtime of its own; their captures become the
    /// newest history tick. Publishes nothing here.
    bool EvaluateHistory(const RigExecSceneProgram &,size_t identity,std::string *error=nullptr);
private:
    struct Window {
        RigExecWindowRing ring,read;
        RigExecWindowPacketPtr capture;
        std::shared_ptr<const void> cache;
    };
    std::vector<Window> _windows;
    std::vector<int32_t> _windowOfOperation;
    bool _historyTick=false;
    std::unique_ptr<RigExecSceneProgramRuntime> _history;
    bool Execute(const RigExecSceneProgram &,size_t identity,const std::map<SdfPath,VtValue> &overlays,
        const std::vector<char> *active,std::string *error);
    std::vector<RigExecSceneOperation> _operations;
    std::vector<RigExecSceneOperationWorkspace> _workspaces;
    RigExecOpWorkspace _ready;
    std::vector<RigExecValueId> _changed;
    bool _ran=false;
    const RigExecSceneProgram *_preparedProgram=nullptr;
    std::shared_ptr<const char> _identity;
    std::vector<RigExecValueId> _excluded;
    bool Run(const RigExecSceneProgram &,uint32_t,std::string *);
};
}
#endif
