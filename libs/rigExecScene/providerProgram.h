#ifndef RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#define RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#include "sceneDescriptors.h"
#include "typedValues.h"
#include "rigExecGraph/providerRecords.h"
#include <limits>
#include <set>
#include <string_view>

namespace rigExec {
constexpr RigExecValueId RigExecNoProviderValue=std::numeric_limits<RigExecValueId>::max();
struct RigExecProviderXformInput {
    TfToken name;
    RigExecValueId raw=RigExecNoProviderValue;
    int type=0;
};
constexpr uint32_t RigExecNoProviderOwnerText=std::numeric_limits<uint32_t>::max();
struct RigExecProviderOp {
    RigExecProviderOpKind kind;
    SdfPath owner;
    RigExecValueId output=RigExecNoProviderValue;
    std::vector<RigExecValueId> inputs;
    std::vector<RigExecProviderXformInput> xforms;
    bool scaleAvars=true;
    /// `owner`'s entry in RigExecProviderProgram::ownerTexts.
    uint32_t ownerText=RigExecNoProviderOwnerText;
    /// AffineFrame: which frame expression it computes, and how many of its
    /// trailing frame inputs are a constraint frame's targets (the rest are
    /// their target objects).
    uint32_t affineKind=0;
    uint32_t affineTargets=0;
};
struct RigExecProviderLeaf {
    RigExecValueId value;
    SdfPath attribute;
};
struct RigExecProviderExternalInput {
    RigExecValueId value;
    SdfPath owner;
    std::string computation;
};
struct RigExecProviderRoutedInput {
    RigExecValueId value;
    SdfPath consumer,source;
    TfToken readPhase;
};
struct RigExecProviderProgram {
    std::vector<std::string> valueKeys;
    std::map<std::string,RigExecValueId> valueIds;
    std::vector<RigExecProviderLeaf> sampled;
    std::map<SdfPath,RigExecValueId> rawInputs, attributeValues;
    std::vector<RigExecProviderExternalInput> externalInputs;
    std::vector<RigExecProviderRoutedInput> routedInputs;
    std::vector<RigExecValueId> leaves;
    std::vector<RigExecProviderOp> ops;
    std::vector<RigExecOpDescriptor> descriptors;
    /// Each distinct op owner, spelled once on the thread that builds or
    /// extends the program. An op body names its owner only through this
    /// table: SdfPath::GetString interns under Sdf's table locks.
    std::vector<std::string> ownerTexts;
    RigExecValueId FindValue(const std::string &key) const;
};
/// Owning-thread, compilation-local indexes for an append-only program.
/// First producers win, including duplicate outputs. Discard these indexes
/// if existing rows are changed or removed, or before using another program.
struct RigExecProviderContextIndex {
    std::map<RigExecValueId,size_t> producers, externals;
    size_t indexedOps=0, indexedExternals=0;
    void Extend(const RigExecProviderProgram &program);
};
/// Spells the owner of every op that has no text yet into ownerTexts, once
/// per distinct owner. The owning thread calls it after adding ops; a copied
/// op keeps its text.
void RigExecSpellProviderOwners(RigExecProviderProgram *program);
std::string RigExecProviderValueKey(const SdfPath &owner,const std::string &computation);
std::string RigExecProviderRawKey(const SdfPath &attribute);
std::string RigExecProviderAttributeKey(const SdfPath &attribute);
std::string RigExecProviderRoutedKey(const SdfPath &consumer,const TfToken &phase);
/// With composeFrames=false, native rest/posed frames are explicit leaves;
/// each expression kernel can enter the native graph as one ordinary op.
bool RigExecBuildProviderProgram(const RigExecSceneDescriptors &scene,
    bool composeFrames,RigExecProviderProgram *program,std::string *error=nullptr,
    const std::set<SdfPath> *selectedProviders=nullptr);
/// Clone one connected expression closure into a consumer's selected frame
/// context. Explicit replacements are typed SSA IDs; computation name remaps
/// create external bridge leaves. Other external/raw leaves remain shared.
bool RigExecCloneProviderContext(RigExecProviderProgram *,RigExecValueId result,
    const std::string &prefix,const std::map<std::string,std::string> &externalNames,
    const std::map<RigExecValueId,RigExecValueId> &replacements,
    RigExecValueId *output,std::string *error=nullptr);
/// Reuses and extends the compilation-local producer indexes.
bool RigExecCloneProviderContext(RigExecProviderProgram *,RigExecValueId result,
    const std::string &prefix,const std::map<std::string,std::string> &externalNames,
    const std::map<RigExecValueId,RigExecValueId> &replacements,
    RigExecValueId *output,std::string *error,RigExecProviderContextIndex *index);
/// Where RigExecComposeLocalXform finds an xform operation's value. \p find
/// receives the operation name as the order spells it (the "!invert!" prefix
/// removed) and answers false when the prim captured no such operation. On
/// true it sets \p *value to the operation's value, or to null when it has
/// none; \p *type to a UsdGeomXformOp::Type, or -1 to derive it from the
/// name. Plain function pointer and context: no allocation per call.
struct RigExecXformOpLookup {
    void *context;
    bool (*find)(void *context,std::string_view name,int *type,const VtValue **value);
};
/// The local transform of one prim from its xformOpOrder \p order: the
/// operations compose last-to-first and stop at "!resetXformStack!" (\p
/// *resets), an adjacent op/!invert!op pair cancels, an identity operation
/// is skipped. The one body behind the provider LocalXform op and the
/// setup transforms movers resolve at compile. An operation whose value is
/// null fails ("xform operation unavailable"), unless \p missingIsIdentity:
/// an op with no Default value then contributes identity. On false \p error
/// gets the reason and \p failedOp (optional) the order token it names.
/// Spells no path and builds no token.
bool RigExecComposeLocalXform(const VtTokenArray &order,const RigExecXformOpLookup &lookup,
    bool missingIsIdentity,GfMatrix4d *local,bool *resets,std::string *error,
    TfToken *failedOp=nullptr);
bool RigExecRunProviderOp(const RigExecProviderProgram &program,uint32_t originalIndex,
    RigExecTypedValueStore *store,std::string *error=nullptr);
/// Whether op \p index of \p program can run at all: it exists, its output
/// is inside the typed layout, its kind is one the kernels handle, and a
/// LocalXform or InterveningXform owner was spelled at build. The per-op
/// half of RigExecRunProviderOp's failures (the other half is the store
/// binding); on false, \p error gets the message RigExecRunProviderOp
/// fails with. A provider op that fails here is a pruning root.
bool RigExecProviderOpStructurallyValid(const RigExecProviderProgram &program,
                                        uint32_t index, std::string *error);
/// Sampling/overlays publish raw slots before entering the readiness loop.
bool RigExecSampleProviderProgram(const RigExecProviderProgram &program,
    const RigExecSceneDescriptors &scene,size_t identity,
    const std::map<SdfPath,VtValue> &overlays,RigExecTypedValueStore *store,
    std::vector<RigExecValueId> *changed,std::string *error=nullptr);
}
#endif
