#ifndef RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#define RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#include "sceneDescriptors.h"
#include "typedValues.h"
#include "rigExecGraph/providerRecords.h"
#include "pxr/base/vt/array.h"
#include <limits>
#include <optional>
#include <set>
#include <string_view>

namespace rigExec {
constexpr RigExecValueId RigExecNoProviderValue =
    std::numeric_limits<RigExecValueId>::max();
struct RigExecProviderXformInput {
    TfToken name;
    RigExecValueId raw = RigExecNoProviderValue;
    int type = 0;
};
constexpr uint32_t RigExecNoProviderOwnerText =
    std::numeric_limits<uint32_t>::max();
struct RigExecProviderOp {
    RigExecProviderOpKind kind;
    SdfPath owner;
    RigExecValueId output = RigExecNoProviderValue;
    std::vector<RigExecValueId> inputs;
    std::vector<RigExecProviderXformInput> xforms;
    bool scaleAvars = true;
    /// `owner`'s entry in RigExecProviderProgram::ownerTexts.
    uint32_t ownerText = RigExecNoProviderOwnerText;
    /// AffineFrame: which frame expression it computes, and how many of its
    /// trailing frame inputs are a constraint frame's targets (the rest are
    /// their target objects).
    uint32_t affineKind = 0;
    uint32_t affineTargets = 0;
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
    SdfPath consumer, source;
    TfToken readPhase;
};
struct RigExecProviderProgram {
    std::vector<std::string> valueKeys;
    std::map<std::string, RigExecValueId> valueIds;
    std::vector<RigExecProviderLeaf> sampled;
    std::map<SdfPath, RigExecValueId> rawInputs, attributeValues;
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
    std::map<RigExecValueId, size_t> producers, externals;
    size_t indexedOps = 0, indexedExternals = 0;
    void Extend(const RigExecProviderProgram &program);
};
/// Spells the owner of every op that has no text yet into ownerTexts, once
/// per distinct owner. The owning thread calls it after adding ops; a copied
/// op keeps its text.
void RigExecSpellProviderOwners(RigExecProviderProgram *program);
std::string RigExecProviderValueKey(const SdfPath &owner,
                                    const std::string &computation);
std::string RigExecProviderRawKey(const SdfPath &attribute);
std::string RigExecProviderAttributeKey(const SdfPath &attribute);
std::string RigExecProviderRoutedKey(const SdfPath &consumer,
                                     const TfToken &phase);
/// Native rest and posed frames and intervening spaces are external leaves;
/// each expression kernel enters the native graph as one ordinary op.
bool RigExecBuildProviderProgram(
    const RigExecSceneDescriptors &scene, RigExecProviderProgram *program,
    std::string *error = nullptr,
    const std::set<SdfPath> *selectedProviders = nullptr);
/// Clone one connected expression closure into a consumer's selected frame
/// context. Explicit replacements are typed SSA IDs; computation name remaps
/// create external bridge leaves. Other external/raw leaves remain shared.
bool RigExecCloneProviderContext(
    RigExecProviderProgram *, RigExecValueId result, const std::string &prefix,
    const std::map<std::string, std::string> &externalNames,
    const std::map<RigExecValueId, RigExecValueId> &replacements,
    RigExecValueId *output, std::string *error = nullptr);
/// Reuses and extends the compilation-local producer indexes.
bool RigExecCloneProviderContext(
    RigExecProviderProgram *, RigExecValueId result, const std::string &prefix,
    const std::map<std::string, std::string> &externalNames,
    const std::map<RigExecValueId, RigExecValueId> &replacements,
    RigExecValueId *output, std::string *error,
    RigExecProviderContextIndex *index);
/// Where RigExecComposeLocalXform finds an xform operation's value. \p find
/// receives the operation name as the order spells it (the "!invert!" prefix
/// removed) and answers false when the prim captured no such operation. On
/// true it sets \p *value to the operation's value, or to null when it has
/// none; \p *type to a UsdGeomXformOp::Type, or -1 to derive it from the
/// name. Plain function pointer and context: no allocation per call.
struct RigExecXformOpLookup {
    void *context;
    bool (*find)(void *context, std::string_view name, int *type,
                 const VtValue **value);
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
bool RigExecComposeLocalXform(const VtTokenArray &order,
                              const RigExecXformOpLookup &lookup,
                              bool missingIsIdentity, GfMatrix4d *local,
                              bool *resets, std::string *error,
                              TfToken *failedOp = nullptr);
/// The typed store behind RigExecRunProviderArithmetic: present unblocked
/// values read by type, missing inputs invalidate the output, results carry
/// their change, count and error state. Shared by the program runner here
/// and the baked space-step host.
struct GfProviderStore {
    RigExecTypedValueStore &store;
    const GfMatrix4d *ReadMatrix(uint64_t id) const
    {
        return store.Read<GfMatrix4d>(id);
    }
    const RigExecPointFrame *ReadFrame(uint64_t id) const
    {
        return store.Read<RigExecPointFrame>(id);
    }
    std::optional<double> ReadScalar(uint64_t id) const
    {
        if (const auto *value = store.Read<double>(id))
            return *value;
        if (const auto *narrow = store.Read<float>(id))
            return double(*narrow);
        return std::nullopt;
    }
    const GfVec3d *ReadVector(uint64_t id) const
    {
        return store.Read<GfVec3d>(id);
    }
    std::string_view ReadToken(uint64_t id) const
    {
        // A field read: GetString() on an empty token reads a function-local static.
        const auto *token = store.Read<TfToken>(id);
        return token ? std::string_view(token->GetText(), token->size())
                     : std::string_view();
    }
    bool ResetXformStack(uint64_t id) const
    {
        if (id >= store.values.size() ||
            !store.values[size_t(id)].initialized ||
            store.values[size_t(id)].blocked)
            return false;
        const auto &value = store.values[size_t(id)].raw;
        if (!value.IsHolding<VtTokenArray>())
            return false;
        for (const auto &token : value.UncheckedGet<VtTokenArray>())
            if (token == "!resetXformStack!")
                return true;
        return false;
    }
    bool Present(uint64_t id) const
    {
        return id < store.values.size() && store.values[size_t(id)].initialized;
    }
    bool Authoritative(uint64_t id) const
    {
        return id < store.values.size() &&
               store.values[size_t(id)].authoritative;
    }
    void Copy(uint64_t to, uint64_t from, bool authoritative)
    {
        store.Copy(to, from, authoritative);
    }
    void Invalidate(uint64_t to, uint64_t from)
    {
        const bool blocked =
            from < store.values.size() && store.values[size_t(from)].blocked;
        store.values.at(size_t(to)).raw = VtValue();
        store.Publish(to, std::monostate(), blocked, false, 0,
                      "provider input unavailable");
    }
    void PublishMatrix(uint64_t id, const GfMatrix4d &value, bool authoritative)
    {
        store.Publish(id, value, false, authoritative);
    }
    void PublishFrame(uint64_t id, const RigExecPointFrame &value)
    {
        store.Publish(id, value);
    }
};
/// Runs Attribute op \p originalIndex: its authored value where that is
/// authoritative or it has no connection, else its connection's. Every
/// other kind runs as an op node.
bool RigExecRunProviderOp(const RigExecProviderProgram &program,
                          uint32_t originalIndex, RigExecTypedValueStore *store,
                          std::string *error = nullptr);
/// Whether op \p index of \p program can run at all: it exists, its output
/// is inside the typed layout and its kind is one the kernels handle. On
/// false, \p error gets the reason. A provider op that fails here is a
/// pruning root.
bool RigExecProviderOpStructurallyValid(const RigExecProviderProgram &program,
                                        uint32_t index, std::string *error);
/// Sampling/overlays publish raw slots before entering the readiness loop.
bool RigExecSampleProviderProgram(const RigExecProviderProgram &program,
                                  const RigExecSceneDescriptors &scene,
                                  size_t identity,
                                  const std::map<SdfPath, VtValue> &overlays,
                                  RigExecTypedValueStore *store,
                                  std::vector<RigExecValueId> *changed,
                                  std::string *error = nullptr);
} // namespace rigExec
#endif
