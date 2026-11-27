#include "providerProgram.h"
#include "rigExecGraph/providerArithmetic.h"
#include "rigExecMath/affineFrameKernels.h"
#include "rigExecMath/avarScale.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecSchema/schemaTokens.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/staticTokens.h"
#include <iterator>
#include "pxr/base/tf/hash.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usdGeom/xformOp.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>

namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (xformOpOrder) ((restSpace, SchemaAttrText::restSpace))
    ((posedSpace, SchemaAttrText::posedSpace))
    ((defaultSpace, SchemaAttrText::defaultSpace))
    ((avarsDefaultSpace, SchemaAttrText::avarsDefaultSpace))
    ((posedDefaultSpace, SchemaAttrText::posedDefaultSpace))
    ((parentSpace, SchemaAttrText::parentSpace))
    ((parentDefaultSpace, SchemaAttrText::parentDefaultSpace))
    ((restTx, SchemaAttrText::restTx)) ((restTy, SchemaAttrText::restTy))
    ((restTz, SchemaAttrText::restTz)) ((restRx, SchemaAttrText::restRx))
    ((restRy, SchemaAttrText::restRy)) ((restRz, SchemaAttrText::restRz))
    ((defaultTx, SchemaAttrText::defaultTx))
    ((defaultTy, SchemaAttrText::defaultTy))
    ((defaultTz, SchemaAttrText::defaultTz))
    ((defaultRx, SchemaAttrText::defaultRx))
    ((defaultRy, SchemaAttrText::defaultRy))
    ((defaultRz, SchemaAttrText::defaultRz))
    ((avarsUnitScaleFactor, SchemaAttrText::avarsUnitScaleFactor))
    ((avarsTx, SchemaAttrText::avarsTx)) ((avarsTy, SchemaAttrText::avarsTy))
    ((avarsTz, SchemaAttrText::avarsTz)) ((avarsSx, SchemaAttrText::avarsSx))
    ((avarsSy, SchemaAttrText::avarsSy)) ((avarsSz, SchemaAttrText::avarsSz))
    ((avarsRx, SchemaAttrText::avarsRx)) ((avarsRy, SchemaAttrText::avarsRy))
    ((avarsRz, SchemaAttrText::avarsRz))
    ((avarsRspin, SchemaAttrText::avarsRspin))
    ((avarsRotationOrder, SchemaAttrText::avarsRotationOrder))
    ((avarsRotationSign, SchemaAttrText::avarsRotationSign))
);
namespace {
bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}
bool
Provider(const TfToken &type)
{
    return type == SchemaType::RigExecControl ||
           type == SchemaType::RigExecJoint ||
           type == SchemaType::RigExecSphereWeight ||
           type == SchemaType::RigExecPlaneWeight ||
           type == SchemaType::RigExecCurveWeight;
}
TF_DEFINE_PRIVATE_TOKENS(_computationTokens,
    (computeRestFrame)(computedDefaultSpace)(computeDefaultFrame)(computedParentDefaultSpace)
    (computedParentSpace)(computePointFrame)(computeMatrix)(interveningSpace)(computeAvarMatrix));
// A value key by structure, so a lookup builds no text. Its text, made once
// at creation, stays the value's name and decides its id.
enum class _KeyKind : uint8_t { Computation, Connection, Raw, Value };
struct _Key {
    SdfPath path;
    TfToken name;
    _KeyKind kind;
    bool operator==(const _Key &other) const
    {
        return kind == other.kind && path == other.path && name == other.name;
    }
};
struct _KeyHash {
    size_t operator()(const _Key &key) const
    {
        return TfHash::Combine(key.path, key.name, uint8_t(key.kind));
    }
};
const char *const expressions[] = {
    SchemaAttrText::defaultSpace, SchemaAttrText::avarsDefaultSpace,
    SchemaAttrText::posedDefaultSpace, SchemaAttrText::parentSpace,
    SchemaAttrText::parentDefaultSpace};
bool
Expression(const std::string &name)
{
    for (const auto *expression : expressions)
        if (name == expression)
            return true;
    return false;
}
SdfPath
ParentProvider(const RigExecSceneDescriptors &scene, const SdfPath &path)
{
    for (auto parent = path.GetParentPath(); !parent.IsEmpty();
         parent = parent.GetParentPath()) {
        const auto found = scene.nodes.find(parent);
        if (found != scene.nodes.end() && found->second.fact.active &&
            Provider(found->second.fact.type))
            return parent;
    }
    return {};
}
// Affine frame expressions: each publishes its outputs:matrix from its own
// input attributes and the point frames of the providers its relationships
// name, as an ordinary op whose inputs are those values. The layout
// (RigExecAffineFrameTypes) states every input in op order; the kernels are
// affineFrameKernel.h's, which the runtime runs too.
constexpr uint32_t affineTypeCount = RigExecAffineFrameTypeCount;
// The affine type of \p type, or affineTypeCount.
uint32_t
AffineTypeOf(const TfToken &type)
{
    for (uint32_t k = 0; k < affineTypeCount; ++k)
        if (type == RigExecAffineFrameTypes[k].type)
            return k;
    return affineTypeCount;
}
bool
AffineOutput(const RigExecSceneDescriptors &scene, const SdfPath &attribute,
             uint32_t *kind)
{
    if (attribute.GetName() != SchemaAttrText::outputsMatrix)
        return false;
    const auto node = scene.nodes.find(attribute.GetPrimPath());
    if (node == scene.nodes.end() || !node->second.fact.active)
        return false;
    *kind = AffineTypeOf(node->second.fact.type);
    return *kind < affineTypeCount;
}
// RigExecAffineFrameTypes' USD-free names as tokens, built once when the
// library loads, so translation never builds one from text.
struct AffineTokens {
    std::vector<TfToken> attributes, relations;
    TfToken targets, targetObjects;
};
const std::vector<AffineTokens> _affineTokens = [] {
    std::vector<AffineTokens> table;
    for (const auto &type : RigExecAffineFrameTypes) {
        AffineTokens tokens;
        for (uint32_t k = 0; k < type.attributeCount; ++k)
            tokens.attributes.emplace_back(type.attributes[k].name);
        for (uint32_t k = 0; k < type.relationCount; ++k)
            tokens.relations.emplace_back(type.relations[k].name);
        if (type.targets)
            tokens.targets = TfToken(type.targets);
        if (type.targetObjects)
            tokens.targetObjects = TfToken(type.targetObjects);
        table.push_back(std::move(tokens));
    }
    return table;
}();
// The xform operation type a name's type part spells, from the type tokens.
UsdGeomXformOp::Type
XformOpType(std::string_view name, size_t begin, size_t count)
{
    for (int t = UsdGeomXformOp::TypeInvalid + 1;
         t <= UsdGeomXformOp::TypeTransform; ++t)
        if (!name.compare(
                begin, count,
                UsdGeomXformOp::GetOpTypeToken(UsdGeomXformOp::Type(t))
                    .GetString()))
            return UsdGeomXformOp::Type(t);
    return UsdGeomXformOp::TypeInvalid;
}
SdfPathVector
AffineTargets(const RigExecSceneDescriptors &scene, const SdfPath &prim,
              const TfToken &name)
{
    if (name.IsEmpty())
        return {};
    const auto found = scene.relationships.find(prim.AppendProperty(name));
    return found == scene.relationships.end() ? SdfPathVector()
                                              : found->second.fact.targets;
}
struct GfProviderMath {
    using Matrix = GfMatrix4d;
    using Vector = GfVec3d;
    using Rotation = GfRotation;
    using Frame = RigExecPointFrame;
    static bool PointsToMatrix(const std::array<Vector, 4> &rest,
                               const std::array<Vector, 4> &pose, Matrix *out)
    {
        return RigExecPointsToMatrix(rest, pose, out);
    }
};
} // namespace
std::string
RigExecProviderValueKey(const SdfPath &owner, const std::string &computation)
{
    return "computation:" + owner.GetString() + ":" + computation;
}
std::string
RigExecProviderRawKey(const SdfPath &attribute)
{
    return "raw:" + attribute.GetString();
}
std::string
RigExecProviderAttributeKey(const SdfPath &attribute)
{
    return "value:" + attribute.GetString();
}
std::string
RigExecProviderRoutedKey(const SdfPath &consumer, const TfToken &phase)
{
    return "connection:" + consumer.GetString() + ":" + phase.GetString();
}
RigExecValueId
RigExecProviderProgram::FindValue(const std::string &key) const
{
    const auto found = valueIds.find(key);
    return found == valueIds.end() ? RigExecNoProviderValue : found->second;
}
bool
RigExecBuildProviderProgram(const RigExecSceneDescriptors &scene,
                            RigExecProviderProgram *output, std::string *error,
                            const std::set<SdfPath> *selectedProviders)
{
    if (!output)
        return Fail(error, "provider compiler needs an output");
    RigExecProviderProgram program;
    std::set<SdfPath> providerPaths, usedAttributes;
    std::function<void(const SdfPath &)> includeAttribute, includeProvider;
    std::function<void(const SdfPath &, uint32_t)> includeAffine;
    includeAttribute = [&](const SdfPath &path) {
        const auto attribute = scene.attributes.find(path);
        const auto owner = scene.nodes.find(path.GetPrimPath());
        if (attribute == scene.attributes.end() || owner == scene.nodes.end() ||
            !owner->second.fact.active || !usedAttributes.insert(path).second)
            return;
        if (Provider(owner->second.fact.type) && Expression(path.GetName()))
            includeProvider(path.GetPrimPath());
        uint32_t affine = 0;
        if (AffineOutput(scene, path, &affine))
            includeAffine(path.GetPrimPath(), affine);
        for (const auto &connection : attribute->second.fact.connections)
            includeAttribute(connection);
    };
    // An affine frame expression reads its input attributes and the point
    // frames of the providers its relationships name.
    includeAffine = [&](const SdfPath &prim, uint32_t kind) {
        const AffineTokens &tokens = _affineTokens[kind];
        for (const auto &name : tokens.attributes)
            includeAttribute(prim.AppendProperty(name));
        const auto frames = [&](const TfToken &name) {
            if (name.IsEmpty())
                return;
            for (const auto &target : AffineTargets(scene, prim, name)) {
                const auto node = scene.nodes.find(target);
                if (target.IsPrimPath() && node != scene.nodes.end() &&
                    node->second.fact.active &&
                    Provider(node->second.fact.type))
                    includeProvider(target);
            }
        };
        for (const auto &name : tokens.relations)
            frames(name);
        frames(tokens.targets);
        frames(tokens.targetObjects);
    };
    includeProvider = [&](const SdfPath &path) {
        if (path.IsEmpty() || !providerPaths.insert(path).second)
            return;
        includeProvider(ParentProvider(scene, path));
        const TfToken *const inputs[] = {&_tokens->restSpace,
                                         &_tokens->posedSpace,
                                         &_tokens->defaultSpace,
                                         &_tokens->avarsDefaultSpace,
                                         &_tokens->posedDefaultSpace,
                                         &_tokens->parentSpace,
                                         &_tokens->parentDefaultSpace,
                                         &_tokens->restTx,
                                         &_tokens->restTy,
                                         &_tokens->restTz,
                                         &_tokens->restRx,
                                         &_tokens->restRy,
                                         &_tokens->restRz,
                                         &_tokens->defaultTx,
                                         &_tokens->defaultTy,
                                         &_tokens->defaultTz,
                                         &_tokens->defaultRx,
                                         &_tokens->defaultRy,
                                         &_tokens->defaultRz,
                                         &_tokens->avarsUnitScaleFactor,
                                         &_tokens->avarsTx,
                                         &_tokens->avarsTy,
                                         &_tokens->avarsTz,
                                         &_tokens->avarsSx,
                                         &_tokens->avarsSy,
                                         &_tokens->avarsSz,
                                         &_tokens->avarsRx,
                                         &_tokens->avarsRy,
                                         &_tokens->avarsRz,
                                         &_tokens->avarsRspin,
                                         &_tokens->avarsRotationOrder,
                                         &_tokens->avarsRotationSign};
        for (const auto *input : inputs)
            includeAttribute(path.AppendProperty(*input));
    };
    for (const auto &[path, node] : scene.nodes)
        if (node.fact.active && Provider(node.fact.type) &&
            (!selectedProviders || selectedProviders->count(path)))
            includeProvider(path);
    std::map<std::string, _Key> keys;
    std::map<SdfPath, RigExecValueId> routed;
    const auto &C = *_computationTokens;
    for (const auto &[path, attr] : scene.attributes) {
        if (!usedAttributes.count(path))
            continue;
        const auto node = scene.nodes.find(path.GetPrimPath());
        if (node == scene.nodes.end() || !node->second.fact.active)
            continue;
        keys.emplace(RigExecProviderRawKey(path),
                     _Key{path, TfToken(), _KeyKind::Raw});
        keys.emplace(RigExecProviderAttributeKey(path),
                     _Key{path, TfToken(), _KeyKind::Value});
        if (attr.readPhase != "base" && attr.fact.connections.size() == 1) {
            const auto input =
                scene.attributes.find(attr.fact.connections.front());
            const auto inputNode =
                scene.nodes.find(attr.fact.connections.front().GetPrimPath());
            if (input != scene.attributes.end() &&
                inputNode != scene.nodes.end() &&
                inputNode->second.fact.active &&
                input->second.fact.type.GetType() == attr.fact.type.GetType())
                keys.emplace(RigExecProviderRoutedKey(path, attr.readPhase),
                             _Key{path, attr.readPhase, _KeyKind::Connection});
        }
    }
    for (const auto &[path, node] : scene.nodes)
        if (node.fact.active) {
            if (providerPaths.count(path))
                // Every computation a provider node can produce. The token is the
                // only spelling; the value key's text is the token's own name.
                for (const TfToken &computation :
                     {C.computeRestFrame, C.computedDefaultSpace,
                      C.computeDefaultFrame, C.computedParentDefaultSpace,
                      C.computedParentSpace, C.computePointFrame,
                      C.computeMatrix, C.interveningSpace, C.computeAvarMatrix})
                    keys.emplace(
                        RigExecProviderValueKey(path, computation.GetString()),
                        _Key{path, computation, _KeyKind::Computation});
        }
    std::unordered_map<_Key, RigExecValueId, _KeyHash> index;
    index.reserve(keys.size());
    for (const auto &[text, key] : keys) {
        const auto id = RigExecValueId(program.valueKeys.size());
        program.valueIds.emplace(text, id);
        program.valueKeys.push_back(text);
        index.emplace(key, id);
    }
    const auto find = [&index](_KeyKind kind, const SdfPath &path,
                               const TfToken &name = TfToken()) {
        const auto found = index.find(_Key{path, name, kind});
        return found == index.end() ? RigExecNoProviderValue : found->second;
    };
    for (const auto &path : usedAttributes) {
        const auto &attr = scene.attributes.at(path);
        const auto value = find(_KeyKind::Connection, path, attr.readPhase);
        if (value != RigExecNoProviderValue) {
            routed.emplace(path, value);
            program.leaves.push_back(value);
            program.routedInputs.push_back(
                {value, path, attr.fact.connections.front(), attr.readPhase});
        }
    }
    const auto attrId = [&](const SdfPath &path, const TfToken &name) {
        return find(_KeyKind::Value, path.AppendProperty(name));
    };
    const auto compId = [&](const SdfPath &path, const TfToken &name) {
        return path.IsEmpty() ? RigExecNoProviderValue
                              : find(_KeyKind::Computation, path, name);
    };
    const auto connect = [&](const RigExecSceneAttribute &fact) {
        const auto route = routed.find(fact.path);
        if (route != routed.end())
            return route->second;
        if (fact.connections.size() != 1)
            return RigExecNoProviderValue;
        const auto found = scene.attributes.find(fact.connections.front());
        const auto node =
            scene.nodes.find(fact.connections.front().GetPrimPath());
        if (found == scene.attributes.end() || node == scene.nodes.end() ||
            !node->second.fact.active ||
            found->second.fact.type.GetType() != fact.type.GetType())
            return RigExecNoProviderValue;
        return find(_KeyKind::Value, fact.connections.front());
    };
    const auto add = [&](RigExecProviderOp op, const std::string &key) {
        RigExecOpDescriptor descriptor;
        descriptor.key = key;
        descriptor.kind = uint32_t(op.kind);
        descriptor.writes = {op.output};
        for (const auto id : op.inputs)
            if (id != RigExecNoProviderValue)
                descriptor.reads.push_back(id);
        for (const auto &xform : op.xforms)
            if (xform.raw != RigExecNoProviderValue)
                descriptor.reads.push_back(xform.raw);
        std::sort(descriptor.reads.begin(), descriptor.reads.end());
        descriptor.reads.erase(
            std::unique(descriptor.reads.begin(), descriptor.reads.end()),
            descriptor.reads.end());
        program.ops.push_back(std::move(op));
        program.descriptors.push_back(std::move(descriptor));
    };
    for (const auto &[path, attr] : scene.attributes) {
        if (!usedAttributes.count(path))
            continue;
        const auto node = scene.nodes.find(path.GetPrimPath());
        if (node == scene.nodes.end() || !node->second.fact.active)
            continue;
        const auto raw = find(_KeyKind::Raw, path);
        const auto value = find(_KeyKind::Value, path);
        program.rawInputs.emplace(path, raw);
        program.attributeValues.emplace(path, value);
        program.sampled.push_back({raw, path});
        program.leaves.push_back(raw);
        if (Provider(node->second.fact.type) && Expression(path.GetName()))
            continue;
        uint32_t affine = 0;
        if (AffineOutput(scene, path, &affine)) {
            // Inputs in the table's order: the attributes, one frame per
            // single relation, then a constraint frame's target frames and
            // target-object frames.
            const RigExecAffineFrameType &type =
                RigExecAffineFrameTypes[affine];
            const SdfPath prim = path.GetPrimPath();
            RigExecProviderOp op{
                RigExecProviderOpKind::AffineFrame, prim, value, {}};
            op.affineKind = affine;
            const auto frame = [&](const SdfPath &target) {
                return target.IsPrimPath() ? compId(target, C.computePointFrame)
                                           : RigExecNoProviderValue;
            };
            const AffineTokens &tokens = _affineTokens[affine];
            for (uint32_t k = 0; k < type.attributeCount; ++k)
                op.inputs.push_back(attrId(prim, tokens.attributes[k]));
            for (uint32_t k = 0; k < type.relationCount; ++k) {
                const auto targets =
                    AffineTargets(scene, prim, tokens.relations[k]);
                op.inputs.push_back(targets.empty() ? RigExecNoProviderValue
                                                    : frame(targets.front()));
            }
            if (type.targets) {
                const auto targets = AffineTargets(scene, prim, tokens.targets);
                for (const auto &target : targets)
                    op.inputs.push_back(frame(target));
                op.affineTargets = uint32_t(targets.size());
                for (const auto &target :
                     AffineTargets(scene, prim, tokens.targetObjects))
                    op.inputs.push_back(frame(target));
            }
            add(std::move(op), "affineFrame:" + path.GetString());
            continue;
        }
        if (attr.fact.connections.size() > 1 && attr.fact.type.IsArray() &&
            node->second.fact.type.GetString().rfind(SchemaNamePrefix::type,
                                                     0) == 0)
            return Fail(error, "provider translation needs typed array "
                               "connection concatenation: " +
                                   path.GetString());
        add({RigExecProviderOpKind::Attribute,
             path,
             value,
             {raw, connect(attr.fact)}},
            "attribute:" + path.GetString());
    }
    for (const auto &[path, node] : scene.nodes) {
        if (!providerPaths.count(path))
            continue;
        const SdfPath parent = ParentProvider(scene, path);
        const auto selfRest = compId(path, C.computeRestFrame),
                   selfPose = compId(path, C.computePointFrame);
        const auto parentRest = compId(parent, C.computeRestFrame);
        for (const TfToken *computation :
             {&C.computeRestFrame, &C.computePointFrame, &C.interveningSpace}) {
            const auto id = compId(path, *computation);
            program.leaves.push_back(id);
            program.externalInputs.push_back(
                {id, path, computation->GetString()});
        }
        add({RigExecProviderOpKind::FrameToSpace,
             path,
             compId(path, C.computedParentDefaultSpace),
             {compId(parent, C.computeDefaultFrame)}},
            "parentDefault:" + path.GetString());
        add({RigExecProviderOpKind::FrameToSpace,
             path,
             compId(path, C.computedParentSpace),
             {compId(parent, C.computePointFrame)}},
            "parentPosed:" + path.GetString());
        add({RigExecProviderOpKind::DefaultSpace,
             path,
             compId(path, C.computedDefaultSpace),
             {selfRest, parentRest, attrId(path, _tokens->parentDefaultSpace),
              attrId(path, _tokens->defaultTx),
              attrId(path, _tokens->defaultTy),
              attrId(path, _tokens->defaultTz),
              attrId(path, _tokens->defaultRx),
              attrId(path, _tokens->defaultRy),
              attrId(path, _tokens->defaultRz)}},
            "computedDefault:" + path.GetString());
        for (const auto *expression :
             {&_tokens->defaultSpace, &_tokens->avarsDefaultSpace,
              &_tokens->posedDefaultSpace, &_tokens->parentSpace,
              &_tokens->parentDefaultSpace}) {
            const SdfPath property = path.AppendProperty(*expression);
            const auto fact = scene.attributes.find(property);
            if (fact == scene.attributes.end())
                return Fail(error, "provider expression attribute missing: " +
                                       property.GetString());
            RigExecValueId fallback;
            if (*expression == _tokens->defaultSpace)
                fallback = compId(path, C.computedDefaultSpace);
            else if (*expression == _tokens->avarsDefaultSpace)
                fallback = attrId(path, _tokens->defaultSpace);
            else if (*expression == _tokens->posedDefaultSpace)
                fallback = attrId(path, _tokens->avarsDefaultSpace);
            else if (*expression == _tokens->parentSpace)
                fallback = compId(path, C.computedParentSpace);
            else
                fallback = compId(path, C.computedParentDefaultSpace);
            add({RigExecProviderOpKind::SpaceExpression,
                 property,
                 find(_KeyKind::Value, property),
                 {find(_KeyKind::Raw, property), connect(fact->second.fact),
                  fallback}},
                "spaceExpression:" + property.GetString());
        }
        add({RigExecProviderOpKind::MatrixToFrame,
             path,
             compId(path, C.computeDefaultFrame),
             {attrId(path, _tokens->defaultSpace)}},
            "defaultFrame:" + path.GetString());
        add({RigExecProviderOpKind::JointMatrix,
             path,
             compId(path, C.computeMatrix),
             {selfRest, selfPose}},
            "jointMatrix:" + path.GetString());
    }
    std::sort(program.leaves.begin(), program.leaves.end());
    program.leaves.erase(
        std::unique(program.leaves.begin(), program.leaves.end()),
        program.leaves.end());
    RigExecSpellProviderOwners(&program);
    *output = std::move(program);
    return true;
}
void
RigExecSpellProviderOwners(RigExecProviderProgram *program)
{
    if (!program)
        return;
    std::unordered_map<SdfPath, uint32_t, SdfPath::Hash> spelled;
    for (const auto &op : program->ops)
        if (op.ownerText < program->ownerTexts.size())
            spelled.emplace(op.owner, op.ownerText);
    for (auto &op : program->ops) {
        if (op.ownerText < program->ownerTexts.size())
            continue;
        const auto entry =
            spelled.emplace(op.owner, uint32_t(program->ownerTexts.size()));
        if (entry.second)
            program->ownerTexts.push_back(op.owner.GetString());
        op.ownerText = entry.first->second;
    }
}

void
RigExecProviderContextIndex::Extend(const RigExecProviderProgram &program)
{
    for (; indexedOps < program.ops.size(); ++indexedOps)
        producers.emplace(program.ops[indexedOps].output, indexedOps);
    for (; indexedExternals < program.externalInputs.size(); ++indexedExternals)
        externals.emplace(program.externalInputs[indexedExternals].value,
                          indexedExternals);
}

bool
RigExecCloneProviderContext(
    RigExecProviderProgram *program, RigExecValueId result,
    const std::string &prefix,
    const std::map<std::string, std::string> &externalNames,
    const std::map<RigExecValueId, RigExecValueId> &replacements,
    RigExecValueId *output, std::string *error)
{
    return RigExecCloneProviderContext(program, result, prefix, externalNames,
                                       replacements, output, error, nullptr);
}
bool
RigExecCloneProviderContext(
    RigExecProviderProgram *program, RigExecValueId result,
    const std::string &prefix,
    const std::map<std::string, std::string> &externalNames,
    const std::map<RigExecValueId, RigExecValueId> &replacements,
    RigExecValueId *output, std::string *error,
    RigExecProviderContextIndex *index)
{
    if (!program || !output || result >= program->valueKeys.size() ||
        program->ops.size() != program->descriptors.size())
        return Fail(error, "invalid provider context clone binding");
    RigExecProviderContextIndex local;
    if (!index)
        index = &local;
    index->Extend(*program);
    const auto &producer = index->producers;
    const auto &external = index->externals;
    std::map<RigExecValueId, RigExecValueId> remap;
    std::function<RigExecValueId(RigExecValueId)> clone =
        [&](RigExecValueId value) -> RigExecValueId {
        if (value == RigExecNoProviderValue)
            return value;
        const auto replacement = replacements.find(value);
        if (replacement != replacements.end())
            return replacement->second;
        const auto existing = remap.find(value);
        if (existing != remap.end())
            return existing->second;
        const auto frame = external.find(value);
        const auto operation = producer.find(value);
        if (frame != external.end() &&
            !externalNames.count(
                program->externalInputs[frame->second].computation))
            return value;
        if (frame == external.end() && operation == producer.end())
            return value;
        const RigExecValueId next = RigExecValueId(program->valueKeys.size());
        const std::string key = prefix + program->valueKeys[size_t(value)];
        const auto already = program->valueIds.find(key);
        if (already != program->valueIds.end()) {
            remap.emplace(value, already->second);
            return already->second;
        }
        remap.emplace(value, next);
        program->valueKeys.push_back(key);
        program->valueIds.emplace(key, next);
        if (frame != external.end()) {
            auto input = program->externalInputs[frame->second];
            input.value = next;
            input.computation = externalNames.at(input.computation);
            program->externalInputs.push_back(std::move(input));
            return next;
        }
        auto op = program->ops[operation->second];
        auto descriptor = program->descriptors[operation->second];
        op.output = next;
        for (auto &input : op.inputs)
            input = clone(input);
        for (auto &xform : op.xforms)
            xform.raw = clone(xform.raw);
        descriptor.key = prefix + descriptor.key;
        descriptor.reads.clear();
        for (const auto input : op.inputs)
            if (input != RigExecNoProviderValue)
                descriptor.reads.push_back(input);
        std::sort(descriptor.reads.begin(), descriptor.reads.end());
        descriptor.reads.erase(
            std::unique(descriptor.reads.begin(), descriptor.reads.end()),
            descriptor.reads.end());
        descriptor.writes = {next};
        program->ops.push_back(std::move(op));
        program->descriptors.push_back(std::move(descriptor));
        return next;
    };
    *output = clone(result);
    return true;
}
namespace {
// The kinds RigExecRunProviderOp hands to RigExecRunProviderArithmetic.
bool
ArithmeticKind(RigExecProviderOpKind kind)
{
    return kind <= RigExecProviderOpKind::JointMatrix ||
           kind == RigExecProviderOpKind::AvarMatrix ||
           kind == RigExecProviderOpKind::RelativeXform;
}
// The cases of RigExecRunProviderArithmetic's switch: the only kinds for
// which it does not return false.
bool
ArithmeticHandles(RigExecProviderOpKind kind)
{
    switch (kind) {
    case RigExecProviderOpKind::Attribute:
    case RigExecProviderOpKind::SpaceExpression:
    case RigExecProviderOpKind::RestFrame:
    case RigExecProviderOpKind::DefaultSpace:
    case RigExecProviderOpKind::FrameToSpace:
    case RigExecProviderOpKind::MatrixToFrame:
    case RigExecProviderOpKind::PosedFrame:
    case RigExecProviderOpKind::AvarMatrix:
    case RigExecProviderOpKind::RelativeXform:
    case RigExecProviderOpKind::JointMatrix:
        return true;
    default:
        return false;
    }
}
} // namespace
bool
RigExecProviderOpStructurallyValid(const RigExecProviderProgram &program,
                                   uint32_t index, std::string *error)
{
    if (index >= program.ops.size())
        return Fail(error, "invalid provider kernel/store binding");
    const auto &op = program.ops[index];
    if (op.output >= program.valueKeys.size())
        return Fail(error, "provider output is outside its typed layout");
    if (ArithmeticKind(op.kind))
        return ArithmeticHandles(op.kind) ||
               Fail(error, "unsupported provider arithmetic operation kind");
    if (op.kind == RigExecProviderOpKind::AffineFrame) {
        if (op.affineKind >= affineTypeCount)
            return Fail(error, "unknown affine frame expression");
        if (!RigExecAffineFrameInputsMatch(op.affineKind, op.inputs.size(),
                                           op.affineTargets))
            return Fail(error,
                        "affine frame inputs do not match their expression");
        return true;
    }
    return Fail(error, "unknown provider operation kind");
}
bool
RigExecComposeLocalXform(const VtTokenArray &order,
                         const RigExecXformOpLookup &lookup,
                         bool missingIsIdentity, GfMatrix4d *local,
                         bool *resets, std::string *error, TfToken *failedOp)
{
    GfMatrix4d result(1.0);
    if (resets)
        *resets = false;
    for (size_t reverse = order.size(); reverse > 0; --reverse) {
        const auto &token = order[reverse - 1];
        std::string_view name = token.GetString();
        if (name == "!resetXformStack!") {
            if (resets)
                *resets = true;
            break;
        }
        const bool inverse = name.rfind("!invert!", 0) == 0;
        if (inverse)
            name.remove_prefix(8);
        if (reverse > 1) {
            std::string_view next = order[reverse - 2].GetString();
            const bool nextInverse = next.rfind("!invert!", 0) == 0;
            if (nextInverse)
                next.remove_prefix(8);
            if (name == next && inverse != nextInverse) {
                --reverse;
                continue;
            }
        }
        int type = -1;
        const VtValue *value = nullptr;
        if (!lookup.find(lookup.context, name, &type, &value)) {
            if (failedOp)
                *failedOp = token;
            return Fail(error, "xform order names an uncaptured operation");
        }
        if (!value || value->IsEmpty()) {
            if (missingIsIdentity)
                continue;
            if (failedOp)
                *failedOp = token;
            return Fail(error, "xform operation unavailable");
        }
        if (type < 0 && name.rfind("xformOp:", 0) != 0) {
            if (failedOp)
                *failedOp = token;
            return Fail(error, "unknown xform operation type");
        }
        if (type < 0) {
            const auto end = name.find(':', 8);
            type = int(XformOpType(
                name, 8, end == std::string_view::npos ? end : end - 8));
        }
        if (type == UsdGeomXformOp::TypeInvalid) {
            if (failedOp)
                *failedOp = token;
            return Fail(error, "unknown xform operation type");
        }
        const auto transform = UsdGeomXformOp::GetOpTransform(
            UsdGeomXformOp::Type(type), *value, inverse);
        if (transform != GfMatrix4d(1.0))
            result *= transform;
    }
    *local = result;
    return true;
}
bool
RigExecRunProviderOp(const RigExecProviderProgram &program, uint32_t index,
                     RigExecTypedValueStore *store, std::string *error)
{
    if (!store || index >= program.ops.size() ||
        store->values.size() < program.valueKeys.size())
        return Fail(error, "invalid provider kernel/store binding");
    if (!RigExecProviderOpStructurallyValid(program, index, error))
        return false;
    const auto &op = program.ops[index];
    if (op.kind != RigExecProviderOpKind::Attribute)
        return Fail(error, "the provider operation runs as an op node");
    GfProviderStore adapter{*store};
    return RigExecRunProviderArithmetic<GfProviderMath>(op, adapter) ||
           Fail(error, "unsupported provider arithmetic operation kind");
}

bool
RigExecSampleProviderProgram(const RigExecProviderProgram &program,
                             const RigExecSceneDescriptors &scene,
                             size_t identity,
                             const std::map<SdfPath, VtValue> &overlays,
                             RigExecTypedValueStore *store,
                             std::vector<RigExecValueId> *changed,
                             std::string *error)
{
    if (!store || !changed || identity >= scene.identities.size() ||
        store->values.size() < program.valueKeys.size())
        return Fail(error, "invalid provider sample identity/store binding");
    for (const auto &[path, value] : overlays) {
        const auto attribute = scene.attributes.find(path);
        if (attribute == scene.attributes.end() ||
            (!value.IsEmpty() &&
             value.GetTypeid() !=
                 attribute->second.fact.type.GetType().GetTypeid()))
            return Fail(error, "provider overlay native type mismatch: " +
                                   path.GetString());
    }
    for (const auto &leaf : program.sampled) {
        const auto attribute = scene.attributes.find(leaf.attribute);
        if (attribute == scene.attributes.end() ||
            identity >= attribute->second.inputs.size())
            return Fail(error, "provider source state missing");
    }
    changed->clear();
    for (const auto &leaf : program.sampled) {
        const auto attr = scene.attributes.find(leaf.attribute);
        if (attr == scene.attributes.end() ||
            identity >= attr->second.inputs.size())
            return Fail(error, "provider source state missing");
        const auto overlay = overlays.find(leaf.attribute);
        const auto &source = attr->second.inputs[identity];
        const auto &value =
            overlay == overlays.end() ? source.raw : overlay->second;
        if (!value.IsEmpty() &&
            value.GetTypeid() != attr->second.fact.type.GetType().GetTypeid())
            return Fail(error, "provider overlay native type mismatch: " +
                                   leaf.attribute.GetString());
        if (store->PublishSource(leaf.value, value,
                                 overlay == overlays.end() ? source.rawBlocked
                                                           : false,
                                 overlay != overlays.end()))
            changed->push_back(leaf.value);
    }
    return true;
}
} // namespace rigExec
