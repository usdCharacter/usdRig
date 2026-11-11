#include "rigExecSchema/schemaNames.h"
#include "jointGroupTranslation.h"
#include "sceneCompileInputs.h"
#include "rigExec/pathText.h"

#include "pxr/base/tf/staticTokens.h"

#include <algorithm>
#include <set>

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (RigExecJointGroupAPI) (explicitOnly)
    ((jointGroups, SchemaAttrText::rigExecJointGroups))
    ((includes, CollectionMemberText::includes))
    ((excludes, CollectionMemberText::excludes))
    ((expansionRule, CollectionMemberText::expansionRule))
    ((expression, CollectionMemberText::membershipExpression))
    ((enabled, SchemaAttrText::rigExecEnabled))
    ((activeMember, SchemaAttrText::rigExecActiveMember))
);

namespace {

std::string
Text(const SdfPath &path)
{
    std::string text;
    if (!RigExecSpellPathText(path, &text))
        text = path.GetAsString();
    return text;
}

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

// A structural attribute's value at Default; refused when it is animated.
template <class T>
bool
Uniform(const RigExecSceneCompileInputs &source, const SdfPath &property,
        const T &fallback, T *value, std::string *error)
{
    *value = fallback;
    const auto *attribute = source.Attribute(property);
    if (!attribute)
        return true;
    if (!attribute->fact.sampleTimes.empty() ||
        !attribute->fact.spline.IsEmpty() ||
        attribute->fact.mightBeTimeVarying) {
        return Fail(error, Text(property) +
                               " is a setup value and must not be animated "
                               "(an edit recompiles the solver)");
    }
    RigExecSceneBoundInput binding;
    VtValue boxed;
    if (!source.Bind(property, RigExecSceneReadRoute::Raw, &binding, error) ||
        !source.Read(binding, UsdTimeCode::Default(), &boxed, nullptr, error)) {
        return false;
    }
    if (boxed.IsHolding<T>())
        *value = boxed.UncheckedGet<T>();
    return true;
}

} // namespace

bool
RigExecTranslateSceneJointGroups(
    const RigExecSceneDescriptors &scene, const SdfPath &solver,
    const std::map<SdfPath, int32_t> &jointIndex, const std::string &who,
    std::vector<RigExecJointGroup> *groups, std::vector<int32_t> *members,
    std::vector<RigExecSceneTypedRead> *selectorReads, std::string *error)
{
    groups->clear();
    members->clear();
    selectorReads->clear();
    const RigExecSceneCompileInputs source(scene);
    std::set<SdfPath> grouped;
    for (const SdfPath &groupPath :
         source.Targets(solver.AppendProperty(_tokens->jointGroups))) {
        const auto *node = source.Node(groupPath);
        if (!node || !node->fact.active ||
            std::find(node->fact.appliedSchemas.begin(),
                      node->fact.appliedSchemas.end(),
                      _tokens->RigExecJointGroupAPI) ==
                node->fact.appliedSchemas.end()) {
            return Fail(
                error, who + ": rigExec:jointGroups target " + Text(groupPath) +
                           " is not an active prim with RigExecJointGroupAPI");
        }
        TfToken expansion;
        if (!Uniform(source, groupPath.AppendProperty(_tokens->expansionRule),
                     _tokens->explicitOnly, &expansion, error)) {
            return false;
        }
        if (expansion != _tokens->explicitOnly) {
            return Fail(
                error, who + ": group " + Text(groupPath) +
                           " must list its members explicitly "
                           "(collection:members:expansionRule = explicitOnly)");
        }
        if (const auto *expression = source.Attribute(
                groupPath.AppendProperty(_tokens->expression))) {
            if (expression->fact.hasAuthoredValue) {
                return Fail(
                    error,
                    who + ": group " + Text(groupPath) +
                        " authors collection:members:membershipExpression; "
                        "a joint group lists its members explicitly");
            }
        }
        const SdfPathVector excluded =
            source.Targets(groupPath.AppendProperty(_tokens->excludes));
        RigExecJointGroup group;
        group.memberBegin = uint32_t(members->size());
        for (const SdfPath &member :
             source.Targets(groupPath.AppendProperty(_tokens->includes))) {
            if (std::find(excluded.begin(), excluded.end(), member) !=
                excluded.end()) {
                continue;
            }
            const auto found = jointIndex.find(member);
            if (found == jointIndex.end()) {
                return Fail(error, who + ": group " + Text(groupPath) +
                                       " has member " + Text(member) +
                                       ", which is not a joint of this solver");
            }
            if (!grouped.insert(member).second) {
                return Fail(error, who + ": " + Text(member) +
                                       " is a member of two joint groups");
            }
            members->push_back(found->second);
        }
        group.memberCount = uint32_t(members->size()) - group.memberBegin;
        if (!Uniform(source, groupPath.AppendProperty(_tokens->enabled), true,
                     &group.enabled, error)) {
            return false;
        }
        RigExecSceneTypedRead activeRead;
        if (!RigExecBindSceneTypedRead(
                scene, groupPath.AppendProperty(_tokens->activeMember),
                SdfValueTypeNames->Double, &activeRead, error)) {
            return false;
        }
        selectorReads->push_back(std::move(activeRead));
        groups->push_back(group);
    }
    return true;
}

} // namespace rigExec
