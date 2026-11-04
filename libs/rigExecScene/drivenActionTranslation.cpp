#include "drivenActionTranslation.h"
#include "sceneCompileInputs.h"
#include "rigExec/pathText.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/types.h"

#include <cmath>
#include <utility>

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (fromSamples) (positive) (negative) (earlier) (later) (offset) (value)
    ((rigExecDriverAttributes, "rigExec:driverAttributes"))
    ((rigExecDriverSamples, "rigExec:driverSamples"))
    ((rigExecSide, "rigExec:side"))
    ((rigExecFrameTies, "rigExec:frameTies"))
    ((rigExecSamples, "rigExec:samples"))
    ((rigExecChannelMode, "rigExec:channelMode"))
    ((inputsEnabled, "inputs:enabled"))
    ((outputsFrame, "outputs:frame"))
    ((outputsWeight, "outputs:weight"))
    ((outputsActive, "outputs:active"))
    ((outputsValue, "outputs:value"))
);

namespace {

std::string
Text(const SdfPath &path)
{
    std::string text;
    if (!RigExecSpellPathText(path, &text)) text = path.GetAsString();
    return text;
}

bool
Fail(std::string *error, const std::string &message)
{
    if (error) *error = message;
    return false;
}

// An authored value at Default, or \p fallback when the attribute is not
// captured or holds another type.
template <class T>
bool
Raw(const RigExecSceneCompileInputs &source, const SdfPath &path,
    const T &fallback, T *value, std::string *error)
{
    *value = fallback;
    if (!source.Attribute(path)) return true;
    RigExecSceneBoundInput binding;
    VtValue boxed;
    if (!source.Bind(path, RigExecSceneReadRoute::Raw, &binding, error) ||
        !source.Read(binding, UsdTimeCode::Default(), &boxed, nullptr, error)) {
        return false;
    }
    if (boxed.IsHolding<T>()) *value = boxed.UncheckedGet<T>();
    return true;
}

bool
Finite(const VtDoubleArray &values)
{
    for (const double v : values) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}

// An output is published by the action; a connection authored on it would
// be a second, ignored source.
bool
Unconnected(const RigExecSceneCompileInputs &source, const SdfPath &output,
            std::string *error)
{
    if (const auto *attribute = source.Attribute(output)) {
        if (attribute->fact.hasAuthoredConnections) {
            return Fail(error, "driven action output carries authored connections: " +
                                   Text(output));
        }
    }
    return true;
}

}  // namespace

bool
RigExecTranslateSceneDrivenAction(const RigExecSceneDescriptors &scene,
    const SdfPath &path, RigExecSceneDrivenActionDescriptor *output,
    std::string *error)
{
    if (!output) return Fail(error, "null driven action descriptor");
    *output = RigExecSceneDrivenActionDescriptor();
    const RigExecSceneCompileInputs source(scene);
    const auto *node = source.Node(path);
    if (!node || !node->fact.active || node->fact.type != "RigExecDrivenAction") {
        return Fail(error, "missing active driven action: " + Text(path));
    }
    RigExecSceneDrivenActionDescriptor result;
    result.path = path;
    const auto property = [&](const TfToken &name) { return path.AppendProperty(name); };

    // The driver: exactly one float or double property, read in its own
    // precision (a double read of a float answers only its fallback).
    const SdfPathVector drivers = source.Targets(property(_tokens->rigExecDriverAttributes));
    if (drivers.size() != 1) {
        return Fail(error, Text(path) + ": rigExec:driverAttributes must name exactly "
                           "one float or double property, not " +
                           std::to_string(drivers.size()));
    }
    const SdfPath &driver = drivers[0];
    const auto *driverAttribute = driver.IsPropertyPath() ? source.Attribute(driver) : nullptr;
    if (!driverAttribute) {
        return Fail(error, Text(path) + ": rigExec:driverAttributes target is not an "
                           "attribute: " + Text(driver));
    }
    const auto driverType = driverAttribute->fact.type.GetType();
    result.driverIsFloat = driverType == SdfValueTypeNames->Float.GetType();
    if (!result.driverIsFloat && driverType != SdfValueTypeNames->Double.GetType()) {
        return Fail(error, Text(path) + ": rigExec:driverAttributes target is not a "
                           "float or double: " + Text(driver));
    }
    result.driverAttribute = driver;
    if (!RigExecBindSceneTypedRead(scene, driver,
            result.driverIsFloat ? SdfValueTypeNames->Float : SdfValueTypeNames->Double,
            &result.driver, error) ||
        !RigExecBindSceneTypedRead(scene, property(_tokens->inputsEnabled),
            SdfValueTypeNames->Bool, &result.enabled, error)) {
        return false;
    }

    // Structural table: samples, side, wrap and tie rule.
    auto table = std::make_shared<RigExecDrivenActionTable>();
    VtDoubleArray samples;
    TfToken side, ties;
    if (!Raw(source, property(_tokens->rigExecDriverSamples), VtDoubleArray{}, &samples, error) ||
        !Raw(source, property(_tokens->rigExecSide), _tokens->fromSamples, &side, error) ||
        !Raw(source, property(_tokens->rigExecFrameTies), _tokens->earlier, &ties, error)) {
        return false;
    }
    if (samples.size() < 2) {
        return Fail(error, Text(path) + ": rigExec:driverSamples needs at least two "
                           "samples (action frames 0 and 1)");
    }
    if (!Finite(samples)) {
        return Fail(error, Text(path) + ": rigExec:driverSamples is not finite");
    }
    table->driver.assign(samples.begin(), samples.end());
    if (side == _tokens->positive) {
        table->side = 1;
    } else if (side == _tokens->negative) {
        table->side = -1;
    } else if (side == _tokens->fromSamples || side.IsEmpty()) {
        bool disagree = false;
        table->side = RigExecDrivenActionSide(table->driver.data(), table->driver.size(),
                                             &disagree);
        if (disagree) {
            result.notes.push_back("note: driven action " + Text(path) +
                " first moves to the other side of sample 0; it serves the side of "
                "its last sample (" + std::string(table->side > 0 ? "positive" : "negative") +
                ")");
        }
        if (table->side == 0) {
            result.notes.push_back("warning: driven action " + Text(path) +
                " never leaves sample 0 at frame 1 or at its end; it stays at frame 0");
        }
    } else {
        return Fail(error, Text(path) + ": unknown rigExec:side " + side.GetString());
    }
    if (ties == _tokens->later) {
        table->tiesLater = true;
    } else if (ties != _tokens->earlier && !ties.IsEmpty()) {
        return Fail(error, Text(path) + ": unknown rigExec:frameTies " + ties.GetString());
    }
    table->fullTurn = RigExecDrivenActionFullTurn(table->driver.data(), table->driver.size());

    // Outputs: frame, weight, active, then one value per channel child in
    // composed child order.
    for (const TfToken &name : {_tokens->outputsFrame, _tokens->outputsWeight,
                                _tokens->outputsActive}) {
        const SdfPath out = property(name);
        if (!Unconnected(source, out, error)) return false;
        result.outputs.push_back(out);
    }
    for (const SdfPath &childPath : node->fact.children) {
        const auto *child = source.Node(childPath);
        if (!child || !child->fact.active || child->fact.type != "RigExecActionChannel") {
            continue;
        }
        const auto at = [&](const TfToken &name) { return childPath.AppendProperty(name); };
        VtDoubleArray values;
        TfToken mode;
        if (!Raw(source, at(_tokens->rigExecSamples), VtDoubleArray{}, &values, error) ||
            !Raw(source, at(_tokens->rigExecChannelMode), _tokens->offset, &mode, error)) {
            return false;
        }
        if (values.empty()) {
            return Fail(error, Text(childPath) + ": action channel has no rigExec:samples");
        }
        if (!Finite(values)) {
            return Fail(error, Text(childPath) + ": action channel rigExec:samples is not finite");
        }
        if (mode != _tokens->offset && mode != _tokens->value && !mode.IsEmpty()) {
            return Fail(error, Text(childPath) + ": unknown rigExec:channelMode " +
                                   mode.GetString());
        }
        const SdfPath out = at(_tokens->outputsValue);
        if (!Unconnected(source, out, error)) return false;
        table->channelSamples.emplace_back(values.begin(), values.end());
        table->channelOffset.push_back(mode == _tokens->value ? 0 : 1);
        result.outputs.push_back(out);
    }
    result.table = std::move(table);
    *output = std::move(result);
    return true;
}

}  // namespace rigExec
