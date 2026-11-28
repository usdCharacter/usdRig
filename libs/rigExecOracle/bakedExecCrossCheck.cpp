#include "rigExec/internal/bakedExecCrossCheck.h"
#include "rigExec/api/goldenPose.h"
#include <algorithm>
#include <set>
#include <tuple>

namespace rigExec {
namespace {
using Key = std::tuple<SdfPath, TfToken, TfToken>;
Key
OverrideKey(const RigExecValueOverride &value)
{
    return {value.prim,
            value.attribute.IsEmpty() ? value.computation : TfToken(),
            value.attribute};
}
Key
AddressKey(const RigExecValueAddress &address)
{
    return address.target.IsPropertyPath()
               ? Key{address.target.GetPrimPath(), TfToken(),
                     address.target.GetNameToken()}
               : Key{address.target, address.publicComputation, TfToken()};
}
} // namespace

bool
RigExecBakedExecCrossCheck::Add(RigExecExecCheckDescriptor descriptor,
                                std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    if (!_stage || descriptor.key.empty() ||
        (!descriptor.skipOnly && descriptor.address.target.IsEmpty()))
        return fail("invalid exec check descriptor");
    if (std::any_of(_rows.begin(), _rows.end(),
                    [&](const RigExecExecCheckDescriptor &row) {
                        return row.key == descriptor.key;
                    }))
        return fail("duplicate exec check key: " + descriptor.key);
    if (descriptor.skipOnly) {
        if (!descriptor.requiredOverrides.empty() || descriptor.project ||
            descriptor.acquireReference)
            return fail("skip-only exec check cannot declare a computation: " +
                        descriptor.key);
        _rows.push_back(std::move(descriptor));
        return true;
    }
    const Key output = AddressKey(descriptor.address);
    std::set<Key> required;
    for (const auto &address : descriptor.requiredOverrides) {
        if (address.target.IsEmpty() ||
            !required.insert(AddressKey(address)).second)
            return fail("invalid or duplicate required override: " +
                        descriptor.key);
        if (AddressKey(address) == output)
            return fail("exec check cannot override its own result: " +
                        descriptor.key);
    }
    if (descriptor.acquireReference && descriptor.project)
        return fail("independent reference cannot project a public result: " +
                    descriptor.key);
    _rows.push_back(std::move(descriptor));
    return true;
}

RigExecExecCrossCheckReport
RigExecBakedExecCrossCheck::Evaluate(
    UsdTimeCode time, const std::vector<RigExecExecCheckInput> &inputs)
{
    RigExecExecCrossCheckReport report;
    if (inputs.size() != _rows.size()) {
        ++report.failed;
        report.diagnostics.push_back(
            "exec check input count differs from compiled row count");
        return report;
    }
    for (size_t i = 0; i < _rows.size(); ++i) {
        const auto &row = _rows[i];
        const auto &input = inputs[i];
        if (input.skip == RigExecExecCheckSkip::MissingName) {
            ++report.failed;
            report.diagnostics.push_back(row.key + ": interned name missing");
            continue;
        }
        if (input.skip != RigExecExecCheckSkip::None) {
            ++report.skipped[input.skip];
            continue;
        }
        const auto fail = [&](const std::string &reason) {
            ++report.failed;
            report.diagnostics.push_back(row.key + ": " + reason);
        };
        if (row.skipOnly) {
            fail("skip-only row requires an explicit reason");
            continue;
        }
        if (!row.acquireReference) {
            ++report.skipped[RigExecExecCheckSkip::NoEquivalent];
            continue;
        }
        const Key output = AddressKey(row.address);
        std::set<Key> present;
        bool valid = true;
        for (const auto &override : input.overrides) {
            const Key key = OverrideKey(override);
            if (key == output || !present.insert(key).second) {
                fail("duplicate override or attempted result override");
                valid = false;
                break;
            }
        }
        if (!valid)
            continue;
        for (const auto &address : row.requiredOverrides)
            if (!present.count(AddressKey(address))) {
                fail("missing bound input override: " +
                     address.target.GetString());
                valid = false;
                break;
            }
        if (!valid)
            continue;
        if (present.size() != row.requiredOverrides.size()) {
            fail("independent reference received an undeclared input "
                 "override");
            continue;
        }
        RigExecExecCheckDescriptor::Reference reference;
        std::string referenceError;
        VtValue actual;
        if (!row.acquireReference(_stage, time, input.overrides, &reference,
                                  &referenceError) ||
            !reference) {
            fail("independent context acquisition failed: " + referenceError);
            continue;
        }
        if (!reference(&actual, &referenceError)) {
            fail("independent numerical witness failed: " + referenceError);
            continue;
        }
        std::string expectedText, actualText, error;
        if (!RigExecEncodeGoldenValue(input.expected, &expectedText, &error) ||
            !RigExecEncodeGoldenValue(actual, &actualText, &error)) {
            fail(error);
            continue;
        }
        ++report.checked;
        ++report.checkedByKind[row.kind];
        if (actualText != expectedText)
            fail("value differs\nexpected: " + expectedText +
                 "\nactual: " + actualText);
    }
    return report;
}
} // namespace rigExec
