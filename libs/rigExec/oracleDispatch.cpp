#include "oracleDispatch.h"
#include "goldenSuite.h"

#include <atomic>
#include <cstdlib>
#include <stdexcept>

namespace rigExec {
namespace {
std::atomic<const RigExecOracleHooks *> g_hooks{nullptr};
static_assert(decltype(g_hooks)::is_always_lock_free,
              "Oracle dispatch must not acquire a lock");
const RigExecOracleHooks g_emptyHooks;
constexpr const char *g_missing =
    "RigExec oracle library is unavailable; link rigExec::rigExecOracle "
    "or call RigExecRegisterOracle() from rigExecOracle before using oracle checks";
const bool g_goldenSuiteRequested = [] {
    const char *value = std::getenv("RIGEXEC_GOLDEN_SUITE");
    return value && *value;
}();

bool Missing(std::string *error)
{
    if (error) *error = g_missing;
    return false;
}
}

void RigExecInstallOracleHooks(const RigExecOracleHooks *hooks)
{
    g_hooks.store(hooks, std::memory_order_release);
}

bool RigExecOracleAvailable(std::string *error)
{
    return g_hooks.load(std::memory_order_acquire) || Missing(error);
}

const RigExecOracleHooks &RigExecOracleInstalledHooks()
{
    const auto *hooks = g_hooks.load(std::memory_order_acquire);
    return hooks ? *hooks : g_emptyHooks;
}

void RigExecOracleMissing(const char *feature)
{
    throw std::runtime_error(std::string(g_missing) + "; " + feature + " cannot run");
}

namespace {
template <class Fn>
Fn Hook(Fn fn, const char *feature)
{
    if (!fn) RigExecOracleMissing(feature);
    return fn;
}
}

RigExecOracleScene RigExecDispatchCaptureOracleInputs(
    const UsdStageRefPtr &stage, const RigExecResolvedInputs &resolved,
    UsdTimeCode time, size_t pointCount, const std::vector<SdfPath> &roots,
    const std::map<SdfPath, VtValue> &upstream, RigExecOracleCaptureMode mode)
{
    return Hook(RigExecOracleInstalledHooks().captureInputs, "cpu reference")(
        stage, resolved, time, pointCount, roots, upstream, mode);
}

RigExecWeightReferenceContext RigExecDispatchCaptureWeightReference(
    const UsdStageRefPtr &stage, const SdfPath &root,
    const RigExecResolvedInputs &resolved, const std::map<SdfPath, VtValue> &upstream,
    UsdTimeCode time, const std::function<const GfMatrix4d *(const SdfPath &)> &placement)
{
    return Hook(RigExecOracleInstalledHooks().captureWeight, "cpu reference")(
        stage, root, resolved, upstream, time, placement);
}

bool RigExecEncodeGoldenValue(const VtValue &value, std::string *encoded, std::string *error)
{
    const auto encode = RigExecOracleInstalledHooks().encodeValue;
    return encode ? encode(value, encoded, error) : Missing(error);
}

bool RigExecEncodeGoldenPose(const RigExecRigPose &pose,
    std::vector<RigExecGoldenValue> *values, std::string *error, bool includeOracle)
{
    const auto encode = RigExecOracleInstalledHooks().encodePose;
    return encode ? encode(pose, values, error, includeOracle) : Missing(error);
}

std::string RigExecGoldenDouble(double value)
{
    return Hook(RigExecOracleInstalledHooks().goldenDouble, "golden pose")(value);
}

std::string RigExecGoldenEscape(const std::string &value)
{
    return Hook(RigExecOracleInstalledHooks().goldenEscape, "golden pose")(value);
}

uint64_t RigExecGoldenDigest(const std::string &bytes)
{
    return Hook(RigExecOracleInstalledHooks().goldenDigest, "golden pose")(bytes);
}

std::string RigExecGoldenHex(uint64_t value)
{
    return Hook(RigExecOracleInstalledHooks().goldenHex, "golden pose")(value);
}

std::string RigExecGoldenVisit(const std::string &leg, size_t ordinal,
    const RigExecRigPose &pose, const std::vector<RigExecGoldenValue> &values,
    bool digestDomains)
{
    return Hook(RigExecOracleInstalledHooks().goldenVisit, "golden pose")(
        leg, ordinal, pose, values, digestDomains);
}

bool RigExecCompareGolden(const std::string &expected, const std::string &actual,
    std::string *error)
{
    const auto compare = RigExecOracleInstalledHooks().compareGolden;
    return compare ? compare(expected, actual, error) : Missing(error);
}

std::unique_ptr<RigExecGoldenSuiteObserver>
RigExecGoldenSuiteObserver::Create(const SdfPath &rigPath)
{
    const auto create = RigExecOracleInstalledHooks().createGolden;
    if (!create) {
        if (g_goldenSuiteRequested) RigExecOracleMissing("golden suite");
        return nullptr;
    }
    return create(rigPath);
}

void RigExecFinalizeGoldenSuite()
{
    const auto finalize = RigExecOracleInstalledHooks().finalizeGolden;
    if (finalize) {
        finalize();
        return;
    }
    if (g_goldenSuiteRequested) RigExecOracleMissing("golden suite");
}
}
