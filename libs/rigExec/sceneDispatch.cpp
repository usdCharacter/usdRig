#include "sceneDispatch.h"

#include <atomic>
#include <utility>

namespace rigExec {
namespace {
std::atomic<const RigExecSceneHooks *> g_hooks{nullptr};
static_assert(decltype(g_hooks)::is_always_lock_free,
              "Scene dispatch must not acquire a lock");

bool Missing(std::string *error)
{
    if (error) {
        *error = "RigExec scene library is unavailable; link rigExec::rigExecScene "
                 "or call RigExecRegisterScene() from rigExecScene before compiling";
    }
    return false;
}

template <class Fn, class... Args>
bool Dispatch(Fn RigExecSceneHooks::*member, std::string *error, Args &&...args)
{
    const auto *hooks = g_hooks.load(std::memory_order_acquire);
    if (!hooks || !(hooks->*member)) return Missing(error);
    return (hooks->*member)(std::forward<Args>(args)...);
}
}

void RigExecInstallSceneHooks(const RigExecSceneHooks *hooks)
{
    g_hooks.store(hooks, std::memory_order_release);
}

bool RigExecSceneAvailable(std::string *error)
{
    return g_hooks.load(std::memory_order_acquire) || Missing(error);
}

bool RigExecDispatchCaptureSceneDescriptors(
    const UsdStageRefPtr &stage, const SdfPath &rigRoot,
    const std::vector<UsdTimeCode> &identities, RigExecSceneDescriptors *result,
    std::string *error)
{
    return Dispatch(&RigExecSceneHooks::capture, error,
        stage, rigRoot, identities, result, error);
}

bool RigExecDispatchSelectSceneDescriptorIdentities(
    const RigExecSceneDescriptors &scene, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error)
{
    return Dispatch(&RigExecSceneHooks::select, error, scene, identities, result, error);
}

bool RigExecDispatchTranslateScenePoseInterpolator(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *descriptor, std::string *error)
{
    return Dispatch(&RigExecSceneHooks::translatePoseInterpolator, error,
        scene, path, descriptor, error);
}

bool RigExecDispatchTranslateSceneSpaceSwitch(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *descriptor, std::string *error)
{
    return Dispatch(&RigExecSceneHooks::translateSpaceSwitch, error,
        scene, path, descriptor, error);
}

bool RigExecDispatchTranslateSceneGeometry(
    const RigExecSceneDescriptors &scene, const SdfPath &mover, const SdfPath &target,
    RigExecSceneGeometryDescriptor *descriptor, std::string *error,
    const RigExecGeometryRecord *normalized)
{
    return Dispatch(&RigExecSceneHooks::translateGeometry, error,
        scene, mover, target, descriptor, error, normalized);
}

bool RigExecDispatchTranslateSceneSolver(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const std::map<SdfPath, RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *descriptor, std::string *error)
{
    return Dispatch(&RigExecSceneHooks::translateSolver, error,
        scene, path, providerRests, descriptor, error);
}

bool RigExecDispatchTranslateSceneConstraint(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const SdfPathVector &targets, RigExecSceneConstraintDescriptor *descriptor,
    std::string *error)
{
    return Dispatch(&RigExecSceneHooks::translateConstraint, error,
        scene, path, targets, descriptor, error);
}

bool RigExecDispatchBindAutoClavicle(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const RigExecSceneGraphBindingContext &context, RigExecValueId incoming,
    RigExecValueId output, RigExecBoundAutoClavicle *operation, std::string *error)
{
    return Dispatch(&RigExecSceneHooks::bindAutoClavicle, error,
        scene, path, context, incoming, output, operation, error);
}

bool RigExecDispatchRunAutoClavicle(
    const RigExecBoundAutoClavicle &operation, RigExecTypedValueStore *values,
    std::string *error)
{
    return Dispatch(&RigExecSceneHooks::runAutoClavicle, error, operation, values, error);
}

}
