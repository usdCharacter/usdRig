// Function pointers the scene-translation library installs. The evaluator calls
// the dispatch functions below; those forward here. A missing hook aborts, so
// a compile cannot succeed without the library. The frame-path kernels live
// in librigExec and are not dispatched.
#ifndef RIGEXEC_SCENE_DISPATCH_H
#define RIGEXEC_SCENE_DISPATCH_H

#include "rigExecScene/autoClavicleGraph.h"
#include "rigExecScene/constraintSceneTranslation.h"
#include "rigExecScene/geometrySceneTranslation.h"
#include "rigExecScene/poseSceneTranslation.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/solverSceneTranslation.h"

#include "pxr/usd/usd/stage.h"

#include <map>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecSceneHooks {
    bool (*capture)(const UsdStageRefPtr &, const SdfPath &,
                    const std::vector<UsdTimeCode> &, RigExecSceneDescriptors *,
                    std::string *) = nullptr;
    bool (*select)(const RigExecSceneDescriptors &, const std::vector<UsdTimeCode> &,
                   RigExecSceneDescriptors *, std::string *) = nullptr;
    bool (*translatePoseInterpolator)(const RigExecSceneDescriptors &, const SdfPath &,
                                  RigExecScenePoseInterpolatorDescriptor *,
                                  std::string *) = nullptr;
    bool (*translateSpaceSwitch)(const RigExecSceneDescriptors &, const SdfPath &,
                             RigExecSceneSpaceSwitchDescriptor *, std::string *) = nullptr;
    bool (*translateGeometry)(const RigExecSceneDescriptors &, const SdfPath &, const SdfPath &,
                          RigExecSceneGeometryDescriptor *, std::string *,
                          const RigExecGeometryRecord *) = nullptr;
    bool (*translateSolver)(const RigExecSceneDescriptors &, const SdfPath &,
                        const std::map<SdfPath, RigExecPointFrame> &,
                        RigExecSceneSolverDescriptor *, std::string *) = nullptr;
    bool (*translateConstraint)(const RigExecSceneDescriptors &, const SdfPath &,
                            const SdfPathVector &, RigExecSceneConstraintDescriptor *,
                            std::string *) = nullptr;
    bool (*bindAutoClavicle)(const RigExecSceneDescriptors &, const SdfPath &,
                             const RigExecSceneGraphBindingContext &, RigExecValueId,
                             RigExecValueId, RigExecBoundAutoClavicle *,
                             std::string *) = nullptr;
    bool (*runAutoClavicle)(const RigExecBoundAutoClavicle &, RigExecTypedValueStore *,
                            std::string *) = nullptr;
};

void RigExecInstallSceneHooks(const RigExecSceneHooks &hooks);
[[noreturn]] void RigExecSceneMissing(const char *feature);

bool RigExecDispatchCaptureSceneDescriptors(
    const UsdStageRefPtr &stage, const SdfPath &rigRoot,
    const std::vector<UsdTimeCode> &identities, RigExecSceneDescriptors *result,
    std::string *error);

bool RigExecDispatchSelectSceneDescriptorIdentities(
    const RigExecSceneDescriptors &scene, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *result, std::string *error);

bool RigExecDispatchTranslateScenePoseInterpolator(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *descriptor, std::string *error);

bool RigExecDispatchTranslateSceneSpaceSwitch(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *descriptor, std::string *error);

bool RigExecDispatchTranslateSceneGeometry(
    const RigExecSceneDescriptors &scene, const SdfPath &mover, const SdfPath &target,
    RigExecSceneGeometryDescriptor *descriptor, std::string *error,
    const RigExecGeometryRecord *normalized);

bool RigExecDispatchTranslateSceneSolver(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const std::map<SdfPath, RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *descriptor, std::string *error);

bool RigExecDispatchTranslateSceneConstraint(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const SdfPathVector &targets, RigExecSceneConstraintDescriptor *descriptor,
    std::string *error);

bool RigExecDispatchBindAutoClavicle(
    const RigExecSceneDescriptors &scene, const SdfPath &path,
    const RigExecSceneGraphBindingContext &context, RigExecValueId incoming,
    RigExecValueId output, RigExecBoundAutoClavicle *operation, std::string *error);

bool RigExecDispatchRunAutoClavicle(
    const RigExecBoundAutoClavicle &operation, RigExecTypedValueStore *values,
    std::string *error);

}
#endif
