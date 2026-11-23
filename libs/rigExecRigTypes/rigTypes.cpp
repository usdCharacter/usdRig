#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/valueOps.h"

#include <iterator>

namespace rigExecRig {

RIGEXEC_DEFINE_TYPE_HANDLE(PointLandmarksType)
RIGEXEC_DEFINE_TYPE_HANDLE(PointLandmarksList)
RIGEXEC_DEFINE_TYPE_HANDLE(PointFrameType)
RIGEXEC_DEFINE_TYPE_HANDLE(PointFrameList)
RIGEXEC_DEFINE_TYPE_HANDLE(PointFrameArrayType)
RIGEXEC_DEFINE_TYPE_HANDLE(PointWeightsType)
RIGEXEC_DEFINE_TYPE_HANDLE(ScalarCurveType)
RIGEXEC_DEFINE_TYPE_HANDLE(MeshTopologyType)
RIGEXEC_DEFINE_TYPE_HANDLE(SkinInfluencesType)
RIGEXEC_DEFINE_TYPE_HANDLE(BlendShapeTargetsType)
RIGEXEC_DEFINE_TYPE_HANDLE(CollisionShapesType)
RIGEXEC_DEFINE_TYPE_HANDLE(NurbsCurveType)

namespace {

using rigExec::RigExecTypeDesc;
using rigExec::RigExecTypeHandle;
using rigExecStd::TypeDescFor;

using Fields = const RigExecTypeHandle *const[];

namespace S = rigExecStd;

Fields kLandmarkFields = {&S::Vec3dType(), &S::Vec3dType(), &S::Vec3dType(),
                          &S::Vec3dType()};
Fields kFrameFields = {&PointLandmarksType(), &S::UIntType()};
Fields kFrameArrayFields = {&PointFrameList(), &PointLandmarksList()};
Fields kWeightFields = {&S::FloatArray(), &S::IntArray(), &S::FloatType(),
                        &S::UIntType(),   &S::UIntType(), &S::BoolType()};
Fields kCurveFields = {&S::FloatArray()};
Fields kMeshFields = {&S::IntArray(), &S::IntArray(), &S::UIntType()};
Fields kSkinFields = {&S::IntArray(), &S::FloatArray(), &S::UIntType(),
                      &S::UIntType(), &S::UIntType()};
Fields kBlendFields = {&S::Vec3fArray(), &S::IntArray(), &S::UIntArray(),
                       &S::UIntArray(), &S::UIntType()};
Fields kCollisionFields = {&S::IntArray(), &S::UIntType()};
Fields kNurbsFields = {&S::DoubleArray(), &S::Vec3dArray(), &S::DoubleArray(),
                       &S::UIntType()};

struct Entry {
    const RigExecTypeHandle &handle;
    RigExecTypeDesc desc;
};

const Entry kEntries[] = {
    {PointLandmarksType(),
     TypeDescFor<PointLandmarks>("rig", "PointLandmarks", nullptr,
                                 kLandmarkFields, std::size(kLandmarkFields))},
    {PointLandmarksList(),
     TypeDescFor<RigExecArray<PointLandmarks>>("rig", "PointLandmarksList",
                                               &PointLandmarksType())},
    {PointFrameType(),
     TypeDescFor<PointFrame>("rig", "PointFrame", nullptr, kFrameFields,
                             std::size(kFrameFields))},
    {PointFrameList(), TypeDescFor<RigExecArray<PointFrame>>(
                           "rig", "PointFrameList", &PointFrameType())},
    {PointFrameArrayType(),
     TypeDescFor<PointFrameArray>("rig", "PointFrameArray", nullptr,
                                  kFrameArrayFields,
                                  std::size(kFrameArrayFields))},
    {PointWeightsType(),
     TypeDescFor<PointWeights>("rig", "PointWeights", nullptr, kWeightFields,
                               std::size(kWeightFields))},
    {ScalarCurveType(),
     TypeDescFor<ScalarCurve>("rig", "ScalarCurve", nullptr, kCurveFields,
                              std::size(kCurveFields))},
    {MeshTopologyType(),
     TypeDescFor<MeshTopology>("rig", "MeshTopology", nullptr, kMeshFields,
                               std::size(kMeshFields))},
    {SkinInfluencesType(),
     TypeDescFor<SkinInfluences>("rig", "SkinInfluences", nullptr, kSkinFields,
                                 std::size(kSkinFields))},
    {BlendShapeTargetsType(),
     TypeDescFor<BlendShapeTargets>("rig", "BlendShapeTargets", nullptr,
                                    kBlendFields, std::size(kBlendFields))},
    {CollisionShapesType(), TypeDescFor<CollisionShapes>(
                                "rig", "CollisionShapes", nullptr,
                                kCollisionFields, std::size(kCollisionFields))},
    {NurbsCurveType(),
     TypeDescFor<NurbsCurve>("rig", "NurbsCurve", nullptr, kNurbsFields,
                             std::size(kNurbsFields))},
};

void
Register(rigExec::RigExecTypeRegistrar &registrar)
{
    for (const Entry &entry : kEntries)
        registrar.Add(entry.handle, entry.desc);
}

const rigExec::RigExecTypeLibraryDesc *const kRequired[] = {
    rigExecStdTypes_RigExecTypeLibrary()};

const rigExec::RigExecTypeLibraryDesc kLibrary = [] {
    rigExec::RigExecTypeLibraryDesc d;
    d.name = "rigExecRigTypes";
    d.required = kRequired;
    d.requiredCount = std::size(kRequired);
    d.registerTypes = &Register;
    return d;
}();

} // namespace

} // namespace rigExecRig

RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecRigTypes)
{
    return &rigExecRig::kLibrary;
}
