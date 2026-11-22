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

} // namespace

static void
Register(rigExec::RigExecTypeRegistrar &registrar)
{
    namespace S = rigExecStd;
    static Fields landmarkFields = {&S::Vec3dType(), &S::Vec3dType(),
                                    &S::Vec3dType(), &S::Vec3dType()};
    static Fields frameFields = {&PointLandmarksType(), &S::UIntType()};
    static Fields frameArrayFields = {&PointFrameList(), &PointLandmarksList()};
    static Fields weightFields = {&S::FloatArray(), &S::IntArray(),
                                  &S::FloatType(),  &S::UIntType(),
                                  &S::UIntType(),   &S::BoolType()};
    static Fields curveFields = {&S::FloatArray()};
    static Fields meshFields = {&S::IntArray(), &S::IntArray(), &S::UIntType()};
    static Fields skinFields = {&S::IntArray(), &S::FloatArray(),
                                &S::UIntType(), &S::UIntType(), &S::UIntType()};
    static Fields blendFields = {&S::Vec3fArray(), &S::IntArray(),
                                 &S::UIntArray(), &S::UIntArray(),
                                 &S::UIntType()};
    static Fields collisionFields = {&S::IntArray(), &S::UIntType()};
    static Fields nurbsFields = {&S::DoubleArray(), &S::Vec3dArray(),
                                 &S::DoubleArray(), &S::UIntType()};

    struct Entry {
        const RigExecTypeHandle &handle;
        RigExecTypeDesc desc;
    };
    // Built once, on the owner thread, during freeze.
    static const Entry entries[] = {
        {PointLandmarksType(), TypeDescFor<PointLandmarks>(
                                   "rig", "PointLandmarks", nullptr,
                                   landmarkFields, std::size(landmarkFields))},
        {PointLandmarksList(),
         TypeDescFor<RigExecArray<PointLandmarks>>("rig", "PointLandmarksList",
                                                   &PointLandmarksType())},
        {PointFrameType(),
         TypeDescFor<PointFrame>("rig", "PointFrame", nullptr, frameFields,
                                 std::size(frameFields))},
        {PointFrameList(), TypeDescFor<RigExecArray<PointFrame>>(
                               "rig", "PointFrameList", &PointFrameType())},
        {PointFrameArrayType(),
         TypeDescFor<PointFrameArray>("rig", "PointFrameArray", nullptr,
                                      frameArrayFields,
                                      std::size(frameArrayFields))},
        {PointWeightsType(),
         TypeDescFor<PointWeights>("rig", "PointWeights", nullptr, weightFields,
                                   std::size(weightFields))},
        {ScalarCurveType(),
         TypeDescFor<ScalarCurve>("rig", "ScalarCurve", nullptr, curveFields,
                                  std::size(curveFields))},
        {MeshTopologyType(),
         TypeDescFor<MeshTopology>("rig", "MeshTopology", nullptr, meshFields,
                                   std::size(meshFields))},
        {SkinInfluencesType(),
         TypeDescFor<SkinInfluences>("rig", "SkinInfluences", nullptr,
                                     skinFields, std::size(skinFields))},
        {BlendShapeTargetsType(),
         TypeDescFor<BlendShapeTargets>("rig", "BlendShapeTargets", nullptr,
                                        blendFields, std::size(blendFields))},
        {CollisionShapesType(),
         TypeDescFor<CollisionShapes>("rig", "CollisionShapes", nullptr,
                                      collisionFields,
                                      std::size(collisionFields))},
        {NurbsCurveType(),
         TypeDescFor<NurbsCurve>("rig", "NurbsCurve", nullptr, nurbsFields,
                                 std::size(nurbsFields))},
    };
    for (const Entry &entry : entries)
        registrar.Add(entry.handle, entry.desc);
}

} // namespace rigExecRig

RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecRigTypes)
{
    static const rigExec::RigExecTypeLibraryDesc *const required[] = {
        rigExecStdTypes_RigExecTypeLibrary()};
    static const rigExec::RigExecTypeLibraryDesc library = [] {
        rigExec::RigExecTypeLibraryDesc d;
        d.name = "rigExecRigTypes";
        d.required = required;
        d.requiredCount = std::size(required);
        d.registerTypes = &rigExecRig::Register;
        return d;
    }();
    return &library;
}
