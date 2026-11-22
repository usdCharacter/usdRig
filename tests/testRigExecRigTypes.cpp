// The rig type library: records built from standard types register after
// them, construct to their defaults, copy deeply, compare memberwise
// (padding ignored, NaN modes honoured) and run through the core.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecRig;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

const RigExecTypeDesc &
Desc(const RigExecTypeHandle &handle)
{
    return *RigExecRegistry::Get().Type(handle).desc;
}

// FaceCount: MeshTopology -> UInt.
RIGEXEC_DEFINE_OP_HANDLE(FaceCount)
RIGEXEC_DEFINE_PORT_HANDLE(FaceCountIn)
RIGEXEC_DEFINE_PORT_HANDLE(FaceCountOut)

bool
RunFaceCount(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const auto &mesh =
        *static_cast<const MeshTopology *>(io.inputs[FaceCountIn().Index()]);
    *static_cast<uint32_t *>(io.outputs[FaceCountOut().Index()]) =
        uint32_t(mesh.faceVertexCounts.Size());
    return true;
}

void
RegisterOps(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&FaceCountIn(), &MeshTopologyType(), kRigExecPortInput, "mesh"},
        {&FaceCountOut(), &rigExecStd::UIntType(), kRigExecPortOutput, "count"},
    };
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "FaceCount";
        d.ports = ports;
        d.portCount = 2;
        d.run = &RunFaceCount;
        return d;
    }();
    registrar.Add(FaceCount(), desc);
}

const RigExecOpLibraryDesc kOps = [] {
    RigExecOpLibraryDesc d;
    d.name = "testRigTypesOps";
    d.registerOps = &RegisterOps;
    return d;
}();

const RigExecTypeHandle *const kRigTypes[] = {
    &PointLandmarksType(),    &PointLandmarksList(),  &PointFrameType(),
    &PointFrameList(),        &PointFrameArrayType(), &PointWeightsType(),
    &ScalarCurveType(),       &MeshTopologyType(),    &SkinInfluencesType(),
    &BlendShapeTargetsType(), &CollisionShapesType(), &NurbsCurveType(),
};

void
TestRegistered()
{
    std::set<uint64_t> layouts;
    for (const RigExecTypeHandle *handle : kRigTypes) {
        CHECK(handle->IsValid());
        layouts.insert(Desc(*handle).layoutHash);
        CHECK(std::strcmp(Desc(*handle).nameSpace, "rig") == 0);
    }
    CHECK(layouts.size() == std::size(kRigTypes));
    CHECK(Desc(PointFrameType()).size == sizeof(PointFrame));
    CHECK(Desc(PointFrameType()).triviallyCopyable);
    CHECK(Desc(PointFrameType()).construct);
    CHECK(!Desc(MeshTopologyType()).triviallyCopyable);
    CHECK(Desc(PointFrameList()).element == &PointFrameType());
    CHECK(&Points3fType() == &rigExecStd::Vec3fArray());
    // Standard types freeze first.
    CHECK(rigExecStd::Vec3dType().Index() < PointFrameType().Index());
}

void
TestPointFrame()
{
    const RigExecTypeDesc &d = Desc(PointFrameType());
    alignas(PointFrame) unsigned char a[sizeof(PointFrame)];
    alignas(PointFrame) unsigned char b[sizeof(PointFrame)];
    std::memset(a, 0x00, sizeof(a));
    std::memset(b, 0xff, sizeof(b));
    d.construct(a);
    d.construct(b);
    const auto &frame = *reinterpret_cast<const PointFrame *>(a);
    CHECK(frame.flags == kPointFrameValid);
    CHECK(frame.points[1]._data[0] == 1.0 && frame.points[3]._data[2] == 1.0);
    // Padding differs; the values do not.
    CHECK(d.equal(a, b, RigExecEqualMode::Bitwise));
    CHECK(d.hash(a) == d.hash(b));
    reinterpret_cast<PointFrame *>(b)->flags = kPointFrameDegenerate;
    CHECK(!d.equal(a, b, RigExecEqualMode::Bitwise));
}

void
TestRecords()
{
    const RigExecTypeDesc &d = Desc(PointFrameArrayType());
    PointFrameArray x;
    x.frames.Resize(3);
    x.rests.Resize(3);
    x.frames[1].points[0]._data[1] = NAN;
    PointFrameArray y;
    d.construct(&y);
    d.copy(&y, &x);
    x.frames[2].flags = kPointFrameAffine;
    CHECK(y.frames[2].flags == kPointFrameValid);
    x.frames[2].flags = kPointFrameValid;
    CHECK(d.equal(&x, &y, RigExecEqualMode::Bitwise));
    CHECK(d.equal(&x, &y, RigExecEqualMode::NaNEqual));
    CHECK(!d.equal(&x, &y, RigExecEqualMode::Ieee));
    CHECK(d.hash(&x) == d.hash(&y));
    y.rests.Resize(4);
    CHECK(!d.equal(&x, &y, RigExecEqualMode::NaNEqual));

    const RigExecTypeDesc &m = Desc(MeshTopologyType());
    MeshTopology quad;
    quad.faceVertexCounts = RigExecArray<int32_t>(1);
    quad.faceVertexCounts[0] = 4;
    quad.faceVertexIndices = RigExecArray<int32_t>(4);
    quad.pointCount = 4;
    MeshTopology other = quad;
    CHECK(m.equal(&quad, &other, RigExecEqualMode::Ieee));
    other.pointCount = 5;
    CHECK(!m.equal(&quad, &other, RigExecEqualMode::Ieee));
    CHECK(m.hash(&quad) != m.hash(&other));
}

void
TestEvaluate()
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId node = builder.AddOp(FaceCount());
    const RigExecInputId in = builder.BindInput(node, FaceCountIn());
    const RigExecOutputId out = builder.BindOutput(node, FaceCountOut());
    std::string error;
    auto desc = builder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator) {
        std::printf("%s\n", error.c_str());
        return;
    }
    MeshTopology mesh;
    mesh.faceVertexCounts = RigExecArray<int32_t>(7);
    CHECK(evaluator->SetInput(in, &mesh, &error));
    CHECK(evaluator->Evaluate(&error));
    CHECK(*static_cast<const uint32_t *>(evaluator->GetOutput(out)) == 7);
}

} // namespace

int
main()
{
    RigExecRegistryInit init = RigExecLinkedRegistrations();
    init.opLibraries.push_back(&kOps);
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(init, &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestRegistered();
    TestPointFrame();
    TestRecords();
    TestEvaluate();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
