// Op nodes in a .rigexec (format 35): a file holding a math chain feeding a
// Smooth and a VolumeCorrect over its inputs opens in the runtime, runs as
// the OpGraph step of its common graph, and publishes the same bits as the
// op core run directly and as the kernels; files with an unknown op, a
// damaged payload, a bad index, a cycle, a type mismatch or a missing
// library are refused at Open.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecBinary/format.h"
#include "rigExecBinary/manifest.h"
#include "rigExecBinary/opGraphWriter.h"
#include "rigExecMath/meshBoundsKernel.h"
#include "rigExecMath/propertyMathKernel.h"
#include "rigExecMath/smoothKernel.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/property/mathOps.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3f;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

bool
Contains(const std::string &text, const char *part)
{
    const bool found = text.find(part) != std::string::npos;
    if (!found)
        std::printf("  expected \"%s\" in: %s\n", part, text.c_str());
    return found;
}

using Points = RigExecArray<Vec3f>;

// The inputs, and the slots that hold them.
enum Slot : uint32_t { A, B, C, PointsSlot, Reference, W, Counts, Indices };
const char *const kSlotNames[] = {"a",         "b", "c",      "points",
                                  "reference", "w", "counts", "indices"};

struct Mesh {
    std::vector<Vec3f> points;
    std::vector<int> counts, indices;
};

/// A 3 x 3 quad grid, bumped so smoothing moves it.
Mesh
Grid()
{
    Mesh mesh;
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x)
            mesh.points.push_back(
                Vec3f(float(x), float(y), 0.25f * float((x * 7 + y * 3) % 5)));
    for (int y = 0; y < 3; ++y)
        for (int x = 0; x < 3; ++x) {
            mesh.counts.push_back(4);
            const int at = y * 4 + x;
            mesh.indices.insert(mesh.indices.end(),
                                {at, at + 1, at + 5, at + 4});
        }
    return mesh;
}

struct Values {
    float a = 0.25f, b = 0.375f, c = 1.5f, w = 0.75f;
    double reference = 2.5;
};

uint64_t
Bits(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

uint64_t
Bits(double value)
{
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

/// The smallest valid program, with the slots above as its inputs.
fb::RigExecWireFile
BaseFile(const Values &values, const Mesh &mesh)
{
    using namespace fb;
    RigExecWireFile file;
    file.formatVersion = RigExecFormatVersion;
    file.manifest = RigExecManifestBuild(RigExecManifestCurrent());
    file.names = {"", "Rig"};
    file.paths = {PathNode(0, 0, PathKind::None),
                  PathNode(0, 1, PathKind::Prim)};
    file.rig = 1;
    file.values.resize(1);
    file.intArrays.resize(1);
    file.floatArrays.resize(1);
    file.doubleArrays.resize(1);
    file.vec2fArrays.resize(1);
    file.vec3fArrays.resize(1);
    file.vec3dArrays.resize(1);
    file.matrix4dArrays.resize(1);
    file.tokenArrays.resize(1);
    file.boolArrays.resize(1);
    const auto add = [&](uint32_t slot, InputTag tag, uint64_t bits,
                         uint32_t array, bool listed) {
        file.names.push_back(kSlotNames[slot]);
        file.paths.push_back(
            PathNode(1, uint32_t(file.names.size() - 1), PathKind::Property));
        RigExecWireValue value;
        value.tag = tag;
        value.bits = bits;
        value.array = array;
        file.values.push_back(std::move(value));
        const uint8_t flags = uint8_t(InputSlotFlags::HasValue) |
                              (listed ? uint8_t(InputSlotFlags::Listed) : 0);
        file.inputs.push_back(InputSlot(uint32_t(file.paths.size() - 1),
                                        uint32_t(file.values.size() - 1), -1,
                                        -1, tag, flags));
    };
    RigExecWireVec3fArray points;
    for (const rigExecStd::Vec3f &p : mesh.points)
        points.v.push_back({p[0], p[1], p[2]});
    file.vec3fArrays.push_back(std::move(points));
    RigExecWireIntArray counts, indices;
    counts.v = mesh.counts;
    indices.v = mesh.indices;
    file.intArrays.push_back(std::move(counts));
    file.intArrays.push_back(std::move(indices));
    add(A, InputTag::Float, Bits(values.a), 0, true);
    add(B, InputTag::Float, Bits(values.b), 0, true);
    add(C, InputTag::Float, Bits(values.c), 0, true);
    add(PointsSlot, InputTag::Vec3fArray, 0, 1, true);
    add(Reference, InputTag::Double, Bits(values.reference), 0, true);
    add(W, InputTag::Float, Bits(values.w), 0, true);
    add(Counts, InputTag::IntArray, 0, 1, false);
    add(Indices, InputTag::IntArray, 0, 2, false);
    file.listedInputs = 6;
    file.slotMeta = std::make_unique<RigExecWireSlotMeta>();
    file.constants = std::make_unique<RigExecWireConstants>();
    file.clustering = std::make_unique<RigExecWireClustering>();
    file.cones = std::make_unique<RigExecWireCones>();
    file.cones->always = std::make_unique<RigExecWireClusterSet>();
    file.cones->poseClusters = std::make_unique<RigExecWireClusterSet>();
    file.pose = std::make_unique<RigExecWireDomainPose>();
    file.pose->requiredStageFramesAdmission =
        std::make_unique<RigExecWireRequiredStageFramesAdmission>();
    file.geometry = std::make_unique<RigExecWireDomainGeometry>();
    file.commonGraph = std::make_unique<RigExecWireCommonGraph>();
    return file;
}

rigExecOps::MathPayload
Math(RigExecPropertyOp op)
{
    rigExecOps::MathPayload payload;
    payload.op = op;
    return payload;
}

/// The op graph, through the export API the bake calls.
bool
AddOpGraph(fb::RigExecWireFile *file, std::string *error)
{
    using namespace rigExecOps;
    const MathPayload add = Math(RigExecPropertyOp::Add);
    const MathPayload multiply = Math(RigExecPropertyOp::Multiply);
    RigExecOpGraphWriter writer(*file);
    const uint32_t sum = writer.AddNode(FloatMath(), &add);
    const uint32_t product = writer.AddNode(FloatMath(), &multiply);
    const uint32_t smooth = writer.AddNode(Smooth());
    const uint32_t volume = writer.AddNode(VolumeCorrect());
    writer.BindSlot(sum, FloatIn(), A);
    writer.BindSlot(sum, FloatValue(), B);
    writer.BindSlot(sum, FloatWeight(), W);
    writer.Connect(sum, FloatOut(), product, FloatIn());
    writer.BindSlot(product, FloatValue(), C);
    writer.BindSlot(product, FloatWeight(), W);
    writer.BindSlot(smooth, SmoothPoints(), PointsSlot);
    writer.BindSlot(smooth, SmoothCounts(), Counts);
    writer.BindSlot(smooth, SmoothIndices(), Indices);
    writer.Connect(product, FloatOut(), smooth, SmoothStrength());
    writer.Connect(smooth, SmoothOut(), volume, VolumeCorrectPoints());
    writer.BindSlot(volume, VolumeCorrectReference(), Reference);
    writer.Order(sum, volume);
    writer.Publish(product, FloatOut(), "math");
    writer.Publish(smooth, SmoothOut(), "smooth");
    writer.Publish(volume, VolumeCorrectOut(), "volume");
    return writer.Finish(error);
}

fb::RigExecWireFile
OpFile(const Values &values, const Mesh &mesh)
{
    fb::RigExecWireFile file = BaseFile(values, mesh);
    std::string error;
    if (!AddOpGraph(&file, &error))
        std::printf("AddOpGraph: %s\n", error.c_str());
    return file;
}

std::vector<uint8_t>
Write(const fb::RigExecWireFile &file)
{
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecFormatWrite(file, &bytes, &error))
        std::printf("write: %s\n", error.c_str());
    return bytes;
}

/// The file packed without validation, for a writer would refuse it.
std::vector<uint8_t>
Pack(const fb::RigExecWireFile &file)
{
    flatbuffers::FlatBufferBuilder builder;
    fb::FinishFileBuffer(builder, fb::File::Pack(builder, &file));
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() + builder.GetSize());
}

struct Results {
    float math = 0.0f;
    std::vector<Vec3f> smooth, volume;
};

/// The kernels the native evaluators run, called directly.
Results
Expected(const Values &values, const Mesh &mesh)
{
    Results out;
    RigExecPropertyMathKernelParams<float, Vec2f> params;
    params.op = RigExecPropertyOp::Add;
    params.value = values.b;
    params.weight = values.w;
    const float sum = RigExecApplyFloatMathKernel(values.a, params);
    params.op = RigExecPropertyOp::Multiply;
    params.value = values.c;
    out.math = RigExecApplyFloatMathKernel(sum, params);
    out.smooth = mesh.points;
    RigExecApplyLaplacianSmoothMeshKernel(
        &out.smooth, mesh.counts.data(), mesh.counts.size(),
        mesh.indices.data(), mesh.indices.size(), double(out.math));
    out.volume = out.smooth;
    RigExecApplyVolumeCorrectKernel(&out.volume, values.reference, 1.0f);
    return out;
}

/// The same nodes through the op core alone, compiled with batches of at
/// most \p maxBatchNodes nodes.
Results
Direct(const Values &values, const Mesh &mesh, size_t maxBatchNodes)
{
    using namespace rigExecOps;
    Results out;
    const MathPayload add = Math(RigExecPropertyOp::Add);
    const MathPayload multiply = Math(RigExecPropertyOp::Multiply);
    RigExecRigDescBuilder b;
    const auto sum = b.AddOp(FloatMath(), &add);
    const auto product = b.AddOp(FloatMath(), &multiply);
    const auto smooth = b.AddOp(Smooth());
    const auto volume = b.AddOp(VolumeCorrect());
    const auto a = b.BindInput(sum, FloatIn());
    const auto bValue = b.BindInput(sum, FloatValue());
    const auto w1 = b.BindInput(sum, FloatWeight());
    b.Connect(sum, FloatOut(), product, FloatIn());
    const auto c = b.BindInput(product, FloatValue());
    const auto w2 = b.BindInput(product, FloatWeight());
    const auto points = b.BindInput(smooth, SmoothPoints());
    const auto counts = b.BindInput(smooth, SmoothCounts());
    const auto indices = b.BindInput(smooth, SmoothIndices());
    b.Connect(product, FloatOut(), smooth, SmoothStrength());
    b.Connect(smooth, SmoothOut(), volume, VolumeCorrectPoints());
    const auto reference = b.BindInput(volume, VolumeCorrectReference());
    b.Order(sum, volume);
    const auto mathOut = b.BindOutput(product, FloatOut());
    const auto smoothOut = b.BindOutput(smooth, SmoothOut());
    const auto volumeOut = b.BindOutput(volume, VolumeCorrectOut());
    std::string error;
    auto desc = b.Build(&error);
    RigExecCompileOptions options;
    options.maxBatchNodes = maxBatchNodes;
    auto program = desc ? RigExecCompile(*desc, options, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator) {
        std::printf("direct: %s\n", error.c_str());
        return out;
    }
    const Points pointArray(mesh.points.data(), mesh.points.size());
    const RigExecArray<int> countArray(mesh.counts.data(), mesh.counts.size());
    const RigExecArray<int> indexArray(mesh.indices.data(),
                                       mesh.indices.size());
    CHECK(evaluator->SetInput(a, &values.a) &&
          evaluator->SetInput(bValue, &values.b) &&
          evaluator->SetInput(w1, &values.w) &&
          evaluator->SetInput(c, &values.c) &&
          evaluator->SetInput(w2, &values.w) &&
          evaluator->SetInput(points, &pointArray) &&
          evaluator->SetInput(counts, &countArray) &&
          evaluator->SetInput(indices, &indexArray) &&
          evaluator->SetInput(reference, &values.reference));
    CHECK(evaluator->Evaluate(&error));
    out.math = *static_cast<const float *>(evaluator->GetOutput(mathOut));
    const auto &s =
        *static_cast<const Points *>(evaluator->GetOutput(smoothOut));
    const auto &v =
        *static_cast<const Points *>(evaluator->GetOutput(volumeOut));
    out.smooth.assign(s.begin(), s.end());
    out.volume.assign(v.begin(), v.end());
    return out;
}

bool
SameBits(const std::vector<Vec3f> &a, const std::vector<Vec3f> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(Vec3f)) == 0);
}

Results
FromRuntime(const RigExecRuntimeReader &reader)
{
    Results out;
    size_t index = 0;
    RigExecRuntimeOpOutput output;
    const auto points = [&](const char *name, std::vector<Vec3f> *to) {
        if (reader.FindOpOutput(name, &index) &&
            reader.GetOpOutput(index, &output) && output.value) {
            const auto &array = *static_cast<const Points *>(output.value);
            to->assign(array.begin(), array.end());
        }
    };
    if (reader.FindOpOutput("math", &index) &&
        reader.GetOpOutput(index, &output) && output.value)
        out.math = *static_cast<const float *>(output.value);
    points("smooth", &out.smooth);
    points("volume", &out.volume);
    return out;
}

void
CheckSame(const Results &a, const Results &b)
{
    CHECK(std::memcmp(&a.math, &b.math, sizeof(float)) == 0);
    CHECK(!a.smooth.empty() && SameBits(a.smooth, b.smooth));
    CHECK(!a.volume.empty() && SameBits(a.volume, b.volume));
}

void
TestPlayback()
{
    const Mesh mesh = Grid();
    Values values;
    const std::vector<uint8_t> bytes = Write(OpFile(values, mesh));
    std::string error;
    auto reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr, 0,
                                   RigExecLinkedRegistrations(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open: %s\n", error.c_str());
        return;
    }
    CHECK(reader->GetOpOutputCount() == 3);
    CHECK(reader->Execute(&error));
    const Results expected = Expected(values, mesh);
    CHECK(expected.math > 0.0f && expected.math < 1.0f);
    const Results got = FromRuntime(*reader);
    CheckSame(got, expected);
    for (const size_t batch : {size_t(0), size_t(1), size_t(3)}) {
        const Results direct = Direct(values, mesh, batch);
        CheckSame(direct, expected);
        CheckSame(direct, got);
    }

    // A set input reruns the step.
    values.a = -0.125f;
    values.reference = 4.0;
    CHECK(reader->SetInput("/Rig.a", double(values.a), &error));
    CHECK(reader->SetInput("/Rig.reference", values.reference, &error));
    CHECK(reader->Execute(&error));
    const Results moved = Expected(values, mesh);
    CheckSame(FromRuntime(*reader), moved);
    for (const size_t batch : {size_t(0), size_t(1), size_t(3)})
        CheckSame(Direct(values, mesh, batch), moved);
    CHECK(!SameBits(moved.volume, expected.volume));

    // The written file reads back as written.
    std::unique_ptr<fb::RigExecWireFile> opened;
    CHECK(RigExecFormatOpen(bytes.data(), bytes.size(), &opened, &error));
    CHECK(opened && opened->opGraph && opened->opGraph->nodes.size() == 4 &&
          opened->manifest->coreOps.size() == 3);
}

/// Open refuses \p file, with \p why in the reason.
void
Refused(const std::vector<uint8_t> &bytes, const char *why)
{
    std::string error;
    auto reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr, 0,
                                   RigExecLinkedRegistrations(), &error);
    CHECK(!reader);
    CHECK(Contains(error, why));
}

void
TestRefusals()
{
    const Mesh mesh = Grid();
    const Values values;
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.manifest->coreOps[0]->name = "Missing";
        Refused(Pack(file), "rig:Missing v1 is in no op library");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.manifest->coreTypes[0]->name = "Missing";
        Refused(Pack(file), "Missing v1 is in no type library");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[0]->payload.resize(3);
        Refused(Pack(file), "settings do not decode");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[0]->payload.clear();
        file.opGraph->nodes[0]->payload.push_back(7);
        Refused(Pack(file), "settings do not decode");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[1]->inputs[0]->node = 9;
        Refused(Pack(file), "node 9 out of range");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[0]->inputs[0]->slot = 99;
        Refused(Pack(file), "slot 99 out of range");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[0]->inputs[0]->port = 40;
        Refused(Pack(file), "has no input port 40");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->nodes[0]->op = 7;
        Refused(Pack(file), "past manifest core_ops");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->order.emplace_back(3, 0);
        Refused(Pack(file), "cycle");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        // A float slot cannot feed Smooth's points.
        file.opGraph->nodes[2]->inputs[0]->slot = A;
        file.steps.back().headInputSlots = {A, B,      C,      Reference,
                                            W, Counts, Indices};
        Refused(Pack(file), "is not of the type input port");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.opGraph->outputs[0]->type = file.opGraph->outputs[1]->type;
        Refused(Pack(file), "is not of its port's type");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.steps.back().headInputSlots.pop_back();
        Refused(Pack(file), "memo does not cover exactly");
    }
    {
        fb::RigExecWireFile file = OpFile(values, mesh);
        file.formatVersion = 34;
        Refused(Pack(file), "op_graph needs format 35");
    }
    {
        // A library this process never registered.
        const std::vector<uint8_t> bytes = Write(OpFile(values, mesh));
        RigExecOpLibraryDesc missing;
        missing.name = "notRegistered";
        RigExecRegistryInit init = RigExecLinkedRegistrations();
        init.opLibraries.push_back(&missing);
        std::string error;
        CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr,
                                          0, init, &error));
        CHECK(Contains(error, "op library notRegistered is not in this"));
    }
}

} // namespace

int
main()
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    // Before any registry exists, a file with op nodes needs the libraries.
    {
        const Mesh mesh = Grid();
        const Values values;
        fb::RigExecWireFile file = BaseFile(values, mesh);
        // The writer needs the registry; this file is built by hand.
        file.manifest->coreOps.push_back(
            std::make_unique<fb::RigExecWireManifestEntry>());
        file.manifest->coreOps[0]->nameSpace = "rig";
        file.manifest->coreOps[0]->name = "Smooth";
        auto graph = std::make_unique<fb::RigExecWireOpGraph>();
        graph->nodes.push_back(std::make_unique<fb::RigExecWireOpNode>());
        graph->step = 0;
        file.opGraph = std::move(graph);
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::OpGraph;
        step.object = 0;
        step.cluster = 0;
        step.descriptorKey = "opGraph";
        step.reads.push_back(
            fb::SlotRange(fb::SlotDomain::RequiredStageFramesAdmission, 0, 1));
        file.steps.push_back(std::move(step));
        fb::RigExecWireCommonValueSpec admission;
        admission.domain =
            uint32_t(fb::SlotDomain::RequiredStageFramesAdmission);
        file.commonGraph->valueSpecs.push_back(std::move(admission));
        file.commonGraph->leaves = {0};
        fb::RigExecWireCommonReaders readers;
        readers.ops = {0};
        file.commonGraph->readers.push_back(std::move(readers));
        fb::RigExecWireCommonOp op;
        op.key = "opGraph";
        op.kind = uint32_t(fb::StepKind::OpGraph);
        op.reads = {0};
        file.commonGraph->ops.push_back(std::move(op));
        file.commonGraph->canonicalIndex = {0};
        file.commonGraph->longestPath = 1;
        fb::RigExecWireCommonCluster commonCluster;
        commonCluster.members = {0};
        file.commonGraph->clusters.push_back(std::move(commonCluster));
        file.commonGraph->opClusters = {0};
        fb::RigExecWireCluster cluster;
        cluster.members = {0};
        file.clustering->clusters.push_back(std::move(cluster));
        file.clustering->clusterOf = {0};
        for (auto *set :
             {file.cones->always.get(), file.cones->poseClusters.get()}) {
            set->clusters = 1;
            set->words = {0};
        }
        const std::vector<uint8_t> bytes = Pack(file);
        std::string error;
        CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error));
        CHECK(Contains(error, "open it with the op and type libraries"));
        CHECK(!RigExecRegistry::IsInitialized());
        // Given the libraries, Open registers them; this Smooth has no
        // points, which the op core refuses.
        CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr,
                                          0, RigExecLinkedRegistrations(),
                                          &error));
        CHECK(RigExecRegistry::IsInitialized());
        CHECK(error.find("libraries") == std::string::npos && !error.empty());
    }
    TestPlayback();
    TestRefusals();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecRuntimeOpNodes passed\n");
    return 0;
}
