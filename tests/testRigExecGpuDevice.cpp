// RigExecGpuDevice on the CPU reference device: MatrixDeform run as a device
// kernel over point columns equals the CPU op's matrix kernel bit for bit,
// and the device refuses every malformed submission before running any of
// it.
#include "rigExec/api/gpuDevice.h"
#include "rigExecEvalCore/envelope.h"
#include "rigExecMath/matrixKernel.h"
#include "rigExecOps/deform/matrixDeformDevice.h"
#include "rigExecMath/runtimeMath.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;
using namespace rigExecOps;

namespace {

int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

using Math = eval::MatrixKernelMath<RrVec3f, RrVec3d, RrMat4d>;

constexpr uint32_t kPoints = 1001;

uint64_t
Align(uint64_t value, uint64_t alignment)
{
    return (value + alignment - 1) / alignment * alignment;
}

uint32_t
Bits(float f)
{
    uint32_t u;
    std::memcpy(&u, &f, sizeof u);
    return u;
}

/// Every coordinate of \p points equals \p x, \p y and \p z bit for bit.
bool
SameBits(const std::vector<RrVec3f> &points, const std::vector<float> &x,
         const std::vector<float> &y, const std::vector<float> &z)
{
    if (points.size() != x.size() || x.size() != y.size() ||
        y.size() != z.size())
        return false;
    for (size_t i = 0; i < points.size(); ++i)
        if (Bits(points[i][0]) != Bits(x[i]) ||
            Bits(points[i][1]) != Bits(y[i]) ||
            Bits(points[i][2]) != Bits(z[i])) {
            std::printf("point %zu differs\n", i);
            return false;
        }
    return true;
}

struct Inputs {
    std::vector<RrVec3f> points;
    std::vector<float> x, y, z, weights;
    RrMat4d transform;
};

Inputs
MakeInputs()
{
    Inputs in;
    uint32_t state = 12345u;
    auto next = [&state] {
        state = state * 1664525u + 1013904223u;
        return float(state >> 8) / float(1u << 24);
    };
    for (uint32_t i = 0; i < kPoints; ++i) {
        const RrVec3f p(next() * 20.0f - 10.0f, next() * 20.0f - 10.0f,
                        next() * 20.0f - 10.0f);
        in.points.push_back(p);
        in.x.push_back(p[0]);
        in.y.push_back(p[1]);
        in.z.push_back(p[2]);
        const float edges[] = {-0.5f, 0.0f, 1.0f, 1.5f};
        in.weights.push_back(i % 7 < 4 ? edges[i % 7] : next());
    }
    in.transform = RrMat4d(0.8, 0.5, -0.2, 0.0, -0.45, 0.83, 0.31, 0.0, 0.27,
                           -0.18, 1.1, 0.0, 1.5, -2.0, 0.25, 1.0);
    return in;
}

/// One arena buffer of float columns, each starting on the device's
/// binding offset alignment: in x, y, z, weights, out x, y, z.
struct Arena {
    RigExecGpuBuffer buffer;
    uint64_t stride = 0;
    uint64_t columnBytes = 0;

    RigExecGpuBinding Column(uint32_t column) const
    {
        RigExecGpuBinding b;
        b.buffer = buffer;
        b.offset = stride * column;
        b.bytes = columnBytes;
        return b;
    }
};

Arena
MakeArena(RigExecGpuDevice &device)
{
    Arena arena;
    arena.columnBytes = uint64_t(kPoints) * sizeof(float);
    arena.stride =
        Align(arena.columnBytes, device.Capabilities().bindingOffsetAlignment);
    RigExecGpuBufferDesc desc;
    desc.bytes = arena.stride * kMatrixDeviceBindingCount;
    desc.usage = RigExecGpuBufferStorage | RigExecGpuBufferUploadTarget |
                 RigExecGpuBufferDownloadSource;
    std::string error;
    arena.buffer = device.CreateBuffer(desc, &error);
    CHECK(arena.buffer.IsValid());
    return arena;
}

void
RecordDeform(RigExecGpuCommandList &list, RigExecGpuKernel kernel,
             const Arena &arena, const Inputs &in, const RrMat4d &transform,
             bool weighted, float weight)
{
    list.Upload(arena.buffer, arena.Column(kMatrixDeviceInX).offset,
                in.x.data(), arena.columnBytes);
    list.Upload(arena.buffer, arena.Column(kMatrixDeviceInY).offset,
                in.y.data(), arena.columnBytes);
    list.Upload(arena.buffer, arena.Column(kMatrixDeviceInZ).offset,
                in.z.data(), arena.columnBytes);
    if (weighted)
        list.Upload(arena.buffer, arena.Column(kMatrixDeviceWeights).offset,
                    in.weights.data(), arena.columnBytes);
    RigExecGpuBinding bindings[kMatrixDeviceBindingCount];
    for (uint32_t c = 0; c < kMatrixDeviceBindingCount; ++c)
        bindings[c] = arena.Column(c);
    if (!weighted)
        bindings[kMatrixDeviceWeights].bytes = 0;
    const MatrixDeformDeviceUniforms u =
        MatrixDeformDeviceUniformsOf(transform, weight, weighted);
    list.Dispatch(kernel, bindings, kMatrixDeviceBindingCount, &u, sizeof u,
                  kPoints);
}

struct Readback {
    std::vector<float> x, y, z;
    Readback() : x(kPoints), y(kPoints), z(kPoints)
    {
    }
};

void
RecordReadback(RigExecGpuCommandList &list, const Arena &arena,
               uint32_t firstColumn, Readback &out)
{
    list.Download(arena.buffer, arena.Column(firstColumn).offset, out.x.data(),
                  arena.columnBytes);
    list.Download(arena.buffer, arena.Column(firstColumn + 1).offset,
                  out.y.data(), arena.columnBytes);
    list.Download(arena.buffer, arena.Column(firstColumn + 2).offset,
                  out.z.data(), arena.columnBytes);
}

bool
SubmitAndWait(RigExecGpuDevice &device, const RigExecGpuCommandList &list)
{
    std::string error;
    const RigExecGpuFence fence = device.Submit(list, &error);
    if (!fence.value) {
        std::printf("submit refused: %s\n", error.c_str());
        return false;
    }
    return device.Wait(fence, &error) && device.IsComplete(fence);
}

void
TestCapabilities(RigExecGpuDevice &device)
{
    const RigExecGpuCapabilities caps = device.Capabilities();
    CHECK(caps.backend == RigExecGpuBackend::CpuReference);
    CHECK(caps.hostExact);
    CHECK(caps.bindingOffsetAlignment == 256);
    CHECK(caps.maxBindings >= kMatrixDeviceBindingCount);
    CHECK(caps.maxUniformBytes >= sizeof(MatrixDeformDeviceUniforms));
}

void
TestMatchesOpKernel(RigExecGpuDevice &device, RigExecGpuKernel kernel,
                    const Inputs &in)
{
    const Arena arena = MakeArena(device);

    std::vector<RrVec3f> weighted(kPoints);
    eval::ApplyMatrixKernelRange<Math>(false, in.transform, in.weights.data(),
                                       0, kPoints, in.points.data(),
                                       weighted.data(), true);
    RigExecGpuCommandList list;
    Readback out;
    RecordDeform(list, kernel, arena, in, in.transform, true, 0.0f);
    RecordReadback(list, arena, kMatrixDeviceOutX, out);
    CHECK(SubmitAndWait(device, list));
    CHECK(SameBits(weighted, out.x, out.y, out.z));

    std::vector<RrVec3f> full(kPoints);
    eval::ApplyMatrixKernelRange<Math>(false, in.transform, nullptr, 0, kPoints,
                                       in.points.data(), full.data(), true);
    list.Clear();
    RecordDeform(list, kernel, arena, in, in.transform, false, 1.0f);
    RecordReadback(list, arena, kMatrixDeviceOutX, out);
    CHECK(SubmitAndWait(device, list));
    CHECK(SameBits(full, out.x, out.y, out.z));

    std::vector<RrVec3f> uniform(kPoints);
    RigExecApplyWeightedMatrixSimdKernel<RrVec3f, RrVec3d>(
        in.points.data(), uniform.data(), 0.37f, kPoints, in.transform);
    list.Clear();
    RecordDeform(list, kernel, arena, in, in.transform, false, 0.37f);
    RecordReadback(list, arena, kMatrixDeviceOutX, out);
    CHECK(SubmitAndWait(device, list));
    CHECK(SameBits(uniform, out.x, out.y, out.z));
    device.DestroyBuffer(arena.buffer);
}

/// Two dispatches in one submission, the second reading what the first
/// wrote without a round trip through the host.
void
TestChained(RigExecGpuDevice &device, RigExecGpuKernel kernel, const Inputs &in)
{
    const Arena arena = MakeArena(device);
    const RrMat4d second(0.9, -0.1, 0.0, 0.0, 0.1, 0.95, 0.2, 0.0, 0.0, -0.2,
                         1.05, 0.0, -0.5, 0.75, 3.0, 1.0);
    std::vector<RrVec3f> expected(kPoints);
    eval::ApplyMatrixKernelRange<Math>(false, in.transform, in.weights.data(),
                                       0, kPoints, in.points.data(),
                                       expected.data(), true);
    eval::ApplyMatrixKernelRange<Math>(false, second, in.weights.data(), 0,
                                       kPoints, expected.data(),
                                       expected.data(), true);

    RigExecGpuCommandList list;
    RecordDeform(list, kernel, arena, in, in.transform, true, 0.0f);
    RigExecGpuBinding back[kMatrixDeviceBindingCount] = {
        arena.Column(kMatrixDeviceOutX), arena.Column(kMatrixDeviceOutY),
        arena.Column(kMatrixDeviceOutZ), arena.Column(kMatrixDeviceWeights),
        arena.Column(kMatrixDeviceInX),  arena.Column(kMatrixDeviceInY),
        arena.Column(kMatrixDeviceInZ)};
    const MatrixDeformDeviceUniforms u =
        MatrixDeformDeviceUniformsOf(second, 0.0f, true);
    list.Dispatch(kernel, back, kMatrixDeviceBindingCount, &u, sizeof u,
                  kPoints);
    Readback out;
    RecordReadback(list, arena, kMatrixDeviceInX, out);
    CHECK(SubmitAndWait(device, list));
    CHECK(SameBits(expected, out.x, out.y, out.z));
    device.DestroyBuffer(arena.buffer);
}

/// Results do not depend on how invocations are grouped.
void
TestWorkgroupSizes(RigExecGpuDevice &device, const Inputs &in)
{
    std::vector<RrVec3f> expected(kPoints);
    eval::ApplyMatrixKernelRange<Math>(false, in.transform, in.weights.data(),
                                       0, kPoints, in.points.data(),
                                       expected.data(), true);
    for (uint32_t size : {1u, 7u, 256u, 4096u}) {
        RigExecGpuKernelDesc desc = MatrixDeformDeviceKernel();
        desc.workgroupSize = size;
        std::string error;
        const RigExecGpuKernel kernel = device.RegisterKernel(desc, &error);
        CHECK(kernel.IsValid());
        const Arena arena = MakeArena(device);
        RigExecGpuCommandList list;
        Readback out;
        RecordDeform(list, kernel, arena, in, in.transform, true, 0.0f);
        RecordReadback(list, arena, kMatrixDeviceOutX, out);
        CHECK(SubmitAndWait(device, list));
        CHECK(SameBits(expected, out.x, out.y, out.z));
        device.DestroyBuffer(arena.buffer);
    }
}

/// A refused submission runs none of its commands, not even the upload
/// listed before the bad dispatch.
void
ExpectRefused(RigExecGpuDevice &device, const Arena &arena,
              RigExecGpuCommandList &list, const char *what)
{
    std::string error;
    const RigExecGpuFence fence = device.Submit(list, &error);
    if (fence.value || error.empty()) {
        std::printf("accepted: %s\n", what);
        ++failures;
    }
    std::vector<float> outX(kPoints, 1.0f);
    RigExecGpuCommandList read;
    read.Download(arena.buffer, arena.Column(kMatrixDeviceOutX).offset,
                  outX.data(), arena.columnBytes);
    CHECK(SubmitAndWait(device, read));
    CHECK(Bits(outX[0]) == Bits(0.0f));
}

void
TestRefusals(RigExecGpuDevice &device, RigExecGpuKernel kernel,
             const Inputs &in)
{
    const Arena arena = MakeArena(device);
    const std::vector<float> sentinel(kPoints, 42.0f);
    const MatrixDeformDeviceUniforms u =
        MatrixDeformDeviceUniformsOf(in.transform, 0.0f, true);
    RigExecGpuBinding good[kMatrixDeviceBindingCount];
    for (uint32_t c = 0; c < kMatrixDeviceBindingCount; ++c)
        good[c] = arena.Column(c);
    auto refused = [&](const RigExecGpuBinding *bindings, uint32_t count,
                       uint32_t uniformBytes, RigExecGpuKernel k,
                       const char *what) {
        RigExecGpuCommandList list;
        list.Upload(arena.buffer, arena.Column(kMatrixDeviceOutX).offset,
                    sentinel.data(), arena.columnBytes);
        list.Dispatch(k, bindings, count, &u, uniformBytes, kPoints);
        ExpectRefused(device, arena, list, what);
    };

    RigExecGpuBinding b[kMatrixDeviceBindingCount];
    std::copy(good, good + kMatrixDeviceBindingCount, b);
    b[kMatrixDeviceInY].offset += 4;
    refused(b, kMatrixDeviceBindingCount, sizeof u, kernel, "unaligned");

    std::copy(good, good + kMatrixDeviceBindingCount, b);
    b[kMatrixDeviceOutZ].bytes += arena.stride;
    refused(b, kMatrixDeviceBindingCount, sizeof u, kernel, "past the end");

    std::copy(good, good + kMatrixDeviceBindingCount, b);
    b[kMatrixDeviceOutX] = good[kMatrixDeviceInX];
    refused(b, kMatrixDeviceBindingCount, sizeof u, kernel, "in place");

    refused(good, kMatrixDeviceBindingCount, sizeof u - 4, kernel,
            "uniform size");
    refused(good, kMatrixDeviceBindingCount - 1, sizeof u, kernel,
            "binding count");
    RigExecGpuKernel unknown;
    unknown.index = 99;
    refused(good, kMatrixDeviceBindingCount, sizeof u, unknown,
            "unknown kernel");

    std::string error;
    RigExecGpuBufferDesc small;
    small.bytes = arena.columnBytes;
    small.usage = RigExecGpuBufferStorage;
    const RigExecGpuBuffer stale = device.CreateBuffer(small, &error);
    device.DestroyBuffer(stale);
    const RigExecGpuBuffer reused = device.CreateBuffer(small, &error);
    CHECK(reused.IsValid() && reused.index == stale.index &&
          reused.generation != stale.generation);
    std::copy(good, good + kMatrixDeviceBindingCount, b);
    b[kMatrixDeviceInX].buffer = stale;
    b[kMatrixDeviceInX].offset = 0;
    refused(b, kMatrixDeviceBindingCount, sizeof u, kernel, "stale buffer");

    RigExecGpuCommandList upload;
    upload.Upload(arena.buffer, arena.Column(kMatrixDeviceOutX).offset,
                  sentinel.data(), arena.columnBytes);
    upload.Upload(reused, 0, sentinel.data(), arena.columnBytes);
    ExpectRefused(device, arena, upload, "upload without its usage");
    device.DestroyBuffer(reused);

    RigExecGpuKernelDesc noHost = MatrixDeformDeviceKernel();
    noHost.host = nullptr;
    CHECK(!device.RegisterKernel(noHost, &error).IsValid());
    RigExecGpuFence never;
    never.value = 1u << 30;
    CHECK(!device.Wait(never, &error) && !error.empty());
    CHECK(!device.IsComplete(never));
    device.DestroyBuffer(arena.buffer);
}

/// A worker records a list without the device and cannot submit it; the
/// owning thread submits what the worker recorded.
void
TestOwnership(RigExecGpuDevice &device, RigExecGpuKernel kernel,
              const Inputs &in)
{
    const Arena arena = MakeArena(device);
    RigExecGpuCommandList list;
    Readback out;
    bool workerRefused = false;
    std::thread worker([&] {
        RecordDeform(list, kernel, arena, in, in.transform, true, 0.0f);
        RecordReadback(list, arena, kMatrixDeviceOutX, out);
        std::string error;
        workerRefused = !device.Submit(list, &error).value && !error.empty();
    });
    worker.join();
    CHECK(workerRefused);
    std::vector<RrVec3f> expected(kPoints);
    eval::ApplyMatrixKernelRange<Math>(false, in.transform, in.weights.data(),
                                       0, kPoints, in.points.data(),
                                       expected.data(), true);
    CHECK(SubmitAndWait(device, list));
    CHECK(SameBits(expected, out.x, out.y, out.z));
    device.DestroyBuffer(arena.buffer);
}

} // namespace

int
main()
{
    std::unique_ptr<RigExecGpuDevice> device =
        RigExecCreateCpuReferenceGpuDevice();
    TestCapabilities(*device);
    std::string error;
    const RigExecGpuKernel kernel =
        device->RegisterKernel(MatrixDeformDeviceKernel(), &error);
    CHECK(kernel.IsValid());
    const Inputs in = MakeInputs();
    TestMatchesOpKernel(*device, kernel, in);
    TestChained(*device, kernel, in);
    TestWorkgroupSizes(*device, in);
    TestRefusals(*device, kernel, in);
    TestOwnership(*device, kernel, in);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
