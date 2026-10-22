#include "rigExecGpu/gpuDeformExecutor.h"

#include "rigExec/gpuDeformTranslation.h"
#include "rigExecGpu/clothShader.h"
#include "rigExecGpu/glContext.h"
#include "rigExecGpu/gpuDeformShader.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/imaging/garch/glApi.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <thread>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

// Storage-buffer binding points of the kernel.
enum _Binding : GLuint {
    _Ops = 0,
    _StaticF,
    _StaticI,
    _Frame,
    _Base,
    _Out,
    _Status,
};

constexpr int _LocalSize = 64;

const char *const _header = R"GLSL(#version 450
layout(local_size_x = 64) in;
layout(std430, binding = 0) readonly buffer RxOps { int ops[]; };
layout(std430, binding = 1) readonly buffer RxStaticF { float staticF[]; };
layout(std430, binding = 2) readonly buffer RxStaticI { int staticI[]; };
layout(std430, binding = 3) readonly buffer RxFrame { float frameF[]; };
layout(std430, binding = 4) readonly buffer RxBase { float basePoints[]; };
layout(std430, binding = 5) writeonly buffer RxOut { float outPoints[]; };
layout(std430, binding = 6) buffer RxStatus { int status[]; };
uniform int pointCount;
uniform int opCount;
#define RX_OP(i) ops[i]
#define RX_SF(i) staticF[i]
#define RX_SI(i) staticI[i]
#define RX_FR(i) frameF[i]
#define RX_OP_COUNT opCount
)GLSL";

const char *const _main = R"GLSL(
void main()
{
    int i = int(gl_GlobalInvocationID.x);
    if (i >= pointCount) return;
    vec3 p = vec3(basePoints[3 * i], basePoints[3 * i + 1], basePoints[3 * i + 2]);
    p = rxDeform(i, p);
    outPoints[3 * i] = p.x;
    outPoints[3 * i + 1] = p.y;
    outPoints[3 * i + 2] = p.z;
    if (rxFailed) status[0] = 1;
}
)GLSL";

/// One chain's device buffers, kept between runs. The output and status
/// buffers are persistently mapped client storage: the kernel writes the
/// points where the CPU reads them, so a run's readback is a fence and a
/// memcpy, not a copy back from video memory.
struct _ChainBuffers {
    uint64_t layoutSerial = 0;
    uint32_t pointCount = 0;
    GLuint ops = 0, staticF = 0, staticI = 0, frame = 0, base = 0, out = 0,
           status = 0;
    size_t frameBytes = 0;
    uint64_t baseVersion = ~uint64_t(0);
    const float *mappedOut = nullptr;
    int32_t *mappedStatus = nullptr;
};

/// One cloth's device buffers, kept between solves: the mesh while its
/// compiled rig holds, and storage the per-solve uploads reuse.
struct _ClothBuffers {
    uint64_t compiledRigSerial = 0;
    GLuint meshI = 0, meshF = 0, tickP = 0, params = 0, grid = 0, head = 0,
           state = 0, out = 0;
    // The mesh layout's offsets and counts, as the kernel's uniforms.
    int oEnds = 0, oColor = 0, oAnchor = 0, oTris = 0, oKind = 0;
    int oRest = 0, oInv = 0, oPin = 0, oTether = 0;
    int constraints = 0, colors = 0, triangles = 0;
};

/// Replaces \p id's contents (creating it on first use). A zero-length
/// storage buffer is not a buffer; keep a word.
void
_Upload(GLuint *id, size_t bytes, const void *data)
{
    if (!*id) glCreateBuffers(1, id);
    static const int32_t zero = 0;
    glNamedBufferData(*id, GLsizeiptr(bytes ? bytes : 4), bytes ? data : &zero,
                      GL_STREAM_DRAW);
}

GLuint
_ImmutableBuffer(size_t bytes, const void *data, GLbitfield flags)
{
    GLuint id = 0;
    glCreateBuffers(1, &id);
    // A zero-length storage buffer is not a buffer; keep a word.
    static const int32_t zero = 0;
    glNamedBufferStorage(id, GLsizeiptr(bytes ? bytes : 4),
                         bytes ? data : &zero, flags);
    return id;
}

}  // namespace

struct RigExecGpuDeformExecutor::Impl {
    // Worker-thread state; touched only by the worker.
    RigExecGpuGLContext context;
    GLuint program = 0;
    GLint pointCountLocation = -1;
    GLint opCountLocation = -1;
    std::map<uint64_t, _ChainBuffers> chains;
    std::string description;
    // The cloth kernel, compiled on its first solve; failure is remembered.
    GLuint clothProgram = 0;
    bool clothTried = false;
    std::map<uint64_t, _ClothBuffers> cloths;
    std::atomic<size_t> clothSolves{0};
    // Per-solve staging, reused.
    std::vector<float> clothTicks, clothParams;
    std::vector<uint32_t> clothGrid, clothHead;
    RigExecClothGrid clothGridScratch;
    std::vector<uint64_t> clothCellScratch;

    // The queue between callers and the worker.
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<std::function<void()>> tasks;
    bool stop = false;
    std::thread worker;
    bool started = false;
    double lastRunMs = 0.0;

    void Loop()
    {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wake.wait(lock, [this] { return stop || !tasks.empty(); });
                if (stop && tasks.empty()) break;
                task = std::move(tasks.front());
                tasks.pop_front();
            }
            task();
        }
        // Device objects go with the context that made them.
        for (auto &[key, b] : chains) Release(&b);
        chains.clear();
        for (auto &[key, b] : cloths) ReleaseCloth(&b);
        cloths.clear();
        if (clothProgram) glDeleteProgram(clothProgram);
        if (program) glDeleteProgram(program);
        context.Destroy();
    }

    template <class F>
    auto Post(F &&f) -> decltype(f())
    {
        using R = decltype(f());
        auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
        std::future<R> result = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex);
            tasks.emplace_back([task] { (*task)(); });
        }
        wake.notify_one();
        return result.get();
    }

    bool Start()
    {
        worker = std::thread([this] { Loop(); });
        started = true;
        return Post([this] { return Create(); });
    }

    void Shutdown()
    {
        if (!started) return;
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        wake.notify_one();
        worker.join();
        started = false;
    }

    bool Create()
    {
        if (!context.Create()) return false;
        GarchGLApiLoad();
        const char *vendor = (const char *)glGetString(GL_VENDOR);
        const char *renderer = (const char *)glGetString(GL_RENDERER);
        const char *version = (const char *)glGetString(GL_VERSION);
        description = std::string(vendor ? vendor : "?") + " / " +
                      (renderer ? renderer : "?") + " / " +
                      (version ? version : "?");
        GLint major = 0, minor = 0;
        glGetIntegerv(GL_MAJOR_VERSION, &major);
        glGetIntegerv(GL_MINOR_VERSION, &minor);
        if (major * 10 + minor < 45 || !glDispatchCompute ||
            !glNamedBufferStorage) {
            TF_WARN("rigExecGpu: OpenGL 4.5 compute is unavailable (%s)",
                    description.c_str());
            return false;
        }
        const std::string source = std::string(_header) +
                                   RigExecGpuDeformKernelBody() + _main;
        const char *text = source.c_str();
        GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &text, nullptr);
        glCompileShader(shader);
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[4096] = {0};
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            TF_WARN("rigExecGpu: deform kernel failed to compile:\n%s", log);
            glDeleteShader(shader);
            return false;
        }
        program = glCreateProgram();
        glAttachShader(program, shader);
        glLinkProgram(program);
        glDeleteShader(shader);
        glGetProgramiv(program, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[4096] = {0};
            glGetProgramInfoLog(program, sizeof(log), nullptr, log);
            TF_WARN("rigExecGpu: deform program failed to link:\n%s", log);
            return false;
        }
        pointCountLocation = glGetUniformLocation(program, "pointCount");
        opCountLocation = glGetUniformLocation(program, "opCount");
        return true;
    }

    void ReleaseCloth(_ClothBuffers *b)
    {
        for (GLuint *id : {&b->meshI, &b->meshF, &b->tickP, &b->params,
                           &b->grid, &b->head, &b->state, &b->out})
            if (*id) glDeleteBuffers(1, id);
        *b = _ClothBuffers();
    }

    bool CompileCloth()
    {
        if (clothTried) return clothProgram != 0;
        clothTried = true;
        const char *text = RigExecGpuClothKernelSource();
        GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &text, nullptr);
        glCompileShader(shader);
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[4096] = {0};
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            TF_WARN("rigExecGpu: cloth kernel failed to compile:\n%s", log);
            glDeleteShader(shader);
            return false;
        }
        GLuint linked = glCreateProgram();
        glAttachShader(linked, shader);
        glLinkProgram(linked);
        glDeleteShader(shader);
        glGetProgramiv(linked, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[4096] = {0};
            glGetProgramInfoLog(linked, sizeof(log), nullptr, log);
            TF_WARN("rigExecGpu: cloth program failed to link:\n%s", log);
            glDeleteProgram(linked);
            return false;
        }
        clothProgram = linked;
        return true;
    }

    // The mesh's static arrays, laid out as the kernel reads them.
    void UploadClothMesh(const RigExecGpuClothJob &job, _ClothBuffers *b)
    {
        const RigExecClothMesh &mesh = *job.mesh;
        const RigExecClothColliders &colliders = *job.colliders;
        std::vector<int32_t> ints;
        b->oEnds = 0;
        ints.insert(ints.end(), mesh.constraintEnds.begin(), mesh.constraintEnds.end());
        b->oColor = int(ints.size());
        for (const uint32_t start : mesh.colorStart) ints.push_back(int32_t(start));
        b->oAnchor = int(ints.size());
        ints.insert(ints.end(), mesh.tetherAnchor.begin(), mesh.tetherAnchor.end());
        b->oTris = int(ints.size());
        ints.insert(ints.end(), colliders.triangles.begin(), colliders.triangles.end());
        b->oKind = int(ints.size());
        for (const uint8_t kind : mesh.constraintKind) ints.push_back(int32_t(kind));
        std::vector<float> floats;
        b->oRest = 0;
        floats.insert(floats.end(), mesh.constraintRest.begin(), mesh.constraintRest.end());
        b->oInv = int(floats.size());
        floats.insert(floats.end(), mesh.inverseMass.begin(), mesh.inverseMass.end());
        b->oPin = int(floats.size());
        floats.insert(floats.end(), mesh.pin.begin(), mesh.pin.end());
        b->oTether = int(floats.size());
        floats.insert(floats.end(), mesh.tetherRest.begin(), mesh.tetherRest.end());
        _Upload(&b->meshI, ints.size() * 4, ints.data());
        _Upload(&b->meshF, floats.size() * 4, floats.data());
        b->constraints = int(mesh.constraintRest.size());
        b->colors = int(mesh.ColorCount());
        b->triangles = int(colliders.triangles.size() / 3);
        b->compiledRigSerial = job.compiledRigSerial;
    }

    bool RunClothOnWorker(std::vector<RigExecGpuClothJob> *jobs)
    {
        if (!program || !CompileCloth()) return false;
        // RIGEXEC_GPU_CLOTH_STATS prints where a solve's time went.
        const bool stats = TfGetenvBool("RIGEXEC_GPU_CLOTH_STATS", false);
        const auto now = [] { return std::chrono::steady_clock::now(); };
        const auto ms = [](auto a, auto b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        double gridMs = 0.0, uploadMs = 0.0, submitMs = 0.0;
        size_t dispatches = 0;
        glUseProgram(clothProgram);
        const auto uniform = [this](const char *name) {
            return glGetUniformLocation(clothProgram, name);
        };
        std::vector<_ClothBuffers *> used;
        for (RigExecGpuClothJob &job : *jobs) {
            job.ok = false;
            used.push_back(nullptr);
            if (!job.mesh || !job.colliders || !job.ticks || job.tickCount < 2 ||
                !job.out || !job.mesh->valid) {
                continue;
            }
            const RigExecClothMesh &mesh = *job.mesh;
            // Relationship profiles use the shared CPU solve until the
            // device implements these constraint types too.
            if (!mesh.model.Empty()) continue;
            const RigExecClothColliders &colliders = *job.colliders;
            const size_t n = mesh.pointCount, m = colliders.pointCount;
            _ClothBuffers &b = cloths[job.key];
            if (b.compiledRigSerial != job.compiledRigSerial || !b.meshI) UploadClothMesh(job, &b);
            // The window: entering then collider points per tick, and the
            // parameters.
            const size_t stride = 3 * n + 3 * m;
            clothTicks.resize(job.tickCount * stride);
            clothParams.resize(job.tickCount * 10);
            for (size_t k = 0; k < job.tickCount; ++k) {
                const RigExecClothTick &tick = job.ticks[k];
                std::copy(tick.entering, tick.entering + 3 * n,
                          clothTicks.begin() + std::ptrdiff_t(k * stride));
                if (m && tick.colliders)
                    std::copy(tick.colliders, tick.colliders + 3 * m,
                              clothTicks.begin() + std::ptrdiff_t(k * stride + 3 * n));
                const RigExecClothParameters &p = tick.parameters;
                const float values[10] = {p.stretchCompliance, p.bendCompliance,
                                          p.gravity[0], p.gravity[1], p.gravity[2],
                                          p.damping, p.attract, p.thickness,
                                          p.collisionDepth, p.friction};
                std::copy(values, values + 10, clothParams.begin() + std::ptrdiff_t(k * 10));
            }
            // Each tick's collider grid, built here as the CPU solve builds
            // it: swept from the tick before, padded by the tick's reach.
            const auto g0 = now();
            clothGrid.clear();
            clothHead.assign(4 * job.tickCount, 0u);
            const bool collide = m && !colliders.triangles.empty();
            for (size_t k = 1; collide && k < job.tickCount; ++k) {
                const RigExecClothParameters &p = job.ticks[k].parameters;
                RigExecBuildClothGrid(colliders, job.ticks[k - 1].colliders,
                                      job.ticks[k].colliders,
                                      std::max(std::max(0.0f, p.thickness),
                                               std::max(0.0f, p.collisionDepth)),
                                      &clothGridScratch, &clothCellScratch);
                clothHead[4 * k] = uint32_t(clothGrid.size());
                clothGrid.insert(clothGrid.end(), clothGridScratch.bucketStart.begin(),
                                 clothGridScratch.bucketStart.end());
                clothHead[4 * k + 1] = uint32_t(clothGrid.size());
                for (const int32_t tri : clothGridScratch.bucketTriangles)
                    clothGrid.push_back(uint32_t(tri));
                clothHead[4 * k + 2] = clothGridScratch.bucketMask;
                float cell = clothGridScratch.cellSize;
                uint32_t bits = 0;
                std::memcpy(&bits, &cell, sizeof(bits));
                clothHead[4 * k + 3] = bits;
            }
            const auto u0 = now();
            gridMs += ms(g0, u0);
            _Upload(&b.tickP, clothTicks.size() * 4, clothTicks.data());
            _Upload(&b.params, clothParams.size() * 4, clothParams.data());
            _Upload(&b.grid, clothGrid.size() * 4, clothGrid.data());
            _Upload(&b.head, clothHead.size() * 4, clothHead.data());
            _Upload(&b.state,
                    (9 * n + size_t(b.constraints) + 3 * m + 16 * size_t(b.triangles)) * 4,
                    nullptr);
            _Upload(&b.out, 3 * n * 4, nullptr);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, b.meshI);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, b.meshF);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, b.tickP);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, b.params);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, b.grid);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, b.head);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, b.state);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, b.out);
            const RigExecClothSettings &settings = job.settings;
            const auto s0 = now();
            uploadMs += ms(u0, s0);
            const int substeps =
                std::max(1, std::min(settings.substeps, RigExecClothMaxSubsteps));
            const int iterations =
                std::max(1, std::min(settings.iterations, RigExecClothMaxIterations));
            glUniform1i(uniform("N"), GLint(n));
            glUniform1i(uniform("M"), GLint(m));
            glUniform1i(uniform("C"), GLint(b.constraints));
            glUniform1i(uniform("T"), GLint(collide ? b.triangles : 0));
            glUniform1i(uniform("tickCount"), GLint(job.tickCount));
            glUniform1i(uniform("substeps"), GLint(substeps));
            glUniform1f(uniform("frameSeconds"), float(job.frameSeconds));
            glUniform1f(uniform("tetherSlack"), settings.tetherSlack);
            glUniform1i(uniform("oEnds"), b.oEnds);
            glUniform1i(uniform("oColor"), b.oColor);
            glUniform1i(uniform("oAnchor"), b.oAnchor);
            glUniform1i(uniform("oTris"), b.oTris);
            glUniform1i(uniform("oKind"), b.oKind);
            glUniform1i(uniform("oRest"), b.oRest);
            glUniform1i(uniform("oInv"), b.oInv);
            glUniform1i(uniform("oPin"), b.oPin);
            glUniform1i(uniform("oTether"), b.oTether);
            const GLint passAt = uniform("pass"), tickAt = uniform("tick"),
                        subAt = uniform("sub"), colorAt = uniform("color"),
                        firstAt = uniform("firstIteration"),
                        lastAt = uniform("lastIteration");
            const auto groups = [](size_t items) {
                return GLuint((items + RigExecGpuClothLocalSize - 1) /
                              RigExecGpuClothLocalSize);
            };
            // The CPU solve's order, one pass per dispatch; each pass reads
            // what the one before it wrote.
            const auto run = [&](GLuint count) {
                if (count) glDispatchCompute(count, 1, 1);
                ++dispatches;
                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            };
            glUniform1i(passAt, 0);
            run(groups(n));
            const size_t colors = mesh.ColorCount();
            for (size_t k = 1; k < job.tickCount; ++k) {
                glUniform1i(tickAt, GLint(k));
                for (int sub = 1; sub <= substeps; ++sub) {
                    glUniform1i(subAt, sub);
                    if (collide) {
                        glUniform1i(passAt, 3);
                        run(groups(std::max(m, size_t(b.triangles))));
                    }
                    for (int it = 0; it < iterations; ++it) {
                        glUniform1i(firstAt, it == 0 ? 1 : 0);
                        glUniform1i(lastAt, it + 1 == iterations ? 1 : 0);
                        glUniform1i(passAt, 1);
                        for (size_t c = 0; c < colors; ++c) {
                            glUniform1i(colorAt, GLint(c));
                            run(groups(mesh.colorStart[c + 1] - mesh.colorStart[c]));
                        }
                        glUniform1i(passAt, 2);
                        run(groups(n));
                    }
                }
            }
            submitMs += ms(s0, now());
            used.back() = &b;
        }
        glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
        const auto r0 = now();
        for (size_t j = 0; j < jobs->size(); ++j) {
            _ClothBuffers *b = used[j];
            if (!b) continue;
            RigExecGpuClothJob &job = (*jobs)[j];
            const size_t floats = 3 * job.mesh->pointCount;
            // Waits for the dispatch that wrote it.
            glGetNamedBufferSubData(b->out, 0, GLsizeiptr(floats * 4), job.out);
            bool finite = true;
            for (size_t i = 0; i < floats && finite; ++i)
                finite = std::isfinite(job.out[i]);
            job.ok = finite;
            if (finite) ++clothSolves;
        }
        if (stats) {
            std::fprintf(stderr, "GPUCLOTH grids %.3f upload %.3f submit %.3f "
                         "wait+read %.3f ms, %zu dispatches\n",
                         gridMs, uploadMs, submitMs, ms(r0, now()), dispatches);
        }
        return glGetError() == GL_NO_ERROR;
    }

    void Release(_ChainBuffers *b)
    {
        if (b->out && b->mappedOut) glUnmapNamedBuffer(b->out);
        if (b->status && b->mappedStatus) glUnmapNamedBuffer(b->status);
        for (GLuint *id : {&b->ops, &b->staticF, &b->staticI, &b->frame,
                           &b->base, &b->out, &b->status})
            if (*id) glDeleteBuffers(1, id);
        *b = _ChainBuffers();
    }

    bool RunOnWorker(std::vector<RigExecGpuDeformJob> *jobs)
    {
        if (!program) return false;
        static const bool stats =
            TfGetenvBool("RIGEXEC_GPU_DEFORM_STATS", false);
        const auto now = [] { return std::chrono::steady_clock::now(); };
        const auto t0 = now();
        constexpr GLbitfield mapped = GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT |
                                      GL_MAP_COHERENT_BIT;
        std::vector<_ChainBuffers *> used;
        glUseProgram(program);
        for (RigExecGpuDeformJob &job : *jobs) {
            job.ok = false;
            const RigExecGpuDeformFrame *f = job.frame;
            if (!f || !f->layout || !job.base || !job.out) {
                used.push_back(nullptr);
                continue;
            }
            const RigExecGpuDeformLayout &L = *f->layout;
            _ChainBuffers &b = chains[job.chain];
            const size_t pointBytes = size_t(L.pointCount) * 12;
            if (b.layoutSerial != L.serial || b.pointCount != L.pointCount) {
                Release(&b);
                b.layoutSerial = L.serial;
                b.pointCount = L.pointCount;
                b.ops = _ImmutableBuffer(L.ops.size() * 4, L.ops.data(), 0);
                b.staticF = _ImmutableBuffer(L.staticF.size() * 4,
                                             L.staticF.data(), 0);
                b.staticI = _ImmutableBuffer(L.staticI.size() * 4,
                                             L.staticI.data(), 0);
                b.base = _ImmutableBuffer(pointBytes, job.base,
                                          GL_DYNAMIC_STORAGE_BIT);
                b.baseVersion = job.baseVersion;
                b.out = _ImmutableBuffer(pointBytes, nullptr,
                                         mapped | GL_CLIENT_STORAGE_BIT);
                b.mappedOut = static_cast<const float *>(glMapNamedBufferRange(
                    b.out, 0, GLsizeiptr(pointBytes ? pointBytes : 4), mapped));
                b.status = _ImmutableBuffer(
                    4, nullptr, mapped | GL_MAP_WRITE_BIT | GL_CLIENT_STORAGE_BIT);
                b.mappedStatus = static_cast<int32_t *>(glMapNamedBufferRange(
                    b.status, 0, 4, mapped | GL_MAP_WRITE_BIT));
            }
            if (!b.mappedOut || !b.mappedStatus) {
                used.push_back(nullptr);
                continue;
            }
            const size_t frameBytes = f->frame.size() * 4;
            if (!b.frame || b.frameBytes != frameBytes) {
                if (b.frame) glDeleteBuffers(1, &b.frame);
                b.frame = _ImmutableBuffer(frameBytes, f->frame.data(),
                                           GL_DYNAMIC_STORAGE_BIT);
                b.frameBytes = frameBytes;
            } else if (frameBytes) {
                glNamedBufferSubData(b.frame, 0, GLsizeiptr(frameBytes),
                                     f->frame.data());
            }
            if (b.baseVersion != job.baseVersion && pointBytes) {
                glNamedBufferSubData(b.base, 0, GLsizeiptr(pointBytes),
                                     job.base);
                b.baseVersion = job.baseVersion;
            }
            *b.mappedStatus = 0;
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _Ops, b.ops);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _StaticF, b.staticF);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _StaticI, b.staticI);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _Frame, b.frame);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _Base, b.base);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _Out, b.out);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, _Status, b.status);
            glUniform1i(pointCountLocation, GLint(L.pointCount));
            glUniform1i(opCountLocation, GLint(L.OpCount()));
            glDispatchCompute(
                GLuint((L.pointCount + _LocalSize - 1) / _LocalSize), 1, 1);
            used.push_back(&b);
        }
        // The kernel's writes to the mapped buffers are visible to the CPU
        // once the fence signals.
        glMemoryBarrier(GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT);
        GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        const auto t1 = now();
        GLenum wait = GL_TIMEOUT_EXPIRED;
        GLbitfield flags = GL_SYNC_FLUSH_COMMANDS_BIT;
        while (wait == GL_TIMEOUT_EXPIRED) {
            wait = glClientWaitSync(fence, flags, 1000000000ull);
            flags = 0;
        }
        glDeleteSync(fence);
        const auto t2 = now();
        if (wait == GL_WAIT_FAILED) return false;
        for (size_t j = 0; j < jobs->size(); ++j) {
            _ChainBuffers *b = used[j];
            if (!b) continue;
            RigExecGpuDeformJob &job = (*jobs)[j];
            std::memcpy(job.out, b->mappedOut, size_t(b->pointCount) * 12);
            job.ok = *b->mappedStatus == 0;
        }
        if (stats) {
            const auto ms = [](auto a, auto b) {
                return std::chrono::duration<double, std::milli>(b - a).count();
            };
            std::fprintf(stderr, "GPUEXEC submit %.3f wait %.3f copy %.3f ms\n",
                         ms(t0, t1), ms(t1, t2), ms(t2, now()));
        }
        return true;
    }
};

RigExecGpuDeformExecutor::RigExecGpuDeformExecutor()
    : _impl(std::make_unique<Impl>())
{
}

RigExecGpuDeformExecutor::~RigExecGpuDeformExecutor()
{
    _impl->Shutdown();
}

RigExecGpuDeformExecutor *
RigExecGpuDeformExecutor::Get()
{
    // Started once; a device that could not be created is remembered as
    // null rather than retried every frame.
    static RigExecGpuDeformExecutor *const executor = [] {
        auto *e = new RigExecGpuDeformExecutor();
        if (!e->_impl->Start()) {
            delete e;
            return static_cast<RigExecGpuDeformExecutor *>(nullptr);
        }
        return e;
    }();
    return executor;
}

bool
RigExecGpuDeformExecutor::Run(std::vector<RigExecGpuDeformJob> *jobs)
{
    if (!jobs) return false;
    const auto start = std::chrono::steady_clock::now();
    const bool ok =
        _impl->Post([this, jobs] { return _impl->RunOnWorker(jobs); });
    _impl->lastRunMs = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start).count();
    return ok;
}

bool
RigExecGpuDeformExecutor::RunCloth(std::vector<RigExecGpuClothJob> *jobs)
{
    if (!jobs) return false;
    return _impl->Post([this, jobs] { return _impl->RunClothOnWorker(jobs); });
}

size_t
RigExecGpuDeformExecutor::ClothSolveCount() const
{
    return _impl->clothSolves.load();
}

double
RigExecGpuDeformExecutor::LastRunMs() const
{
    return _impl->lastRunMs;
}

std::string
RigExecGpuDeformExecutor::Describe() const
{
    return _impl->Post([this] { return _impl->description; });
}

namespace {

bool
_RunOnDevice(std::vector<RigExecGpuDeformJob> *jobs)
{
    RigExecGpuDeformExecutor *executor = RigExecGpuDeformExecutor::Get();
    return executor && executor->Run(jobs);
}

bool
_RunClothOnDevice(std::vector<RigExecGpuClothJob> *jobs)
{
    RigExecGpuDeformExecutor *executor = RigExecGpuDeformExecutor::Get();
    return executor && executor->RunCloth(jobs);
}

}  // namespace

void
RigExecGpuRegisterDeformRunner()
{
    RigExecSetGpuDeformRunner(&_RunOnDevice);
}

void
RigExecGpuRegisterClothRunner()
{
    RigExecSetGpuClothRunner(&_RunClothOnDevice);
}

}  // namespace rigExec
