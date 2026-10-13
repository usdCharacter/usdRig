#include "rigExecGpu/gpuDeformExecutor.h"

#include "rigExec/gpuDeformLowering.h"
#include "rigExecGpu/glContext.h"
#include "rigExecGpu/gpuDeformShader.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/imaging/garch/glApi.h"

#include <chrono>
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

}  // namespace

void
RigExecGpuRegisterDeformRunner()
{
    RigExecSetGpuDeformRunner(&_RunOnDevice);
}

}  // namespace rigExec
