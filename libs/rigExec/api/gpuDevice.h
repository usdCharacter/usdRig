#ifndef RIGEXEC_API_GPU_DEVICE_H
#define RIGEXEC_API_GPU_DEVICE_H

/// The compute device GPU-capable ops run on. A host implements
/// RigExecGpuDevice on its backend; RigExecCreateCpuReferenceGpuDevice makes
/// the reference device every backend is judged against
/// (docs/specs/gpu-device.md).
///
/// Threading: a device belongs to the thread that created it. Every member
/// but Capabilities runs on that thread alone; the reference device refuses
/// a call from any other. Op bodies never call a device. A worker records
/// its work into a RigExecGpuCommandList, which is plain data and touches no
/// device, and hands the list to the owning thread through the host's queue
/// at a publication boundary; the owning thread submits it and publishes the
/// downloads once the fence completes.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

enum class RigExecGpuBackend : uint32_t {
    CpuReference = 0,
    Vulkan = 1,
    Cuda = 2
};

/// A buffer of one device; generation 0 is no buffer. A destroyed buffer's
/// handle stays invalid when its slot is reused.
struct RigExecGpuBuffer {
    uint32_t index = 0;
    uint32_t generation = 0;
    bool IsValid() const
    {
        return generation != 0;
    }
};

/// A kernel registered on one device.
struct RigExecGpuKernel {
    uint32_t index = ~uint32_t(0);
    bool IsValid() const
    {
        return index != ~uint32_t(0);
    }
};

/// A submission's completion point; values rise with each submission and
/// 0 is no submission.
struct RigExecGpuFence {
    uint64_t value = 0;
};

/// What a buffer is for; a command that uses a buffer in a way its usage
/// does not name is refused.
enum RigExecGpuBufferUsage : uint32_t {
    RigExecGpuBufferStorage = 1u << 0,
    RigExecGpuBufferUploadTarget = 1u << 1,
    RigExecGpuBufferDownloadSource = 1u << 2,
};

/// A buffer is untyped bytes, zeroed at creation; layout belongs to the
/// kernels that bind it.
struct RigExecGpuBufferDesc {
    uint64_t bytes = 0;
    uint32_t usage = 0;
};

enum class RigExecGpuAccess : uint8_t { Read, Write, ReadWrite };

struct RigExecGpuCapabilities {
    RigExecGpuBackend backend = RigExecGpuBackend::CpuReference;
    uint64_t maxBufferBytes = 0;
    /// Every binding offset is a multiple of this; at most 256 on every
    /// backend, so a 256-aligned column binds as it lies.
    uint32_t bindingOffsetAlignment = 0;
    uint32_t maxBindings = 0;
    uint32_t maxUniformBytes = 0;
    /// The kernels' results equal the host kernels' bit for bit.
    bool hostExact = false;
};

/// One bound byte range as a host kernel sees it; data is null for an
/// empty range. Ranges are only as aligned as the binding offset alignment.
struct RigExecGpuHostRange {
    unsigned char *data = nullptr;
    uint64_t bytes = 0;
};

/// The invocations [begin, end) of one dispatch, run by a host kernel. Each
/// invocation reads and writes only what its own index selects, so results
/// do not depend on how invocations are grouped.
struct RigExecGpuHostInvocation {
    const RigExecGpuHostRange *bindings = nullptr;
    uint32_t bindingCount = 0;
    const unsigned char *uniforms = nullptr;
    uint32_t uniformBytes = 0;
    uint32_t begin = 0;
    uint32_t end = 0;
};

using RigExecGpuHostKernelFn = void (*)(const RigExecGpuHostInvocation &);

/// A compiled kernel for one backend: SPIR-V words for Vulkan, PTX or a
/// cubin for CUDA.
struct RigExecGpuKernelCode {
    RigExecGpuBackend backend = RigExecGpuBackend::Vulkan;
    const void *bytes = nullptr;
    size_t size = 0;
};

/// A kernel and its binding signature. Registration copies everything but
/// the code bytes, which stay valid for the device's life. A device takes
/// the code for its own backend; the reference device runs \p host.
struct RigExecGpuKernelDesc {
    const RigExecGpuAccess *access = nullptr;
    uint32_t bindingCount = 0;
    uint32_t uniformBytes = 0;
    /// Invocations per workgroup; must equal the compiled code's.
    uint32_t workgroupSize = 64;
    RigExecGpuHostKernelFn host = nullptr;
    const RigExecGpuKernelCode *code = nullptr;
    uint32_t codeCount = 0;
    /// For errors and profiling only; nothing is looked up by it.
    const char *debugName = nullptr;
};

/// A byte range of a buffer bound to one kernel binding.
struct RigExecGpuBinding {
    RigExecGpuBuffer buffer;
    uint64_t offset = 0;
    uint64_t bytes = 0;
};

/// Commands for one submission, run in order: each sees the writes of the
/// ones before it. Built on any thread; holds no device state. Upload
/// sources are read during Submit. Download destinations are written by
/// the time the submission's fence completes and must stay valid until
/// then. Clear keeps the capacity for the next frame.
class RigExecGpuCommandList {
public:
    enum class Kind : uint8_t { Upload, Dispatch, Download };

    struct Command {
        Kind kind = Kind::Upload;
        RigExecGpuBuffer buffer;
        uint64_t offset = 0;
        uint64_t bytes = 0;
        const void *source = nullptr;
        void *destination = nullptr;
        RigExecGpuKernel kernel;
        uint32_t invocations = 0;
        /// Dispatch: its bindings and uniform bytes in Bindings() and
        /// Uniforms().
        uint32_t firstBinding = 0;
        uint32_t bindingCount = 0;
        uint32_t firstUniform = 0;
        uint32_t uniformBytes = 0;
    };

    void Upload(RigExecGpuBuffer buffer, uint64_t offset, const void *source,
                uint64_t bytes)
    {
        Command c;
        c.kind = Kind::Upload;
        c.buffer = buffer;
        c.offset = offset;
        c.bytes = bytes;
        c.source = source;
        _commands.push_back(c);
    }

    void Dispatch(RigExecGpuKernel kernel, const RigExecGpuBinding *bindings,
                  uint32_t bindingCount, const void *uniforms,
                  uint32_t uniformBytes, uint32_t invocations)
    {
        Command c;
        c.kind = Kind::Dispatch;
        c.kernel = kernel;
        c.invocations = invocations;
        c.firstBinding = uint32_t(_bindings.size());
        c.bindingCount = bindingCount;
        c.firstUniform = uint32_t(_uniforms.size());
        c.uniformBytes = uniformBytes;
        _bindings.insert(_bindings.end(), bindings, bindings + bindingCount);
        _uniforms.resize(_uniforms.size() + uniformBytes);
        if (uniformBytes)
            std::memcpy(_uniforms.data() + c.firstUniform, uniforms,
                        uniformBytes);
        _commands.push_back(c);
    }

    void Download(RigExecGpuBuffer buffer, uint64_t offset, void *destination,
                  uint64_t bytes)
    {
        Command c;
        c.kind = Kind::Download;
        c.buffer = buffer;
        c.offset = offset;
        c.bytes = bytes;
        c.destination = destination;
        _commands.push_back(c);
    }

    void Clear()
    {
        _commands.clear();
        _bindings.clear();
        _uniforms.clear();
    }

    const std::vector<Command> &Commands() const
    {
        return _commands;
    }
    const std::vector<RigExecGpuBinding> &Bindings() const
    {
        return _bindings;
    }
    const std::vector<unsigned char> &Uniforms() const
    {
        return _uniforms;
    }

private:
    std::vector<Command> _commands;
    std::vector<RigExecGpuBinding> _bindings;
    std::vector<unsigned char> _uniforms;
};

class RigExecGpuDevice {
public:
    virtual ~RigExecGpuDevice() = default;

    /// Fixed at creation; any thread may read it.
    virtual RigExecGpuCapabilities Capabilities() const = 0;

    /// Registers \p desc once, before it is dispatched; an invalid handle,
    /// with \p error, when this device has no code for it.
    virtual RigExecGpuKernel RegisterKernel(const RigExecGpuKernelDesc &desc,
                                            std::string *error) = 0;

    virtual RigExecGpuBuffer CreateBuffer(const RigExecGpuBufferDesc &desc,
                                          std::string *error) = 0;
    /// The storage outlives the handle until every submission that names
    /// the buffer has completed.
    virtual void DestroyBuffer(RigExecGpuBuffer buffer) = 0;

    /// Checks every command, then runs them all: a fence, or fence 0 with
    /// \p error and nothing run. A dispatch is refused when its kernel,
    /// binding count or uniform size differs from the registered kernel's,
    /// a range leaves its buffer or misses the offset alignment, or two of
    /// its ranges overlap where either is written.
    virtual RigExecGpuFence Submit(const RigExecGpuCommandList &commands,
                                   std::string *error) = 0;
    virtual bool IsComplete(RigExecGpuFence fence) const = 0;
    /// Blocks the owning thread until \p fence completes; false, with
    /// \p error, for a fence never submitted or a lost device.
    virtual bool Wait(RigExecGpuFence fence, std::string *error) = 0;
};

/// Plain C++ on the calling thread, which owns it: buffers are host memory,
/// a dispatch runs the kernel's host function workgroup by workgroup in
/// index order, and Submit completes before it returns.
std::unique_ptr<RigExecGpuDevice> RigExecCreateCpuReferenceGpuDevice();

} // namespace rigExec

#endif
