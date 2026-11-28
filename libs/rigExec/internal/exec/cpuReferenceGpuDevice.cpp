#include "rigExec/api/gpuDevice.h"

#include <algorithm>
#include <thread>
#include <utility>

namespace rigExec {

namespace {

constexpr uint32_t kOffsetAlignment = 256;
constexpr uint64_t kCopyAlignment = 4;
constexpr uint32_t kMaxBindings = 16;
constexpr uint32_t kMaxUniformBytes = 128;
constexpr uint64_t kMaxBufferBytes = uint64_t(1) << 32;

struct BufferSlot {
    std::unique_ptr<unsigned char[]> storage;
    unsigned char *base = nullptr;
    uint64_t bytes = 0;
    uint32_t usage = 0;
    uint32_t generation = 1;
    bool alive = false;
};

struct KernelEntry {
    std::vector<RigExecGpuAccess> access;
    uint32_t uniformBytes = 0;
    uint32_t workgroupSize = 0;
    RigExecGpuHostKernelFn host = nullptr;
    std::string name;
};

bool
Fail(std::string *error, std::string message)
{
    if (error)
        *error = std::move(message);
    return false;
}

bool
Writes(RigExecGpuAccess access)
{
    return access != RigExecGpuAccess::Read;
}

class CpuReferenceDevice final : public RigExecGpuDevice {
public:
    RigExecGpuCapabilities Capabilities() const override
    {
        RigExecGpuCapabilities caps;
        caps.backend = RigExecGpuBackend::CpuReference;
        caps.maxBufferBytes = kMaxBufferBytes;
        caps.bindingOffsetAlignment = kOffsetAlignment;
        caps.maxBindings = kMaxBindings;
        caps.maxUniformBytes = kMaxUniformBytes;
        caps.hostExact = true;
        return caps;
    }

    RigExecGpuKernel RegisterKernel(const RigExecGpuKernelDesc &desc,
                                    std::string *error) override
    {
        if (!OnOwner(error))
            return {};
        const std::string name = desc.debugName ? desc.debugName : "kernel";
        if (!desc.host) {
            Fail(error, name + ": no host code for the reference device");
            return {};
        }
        if (desc.bindingCount > kMaxBindings ||
            (desc.bindingCount && !desc.access)) {
            Fail(error, name + ": bindings exceed the device or lack access");
            return {};
        }
        if (desc.uniformBytes > kMaxUniformBytes || !desc.workgroupSize) {
            Fail(error, name + ": uniform block or workgroup size unusable");
            return {};
        }
        KernelEntry entry;
        entry.access.assign(desc.access, desc.access + desc.bindingCount);
        entry.uniformBytes = desc.uniformBytes;
        entry.workgroupSize = desc.workgroupSize;
        entry.host = desc.host;
        entry.name = name;
        _kernels.push_back(std::move(entry));
        RigExecGpuKernel kernel;
        kernel.index = uint32_t(_kernels.size() - 1);
        return kernel;
    }

    RigExecGpuBuffer CreateBuffer(const RigExecGpuBufferDesc &desc,
                                  std::string *error) override
    {
        if (!OnOwner(error))
            return {};
        if (!desc.bytes || desc.bytes > kMaxBufferBytes || !desc.usage) {
            Fail(error, "buffer size or usage unusable");
            return {};
        }
        uint32_t index = uint32_t(_buffers.size());
        if (!_free.empty()) {
            index = _free.back();
            _free.pop_back();
        } else {
            _buffers.emplace_back();
        }
        BufferSlot &slot = _buffers[index];
        slot.storage = std::make_unique<unsigned char[]>(size_t(desc.bytes) +
                                                         kOffsetAlignment - 1);
        const uintptr_t raw = reinterpret_cast<uintptr_t>(slot.storage.get());
        slot.base =
            slot.storage.get() +
            (kOffsetAlignment - raw % kOffsetAlignment) % kOffsetAlignment;
        slot.bytes = desc.bytes;
        slot.usage = desc.usage;
        slot.alive = true;
        RigExecGpuBuffer buffer;
        buffer.index = index;
        buffer.generation = slot.generation;
        return buffer;
    }

    void DestroyBuffer(RigExecGpuBuffer buffer) override
    {
        if (!OnOwner(nullptr) || !Find(buffer))
            return;
        BufferSlot &slot = _buffers[buffer.index];
        slot.storage.reset();
        slot.base = nullptr;
        slot.bytes = 0;
        slot.alive = false;
        if (++slot.generation == 0)
            slot.generation = 1;
        _free.push_back(buffer.index);
    }

    RigExecGpuFence Submit(const RigExecGpuCommandList &list,
                           std::string *error) override
    {
        if (!OnOwner(error) || !Check(list, error))
            return {};
        for (const RigExecGpuCommandList::Command &c : list.Commands())
            Run(list, c);
        RigExecGpuFence fence;
        fence.value = ++_submitted;
        return fence;
    }

    bool IsComplete(RigExecGpuFence fence) const override
    {
        return std::this_thread::get_id() == _owner && fence.value &&
               fence.value <= _submitted;
    }

    bool Wait(RigExecGpuFence fence, std::string *error) override
    {
        if (!OnOwner(error))
            return false;
        if (!fence.value || fence.value > _submitted)
            return Fail(error, "fence was never submitted");
        return true;
    }

private:
    bool OnOwner(std::string *error) const
    {
        return std::this_thread::get_id() == _owner ||
               Fail(error, "device called off its owning thread");
    }

    const BufferSlot *Find(RigExecGpuBuffer buffer) const
    {
        if (!buffer.IsValid() || buffer.index >= _buffers.size())
            return nullptr;
        const BufferSlot &slot = _buffers[buffer.index];
        return slot.alive && slot.generation == buffer.generation ? &slot
                                                                  : nullptr;
    }

    static bool Inside(const BufferSlot &slot, uint64_t offset, uint64_t bytes)
    {
        return offset <= slot.bytes && bytes <= slot.bytes - offset;
    }

    bool CheckCopy(const RigExecGpuCommandList::Command &c, uint32_t usage,
                   const void *host, std::string *error) const
    {
        const BufferSlot *slot = Find(c.buffer);
        if (!slot)
            return Fail(error, "copy names no live buffer");
        if (!(slot->usage & usage))
            return Fail(error, "copy uses a buffer against its usage");
        if (!Inside(*slot, c.offset, c.bytes) || c.offset % kCopyAlignment ||
            c.bytes % kCopyAlignment)
            return Fail(error, "copy range leaves its buffer or is unaligned");
        if (c.bytes && !host)
            return Fail(error, "copy has no host memory");
        return true;
    }

    bool CheckDispatch(const RigExecGpuCommandList &list,
                       const RigExecGpuCommandList::Command &c,
                       std::string *error) const
    {
        if (!c.kernel.IsValid() || c.kernel.index >= _kernels.size())
            return Fail(error, "dispatch names no registered kernel");
        const KernelEntry &kernel = _kernels[c.kernel.index];
        if (c.bindingCount != kernel.access.size() ||
            c.uniformBytes != kernel.uniformBytes)
            return Fail(error, kernel.name +
                                   ": bindings or uniform size differ from "
                                   "the registered kernel");
        const RigExecGpuBinding *bindings =
            list.Bindings().data() + c.firstBinding;
        for (uint32_t i = 0; i < c.bindingCount; ++i) {
            const RigExecGpuBinding &b = bindings[i];
            const BufferSlot *slot = Find(b.buffer);
            if (!slot || !(slot->usage & RigExecGpuBufferStorage))
                return Fail(error, kernel.name + ": binding " +
                                       std::to_string(i) +
                                       " names no live storage buffer");
            if (!Inside(*slot, b.offset, b.bytes) ||
                b.offset % kOffsetAlignment)
                return Fail(error, kernel.name + ": binding " +
                                       std::to_string(i) +
                                       " leaves its buffer or is unaligned");
            for (uint32_t j = 0; j < i; ++j) {
                const RigExecGpuBinding &o = bindings[j];
                if (o.buffer.index != b.buffer.index || !o.bytes || !b.bytes ||
                    !(Writes(kernel.access[i]) || Writes(kernel.access[j])))
                    continue;
                if (o.offset < b.offset + b.bytes &&
                    b.offset < o.offset + o.bytes)
                    return Fail(error, kernel.name + ": bindings " +
                                           std::to_string(j) + " and " +
                                           std::to_string(i) +
                                           " overlap where one is written");
            }
        }
        return true;
    }

    bool Check(const RigExecGpuCommandList &list, std::string *error) const
    {
        using Kind = RigExecGpuCommandList::Kind;
        for (const RigExecGpuCommandList::Command &c : list.Commands()) {
            const bool ok = c.kind == Kind::Upload
                                ? CheckCopy(c, RigExecGpuBufferUploadTarget,
                                            c.source, error)
                            : c.kind == Kind::Download
                                ? CheckCopy(c, RigExecGpuBufferDownloadSource,
                                            c.destination, error)
                                : CheckDispatch(list, c, error);
            if (!ok)
                return false;
        }
        return true;
    }

    void Run(const RigExecGpuCommandList &list,
             const RigExecGpuCommandList::Command &c)
    {
        using Kind = RigExecGpuCommandList::Kind;
        if (c.kind == Kind::Upload) {
            if (c.bytes)
                std::memcpy(_buffers[c.buffer.index].base + c.offset, c.source,
                            size_t(c.bytes));
            return;
        }
        if (c.kind == Kind::Download) {
            if (c.bytes)
                std::memcpy(c.destination,
                            _buffers[c.buffer.index].base + c.offset,
                            size_t(c.bytes));
            return;
        }
        const KernelEntry &kernel = _kernels[c.kernel.index];
        _ranges.resize(c.bindingCount);
        const RigExecGpuBinding *bindings =
            list.Bindings().data() + c.firstBinding;
        for (uint32_t i = 0; i < c.bindingCount; ++i) {
            const RigExecGpuBinding &b = bindings[i];
            _ranges[i].data =
                b.bytes ? _buffers[b.buffer.index].base + b.offset : nullptr;
            _ranges[i].bytes = b.bytes;
        }
        RigExecGpuHostInvocation invocation;
        invocation.bindings = _ranges.data();
        invocation.bindingCount = c.bindingCount;
        invocation.uniforms = list.Uniforms().data() + c.firstUniform;
        invocation.uniformBytes = c.uniformBytes;
        for (uint32_t begin = 0; begin < c.invocations;
             begin += std::min(kernel.workgroupSize, c.invocations - begin)) {
            invocation.begin = begin;
            invocation.end =
                begin + std::min(kernel.workgroupSize, c.invocations - begin);
            kernel.host(invocation);
        }
    }

    const std::thread::id _owner = std::this_thread::get_id();
    std::vector<BufferSlot> _buffers;
    std::vector<uint32_t> _free;
    std::vector<KernelEntry> _kernels;
    std::vector<RigExecGpuHostRange> _ranges;
    uint64_t _submitted = 0;
};

} // namespace

std::unique_ptr<RigExecGpuDevice>
RigExecCreateCpuReferenceGpuDevice()
{
    return std::make_unique<CpuReferenceDevice>();
}

} // namespace rigExec
