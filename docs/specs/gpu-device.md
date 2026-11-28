# GPU device

`RigExecGpuDevice` ([gpuDevice.h](../../libs/rigExec/api/gpuDevice.h)) is the
interface GPU-capable RigExec ops will run on. A host implements it on its
backend, the way it implements `RigExecTaskSystem` on its thread pool. It is
USD-free and names no graphics API. Nothing evaluates through it yet: it
exists with a CPU reference device and one worked kernel, so the backends
have a contract and a judge before they are written.

## Today's GPU path

[libs/rigExecGpu](../../libs/rigExecGpu/gpuDeformExecutor.h) runs per-vertex
deformation and cloth as OpenGL 4.5 compute shaders. One worker thread owns
a hidden GL context; callers on any thread post work to it through a queue
at a publication boundary and wait for the result. A point chain is
translated into one per-vertex program
([gpuDeformProgram.h](../../libs/rigExecMath/gpuDeformProgram.h)) whose
float CPU mirror judges the shader to a tolerance. Points travel as
interleaved x, y, z floats, and the library links `garch`.

## The contract

- **One owning thread.** A device belongs to the thread that created it.
  Every member but `Capabilities` runs on that thread; the reference device
  refuses calls from any other. This is the GL executor's worker thread made
  explicit.
- **Command lists carry work across threads.** Op bodies never call a
  device. A worker records uploads, dispatches and downloads into a
  `RigExecGpuCommandList`, which is plain data, and hands it to the owning
  thread through the host's queue. No op body takes a lock for this. The
  owning thread submits the list and publishes the downloaded values once
  the fence completes.
- **Kernels by handle.** A kernel is registered once with its binding
  signature (count and read/write access per binding), its uniform block
  size, its workgroup size, and code per backend. Dispatch names the
  returned handle. The debug name only appears in errors.
- **Untyped buffers, byte-range bindings.** A buffer is bytes with a usage
  mask. A binding is a buffer, an offset and a size, and the offset is a
  multiple of `bindingOffsetAlignment`, at most 256 on every backend.
- **All or nothing.** `Submit` checks every command first. A dispatch is
  refused when its bindings or uniform size differ from the kernel's, when
  a range leaves its buffer or misses the alignment, or when two of its
  ranges overlap and either is written. A refused list runs nothing.
  Commands run in list order, and each sees the writes before it.
- **Fences.** Each submission returns a fence. Download destinations are
  written by the time it completes. A destroyed buffer's storage lives until
  every submission that names it completes. Its handle carries a generation,
  so a stale handle is refused after its slot is reused.

## Column layout

The interface knows no element type, stride or vector width, so it fixes no
layout. Today's interleaved points are the GL executor's own format; the
interface does not carry them over. The store's float SoA column arenas
(step 8) bind as they lie. An arena is one buffer, and each column is a
binding at its own offset: a column of 4 KiB or more starts on 256 bytes.
A small-channel block is 256-aligned and bound once, with its columns'
offsets passed in the uniform block. Moving an output to the device is
then one upload per changed column, with no repacking.

The worked kernel shows the shape. MatrixDeform as a device kernel
([matrixDeformDevice.h](../../libs/rigExecOps/deform/matrixDeformDevice.h))
reads x, y and z from three columns and the weights from a fourth, writes
three output columns, and takes the transform in its uniform block.

## The CPU reference device

`RigExecCreateCpuReferenceGpuDevice` implements the interface in plain C++.
Buffers are 256-aligned host memory. A dispatch runs the kernel's host
function workgroup by workgroup in index order, and `Submit` completes
before it returns. It reports the strictest limits a backend may have:
256-byte binding offsets and a 128-byte uniform block, which is Vulkan's
guaranteed push-constant size.

A kernel's host code calls the shared CPU kernel, so the reference device's
answer is the CPU op's answer. `testRigExecGpuDevice` runs MatrixDeform
through the device and compares it bit for bit with the op's matrix kernel
(`ApplyMatrixKernelRange`). It covers painted, uniform and full weights,
two chained dispatches in one submission, four workgroup sizes, and a list
recorded on a worker thread and submitted by the owner. It also checks that
each malformed submission is refused before anything runs. The test links
no OpenUSD library, so CI without a GPU runs the whole path.

## What moves at step 11

- A Vulkan backend comes first, then CUDA. Each takes kernels' SPIR-V or
  PTX through `RigExecGpuKernelCode` and is judged against the reference
  device in units in the last place: fused multiply-add and reduction order
  may differ in the last bits.
- Ops gain device lowering on their descriptors. An op that can run on the
  device names its kernel, and compile registers it once and records
  dispatches into the program's command lists.
- The GL executor's deform and cloth work moves onto registered kernels.
  `rigExecGpu` drops OpenGL and `garch`, and imaging stops sharing a GL
  context with it.
- Device outputs are store columns. The USD adapter interleaves them for
  Storm as it does host columns.

## Open questions

- Where the host's queue lives. The GL executor has its own; with several
  ops recording lists, one queue per evaluator or one per device decides
  how submissions from one frame are merged.
- Whether downloads stay explicit. On unified-memory devices a mapped
  buffer could replace the copy; the interface would then need a map call
  with the same fence rule.
- Uniform blocks over 128 bytes, such as skin palettes. These become
  storage bindings, not uniforms, so the limit stays Vulkan's guaranteed
  push-constant size.
- Asynchronous submission. A backend can complete fences after `Submit`
  returns. How evaluation overlaps host work with device work, and where
  it waits, is a scheduling decision that is still open.
