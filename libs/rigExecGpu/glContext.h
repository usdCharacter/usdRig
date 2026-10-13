// A hidden OpenGL context owned by one thread: created, made current and
// destroyed there. The GPU executor's device lives in it, apart from any
// viewer's context.
#ifndef RIGEXEC_GPU_GL_CONTEXT_H
#define RIGEXEC_GPU_GL_CONTEXT_H

namespace rigExec {

class RigExecGpuGLContext {
public:
    RigExecGpuGLContext() = default;
    ~RigExecGpuGLContext();
    RigExecGpuGLContext(const RigExecGpuGLContext &) = delete;
    RigExecGpuGLContext &operator=(const RigExecGpuGLContext &) = delete;

    /// Creates the context and makes it current on the calling thread.
    /// False where the platform has no implementation or creation failed.
    bool Create();
    void Destroy();

private:
    void *_window = nullptr;
    void *_dc = nullptr;
    void *_rc = nullptr;
};

}  // namespace rigExec

#endif  // RIGEXEC_GPU_GL_CONTEXT_H
