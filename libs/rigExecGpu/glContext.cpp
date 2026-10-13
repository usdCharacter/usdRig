#include "rigExecGpu/glContext.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace rigExec {

#if defined(_WIN32)

namespace {

const char *const _className = "RigExecGpuHiddenWindow";

bool
_RegisterClass()
{
    static const bool registered = [] {
        WNDCLASSA wc = {};
        wc.style = CS_OWNDC;
        wc.lpfnWndProc = DefWindowProcA;
        wc.hInstance = GetModuleHandleA(nullptr);
        wc.lpszClassName = _className;
        return RegisterClassA(&wc) != 0 ||
               GetLastError() == ERROR_CLASS_ALREADY_EXISTS;
    }();
    return registered;
}

}  // namespace

bool
RigExecGpuGLContext::Create()
{
    if (_rc) return true;
    if (!_RegisterClass()) return false;
    // Never shown: the context only needs a pixel format to exist.
    HWND window = CreateWindowExA(0, _className, "", WS_OVERLAPPEDWINDOW, 0, 0,
                                  16, 16, nullptr, nullptr,
                                  GetModuleHandleA(nullptr), nullptr);
    if (!window) return false;
    HDC dc = GetDC(window);
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;
    const int format = ChoosePixelFormat(dc, &pfd);
    if (!format || !SetPixelFormat(dc, format, &pfd)) {
        ReleaseDC(window, dc);
        DestroyWindow(window);
        return false;
    }
    // A legacy context is the driver's highest compatibility profile.
    HGLRC rc = wglCreateContext(dc);
    if (!rc || !wglMakeCurrent(dc, rc)) {
        if (rc) wglDeleteContext(rc);
        ReleaseDC(window, dc);
        DestroyWindow(window);
        return false;
    }
    _window = window;
    _dc = dc;
    _rc = rc;
    return true;
}

void
RigExecGpuGLContext::Destroy()
{
    if (!_rc) return;
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(HGLRC(_rc));
    ReleaseDC(HWND(_window), HDC(_dc));
    DestroyWindow(HWND(_window));
    _rc = _dc = _window = nullptr;
}

#else

bool
RigExecGpuGLContext::Create()
{
    return false;
}

void
RigExecGpuGLContext::Destroy()
{
}

#endif

RigExecGpuGLContext::~RigExecGpuGLContext()
{
    Destroy();
}

}  // namespace rigExec
