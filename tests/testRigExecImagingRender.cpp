// Renders a rig through Storm headlessly and compares the color AOV with
// checked-in baselines. Set RIGEXEC_RENDER_UPDATE_BASELINES=1 to rewrite them.
#include "pxr/base/gf/frustum.h"
#include "pxr/base/gf/half.h"
#include "pxr/base/gf/rect2i.h"
#include "pxr/imaging/cameraUtil/framing.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/plug/registry.h"
#include "pxr/imaging/hd/renderBuffer.h"
#include "pxr/imaging/hio/image.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/bboxCache.h"
#include "pxr/usdImaging/usdImagingGL/engine.h"
#include "pxr/usdImaging/usdImagingGL/renderParams.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/OpenGL.h>
#include <sys/utsname.h>
#endif

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

constexpr int kSize = 128;
constexpr int kSkip = 77;
// A pixel differs when any channel moves by more than kChannelThreshold;
// the image fails when more than kPixelCountThreshold pixels differ.
constexpr int kChannelThreshold = 8;
constexpr int kPixelCountThreshold = 40;

struct Image {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> rgba;
};

uint8_t
ToByte(float v)
{
    return uint8_t(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
}

bool
Capture(UsdImagingGLEngine &engine, Image &out)
{
    HdRenderBuffer *buffer = engine.GetAovRenderBuffer(HdAovTokens->color);
    if (!buffer)
        return false;
    buffer->Resolve();
    const int w = int(buffer->GetWidth());
    const int h = int(buffer->GetHeight());
    const HdFormat format = buffer->GetFormat();
    const void *data = buffer->Map();
    if (!data)
        return false;
    out.width = w;
    out.height = h;
    out.rgba.assign(size_t(w) * h * 4, 255);
    bool ok = true;
    for (size_t i = 0; i < size_t(w) * h; ++i) {
        for (int c = 0; c < 4; ++c) {
            uint8_t &dst = out.rgba[i * 4 + c];
            if (format == HdFormatUNorm8Vec4) {
                dst = static_cast<const uint8_t *>(data)[i * 4 + c];
            } else if (format == HdFormatFloat16Vec4) {
                dst =
                    ToByte(float(static_cast<const GfHalf *>(data)[i * 4 + c]));
            } else if (format == HdFormatFloat32Vec4) {
                dst = ToByte(static_cast<const float *>(data)[i * 4 + c]);
            } else {
                ok = false;
            }
        }
    }
    buffer->Unmap();
    // Storm writes rows bottom-up relative to image files.
    for (int y = 0; y < h / 2; ++y) {
        std::swap_ranges(out.rgba.begin() + size_t(y) * w * 4,
                         out.rgba.begin() + size_t(y + 1) * w * 4,
                         out.rgba.begin() + size_t(h - 1 - y) * w * 4);
    }
    return ok;
}

bool
Write(const std::string &path, const Image &image)
{
    HioImage::StorageSpec spec;
    spec.width = image.width;
    spec.height = image.height;
    spec.format = HioFormatUNorm8Vec4;
    spec.flipped = false;
    spec.data = const_cast<uint8_t *>(image.rgba.data());
    HioImageSharedPtr file = HioImage::OpenForWriting(path);
    return file && file->Write(spec);
}

bool
Read(const std::string &path, Image &image)
{
    HioImageSharedPtr file = HioImage::OpenForReading(path);
    if (!file || file->GetBytesPerPixel() != 4)
        return false;
    image.width = file->GetWidth();
    image.height = file->GetHeight();
    image.rgba.assign(size_t(image.width) * image.height * 4, 0);
    HioImage::StorageSpec spec;
    spec.width = image.width;
    spec.height = image.height;
    spec.format = file->GetFormat();
    spec.flipped = false;
    spec.data = image.rgba.data();
    return file->Read(spec);
}

int
CountDiffering(const Image &a, const Image &b)
{
    int count = 0;
    for (size_t i = 0; i < size_t(a.width) * a.height; ++i) {
        for (int c = 0; c < 4; ++c) {
            if (std::abs(int(a.rgba[i * 4 + c]) - int(b.rgba[i * 4 + c])) >
                kChannelThreshold) {
                ++count;
                break;
            }
        }
    }
    return count;
}

// Storm's GL backend needs a current context on the calling thread.
bool
MakeHiddenContext()
{
#if defined(__APPLE__)
    const CGLPixelFormatAttribute attributes[] = {
        kCGLPFAOpenGLProfile,
        CGLPixelFormatAttribute(kCGLOGLPVersion_GL4_Core),
        kCGLPFAAccelerated,
        kCGLPFAColorSize,
        CGLPixelFormatAttribute(24),
        kCGLPFADepthSize,
        CGLPixelFormatAttribute(24),
        CGLPixelFormatAttribute(0)};
    CGLPixelFormatObj format = nullptr;
    GLint count = 0;
    if (CGLChoosePixelFormat(attributes, &format, &count) != kCGLNoError ||
        !format) {
        return false;
    }
    CGLContextObj context = nullptr;
    const bool ok =
        CGLCreateContext(format, nullptr, &context) == kCGLNoError &&
        CGLSetCurrentContext(context) == kCGLNoError;
    CGLDestroyPixelFormat(format);
    return ok;
#else
    return false;
#endif
}

// Identifies the GPU, graphics API, OS release and architecture the baselines
// were captured on; pixels are only compared on the same configuration.
std::string
CaptureConfiguration(UsdImagingGLEngine &engine)
{
    std::ostringstream out;
#if defined(__APPLE__)
    const char *renderer =
        reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    utsname name;
    uname(&name);
    out << "renderer: " << (renderer ? renderer : "unknown") << "\n"
        << "hgi: "
        << (engine.GetHgi() ? engine.GetHgi()->GetAPIName().GetString()
                            : std::string("none"))
        << "\n"
        << "os: " << name.sysname << " " << name.release << " " << name.machine
        << "\n";
#else
    (void)engine;
#endif
    return out.str();
}

bool
IsBlank(const Image &image)
{
    for (size_t i = 4; i < image.rgba.size(); i += 4) {
        if (std::abs(int(image.rgba[i]) - int(image.rgba[0])) > 2 ||
            std::abs(int(image.rgba[i + 1]) - int(image.rgba[1])) > 2 ||
            std::abs(int(image.rgba[i + 2]) - int(image.rgba[2])) > 2) {
            return false;
        }
    }
    return true;
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 3) {
        std::printf("usage: %s <examples dir> <baseline dir>\n", argv[0]);
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_IMAGING_RESOURCE_DIR);
    const bool update = std::getenv("RIGEXEC_RENDER_UPDATE_BASELINES");

    UsdStageRefPtr stage =
        UsdStage::Open(std::string(argv[1]) + "/05_TwistRibbonSpine.usda");
    if (!stage) {
        std::printf("FAIL: stage did not open\n");
        return 1;
    }

    if (!MakeHiddenContext()) {
        std::printf("SKIP: no OpenGL context\n");
        return kSkip;
    }
    UsdImagingGLEngine engine;
    if (!engine.SetRendererPlugin(TfToken("HdStormRendererPlugin")) ||
        !engine.SetRendererAov(HdAovTokens->color)) {
        std::printf("SKIP: Storm is unavailable\n");
        return kSkip;
    }
    engine.SetRenderBufferSize(GfVec2i(kSize, kSize));
    engine.SetFraming(CameraUtilFraming(GfRect2i(GfVec2i(0, 0), kSize, kSize)));

    const GfRange3d bounds =
        UsdGeomBBoxCache(UsdTimeCode(1001),
                         {UsdGeomTokens->default_, UsdGeomTokens->guide})
            .ComputeWorldBound(stage->GetPseudoRoot())
            .ComputeAlignedRange();
    const GfVec3d center = bounds.GetMidpoint();
    const double distance = bounds.GetSize().GetLength() * 1.6;
    GfFrustum frustum;
    frustum.SetPerspective(40.0, 1.0, 0.1, 10000.0);
    frustum.SetPositionAndRotationFromMatrix(
        GfMatrix4d(1.0)
            .SetLookAt(center + GfVec3d(0, 0, distance), center,
                       GfVec3d(0, 1, 0))
            .GetInverse());
    engine.SetCameraState(frustum.ComputeViewMatrix(),
                          frustum.ComputeProjectionMatrix());

    UsdImagingGLRenderParams params;
    params.clearColor = GfVec4f(0.2f, 0.25f, 0.3f, 1.0f);
    params.enableLighting = false;
    params.showGuides = true;
    params.showProxy = true;
    params.showRender = true;
    params.drawMode = UsdImagingGLDrawMode::DRAW_SHADED_SMOOTH;

    int failures = 0;
    const double times[] = {1001, 1024};
    std::vector<Image> images;
    std::vector<std::string> names;
    for (double time : times) {
        params.frame = UsdTimeCode(time);
        for (int i = 0; i < 200; ++i) {
            engine.Render(stage->GetPseudoRoot(), params);
            if (engine.IsConverged())
                break;
        }
        Image image;
        if (!Capture(engine, image)) {
            std::printf("SKIP: color AOV unavailable\n");
            return kSkip;
        }
        if (IsBlank(image)) {
            std::printf("FAIL: frame %d rendered blank\n", int(time));
            ++failures;
        }
        images.push_back(image);
        names.push_back("twistRibbon_" + std::to_string(int(time)) + ".png");
    }
    if (images[0].rgba == images[1].rgba) {
        std::printf("FAIL: frames 1001 and 1024 rendered identically\n");
        ++failures;
    }

    const std::string directory = argv[2];
    const std::string configuration = CaptureConfiguration(engine);
    if (update) {
        for (size_t i = 0; i < images.size(); ++i) {
            if (!Write(directory + "/" + names[i], images[i])) {
                std::printf("FAIL: cannot write %s\n", names[i].c_str());
                ++failures;
            }
        }
        std::ofstream(directory + "/reference.txt") << configuration;
        std::printf(failures ? "FAILED\n" : "PASS (baselines updated)\n");
        return failures ? 1 : 0;
    }

    std::ifstream reference(directory + "/reference.txt");
    std::ostringstream referenceText;
    referenceText << reference.rdbuf();
    if (referenceText.str() != configuration) {
        std::printf("NOTE: pixel comparison skipped, not the baseline "
                    "configuration.\nbaseline:\n%s\nrunning:\n%s",
                    referenceText.str().c_str(), configuration.c_str());
        std::printf(failures ? "FAILED\n" : "PASS (renders only)\n");
        return failures ? 1 : 0;
    }

    for (size_t i = 0; i < images.size(); ++i) {
        const std::string baseline = directory + "/" + names[i];
        Image expected;
        if (!Read(baseline, expected) || expected.width != images[i].width ||
            expected.height != images[i].height) {
            std::printf("FAIL: baseline %s missing or wrong size\n",
                        baseline.c_str());
            ++failures;
            continue;
        }
        const int differing = CountDiffering(images[i], expected);
        if (differing > kPixelCountThreshold) {
            const std::string actual = "actual_" + names[i];
            Write(actual, images[i]);
            std::printf("FAIL: %s differs in %d pixels (limit %d); wrote %s\n",
                        names[i].c_str(), differing, kPixelCountThreshold,
                        actual.c_str());
            ++failures;
        }
    }
    std::printf(failures ? "FAILED\n" : "PASS\n");
    return failures ? 1 : 0;
}
