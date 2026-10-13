// Lowers one point chain's revision packets to a RigExecGpuDeformFrame:
// the per-vertex program a GPU kernel runs in place of the chain's CPU
// kernels (rigExecMath/gpuDeformProgram.h).
//
// The packets are the ones the op graph assembled for the CPU kernels; the
// lowering adds no reads of its own. What an epoch fixes is lowered once
// into a shared layout and reused while every op's static key holds; each
// frame then copies only matrices, palettes, sample scales and control-point
// deltas.
#ifndef RIGEXEC_GPU_DEFORM_LOWERING_H
#define RIGEXEC_GPU_DEFORM_LOWERING_H

#include "rigExec/moverGraphTypes.h"
#include "rigExec/types.h"
#include "rigExecMath/gpuDeformProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// One revision of a chain, as the lowering reads it.
struct RigExecGpuRevisionInput {
    RigExecRevisionOp op = RigExecRevisionOp::Matrix;
    const RigExecMoverParameters *parameters = nullptr;
    /// Skin: the folded influence table, which the baked packet carries as
    /// identities. Null reads parameters->skinTransforms.
    const std::vector<GfMatrix4d> *influences = nullptr;
    /// Blend: the channels unsummed. Null for every other op.
    const std::vector<RigExecBlendChannel> *channels = nullptr;
    /// The status and influence checks passed: the revision applies this
    /// run. False passes its points through, as the fuse does.
    bool applies = true;
    /// The shared weight packet the parameters carry, by object and op
    /// value revision: equal pairs hold equal packets, so the steady state
    /// skips comparing the weights by value. -1 when not known.
    int weightsObject = -1;
    uint64_t weightsRevision = 0;
};

/// Whether \p op with packet \p p has a GPU program op at all. A chain is
/// lowered only when every revision answers true; anything else stays on
/// the CPU kernels.
bool RigExecGpuDeformSupports(RigExecRevisionOp op,
                              const RigExecMoverParameters &p,
                              const std::vector<RigExecBlendChannel> *channels,
                              std::string *why = nullptr);

/// Where a chain's points are deformed.
enum class RigExecGpuDeformMode {
    /// The CPU revision kernels, as always.
    Off,
    /// The program runs on the GPU and the points are read back at
    /// publication; everything downstream sees ordinary points.
    Readback,
    /// The program is published for a renderer to run (a Storm
    /// ExtComputation); the points stay on the GPU.
    Shader,
};

/// The mode RIGEXEC_GPU_DEFORM names ("readback", "shader"; anything else
/// is Off).
RigExecGpuDeformMode RigExecGpuDeformModeFromEnv();

/// Runs device jobs synchronously. False when the device failed; each job's
/// `ok` says how its chain fared.
using RigExecGpuDeformRunner = bool (*)(std::vector<RigExecGpuDeformJob> *jobs);

/// The runner a GPU library registers (rigExecGpu's
/// RigExecGpuRegisterDeformRunner). rigExec links no GPU library itself;
/// with no runner, Readback chains run their CPU kernels at publication.
void RigExecSetGpuDeformRunner(RigExecGpuDeformRunner runner);
RigExecGpuDeformRunner RigExecGetGpuDeformRunner();

/// Whether the blend sum would accept \p channels over \p pointCount
/// points (RigExecSumBlendChannels' structural checks), without summing.
bool RigExecBlendChannelsAreUsable(
    const std::vector<RigExecBlendChannel> &channels, size_t pointCount);

/// A chain published as its program (RigExecGpuDeformMode::Shader): what a
/// renderer needs to deform the entering points itself. Held in a pose's
/// moved properties in place of the points; copies share everything.
struct RigExecGpuDeformPublication {
    std::shared_ptr<const RigExecGpuDeformLayout> layout;
    std::shared_ptr<const std::vector<float>> frame;
    /// The entering points and the version that moves with their bytes.
    VtVec3fArray base;
    uint64_t baseVersion = 0;
    /// Moves whenever the frame's floats do.
    uint64_t frameVersion = 0;

    bool operator==(const RigExecGpuDeformPublication &o) const {
        return layout == o.layout && baseVersion == o.baseVersion &&
               frameVersion == o.frameVersion;
    }
    bool operator!=(const RigExecGpuDeformPublication &o) const {
        return !(*this == o);
    }
};

/// The points whose deformed positions stand in for the whole mesh when a
/// shader-mode chain's extent is measured without its points: per region
/// -- the points a skin op's dominant influence owns, or the whole mesh
/// with no skin -- the six base points extreme along x, y and z, plus every
/// 64th point. A mesh of 4096 points or fewer is sampled whole.
std::vector<int32_t> RigExecGpuDeformExtentSample(
    const RigExecGpuDeformLayout &layout, const GfVec3f *base, size_t count);

/// Samples [\p begin, \p end) run through \p frame on the CPU judge,
/// grown into \p min and \p max. False where a blend is degenerate.
/// Disjoint ranges of one sample may be measured side by side.
bool RigExecGpuDeformSampleBounds(const RigExecGpuDeformFrame &frame,
                                  const GfVec3f *base,
                                  const std::vector<int32_t> &sample,
                                  size_t begin, size_t end,
                                  GfVec3f *min, GfVec3f *max);

/// The sampled box grown by 2% of its diagonal: an approximation of the
/// deformed extent, not the extent. Points the sample misses can lie
/// outside it by less than the padding in practice.
void RigExecGpuDeformPadExtent(GfVec3f *min, GfVec3f *max);

/// Per-chain lowering state: the layout last built and the static keys it
/// was built from. Owned by the one body that lowers the chain.
class RigExecGpuDeformLowering {
public:
    RigExecGpuDeformLowering();
    ~RigExecGpuDeformLowering();
    RigExecGpuDeformLowering(const RigExecGpuDeformLowering &);
    RigExecGpuDeformLowering &operator=(const RigExecGpuDeformLowering &);

    /// Fills \p frame for \p revisions over \p pointCount points, rebuilding
    /// the layout only when a static key moved. Returns false, leaving
    /// \p frame untouched, when a revision is unsupported.
    bool Lower(const std::vector<RigExecGpuRevisionInput> &revisions,
               size_t pointCount, RigExecGpuDeformFrame *frame,
               std::string *why = nullptr);

    /// How many layouts this state has built, for tests.
    size_t LayoutBuilds() const { return _layoutBuilds; }

private:
    struct _Key;
    static bool _KeyHolds(const _Key &key, const RigExecGpuRevisionInput &in);
    void _FillFrame(const std::vector<RigExecGpuRevisionInput> &revisions,
                    RigExecGpuDeformFrame *frame) const;
    std::vector<_Key> _keys;
    std::shared_ptr<const RigExecGpuDeformLayout> _layout;
    size_t _pointCount = 0;
    size_t _layoutBuilds = 0;
};

}  // namespace rigExec

#endif  // RIGEXEC_GPU_DEFORM_LOWERING_H
