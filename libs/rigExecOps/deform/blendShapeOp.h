#ifndef RIGEXEC_OPS_DEFORM_BLEND_SHAPE_OP_H
#define RIGEXEC_OPS_DEFORM_BLEND_SHAPE_OP_H

/// BlendShape: the channels' summed deltas added to the points, optionally
/// carried on the posed surface's frames first, then blended by the envelope.
/// Each channel's weight picks and lerps its in-between pair. The kernel is
/// rigExecMath/blendShapeKernel.h.

#include "rigExecMath/blendShapeKernel.h"
#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <memory>
#include <vector>

namespace rigExecOps {

using BlendShapeLayout = rigExec::RigExecBlendLayoutOf<rigExecStd::Vec3f>;

/// One in-between: either the target's moved points (dense, measured
/// against Base) or an offset layout.
struct BlendShapeSample {
    float activation = 1.0f;
    std::vector<rigExecStd::Vec3f> points;
    std::shared_ptr<const BlendShapeLayout> layout;

    bool operator==(const BlendShapeSample &o) const
    {
        if (activation != o.activation || points != o.points)
            return false;
        if (layout == o.layout)
            return true;
        return layout && o.layout && layout->offsets == o.layout->offsets &&
               layout->indices == o.layout->indices &&
               layout->pointCount == o.layout->pointCount &&
               layout->valid == o.layout->valid;
    }
};

struct BlendShapeChannel {
    /// Sorted by activation, each above zero and distinct.
    std::vector<BlendShapeSample> samples;

    bool operator==(const BlendShapeChannel &o) const
    {
        return samples == o.samples;
    }
};

/// What a blend shape is configured with at compile.
struct BlendShapePayload {
    std::vector<BlendShapeChannel> channels;
    /// Carry the deltas from Rest's surface frames to the incoming points'.
    bool surfaceFrame = false;

    bool operator==(const BlendShapePayload &o) const
    {
        return channels == o.channels && surfaceFrame == o.surfaceFrame;
    }
};

const rigExec::RigExecOpHandle &BlendShape();

// Points (Vec3fArray), Weights (FloatArray, one per channel), Base
// (optional Vec3fArray the dense targets are measured against; Points when
// unfed), Rest, Counts and Indices (optional; the surface frame's rest mesh),
// Envelope (optional FloatArray), Out.
const rigExec::RigExecPortHandle &BlendShapePoints();
const rigExec::RigExecPortHandle &BlendShapeWeights();
const rigExec::RigExecPortHandle &BlendShapeBase();
const rigExec::RigExecPortHandle &BlendShapeRest();
const rigExec::RigExecPortHandle &BlendShapeCounts();
const rigExec::RigExecPortHandle &BlendShapeIndices();
const rigExec::RigExecPortHandle &BlendShapeEnvelope();
const rigExec::RigExecPortHandle &BlendShapeOut();

} // namespace rigExecOps

#endif
