#ifndef RIGEXEC_SCENE_GEOMETRY_OPS_H
#define RIGEXEC_SCENE_GEOMETRY_OPS_H

/// Host-side bridge from a geometry revision to rigExecOps' deform ops: the
/// packet's settings become the op's payload and its per-frame values its
/// inputs, so each RigExecMoverParameters field has a port or a payload field.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecScene/geometryProgram.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// Reused storage for the payload a write rebuilds from its packet to
/// compare with the node's.
struct RigExecGeometryPayloadScratch;
struct RigExecGeometryPayloadScratchDeleter {
    void operator()(RigExecGeometryPayloadScratch *scratch) const;
};
using RigExecGeometryPayloadScratchPtr =
    std::unique_ptr<RigExecGeometryPayloadScratch,
                    RigExecGeometryPayloadScratchDeleter>;
RigExecGeometryPayloadScratchPtr RigExecMakeGeometryPayloadScratch();

/// One deform op node and its host-written ports. Ports the op lacks keep
/// an invalid id.
struct RigExecGeometryOpNode {
    RigExecNodeId node = 0;
    RigExecRevisionOp op = RigExecRevisionOp::Matrix;
    /// Invalid when an upstream node feeds the points.
    RigExecInputId points;
    RigExecInputId envelope;
    RigExecInputId transform, strength, reference;
    RigExecInputId counts, indices, rest;
    RigExecInputId transforms, skinIndices, skinWeights;
    /// Lattice cages, a projection's surface, curve and frame drivers.
    RigExecInputId restCage, posedCage, surface, curve;
    RigExecInputId bindCoords, frames;
    /// A space pair and a metric: lattice or surface, from and to the target.
    RigExecInputId toSpace, fromSpace, metric;
    /// A wire's sparse field, in place of the envelope.
    RigExecInputId weightIndices, weightValues;
    /// A derived op's source: the final points, and an extent's widths.
    /// Its chained value is the authored normals or extent.
    RigExecInputId finalPoints, widths;
    /// A blend shape's channel weights and the base its dense targets are
    /// measured against.
    RigExecInputId channelWeights, base;
    /// The blend samples baked into the payload, by channel, to refuse a
    /// frame whose samples moved.
    struct BlendSample {
        float activation = 1.0f;
        std::vector<GfVec3f> points;
        std::shared_ptr<const RigExecBlendSampleLayout> layout;
    };
    std::vector<std::vector<BlendSample>> blendSamples;
    /// The payload the node was built with, compared with the packet's
    /// through the op's payload equality to refuse a frame whose settings
    /// moved; null for an op without one. A blend shape's channels are
    /// left out of it and kept in blendSamples.
    std::shared_ptr<const void> payload;
    RigExecOutputId out;
    /// The op's output port, for a downstream node to connect to.
    const RigExecPortHandle *outPort = nullptr;
};

/// The registered op \p op maps to, or null where none does (External).
const RigExecOpHandle *RigExecGeometryOpHandle(RigExecRevisionOp op);

/// True for the revision ops this bridge maps. A blend shape is added with
/// RigExecAddGeometryBlendOp.
bool RigExecGeometryOpSupported(RigExecRevisionOp op);

/// Adds the deform op for \p op to \p builder, its payload taken from
/// \p p. The points come from \p upstream's output when given, else from a
/// host input. False with \p error for an op the bridge does not map.
bool RigExecAddGeometryOp(RigExecRevisionOp op, const RigExecMoverParameters &p,
                          RigExecRigDescBuilder *builder,
                          const RigExecGeometryOpNode *upstream,
                          RigExecGeometryOpNode *node,
                          std::string *error = nullptr);

/// Writes \p p's per-frame values into \p evaluator for \p pointCount
/// points, and \p points when \p node takes them from the host. For a
/// derived op the points are the authored normals or extent. The
/// envelope is \p p's weights resolved at \p pointCount. False, writing
/// nothing more, where the packet's envelope does not resolve, or where a
/// setting baked into the payload differs from compile: the node must then
/// be rebuilt. \p scratch, when given, holds the payload rebuilt to compare.
bool RigExecWriteGeometryOpInputs(
    const RigExecMoverParameters &p, const RigExecGeometryOpNode &node,
    const std::vector<GfVec3f> *points, size_t pointCount,
    RigExecEvaluator *evaluator, std::string *error = nullptr,
    RigExecGeometryPayloadScratch *scratch = nullptr);

/// Adds a blend shape whose payload is \p channels' samples; \p p gives
/// its delta space, and the points chain as for RigExecAddGeometryOp.
bool RigExecAddGeometryBlendOp(const std::vector<RigExecBlendChannel> &channels,
                               const RigExecMoverParameters &p,
                               RigExecRigDescBuilder *builder,
                               const RigExecGeometryOpNode *upstream,
                               RigExecGeometryOpNode *node,
                               std::string *error = nullptr);

/// RigExecWriteGeometryOpInputs for a blend shape, plus \p channels'
/// weights and the \p base the targets are measured against. False where
/// a channel's samples differ from compile: the node must be rebuilt.
bool RigExecWriteGeometryBlendInputs(
    const std::vector<RigExecBlendChannel> &channels,
    const RigExecMoverParameters &p, const RigExecGeometryOpNode &node,
    const std::vector<GfVec3f> &base, const std::vector<GfVec3f> *points,
    size_t pointCount, RigExecEvaluator *evaluator,
    std::string *error = nullptr,
    RigExecGeometryPayloadScratch *scratch = nullptr);

/// A derived matrix op (surface projector or shader dials) and its
/// host-written ports.
struct RigExecGeometryMatrixNode {
    RigExecNodeId node = 0;
    RigExecRevisionOp op = RigExecRevisionOp::ShaderDials;
    RigExecInputId basePoints, points, counts, indices, dials, worldToMesh;
    /// Source, source space and rig space: base and final each, by frame
    /// slot. A slot is fed only when it resolved at compile.
    RigExecInputId frameBase[3], frameFinal[3];
    bool resolved[3] = {false, false, false};
    /// The ray settings baked into the payload.
    RigExecProjectorReads settings;
    RigExecOutputId out;
    const RigExecPortHandle *outPort = nullptr;
};

/// True for the derived matrix ops this bridge maps.
bool RigExecGeometryMatrixOpSupported(RigExecRevisionOp op);

/// Adds the matrix op for \p op: its payload from \p reads' ray settings,
/// its frame ports from which of \p frames named and resolved.
bool RigExecAddGeometryMatrixOp(RigExecRevisionOp op,
                                const RigExecSurfaceProjectorFrames &frames,
                                const RigExecProjectorReads &reads,
                                RigExecRigDescBuilder *builder,
                                RigExecGeometryMatrixNode *node,
                                std::string *error = nullptr);

/// Writes one frame's values. False, writing nothing more, where a frame
/// slot's resolution or the ray settings differ from compile: the node
/// must be rebuilt.
bool RigExecWriteGeometryMatrixInputs(
    const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const RigExecProjectorReads &reads, const std::vector<GfVec3f> &base,
    const std::vector<GfVec3f> &final, const RigExecGeometryMatrixNode &node,
    RigExecEvaluator *evaluator, std::string *error = nullptr);

/// Whether \p node was built from settings that \p p or \p channels no
/// longer carry: its write then refuses, and it must be rebuilt.
bool
RigExecGeometryOpNodeStale(RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           const std::vector<RigExecBlendChannel> *channels,
                           const RigExecGeometryOpNode &node,
                           RigExecGeometryPayloadScratch *scratch = nullptr);

/// RigExecGeometryOpNodeStale for a derived matrix node.
bool RigExecGeometryMatrixNodeStale(RigExecRevisionOp op,
                                    const RigExecSurfaceProjectorFrames &frames,
                                    const RigExecProjectorReads &reads,
                                    const RigExecGeometryMatrixNode &node);

} // namespace rigExec

#endif
