#ifndef RIGEXEC_SCENE_EXTERNAL_OPS_H
#define RIGEXEC_SCENE_EXTERNAL_OPS_H

/// Host-side binding of External movers to ops. A binder adds the op node
/// of one External revision at Build and names, per op input, what feeds it
/// (RigExecExternalOpFeed): a static mapping the baked program and the
/// .rigexec carry, resolved each frame with no plugin code. The op computes
/// the full-strength candidate; the engine owns the mover's enabled flag
/// and envelope. An op plugin brings its binders through
/// RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/api/types.h"
#include "rigExecScene/geometryProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/value.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// What feeds one input of an External revision's op.
enum class RigExecExternalOpSource : uint8_t {
    /// The points entering the revision (Vec3fArray). The op runs whole:
    /// declare the port neighbourhood and the op ranged with a Whole plan.
    Points,
    /// The target's base points (Vec3fArray).
    BasePoints,
    /// Declared input `index` (RigExecMoverHandler::declareExternalInputs),
    /// as the port's standard type reads it.
    Input,
    /// The mover's transform provider (Mat4d).
    Transform,
    /// Influence `index` (Mat4d).
    Influence,
};

struct RigExecExternalOpFeed {
    const RigExecPortHandle *port = nullptr;
    RigExecExternalOpSource source = RigExecExternalOpSource::Points;
    uint32_t index = 0;
};

/// The node a binder added: its full-strength output and its feeds.
struct RigExecExternalOpPlan {
    /// The binder's op; RigExecBuildExternalOp fills it.
    const RigExecOpHandle *op = nullptr;
    RigExecNodeId node = kRigExecInvalidIndex;
    const RigExecPortHandle *out = nullptr;
    std::vector<RigExecExternalOpFeed> feeds;
};

/// How one External mover runs as an op: the mover's schema type, the op,
/// and build, which adds the op's node to the builder from compile facts
/// alone (the binding and its declared inputs' keys) and lists its feeds.
struct RigExecExternalOpBinder {
    uint32_t structSize = sizeof(RigExecExternalOpBinder);
    const char *schemaType = "";
    const RigExecOpHandle &(*op)() = nullptr;
    bool (*build)(const RigExecRevisionBinding &, RigExecRigDescBuilder &,
                  RigExecExternalOpPlan &, std::string *) = nullptr;
};

/// Version of RigExecExternalOpBinder and RigExecExternalOpBinderTable;
/// discovery refuses a table built with another.
inline constexpr uint32_t kRigExecExternalOpBinderAbiVersion = 2;

/// A library's binders.
struct RigExecExternalOpBinderTable {
    uint32_t abiVersion = kRigExecExternalOpBinderAbiVersion;
    const RigExecExternalOpBinder *binders = nullptr;
    size_t count = 0;
};

/// Adds binders to the ones the built-ins bring, once, before the first
/// lookup that needs them: op plugin discovery calls it. False with
/// \p error, adding none, when one names a schema type that already has a
/// binder, or is incomplete.
bool RigExecAddExternalOpBinders(
    const std::vector<const RigExecExternalOpBinder *> &binders,
    std::string *error);

/// Whether an op plugin's binder binds \p binding's mover, after op plugin
/// discovery.
bool RigExecBindsExternalOp(const RigExecRevisionBinding &binding);

/// Adds \p binding's op node through its plugin binder. False with
/// \p error when none binds it, or the binder refuses.
bool RigExecBuildExternalOp(const RigExecRevisionBinding &binding,
                            RigExecRigDescBuilder &builder,
                            RigExecExternalOpPlan *plan, std::string *error);

/// What one frame of an External mover reads, for a host that writes an
/// External op's inputs itself: its declared inputs, in declaration order,
/// and the provider values.
struct RigExecExternalOpValues {
    const std::vector<VtValue> *inputs = nullptr;
    const GfMatrix4d *transform = nullptr;
    const std::vector<GfMatrix4d> *influenceTransforms = nullptr;
    /// The points the mover measures against: the target's base points, or
    /// the entering points where it reads no base.
    const std::vector<GfVec3f> *basePoints = nullptr;
};

/// One External mover's op node and its host-written inputs.
struct RigExecExternalOpNode {
    RigExecNodeId node = 0;
    const RigExecOpHandle *op = nullptr;
    RigExecInputId points, envelope;
    /// The op's other inputs, in the order the binding writes them; an
    /// input that takes many values has one entry per value.
    std::vector<RigExecInputId> inputs;
    /// The payload the node was built with; a write whose payload differs
    /// refuses, and the node must be rebuilt.
    std::shared_ptr<const void> payload;
    RigExecOutputId out;
    /// A windowed op, and the declared inputs its payload was built from.
    bool windowed = false;
    std::vector<VtValue> source;
    /// Built by a plugin binder: `plan` names its feeds, `inputs` holds
    /// one input per feed, and `out` is the full-strength candidate, which
    /// the host blends.
    bool planned = false;
    RigExecExternalOpPlan plan;
};

/// The op \p binding's mover runs as, or null for a mover with none.
const RigExecOpHandle *
RigExecExternalOpHandle(const RigExecRevisionBinding &binding);

/// Adds \p binding's op, its payload and the number of values of each
/// many-valued input taken from \p values. False with \p error when the
/// mover has no op or \p values do not translate.
bool RigExecAddExternalOp(const RigExecRevisionBinding &binding,
                          const RigExecExternalOpValues &values,
                          RigExecRigDescBuilder *builder,
                          RigExecExternalOpNode *node,
                          std::string *error = nullptr);

/// Writes one frame: \p values, \p points and the envelope \p weights
/// resolves over them. False, with nothing written to \p evaluator past
/// the failure, where \p values do not translate, the envelope does not
/// resolve, or the payload or a many-valued input's count differs from
/// \p node's.
bool RigExecWriteExternalOpInputs(const RigExecRevisionBinding &binding,
                                  const RigExecExternalOpValues &values,
                                  const RigExecWeightPacket &weights,
                                  const std::vector<GfVec3f> &points,
                                  const RigExecExternalOpNode &node,
                                  RigExecEvaluator *evaluator,
                                  std::string *error = nullptr);

/// Whether \p values translate to another payload or input count than
/// \p node was built with: its write then refuses, and it must be rebuilt.
bool RigExecExternalOpNodeStale(const RigExecRevisionBinding &binding,
                                const RigExecExternalOpValues &values,
                                const RigExecExternalOpNode &node);

} // namespace rigExec

/// Defines the function through which an op plugin hands its binders to
/// discovery; its plugInfo names the symbol.
#define RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY(lib)                                 \
    extern "C" const ::rigExec::RigExecExternalOpBinderTable                   \
        *lib##_RigExecExternalOpBinders()

#endif
