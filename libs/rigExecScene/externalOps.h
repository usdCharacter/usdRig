#ifndef RIGEXEC_SCENE_EXTERNAL_OPS_H
#define RIGEXEC_SCENE_EXTERNAL_OPS_H

/// Host-side binding of External movers to ops: the mover's declared inputs
/// and provider values become the op's payload and inputs, and its envelope
/// the op's. The built-in movers' binders live here; an op plugin brings
/// its own (see RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY).

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

/// What one frame of an External mover reads: its declared inputs, in
/// declaration order, and the provider values.
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
    /// A windowed op's run status output and the status that means it
    /// solved, and the declared inputs its payload was built from.
    RigExecOutputId status;
    uint32_t solved = 0;
    bool windowed = false;
    std::vector<VtValue> source;
};

/// How one External mover runs as an op: the mover's schema type, the op,
/// and the functions that add its node, write one frame's inputs, and say
/// whether a node was built from settings the inputs no longer carry.
struct RigExecExternalOpBinder {
    uint32_t structSize = sizeof(RigExecExternalOpBinder);
    const char *schemaType = "";
    const RigExecOpHandle &(*op)() = nullptr;
    bool (*add)(const RigExecRevisionBinding &, const RigExecExternalOpValues &,
                RigExecRigDescBuilder &, RigExecExternalOpNode &,
                std::string *) = nullptr;
    bool (*write)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &, RigExecEvaluator &,
                  std::string *) = nullptr;
    bool (*stale)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &) = nullptr;
};

/// Version of RigExecExternalOpBinder and RigExecExternalOpBinderTable;
/// discovery refuses a table built with another.
inline constexpr uint32_t kRigExecExternalOpBinderAbiVersion = 1;

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
/// \p node's. A windowed node takes a full envelope where \p weights does
/// not resolve, so its tick records, and \p envelopeResolved says so.
bool RigExecWriteExternalOpInputs(const RigExecRevisionBinding &binding,
                                  const RigExecExternalOpValues &values,
                                  const RigExecWeightPacket &weights,
                                  const std::vector<GfVec3f> &points,
                                  const RigExecExternalOpNode &node,
                                  RigExecEvaluator *evaluator,
                                  std::string *error = nullptr,
                                  bool *envelopeResolved = nullptr);

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
