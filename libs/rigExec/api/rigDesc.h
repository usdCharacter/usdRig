#ifndef RIGEXEC_API_RIG_DESC_H
#define RIGEXEC_API_RIG_DESC_H

/// Describes a rig to compile: which ops it runs, how their outputs feed
/// each other's inputs, and which inputs and outputs the host sets and
/// reads. Build one with RigExecRigDescBuilder, then pass it to
/// RigExecCompile.

#include "rigExec/api/registry.h"

#include <cstdint>
#include <memory>
#include <string>

namespace rigExec {

/// One op instance in a rig description, numbered in the order it was
/// added.
using RigExecNodeId = uint32_t;

/// An input value the host sets on the evaluator (RigExecEvaluator::SetInput).
struct RigExecInputId {
    uint32_t index = kRigExecInvalidIndex;
};

/// An output value the host reads from the evaluator
/// (RigExecEvaluator::GetOutput).
struct RigExecOutputId {
    uint32_t index = kRigExecInvalidIndex;
};

struct RigExecRigDescData;

/// A finished rig description. It cannot be changed; compile it as often
/// as needed.
class RigExecRigDesc {
    friend class RigExecRigDescBuilder;
    struct _BuildKey {
        explicit _BuildKey(int)
        {
        }
    };

public:
    RigExecRigDesc(_BuildKey, std::unique_ptr<const RigExecRigDescData> data);
    ~RigExecRigDesc();
    RigExecRigDesc(const RigExecRigDesc &) = delete;
    RigExecRigDesc &operator=(const RigExecRigDesc &) = delete;

    const RigExecRigDescData &Data() const
    {
        return *_data;
    }

private:
    std::unique_ptr<const RigExecRigDescData> _data;
};

/// Builds a rig description one call at a time. After a call with a bad
/// argument, later calls do nothing and Build reports that first error.
/// The registry must be initialized first.
class RigExecRigDescBuilder {
public:
    RigExecRigDescBuilder();
    ~RigExecRigDescBuilder();
    RigExecRigDescBuilder(const RigExecRigDescBuilder &) = delete;
    RigExecRigDescBuilder &operator=(const RigExecRigDescBuilder &) = delete;

    /// Adds a node that runs \p op with the settings in \p payload, which
    /// is copied; pass null for an op that has no settings.
    RigExecNodeId AddOp(const RigExecOpHandle &op,
                        const void *payload = nullptr);
    /// Feeds input \p in of node \p to with output \p out of node \p from.
    /// An input that takes many values gets one more value per call.
    void Connect(RigExecNodeId from, const RigExecPortHandle &out,
                 RigExecNodeId to, const RigExecPortHandle &in);
    /// Makes input \p in of \p node a value the host sets.
    RigExecInputId BindInput(RigExecNodeId node, const RigExecPortHandle &in);
    /// Makes output \p out of \p node a value the host reads.
    RigExecOutputId BindOutput(RigExecNodeId node,
                               const RigExecPortHandle &out);
    /// Makes \p first run before \p second when no connection already
    /// does, for example when both touch the same outside resource.
    void Order(RigExecNodeId first, RigExecNodeId second);

    /// The finished description, or null with \p error naming the first
    /// mistake. Call it once.
    std::shared_ptr<const RigExecRigDesc> Build(std::string *error = nullptr);

private:
    void _Fail(std::string message);

    std::unique_ptr<RigExecRigDescData> _data;
    std::string _error;
};

} // namespace rigExec

#endif
