#ifndef RIGEXEC_API_RIG_DESC_H
#define RIGEXEC_API_RIG_DESC_H

/// The ordered op list a host submits for compile. USD-free.

#include "rigExec/api/registry.h"

#include <cstdint>
#include <memory>
#include <string>

namespace rigExec {

using RigExecNodeId = uint32_t;

/// A host-written value, bound to one op input.
struct RigExecInputId {
    uint32_t index = kRigExecInvalidIndex;
};

/// A host-read value, bound to one op output.
struct RigExecOutputId {
    uint32_t index = kRigExecInvalidIndex;
};

struct RigExecRigDescData;

/// Immutable result of RigExecRigDescBuilder::Build.
class RigExecRigDesc {
public:
    struct _BuildKey {
        explicit _BuildKey(int)
        {
        }
    };
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

/// Records nodes, connections, bindings and explicit order in submission
/// order. Errors are kept and reported by Build. Requires an initialized
/// registry.
class RigExecRigDescBuilder {
public:
    RigExecRigDescBuilder();
    ~RigExecRigDescBuilder();
    RigExecRigDescBuilder(const RigExecRigDescBuilder &) = delete;
    RigExecRigDescBuilder &operator=(const RigExecRigDescBuilder &) = delete;

    /// Adds a node running \p op; \p payload is copied with the op's
    /// payload handling and may be null for an op without one.
    RigExecNodeId AddOp(const RigExecOpHandle &op,
                        const void *payload = nullptr);
    /// Feeds input \p in of \p to from output \p out of \p from. On an
    /// array input each Connect or BindInput appends one element.
    void Connect(RigExecNodeId from, const RigExecPortHandle &out,
                 RigExecNodeId to, const RigExecPortHandle &in);
    /// Makes input \p in of \p node host-written.
    RigExecInputId BindInput(RigExecNodeId node, const RigExecPortHandle &in);
    /// Makes output \p out of \p node host-readable.
    RigExecOutputId BindOutput(RigExecNodeId node,
                               const RigExecPortHandle &out);
    /// Runs \p first before \p second with no value flowing between them.
    void Order(RigExecNodeId first, RigExecNodeId second);

    /// Null with \p error on the first recorded error. The builder is spent
    /// afterwards.
    std::shared_ptr<const RigExecRigDesc> Build(std::string *error = nullptr);

private:
    void _Fail(std::string message);

    std::unique_ptr<RigExecRigDescData> _data;
    std::string _error;
};

} // namespace rigExec

#endif
