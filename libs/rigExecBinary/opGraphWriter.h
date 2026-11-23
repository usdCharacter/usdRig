#ifndef RIGEXEC_BINARY_OP_GRAPH_WRITER_H
#define RIGEXEC_BINARY_OP_GRAPH_WRITER_H

/// Adds op nodes to a .rigexec being written: each node's op, its saved
/// settings and what feeds its inputs, and the outputs a host reads by
/// name. Finish stores them as File.op_graph with the OpGraph step that
/// runs them, and names their ops and types in the manifest. The registry
/// must be initialized first.

#include "rigExec/api/registry.h"
#include "rigExecBinary/format.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecOpGraphWriter {
public:
    /// Borrows \p file, which must outlive the writer and hold a manifest.
    explicit RigExecOpGraphWriter(fb::RigExecWireFile &file);
    ~RigExecOpGraphWriter();
    RigExecOpGraphWriter(const RigExecOpGraphWriter &) = delete;
    RigExecOpGraphWriter &operator=(const RigExecOpGraphWriter &) = delete;

    /// Adds a node running \p op with the settings \p payload points to
    /// (null for an op without settings); returns its index.
    uint32_t AddNode(const RigExecOpHandle &op, const void *payload = nullptr);
    /// Feeds input \p in of node \p to with output \p out of node \p from.
    void Connect(uint32_t from, const RigExecPortHandle &out, uint32_t to,
                 const RigExecPortHandle &in);
    /// Feeds input \p in of \p node with the value of inputs[] slot \p slot.
    void BindSlot(uint32_t node, const RigExecPortHandle &in, uint32_t slot);
    /// Runs \p first before \p second.
    void Order(uint32_t first, uint32_t second);
    /// Publishes output \p out of \p node under \p name.
    void Publish(uint32_t node, const RigExecPortHandle &out,
                 const std::string &name);

    /// Stores the nodes in the file and appends the OpGraph step as the
    /// last step of its common graph. False with \p error naming the first
    /// mistake of any call, or a file the step cannot be appended to.
    bool Finish(std::string *error);

private:
    bool _Node(uint32_t node, const char *what);
    uint32_t _Type(uint32_t type);

    fb::RigExecWireFile &_file;
    std::unique_ptr<fb::RigExecWireOpGraph> _graph;
    /// Registry op index per node.
    std::vector<uint32_t> _ops;
    /// Registry op and type index per manifest entry.
    std::vector<uint32_t> _coreOps, _coreTypes;
    std::string _error;
};

} // namespace rigExec

#endif
