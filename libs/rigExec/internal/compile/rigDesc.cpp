#include "rigExec/api/rigDesc.h"
#include "rigExec/internal/compile/rigDescData.h"

#include <utility>

namespace rigExec {

namespace {

std::string
OpName(const RigExecOpDesc &d)
{
    return std::string(d.nameSpace ? d.nameSpace : "") + ":" +
           (d.name ? d.name : "");
}

void
DestroyPayloads(std::vector<RigExecDescNode> &nodes)
{
    if (!RigExecRegistry::IsInitialized())
        return;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (RigExecDescNode &node : nodes) {
        if (!node.hasPayload)
            continue;
        const RigExecOpDesc &d = *registry.OpAt(node.op).desc;
        if (d.payload.destroy)
            d.payload.destroy(node.payload.Data());
        node.hasPayload = false;
    }
}

} // namespace

RigExecRigDescData::~RigExecRigDescData()
{
    DestroyPayloads(nodes);
}

RigExecRigDesc::RigExecRigDesc(_BuildKey,
                               std::unique_ptr<const RigExecRigDescData> data)
    : _data(std::move(data))
{
}

RigExecRigDesc::~RigExecRigDesc() = default;

RigExecRigDescBuilder::RigExecRigDescBuilder()
    : _data(std::make_unique<RigExecRigDescData>())
{
    if (!RigExecRegistry::IsInitialized()) {
        _Fail("the registry is not initialized");
        return;
    }
    _data->registryFingerprint = RigExecRegistry::Get().Fingerprint();
}

RigExecRigDescBuilder::~RigExecRigDescBuilder() = default;

void
RigExecRigDescBuilder::_Fail(std::string message)
{
    if (_error.empty())
        _error = std::move(message);
}

RigExecNodeId
RigExecRigDescBuilder::AddOp(const RigExecOpHandle &op, const void *payload)
{
    if (!_data || !_error.empty())
        return 0;
    if (!op.IsValid() || op.Index() >= RigExecRegistry::Get().OpCount()) {
        _Fail("AddOp: op handle is not registered");
        return 0;
    }
    const RigExecOpDesc &d = *RigExecRegistry::Get().Op(op).desc;
    if (d.payloadFieldCount) {
        if (!payload || !d.payloadValues) {
            _Fail("AddOp: " + OpName(d) + " needs its field values");
            return 0;
        }
        RigExecFieldValues values(d.payloadFieldCount);
        d.payloadValues(payload, values);
        return AddOp(op, values);
    }
    const RigExecNodeId id = RigExecNodeId(_data->nodes.size());
    RigExecDescNode &node = _data->nodes.emplace_back();
    node.op = op.Index();
    if (d.payload.size) {
        if (!payload) {
            _Fail("AddOp: " + OpName(d) + " needs a payload");
            return id;
        }
        node.payload.Allocate(d.payload.size);
        d.payload.copy(node.payload.Data(), payload);
        node.hasPayload = true;
    }
    return id;
}

RigExecNodeId
RigExecRigDescBuilder::AddOp(const RigExecOpHandle &op,
                             const RigExecFieldValues &values)
{
    if (!_data || !_error.empty())
        return 0;
    if (!op.IsValid() || op.Index() >= RigExecRegistry::Get().OpCount()) {
        _Fail("AddOp: op handle is not registered");
        return 0;
    }
    const RigExecOpDesc &d = *RigExecRegistry::Get().Op(op).desc;
    std::string why;
    if (!d.payloadFieldCount)
        why = "declares no payload fields";
    else
        RigExecCheckPayloadFields(d, values, &why);
    if (!why.empty()) {
        _Fail("AddOp: " + OpName(d) + ": " + why);
        return 0;
    }
    const RigExecNodeId id = RigExecNodeId(_data->nodes.size());
    RigExecDescNode &node = _data->nodes.emplace_back();
    node.op = op.Index();
    node.fields = values;
    return id;
}

namespace {

/// Port index of \p handle on \p node's op with direction \p flag, or
/// kRigExecInvalidIndex.
uint32_t
FindPort(const RigExecRigDescData &data, RigExecNodeId node,
         const RigExecPortHandle &handle, uint32_t flag)
{
    if (node >= data.nodes.size() || !handle.IsValid())
        return kRigExecInvalidIndex;
    const RigExecOpDesc &d =
        *RigExecRegistry::Get().OpAt(data.nodes[node].op).desc;
    const uint32_t index = handle.Index();
    if (index >= d.portCount || d.ports[index].port != &handle ||
        !(d.ports[index].flags & flag))
        return kRigExecInvalidIndex;
    return index;
}

/// The next element of an array input; 0 for any other input.
uint32_t
NextElement(RigExecRigDescData &data, RigExecNodeId node, uint32_t port)
{
    const RigExecOpDesc &d =
        *RigExecRegistry::Get().OpAt(data.nodes[node].op).desc;
    if (!(d.ports[port].flags & kRigExecPortArray))
        return 0;
    return data.listSizes[{node, port}]++;
}

} // namespace

void
RigExecRigDescBuilder::Connect(RigExecNodeId from, const RigExecPortHandle &out,
                               RigExecNodeId to, const RigExecPortHandle &in)
{
    if (!_data || !_error.empty())
        return;
    const uint32_t o = FindPort(*_data, from, out, kRigExecPortOutput);
    const uint32_t i = FindPort(*_data, to, in, kRigExecPortInput);
    if (o == kRigExecInvalidIndex || i == kRigExecInvalidIndex) {
        _Fail("Connect: node " + std::to_string(from) + " to node " +
              std::to_string(to) + " names a port its op does not have");
        return;
    }
    _data->connections.push_back(
        {{from, o}, {to, i}, NextElement(*_data, to, i)});
}

RigExecInputId
RigExecRigDescBuilder::BindInput(RigExecNodeId node,
                                 const RigExecPortHandle &in)
{
    if (!_data || !_error.empty())
        return {};
    const uint32_t i = FindPort(*_data, node, in, kRigExecPortInput);
    if (i == kRigExecInvalidIndex) {
        _Fail("BindInput: node " + std::to_string(node) + " has no such input");
        return {};
    }
    _data->inputs.push_back({{node, i}, NextElement(*_data, node, i)});
    return {uint32_t(_data->inputs.size() - 1)};
}

RigExecOutputId
RigExecRigDescBuilder::BindOutput(RigExecNodeId node,
                                  const RigExecPortHandle &out)
{
    if (!_data || !_error.empty())
        return {};
    const uint32_t o = FindPort(*_data, node, out, kRigExecPortOutput);
    if (o == kRigExecInvalidIndex) {
        _Fail("BindOutput: node " + std::to_string(node) +
              " has no such output");
        return {};
    }
    _data->outputs.push_back({node, o});
    return {uint32_t(_data->outputs.size() - 1)};
}

void
RigExecRigDescBuilder::Order(RigExecNodeId first, RigExecNodeId second)
{
    if (!_data || !_error.empty())
        return;
    if (first >= _data->nodes.size() || second >= _data->nodes.size() ||
        first == second) {
        _Fail("Order: invalid node pair");
        return;
    }
    _data->orders.emplace_back(first, second);
}

std::shared_ptr<const RigExecRigDesc>
RigExecRigDescBuilder::Build(std::string *error)
{
    if (!_data && _error.empty())
        _Fail("Build called twice");
    if (!_error.empty()) {
        if (error)
            *error = _error;
        return nullptr;
    }
    return std::make_shared<const RigExecRigDesc>(RigExecRigDesc::_BuildKey(0),
                                                  std::move(_data));
}

} // namespace rigExec
