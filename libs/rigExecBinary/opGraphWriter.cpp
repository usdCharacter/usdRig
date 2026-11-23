#include "rigExecBinary/opGraphWriter.h"

#include "rigExec/api/opFields.h"

#include <algorithm>
#include <set>
#include <utility>

namespace rigExec {

namespace {

constexpr char kOpGraphKey[] = "opGraph";

std::unique_ptr<fb::RigExecWireManifestEntry>
Entry(const char *nameSpace, const char *name, uint32_t version)
{
    auto entry = std::make_unique<fb::RigExecWireManifestEntry>();
    entry->nameSpace = nameSpace;
    entry->name = name;
    entry->version = version;
    return entry;
}

uint32_t
IndexOf(std::vector<uint32_t> *list, uint32_t value, bool *added)
{
    const auto at = std::find(list->begin(), list->end(), value);
    *added = at == list->end();
    if (*added)
        list->push_back(value);
    return uint32_t(std::find(list->begin(), list->end(), value) -
                    list->begin());
}

} // namespace

RigExecOpGraphWriter::RigExecOpGraphWriter(fb::RigExecWireFile &file)
    : _file(file), _graph(std::make_unique<fb::RigExecWireOpGraph>())
{
    if (!_file.manifest)
        _error = "the file has no manifest";
    else if (_file.opGraph)
        _error = "the file already holds op nodes";
    else if (!RigExecRegistry::IsInitialized())
        _error = "the op registry is not initialized";
}

RigExecOpGraphWriter::~RigExecOpGraphWriter() = default;

bool
RigExecOpGraphWriter::_Node(uint32_t node, const char *what)
{
    if (!_error.empty())
        return false;
    if (node >= _graph->nodes.size()) {
        _error = std::string(what) + " names node " + std::to_string(node) +
                 ", which was not added";
        return false;
    }
    return true;
}

uint32_t
RigExecOpGraphWriter::_Type(uint32_t type)
{
    bool added = false;
    const uint32_t index = IndexOf(&_coreTypes, type, &added);
    if (added) {
        const RigExecTypeDesc &desc = *RigExecRegistry::Get().TypeAt(type).desc;
        _file.manifest->coreTypes.push_back(
            Entry(desc.nameSpace, desc.name, desc.major));
    }
    return index;
}

uint32_t
RigExecOpGraphWriter::AddNode(const RigExecOpHandle &op, const void *payload)
{
    const uint32_t node = uint32_t(_graph->nodes.size());
    if (!_error.empty())
        return node;
    if (!op.IsValid()) {
        _error = "AddNode names an op that is not registered";
        return node;
    }
    const RigExecRegisteredOp &registered = RigExecRegistry::Get().Op(op);
    const RigExecOpDesc &desc = *registered.desc;
    auto wire = std::make_unique<fb::RigExecWireOpNode>();
    if (!RigExecEncodeOpPayload(desc, payload, wire->payload, &_error))
        return node;
    bool added = false;
    wire->op = IndexOf(&_coreOps, registered.index, &added);
    if (added)
        _file.manifest->coreOps.push_back(
            Entry(desc.nameSpace, desc.name, desc.version));
    _graph->nodes.push_back(std::move(wire));
    _ops.push_back(registered.index);
    return node;
}

void
RigExecOpGraphWriter::Connect(uint32_t from, const RigExecPortHandle &out,
                              uint32_t to, const RigExecPortHandle &in)
{
    if (!_Node(from, "Connect") || !_Node(to, "Connect"))
        return;
    auto input = std::make_unique<fb::RigExecWireOpNodeInput>();
    input->port = in.Index();
    input->node = int32_t(from);
    input->output = out.Index();
    _graph->nodes[to]->inputs.push_back(std::move(input));
}

void
RigExecOpGraphWriter::BindSlot(uint32_t node, const RigExecPortHandle &in,
                               uint32_t slot)
{
    if (!_Node(node, "BindSlot"))
        return;
    auto input = std::make_unique<fb::RigExecWireOpNodeInput>();
    input->port = in.Index();
    input->slot = int32_t(slot);
    _graph->nodes[node]->inputs.push_back(std::move(input));
}

void
RigExecOpGraphWriter::Order(uint32_t first, uint32_t second)
{
    if (_Node(first, "Order") && _Node(second, "Order"))
        _graph->order.emplace_back(int32_t(first), int32_t(second));
}

void
RigExecOpGraphWriter::Publish(uint32_t node, const RigExecPortHandle &out,
                              const std::string &name)
{
    if (!_Node(node, "Publish"))
        return;
    const RigExecRegisteredOp &op = RigExecRegistry::Get().OpAt(_ops[node]);
    if (out.Index() >= op.portTypes.size()) {
        _error = "Publish names a port node " + std::to_string(node) +
                 " does not have";
        return;
    }
    auto output = std::make_unique<fb::RigExecWireOpGraphOutput>();
    output->node = node;
    output->port = out.Index();
    output->type = _Type(op.portTypes[out.Index()]);
    output->name = name;
    _graph->outputs.push_back(std::move(output));
}

bool
RigExecOpGraphWriter::Finish(std::string *error)
{
    const auto fail = [&](std::string why) {
        if (error)
            *error = std::move(why);
        return false;
    };
    if (!_error.empty())
        return fail(_error);
    if (!_graph)
        return fail("Finish was already called");
    if (!_file.commonGraph || !_file.clustering || !_file.cones ||
        !_file.cones->always || !_file.cones->poseClusters)
        return fail("the file has no common graph, clustering or cones");
    for (const auto &op : _file.commonGraph->ops)
        if (op.key >= kOpGraphKey)
            return fail("common operation " + op.key +
                        " orders after the op nodes' step, which Finish "
                        "appends last");

    std::set<uint32_t> slots;
    for (const auto &node : _graph->nodes)
        for (const auto &input : node->inputs)
            if (input->slot >= 0)
                slots.insert(uint32_t(input->slot));

    const uint32_t index = uint32_t(_file.steps.size());
    auto &clustering = *_file.clustering;
    auto &common = *_file.commonGraph;
    const uint32_t cluster = uint32_t(clustering.clusters.size());

    // Like every ordinary step, it runs only once the stage frames are
    // admitted: it reads the admission's one source slot.
    const auto admission =
        uint32_t(fb::SlotDomain::RequiredStageFramesAdmission);
    uint64_t admissionValue = 0;
    while (admissionValue < common.valueSpecs.size() &&
           !(common.valueSpecs[admissionValue].domain == admission &&
             common.valueSpecs[admissionValue].slot == 0))
        ++admissionValue;
    if (admissionValue == common.valueSpecs.size()) {
        fb::RigExecWireCommonValueSpec spec;
        spec.domain = admission;
        spec.slot = 0;
        common.valueSpecs.push_back(std::move(spec));
    }
    if (std::find(common.leaves.begin(), common.leaves.end(), admissionValue) ==
        common.leaves.end())
        common.leaves.push_back(admissionValue);
    auto readers = std::find_if(
        common.readers.begin(), common.readers.end(),
        [&](const auto &row) { return row.value == admissionValue; });
    if (readers == common.readers.end()) {
        common.readers.emplace_back();
        readers = common.readers.end() - 1;
        readers->value = admissionValue;
    }
    readers->ops.push_back(index);

    fb::RigExecWireStep step;
    step.kind = fb::StepKind::OpGraph;
    step.reads.push_back(
        fb::SlotRange(fb::SlotDomain::RequiredStageFramesAdmission, 0, 1));
    step.object = 0;
    step.part = -1;
    step.headInputSlots.assign(slots.begin(), slots.end());
    step.cluster = int32_t(cluster);
    step.descriptorKey = kOpGraphKey;
    _file.steps.push_back(std::move(step));

    fb::RigExecWireCommonOp op;
    op.key = kOpGraphKey;
    op.kind = uint32_t(fb::StepKind::OpGraph);
    op.originalIndex = index;
    op.reads = {admissionValue};
    common.ops.push_back(std::move(op));
    common.canonicalIndex.push_back(int32_t(index));
    common.longestPath = std::max<uint64_t>(common.longestPath, 1);
    fb::RigExecWireCommonCluster commonCluster;
    commonCluster.members = {index};
    common.clusters.push_back(std::move(commonCluster));
    common.opClusters.push_back(cluster);

    fb::RigExecWireCluster wireCluster;
    wireCluster.members = {int32_t(index)};
    clustering.clusters.push_back(std::move(wireCluster));
    clustering.clusterOf.push_back(int32_t(cluster));
    const size_t clusters = clustering.clusters.size();
    for (auto *set :
         {_file.cones->always.get(), _file.cones->poseClusters.get()}) {
        set->clusters = uint32_t(clusters);
        set->words.resize((clusters + 63) / 64, 0);
    }

    _graph->step = int32_t(index);
    _file.opGraph = std::move(_graph);
    return true;
}

} // namespace rigExec
