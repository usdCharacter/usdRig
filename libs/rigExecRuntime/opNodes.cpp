#include "rigExecRuntime/opNodes.h"

#include "rigExec/api/opFields.h"

#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/store.h"
#include "rigExecStdTypes/array.h"

#include <cstring>
#include <utility>

namespace rigExec {

namespace {

using rigExecStd::RigExecArray;

struct SlotType {
    RrOpSlotKind kind;
    const char *name;
};

/// The standard type each slot tag feeds, by registered name; looked up
/// once at Open. Token, TokenArray and Vec3i slots feed no op.
bool
SlotTypeOf(RigExecWireInputTag tag, SlotType *out)
{
    using T = RigExecWireInputTag;
    using K = RrOpSlotKind;
    switch (tag) {
    case T::Double:
        *out = {K::Double, "Double"};
        return true;
    case T::Float:
        *out = {K::Float, "Float"};
        return true;
    case T::Bool:
        *out = {K::Bool, "Bool"};
        return true;
    case T::Int:
        *out = {K::Int, "Int"};
        return true;
    case T::Matrix4d:
        *out = {K::Mat4d, "Mat4d"};
        return true;
    case T::Vec3d:
        *out = {K::Vec3d, "Vec3d"};
        return true;
    case T::Vec3f:
        *out = {K::Vec3f, "Vec3f"};
        return true;
    case T::IntArray:
        *out = {K::IntArray, "IntArray"};
        return true;
    case T::FloatArray:
        *out = {K::FloatArray, "FloatArray"};
        return true;
    case T::DoubleArray:
        *out = {K::DoubleArray, "DoubleArray"};
        return true;
    case T::Vec2fArray:
        *out = {K::Vec2fArray, "Vec2fArray"};
        return true;
    case T::Vec3fArray:
        *out = {K::Vec3fArray, "Vec3fArray"};
        return true;
    case T::Vec3dArray:
        *out = {K::Vec3dArray, "Vec3dArray"};
        return true;
    case T::Matrix4dArray:
        *out = {K::Mat4dArray, "Mat4dArray"};
        return true;
    case T::BoolArray:
        *out = {K::BoolArray, "BoolArray"};
        return true;
    default:
        return false;
    }
}

constexpr char kStdTypes[] = "std";

bool
Fail(std::string *error, std::string why)
{
    if (error)
        *error = "op nodes: " + std::move(why);
    return false;
}

std::string
Text(const fb::RigExecWireManifestEntry &entry)
{
    return entry.nameSpace + ":" + entry.name + " v" +
           std::to_string(entry.version);
}

uint32_t
FindOp(const RigExecRegistry &registry,
       const fb::RigExecWireManifestEntry &entry)
{
    for (size_t i = 0; i < registry.OpCount(); ++i) {
        const RigExecOpDesc &desc = *registry.OpAt(uint32_t(i)).desc;
        if (entry.nameSpace == desc.nameSpace && entry.name == desc.name &&
            entry.version == desc.version)
            return uint32_t(i);
    }
    return kRigExecInvalidIndex;
}

uint32_t
FindType(const RigExecRegistry &registry, const char *nameSpace,
         const char *name, uint32_t major)
{
    for (size_t i = 0; i < registry.TypeCount(); ++i) {
        const RigExecTypeDesc &desc = *registry.TypeAt(uint32_t(i)).desc;
        if (std::strcmp(nameSpace, desc.nameSpace) == 0 &&
            std::strcmp(name, desc.name) == 0 && major == desc.major)
            return uint32_t(i);
    }
    return kRigExecInvalidIndex;
}

/// A node's settings decoded into owned storage for AddOp, which copies
/// them.
class Payload {
public:
    explicit Payload(const RigExecOpPayloadHandling &h)
        : _h(h), _storage(h.size / sizeof(std::max_align_t) + 1)
    {
        _h.construct(_storage.data());
    }
    ~Payload()
    {
        _h.destroy(_storage.data());
    }
    Payload(const Payload &) = delete;
    Payload &operator=(const Payload &) = delete;
    void *Data()
    {
        return _storage.data();
    }

private:
    const RigExecOpPayloadHandling &_h;
    std::vector<std::max_align_t> _storage;
};

template <class T, class Wire = T>
bool
SetArray(RigExecEvaluator &evaluator, const RrProgram &program,
         const RrOpNodeInput &input, std::string *error)
{
    static_assert(sizeof(T) == sizeof(Wire), "array element layouts differ");
    const std::vector<Wire> *values = RrInputArray<Wire>(&program, input.slot);
    const RigExecArray<T> array =
        values ? RigExecArray<T>(reinterpret_cast<const T *>(values->data()),
                                 values->size())
               : RigExecArray<T>();
    return evaluator.SetInput(input.id, &array, error);
}

template <class T>
bool
SetValue(RigExecEvaluator &evaluator, const RrOpNodeInput &input,
         const T &value, std::string *error)
{
    return evaluator.SetInput(input.id, &value, error);
}

bool
SetInput(RigExecEvaluator &evaluator, const RrProgram &program,
         const RrOpNodeInput &input, std::string *error)
{
    using K = RrOpSlotKind;
    const RrInputValue value =
        RrValueFromWire(program.inputState.slotCurrent[input.slot]);
    switch (input.kind) {
    case K::Double:
        return SetValue(evaluator, input, value.f64, error);
    case K::Float:
        return SetValue(evaluator, input, value.f32, error);
    case K::Bool:
        return SetValue(evaluator, input, uint8_t(value.boolean ? 1 : 0),
                        error);
    case K::Int:
        return SetValue(evaluator, input, int32_t(value.i32), error);
    case K::Mat4d:
        return SetValue(evaluator, input, value.matrix, error);
    case K::Vec3d:
        return SetValue(evaluator, input, value.vec, error);
    case K::Vec3f:
        return SetValue(evaluator, input, value.vec3f, error);
    case K::IntArray:
        return SetArray<int32_t>(evaluator, program, input, error);
    case K::FloatArray:
        return SetArray<float>(evaluator, program, input, error);
    case K::DoubleArray:
        return SetArray<double>(evaluator, program, input, error);
    case K::Vec2fArray:
        return SetArray<RrVec2f>(evaluator, program, input, error);
    case K::Vec3fArray:
        return SetArray<RrVec3f>(evaluator, program, input, error);
    case K::Vec3dArray:
        return SetArray<RrVec3d, RigExecWireVec3d>(evaluator, program, input,
                                                   error);
    case K::Mat4dArray:
        return SetArray<RrMat4d, RigExecWireMatrix4d>(evaluator, program, input,
                                                      error);
    case K::BoolArray:
        return SetArray<uint8_t>(evaluator, program, input, error);
    }
    return false;
}

void
Release(RrOpNodeOutput &output)
{
    const RigExecTypeDesc &type =
        *RigExecRegistry::Get().TypeAt(output.type).desc;
    if (output.held && type.destroy)
        type.destroy(output.storage.data());
    output.held = false;
}

/// A type's copy assigns over a constructed value.
void
Hold(RrOpNodeOutput &output, const void *value)
{
    const RigExecTypeDesc &type =
        *RigExecRegistry::Get().TypeAt(output.type).desc;
    if (!output.held && type.construct)
        type.construct(output.storage.data());
    output.held = true;
    if (type.copy)
        type.copy(output.storage.data(), value);
    else
        std::memcpy(output.storage.data(), value, type.size);
}

} // namespace

RrOpNodes::~RrOpNodes()
{
    for (RrOpNodeOutput &output : outputs)
        Release(output);
}

bool
RrOpenOpNodes(RrProgram *p, std::string *error)
{
    const fb::RigExecWireFile *file = p->file.get();
    if (!file || !file->opGraph)
        return true;
    if (!RigExecRegistry::IsInitialized())
        return Fail(error, "the file holds op nodes; open it with the op "
                           "and type libraries");
    const RigExecRegistry &registry = RigExecRegistry::Get();
    const fb::RigExecWireOpGraph &graph = *file->opGraph;
    const fb::RigExecWireManifest &manifest = *file->manifest;

    std::vector<uint32_t> ops, types;
    for (const auto &entry : manifest.coreOps) {
        ops.push_back(FindOp(registry, *entry));
        if (ops.back() == kRigExecInvalidIndex)
            return Fail(error, "op " + Text(*entry) +
                                   " is in no op library this process "
                                   "registered");
    }
    for (const auto &entry : manifest.coreTypes) {
        types.push_back(FindType(registry, entry->nameSpace.c_str(),
                                 entry->name.c_str(), entry->version));
        if (types.back() == kRigExecInvalidIndex)
            return Fail(error, "type " + Text(*entry) +
                                   " is in no type library this process "
                                   "registered");
    }

    auto nodes = std::make_unique<RrOpNodes>();
    RigExecRigDescBuilder builder;
    const auto opOf = [&](size_t node) -> const RigExecRegisteredOp & {
        return registry.OpAt(ops[graph.nodes[node]->op]);
    };
    const auto port = [&](size_t node, uint32_t index, uint32_t flag,
                          const RigExecPortHandle **out) {
        const RigExecOpDesc &desc = *opOf(node).desc;
        if (index >= desc.portCount || !(desc.ports[index].flags & flag))
            return Fail(error,
                        "node " + std::to_string(node) + " (" + desc.name +
                            ") has no " +
                            (flag == kRigExecPortInput ? "input" : "output") +
                            " port " + std::to_string(index));
        *out = desc.ports[index].port;
        return true;
    };
    for (size_t n = 0; n < graph.nodes.size(); ++n) {
        const fb::RigExecWireOpNode &node = *graph.nodes[n];
        const RigExecRegisteredOp &op = opOf(n);
        const RigExecOpPayloadHandling &handling = op.desc->payload;
        const std::string where =
            "node " + std::to_string(n) + " (" + op.desc->name + ")";
        if (op.desc->payloadFieldCount) {
            RigExecFieldValues values;
            std::string why;
            if (!RigExecDecodePayloadFields(*op.desc, node.payload.data(),
                                            node.payload.size(), &values,
                                            &why) ||
                !RigExecCheckPayloadFields(*op.desc, values, &why))
                return Fail(error, where + ": settings do not decode: " + why);
            builder.AddOp(*op.handle, values);
            continue;
        }
        if (!handling.size) {
            if (!node.payload.empty())
                return Fail(error, where + " has settings its op does not "
                                           "take");
            builder.AddOp(*op.handle);
            continue;
        }
        if (!handling.decode)
            return Fail(error, where + ": its op cannot read saved settings");
        Payload payload(handling);
        std::string why;
        if (!handling.decode(node.payload.data(), node.payload.size(),
                             payload.Data(), &why))
            return Fail(error, where + ": settings do not decode: " + why);
        builder.AddOp(*op.handle, payload.Data());
    }
    for (size_t n = 0; n < graph.nodes.size(); ++n) {
        for (const auto &input : graph.nodes[n]->inputs) {
            const RigExecPortHandle *in = nullptr;
            if (!port(n, input->port, kRigExecPortInput, &in))
                return false;
            if (input->node >= 0) {
                const RigExecPortHandle *out = nullptr;
                if (!port(size_t(input->node), input->output,
                          kRigExecPortOutput, &out))
                    return false;
                builder.Connect(RigExecNodeId(input->node), *out,
                                RigExecNodeId(n), *in);
                continue;
            }
            const uint32_t slot = uint32_t(input->slot);
            SlotType slotType;
            const auto tag = file->inputs[slot].type();
            const uint32_t wanted = opOf(n).portTypes[input->port];
            if (!SlotTypeOf(tag, &slotType) ||
                FindType(registry, kStdTypes, slotType.name, 1) != wanted)
                return Fail(error, "slot " + std::to_string(slot) +
                                       " is not of the type input port " +
                                       std::to_string(input->port) +
                                       " of node " + std::to_string(n) +
                                       " takes");
            RrOpNodeInput bound;
            bound.id = builder.BindInput(RigExecNodeId(n), *in);
            bound.slot = slot;
            bound.kind = slotType.kind;
            nodes->inputs.push_back(bound);
        }
    }
    for (const auto &pair : graph.order)
        builder.Order(RigExecNodeId(pair.first), RigExecNodeId(pair.second));
    for (const auto &output : graph.outputs) {
        const RigExecPortHandle *out = nullptr;
        if (!port(output->node, output->port, kRigExecPortOutput, &out))
            return false;
        const uint32_t type = opOf(output->node).portTypes[output->port];
        if (types[output->type] != type)
            return Fail(error, "output " + output->name +
                                   " is not of its port's type");
        if (registry.TypeAt(type).desc->align > alignof(std::max_align_t))
            return Fail(error, "output " + output->name +
                                   " needs more alignment than it can get");
        RrOpNodeOutput published;
        published.name = output->name;
        published.id = builder.BindOutput(output->node, *out);
        published.type = type;
        published.storage.resize(
            registry.TypeAt(type).desc->size / sizeof(std::max_align_t) + 1);
        nodes->outputs.push_back(std::move(published));
    }
    std::string why;
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build(&why);
    if (desc)
        nodes->program = RigExecCompile(*desc, {}, &why);
    if (nodes->program)
        nodes->evaluator = RigExecEvaluator::Create(nodes->program, {}, &why);
    if (!nodes->evaluator)
        return Fail(error, why);
    // The evaluator belongs to the thread that opened the reader, so the
    // step runs there: Execute stays serial for this program.
    p->parallelSafe = false;
    p->opNodes = std::move(nodes);
    return true;
}

bool
RrRunOpNodes(RrProgram *p, std::string *error)
{
    RrOpNodes *nodes = p->opNodes.get();
    if (!nodes)
        return Fail(error, "the OpGraph step has no op nodes");
    RigExecEvaluator &evaluator = *nodes->evaluator;
    for (const RrOpNodeInput &input : nodes->inputs)
        if (!SetInput(evaluator, *p, input, error))
            return false;
    if (!evaluator.Evaluate(error))
        return false;
    for (RrOpNodeOutput &output : nodes->outputs) {
        const void *value = evaluator.GetOutput(output.id);
        if (!value)
            return Fail(error, "output " + output.name + " has no value");
        Hold(output, value);
    }
    return true;
}

namespace {

bool
Registered(const RigExecRegistry &registry, const char *library, bool op)
{
    const size_t count = op ? registry.OpCount() : registry.TypeCount();
    for (size_t i = 0; i < count; ++i) {
        const char *name = op ? registry.OpAt(uint32_t(i)).library->name
                              : registry.TypeAt(uint32_t(i)).library->name;
        if (std::strcmp(name, library) == 0)
            return true;
    }
    return false;
}

} // namespace

std::unique_ptr<RigExecRuntimeReader>
RigExecRuntimeReader::Open(const uint8_t *bytes, size_t size,
                           const RigExecMoverDefinition *movers,
                           size_t moverCount,
                           const RigExecRegistryInit &opLibraries,
                           std::string *error)
{
    const auto fail = [error](std::string why) {
        if (error)
            *error = std::move(why);
        return std::unique_ptr<RigExecRuntimeReader>();
    };
    if (!RigExecRegistry::IsInitialized()) {
        std::vector<std::string> errors;
        if (!RigExecRegistry::Initialize(opLibraries, &errors))
            return fail("op libraries: " + (errors.empty()
                                                ? std::string("refused")
                                                : errors.front()));
    } else {
        const RigExecRegistry &registry = RigExecRegistry::Get();
        for (const auto *library : opLibraries.typeLibraries)
            if (library && !Registered(registry, library->name, false))
                return fail(std::string("type library ") + library->name +
                            " is not in this process's registry");
        for (const auto *library : opLibraries.opLibraries)
            if (library && !Registered(registry, library->name, true))
                return fail(std::string("op library ") + library->name +
                            " is not in this process's registry");
    }
    return Open(bytes, size, movers, moverCount, error);
}

size_t
RigExecRuntimeReader::GetOpOutputCount() const
{
    return _program->opNodes ? _program->opNodes->outputs.size() : 0;
}

bool
RigExecRuntimeReader::GetOpOutput(size_t index,
                                  RigExecRuntimeOpOutput *out) const
{
    if (index >= GetOpOutputCount())
        return false;
    const RrOpNodeOutput &output = _program->opNodes->outputs[index];
    out->name = output.name;
    out->type = output.type;
    out->value = output.held ? output.storage.data() : nullptr;
    return true;
}

bool
RigExecRuntimeReader::FindOpOutput(const std::string &name, size_t *index) const
{
    for (size_t i = 0; i < GetOpOutputCount(); ++i)
        if (_program->opNodes->outputs[i].name == name) {
            *index = i;
            return true;
        }
    return false;
}

} // namespace rigExec
