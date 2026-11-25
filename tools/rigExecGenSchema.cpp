// Describes, in a usda-like text, the schema each registered op implies.
//
// Usage: rigExecGenSchema [output-file]
//   Reads the op registry of the libraries this binary links
//   (rigexec_link_registrations) and takes no other input. Writes to
//   output-file, or to standard output without one.
//
// Output: a header line, then one block per op sorted by "namespace:name"
// (no timestamps, deterministic). A block is
//     class "<ClassName>" (  # op <namespace>:<name> v<version>
//     {
//         <usdType> inputs:<port>   # <flags>
//         <usdType> outputs:<port>  # <flags>
//         uniform <usdType> payload:<field>  # <extent>
//     }
// ClassName is the op name with its first letter upper-cased. A port's USD
// type is its registered value type mapped by name (see UsdTypeOf); an
// array-flagged input, or an array value type, is "T[]". A value type
// without a USD counterpart is written as "<registryType>" in angle
// brackets. tools/schema_report.py compares the output with schema.usda.
#include "rigExec/api/registry.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace rigExec;

namespace {

struct ScalarMap {
    const char *registryName;
    const char *usd;
};

const ScalarMap kScalars[] = {
    {"Bool", "bool"},      {"UInt8", "uchar"},    {"Int", "int"},
    {"UInt", "uint"},      {"UInt64", "uint64"},  {"Int64", "int64"},
    {"Float", "float"},    {"Double", "double"},  {"Vec2i", "int2"},
    {"Vec3i", "int3"},     {"Vec4i", "int4"},     {"Vec2f", "float2"},
    {"Vec3f", "float3"},   {"Vec4f", "float4"},   {"Vec2d", "double2"},
    {"Vec3d", "double3"},  {"Vec4d", "double4"},  {"Mat3f", "matrix3f"},
    {"Mat3d", "matrix3d"}, {"Mat4f", "matrix4f"}, {"Mat4d", "matrix4d"},
    {"Quatf", "quatf"},    {"Quatd", "quatd"},    {"Token", "token"},
    {"String", "string"},
};

std::string
UsdTypeOf(const RigExecRegistry &registry, uint32_t typeIndex)
{
    const RigExecRegisteredType &type = registry.TypeAt(typeIndex);
    if (type.desc->element)
        return UsdTypeOf(registry, type.desc->element->Index()) + "[]";
    for (const ScalarMap &entry : kScalars)
        if (std::string(type.desc->name) == entry.registryName)
            return entry.usd;
    return std::string("<") + type.desc->nameSpace + "::" + type.desc->name +
           ">";
}

std::string
ClassName(const char *name)
{
    std::string out = name;
    if (!out.empty())
        out[0] =
            static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

std::string
PortFlags(uint32_t flags)
{
    struct Flag {
        uint32_t bit;
        const char *text;
    };
    static const Flag kFlags[] = {
        {kRigExecPortOptional, "optional"},
        {kRigExecPortArray, "list"},
        {kRigExecPortPointwise, "pointwise"},
        {kRigExecPortNeighbourhood, "neighbourhood"},
        {kRigExecPortChecked, "checked"},
    };
    std::string out;
    for (const Flag &flag : kFlags)
        if (flags & flag.bit)
            out += (out.empty() ? "" : " ") + std::string(flag.text);
    return out;
}

const char *
BoundText(RigExecOutputBound bound)
{
    switch (bound) {
    case RigExecOutputBound::None:
        return "";
    case RigExecOutputBound::Fixed:
        return "bound=fixed";
    case RigExecOutputBound::Input:
        return "bound=input";
    case RigExecOutputBound::InputValue:
        return "bound=inputValue";
    case RigExecOutputBound::Field:
        return "bound=field";
    case RigExecOutputBound::Host:
        return "bound=host";
    }
    return "";
}

void
Append(std::string &out, const std::string &line, const std::string &comment)
{
    out += "    " + line;
    if (!comment.empty())
        out += "  # " + comment;
    out += "\n";
}

std::string
DescribeOp(const RigExecRegistry &registry, const RigExecRegisteredOp &op)
{
    const RigExecOpDesc &desc = *op.desc;
    std::string out = "class \"" + ClassName(desc.name) + "\" (  # op " +
                      desc.nameSpace + ":" + desc.name + " v" +
                      std::to_string(desc.version) + "\n{\n";
    for (size_t i = 0; i < desc.portCount; ++i) {
        const RigExecPortDesc &port = desc.ports[i];
        std::string type = UsdTypeOf(registry, op.portTypes[i]);
        if ((port.flags & kRigExecPortArray) && type.back() != ']')
            type += "[]";
        const bool input = port.flags & kRigExecPortInput;
        std::string comment = PortFlags(port.flags);
        const std::string bound = BoundText(port.bound);
        if (!bound.empty())
            comment += (comment.empty() ? "" : " ") + bound;
        Append(out, type + (input ? " inputs:" : " outputs:") + port.name,
               comment);
    }
    for (size_t i = 0; i < desc.payloadFieldCount; ++i) {
        const RigExecFieldDesc &field = desc.payloadFields[i];
        std::string type =
            field.type ? UsdTypeOf(registry, field.type->Index()) : "<none>";
        std::string comment = "scalar";
        switch (field.extent) {
        case RigExecFieldExtent::Scalar:
            break;
        case RigExecFieldExtent::Fixed:
            comment = "fixed " + std::to_string(field.count);
            type += "[]";
            break;
        case RigExecFieldExtent::FromField:
            comment = "fromField";
            type += "[]";
            break;
        case RigExecFieldExtent::FromInput:
            comment = "fromInput";
            type += "[]";
            break;
        }
        if (field.limit != UINT64_MAX)
            comment += " limit " + std::to_string(field.limit);
        Append(out, "uniform " + type + " payload:" + field.name, comment);
    }
    out += "}\n";
    return out;
}

} // namespace

int
main(int argc, char **argv)
{
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &error : errors)
            std::fprintf(stderr, "%s\n", error.c_str());
        return 1;
    }
    const RigExecRegistry &registry = RigExecRegistry::Get();

    std::vector<std::pair<std::string, std::string>> blocks;
    for (uint32_t i = 0; i < registry.OpCount(); ++i) {
        const RigExecRegisteredOp &op = registry.OpAt(i);
        blocks.emplace_back(std::string(op.desc->nameSpace) + ":" +
                                op.desc->name,
                            DescribeOp(registry, op));
    }
    std::sort(blocks.begin(), blocks.end());

    std::string text =
        "#rigExecGenSchema 1: " + std::to_string(blocks.size()) + " ops\n";
    for (const auto &block : blocks)
        text += "\n" + block.second;

    if (argc < 2) {
        std::fwrite(text.data(), 1, text.size(), stdout);
        return 0;
    }
    std::ofstream file(argv[1], std::ios::binary);
    file << text;
    if (!file) {
        std::fprintf(stderr, "cannot write %s\n", argv[1]);
        return 1;
    }
    return 0;
}
