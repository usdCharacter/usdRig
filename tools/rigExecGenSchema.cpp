// Describes the schema each registered op implies, from its ports, payload
// fields and authoring metadata.
//
// Usage: rigExecGenSchema [--json] [output-file]
//   Reads the op registry of the libraries this binary links
//   (rigexec_link_registrations) and takes no other input. Writes to
//   output-file, or to standard output without one.
//
// Text output: a header line, then one block per op sorted by
// "namespace:name" (no timestamps, deterministic). A block is
//     class "<ClassName>" (  # op <namespace>:<name> v<version> [<mode>]
//     {
//         <usdType> inputs:<port>   # <flags>
//         <usdType> outputs:<port>  # <flags>
//         uniform <usdType> payload:<field>  # <extent>
//         [uniform] <usdType> <schemaName> [= <fallback>]  # from <sources>
//     }
// ClassName is the op's RigExecSchemaInfo class, or the op name with its
// first letter upper-cased. A port or field with authoring metadata is
// written as the property it is authored with; payload fields that name one
// property are written once. Any other port's USD type is its registered
// value type mapped by name (see UsdTypeOf); an array-flagged input, or an
// array value type, is "T[]". A value type without a USD counterpart is
// written as "<registryType>" in angle brackets.
//
// --json writes every op's schema info and authored properties (type,
// uniform, fallback, allowed tokens, doc, binding) as one JSON document;
// tools/schema_report.py --diff compares it with schema.usda.
#include "rigExec/api/registry.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
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
ClassName(const RigExecOpDesc &desc)
{
    if (desc.schema && desc.schema->className && *desc.schema->className)
        return desc.schema->className;
    std::string out = desc.name;
    if (!out.empty())
        out[0] =
            static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
    return out;
}

const char *
ModeText(const RigExecSchemaInfo *schema)
{
    if (!schema)
        return "";
    switch (schema->mode) {
    case RigExecSchemaMode::Generated:
        return "generated";
    case RigExecSchemaMode::HandAuthored:
        return "handAuthored";
    case RigExecSchemaMode::Internal:
        return "internal";
    }
    return "";
}

const char *
BindingText(RigExecPropertyBinding binding)
{
    switch (binding) {
    case RigExecPropertyBinding::Value:
        return "value";
    case RigExecPropertyBinding::Connection:
        return "connection";
    case RigExecPropertyBinding::ConnectionList:
        return "connectionList";
    case RigExecPropertyBinding::Collection:
        return "collection";
    case RigExecPropertyBinding::Children:
        return "children";
    }
    return "";
}

/// \p usd with \p role applied: float3 with Point is point3f.
std::string
WithRole(const std::string &usd, RigExecPropertyRole role)
{
    const char *stem = nullptr;
    switch (role) {
    case RigExecPropertyRole::None:
        return usd;
    case RigExecPropertyRole::Point:
        stem = "point3";
        break;
    case RigExecPropertyRole::Normal:
        stem = "normal3";
        break;
    case RigExecPropertyRole::Vector:
        stem = "vector3";
        break;
    case RigExecPropertyRole::Color:
        stem = "color3";
        break;
    }
    const bool array =
        usd.size() > 2 && usd.compare(usd.size() - 2, 2, "[]") == 0;
    const std::string base = array ? usd.substr(0, usd.size() - 2) : usd;
    if (base != "float3" && base != "double3")
        return usd;
    return std::string(stem) + (base == "float3" ? "f" : "d") +
           (array ? "[]" : "");
}

/// The shortest decimal text that reads back as \p v at \p digits
/// significant digits at most (9 for float, 17 for double).
std::string
Number(double v, int digits, bool single)
{
    char buffer[40];
    for (int p = 1; p <= digits; ++p) {
        std::snprintf(buffer, sizeof(buffer), "%.*g", p, v);
        const double back = std::strtod(buffer, nullptr);
        if (single ? float(back) == float(v) : back == v)
            break;
    }
    return buffer;
}

std::string
ScalarText(RigExecColumnKind kind, const std::byte *data, size_t index,
           bool boolean)
{
    const std::byte *at = data + index * RigExecColumnKindSize(kind);
    switch (kind) {
    case RigExecColumnKind::UInt8: {
        uint8_t v = 0;
        std::memcpy(&v, at, 1);
        return boolean ? (v ? "true" : "false") : std::to_string(v);
    }
    case RigExecColumnKind::Int32: {
        int32_t v = 0;
        std::memcpy(&v, at, 4);
        return std::to_string(v);
    }
    case RigExecColumnKind::UInt32: {
        uint32_t v = 0;
        std::memcpy(&v, at, 4);
        return std::to_string(v);
    }
    case RigExecColumnKind::UInt64: {
        uint64_t v = 0;
        std::memcpy(&v, at, 8);
        return std::to_string(v);
    }
    case RigExecColumnKind::Float32: {
        float v = 0.0f;
        std::memcpy(&v, at, 4);
        return Number(v, 9, true);
    }
    case RigExecColumnKind::Float64: {
        double v = 0.0;
        std::memcpy(&v, at, 8);
        return Number(v, 17, false);
    }
    case RigExecColumnKind::None:
        break;
    }
    return "?";
}

/// One leaf value (no element or member types) as usda text: a scalar, a
/// tuple, or a matrix's rows.
bool
LeafText(const RigExecTypeDesc &type, const std::byte *data,
         const std::string &usd, std::string *out)
{
    const size_t n = type.columnCount;
    if (type.columnKind == RigExecColumnKind::None || !n)
        return false;
    const bool boolean = usd.rfind("bool", 0) == 0;
    if (n == 1) {
        *out = ScalarText(type.columnKind, data, 0, boolean);
        return true;
    }
    size_t rows = 0;
    if (usd.rfind("matrix", 0) == 0)
        rows = usd[6] - '0';
    std::string text = "(";
    if (rows && rows * rows == n) {
        for (size_t r = 0; r < rows; ++r) {
            text += r ? ", (" : " (";
            for (size_t c = 0; c < rows; ++c)
                text += (c ? ", " : "") +
                        ScalarText(type.columnKind, data, r * rows + c, false);
            text += ")";
        }
        *out = text + " )";
        return true;
    }
    for (size_t i = 0; i < n; ++i)
        text += (i ? ", " : "") + ScalarText(type.columnKind, data, i, boolean);
    *out = text + ")";
    return true;
}

/// A value of registered type \p typeIndex as usda text; false when the type
/// is not one this tool can print.
bool
ValueText(const RigExecRegistry &registry, uint32_t typeIndex,
          const void *value, const std::string &usd, std::string *out)
{
    const RigExecTypeDesc &type = *registry.TypeAt(typeIndex).desc;
    if (!type.element)
        return !type.fieldCount &&
               LeafText(type, static_cast<const std::byte *>(value), usd, out);
    if (!type.count || !type.data || !type.construct || !type.copy ||
        !type.destroy || type.align > alignof(std::max_align_t))
        return false;
    const RigExecTypeDesc &element =
        *registry.TypeAt(type.element->Index()).desc;
    const size_t words =
        (type.size + sizeof(std::max_align_t) - 1) / sizeof(std::max_align_t);
    auto storage = std::make_unique<std::max_align_t[]>(words);
    void *copy = storage.get();
    type.construct(copy);
    type.copy(copy, value);
    const size_t count = type.count(copy);
    const auto *items = static_cast<const std::byte *>(type.data(copy));
    const std::string elementUsd = usd.substr(0, usd.size() - 2);
    std::string text = "[";
    bool ok = true;
    for (size_t i = 0; ok && i < count; ++i) {
        std::string item;
        ok = !element.fieldCount &&
             LeafText(element, items + i * element.size, elementUsd, &item);
        text += (i ? ", " : "") + item;
    }
    type.destroy(copy);
    *out = text + "]";
    return ok;
}

/// One authored property, from one port or from the payload fields that
/// name it.
struct Property {
    std::string name;
    std::string type;
    bool uniform = false;
    bool hasFallback = false;
    std::string fallback;
    std::vector<std::string> tokens;
    const char *doc = nullptr;
    const char *binding = "value";
    std::vector<std::string> sources;
    std::vector<std::string> errors;
};

std::string
AuthoredType(const RigExecRegistry &registry, uint32_t typeIndex, bool listPort,
             const RigExecPropertyInfo &info)
{
    if (info.authoredType)
        return info.authoredType;
    if (info.binding == RigExecPropertyBinding::Connection ||
        info.binding == RigExecPropertyBinding::ConnectionList)
        return "rel";
    if (info.tokens)
        return "token";
    std::string usd = UsdTypeOf(registry, typeIndex);
    if (listPort && usd.back() != ']')
        usd += "[]";
    return WithRole(usd, info.role);
}

std::string
TokenOf(const RigExecRegistry &registry, uint32_t typeIndex,
        const RigExecPropertyInfo &info, bool *ok)
{
    std::string index;
    *ok = ValueText(registry, typeIndex, info.fallback, "uint", &index);
    const unsigned long long v = std::strtoull(index.c_str(), nullptr, 10);
    if (!*ok || v >= info.tokenCount || !info.tokens[v]) {
        *ok = false;
        return "";
    }
    return info.tokens[v];
}

void
Describe(const RigExecRegistry &registry, uint32_t typeIndex, bool listPort,
         const RigExecPropertyInfo &info, Property &p)
{
    p.uniform = info.uniform;
    p.doc = info.doc;
    p.binding = BindingText(info.binding);
    for (size_t t = 0; info.tokens && t < info.tokenCount; ++t)
        if (info.tokens[t])
            p.tokens.push_back(info.tokens[t]);
    if (!info.fallback)
        return;
    bool ok = false;
    std::string text;
    if (info.tokens)
        text = "\"" + TokenOf(registry, typeIndex, info, &ok) + "\"";
    else
        ok =
            ValueText(registry, typeIndex, info.fallback,
                      AuthoredType(registry, typeIndex, listPort, info), &text);
    if (!ok) {
        p.errors.push_back(p.name + ": fallback cannot be written");
        return;
    }
    p.hasFallback = true;
    p.fallback = text;
}

std::vector<Property>
AuthoredProperties(const RigExecRegistry &registry,
                   const RigExecRegisteredOp &op)
{
    const RigExecOpDesc &desc = *op.desc;
    std::vector<Property> out;
    for (size_t i = 0; i < desc.portCount; ++i) {
        const RigExecPortDesc &port = desc.ports[i];
        if (!port.authoring)
            continue;
        const bool list = port.flags & kRigExecPortArray;
        Property p;
        p.name = port.authoring->schemaName;
        p.type = AuthoredType(registry, op.portTypes[i], list, *port.authoring);
        p.sources.push_back(std::string("port ") + port.name);
        Describe(registry, op.portTypes[i], list, *port.authoring, p);
        out.push_back(std::move(p));
    }
    for (size_t i = 0; i < desc.payloadFieldCount; ++i) {
        const RigExecFieldDesc &field = desc.payloadFields[i];
        if (!field.authoring)
            continue;
        const std::string name = field.authoring->schemaName;
        auto same =
            std::find_if(out.begin(), out.end(),
                         [&](const Property &p) { return p.name == name; });
        Property one;
        one.name = name;
        Describe(registry, op.payloadFieldTypes[i], false, *field.authoring,
                 one);
        const std::string source = std::string("field ") + field.name;
        if (same == out.end()) {
            one.type = AuthoredType(registry, op.payloadFieldTypes[i], false,
                                    *field.authoring);
            one.sources.push_back(source);
            out.push_back(std::move(one));
            continue;
        }
        same->sources.push_back(source);
        same->errors.insert(same->errors.end(), one.errors.begin(),
                            one.errors.end());
        if (same->hasFallback && one.hasFallback) {
            const std::string head =
                same->fallback.front() == '('
                    ? same->fallback.substr(1, same->fallback.size() - 2)
                    : same->fallback;
            same->fallback = "(" + head + ", " + one.fallback + ")";
        } else if (same->hasFallback != one.hasFallback) {
            same->errors.push_back(name +
                                   ": components disagree on a fallback");
        }
    }
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
Joined(const std::vector<std::string> &items, const char *separator)
{
    std::string out;
    for (const std::string &item : items)
        out += (out.empty() ? "" : separator) + item;
    return out;
}

std::string
DescribeOp(const RigExecRegistry &registry, const RigExecRegisteredOp &op,
           const std::vector<Property> &authored)
{
    const RigExecOpDesc &desc = *op.desc;
    const std::string mode = ModeText(desc.schema);
    std::string out = "class \"" + ClassName(desc) + "\" (  # op " +
                      desc.nameSpace + ":" + desc.name + " v" +
                      std::to_string(desc.version) +
                      (mode.empty() ? "" : " " + mode) + "\n{\n";
    for (size_t i = 0; i < desc.portCount; ++i) {
        const RigExecPortDesc &port = desc.ports[i];
        if (port.authoring)
            continue;
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
        if (field.authoring)
            continue;
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
    for (const Property &p : authored) {
        std::string line =
            (p.uniform ? "uniform " : "") + p.type + " " + p.name;
        if (p.hasFallback)
            line += " = " + p.fallback;
        std::string comment = "from " + Joined(p.sources, ", ");
        if (!p.tokens.empty())
            comment += "; tokens " + Joined(p.tokens, " ");
        Append(out, line, comment);
    }
    out += "}\n";
    return out;
}

std::string
JsonText(const char *text)
{
    if (!text)
        return "null";
    std::string out = "\"";
    for (const char *c = text; *c; ++c) {
        switch (*c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(*c) < 0x20) {
                char buffer[8];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x",
                              static_cast<unsigned char>(*c));
                out += buffer;
            } else {
                out += *c;
            }
        }
    }
    return out + "\"";
}

std::string
JsonText(const std::string &text)
{
    return JsonText(text.c_str());
}

std::string
JsonList(const std::vector<std::string> &items)
{
    std::string out = "[";
    for (size_t i = 0; i < items.size(); ++i)
        out += (i ? ", " : "") + JsonText(items[i]);
    return out + "]";
}

std::string
JsonOp(const RigExecRegisteredOp &op, const std::vector<Property> &authored)
{
    const RigExecOpDesc &desc = *op.desc;
    std::string out = "    {\"op\": " +
                      JsonText(std::string(desc.nameSpace) + ":" + desc.name) +
                      ", \"version\": " + std::to_string(desc.version);
    if (desc.schema)
        out += ", \"schema\": {\"className\": " +
               JsonText(desc.schema->className) +
               ", \"doc\": " + JsonText(desc.schema->doc) +
               ", \"base\": " + JsonText(desc.schema->base) +
               ", \"mode\": " + JsonText(ModeText(desc.schema)) + "}";
    else
        out += ", \"schema\": null";
    out += ", \"properties\": [";
    for (size_t i = 0; i < authored.size(); ++i) {
        const Property &p = authored[i];
        out += std::string(i ? "," : "") +
               "\n      {\"name\": " + JsonText(p.name) +
               ", \"type\": " + JsonText(p.type) +
               ", \"uniform\": " + (p.uniform ? "true" : "false") +
               ", \"binding\": " + JsonText(p.binding) + ", \"fallback\": " +
               (p.hasFallback ? JsonText(p.fallback) : "null") +
               ", \"tokens\": " + JsonList(p.tokens) +
               ", \"doc\": " + JsonText(p.doc) +
               ", \"sources\": " + JsonList(p.sources) +
               ", \"errors\": " + JsonList(p.errors) + "}";
    }
    out += "]}";
    return out;
}

} // namespace

int
main(int argc, char **argv)
{
    bool json = false;
    const char *path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--json") == 0)
            json = true;
        else
            path = argv[i];
    }
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
        const std::vector<Property> authored = AuthoredProperties(registry, op);
        blocks.emplace_back(
            std::string(op.desc->nameSpace) + ":" + op.desc->name,
            json ? JsonOp(op, authored) : DescribeOp(registry, op, authored));
    }
    std::sort(blocks.begin(), blocks.end());

    std::string text;
    if (json) {
        text = "{\"rigExecGenSchema\": 2, \"ops\": [\n";
        for (size_t i = 0; i < blocks.size(); ++i)
            text += blocks[i].second + (i + 1 < blocks.size() ? ",\n" : "\n");
        text += "]}\n";
    } else {
        text =
            "#rigExecGenSchema 2: " + std::to_string(blocks.size()) + " ops\n";
        for (const auto &block : blocks)
            text += "\n" + block.second;
    }

    if (!path) {
        std::fwrite(text.data(), 1, text.size(), stdout);
        return 0;
    }
    std::ofstream file(path, std::ios::binary);
    file << text;
    if (!file) {
        std::fprintf(stderr, "cannot write %s\n", path);
        return 1;
    }
    return 0;
}
