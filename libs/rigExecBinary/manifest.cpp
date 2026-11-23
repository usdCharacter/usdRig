#include "rigExecBinary/manifest.h"
#include "rigExecGraph/providerRecords.h"

#include <cstring>
#include <iterator>
#include <vector>

namespace rigExec {

namespace {

// Format 20 to 32 ordinals. Never edit: a new op appends to the current
// tables below, and these keep describing old files.
constexpr RigExecManifestId kRevisionOps32[] = {
    {"rig", "Matrix", 1},
    {"rig", "Skin", 1},
    {"rig", "BlendShape", 1},
    {"rig", "VolumeCorrect", 1},
    {"rig", "Smooth", 1},
    {"rig", "Lattice", 1},
    {"rig", "SurfaceProject", 1},
    {"rig", "Ribbon", 1},
    {"rig", "Wire", 1},
    {"rig", "EmitGuidePoints", 1},
    {"reserved", "Revision10", 1},
    {"reserved", "Revision11", 1},
    {"rig", "RecomputeNormals", 1},
    {"rig", "RecomputeExtent", 1},
    {"rig", "DeltaMush", 1},
    {"rig", "Wrinkle", 1},
    {"rig", "External", 1},
    {"rig", "SurfaceProjector", 1},
    {"rig", "ShaderDials", 1},
};
constexpr RigExecManifestId kPropertyOps32[] = {
    {"rig", "PropertyAdd", 1},    {"rig", "PropertyMultiply", 1},
    {"rig", "PropertyClamp", 1},  {"rig", "PropertyRemap", 1},
    {"rig", "PropertyBlend", 1},  {"rig", "PropertyCurve", 1},
    {"reserved", "Property6", 1}, {"rig", "PropertyQuantize", 1},
    {"rig", "PropertyPath", 1},
};
constexpr RigExecManifestId kProviderOps32[] = {
    {"rig", "ProviderAttribute", 1},    {"rig", "ProviderSpaceExpression", 1},
    {"rig", "ProviderRestFrame", 1},    {"rig", "ProviderDefaultSpace", 1},
    {"rig", "ProviderFrameToSpace", 1}, {"rig", "ProviderMatrixToFrame", 1},
    {"rig", "ProviderPosedFrame", 1},   {"rig", "ProviderJointMatrix", 1},
    {"rig", "ProviderLocalXform", 1},   {"rig", "ProviderInterveningXform", 1},
    {"rig", "ProviderAvarMatrix", 1},   {"rig", "ProviderRelativeXform", 1},
    {"rig", "ProviderAffineFrame", 1},
};
constexpr RigExecManifestId kValueTypes32[] = {
    {"std", "Double", 1},      {"std", "Float", 1},
    {"std", "Bool", 1},        {"std", "Int", 1},
    {"std", "Mat4d", 1},       {"wire", "Token", 1},
    {"std", "Vec3d", 1},       {"std", "Vec3f", 1},
    {"std", "IntArray", 1},    {"std", "FloatArray", 1},
    {"std", "DoubleArray", 1}, {"std", "Vec2fArray", 1},
    {"std", "Vec3fArray", 1},  {"std", "Vec3dArray", 1},
    {"std", "Mat4dArray", 1},  {"wire", "TokenArray", 1},
    {"std", "BoolArray", 1},   {"std", "Vec3i", 1},
};

static_assert(std::size(kRevisionOps32) == size_t(fb::RevisionOp::MAX) + 1);
static_assert(std::size(kPropertyOps32) == size_t(fb::PropertyOp::MAX) + 1);
static_assert(std::size(kProviderOps32) ==
              size_t(RigExecProviderOpKind::AffineFrame) + 1);
static_assert(std::size(kValueTypes32) == size_t(fb::InputTag::MAX) + 1);

template <size_t N>
constexpr RigExecManifestTable
Table(const RigExecManifestId (&ids)[N])
{
    return {ids, N};
}

const RigExecManifestTables kFormat32 = {
    Table(kRevisionOps32), Table(kPropertyOps32), Table(kProviderOps32),
    Table(kValueTypes32)};

// This build's ordinals still equal format 32's.
const RigExecManifestTables kCurrent = kFormat32;

bool
Fail(std::string *error, std::string message)
{
    if (error)
        *error = std::move(message);
    return false;
}

bool
Same(const RigExecManifestId &id, const fb::RigExecWireManifestEntry &entry)
{
    return entry.nameSpace == id.nameSpace && entry.name == id.name &&
           entry.version == id.version;
}

std::string
Text(const fb::RigExecWireManifestEntry &entry)
{
    return entry.nameSpace + ":" + entry.name + " v" +
           std::to_string(entry.version);
}

using Entries = std::vector<std::unique_ptr<fb::RigExecWireManifestEntry>>;

/// map[i]: this build's ordinal for the file's ordinal i.
bool
Map(const char *what, const Entries &entries,
    const RigExecManifestTable &current, std::vector<uint32_t> *map,
    std::string *error)
{
    map->assign(entries.size(), 0);
    std::vector<char> taken(current.count, 0);
    for (size_t i = 0; i < entries.size(); ++i) {
        if (!entries[i])
            return Fail(error, std::string("manifest ") + what + "[" +
                                   std::to_string(i) + "] is missing");
        size_t found = current.count;
        for (size_t c = 0; c < current.count; ++c) {
            if (Same(current.ids[c], *entries[i])) {
                found = c;
                break;
            }
        }
        if (found == current.count)
            return Fail(error, std::string("manifest ") + what + "[" +
                                   std::to_string(i) + "] " +
                                   Text(*entries[i]) +
                                   " is not known to this build");
        if (taken[found]++)
            return Fail(error, std::string("manifest ") + what + " lists " +
                                   Text(*entries[i]) + " twice");
        (*map)[i] = uint32_t(found);
    }
    return true;
}

bool
Identity(const std::vector<uint32_t> &map)
{
    for (size_t i = 0; i < map.size(); ++i) {
        if (map[i] != i)
            return false;
    }
    return true;
}

/// Rewrites \p field through \p map. An ordinal past the file's list maps
/// past this build's, so validation refuses it as an unknown op rather than
/// reading it as another.
template <class Field>
void
Remap(Field *field, const std::vector<uint32_t> &map, size_t currentCount)
{
    const size_t ordinal = size_t(*field);
    *field = ordinal < map.size()
                 ? Field(map[ordinal])
                 : Field(currentCount + (ordinal - map.size()));
}

} // namespace

const RigExecManifestTables &
RigExecManifestCurrent()
{
    return kCurrent;
}

const RigExecManifestTables &
RigExecManifestFormat32()
{
    return kFormat32;
}

std::unique_ptr<fb::RigExecWireManifest>
RigExecManifestBuild(const RigExecManifestTables &tables)
{
    auto manifest = std::make_unique<fb::RigExecWireManifest>();
    const auto fill = [](Entries *out, const RigExecManifestTable &table) {
        for (size_t i = 0; i < table.count; ++i) {
            auto entry = std::make_unique<fb::RigExecWireManifestEntry>();
            entry->nameSpace = table.ids[i].nameSpace;
            entry->name = table.ids[i].name;
            entry->version = table.ids[i].version;
            out->push_back(std::move(entry));
        }
    };
    fill(&manifest->revisionOps, tables.revisionOps);
    fill(&manifest->propertyOps, tables.propertyOps);
    fill(&manifest->providerOps, tables.providerOps);
    fill(&manifest->valueTypes, tables.valueTypes);
    return manifest;
}

bool
RigExecManifestResolve(fb::RigExecWireFile *file, std::string *error)
{
    if (file->formatVersion < RigExecFormatManifestVersion) {
        // The upgrade: old ordinals mean what format 32 says.
        file->manifest = RigExecManifestBuild(RigExecManifestFormat32());
    } else if (!file->manifest) {
        return Fail(error, "format " + std::to_string(file->formatVersion) +
                               " needs a manifest");
    }
    const fb::RigExecWireManifest &manifest = *file->manifest;
    const RigExecManifestTables &current = RigExecManifestCurrent();

    std::vector<uint32_t> types;
    if (!Map("value_types", manifest.valueTypes, current.valueTypes, &types,
             error))
        return false;
    if (!Identity(types))
        return Fail(error, "manifest value_types are not in this build's "
                           "order; rebake");

    std::vector<uint32_t> revisionOps, propertyOps, providerOps;
    if (!Map("revision_ops", manifest.revisionOps, current.revisionOps,
             &revisionOps, error) ||
        !Map("property_ops", manifest.propertyOps, current.propertyOps,
             &propertyOps, error) ||
        !Map("provider_ops", manifest.providerOps, current.providerOps,
             &providerOps, error))
        return false;

    if (file->geometry) {
        for (auto &chain : file->geometry->chains) {
            for (auto &revision : chain.revisions) {
                Remap(&revision.op, revisionOps, current.revisionOps.count);
            }
        }
    }
    for (auto &chain : file->propertyChains) {
        for (auto &revision : chain.revisions) {
            Remap(&revision.op, propertyOps, current.propertyOps.count);
        }
    }
    if (file->providerProgram) {
        for (auto &op : file->providerProgram->ops) {
            Remap(&op.kind, providerOps, current.providerOps.count);
        }
    }
    // The op graph's ids resolve against the reader's op registry, at Open.
    auto rebuilt = RigExecManifestBuild(current);
    rebuilt->coreOps = std::move(file->manifest->coreOps);
    rebuilt->coreTypes = std::move(file->manifest->coreTypes);
    file->manifest = std::move(rebuilt);
    return true;
}

bool
RigExecManifestValidate(const fb::RigExecWireFile &file, std::string *error)
{
    if (!file.manifest) {
        if (file.formatVersion >= RigExecFormatManifestVersion)
            return Fail(error, "manifest is missing");
        return true;
    }
    const RigExecManifestTables &current = RigExecManifestCurrent();
    const auto check = [&](const char *what, const Entries &entries,
                           const RigExecManifestTable &table) {
        if (entries.size() != table.count)
            return Fail(error, std::string("manifest ") + what + " holds " +
                                   std::to_string(entries.size()) +
                                   " entries; this build has " +
                                   std::to_string(table.count));
        for (size_t i = 0; i < table.count; ++i) {
            if (!entries[i] || !Same(table.ids[i], *entries[i]))
                return Fail(error, std::string("manifest ") + what + "[" +
                                       std::to_string(i) +
                                       "] is not this build's");
        }
        return true;
    };
    const fb::RigExecWireManifest &m = *file.manifest;
    return check("revision_ops", m.revisionOps, current.revisionOps) &&
           check("property_ops", m.propertyOps, current.propertyOps) &&
           check("provider_ops", m.providerOps, current.providerOps) &&
           check("value_types", m.valueTypes, current.valueTypes);
}

} // namespace rigExec
