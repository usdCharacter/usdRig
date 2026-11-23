#ifndef RIGEXEC_BINARY_MANIFEST_H
#define RIGEXEC_BINARY_MANIFEST_H

/// The .rigexec manifest: the stable id each op and value-type ordinal in a
/// file stands for, and the translation from a file's ordinals to this
/// reader's. The one place outside core where wire ordinals meet names.

#include "rigExecBinary/format.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rigExec {

/// A stable id: (nameSpace, name, version).
struct RigExecManifestId {
    const char *nameSpace;
    const char *name;
    uint32_t version;
};

/// Ids by ordinal: ids[i] is what ordinal i stands for.
struct RigExecManifestTable {
    const RigExecManifestId *ids = nullptr;
    size_t count = 0;
};

struct RigExecManifestTables {
    RigExecManifestTable revisionOps;
    RigExecManifestTable propertyOps;
    RigExecManifestTable providerOps;
    RigExecManifestTable valueTypes;
};

/// The first format version that stores a manifest.
inline constexpr uint32_t RigExecFormatManifestVersion = 33;

/// This build's ordinals.
const RigExecManifestTables &RigExecManifestCurrent();

/// What the ordinals of format 20 to 32 files mean. Frozen: it never
/// changes when this build's ordinals do.
const RigExecManifestTables &RigExecManifestFormat32();

/// A manifest holding \p tables, as a writer stores it.
std::unique_ptr<fb::RigExecWireManifest>
RigExecManifestBuild(const RigExecManifestTables &tables);

/// Open's step between unpacking and validating. A file older than format
/// 33 gets the format-32 manifest; a newer one must carry its own. Each op
/// field is then rewritten from the file's ordinal to this build's, and the
/// manifest replaced by this build's, keeping the file's core_ops and
/// core_types, which the runtime resolves against its op registry. Fails naming an id this build does
/// not know or an id listed twice, or on value types out of this build's
/// order. An op field past its manifest list is left for validation to
/// refuse.
bool RigExecManifestResolve(fb::RigExecWireFile *file, std::string *error);

/// Validation's rule: a file of format 33 or later carries a manifest, and
/// a manifest's revision, property, provider and value-type lists, when
/// present, are this build's.
bool RigExecManifestValidate(const fb::RigExecWireFile &file,
                             std::string *error);

} // namespace rigExec

#endif
