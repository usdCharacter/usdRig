// The .rigexec manifest: a file's op ordinals are read through
// its manifest, and an id this build does not know is refused at Open.
#include "rigExecBinary/format.h"
#include "rigExecBinary/manifest.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

bool
Contains(const std::string &text, const char *part)
{
    return text.find(part) != std::string::npos;
}

std::vector<uint8_t>
Read(const char *path)
{
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), {}};
}

/// A file whose op fields are file ordinals, with a manifest that swaps
/// two entries of each op list relative to this build.
fb::RigExecWireFile
Permuted()
{
    fb::RigExecWireFile file;
    file.formatVersion = RigExecFormatVersion;
    file.manifest = RigExecManifestBuild(RigExecManifestCurrent());
    auto &m = *file.manifest;
    std::swap(m.revisionOps[0], m.revisionOps[1]);
    std::swap(m.propertyOps[0], m.propertyOps[2]);
    std::swap(m.providerOps[3], m.providerOps[12]);

    file.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    file.geometry->chains.resize(1);
    file.geometry->chains[0].revisions.resize(3);
    // File ordinals: 0 is Skin and 1 is Matrix in this file.
    file.geometry->chains[0].revisions[0].op = 0;
    file.geometry->chains[0].revisions[1].op = 1;
    file.geometry->chains[0].revisions[2].op =
        uint8_t(fb::RevisionOp::DeltaMush);
    file.propertyChains.resize(1);
    file.propertyChains[0].revisions.resize(2);
    file.propertyChains[0].revisions[0].op = fb::PropertyOp(0); // Clamp
    file.propertyChains[0].revisions[1].op = fb::PropertyOp(2); // Add
    file.providerProgram = std::make_unique<fb::RigExecWireProviderProgram>();
    file.providerProgram->ops.resize(2);
    file.providerProgram->ops[0].kind = 3;  // AffineFrame
    file.providerProgram->ops[1].kind = 12; // DefaultSpace
    return file;
}

void
TestTranslate()
{
    fb::RigExecWireFile file = Permuted();
    file.manifest->coreOps.push_back(
        std::make_unique<fb::RigExecWireManifestEntry>());
    file.manifest->coreOps[0]->nameSpace = "studio";
    file.manifest->coreOps[0]->name = "Twist";
    std::string error;
    CHECK(RigExecManifestResolve(&file, &error));
    // The op graph's ids resolve against the reader's registry, later.
    CHECK(file.manifest->coreOps.size() == 1 &&
          file.manifest->coreOps[0]->name == "Twist");
    const auto &revisions = file.geometry->chains[0].revisions;
    CHECK(revisions[0].op == uint8_t(fb::RevisionOp::Skin));
    CHECK(revisions[1].op == uint8_t(fb::RevisionOp::Matrix));
    CHECK(revisions[2].op == uint8_t(fb::RevisionOp::DeltaMush));
    CHECK(file.propertyChains[0].revisions[0].op == fb::PropertyOp::Clamp);
    CHECK(file.propertyChains[0].revisions[1].op == fb::PropertyOp::Add);
    CHECK(file.providerProgram->ops[0].kind == 12);
    CHECK(file.providerProgram->ops[1].kind == 3);
    // The manifest is now this build's.
    CHECK(RigExecManifestValidate(file, &error));
}

void
TestRefusals()
{
    std::string error;
    {
        fb::RigExecWireFile file = Permuted();
        file.manifest->revisionOps[4]->name = "Unheard";
        file.manifest->revisionOps[4]->version = 2;
        CHECK(!RigExecManifestResolve(&file, &error));
        CHECK(Contains(error, "rig:Unheard v2 is not known to this build"));
    }
    {
        fb::RigExecWireFile file = Permuted();
        file.manifest->propertyOps[3]->name = "PropertyMultiply";
        CHECK(!RigExecManifestResolve(&file, &error));
        CHECK(Contains(error, "lists rig:PropertyMultiply v1 twice"));
    }
    {
        fb::RigExecWireFile file = Permuted();
        std::swap(file.manifest->valueTypes[0], file.manifest->valueTypes[1]);
        CHECK(!RigExecManifestResolve(&file, &error));
        CHECK(Contains(error, "value_types are not in this build's order"));
    }
    {
        fb::RigExecWireFile file = Permuted();
        file.manifest->revisionOps.resize(1);
        file.manifest->revisionOps[0]->name = "Matrix";
        // Ordinal 1 is past this file's list: it must not read as Skin.
        CHECK(RigExecManifestResolve(&file, &error));
        CHECK(file.geometry->chains[0].revisions[1].op >
              uint8_t(fb::RevisionOp::MAX));
    }
    {
        fb::RigExecWireFile file = Permuted();
        file.manifest.reset();
        CHECK(!RigExecManifestResolve(&file, &error));
        CHECK(Contains(error, "needs a manifest"));
        CHECK(!RigExecManifestValidate(file, &error));
    }
}

// The checked-in current program opens with its manifest and writes its
// own bytes back.
void
TestOpen(const char *currentPath)
{
    const std::vector<uint8_t> current = Read(currentPath);
    std::unique_ptr<fb::RigExecWireFile> file;
    std::string error;
    CHECK(RigExecFormatOpen(current.data(), current.size(), &file, &error));
    if (!file) {
        std::printf("%s\n", error.c_str());
        return;
    }
    CHECK(file->manifest && RigExecManifestValidate(*file, &error));
    std::vector<uint8_t> rewritten;
    CHECK(RigExecFormatWrite(*file, &rewritten, &error));
    CHECK(rewritten == current);

    // A manifest whose ids this build lacks fails Open, naming the id.
    std::unique_ptr<fb::RigExecWireFile> opened;
    CHECK(RigExecFormatOpen(current.data(), current.size(), &opened, &error));
    if (!opened)
        return;
    opened->manifest->providerOps[0]->nameSpace = "studio";
    flatbuffers::FlatBufferBuilder builder;
    fb::FinishFileBuffer(builder, fb::File::Pack(builder, opened.get()));
    std::unique_ptr<fb::RigExecWireFile> refused;
    CHECK(!RigExecFormatOpen(builder.GetBufferPointer(), builder.GetSize(),
                             &refused, &error));
    CHECK(Contains(error, "studio:ProviderAttribute v1 is not known"));
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("usage: testRigExecManifest minimal.rigexec\n");
        return 2;
    }
    TestTranslate();
    TestRefusals();
    TestOpen(argv[1]);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
