// The generated module factory and runtime linked without the USD adapter.
#include "rigExecRuntime/runtime.h"
#include "testExternalMoverRegistration.h"
#include "testExternalMoverSchema.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

using namespace rigExec;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

const RigExecRuntimePoints &Body(const RigExecRuntimeReader &reader)
{
    for (const auto &points : reader.GetPoints())
        if (points.path == "/Rig/Body.points") return points;
    throw std::runtime_error("no body output");
}

int main(int argc, char **argv)
{
    try {
        CHECK(argc == 2);
        std::ifstream file(argv[1], std::ios::binary);
        CHECK(file.good());
        const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
        const auto definition = RigExecDefinetestExternalMover();
        CHECK(std::string(definition.schemaType) == "ExternalQuadraticMover");
        CHECK(testExternalMoverSchema::inputs_gain == 1.0f);
        CHECK(testExternalMoverSchema::Attributes.size() == 1);
        std::string error;
        auto reader = RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &definition, 1, &error);
        if (!reader) throw std::runtime_error(error);
        CHECK(reader->GetMissingExternalKernels().empty());
        CHECK(reader->Execute(&error));
        const auto retained = Body(*reader).points;
        const std::vector<RrVec3f> expected{{-2, 0, 1}, {1, 2, 3.25f}, {0, 0, 0}};
        CHECK(retained.Vector() == expected);
        CHECK(reader->Execute(&error));
        CHECK(Body(*reader).points.Share() == retained.Share());
        CHECK(reader->SetInput("/Rig/External.inputs:gain", 2.0, &error));
        CHECK(reader->Execute(&error));
        CHECK(Body(*reader).points[0][2] == 4.0f);
        CHECK(retained.Vector() == expected);

        const RigExecMoverDefinition duplicates[] = {definition, definition};
        CHECK(!reader->InstallMoverDefinitions(duplicates, 2, &error));
        CHECK(error.find("duplicate") != std::string::npos);
        CHECK(!RigExecRuntimeReader::Open(bytes.data(), bytes.size(), nullptr, 1, &error));
        auto invalid = definition;
        invalid.kernel.prepare = nullptr;
        CHECK(!reader->InstallMoverDefinitions(&invalid, 1, &error));
        invalid = definition;
        invalid.schemaType = "UnusedMover";
        CHECK(reader->InstallMoverDefinitions(&invalid, 1, &error));
        CHECK(reader->GetMissingExternalKernels().empty());
        CHECK(!definition.kernel.prepare(nullptr, 0, &error));
        CHECK(error.find("format") != std::string::npos);
        std::cout << "Generated mover: standalone playback, schema defaults, inputs and retained output passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
