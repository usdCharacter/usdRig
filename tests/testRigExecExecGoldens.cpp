// The exec-row goldens: the rows the baked program's step bodies store,
// captured from the production evaluator and compared with the checked-in
// set. argv[1] = the rig stage, argv[2] = the golden directory, which holds
// one subdirectory per reference configuration. The configuration that
// recorded a golden compares every bit; any other compares the rows and
// value shapes. argv[3] = the golden's name, <name>.golden in each
// subdirectory. With RIGEXEC_RECORD_EXEC_ROWS=1 the rows at the frames
// argv[4...] are written to this configuration's subdirectory instead
// (tests/fixtures/README.md).
#include "rigExec/api/goldenPose.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/bakedExecCrossCheckRows.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

std::string
ReadFile(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

double
VisitTime(const std::string &line)
{
    const size_t at = line.find("time=");
    if (at == std::string::npos)
        return 0.0;
    const uint64_t bits = std::stoull(line.substr(at + 5, 16), nullptr, 16);
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

bool
IsHexDigit(char ch)
{
    return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
}

bool
IsWordChar(char ch)
{
    return std::isalnum(static_cast<unsigned char>(ch)) != 0;
}

// The golden with every floating-point bit pattern and input digest masked
// and the reference line dropped: which rows were judged, in which order,
// and the type and length of each value. A bit pattern is any run of 8 or 16
// lowercase hex digits between non-alphanumeric characters, so one embedded
// in a string value is masked the same way as a bare one.
std::string
Shape(const std::string &golden)
{
    std::istringstream lines(golden);
    std::string out;
    for (std::string line; std::getline(lines, line);) {
        if (line.compare(0, 9, "protocol ") == 0)
            continue;
        for (size_t i = 0; i < line.size();) {
            size_t end = i;
            while (end < line.size() && IsHexDigit(line[end]))
                ++end;
            const size_t length = end - i;
            const bool bounded = (i == 0 || !IsWordChar(line[i - 1])) &&
                                 (end == line.size() || !IsWordChar(line[end]));
            if (bounded && (length == 8 || length == 16)) {
                out += '#';
                i = end;
            } else if (length) {
                out.append(line, i, length);
                i = end;
            } else {
                out += line[i++];
            }
        }
        out += "\n";
    }
    return out;
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc < 4) {
        std::printf("usage: %s <stage> <golden directory> <name> [frame...]\n",
                    argv[0]);
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const std::string stagePath = argv[1], directory = argv[2], name = argv[3];
    const std::string reference = RigExecExecRowsReference();
    const bool record = TfGetenvBool("RIGEXEC_RECORD_EXEC_ROWS", false);
    if (record && argc < 5) {
        std::printf("FAIL: recording %s needs frames\n", name.c_str());
        return 1;
    }
    std::string golden = TfStringCatPaths(
        directory, TfStringCatPaths(reference, name + ".golden"));
    const bool exact = TfIsFile(golden);
    if (!exact && !record) {
        golden.clear();
        for (const auto &sub : TfListDir(directory))
            if (TfIsFile(TfStringCatPaths(sub, name + ".golden"))) {
                golden = TfStringCatPaths(sub, name + ".golden");
                break;
            }
    }
    if (golden.empty()) {
        std::printf("FAIL: no golden for %s under %s\n", name.c_str(),
                    directory.c_str());
        return 1;
    }
    const std::string expected = record ? std::string() : ReadFile(golden);
    std::vector<UsdTimeCode> times;
    if (record) {
        for (int i = 4; i < argc; ++i)
            times.emplace_back(std::stod(argv[i]));
    } else {
        std::istringstream lines(expected);
        for (std::string line; std::getline(lines, line);)
            if (line.compare(0, 6, "visit ") == 0)
                times.emplace_back(VisitTime(line));
    }
    const auto stage = UsdStage::Open(stagePath);
    if (!stage) {
        std::printf("FAIL: cannot open %s\n", stagePath.c_str());
        return 1;
    }
    SdfPath rigPath;
    for (const auto &prim : stage->Traverse())
        if (prim.GetTypeName() == SchemaTypeName::RigExecRoot) {
            rigPath = prim.GetPath();
            break;
        }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetSolverGuidesEnabled(true);
    std::vector<std::string> errors;
    if (rigPath.IsEmpty() || !evaluator.Compile(&errors) ||
        !evaluator.GetBakedProgram()) {
        std::printf("FAIL: %s does not compile to a baked program\n",
                    name.c_str());
        return 1;
    }
    std::string error;
    if (!RigExecBakedProgramTesting::EnableExecRowCapture(
            *evaluator.GetBakedProgram(), &error)) {
        std::printf("FAIL: row capture: %s\n", error.c_str());
        return 1;
    }
    const auto rows = RigExecBakedProgramTesting::ExecCrossCheckRows(
        *evaluator.GetBakedProgram());
    std::string actual = RigExecExecRowsGoldenHeader(
        TfGetBaseName(stagePath), rigPath,
        RigExecBakedProgramAccess::StepGraph(*evaluator.GetBakedProgram())
            .useSimd);
    size_t visits = 0;
    for (const UsdTimeCode time : times) {
        if (!evaluator.Evaluate(time).valid) {
            std::printf("FAIL: invalid pose at %g\n", time.GetValue());
            return 1;
        }
        actual += rows->GoldenVisit(visits++, time, rows->Judged(time));
    }
    if (!visits) {
        std::printf("FAIL: %s holds no visit\n", golden.c_str());
        return 1;
    }
    if (record) {
        TfMakeDirs(TfGetPathName(golden), -1, true);
        std::ofstream out(golden, std::ios::binary);
        out << actual;
        out.close();
        if (!out) {
            std::printf("FAIL: cannot write %s\n", golden.c_str());
            return 1;
        }
        std::printf("%s: recorded %zu visit(s) to %s\n", name.c_str(), visits,
                    golden.c_str());
        return 0;
    }
    if (exact) {
        if (!RigExecCompareGolden(expected, actual, &error)) {
            std::printf("FAIL: %s\n%s\n", golden.c_str(), error.c_str());
            return 1;
        }
    } else if (!RigExecCompareGolden(Shape(expected), Shape(actual), &error)) {
        std::printf("FAIL: shape of %s\n%s\n", golden.c_str(), error.c_str());
        return 1;
    }
    std::printf("%s: %zu visit(s), %s comparison against %s\n", name.c_str(),
                visits, exact ? "exact" : "shape", golden.c_str());
    return 0;
}
