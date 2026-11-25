// The pose input ops evaluated through the core: bound avar values, and what
// fails compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecOps;
using rigExecStd::RigExecArray;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

template <class T>
bool
SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

/// One op node with the given inputs bound, compiled and ready to run.
struct Harness {
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RigExecInputId> inputs;
    std::vector<RigExecOutputId> outputs;
    std::string error;

    Harness(const RigExecOpHandle &op, const void *payload,
            const std::vector<const RigExecPortHandle *> &ins,
            const std::vector<const RigExecPortHandle *> &outs)
    {
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(op, payload);
        for (const RigExecPortHandle *port : ins)
            inputs.push_back(b.BindInput(node, *port));
        for (const RigExecPortHandle *port : outs)
            outputs.push_back(b.BindOutput(node, *port));
        auto desc = b.Build(&error);
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    }

    bool Run(const std::vector<const void *> &values)
    {
        if (!evaluator)
            return false;
        for (size_t i = 0; i < values.size(); ++i)
            if (!evaluator->SetInput(inputs[i], values[i], &error))
                return false;
        return evaluator->Evaluate(&error);
    }

    template <class T> T Out(size_t i) const
    {
        const void *value =
            evaluator ? evaluator->GetOutput(outputs[i]) : nullptr;
        return value ? *static_cast<const T *>(value) : T{};
    }
};

RigExecArray<double>
Doubles(const std::vector<double> &values)
{
    RigExecArray<double> out(values.size());
    for (size_t i = 0; i < values.size(); ++i)
        out[i] = values[i];
    return out;
}

void
TestAvarInputs()
{
    AvarInputsPayload p;
    p.avarCount = 2;
    p.targets = {0, 1, 0};
    {
        Harness h(AvarInputs(), &p, {&AvarInputsValues()}, {&AvarInputsOut()});
        const auto values = Doubles({1.5, -2.0, 3.25});
        CHECK(h.Run({&values}));
        const auto out = h.Out<RigExecArray<double>>(0);
        CHECK(out.Size() == 2 && out[0] == 3.25 && out[1] == -2.0);
        const auto shortValues = Doubles({1.0, 2.0});
        CHECK(!h.Run({&shortValues}));
        CHECK(h.error.find("one value per binding") != std::string::npos);
    }
    {
        AvarInputsPayload past = p;
        past.targets = {0, 2};
        Harness h(AvarInputs(), &past, {&AvarInputsValues()},
                  {&AvarInputsOut()});
        CHECK(!h.evaluator);
        CHECK(h.error.find("past avarCount") != std::string::npos);
    }
    {
        AvarInputsPayload unbound = p;
        unbound.targets = {1, 1};
        Harness h(AvarInputs(), &unbound, {&AvarInputsValues()},
                  {&AvarInputsOut()});
        CHECK(!h.evaluator);
        CHECK(h.error.find("avar 0 has no binding") != std::string::npos);
    }
}

} // namespace

int
main()
{
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestAvarInputs();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
