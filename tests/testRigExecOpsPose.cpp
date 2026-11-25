// The pose input ops evaluated through the core: bound avar values, a space
// checkpoint's switch frame, and what fails compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecRuntime/poseMath.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecOps;
using rigExecStd::Mat4d;
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

Mat4d
Translate(double x, double y, double z)
{
    Mat4d m;
    m.SetIdentity();
    m._mtx[3][0] = x;
    m._mtx[3][1] = y;
    m._mtx[3][2] = z;
    return m;
}

template <class T>
RigExecArray<T>
ArrayOf(const std::vector<T> &values)
{
    RigExecArray<T> out(values.size());
    for (size_t i = 0; i < values.size(); ++i)
        out[i] = values[i];
    return out;
}

void
TestSpaceCheckpoint()
{
    using Math = RrPoseMathT<rigExecRig::PointFrame>;
    SpaceCheckpointPayload p;
    p.rotOrders = {SpaceCheckpointRotOrderCode("ZXY", 3), 0};
    p.rotationSigns = {0, 1};
    p.noScale = {0, 1};
    const std::vector<const RigExecPortHandle *> ins = {
        &SpaceCheckpointAvars(),
        &SpaceCheckpointPosedDefault(),
        &SpaceCheckpointParentDefaultInverse(),
        &SpaceCheckpointParentExpression(),
        &SpaceCheckpointParentExpressionAuthored(),
        &SpaceCheckpointPosedAuthored(),
        &SpaceCheckpointPosedAuthoredMatrix(),
        &SpaceCheckpointAnchor()};
    Harness h(SpaceCheckpoint(), &p, ins, {&SpaceCheckpointOut()});
    CHECK(h.evaluator != nullptr);
    // tx ty tz sx sy sz rx ry rz spin units, per slot.
    const std::vector<double> a = {1, 0, 0, 1,   1, 1, 0,  30, 0, 0, 1,
                                   0, 2, 0, 0.5, 1, 1, 15, 0,  0, 0, 1};
    const auto avars = ArrayOf(a);
    const auto identities =
        ArrayOf<Mat4d>({Math::Identity4(), Translate(0, 0, 3)});
    const auto parentM =
        ArrayOf<Mat4d>({Translate(5, 0, 0), Translate(0, 7, 0)});
    const auto parentAuthored = ArrayOf<uint8_t>({0, 1});
    const auto posedAuthored = ArrayOf<uint8_t>({0, 0});
    const auto posedAuthoredM =
        ArrayOf<Mat4d>({Translate(9, 9, 9), Translate(9, 9, 9)});
    const Mat4d anchor = Translate(0, 2, 0);
    const std::vector<const void *> values = {
        &avars,          &identities,    &identities,     &parentM,
        &parentAuthored, &posedAuthored, &posedAuthoredM, &anchor};
    CHECK(h.Run(values));
    std::vector<RigExecSpaceCheckpointInputT<Mat4d>> inputs(2);
    for (size_t s = 0; s < 2; ++s) {
        eval::PoseSlotComposeT<Math> view;
        view.avars11 = &a[s * 11];
        if (s == 0)
            view.rotOrder = "ZXY";
        view.rotationSign = p.rotationSigns[s];
        view.noScaleAvars = p.noScale[s] != 0;
        view.posedAuthoredM = &posedAuthoredM[s];
        view.posedD = &identities[s];
        view.parentDinv = &identities[s];
        view.parentSpaceM = &parentM[s];
        view.parentSpaceAuthored = parentAuthored[s] != 0;
        inputs[s] = eval::CheckpointInputOf<Math>(view);
    }
    const Mat4d expected =
        eval::SpaceCheckpointMatrix<Math>(inputs.data(), 2, anchor);
    CHECK(SameBits(h.Out<Mat4d>(0), expected));
    CHECK(std::abs(expected._mtx[3][1] - 9.0) < 1e-9);

    Mat4d collapsed = Math::Identity4();
    collapsed._mtx[0][0] = std::nan("");
    const auto degenerate = ArrayOf<Mat4d>({collapsed, collapsed});
    const auto authoredAll = ArrayOf<uint8_t>({1, 1});
    const std::vector<const void *> broken = {
        &avars,          &identities,  &identities, &parentM,
        &parentAuthored, &authoredAll, &degenerate, &anchor};
    CHECK(h.Run(broken));
    CHECK(std::isnan(h.Out<Mat4d>(0)._mtx[3][0]));

    const auto shortAvars = ArrayOf(std::vector<double>(11, 1.0));
    std::vector<const void *> mismatched = values;
    mismatched[0] = &shortAvars;
    CHECK(!h.Run(mismatched));
    CHECK(h.error.find("11 avars") != std::string::npos);

    SpaceCheckpointPayload uneven = p;
    uneven.noScale = {0};
    Harness bad(SpaceCheckpoint(), &uneven, ins, {&SpaceCheckpointOut()});
    CHECK(!bad.evaluator);
    CHECK(bad.error.find("per slot") != std::string::npos);
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
    TestSpaceCheckpoint();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
