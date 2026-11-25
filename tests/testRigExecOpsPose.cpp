// The pose input ops evaluated through the core: bound avar values, a space
// checkpoint's switch frame, a pose interpolator's weights, and what fails
// compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecRuntime/poseMath.h"
#include "rigExecRuntime/rbfSolver.h"
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

void
TestPoseInterpolator()
{
    PoseInterpolatorPayload maps;
    maps.solve = PoseSolve::Maps;
    maps.poseCount = 2;
    maps.mapsInputCount = 1;
    maps.mapRowInputs = {0, 0};
    maps.mapRowFrom = {0.0, 0.0};
    maps.mapRowTo = {1.0, 1.0};
    maps.mapRowSlope = {2.0, -1.0};
    maps.mapRowCut = {0.0, 3.0};
    maps.mapBegin = {0, 1, 2};
    maps.mapUnclamped = {0, 1};
    const std::vector<const RigExecPortHandle *> mapPorts = {
        &PoseInterpolatorEnabled(), &PoseInterpolatorNumeric()};
    {
        Harness h(PoseInterpolator(), &maps, mapPorts,
                  {&PoseInterpolatorOut()});
        const uint8_t on = 1, off = 0;
        const auto low = Doubles({0.25}), high = Doubles({0.75});
        CHECK(h.Run({&on, &low}));
        auto out = h.Out<RigExecArray<float>>(0);
        CHECK(out.Size() == 2 && out[0] == 0.5f && out[1] == 2.75f);
        CHECK(h.Run({&on, &high}));
        out = h.Out<RigExecArray<float>>(0);
        CHECK(out.Size() == 2 && out[0] == 1.0f && out[1] == 2.25f);
        CHECK(h.Run({&off, &high}));
        out = h.Out<RigExecArray<float>>(0);
        CHECK(out.Size() == 2 && out[0] == 0.0f && out[1] == 0.0f);
        const auto two = Doubles({0.25, 0.5});
        CHECK(!h.Run({&on, &two}));
        CHECK(h.error.find("per map input") != std::string::npos);
    }
    {
        PoseInterpolatorPayload extra = maps;
        extra.poseCount = 3;
        Harness h(PoseInterpolator(), &extra, mapPorts,
                  {&PoseInterpolatorOut()});
        CHECK(!h.evaluator);
        CHECK(h.error.find("do not fit its maps") != std::string::npos);
        Harness unfed(PoseInterpolator(), &maps, {&PoseInterpolatorEnabled()},
                      {&PoseInterpolatorOut()});
        CHECK(!unfed.evaluator);
        CHECK(unfed.error.find("per map input") != std::string::npos);
    }

    PoseInterpolatorPayload rbf;
    rbf.poseCount = 2;
    rbf.rbfPoses = {rigExecStd::Vec3d(0.0, 0.0, 0.0),
                    rigExecStd::Vec3d(1.5707963267948966, 0.0, 0.0)};
    rbf.rbfRadius = 1.0;
    rbf.rbfWeightRows = 2;
    rbf.rbfWeights = {1.0, 0.0, 0.0, 1.0};
    const std::vector<const RigExecPortHandle *> framePorts = {
        &PoseInterpolatorDriverFinal(), &PoseInterpolatorDriverRest()};
    Harness h(PoseInterpolator(), &rbf, framePorts, {&PoseInterpolatorOut()});
    rigExecRig::PointFrame rest, turned;
    turned.points[2] = rigExecStd::Vec3d(0.0, 0.6, 0.8);
    turned.points[3] = rigExecStd::Vec3d(0.0, -0.8, 0.6);
    CHECK(h.Run({&turned, &rest}));
    const auto out = h.Out<RigExecArray<float>>(0);
    RrRbfSolver solver({rbf.rbfPoses[0], rbf.rbfPoses[1]}, {}, {},
                       rbf.rbfTwistAxis, RrRbfKernel::Gaussian, 1.0, 0.0, true,
                       true, false);
    solver.SetSolvedTable({}, {}, {{1.0, 0.0}, {0.0, 1.0}});
    RigExecPoseInterpolatorRecordT<RrRbfSolver &> record{solver, false, true,
                                                         2};
    RigExecPoseInterpolatorInputsT<rigExecRig::PointFrame> input;
    input.driverFinal = &turned;
    input.driverRest = &rest;
    std::vector<double> weights;
    CHECK(RigExecRunPoseInterpolatorArithmetic<
              RrPoseInterpolatorMathT<rigExecRig::PointFrame>>(record, input,
                                                               &weights) ==
          RigExecPoseInterpolatorStatus::Success);
    CHECK(out.Size() == 2 && weights.size() == 2 &&
          out[0] == float(weights[0]) && out[1] == float(weights[1]));
    CHECK(out.Size() == 2 && out[1] > out[0]);
    rigExecRig::PointFrame broken;
    broken.flags = 0;
    CHECK(!h.Run({&broken, &rest}));
    CHECK(h.error.find("no usable frame") != std::string::npos);

    PoseInterpolatorPayload uneven = rbf;
    uneven.rbfWeightRows = 1;
    uneven.rbfWeights = {1.0, 0.0};
    Harness bad(PoseInterpolator(), &uneven, framePorts,
                {&PoseInterpolatorOut()});
    CHECK(!bad.evaluator);
    CHECK(bad.error.find("weight row") != std::string::npos);
    Harness lone(PoseInterpolator(), &rbf, {&PoseInterpolatorDriverFinal()},
                 {&PoseInterpolatorOut()});
    CHECK(!lone.evaluator);
    CHECK(lone.error.find("pairs") != std::string::npos);
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
    TestPoseInterpolator();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
