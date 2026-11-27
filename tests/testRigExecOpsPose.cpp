// The pose input ops evaluated through the core: bound avar values, a space
// checkpoint's switch frame, a pose interpolator's weights, one slot's rest,
// ladder and compose, and what fails compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecOps/constraint/matrixMoverOps.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecRuntime/poseMath.h"
#include "rigExecRuntime/rbfSolver.h"
#include "rigExecRuntime/spaceSwitchMath.h"
#include "rigExecRigTypes/rigTypes.h"

#include "opAllocationCount.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
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
    p.noScale = {0, 1};
    const std::vector<const RigExecPortHandle *> ins = {
        &SpaceCheckpointAvars(),
        &SpaceCheckpointRotOrders(),
        &SpaceCheckpointRotationSigns(),
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
    const auto orders =
        ArrayOf<uint32_t>({SpaceCheckpointRotOrderCode("ZXY", 3), 0});
    const auto signs = ArrayOf<uint32_t>({0, 1});
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
        &avars,   &orders,         &signs,         &identities,     &identities,
        &parentM, &parentAuthored, &posedAuthored, &posedAuthoredM, &anchor};
    CHECK(h.Run(values));
    std::vector<RigExecSpaceCheckpointInputT<Mat4d>> inputs(2);
    for (size_t s = 0; s < 2; ++s) {
        eval::PoseSlotComposeT<Math> view;
        view.avars11 = &a[s * 11];
        if (s == 0)
            view.rotOrder = "ZXY";
        view.rotationSign = signs[s];
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
    Mat4d moving = anchor;
    std::string steadyError;
    CHECK(opAllocationCount::SteadyFrameAllocations(
              SpaceCheckpoint(), &p, ins, values, SpaceCheckpointOut(), 9,
              [&](int frame) {
                  moving = Translate(0, 2 + 0.5 * frame, 0);
                  return &moving;
              },
              &steadyError) == 0);

    Mat4d collapsed = Math::Identity4();
    collapsed._mtx[0][0] = std::nan("");
    const auto degenerate = ArrayOf<Mat4d>({collapsed, collapsed});
    const auto authoredAll = ArrayOf<uint8_t>({1, 1});
    const std::vector<const void *> broken = {
        &avars,   &orders,         &signs,       &identities, &identities,
        &parentM, &parentAuthored, &authoredAll, &degenerate, &anchor};
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
    CHECK(!bad.Run(values));
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

void
TestSlotCompose()
{
    using Math = RrPoseMathT<rigExecRig::PointFrame>;
    const Mat4d identity = Math::Identity4();
    SlotActivityPayload active, inactive;
    inactive.active = false;
    const std::vector<const RigExecPortHandle *> restIns = {
        &RestComposeRestAvars(), &RestComposeRestSpace()};
    const std::vector<const RigExecPortHandle *> restOuts = {
        &RestComposeRestMatrix(), &RestComposeRest(), &RestComposeRoundTrip()};
    const auto restAvars = ArrayOf<double>({1, 2, 3, 0, 0, 0});
    {
        Harness h(RestCompose(), &active, restIns, restOuts);
        CHECK(h.Run({&restAvars, &identity}));
        const Mat4d restM = h.Out<Mat4d>(0);
        CHECK(restM._mtx[3][0] == 1.0 && restM._mtx[3][2] == 3.0);
        CHECK(h.Out<rigExecRig::PointFrame>(1).flags != 0);
        const auto five = ArrayOf<double>({1, 2, 3, 0, 0});
        CHECK(!h.Run({&five, &identity}));
        CHECK(h.error.find("six rest avars") != std::string::npos);
        Harness off(RestCompose(), &inactive, restIns, restOuts);
        CHECK(off.Run({&restAvars, &identity}));
        CHECK(std::isnan(off.Out<Mat4d>(0)._mtx[3][0]));
        CHECK(off.Out<rigExecRig::PointFrame>(1).flags == 0);
    }

    const std::vector<const RigExecPortHandle *> ladderIns = {
        &LadderComposeSpaces(),          &LadderComposeAuthored(),
        &LadderComposeParentSpaceLeaf(), &LadderComposeRotationSign(),
        &LadderComposeDefaultAvars(),    &LadderComposeOwnRest()};
    const std::vector<const RigExecPortHandle *> ladderOuts = {
        &LadderComposeSelfDefault(), &LadderComposePosedDefault(),
        &LadderComposePosedAuthored(), &LadderComposeOutRotationSign()};
    const auto spaces = ArrayOf<Mat4d>(std::vector<Mat4d>(6, identity));
    const auto authored = ArrayOf<uint8_t>(std::vector<uint8_t>(6, 0));
    const auto defaultAvars = ArrayOf<double>({1, 0, 0, 0, 0, 0});
    const rigExecStd::Vec3d sign(1.0, -1.0, 1.0);
    {
        Harness h(LadderCompose(), &active, ladderIns, ladderOuts);
        CHECK(h.Run(
            {&spaces, &authored, &identity, &sign, &defaultAvars, &identity}));
        const Mat4d selfD = h.Out<Mat4d>(0);
        CHECK(selfD._mtx[3][0] == 1.0);
        CHECK(SameBits(h.Out<Mat4d>(1), selfD));
        CHECK(h.Out<uint8_t>(2) == 0);
        CHECK(h.Out<uint32_t>(3) == RigExecRotationSignMask(1.0, -1.0, 1.0));
        std::vector<const RigExecPortHandle *> half = ladderIns;
        half.push_back(&LadderComposeParentRest());
        Harness bad(LadderCompose(), &active, half, ladderOuts);
        CHECK(!bad.evaluator);
        CHECK(bad.error.find("fed together") != std::string::npos);
    }

    const std::vector<const RigExecPortHandle *> slotIns = {
        &ComposeSlotAvars(),         &ComposeSlotRotationSign(),
        &ComposeSlotPosedAuthored(), &ComposeSlotPosedAuthoredMatrix(),
        &ComposeSlotPosedDefault(),  &ComposeSlotParentDefaultInverse(),
        &ComposeSlotParentSpace(),   &ComposeSlotParentSpaceAuthored(),
        &ComposeSlotParentPosed()};
    const std::vector<const RigExecPortHandle *> slotOuts = {
        &ComposeSlotBase(), &ComposeSlotPosedMatrix()};
    const auto slotAvars = ArrayOf<double>({2, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1});
    const uint32_t noSign = 0;
    const uint8_t no = 0, yes = 1;
    const Mat4d authoredM = Translate(0, 5, 0),
                parentPosed = Translate(0, 0, 4);
    ComposeSlotPayload slot;
    Harness h(ComposeSlot(), &slot, slotIns, slotOuts);
    CHECK(h.Run({&slotAvars, &noSign, &no, &authoredM, &identity, &identity,
                 &identity, &no, &parentPosed}));
    auto base = h.Out<rigExecRig::PointFrame>(0);
    CHECK(base.points[0][0] == 2.0 && base.points[0][2] == 4.0);
    CHECK(h.Out<Mat4d>(1)._mtx[3][0] == 2.0);
    CHECK(h.Run({&slotAvars, &noSign, &yes, &authoredM, &identity, &identity,
                 &identity, &no, &parentPosed}));
    base = h.Out<rigExecRig::PointFrame>(0);
    CHECK(base.points[0][1] == 5.0 && base.points[0][2] == 0.0);
    ComposeSlotPayload off;
    off.active = false;
    Harness unavailable(ComposeSlot(), &off, slotIns, slotOuts);
    CHECK(unavailable.Run({&slotAvars, &noSign, &no, &authoredM, &identity,
                           &identity, &identity, &no, &parentPosed}));
    CHECK(unavailable.Out<rigExecRig::PointFrame>(0).flags == 0);
    CHECK(std::isnan(unavailable.Out<Mat4d>(1)._mtx[3][0]));
}

bool
SameFrame(const rigExecRig::PointFrame &a, const rigExecRig::PointFrame &b)
{
    return SameBits(a.points, b.points) && a.flags == b.flags;
}

rigExecRig::PointFrame
FrameOfMatrix(const Mat4d &m, Mat4d *space)
{
    using Math = RrPoseMathT<rigExecRig::PointFrame>;
    rigExecRig::PointFrame frame;
    unsigned flags = 0;
    eval::pose_ops::FrameFromMatrix<Math>(m, &frame.points, &flags);
    frame.flags = flags;
    *space = eval::pose_ops::SpaceOfFrame<Math>(frame.points, flags);
    return frame;
}

void
TestSwitchedSlot()
{
    using Math = RrPoseMathT<rigExecRig::PointFrame>;
    const Mat4d identity = Math::Identity4();
    const std::vector<const RigExecPortHandle *> ins = {
        &SwitchedComposeSlotAvars(),
        &SwitchedComposeSlotRotationSign(),
        &SwitchedComposeSlotPosedAuthored(),
        &SwitchedComposeSlotPosedAuthoredMatrix(),
        &SwitchedComposeSlotPosedDefault(),
        &SwitchedComposeSlotParentDefaultInverse(),
        &SwitchedComposeSlotParentPosed(),
        &SwitchedComposeSlotActive(),
        &SwitchedComposeSlotSourceDefaults(),
        &SwitchedComposeSlotSourcePosed()};
    const std::vector<const RigExecPortHandle *> outs = {
        &SwitchedComposeSlotBase(), &SwitchedComposeSlotPosedMatrix()};
    SwitchedComposeSlotPayload p;
    p.space.world = {1, 0};
    p.space.filters = {0, 1};
    p.space.twistAxis = rigExecStd::Vec3d(0.0, 1.0, 0.0);
    const auto avars = ArrayOf<double>({1, 0, 0, 1, 1, 1, 0, 30, 0, 0, 1});
    const uint32_t noSign = 0;
    const uint8_t no = 0, yes = 1;
    Mat4d turned = Translate(0, 2, 0);
    turned._mtx[0][0] = turned._mtx[2][2] = 0.0;
    turned._mtx[0][2] = 1.0;
    turned._mtx[2][0] = -1.0;
    const auto defaults = ArrayOf<Mat4d>({identity, Translate(0, 1, 0)});
    const auto posed = ArrayOf<Mat4d>({identity, turned});
    const Mat4d parentPosed = Translate(3, 0, 0);
    Harness h(SwitchedComposeSlot(), &p, ins, outs);
    for (const double active : {0.0, 0.25, 1.0, 7.0}) {
        CHECK(h.Run({&avars, &noSign, &no, &identity, &identity, &identity,
                     &parentPosed, &active, &defaults, &posed}));
        eval::PoseSlotComposeT<Math> view;
        view.avars11 = &avars[0];
        RigExecSpaceSwitchInputsT<Mat4d> in;
        in.avars = eval::ComposeSlotAvars<Math>(view);
        in.parentPosed = parentPosed;
        in.active = active;
        in.sources.resize(2);
        in.sources[0].world = true;
        in.sources[1].defaultSpace = defaults[1];
        in.sources[1].posedSpace = posed[1];
        RigExecSpaceSwitchRecordT<rigExecStd::Vec3d> record;
        record.filters = {0, 1};
        record.twistAxis = p.space.twistAxis;
        Mat4d matrix = identity, space = identity;
        CHECK(RigExecRunSpaceSwitchArithmetic<RrSpaceSwitchMath>(record, in,
                                                                 &matrix));
        const auto frame = FrameOfMatrix(matrix, &space);
        CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0), frame));
        CHECK(SameBits(h.Out<Mat4d>(1), space));
    }
    const double one = 1.0;
    const Mat4d authoredM = Translate(0, 5, 0);
    CHECK(h.Run({&avars, &noSign, &yes, &authoredM, &identity, &identity,
                 &parentPosed, &one, &defaults, &posed}));
    Mat4d authoredSpace = identity;
    CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0),
                    FrameOfMatrix(authoredM, &authoredSpace)));
    CHECK(SameBits(h.Out<Mat4d>(1), authoredSpace));

    SwitchedComposeSlotPayload off = p;
    off.active = false;
    Harness unavailable(SwitchedComposeSlot(), &off, ins, outs);
    CHECK(unavailable.Run({&avars, &noSign, &no, &identity, &identity,
                           &identity, &parentPosed, &one, &defaults, &posed}));
    CHECK(unavailable.Out<rigExecRig::PointFrame>(0).flags == 0);
    CHECK(std::isnan(unavailable.Out<Mat4d>(1)._mtx[3][0]));

    SwitchedComposeSlotPayload none = p;
    none.space.world.clear();
    none.space.filters.clear();
    Harness empty(SwitchedComposeSlot(), &none, ins, outs);
    CHECK(!empty.evaluator);
    CHECK(empty.error.find("at least one space") != std::string::npos);
    SwitchedComposeSlotPayload filters = p;
    filters.space.filters = {0};
    Harness unfiltered(SwitchedComposeSlot(), &filters, ins, outs);
    CHECK(!unfiltered.evaluator);
    CHECK(unfiltered.error.find("one filter per space") != std::string::npos);
    SwitchedComposeSlotPayload carry = p;
    carry.space.hasCarry = true;
    Harness uncarried(SwitchedComposeSlot(), &carry, ins, outs);
    CHECK(!uncarried.evaluator);
    CHECK(uncarried.error.find("carry space") != std::string::npos);
    Harness wrongCount(SwitchedComposeSlot(), &p, ins, outs);
    const auto three = ArrayOf<Mat4d>({identity, identity, identity});
    CHECK(!wrongCount.Run({&avars, &noSign, &no, &identity, &identity,
                           &identity, &parentPosed, &one, &three, &posed}));
}

void
TestShiftedSlot()
{
    using Math = RrPoseMathT<rigExecRig::PointFrame>;
    Harness h(ShiftedComposeSlot(), nullptr,
              {&ShiftedComposeSlotBase(), &ShiftedComposeSlotDelta(),
               &ShiftedComposeSlotShifted()},
              {&ShiftedComposeSlotOut(), &ShiftedComposeSlotPosedMatrix()});
    Mat4d baseSpace = Math::Identity4();
    const auto base = FrameOfMatrix(Translate(1, 2, 3), &baseSpace);
    const Mat4d shifted = Translate(1.5, 2, 3);
    const rigExecStd::Vec3d still(0.0, 0.0, 0.0), moved(0.5, 0.0, 0.0);
    CHECK(h.Run({&base, &still, &shifted}));
    CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0), base));
    CHECK(SameBits(h.Out<Mat4d>(1), baseSpace));
    CHECK(h.Run({&base, &moved, &shifted}));
    Mat4d shiftedSpace = Math::Identity4();
    CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0),
                    FrameOfMatrix(shifted, &shiftedSpace)));
    CHECK(SameBits(h.Out<Mat4d>(1), shiftedSpace));
}

/// The identity landmarks moved by (x, y, z), flagged valid.
rigExecRig::PointFrame
Landmarks(double x, double y, double z)
{
    rigExecRig::PointFrame f;
    f.points = {rigExecStd::Vec3d(x, y, z), rigExecStd::Vec3d(x + 1, y, z),
                rigExecStd::Vec3d(x, y + 1, z), rigExecStd::Vec3d(x, y, z + 1)};
    f.flags = rigExecRig::kPointFrameValid;
    return f;
}

void
TestMatrixMover()
{
    MatrixMoverPayload linear;
    const rigExecRig::PointFrame in = Landmarks(0, 0, 0);
    const rigExecRig::PointFrame source = Landmarks(2, 4, 6);
    const double half = 0.5;
    {
        Harness h(
            MatrixMover(), &linear,
            {&MatrixMoverIn(), &MatrixMoverTransform(), &MatrixMoverWeight()},
            {&MatrixMoverOut()});
        CHECK(h.Run({&in, &source, &half}));
        CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0), Landmarks(1, 2, 3)));
    }
    {
        // A rest that spans no frame maps the source to no matrix.
        Harness h(MatrixMover(), &linear,
                  {&MatrixMoverIn(), &MatrixMoverTransform(),
                   &MatrixMoverTransformRest(), &MatrixMoverWeight()},
                  {&MatrixMoverOut()});
        rigExecRig::PointFrame collapsed = in;
        collapsed.points = {in.points[0], in.points[0], in.points[0],
                            in.points[0]};
        CHECK(h.Run({&in, &source, &collapsed, &half}));
        rigExecRig::PointFrame passed = in;
        passed.flags |= rigExec::eval::pose_ops::kPoseFrameDegenerate;
        CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0), passed));
    }
    {
        // The transform measured in a space source that moved as far is
        // the identity; a rest not flagged valid reads as the identity.
        rigExecRig::PointFrame unflagged = Landmarks(9, 9, 9);
        unflagged.flags = 0;
        Harness h(MatrixMover(), &linear,
                  {&MatrixMoverIn(), &MatrixMoverTransform(),
                   &MatrixMoverTransformRest(), &MatrixMoverSpace(),
                   &MatrixMoverSpaceRest(), &MatrixMoverWeight()},
                  {&MatrixMoverOut()});
        const rigExecRig::PointFrame rest = Landmarks(1, 1, 1);
        const rigExecRig::PointFrame moved = Landmarks(3, 5, 7);
        CHECK(h.Run({&in, &moved, &rest, &source, &unflagged, &half}));
        CHECK(SameFrame(h.Out<rigExecRig::PointFrame>(0), in));
    }
    {
        Harness h(MatrixMover(), &linear,
                  {&MatrixMoverIn(), &MatrixMoverTransform(),
                   &MatrixMoverSpaceRest(), &MatrixMoverWeight()},
                  {&MatrixMoverOut()});
        CHECK(!h.evaluator);
        CHECK(h.error.find("a space rest needs Space") != std::string::npos);
    }
}

void
TestConstraintDelta()
{
    Harness h(ConstraintDelta(), nullptr,
              {&ConstraintDeltaSolved(), &ConstraintDeltaBase(),
               &ConstraintDeltaBaseOk(), &ConstraintDeltaWeight()},
              {&ConstraintDeltaDelta(), &ConstraintDeltaMeasured()});
    const rigExecRig::PointFrame solved = Landmarks(4, 0, 0);
    const Mat4d base = Translate(1, 0, 0);
    const uint8_t ok = 1, unread = 0;
    const double full = 1.0, none = 0.0;
    CHECK(h.Run({&solved, &base, &ok, &full}));
    CHECK(SameBits(h.Out<Mat4d>(0), Translate(3, 0, 0)));
    CHECK(h.Out<uint8_t>(1) == 1);
    CHECK(h.Run({&solved, &base, &unread, &none}));
    CHECK(SameBits(h.Out<Mat4d>(0), Translate(0, 0, 0)));
    CHECK(h.Out<uint8_t>(1) == 1);
    // An unread base, or a solved frame the constraint could not find,
    // measures nothing; a non-finite one is an error.
    CHECK(h.Run({&solved, &base, &unread, &full}));
    CHECK(SameBits(h.Out<Mat4d>(0), Translate(0, 0, 0)));
    CHECK(h.Out<uint8_t>(1) == 0);
    rigExecRig::PointFrame degenerate = solved;
    degenerate.flags |= rigExec::eval::pose_ops::kPoseFrameDegenerate;
    CHECK(h.Run({&degenerate, &base, &ok, &full}));
    CHECK(h.Out<uint8_t>(1) == 0);
    rigExecRig::PointFrame nonFinite = solved;
    nonFinite.points[0][0] = std::numeric_limits<double>::quiet_NaN();
    CHECK(!h.Run({&nonFinite, &base, &ok, &full}));
    CHECK(h.error.find("non-finite") != std::string::npos);
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
    TestSlotCompose();
    TestSwitchedSlot();
    TestShiftedSlot();
    TestMatrixMover();
    TestConstraintDelta();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
