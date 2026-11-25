// The provider matrix, frame record and provider refresh ops evaluated
// through the core: the kernels' answers on hand-built frames, the
// fallbacks and record gates, and what fails compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/providerMatrixKernel.h"
#include "rigExecOps/space/spaceOps.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecOps;
using rigExecRig::PointFrame;
using rigExecStd::Mat3d;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;

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

PointFrame
Moved(double x, double y, double z, uint32_t flags = 1)
{
    PointFrame frame;
    for (auto &point : frame.points)
        point = Vec3d(point[0] + x, point[1] + y, point[2] + z);
    frame.flags = flags;
    return frame;
}

bool
SameMatrix(const Mat4d &a, const Mat4d &b)
{
    return SameBits(a, b);
}

void
TestProviderMatrix()
{
    const PointFrame rest = Moved(0, 0, 0);
    const PointFrame unusableRest = Moved(0, 0, 0, 1 | 2);
    const PointFrame posed = Moved(1, 0, 0);
    const PointFrame base = Moved(0, 2, 0);
    const PointFrame invalid = Moved(1, 0, 0, 0);
    for (const bool joint : {false, true}) {
        ProviderMatrixPayload payload;
        payload.joint = joint;
        Harness h(ProviderMatrix(), &payload,
                  {&ProviderMatrixRest(), &ProviderMatrixFinal(),
                   &ProviderMatrixBase()},
                  {&ProviderMatrixOut()});
        CHECK(h.Run({&rest, &posed, &base}));
        CHECK(SameMatrix(h.Out<Mat4d>(0), Translate(1, 0, 0)));
        CHECK(h.Run({&rest, &invalid, &base}));
        CHECK(SameMatrix(h.Out<Mat4d>(0), Translate(0, 0, 0)));
        // An unusable rest falls back to rest -> base, a joint then to
        // base -> final.
        CHECK(h.Run({&unusableRest, &posed, &base}));
        CHECK(SameMatrix(h.Out<Mat4d>(0),
                         joint ? Translate(1, 0, 0) : Translate(0, 2, 0)));
        using F = RigExecProviderFrameT<Vec3d>;
        const Mat4d kernel = RigExecProviderFinalMatrixT<Mat3d, Vec3d, Mat4d>(
            F{unusableRest.points, true, true}, F{posed.points, true, false},
            F{base.points, true, false}, joint);
        CHECK(SameMatrix(h.Out<Mat4d>(0), kernel));
    }
    ProviderMatrixPayload basePayload;
    basePayload.posed = false;
    Harness h(ProviderMatrix(), &basePayload,
              {&ProviderMatrixRest(), &ProviderMatrixBase()},
              {&ProviderMatrixOut()});
    CHECK(h.Run({&rest, &base}));
    CHECK(SameMatrix(h.Out<Mat4d>(0), Translate(0, 2, 0)));
    CHECK(h.Run({&unusableRest, &base}));
    CHECK(SameMatrix(h.Out<Mat4d>(0), Translate(0, 0, 0)));
    const ProviderMatrixPayload posedPayload;
    Harness missing(ProviderMatrix(), &posedPayload,
                    {&ProviderMatrixRest(), &ProviderMatrixBase()},
                    {&ProviderMatrixOut()});
    CHECK(!missing.evaluator);
}

void
TestFrameMatrix()
{
    const PointFrame rest = Moved(0, 0, 0);
    const PointFrame frame = Moved(0, 0, 3);
    FrameMatrixPayload solver;
    solver.solverOutput = true;
    for (const int32_t position : {0, 1}) {
        solver.position = position;
        Harness h(
            FrameMatrix(), &solver,
            {&FrameMatrixRest(), &FrameMatrixFrame(), &FrameMatrixPresent()},
            {&FrameMatrixOut(), &FrameMatrixValid()});
        const uint8_t bits[] = {0, 1};
        const RigExecArray<uint8_t> present(bits, 2);
        CHECK(h.Run({&rest, &frame, &present}));
        CHECK(h.Out<uint8_t>(1) == (position == 1));
        CHECK(SameMatrix(h.Out<Mat4d>(0), position == 1 ? Translate(0, 0, 3)
                                                        : Translate(0, 0, 0)));
    }
    FrameMatrixPayload constraint;
    for (const int32_t target : {0, 1}) {
        constraint.target = target;
        Harness h(FrameMatrix(), &constraint,
                  {&FrameMatrixRest(), &FrameMatrixFrame(),
                   &FrameMatrixRecordAfter(), &FrameMatrixRecordEveryTarget()},
                  {&FrameMatrixOut(), &FrameMatrixValid()});
        const uint8_t yes = 1, no = 0;
        CHECK(h.Run({&rest, &frame, &yes, &no}));
        CHECK(h.Out<uint8_t>(1) == (target == 0));
        CHECK(h.Run({&rest, &frame, &yes, &yes}));
        CHECK(h.Out<uint8_t>(1) == 1);
        CHECK(h.Run({&rest, &frame, &no, &yes}));
        CHECK(h.Out<uint8_t>(1) == 0);
    }
    Harness missing(FrameMatrix(), &solver,
                    {&FrameMatrixRest(), &FrameMatrixFrame()},
                    {&FrameMatrixOut()});
    CHECK(!missing.evaluator);
}

void
TestProviderRefresh()
{
    const std::vector<const RigExecPortHandle *> ins = {
        &ProviderRefreshBase(),        &ProviderRefreshFinal(),
        &ProviderRefreshRaw(),         &ProviderRefreshCurrent(),
        &ProviderRefreshConstrained(), &ProviderRefreshCarryBase(),
        &ProviderRefreshCarryFinal(),  &ProviderRefreshBlockingParents()};
    const std::vector<const RigExecPortHandle *> outs = {
        &ProviderRefreshBaseOut(), &ProviderRefreshFinalOut(),
        &ProviderRefreshCarryBaseOut(), &ProviderRefreshCarryFinalOut()};
    ProviderRefreshPayload payload;
    payload.blockingCounts = {0, 1};
    Harness h(ProviderRefresh(), &payload, ins, outs);
    const PointFrame before = Moved(0, 0, 0), raw = Moved(0, 1, 0),
                     current = Moved(2, 0, 0);
    const PointFrame carries[] = {Moved(5, 0, 0), Moved(0, 5, 0)};
    const RigExecArray<PointFrame> carry(carries, 2);
    const Mat4d parents[] = {Translate(0, 0, 0)};
    const RigExecArray<Mat4d> blocking(parents, 1);
    const uint8_t no[] = {0}, yes[] = {0, 1};
    const RigExecArray<uint8_t> none(no, 1), constrained(yes, 2);
    CHECK(h.Run(
        {&before, &before, &raw, &current, &none, &carry, &carry, &blocking}));
    CHECK(SameBits(h.Out<PointFrame>(0), raw));
    CHECK(SameBits(h.Out<PointFrame>(1), current));
    const auto carried = h.Out<RigExecArray<PointFrame>>(3);
    CHECK(carried.Size() == 2);
    CHECK(carried.Size() == 2 && carried[0].points[0][0] == 7);
    CHECK(carried.Size() == 2 && carried[1].points[0][0] == 2 &&
          carried[1].points[0][1] == 5);
    // A parent space off identity holds its descendant in place.
    const Mat4d moved[] = {Translate(0, 0, 1)};
    const RigExecArray<Mat4d> held(moved, 1);
    CHECK(h.Run(
        {&before, &before, &raw, &current, &none, &carry, &carry, &held}));
    const auto kept = h.Out<RigExecArray<PointFrame>>(3);
    CHECK(kept.Size() == 2 && SameBits(kept[1], carries[1]));
    CHECK(h.Run({&before, &before, &raw, &current, &constrained, &carry, &carry,
                 &blocking}));
    CHECK(SameBits(h.Out<PointFrame>(0), before));
    CHECK(SameBits(h.Out<PointFrame>(1), before));
    const PointFrame degenerate = Moved(2, 0, 0, 1 | 2);
    CHECK(!h.Run({&before, &before, &raw, &degenerate, &none, &carry, &carry,
                  &blocking}));
    CHECK(h.error.find("ProviderRefresh") != std::string::npos);
    payload.blockingCounts = {0, 0, 0};
    Harness mismatch(ProviderRefresh(), &payload, ins, outs);
    CHECK(mismatch.evaluator);
    CHECK(!mismatch.Run(
        {&before, &before, &raw, &current, &none, &carry, &carry, &blocking}));
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
    TestProviderMatrix();
    TestFrameMatrix();
    TestProviderRefresh();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
