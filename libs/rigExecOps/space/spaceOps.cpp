#include "rigExecOps/space/spaceOps.h"
#include "rigExecOps/space/providerOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/providerMatrixKernel.h"
#include "rigExecMath/providerRefreshKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecRigTypes/rigTypes.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(ProviderMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderMatrixRest)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderMatrixFinal)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderMatrixBase)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderMatrixOut)

RIGEXEC_DEFINE_OP_HANDLE(FrameMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixRest)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixFrame)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixPresent)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixRecordAfter)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixRecordEveryTarget)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixOut)
RIGEXEC_DEFINE_PORT_HANDLE(FrameMatrixValid)

RIGEXEC_DEFINE_OP_HANDLE(ProviderRefresh)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshBase)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshFinal)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshRaw)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshCurrent)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshConstrained)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshCarryBase)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshCarryFinal)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshBlockingParents)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshBaseOut)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshFinalOut)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshCarryBaseOut)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderRefreshCarryFinalOut)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecStd::Mat3d;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using namespace opIo;
using namespace fieldIo;

using FrameView = rigExec::RigExecProviderFrameT<Vec3d>;

FrameView
View(const PointFrame &frame)
{
    return FrameView{frame.points,
                     (frame.flags & rigExecRig::kPointFrameValid) != 0,
                     (frame.flags & rigExecRig::kPointFrameDegenerate) != 0};
}

// ---- ProviderMatrix ----

enum ProviderMatrixField : uint32_t { kPosed, kJoint };

bool
CheckProviderMatrix(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (FlagAt(check.batch, kPosed, check.k) &&
        !Fed(check, ProviderMatrixFinal()))
        return Fail(error, "ProviderMatrix", "a final matrix needs Final");
    return true;
}

bool
ProviderMatrixOne(const RigExecOpBatch &batch, size_t k)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const FrameView rest = View(*In<PointFrame>(io, ProviderMatrixRest()));
    const FrameView base = View(*In<PointFrame>(io, ProviderMatrixBase()));
    Mat4d &out = Out<Mat4d>(io, ProviderMatrixOut());
    if (FlagAt(batch, kPosed, k))
        out = rigExec::RigExecProviderFinalMatrixT<Mat3d, Vec3d, Mat4d>(
            rest, View(*In<PointFrame>(io, ProviderMatrixFinal())), base,
            FlagAt(batch, kJoint, k));
    else
        out = rigExec::RigExecProviderBaseMatrixT<Mat3d, Vec3d, Mat4d>(rest,
                                                                       base);
    return true;
}

bool
RunProviderMatrix(const RigExecOpBatch &batch, size_t *failed,
                  std::string * /*error*/)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return ProviderMatrixOne(batch, k); });
}

// ---- FrameMatrix ----

enum FrameMatrixField : uint32_t { kSolverOutput, kPosition, kTarget };

/// What eval::FrameRecordMatrix needs of a math binding.
struct FrameRecordMath {
    using Mat4 = Mat4d;
    using Mat3 = Mat3d;
    using Vec3 = Vec3d;
    static Mat4 Identity4()
    {
        Mat4 m;
        m.SetIdentity();
        return m;
    }
    static Mat3 Identity3()
    {
        Mat3 m;
        m.SetIdentity();
        return m;
    }
    static Mat4 Mat4FromElements(double m00, double m01, double m02, double m03,
                                 double m10, double m11, double m12, double m13,
                                 double m20, double m21, double m22, double m23,
                                 double m30, double m31, double m32, double m33)
    {
        return Mat4(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23,
                    m30, m31, m32, m33);
    }
    static Mat3 Mat3FromElements(double m00, double m01, double m02, double m10,
                                 double m11, double m12, double m20, double m21,
                                 double m22)
    {
        return Mat3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
    static Vec3 Vec3Of(double x, double y, double z)
    {
        return Vec3(x, y, z);
    }
};

bool
CheckFrameMatrix(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (FlagAt(check.batch, kSolverOutput, check.k)) {
        if (!Fed(check, FrameMatrixPresent()))
            return Fail(error, "FrameMatrix",
                        "a solver commit's record needs Present");
        return true;
    }
    if (!Fed(check, FrameMatrixRecordAfter()) ||
        !Fed(check, FrameMatrixRecordEveryTarget()))
        return Fail(error, "FrameMatrix",
                    "a constraint commit's record needs RecordAfter and "
                    "RecordEveryTarget");
    return true;
}

bool
FrameMatrixOne(const RigExecOpBatch &batch, size_t k)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const bool solverOutput = FlagAt(batch, kSolverOutput, k);
    const auto *present = In<RigExecArray<uint8_t>>(io, FrameMatrixPresent());
    const bool recorded = rigExec::eval::FrameRecordRecorded(
        solverOutput, ValueAt<int32_t>(batch, kPosition, k),
        ValueAt<int32_t>(batch, kTarget, k),
        present ? reinterpret_cast<const char *>(present->Data()) : nullptr,
        present ? present->Size() : 0,
        !solverOutput && *In<uint8_t>(io, FrameMatrixRecordAfter()) != 0,
        !solverOutput && *In<uint8_t>(io, FrameMatrixRecordEveryTarget()) != 0);
    const PointFrame &rest = *In<PointFrame>(io, FrameMatrixRest());
    const PointFrame &frame = *In<PointFrame>(io, FrameMatrixFrame());
    Mat4d &out = Out<Mat4d>(io, FrameMatrixOut());
    out.SetIdentity();
    bool valid = false;
    if (recorded)
        valid = rigExec::eval::FrameRecordMatrix<FrameRecordMath>(
            rest.points, rest.flags, frame.points, frame.flags, &out);
    Out<uint8_t>(io, FrameMatrixValid()) = valid ? 1 : 0;
    return true;
}

bool
RunFrameMatrix(const RigExecOpBatch &batch, size_t *failed,
               std::string * /*error*/)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return FrameMatrixOne(batch, k); });
}

// ---- ProviderRefresh ----

enum ProviderRefreshField : uint32_t { kBlockingCount, kBlockingCounts };

/// What RigExecRefreshProviderFrames needs of a math binding.
struct RefreshMath {
    using Frame = PointFrame;
    using Matrix = Mat4d;
    static bool Usable(const Frame &frame)
    {
        return rigExec::RigExecProviderFrameUsableT(View(frame));
    }
    static bool PointsToMatrix(const Frame &before, const Frame &after,
                               Matrix *out)
    {
        return rigExec::RigExecPointsToMatrixT<Mat3d>(before.points,
                                                      after.points, out);
    }
    static Frame Transform(const Frame &frame, const Matrix &delta)
    {
        Frame out;
        bool reflected = false, degenerate = false;
        out.points = rigExec::RigExecMatrixToPointsT<Mat3d>(
            frame.points, delta, &reflected, &degenerate);
        out.flags = rigExecRig::kPointFrameValid |
                    (reflected ? rigExecRig::kPointFrameReflected : 0u) |
                    (degenerate ? rigExecRig::kPointFrameDegenerate : 0u);
        return out;
    }
};

size_t
BlockingTotal(const rigExec::RigExecFieldView<const uint32_t> &counts)
{
    size_t total = 0;
    for (size_t i = 0; i < counts.Size(); ++i)
        total += counts.data[i];
    return total;
}

bool
CheckProviderRefresh(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const auto counts = ListAt<uint32_t>(check.batch, kBlockingCounts, check.k);
    if (!SameCount(CheckElements(check, ProviderRefreshCarryBase()),
                   counts.Size()) ||
        !SameCount(CheckElements(check, ProviderRefreshCarryFinal()),
                   counts.Size()))
        return Fail(error, "ProviderRefresh",
                    "needs one base and one final frame per carried "
                    "descendant");
    const size_t parents = BlockingTotal(counts);
    if (parents && !Fed(check, ProviderRefreshBlockingParents()))
        return Fail(error, "ProviderRefresh", "needs BlockingParents");
    if (Fed(check, ProviderRefreshBlockingParents()) &&
        !SameCount(CheckElements(check, ProviderRefreshBlockingParents()),
                   parents))
        return Fail(error, "ProviderRefresh",
                    "needs one parent space per blocking pose");
    return true;
}

const char *
RefreshFailure(rigExec::RigExecProviderRefreshOutcome outcome)
{
    using O = rigExec::RigExecProviderRefreshOutcome;
    switch (outcome) {
    case O::MissingBase:
        return "connected base pose input incomplete";
    case O::MissingCurrent:
        return "connected final pose input incomplete";
    case O::InvalidBase:
        return "produced an invalid or degenerate base frame";
    case O::InvalidCurrent:
        return "produced an invalid or degenerate frame";
    case O::SingularCurrent:
        return "produced a singular hierarchy delta";
    case O::InvalidDescendant:
        return "could not propagate its pose revision through carried "
               "descendant ";
    case O::InvalidTransformedDescendant:
        return "produced an invalid frame for carried descendant ";
    case O::Kept:
    case O::Published:
        break;
    }
    return nullptr;
}

bool
ProviderRefreshOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto counts = ListAt<uint32_t>(batch, kBlockingCounts, k);
    const size_t count = counts.Size();
    const auto &carryBase =
        *In<RigExecArray<PointFrame>>(io, ProviderRefreshCarryBase());
    const auto &carryFinal =
        *In<RigExecArray<PointFrame>>(io, ProviderRefreshCarryFinal());
    if (carryBase.Size() != count || carryFinal.Size() != count)
        return Fail(error, "ProviderRefresh",
                    "needs one base and one final frame per carried "
                    "descendant");
    bool constrained = false;
    if (const auto *prior =
            In<RigExecArray<uint8_t>>(io, ProviderRefreshConstrained()))
        for (const uint8_t committed : *prior)
            constrained = constrained || committed != 0;
    const auto *parents =
        In<RigExecArray<Mat4d>>(io, ProviderRefreshBlockingParents());
    Mat4d identity;
    identity.SetIdentity();
    RigExecArray<uint8_t> blocked(count);
    size_t at = 0;
    for (size_t c = 0; c < count; ++c) {
        blocked[c] = 0;
        for (uint32_t p = 0; p < counts.data[c]; ++p, ++at)
            if (!parents || at >= parents->Size() || (*parents)[at] != identity)
                blocked[c] = 1;
    }
    auto &carryBaseOut =
        Out<RigExecArray<PointFrame>>(io, ProviderRefreshCarryBaseOut());
    auto &carryFinalOut =
        Out<RigExecArray<PointFrame>>(io, ProviderRefreshCarryFinalOut());
    carryBaseOut.Resize(count);
    carryFinalOut.Resize(count);
    size_t failedCarry = 0;
    const auto outcome = rigExec::RigExecRefreshProviderFrames<RefreshMath>(
        *In<PointFrame>(io, ProviderRefreshBase()),
        *In<PointFrame>(io, ProviderRefreshFinal()),
        In<PointFrame>(io, ProviderRefreshRaw()),
        In<PointFrame>(io, ProviderRefreshCurrent()), constrained,
        carryBase.Data(), carryFinal.Data(), blocked.Data(), count,
        &Out<PointFrame>(io, ProviderRefreshBaseOut()),
        &Out<PointFrame>(io, ProviderRefreshFinalOut()), carryBaseOut.Data(),
        carryFinalOut.Data(), &failedCarry);
    const char *failure = RefreshFailure(outcome);
    if (!failure)
        return true;
    std::string why = failure;
    if (outcome == rigExec::RigExecProviderRefreshOutcome::InvalidDescendant ||
        outcome == rigExec::RigExecProviderRefreshOutcome::
                       InvalidTransformedDescendant)
        why += std::to_string(failedCarry);
    return Fail(error, "ProviderRefresh", why);
}

bool
RunProviderRefresh(const RigExecOpBatch &batch, size_t *failed,
                   std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ProviderRefreshOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kProviderMatrixPorts[] = {
    {&ProviderMatrixRest(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "rest"},
    {&ProviderMatrixFinal(), &rigExecRig::PointFrameType(), kOptional, "final"},
    {&ProviderMatrixBase(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "base"},
    {&ProviderMatrixOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

void
ProviderMatrixValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const ProviderMatrixPayload *>(payload);
    SetFlag(out, kPosed, p.posed);
    SetFlag(out, kJoint, p.joint);
}

const rigExec::RigExecFieldDesc kProviderMatrixFields[] = {
    Flag("posed"),
    Flag("joint"),
};

const rigExec::RigExecOpDesc kProviderMatrixDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "ProviderMatrix";
    d.ports = kProviderMatrixPorts;
    d.portCount = std::size(kProviderMatrixPorts);
    d.runBatch = &RunProviderMatrix;
    d.validate = &CheckProviderMatrix;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kProviderMatrixFields;
    d.payloadFieldCount = std::size(kProviderMatrixFields);
    d.payloadVersion = 1;
    d.payloadValues = &ProviderMatrixValues;
    return d;
}();

const rigExec::RigExecPortDesc kFrameMatrixPorts[] = {
    {&FrameMatrixRest(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "rest"},
    {&FrameMatrixFrame(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "frame"},
    {&FrameMatrixPresent(), &rigExecStd::BoolArray(), kOptional, "present"},
    {&FrameMatrixRecordAfter(), &rigExecStd::BoolType(), kOptional,
     "recordAfter"},
    {&FrameMatrixRecordEveryTarget(), &rigExecStd::BoolType(), kOptional,
     "recordEveryTarget"},
    {&FrameMatrixOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
    {&FrameMatrixValid(), &rigExecStd::BoolType(), kRigExecPortOutput, "valid"},
};

void
FrameMatrixValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const FrameMatrixPayload *>(payload);
    SetFlag(out, kSolverOutput, p.solverOutput);
    out.Set(kPosition, p.position);
    out.Set(kTarget, p.target);
}

const rigExec::RigExecFieldDesc kFrameMatrixFields[] = {
    Flag("solverOutput"),
    Value("position", rigExecStd::IntType()),
    Value("target", rigExecStd::IntType()),
};

const rigExec::RigExecOpDesc kFrameMatrixDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "FrameMatrix";
    d.ports = kFrameMatrixPorts;
    d.portCount = std::size(kFrameMatrixPorts);
    d.runBatch = &RunFrameMatrix;
    d.validate = &CheckFrameMatrix;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFrameMatrixFields;
    d.payloadFieldCount = std::size(kFrameMatrixFields);
    d.payloadVersion = 1;
    d.payloadValues = &FrameMatrixValues;
    return d;
}();

const rigExec::RigExecPortDesc kProviderRefreshPorts[] = {
    {&ProviderRefreshBase(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "base"},
    {&ProviderRefreshFinal(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "final"},
    {&ProviderRefreshRaw(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "raw"},
    {&ProviderRefreshCurrent(), &rigExecRig::PointFrameType(),
     kRigExecPortInput, "current"},
    {&ProviderRefreshConstrained(), &rigExecStd::BoolArray(), kOptional,
     "constrained"},
    {&ProviderRefreshCarryBase(), &rigExecRig::PointFrameList(),
     kRigExecPortInput, "carryBase"},
    {&ProviderRefreshCarryFinal(), &rigExecRig::PointFrameList(),
     kRigExecPortInput, "carryFinal"},
    {&ProviderRefreshBlockingParents(), &rigExecStd::Mat4dArray(), kOptional,
     "blockingParents"},
    {&ProviderRefreshBaseOut(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "baseOut"},
    {&ProviderRefreshFinalOut(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "finalOut"},
    {&ProviderRefreshCarryBaseOut(), &rigExecRig::PointFrameList(),
     kRigExecPortOutput, "carryBaseOut", rigExec::RigExecOutputBound::Field,
     nullptr, nullptr, kBlockingCount},
    {&ProviderRefreshCarryFinalOut(), &rigExecRig::PointFrameList(),
     kRigExecPortOutput, "carryFinalOut", rigExec::RigExecOutputBound::Field,
     nullptr, nullptr, kBlockingCount},
};

void
ProviderRefreshValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const ProviderRefreshPayload *>(payload);
    SetList(out, kBlockingCount, p.blockingCounts);
}

const rigExec::RigExecFieldDesc kProviderRefreshFields[] = {
    Count("blockingCount"),
    Items("blockingCounts", rigExecStd::UIntType(), kBlockingCount),
};

const rigExec::RigExecOpDesc kProviderRefreshDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "ProviderRefresh";
    d.ports = kProviderRefreshPorts;
    d.portCount = std::size(kProviderRefreshPorts);
    d.runBatch = &RunProviderRefresh;
    d.validate = &CheckProviderRefresh;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kProviderRefreshFields;
    d.payloadFieldCount = std::size(kProviderRefreshFields);
    d.payloadVersion = 1;
    d.payloadValues = &ProviderRefreshValues;
    return d;
}();

} // namespace

void
RegisterSpaceOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(ProviderMatrix(), kProviderMatrixDesc);
    registrar.Add(FrameMatrix(), kFrameMatrixDesc);
    registrar.Add(ProviderRefresh(), kProviderRefreshDesc);
    RegisterProviderOps(registrar);
}

} // namespace rigExecOps
