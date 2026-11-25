#include "rigExecOps/pose/poseOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/avarScale.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/poseMath.h"
#include "rigExecStdTypes/stdTypes.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(AvarInputs)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsValues)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsOut)

RIGEXEC_DEFINE_OP_HANDLE(SpaceCheckpoint)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointAvars)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentExpression)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointParentExpressionAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointPosedAuthoredMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointAnchor)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceCheckpointOut)

uint32_t
SpaceCheckpointRotOrderCode(const char *text, size_t size)
{
    if (size != 3)
        return 0;
    return uint32_t(uint8_t(text[0])) | uint32_t(uint8_t(text[1])) << 8 |
           uint32_t(uint8_t(text[2])) << 16;
}

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using namespace opIo;
using namespace fieldIo;

// ---- AvarInputs ----

enum AvarInputsField : uint32_t { kAvarCount, kTargetCount, kTargets };

bool
CheckAvarInputs(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const uint64_t avars = ValueAt<uint64_t>(check.batch, kAvarCount, check.k);
    const auto targets = ListAt<uint32_t>(check.batch, kTargets, check.k);
    std::vector<uint8_t> written(size_t(avars), 0);
    for (size_t i = 0; i < targets.Size(); ++i) {
        if (targets.data[i] >= avars)
            return Fail(error, "AvarInputs",
                        "binding " + std::to_string(i) +
                            " names an avar past avarCount");
        written[targets.data[i]] = 1;
    }
    for (size_t a = 0; a < written.size(); ++a)
        if (!written[a])
            return Fail(error, "AvarInputs",
                        "avar " + std::to_string(a) + " has no binding");
    if (!SameCount(CheckElements(check, AvarInputsValues()), targets.Size()))
        return Fail(error, "AvarInputs", "needs one value per binding");
    return true;
}

bool
AvarInputsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto targets = ListAt<uint32_t>(batch, kTargets, k);
    const auto &values = *In<RigExecArray<double>>(io, AvarInputsValues());
    if (values.Size() != targets.Size())
        return Fail(error, "AvarInputs", "needs one value per binding");
    auto &out = Out<RigExecArray<double>>(io, AvarInputsOut());
    out.Resize(size_t(ValueAt<uint64_t>(batch, kAvarCount, k)));
    rigExec::RigExecWriteAvarBindings(
        targets.Size(), [&](size_t i) { return targets.data[i]; },
        [&](size_t i) { return values[i]; },
        [&](uint32_t avar, double value) { out[avar] = value; });
    return true;
}

bool
RunAvarInputs(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return AvarInputsOne(batch, k, error);
    });
}

// ---- SpaceCheckpoint ----

enum SpaceCheckpointField : uint32_t {
    kRotOrderCount,
    kRotOrders,
    kSignCount,
    kRotationSigns,
    kNoScaleCount,
    kNoScale
};

using CheckpointMath = rigExec::RrPoseMathT<rigExecRig::PointFrame>;

const rigExec::RigExecPortHandle *const kCheckpointSlotPorts[] = {
    &SpaceCheckpointPosedDefault(),
    &SpaceCheckpointParentDefaultInverse(),
    &SpaceCheckpointParentExpression(),
    &SpaceCheckpointParentExpressionAuthored(),
    &SpaceCheckpointPosedAuthored(),
    &SpaceCheckpointPosedAuthoredMatrix(),
};

bool
CheckSpaceCheckpoint(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t slots =
        ListAt<uint32_t>(check.batch, kRotOrders, check.k).Size();
    if (ListAt<uint32_t>(check.batch, kRotationSigns, check.k).Size() !=
            slots ||
        ListAt<uint8_t>(check.batch, kNoScale, check.k).Size() != slots)
        return Fail(error, "SpaceCheckpoint",
                    "needs one rotation order, sign mask and scale flag per "
                    "slot");
    bool fits =
        SameCount(CheckElements(check, SpaceCheckpointAvars()), slots * 11);
    for (const rigExec::RigExecPortHandle *port : kCheckpointSlotPorts)
        fits = fits && SameCount(CheckElements(check, *port), slots);
    if (!fits)
        return Fail(error, "SpaceCheckpoint",
                    "needs 11 avars and one of each matrix and flag per slot");
    return true;
}

bool
SpaceCheckpointOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto orders = ListAt<uint32_t>(batch, kRotOrders, k);
    const auto signs = ListAt<uint32_t>(batch, kRotationSigns, k);
    const auto noScale = ListAt<uint8_t>(batch, kNoScale, k);
    const size_t slots = orders.Size();
    const auto &avars = *In<RigExecArray<double>>(io, SpaceCheckpointAvars());
    const auto &posedD =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointPosedDefault());
    const auto &parentDinv =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointParentDefaultInverse());
    const auto &parentM =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointParentExpression());
    const auto &parentAuthored = *In<RigExecArray<uint8_t>>(
        io, SpaceCheckpointParentExpressionAuthored());
    const auto &posedAuthored =
        *In<RigExecArray<uint8_t>>(io, SpaceCheckpointPosedAuthored());
    const auto &posedAuthoredM =
        *In<RigExecArray<Mat4d>>(io, SpaceCheckpointPosedAuthoredMatrix());
    if (avars.Size() != slots * 11 || posedD.Size() != slots ||
        parentDinv.Size() != slots || parentM.Size() != slots ||
        parentAuthored.Size() != slots || posedAuthored.Size() != slots ||
        posedAuthoredM.Size() != slots)
        return Fail(error, "SpaceCheckpoint",
                    "needs 11 avars and one of each matrix and flag per slot");
    std::vector<rigExec::RigExecSpaceCheckpointInputT<Mat4d>> inputs(slots);
    for (size_t s = 0; s < slots; ++s) {
        rigExec::eval::PoseSlotComposeT<CheckpointMath> view;
        view.avars11 = &avars[s * 11];
        const uint32_t code = orders.data[s];
        if (code) {
            const char text[3] = {char(code & 0xff), char((code >> 8) & 0xff),
                                  char((code >> 16) & 0xff)};
            view.rotOrder.assign(text, 3);
        }
        view.rotationSign = signs.data[s];
        view.noScaleAvars = noScale.data[s] != 0;
        view.posedAuthored = posedAuthored[s] != 0;
        view.posedAuthoredM = &posedAuthoredM[s];
        view.posedD = &posedD[s];
        view.parentDinv = &parentDinv[s];
        view.parentSpaceM = &parentM[s];
        view.parentSpaceAuthored = parentAuthored[s] != 0;
        inputs[s] = rigExec::eval::CheckpointInputOf<CheckpointMath>(view);
    }
    const Mat4d *anchor = In<Mat4d>(io, SpaceCheckpointAnchor());
    Out<Mat4d>(io, SpaceCheckpointOut()) =
        rigExec::eval::SpaceCheckpointMatrix<CheckpointMath>(
            inputs.data(), slots,
            anchor ? *anchor : CheckpointMath::Identity4());
    return true;
}

bool
RunSpaceCheckpoint(const RigExecOpBatch &batch, size_t *failed,
                   std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SpaceCheckpointOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kAvarInputsPorts[] = {
    {&AvarInputsValues(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "values"},
    {&AvarInputsOut(), &rigExecStd::DoubleArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Field, nullptr, nullptr, kAvarCount},
};

void
AvarInputsValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const AvarInputsPayload *>(payload);
    out.Set(kAvarCount, p.avarCount);
    SetList(out, kTargetCount, p.targets);
}

const rigExec::RigExecFieldDesc kAvarInputsFields[] = {
    Value("avarCount", rigExecStd::UInt64Type()),
    Count("targetCount"),
    Items("targets", rigExecStd::UIntType(), kTargetCount),
};

const rigExec::RigExecOpDesc kAvarInputsDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "AvarInputs";
    d.ports = kAvarInputsPorts;
    d.portCount = std::size(kAvarInputsPorts);
    d.runBatch = &RunAvarInputs;
    d.validate = &CheckAvarInputs;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kAvarInputsFields;
    d.payloadFieldCount = std::size(kAvarInputsFields);
    d.payloadVersion = 1;
    d.payloadValues = &AvarInputsValuesOf;
    return d;
}();

const rigExec::RigExecPortDesc kSpaceCheckpointPorts[] = {
    {&SpaceCheckpointAvars(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "avars"},
    {&SpaceCheckpointPosedDefault(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "posedDefault"},
    {&SpaceCheckpointParentDefaultInverse(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "parentDefaultInverse"},
    {&SpaceCheckpointParentExpression(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "parentExpression"},
    {&SpaceCheckpointParentExpressionAuthored(), &rigExecStd::BoolArray(),
     kRigExecPortInput, "parentExpressionAuthored"},
    {&SpaceCheckpointPosedAuthored(), &rigExecStd::BoolArray(),
     kRigExecPortInput, "posedAuthored"},
    {&SpaceCheckpointPosedAuthoredMatrix(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput, "posedAuthoredMatrix"},
    {&SpaceCheckpointAnchor(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "anchor"},
    {&SpaceCheckpointOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "out"},
};

void
SpaceCheckpointValuesOf(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SpaceCheckpointPayload *>(payload);
    SetList(out, kRotOrderCount, p.rotOrders);
    SetList(out, kSignCount, p.rotationSigns);
    SetList(out, kNoScaleCount, p.noScale);
}

const rigExec::RigExecFieldDesc kSpaceCheckpointFields[] = {
    Count("rotOrderCount"),
    Items("rotOrders", rigExecStd::UIntType(), kRotOrderCount),
    Count("rotationSignCount"),
    Items("rotationSigns", rigExecStd::UIntType(), kSignCount),
    Count("noScaleCount"),
    Items("noScale", rigExecStd::BoolType(), kNoScaleCount),
};

const rigExec::RigExecOpDesc kSpaceCheckpointDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SpaceCheckpoint";
    d.ports = kSpaceCheckpointPorts;
    d.portCount = std::size(kSpaceCheckpointPorts);
    d.runBatch = &RunSpaceCheckpoint;
    d.validate = &CheckSpaceCheckpoint;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kSpaceCheckpointFields;
    d.payloadFieldCount = std::size(kSpaceCheckpointFields);
    d.payloadVersion = 1;
    d.payloadValues = &SpaceCheckpointValuesOf;
    return d;
}();

} // namespace

void
RegisterPoseOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(AvarInputs(), kAvarInputsDesc);
    registrar.Add(SpaceCheckpoint(), kSpaceCheckpointDesc);
}

} // namespace rigExecOps
