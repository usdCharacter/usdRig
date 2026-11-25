#include "rigExecOps/pose/poseOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/avarScale.h"
#include "rigExecStdTypes/stdTypes.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(AvarInputs)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsValues)
RIGEXEC_DEFINE_PORT_HANDLE(AvarInputsOut)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
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

} // namespace

namespace {

using rigExec::kRigExecPortInput;
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

} // namespace

void
RegisterPoseOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(AvarInputs(), kAvarInputsDesc);
}

} // namespace rigExecOps
