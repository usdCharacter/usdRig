#include "rigExecOps/deform/matrixDeformOp.h"

#include "rigExecMath/matrixKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(MatrixDeform)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformPoints)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformTransform)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

// The programs' default; the SSE and scalar forms agree only to tolerance.
constexpr bool kUseSimd = true;

enum MatrixDeformField : uint32_t { kRadial };

bool
DeformOne(const rigExec::RigExecOpIo &io, bool radial, std::string *error)
{
    const Points &in = *In<Points>(io, MatrixDeformPoints());
    const Mat4d &transform = *In<Mat4d>(io, MatrixDeformTransform());
    const Envelope *envelope = In<Envelope>(io, MatrixDeformEnvelope());
    if (envelope) {
        if (envelope->Size() != in.Size())
            return Fail(error, "MatrixDeform",
                        "envelope length differs from the points");
        for (float w : *envelope)
            if (!std::isfinite(w) || w < 0.0f || w > 1.0f)
                return Fail(error, "MatrixDeform",
                            "envelope weights must be finite and in [0, 1]");
    }
    Points &out = Out<Points>(io, MatrixDeformOut());
    out.Assign(in.Size());
    if (!envelope)
        rigExec::RigExecApplyMatrixFullStrengthKernel<Vec3f, Vec3d>(
            transform, radial, in.Data(), out.Data(), in.Size(), kUseSimd);
    else
        rigExec::RigExecApplyMatrixRangeKernel<Vec3f, Vec3d>(
            transform, radial, envelope->Data(), 0, in.Size(), in.Data(),
            out.Data(), kUseSimd, false);
    return true;
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint8_t *radial =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kRadial);
    return EachInstance(batch, failed, [&](size_t k) {
        return DeformOne(rigExec::RigExecBatchInstance(batch, k),
                         radial[k] != 0, error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const auto &p = *static_cast<const MatrixDeformPayload *>(payload);
    SetFlag(out, kRadial, p.radial);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Flag("radial"),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&MatrixDeformPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&MatrixDeformTransform(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "transform"},
    {&MatrixDeformEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&MatrixDeformOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &MatrixDeformPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "MatrixDeform";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable | rigExec::kRigExecOpRanged |
                     rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

} // namespace

void
RegisterMatrixDeformOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(MatrixDeform(), kDesc);
}

} // namespace rigExecOps
