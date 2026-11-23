#include "rigExecOps/deform/matrixDeformOp.h"

#include "rigExecMath/matrixKernel.h"
#include "rigExecOps/opIo.h"

#include <cmath>
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
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

// The programs' default; the SSE and scalar forms agree only to tolerance.
constexpr bool kUseSimd = true;

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const bool radial =
        static_cast<const MatrixDeformPayload *>(payload)->radial;
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
    out = Points(in.Size());
    if (!envelope)
        rigExec::RigExecApplyMatrixFullStrengthKernel<Vec3f, Vec3d>(
            transform, radial, in.Data(), out.Data(), in.Size(), kUseSimd);
    else
        rigExec::RigExecApplyMatrixRangeKernel<Vec3f, Vec3d>(
            transform, radial, envelope->Data(), 0, in.Size(), in.Data(),
            out.Data(), kUseSimd, false);
    return true;
}

} // namespace

void
RegisterMatrixDeformOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    const auto &points = rigExecStd::Vec3fArray();
    static const rigExec::RigExecPortDesc ports[] = {
        {&MatrixDeformPoints(), &points, kRigExecPortInput, "points"},
        {&MatrixDeformTransform(), &rigExecStd::Mat4dType(), kRigExecPortInput,
         "transform"},
        {&MatrixDeformEnvelope(), &rigExecStd::FloatArray(),
         kRigExecPortInput | kRigExecPortOptional, "envelope"},
        {&MatrixDeformOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "MatrixDeform";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable |
                         rigExec::kRigExecOpFreezable |
                         rigExec::kRigExecOpRanged | rigExec::kRigExecOpGate |
                         rigExec::kRigExecOpFromLeaves;
        d.payload = rigExec::RigExecPayloadHandlingFor<MatrixDeformPayload>();
        return d;
    }();
    registrar.Add(MatrixDeform(), desc);
}

} // namespace rigExecOps
