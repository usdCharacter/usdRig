#include "rigExecOps/deform/matrixDeformOp.h"

#include "rigExecEvalCore/envelope.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(MatrixDeform)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformPoints)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformTransform)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformWeightIndices)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformWeights)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

enum MatrixDeformField : uint32_t { kRadial };

using Math = rigExec::eval::MatrixKernelMath<Vec3f, Vec3d, Mat4d>;
using Indices = rigExecStd::RigExecArray<int32_t>;

/// The node's weights as the matrix kernel reads them; false, with \p error,
/// for weights it cannot take. Checks only when \p check.
bool
ViewOf(const rigExec::RigExecOpIo &io, bool check,
       rigExec::eval::EnvelopePacketView *view, std::string *error)
{
    const Points &in = *In<Points>(io, MatrixDeformPoints());
    const Envelope *envelope = In<Envelope>(io, MatrixDeformEnvelope());
    const Indices *indices = In<Indices>(io, MatrixDeformWeightIndices());
    const Envelope *weights = In<Envelope>(io, MatrixDeformWeights());
    if (check && !indices != !weights)
        return Fail(error, "MatrixDeform",
                    "weight indices and weights go together");
    if (check && indices && envelope)
        return Fail(error, "MatrixDeform",
                    "an envelope or sparse weights, not both");
    view->valid = true;
    view->rangePolicyEmpty = true;
    if (indices) {
        view->representation = rigExec::eval::kWeightSparse;
        view->values = weights->Data();
        view->valueCount = weights->Size();
        view->indices = indices->Data();
        view->indexCount = indices->Size();
        if (check && (indices->Size() != weights->Size() ||
                      !rigExec::eval::SparseEnvelopeIsUsable(*view, in.Size())))
            return Fail(error, "MatrixDeform",
                        "sparse weights must name points in ascending order "
                        "with weights finite and in [0, 1]");
    } else if (envelope) {
        if (check && envelope->Size() != in.Size())
            return Fail(error, "MatrixDeform",
                        "envelope length differs from the points");
        if (check)
            for (float w : *envelope)
                if (!std::isfinite(w) || w < 0.0f || w > 1.0f)
                    return Fail(error, "MatrixDeform",
                                "envelope weights must be finite and in "
                                "[0, 1]");
        view->representation = rigExec::eval::kWeightDense;
    } else {
        view->representation = rigExec::eval::kWeightConstant;
        view->defaultWeight = 1.0f;
    }
    return true;
}

rigExec::RigExecRangePlan
Accept(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
       std::string *error)
{
    rigExec::eval::EnvelopePacketView view;
    return ViewOf(rigExec::RigExecBatchInstance(batch, k), true, &view, error)
               ? rigExec::RigExecRangePlan::Ranges
               : rigExec::RigExecRangePlan::Refuse;
}

/// ApplyAcceptedMatrixKernel's points [begin, end): the sparse walk over
/// the entries naming them, or the dense or full-strength range.
bool
Deform(const rigExec::RigExecOpBatch &batch, size_t k,
       const rigExec::RigExecOpRange &range, std::string *)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    rigExec::eval::EnvelopePacketView view;
    ViewOf(io, false, &view, nullptr);
    const Points &in = *In<Points>(io, MatrixDeformPoints());
    const Mat4d &transform = *In<Mat4d>(io, MatrixDeformTransform());
    const Envelope *envelope = In<Envelope>(io, MatrixDeformEnvelope());
    const bool radial =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kRadial)[k] != 0;
    Points &out = Out<Points>(io, MatrixDeformOut());
    if (rigExec::eval::EnvelopeIsSparseWalk(view)) {
        CopyRange(in, out, range.begin, range.end);
        if (transform == Math::Identity())
            return true;
        const auto *first = view.indices;
        const auto *last = view.indices + view.indexCount;
        const size_t from =
            size_t(std::lower_bound(first, last, int(range.begin)) - first);
        const size_t to =
            size_t(std::lower_bound(first, last, int(range.end)) - first);
        rigExec::eval::ApplyMatrixSparseWalk<Math>(radial, transform, view,
                                                   from, to, out.Data());
        return true;
    }
    rigExec::eval::ApplyMatrixKernelRange<Math>(
        radial, transform,
        rigExec::eval::EnvelopeIsFullStrength(view)
            ? nullptr
            : (envelope ? envelope->Data() : nullptr),
        range.begin, range.end, in.Data(), out.Data(), io.useSimd);
    return true;
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
using rigExec::kRigExecPortPointwise;

const rigExec::RigExecPortDesc kPorts[] = {
    {&MatrixDeformPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | kRigExecPortPointwise, "points"},
    {&MatrixDeformTransform(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "transform"},
    {&MatrixDeformEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional | kRigExecPortPointwise,
     "envelope"},
    {&MatrixDeformWeightIndices(), &rigExecStd::IntArray(),
     kRigExecPortInput | kRigExecPortOptional, "weightIndices"},
    {&MatrixDeformWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "weights"},
    {&MatrixDeformOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &MatrixDeformPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "MatrixDeform";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.range.accept = &Accept;
    d.range.run = &Deform;
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
