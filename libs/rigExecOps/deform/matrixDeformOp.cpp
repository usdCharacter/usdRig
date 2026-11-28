#include "rigExecOps/deform/matrixDeformOp.h"

#include "rigExecEvalCore/envelope.h"
#include "rigExecMath/moverTransformKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/deform/weightPacket.h"
#include "rigExecRigTypes/rigTypes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(MatrixDeform)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformPoints)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformTransform)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformWeightIndices)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformWeights)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformOut)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformPacket)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformRadial)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixDeformWritten)

RIGEXEC_DEFINE_OP_HANDLE(MoverTransform)

RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformTransform)
RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformSpace)
RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformReference)
RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformReferenceSpace)
RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformCarry)
RIGEXEC_DEFINE_PORT_HANDLE(MoverTransformOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

enum MatrixDeformField : uint32_t { kRadial };

using Math = rigExec::eval::MatrixKernelMath<Vec3f, Vec3d, Mat4d>;
using Indices = rigExecStd::RigExecArray<int32_t>;
using rigExecRig::PointWeights;
using weightPacket::PacketView;
using weightPacket::ResolvePacket;

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

/// Weights validate could not see, checked each frame; this frame's pass
/// through and the weight at every point when the kernel reads them
/// densely, written by prepare.
struct MatrixState {
    bool checkEachFrame = true;
    bool pass = false;
    std::vector<float> dense;
};

bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const bool sparse = Fed(check, MatrixDeformWeightIndices());
    if (sparse != Fed(check, MatrixDeformWeights()))
        return Fail(error, "MatrixDeform",
                    "weight indices and weights go together");
    if (sparse && Fed(check, MatrixDeformEnvelope()))
        return Fail(error, "MatrixDeform",
                    "an envelope or sparse weights, not both");
    if (Fed(check, MatrixDeformDefaultWeight()) &&
        (sparse || Fed(check, MatrixDeformEnvelope())))
        return Fail(error, "MatrixDeform",
                    "a default weight or an envelope, not both");
    if (Fed(check, MatrixDeformPacket()) &&
        (sparse || Fed(check, MatrixDeformEnvelope()) ||
         Fed(check, MatrixDeformDefaultWeight())))
        return Fail(error, "MatrixDeform",
                    "a weight packet or another envelope, not both");
    const size_t points = CheckElements(check, MatrixDeformPoints());
    if (sparse && !SameCount(CheckElements(check, MatrixDeformWeightIndices()),
                             CheckElements(check, MatrixDeformWeights())))
        return Fail(error, "MatrixDeform",
                    "sparse weights must name points in ascending order "
                    "with weights finite and in [0, 1]");
    if (!EnvelopeFits(check, MatrixDeformPoints(), MatrixDeformEnvelope(),
                      "MatrixDeform", error))
        return false;
    bool known = points < rigExec::kRigExecPortUnfed;
    if (sparse) {
        const Indices *indices =
            CheckValue<Indices>(check, MatrixDeformWeightIndices());
        const Envelope *weights =
            CheckValue<Envelope>(check, MatrixDeformWeights());
        known = known && indices && weights;
        if (known) {
            rigExec::eval::EnvelopePacketView view;
            view.valid = true;
            view.rangePolicyEmpty = true;
            view.representation = rigExec::eval::kWeightSparse;
            view.values = weights->Data();
            view.valueCount = weights->Size();
            view.indices = indices->Data();
            view.indexCount = indices->Size();
            if (!rigExec::eval::SparseEnvelopeIsUsable(view, points))
                return Fail(error, "MatrixDeform",
                            "sparse weights must name points in ascending "
                            "order with weights finite and in [0, 1]");
        }
    } else if (Fed(check, MatrixDeformEnvelope())) {
        const Envelope *envelope =
            CheckValue<Envelope>(check, MatrixDeformEnvelope());
        known = envelope != nullptr;
        if (known)
            for (float w : *envelope)
                if (!std::isfinite(w) || w < 0.0f || w > 1.0f)
                    return Fail(error, "MatrixDeform",
                                "envelope weights must be finite and in "
                                "[0, 1]");
    }
    if (MatrixState *state = CheckState<MatrixState>(check))
        state->checkEachFrame = !known;
    return true;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
        std::string *error)
{
    auto &state =
        *static_cast<MatrixState *>(rigExec::RigExecBatchState(batch, k));
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const Points &points = *In<Points>(io, MatrixDeformPoints());
    if (!EnvelopeFits(points, In<Envelope>(io, MatrixDeformEnvelope()),
                      "MatrixDeform", error))
        return false;
    const uint8_t *enabled = In<uint8_t>(io, MatrixDeformEnabled());
    const float *weight = In<float>(io, MatrixDeformDefaultWeight());
    const PointWeights *packet = In<PointWeights>(io, MatrixDeformPacket());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    if (weight && !state.pass && *weight != 1.0f) {
        if (!std::isfinite(*weight) || *weight < 0.0f || *weight > 1.0f)
            return Fail(error, "MatrixDeform",
                        "a default weight is not finite or outside [0, 1]");
        state.dense.assign(points.Size(), *weight);
    }
    const Indices *written = In<Indices>(io, MatrixDeformWritten());
    if (written && !state.pass) {
        rigExec::eval::EnvelopePacketView view;
        if (packet)
            view = PacketView(*packet);
        else
            ViewOf(io, false, &view, nullptr);
        bool holds = rigExec::eval::EnvelopeIsSparseWalk(view);
        size_t at = 0;
        for (size_t e = 0; holds && e < view.indexCount; ++e) {
            const int32_t index = view.indices[e];
            while (at + 1 < written->Size() && (*written)[at + 1] <= index)
                at += 2;
            holds = at + 1 < written->Size() && (*written)[at] <= index;
        }
        if (!holds)
            return Fail(error, "MatrixDeform",
                        "the weights reach a vertex group the revision does "
                        "not write");
    }
    if (packet && !state.pass) {
        const rigExec::eval::EnvelopePacketView view = PacketView(*packet);
        if (rigExec::eval::EnvelopeIsSparseWalk(view)) {
            if (!rigExec::eval::SparseEnvelopeIsUsable(view, points.Size()))
                return Fail(error, "MatrixDeform",
                            "the weight packet's sparse weights do not fit "
                            "the points");
        } else if (!rigExec::eval::EnvelopeIsFullStrength(view) &&
                   !ResolvePacket(*packet, points.Size(), &state.dense)) {
            return Fail(error, "MatrixDeform",
                        "the weight packet does not resolve over the points");
        }
    }
    rigExec::eval::EnvelopePacketView view;
    return !state.checkEachFrame || ViewOf(io, true, &view, error);
}

/// ApplyAcceptedMatrixKernel's points [begin, end): the sparse walk over
/// the entries naming them, or the dense or full-strength range.
bool
Deform(const rigExec::RigExecOpBatch &batch, size_t k,
       const rigExec::RigExecOpRange &range, std::string *)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto &state =
        *static_cast<const MatrixState *>(rigExec::RigExecBatchState(batch, k));
    const PointWeights *packet = In<PointWeights>(io, MatrixDeformPacket());
    rigExec::eval::EnvelopePacketView view;
    if (packet)
        view = PacketView(*packet);
    else
        ViewOf(io, false, &view, nullptr);
    const Vec3f *in = RangeIn<Vec3f>(io, range, MatrixDeformPoints());
    const Mat4d &transform = *In<Mat4d>(io, MatrixDeformTransform());
    const float *envelope = RangeIn<float>(io, range, MatrixDeformEnvelope());
    const uint8_t *blend = In<uint8_t>(io, MatrixDeformRadial());
    const bool radial =
        blend
            ? *blend != 0
            : rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kRadial)[k] != 0;
    Vec3f *out = RangeOut<Vec3f>(io, range, MatrixDeformOut());
    const size_t count = range.end - range.begin;
    if (state.pass) {
        std::copy(in, in + count, out);
        return true;
    }
    if (!state.dense.empty())
        envelope = state.dense.data() + range.begin;
    if (rigExec::eval::EnvelopeIsSparseWalk(view)) {
        std::copy(in, in + count, out);
        auto *memo =
            static_cast<rigExec::RigExecSparseWalkMemo<Vec3f> *>(range.scratch);
        if (memo) {
            memo->computed = 0;
            memo->reused = 0;
        }
        if (transform == Math::Identity())
            return true;
        const auto *first = view.indices;
        const auto *last = view.indices + view.indexCount;
        const size_t from =
            size_t(std::lower_bound(first, last, int(range.begin)) - first);
        const size_t to =
            size_t(std::lower_bound(first, last, int(range.end)) - first);
        if (from != to)
            rigExec::eval::ApplyMatrixSparseWalk<Math>(
                radial, transform, view, from, to, out, range.begin, memo);
        return true;
    }
    rigExec::eval::ApplyMatrixKernelRange<Math>(
        radial, transform,
        rigExec::eval::EnvelopeIsFullStrength(view) && state.dense.empty()
            ? nullptr
            : envelope,
        0, count, in, out, io.useSimd);
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

enum MoverTransformField : uint32_t { kPosedPoints };

bool
CheckMoverTransform(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (Fed(check, MoverTransformReferenceSpace()) &&
        !(Fed(check, MoverTransformSpace()) &&
          Fed(check, MoverTransformReference())))
        return Fail(error, "MoverTransform",
                    "a reference space needs Space and Reference");
    return true;
}

bool
MoverTransformOne(const rigExec::RigExecOpBatch &batch, size_t k)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    Out<Mat4d>(io, MoverTransformOut()) =
        rigExec::RigExecMatrixInPointFrameT<Vec3d>(
            *In<Mat4d>(io, MoverTransformTransform()),
            In<Mat4d>(io, MoverTransformSpace()),
            In<Mat4d>(io, MoverTransformReference()),
            In<Mat4d>(io, MoverTransformReferenceSpace()),
            FlagAt(batch, kPosedPoints, k),
            In<Mat4d>(io, MoverTransformCarry()));
    return true;
}

bool
RunMoverTransform(const rigExec::RigExecOpBatch &batch, size_t *failed,
                  std::string *)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return MoverTransformOne(batch, k); });
}

void
MoverTransformValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    const auto &p = *static_cast<const MoverTransformPayload *>(payload);
    SetFlag(out, kPosedPoints, p.posedPoints);
}

const rigExec::RigExecFieldDesc kMoverTransformFields[] = {
    Flag("posedPoints"),
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
     kRigExecPortInput | kRigExecPortOptional | kRigExecPortPointwise |
         rigExec::kRigExecPortChecked,
     "envelope"},
    {&MatrixDeformWeightIndices(), &rigExecStd::IntArray(),
     kRigExecPortInput | kRigExecPortOptional | rigExec::kRigExecPortChecked,
     "weightIndices"},
    {&MatrixDeformWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional | rigExec::kRigExecPortChecked,
     "weights"},
    {&MatrixDeformOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &MatrixDeformPoints()},
    {&MatrixDeformEnabled(), &rigExecStd::BoolType(),
     kRigExecPortInput | kRigExecPortOptional, "enabled"},
    {&MatrixDeformDefaultWeight(), &rigExecStd::FloatType(),
     kRigExecPortInput | kRigExecPortOptional, "defaultWeight"},
    {&MatrixDeformPacket(), &rigExecRig::PointWeightsType(),
     kRigExecPortInput | kRigExecPortOptional, "packet"},
    {&MatrixDeformRadial(), &rigExecStd::BoolType(),
     kRigExecPortInput | kRigExecPortOptional, "radial"},
    {&MatrixDeformWritten(), &rigExecStd::IntArray(),
     kRigExecPortInput | kRigExecPortOptional, "written"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "MatrixDeform";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.validate = &Validate;
    d.range.prepare = &Prepare;
    d.range.run = &Deform;
    d.state = StateOf<MatrixState>();
    d.range.scratch = StateOf<rigExec::RigExecSparseWalkMemo<Vec3f>>();
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable | rigExec::kRigExecOpRanged |
                     rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

const rigExec::RigExecPortDesc kMoverTransformPorts[] = {
    {&MoverTransformTransform(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "transform"},
    {&MoverTransformSpace(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "space"},
    {&MoverTransformReference(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "reference"},
    {&MoverTransformReferenceSpace(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "referenceSpace"},
    {&MoverTransformCarry(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "carry"},
    {&MoverTransformOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kMoverTransformDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "MoverTransform";
    d.schema = &internalOps::kMoverTransform;
    d.ports = kMoverTransformPorts;
    d.portCount = std::size(kMoverTransformPorts);
    d.runBatch = &RunMoverTransform;
    d.validate = &CheckMoverTransform;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kMoverTransformFields;
    d.payloadFieldCount = std::size(kMoverTransformFields);
    d.payloadVersion = 1;
    d.payloadValues = &MoverTransformValues;
    return d;
}();

} // namespace

void
RegisterMatrixDeformOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(MatrixDeform(), kDesc);
    registrar.Add(MoverTransform(), kMoverTransformDesc);
}

} // namespace rigExecOps
