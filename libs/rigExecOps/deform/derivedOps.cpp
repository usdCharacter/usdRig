#include "rigExecOps/deform/derivedOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/meshBoundsKernel.h"
#include "rigExecMath/normalsLayoutKernel.h"

#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(RecomputeNormals)
RIGEXEC_DEFINE_OP_HANDLE(RecomputeExtent)

RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsCounts)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsIndices)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsOut)

RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentWidths)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

// Writes \p values to \p port, blended from \p authored by \p envelope,
// as the derived revision kernel does.
bool
WriteDerived(const RigExecOpIo &io, const rigExec::RigExecPortHandle &port,
             std::vector<Vec3f> values, const Points *authored,
             const Envelope *envelope, const char *op, std::string *error)
{
    if (authored && authored->Size() != values.size())
        return Fail(error, op, "authored length differs from the result");
    if (envelope && !authored)
        return Fail(error, op, "an envelope needs the authored value");
    if (envelope) {
        if (envelope->Size() != values.size())
            return Fail(error, op, "envelope length differs from the result");
        for (size_t i = 0; i < values.size(); ++i)
            values[i] = rigExec::RigExecBlendEnvelope((*authored)[i], values[i],
                                                      (*envelope)[i]);
    }
    Out<Points>(io, port).Assign(values.data(), values.size());
    return true;
}

enum NormalsField : uint32_t { kPerCorner, kLeftHanded };

bool
NormalsOne(const RigExecOpIo &io, bool perCorner, bool leftHanded,
           std::string *error)
{
    const Points &points = *In<Points>(io, RecomputeNormalsPoints());
    const auto &counts = *In<RigExecArray<int>>(io, RecomputeNormalsCounts());
    const auto &indices = *In<RigExecArray<int>>(io, RecomputeNormalsIndices());
    if (points.Empty())
        return Fail(error, "RecomputeNormals", "no points");
    std::vector<Vec3f> normals = rigExec::RigExecComputeVertexNormalsKernel(
        points.Data(), points.Size(), counts.Data(), counts.Size(),
        indices.Data(), indices.Size());
    if (!rigExec::RigExecLayoutNormals(&normals, indices.Data(), indices.Size(),
                                       perCorner, leftHanded))
        return Fail(error, "RecomputeNormals", "a corner names no point");
    return WriteDerived(io, RecomputeNormalsOut(), std::move(normals),
                        In<Points>(io, RecomputeNormalsAuthored()),
                        In<Envelope>(io, RecomputeNormalsEnvelope()),
                        "RecomputeNormals", error);
}

bool
ExtentOne(const RigExecOpIo &io, std::string *error)
{
    const Points &points = *In<Points>(io, RecomputeExtentPoints());
    const auto *widths = In<RigExecArray<float>>(io, RecomputeExtentWidths());
    if (points.Empty())
        return Fail(error, "RecomputeExtent", "no points");
    return WriteDerived(
        io, RecomputeExtentOut(),
        rigExec::RigExecComputeExtentKernel(points.Data(), points.Size(),
                                            widths ? widths->Data() : nullptr,
                                            widths ? widths->Size() : 0),
        In<Points>(io, RecomputeExtentAuthored()),
        In<Envelope>(io, RecomputeExtentEnvelope()), "RecomputeExtent", error);
}

/// The authored value and envelope agree with a result of \p result
/// elements where the counts are known.
bool
CheckBlend(const rigExec::RigExecOpCheck &check,
           const rigExec::RigExecPortHandle &authored,
           const rigExec::RigExecPortHandle &envelope, size_t result,
           const char *op, std::string *error)
{
    if (Fed(check, envelope) && !Fed(check, authored))
        return Fail(error, op, "an envelope needs the authored value");
    if (Fed(check, authored) &&
        !SameCount(CheckElements(check, authored), result))
        return Fail(error, op, "authored length differs from the result");
    if (Fed(check, envelope) &&
        !SameCount(CheckElements(check, envelope), result))
        return Fail(error, op, "envelope length differs from the result");
    return true;
}

bool
CheckNormals(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t points = CheckElements(check, RecomputeNormalsPoints());
    if (points == 0)
        return Fail(error, "RecomputeNormals", "no points");
    const bool perCorner = rigExec::RigExecPayloadColumnOf<uint8_t>(
                               check.batch, kPerCorner)[check.k] != 0;
    if (const auto *indices =
            CheckValue<RigExecArray<int>>(check, RecomputeNormalsIndices());
        perCorner && indices && points < rigExec::kRigExecPortUnfed)
        for (const int index : *indices)
            if (index < 0 || size_t(index) >= points)
                return Fail(error, "RecomputeNormals",
                            "a corner names no point");
    return CheckBlend(
        check, RecomputeNormalsAuthored(), RecomputeNormalsEnvelope(),
        CheckElements(check, perCorner ? RecomputeNormalsIndices()
                                       : RecomputeNormalsPoints()),
        "RecomputeNormals", error);
}

bool
CheckExtent(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (CheckElements(check, RecomputeExtentPoints()) == 0)
        return Fail(error, "RecomputeExtent", "no points");
    return CheckBlend(check, RecomputeExtentAuthored(),
                      RecomputeExtentEnvelope(), 2, "RecomputeExtent", error);
}

bool
RunNormals(const rigExec::RigExecOpBatch &batch, size_t *failed,
           std::string *error)
{
    const uint8_t *perCorner =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kPerCorner);
    const uint8_t *leftHanded =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kLeftHanded);
    return EachInstance(batch, failed, [&](size_t k) {
        return NormalsOne(rigExec::RigExecBatchInstance(batch, k),
                          perCorner[k] != 0, leftHanded[k] != 0, error);
    });
}

bool
RunExtent(const rigExec::RigExecOpBatch &batch, size_t *failed,
          std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ExtentOne(rigExec::RigExecBatchInstance(batch, k), error);
    });
}

void
NormalsValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    const NormalsPayload &p = *static_cast<const NormalsPayload *>(payload);
    SetFlag(out, kPerCorner, p.perCorner);
    SetFlag(out, kLeftHanded, p.leftHanded);
}

const rigExec::RigExecFieldDesc kNormalsFields[] = {
    Flag("perCorner"),
    Flag("leftHanded"),
};

rigExec::RigExecOpDesc
DerivedDesc(const char *name, const rigExec::RigExecPortDesc *ports,
            size_t portCount, rigExec::RigExecOpBatchFn runBatch,
            bool (*validate)(const rigExec::RigExecOpCheck &, std::string *),
            uint32_t capabilities)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.validate = validate;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpFromLeaves | capabilities;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kNormalsPorts[] = {
    {&RecomputeNormalsPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&RecomputeNormalsCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&RecomputeNormalsIndices(), &rigExecStd::IntArray(),
     kRigExecPortInput | rigExec::kRigExecPortChecked, "faceVertexIndices"},
    {&RecomputeNormalsAuthored(), &rigExecStd::Vec3fArray(), kOptional,
     "authored"},
    {&RecomputeNormalsEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&RecomputeNormalsOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &RecomputeNormalsPoints(),
     &RecomputeNormalsIndices()},
};

const rigExec::RigExecPortDesc kExtentPorts[] = {
    {&RecomputeExtentPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&RecomputeExtentWidths(), &rigExecStd::FloatArray(), kOptional, "widths"},
    {&RecomputeExtentAuthored(), &rigExecStd::Vec3fArray(), kOptional,
     "authored"},
    {&RecomputeExtentEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&RecomputeExtentOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Fixed, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 2},
};

const rigExec::RigExecOpDesc kNormalsDesc = [] {
    rigExec::RigExecOpDesc d =
        DerivedDesc("RecomputeNormals", kNormalsPorts, std::size(kNormalsPorts),
                    &RunNormals, &CheckNormals, 0);
    d.payloadFields = kNormalsFields;
    d.payloadFieldCount = std::size(kNormalsFields);
    d.payloadVersion = 1;
    d.payloadValues = &NormalsValues;
    return d;
}();

const rigExec::RigExecOpDesc kExtentDesc =
    DerivedDesc("RecomputeExtent", kExtentPorts, std::size(kExtentPorts),
                &RunExtent, &CheckExtent, 0);

} // namespace

void
RegisterDerivedOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(RecomputeNormals(), kNormalsDesc);
    registrar.Add(RecomputeExtent(), kExtentDesc);
}

} // namespace rigExecOps
