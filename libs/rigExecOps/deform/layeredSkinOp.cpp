#include "rigExecOps/deform/layeredSkinOp.h"

#include "rigExecMath/layeredSkinKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecSchema/schemaNames.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(LayeredSkin)

RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinPoints)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinTransforms)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinIndices)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinWeights)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinMask)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinBase)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinInputTransform)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

enum LayeredSkinField : uint32_t { kElementSize, kDualQuaternion };

const Points kNoBase;
const Mat4d kIdentity(1.0);

bool
SkinOne(const rigExec::RigExecOpIo &io, int32_t elementSize,
        bool dualQuaternion, std::vector<Vec3f> *points, std::string *error)
{
    const auto &transforms =
        *In<RigExecArray<Mat4d>>(io, LayeredSkinTransforms());
    const auto &indices = *In<RigExecArray<int>>(io, LayeredSkinIndices());
    const Envelope &weights = *In<Envelope>(io, LayeredSkinWeights());
    const Envelope &mask = *In<Envelope>(io, LayeredSkinMask());
    const Points *fed = In<Points>(io, LayeredSkinBase());
    const Points &base = fed ? *fed : kNoBase;
    const size_t count = points->size();
    if (!rigExec::RigExecLayeredSkinAccepts(
            transforms.Data(), transforms.Size(), indices.Data(),
            indices.Size(), weights.Data(), weights.Size(), elementSize,
            mask.Data(), mask.Size(), count))
        return Fail(error, "LayeredSkin", "unusable skin layout or mask");
    const rigExec::RigExecSkinLayoutOf<Mat4d> layout{
        transforms.Data(), transforms.Size(),   indices.Data(), weights.Data(),
        indices.Size(),    size_t(elementSize), count};
    if (!rigExec::RigExecApplyLayeredSkinKernel<Vec3f, Vec3d, Mat4d>(
            layout, dualQuaternion, base.Data(), base.Size(), mask.Data(),
            mask.Size(),
            InOr<Mat4d>(io, LayeredSkinInputTransform(), kIdentity),
            points->data(), points->size()) ||
        !AllFinite(*points))
        return Fail(error, "LayeredSkin",
                    "the base does not fit, the blend is degenerate or a "
                    "point is not finite");
    return true;
}

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const int elementSize = rigExec::RigExecPayloadColumnOf<int32_t>(
        check.batch, kElementSize)[check.k];
    const size_t points = CheckElements(check, LayeredSkinPoints());
    const size_t indices = CheckElements(check, LayeredSkinIndices());
    const size_t mask = CheckElements(check, LayeredSkinMask());
    if (elementSize < 1 || CheckElements(check, LayeredSkinTransforms()) == 0 ||
        !SameCount(indices, CheckElements(check, LayeredSkinWeights())) ||
        (points < rigExec::kRigExecPortUnfed &&
         !SameCount(indices, points * size_t(elementSize))) ||
        (mask != 0 && !SameCount(mask, points)))
        return Fail(error, "LayeredSkin", "unusable skin layout or mask");
    const size_t base = CheckElements(check, LayeredSkinBase());
    if (base != 0 && !SameCount(base, points))
        return Fail(error, "LayeredSkin",
                    "the base does not fit, the blend is degenerate or a "
                    "point is not finite");
    return true;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &, size_t)
{
    return rigExec::RigExecRangePlan::Whole;
}

bool
RunRange(const rigExec::RigExecOpBatch &batch, size_t k,
         const rigExec::RigExecOpRange &range, std::string *error)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    std::vector<Vec3f> points;
    GatherWhole(io, range, LayeredSkinPoints(), &points);
    if (!SkinOne(
            io,
            rigExec::RigExecPayloadColumnOf<int32_t>(batch, kElementSize)[k],
            rigExec::RigExecPayloadColumnOf<uint8_t>(batch,
                                                     kDualQuaternion)[k] != 0,
            &points, error))
        return false;
    std::copy(points.begin() + std::ptrdiff_t(range.begin),
              points.begin() + std::ptrdiff_t(range.end),
              RangeOut<Vec3f>(io, range, LayeredSkinOut()));
    return true;
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const auto &p = *static_cast<const LayeredSkinPayload *>(payload);
    out.Set(kElementSize, p.elementSize);
    SetFlag(out, kDualQuaternion, p.dualQuaternion);
}

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
using authoring::Authored;

const bool kLinear = false;
const int32_t kOneInfluence = 1;
const rigExecStd::RigExecArray<int32_t> kNoInts;
const rigExecStd::RigExecArray<float> kNoFloats;
const char *const kMethods[] = {values::classicLinear, values::dualQuaternion};

const rigExec::RigExecPropertyInfo kInfluences =
    authoring::Relationship(names::rigExecInfluences);
const rigExec::RigExecPropertyInfo kJointIndices =
    authoring::Property(names::rigExecJointIndices, &kNoInts);
const rigExec::RigExecPropertyInfo kJointWeights =
    authoring::Property(names::rigExecJointWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kMask =
    authoring::Property(names::inputsMask, &kNoFloats);
const rigExec::RigExecPropertyInfo kTransform =
    authoring::Relationship(names::rigExecTransform);
const rigExec::RigExecPropertyInfo kElementSizeProperty =
    authoring::Uniform(names::rigExecElementSize, &kOneInfluence);
const rigExec::RigExecPropertyInfo kSkinningMethod =
    authoring::Tokens(names::rigExecSkinningMethod, &kLinear, kMethods);

const rigExec::RigExecSchemaInfo kLayeredSkinSchema{
    rigExec::SchemaTypeName::RigExecLayeredSkinMover, nullptr,
    authoring::kTyped, rigExec::RigExecSchemaMode::HandAuthored};

const rigExec::RigExecFieldDesc kFields[] = {
    Authored(Value("elementSize", rigExecStd::IntType()), kElementSizeProperty),
    Authored(Flag("dualQuaternion"), kSkinningMethod),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&LayeredSkinPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortNeighbourhood, "points"},
    Authored({&LayeredSkinTransforms(), &rigExecStd::Mat4dArray(),
              kRigExecPortInput, "transforms"},
             kInfluences),
    Authored({&LayeredSkinIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
              "jointIndices"},
             kJointIndices),
    Authored({&LayeredSkinWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "jointWeights"},
             kJointWeights),
    Authored({&LayeredSkinMask(), &rigExecStd::FloatArray(), kRigExecPortInput,
              "mask"},
             kMask),
    {&LayeredSkinBase(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | kRigExecPortOptional, "basePoints"},
    Authored({&LayeredSkinInputTransform(), &rigExecStd::Mat4dType(),
              kRigExecPortInput | kRigExecPortOptional, "inputTransform"},
             kTransform),
    {&LayeredSkinOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &LayeredSkinPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "LayeredSkin";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.range.plan = &Plan;
    d.range.run = &RunRange;
    d.validate = &Check;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpSeparateBlend |
                     rigExec::kRigExecOpFromLeaves | rigExec::kRigExecOpRanged;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    d.schema = &kLayeredSkinSchema;
    return d;
}();

} // namespace

void
RegisterLayeredSkinOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(LayeredSkin(), kDesc);
}

} // namespace rigExecOps
