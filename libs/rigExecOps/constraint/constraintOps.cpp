#include "rigExecOps/constraint/constraintOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/constraintKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(PositionConstraint)
RIGEXEC_DEFINE_OP_HANDLE(RotationConstraint)
RIGEXEC_DEFINE_OP_HANDLE(ScaleConstraint)
RIGEXEC_DEFINE_OP_HANDLE(ParentConstraint)
RIGEXEC_DEFINE_OP_HANDLE(AimConstraint)

RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintOffset)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintAffect)
RIGEXEC_DEFINE_PORT_HANDLE(PositionConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintSpace)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintOffset)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintAffect)
RIGEXEC_DEFINE_PORT_HANDLE(RotationConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintOffset)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintAffect)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSourceWeights)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintSpace)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintTranslationAxes)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintRotationAxes)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintScaleAxes)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintTranslationOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintRotationOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(ParentConstraintOut)

RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintIn)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintTarget)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintWorldUp)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintWeight)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintOffset)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintAffect)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintAimVector)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintUpVector)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintOut)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintSources)
RIGEXEC_DEFINE_PORT_HANDLE(AimConstraintSourceWeights)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

/// The constraint payload's declared fields, in saved order.
enum ConstraintField : uint32_t {
    kRotationOrder,
    kBlendShear,
    kPreserveInputUp,
};

/// One instance's ConstraintPayload.
struct ConstraintSettings {
    ConstraintEulerOrder rotationOrder = ConstraintEulerOrder::XYZ;
    bool blendShear = false;
    bool preserveInputUp = true;
};

ConstraintSettings
SettingsAt(const RigExecOpBatch &batch, size_t k)
{
    ConstraintSettings p;
    p.rotationOrder = EnumAt<ConstraintEulerOrder>(batch, kRotationOrder, k);
    p.blendShear = FlagAt(batch, kBlendShear, k);
    p.preserveInputUp = FlagAt(batch, kPreserveInputUp, k);
    return p;
}

using Flags = RigExecArray<uint8_t>;
using Vectors = RigExecArray<Vec3d>;

/// The axis mask fed to \p port, or \p fallback unfed; false when the fed
/// mask is not three flags.
bool
AxesOf(const RigExecOpIo &io, const RigExecPortHandle &port,
       const ConstraintAxes &fallback, ConstraintAxes *axes)
{
    const Flags *flags = In<Flags>(io, port);
    if (!flags) {
        *axes = fallback;
        return true;
    }
    if (flags->Size() != 3)
        return false;
    *axes = {(*flags)[0] != 0, (*flags)[1] != 0, (*flags)[2] != 0};
    return true;
}

bool
Finite(const Vec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
Finite(const PointFrame &frame)
{
    for (const Vec3d &point : frame.points)
        if (!Finite(point))
            return false;
    return true;
}

bool
Finite(const Mat4d &m)
{
    for (size_t r = 0; r < 4; ++r)
        for (size_t c = 0; c < 4; ++c)
            if (!std::isfinite(m[r][c]))
                return false;
    return true;
}

/// The input frame, weight and optional space every kind reads are finite.
/// Past this and the sources, a frame the kernel cannot solve is a
/// geometric degeneracy, not bad input.
bool
CheckInputs(const char *op, const PointFrame &in, double weight,
            const Mat4d *space, std::string *error)
{
    if (!Finite(in) || !std::isfinite(weight) || (space && !Finite(*space)))
        return Fail(error, op, "has a non-finite input");
    return true;
}

/// The mask and offset Position, Rotation, Scale and Aim read.
bool
MaskAndOffset(const char *op, const RigExecOpIo &io,
              const RigExecPortHandle &affectPort,
              const RigExecPortHandle &offsetPort, ConstraintAxes *affect,
              Vec3d *offset, std::string *error)
{
    if (!AxesOf(io, affectPort, ConstraintAxes(), affect))
        return Fail(error, op, "needs three affect flags");
    *offset = InOr(io, offsetPort, Vec3d(0.0, 0.0, 0.0));
    if (!Finite(*offset))
        return Fail(error, op, "has a non-finite offset");
    return true;
}

struct Source {
    PointFrame frame;
    double normalizedWeight = 1.0;
    Vec3d translationOffset{0.0, 0.0, 0.0};
    Vec3d rotationOffsetDegrees{0.0, 0.0, 0.0};
};

struct PositionParams {
    Vec3d offset;
    ConstraintAxes affect;
    double weight;
};

struct RotationParams {
    Vec3d offsetDegrees;
    ConstraintAxes affect;
    ConstraintEulerOrder rotationOrder;
    double weight;
    const Mat4d *carry;
};

struct ScaleParams {
    Vec3d offset;
    ConstraintAxes affect;
    double weight;
    bool blendShear;
};

struct ParentParams {
    ConstraintAxes translationAxes;
    ConstraintAxes rotationAxes;
    ConstraintAxes scaleAxes;
    ConstraintEulerOrder rotationOrder;
    double weight;
    bool blendShear;
    const Mat4d *carry;
};

struct AimParams {
    Vec3d localAimVector;
    Vec3d localUpVector;
    const Vec3d *worldUpDirection;
    bool preserveInputUp;
    Vec3d rotationOffsetDegrees;
    ConstraintAxes affectRotation;
    ConstraintEulerOrder rotationOrder;
    double weight;
};

/// Parent's per-source offset ports; null for the other kinds.
struct OffsetPorts {
    const RigExecPortHandle *translations = nullptr;
    const RigExecPortHandle *rotations = nullptr;
};

/// The sources with their weights and Parent offsets, or false with
/// \p error set when a parallel array has the wrong length, a weight is
/// negative or not finite, or a weighed source or an offset is not finite.
bool
GatherSources(const char *op, const RigExecOpIo &io,
              const RigExecPortHandle &sourcesPort,
              const RigExecPortHandle &weightsPort, const OffsetPorts &offsets,
              std::vector<Source> *sources, std::string *error)
{
    const auto &frames = In<PointFrameArray>(io, sourcesPort)->frames;
    const auto *weights = In<RigExecArray<double>>(io, weightsPort);
    const Vectors *translations =
        offsets.translations ? In<Vectors>(io, *offsets.translations) : nullptr;
    const Vectors *rotations =
        offsets.rotations ? In<Vectors>(io, *offsets.rotations) : nullptr;
    const size_t n = frames.Size();
    if (weights && weights->Size() != n)
        return Fail(error, op, "needs one source weight per source");
    if ((translations && translations->Size() != n) ||
        (rotations && rotations->Size() != n))
        return Fail(error, op, "needs no offsets or one per source");
    sources->resize(n);
    for (size_t i = 0; i < n; ++i) {
        Source &s = (*sources)[i];
        s.frame = frames[i];
        s.normalizedWeight = weights ? (*weights)[i] : 1.0;
        if (translations)
            s.translationOffset = (*translations)[i];
        if (rotations)
            s.rotationOffsetDegrees = (*rotations)[i];
        if (!std::isfinite(s.normalizedWeight) || s.normalizedWeight < 0.0)
            return Fail(error, op, "has an invalid source weight");
        if ((s.normalizedWeight != 0.0 && !Finite(s.frame)) ||
            !Finite(s.translationOffset) || !Finite(s.rotationOffsetDegrees))
            return Fail(error, op, "has a non-finite source");
    }
    return true;
}

/// A fed axis mask holds three flags where its count is known.
bool
CheckAxes(const rigExec::RigExecOpCheck &check, const char *op,
          const RigExecPortHandle &port, std::string *error)
{
    if (Fed(check, port) && !SameCount(CheckElements(check, port), 3))
        return Fail(error, op, "needs three affect flags");
    return true;
}

/// The source weights, and Parent's offsets, agree with the sources where
/// their counts are known before the node runs.
bool
CheckSources(const rigExec::RigExecOpCheck &check, const char *op,
             const RigExecPortHandle &sourcesPort,
             const RigExecPortHandle &weightsPort, const OffsetPorts &offsets,
             std::string *error)
{
    const size_t n = CheckElements(check, sourcesPort);
    if (Fed(check, weightsPort) &&
        !SameCount(CheckElements(check, weightsPort), n))
        return Fail(error, op, "needs one source weight per source");
    for (const RigExecPortHandle *port :
         {offsets.translations, offsets.rotations})
        if (port && Fed(check, *port) &&
            !SameCount(CheckElements(check, *port), n))
            return Fail(error, op, "needs no offsets or one per source");
    return true;
}

bool
CheckPosition(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckAxes(check, "PositionConstraint", PositionConstraintAffect(),
                     error) &&
           CheckSources(check, "PositionConstraint",
                        PositionConstraintSources(),
                        PositionConstraintSourceWeights(), {}, error);
}

bool
CheckRotation(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckAxes(check, "RotationConstraint", RotationConstraintAffect(),
                     error) &&
           CheckSources(check, "RotationConstraint",
                        RotationConstraintSources(),
                        RotationConstraintSourceWeights(), {}, error);
}

bool
CheckScale(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckAxes(check, "ScaleConstraint", ScaleConstraintAffect(),
                     error) &&
           CheckSources(check, "ScaleConstraint", ScaleConstraintSources(),
                        ScaleConstraintSourceWeights(), {}, error);
}

OffsetPorts
ParentOffsets()
{
    return {&ParentConstraintTranslationOffsets(),
            &ParentConstraintRotationOffsets()};
}

bool
CheckParent(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckAxes(check, "ParentConstraint",
                     ParentConstraintTranslationAxes(), error) &&
           CheckAxes(check, "ParentConstraint", ParentConstraintRotationAxes(),
                     error) &&
           CheckAxes(check, "ParentConstraint", ParentConstraintScaleAxes(),
                     error) &&
           CheckSources(check, "ParentConstraint", ParentConstraintSources(),
                        ParentConstraintSourceWeights(), ParentOffsets(),
                        error);
}

bool
CheckAim(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (Fed(check, AimConstraintTarget()) == Fed(check, AimConstraintSources()))
        return Fail(error, "AimConstraint",
                    "needs a target or sources, not both");
    return CheckAxes(check, "AimConstraint", AimConstraintAffect(), error) &&
           CheckSources(check, "AimConstraint", AimConstraintSources(),
                        AimConstraintSourceWeights(), {}, error);
}

/// The kernel's frame: a constraint that cannot solve gives the input flagged
/// degenerate.
void
Commit(const PointFrame &, const PointFrame &solved, PointFrame *out)
{
    *out = solved;
}

bool
PositionOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    std::vector<Source> sources;
    PositionParams params{{}, {}, *In<double>(io, PositionConstraintWeight())};
    if (!MaskAndOffset("PositionConstraint", io, PositionConstraintAffect(),
                       PositionConstraintOffset(), &params.affect,
                       &params.offset, error) ||
        !GatherSources("PositionConstraint", io, PositionConstraintSources(),
                       PositionConstraintSourceWeights(), {}, &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, PositionConstraintIn());
    if (!CheckInputs("PositionConstraint", in, params.weight, nullptr, error))
        return false;
    Commit(in,
           rigExec::RigExecApplyPositionConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, PositionConstraintOut()));
    return true;
}

bool
RunPosition(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return PositionOne(batch, k, error); });
}

bool
RotationOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const ConstraintSettings p = SettingsAt(batch, k);
    std::vector<Source> sources;
    RotationParams params{{},
                          {},
                          p.rotationOrder,
                          *In<double>(io, RotationConstraintWeight()),
                          In<Mat4d>(io, RotationConstraintSpace())};
    if (!MaskAndOffset("RotationConstraint", io, RotationConstraintAffect(),
                       RotationConstraintOffset(), &params.affect,
                       &params.offsetDegrees, error) ||
        !GatherSources("RotationConstraint", io, RotationConstraintSources(),
                       RotationConstraintSourceWeights(), {}, &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, RotationConstraintIn());
    if (!CheckInputs("RotationConstraint", in, params.weight, params.carry,
                     error))
        return false;
    Commit(in,
           rigExec::RigExecApplyRotationConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, RotationConstraintOut()));
    return true;
}

bool
RunRotation(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return RotationOne(batch, k, error); });
}

bool
ScaleOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const ConstraintSettings p = SettingsAt(batch, k);
    std::vector<Source> sources;
    ScaleParams params{
        {}, {}, *In<double>(io, ScaleConstraintWeight()), p.blendShear};
    if (!MaskAndOffset("ScaleConstraint", io, ScaleConstraintAffect(),
                       ScaleConstraintOffset(), &params.affect, &params.offset,
                       error) ||
        !GatherSources("ScaleConstraint", io, ScaleConstraintSources(),
                       ScaleConstraintSourceWeights(), {}, &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, ScaleConstraintIn());
    if (!CheckInputs("ScaleConstraint", in, params.weight, nullptr, error))
        return false;
    Commit(in,
           rigExec::RigExecApplyScaleConstraintKernel<Vec3f, Vec3d>(in, sources,
                                                                    params),
           &Out<PointFrame>(io, ScaleConstraintOut()));
    return true;
}

bool
RunScale(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return ScaleOne(batch, k, error); });
}

bool
ParentOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const ConstraintSettings p = SettingsAt(batch, k);
    std::vector<Source> sources;
    ParentParams params{{},
                        {},
                        {false, false, false},
                        p.rotationOrder,
                        *In<double>(io, ParentConstraintWeight()),
                        p.blendShear,
                        In<Mat4d>(io, ParentConstraintSpace())};
    if (!AxesOf(io, ParentConstraintTranslationAxes(), ConstraintAxes(),
                &params.translationAxes) ||
        !AxesOf(io, ParentConstraintRotationAxes(), ConstraintAxes(),
                &params.rotationAxes) ||
        !AxesOf(io, ParentConstraintScaleAxes(), {false, false, false},
                &params.scaleAxes))
        return Fail(error, "ParentConstraint", "needs three affect flags");
    if (!GatherSources("ParentConstraint", io, ParentConstraintSources(),
                       ParentConstraintSourceWeights(), ParentOffsets(),
                       &sources, error))
        return false;
    const PointFrame &in = *In<PointFrame>(io, ParentConstraintIn());
    if (!CheckInputs("ParentConstraint", in, params.weight, params.carry,
                     error))
        return false;
    Commit(in,
           rigExec::RigExecApplyParentConstraintKernel<Vec3f, Vec3d>(
               in, sources, params),
           &Out<PointFrame>(io, ParentConstraintOut()));
    return true;
}

bool
RunParent(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return ParentOne(batch, k, error); });
}

bool
AimOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const ConstraintSettings p = SettingsAt(batch, k);
    const PointFrame &in = *In<PointFrame>(io, AimConstraintIn());
    AimParams params{InOr(io, AimConstraintAimVector(), Vec3d(1.0, 0.0, 0.0)),
                     InOr(io, AimConstraintUpVector(), Vec3d(0.0, 1.0, 0.0)),
                     In<Vec3d>(io, AimConstraintWorldUp()),
                     p.preserveInputUp,
                     {},
                     {},
                     p.rotationOrder,
                     *In<double>(io, AimConstraintWeight())};
    if (!MaskAndOffset("AimConstraint", io, AimConstraintAffect(),
                       AimConstraintOffset(), &params.affectRotation,
                       &params.rotationOffsetDegrees, error) ||
        !CheckInputs("AimConstraint", in, params.weight, nullptr, error))
        return false;
    if (!Finite(params.localAimVector) || !Finite(params.localUpVector) ||
        (params.worldUpDirection && !Finite(*params.worldUpDirection)))
        return Fail(error, "AimConstraint", "has a non-finite input");
    const Vec3d *fed = In<Vec3d>(io, AimConstraintTarget());
    Vec3d target = fed ? *fed : Vec3d(0.0, 0.0, 0.0);
    if (fed && !Finite(target))
        return Fail(error, "AimConstraint", "has a non-finite target");
    if (!fed) {
        std::vector<Source> sources;
        if (!GatherSources("AimConstraint", io, AimConstraintSources(),
                           AimConstraintSourceWeights(), {}, &sources, error))
            return false;
        double total = 0.0;
        bool usable = true;
        for (const Source &source : sources) {
            const double w = source.normalizedWeight;
            if (w == 0.0)
                continue;
            usable = usable && (source.frame.flags &
                                rigExec::constraintDetail::kFrameValid);
            for (size_t a = 0; a < 3; ++a)
                target[a] = target[a] + source.frame.points[0][a] * w;
            total += w;
        }
        if (!(total > 0)) {
            Out<PointFrame>(io, AimConstraintOut()) = in;
            return true;
        }
        if (!usable) {
            PointFrame &out = Out<PointFrame>(io, AimConstraintOut());
            out = in;
            out.flags |= rigExec::constraintDetail::kFrameDegenerate;
            return true;
        }
        const double scale = 1.0 / total;
        for (size_t a = 0; a < 3; ++a)
            target[a] = target[a] * scale;
    }
    Commit(in,
           rigExec::RigExecApplyAimConstraintKernel<Vec3f, Vec3d>(in, target,
                                                                  params),
           &Out<PointFrame>(io, AimConstraintOut()));
    return true;
}

bool
RunAim(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return AimOne(batch, k, error); });
}

void
ConstraintValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const ConstraintPayload *>(payload);
    SetEnum(out, kRotationOrder, p.rotationOrder);
    SetFlag(out, kBlendShear, p.blendShear);
    SetFlag(out, kPreserveInputUp, p.preserveInputUp);
}

const rigExec::RigExecFieldDesc kConstraintFields[] = {
    Enum("rotationOrder", ConstraintEulerOrder::ZYX),
    Flag("blendShear"),
    Flag("preserveInputUp"),
};

rigExec::RigExecOpDesc
ConstraintDesc(const char *name, const rigExec::RigExecPortDesc *ports,
               size_t portCount, rigExec::RigExecOpBatchFn runBatch,
               bool (*validate)(const rigExec::RigExecOpCheck &, std::string *))
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.validate = validate;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kConstraintFields;
    d.payloadFieldCount = std::size(kConstraintFields);
    d.payloadVersion = 2;
    d.payloadValues = &ConstraintValues;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPositionPorts[] = {
    {&PositionConstraintIn(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "in"},
    {&PositionConstraintSources(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "sources"},
    {&PositionConstraintSourceWeights(), &rigExecStd::DoubleArray(), kOptional,
     "sourceWeights"},
    {&PositionConstraintWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&PositionConstraintOffset(), &rigExecStd::Vec3dType(), kOptional,
     "offset"},
    {&PositionConstraintAffect(), &rigExecStd::BoolArray(), kOptional,
     "affect"},
    {&PositionConstraintOut(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kRotationPorts[] = {
    {&RotationConstraintIn(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "in"},
    {&RotationConstraintSources(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "sources"},
    {&RotationConstraintSourceWeights(), &rigExecStd::DoubleArray(), kOptional,
     "sourceWeights"},
    {&RotationConstraintWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&RotationConstraintSpace(), &rigExecStd::Mat4dType(), kOptional, "space"},
    {&RotationConstraintOffset(), &rigExecStd::Vec3dType(), kOptional,
     "offset"},
    {&RotationConstraintAffect(), &rigExecStd::BoolArray(), kOptional,
     "affect"},
    {&RotationConstraintOut(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kScalePorts[] = {
    {&ScaleConstraintIn(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "in"},
    {&ScaleConstraintSources(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "sources"},
    {&ScaleConstraintSourceWeights(), &rigExecStd::DoubleArray(), kOptional,
     "sourceWeights"},
    {&ScaleConstraintWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&ScaleConstraintOffset(), &rigExecStd::Vec3dType(), kOptional, "offset"},
    {&ScaleConstraintAffect(), &rigExecStd::BoolArray(), kOptional, "affect"},
    {&ScaleConstraintOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kParentPorts[] = {
    {&ParentConstraintIn(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "in"},
    {&ParentConstraintSources(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "sources"},
    {&ParentConstraintSourceWeights(), &rigExecStd::DoubleArray(), kOptional,
     "sourceWeights"},
    {&ParentConstraintWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&ParentConstraintSpace(), &rigExecStd::Mat4dType(), kOptional, "space"},
    {&ParentConstraintTranslationAxes(), &rigExecStd::BoolArray(), kOptional,
     "translationAxes"},
    {&ParentConstraintRotationAxes(), &rigExecStd::BoolArray(), kOptional,
     "rotationAxes"},
    {&ParentConstraintScaleAxes(), &rigExecStd::BoolArray(), kOptional,
     "scaleAxes"},
    {&ParentConstraintTranslationOffsets(), &rigExecStd::Vec3dArray(),
     kOptional, "translationOffsets"},
    {&ParentConstraintRotationOffsets(), &rigExecStd::Vec3dArray(), kOptional,
     "rotationOffsets"},
    {&ParentConstraintOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kAimPorts[] = {
    {&AimConstraintIn(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "in"},
    {&AimConstraintTarget(), &rigExecStd::Vec3dType(), kOptional, "target"},
    {&AimConstraintWorldUp(), &rigExecStd::Vec3dType(), kOptional, "worldUp"},
    {&AimConstraintWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&AimConstraintOffset(), &rigExecStd::Vec3dType(), kOptional, "offset"},
    {&AimConstraintAffect(), &rigExecStd::BoolArray(), kOptional, "affect"},
    {&AimConstraintAimVector(), &rigExecStd::Vec3dType(), kOptional,
     "aimVector"},
    {&AimConstraintUpVector(), &rigExecStd::Vec3dType(), kOptional, "upVector"},
    {&AimConstraintOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "out"},
    {&AimConstraintSources(), &rigExecRig::PointFrameArrayType(), kOptional,
     "sources"},
    {&AimConstraintSourceWeights(), &rigExecStd::DoubleArray(), kOptional,
     "sourceWeights"},
};

const rigExec::RigExecOpDesc kPositionDesc =
    ConstraintDesc("PositionConstraint", kPositionPorts,
                   std::size(kPositionPorts), &RunPosition, &CheckPosition);

const rigExec::RigExecOpDesc kRotationDesc =
    ConstraintDesc("RotationConstraint", kRotationPorts,
                   std::size(kRotationPorts), &RunRotation, &CheckRotation);

const rigExec::RigExecOpDesc kScaleDesc =
    ConstraintDesc("ScaleConstraint", kScalePorts, std::size(kScalePorts),
                   &RunScale, &CheckScale);

const rigExec::RigExecOpDesc kParentDesc =
    ConstraintDesc("ParentConstraint", kParentPorts, std::size(kParentPorts),
                   &RunParent, &CheckParent);

const rigExec::RigExecOpDesc kAimDesc = ConstraintDesc(
    "AimConstraint", kAimPorts, std::size(kAimPorts), &RunAim, &CheckAim);

} // namespace

void
RegisterConstraintOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(PositionConstraint(), kPositionDesc);
    registrar.Add(RotationConstraint(), kRotationDesc);
    registrar.Add(ScaleConstraint(), kScaleDesc);
    registrar.Add(ParentConstraint(), kParentDesc);
    registrar.Add(AimConstraint(), kAimDesc);
}

} // namespace rigExecOps
