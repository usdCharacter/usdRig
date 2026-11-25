#include "rigExecOps/constraint/autoClavicleOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <cmath>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(AutoClavicle)

RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleTarget)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClaviclePivot)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleAnchor)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleAnchorDefault)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleFkPosed)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleFkDefault)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleIkTarget)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClaviclePole)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleIkBlend)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleAmount)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleLimbRests)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleStretch)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClaviclePin)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleUpperScale)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleLowerScale)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleSoftDistance)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleScaleCalibration)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleTwist)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleUpperOffset)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleLowerOffset)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleDelta)
RIGEXEC_DEFINE_PORT_HANDLE(AutoClavicleOut)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using namespace opIo;
using namespace fieldIo;

/// The auto clavicle payload's declared fields, in saved order.
enum ClavicleField : uint32_t {
    kBasis,
    kIkValue,
    kGain,
    kKernel,
    kNormalize,
    kSwingCount,
    kSwings,
    kWidthCount,
    kWidths,
    kGainCount,
    kGains,
    kWeightCount,
    kWeights,
    kHasLimb,
};

/// Instance \p k's constants, read in place.
rigExec::RigExecAutoClavicleConstantsView
ConstantsAt(const RigExecOpBatch &batch, size_t k)
{
    rigExec::RigExecAutoClavicleConstantsView c;
    c.basis = rigExec::RigExecPayloadField<double>(batch, kBasis, k).data;
    c.ikValue = ValueAt<double>(batch, kIkValue, k);
    c.gain = ValueAt<double>(batch, kGain, k);
    c.kernel = ValueAt<int32_t>(batch, kKernel, k);
    c.normalize = FlagAt(batch, kNormalize, k);
    const auto swings = ListAt<double>(batch, kSwings, k);
    const auto widths = ListAt<double>(batch, kWidths, k);
    const auto gains = ListAt<double>(batch, kGains, k);
    const auto weights = ListAt<double>(batch, kWeights, k);
    c.swings = swings.data;
    c.swingCount = swings.Size();
    c.widths = widths.data;
    c.widthCount = widths.Size();
    c.gains = gains.data;
    c.gainCount = gains.Size();
    c.weights = weights.data;
    c.weightCount = weights.Size();
    return c;
}

const double *
Rows(const Mat4d *m)
{
    return m ? &m->_mtx[0][0] : nullptr;
}

/// The distance between two frames' origins, as GfVec3d::GetLength
/// measures it.
double
OriginDistance(const Mat4d &a, const Mat4d &b)
{
    const double x = b._mtx[3][0] - a._mtx[3][0],
                 y = b._mtx[3][1] - a._mtx[3][1],
                 z = b._mtx[3][2] - a._mtx[3][2];
    return std::sqrt(x * x + y * y + z * z);
}

bool
CheckAutoClavicle(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, AutoClavicleFkDefault()), 3))
        return Fail(error, "AutoClavicle", "needs three FK default frames");
    if (!FlagAt(check.batch, kHasLimb, check.k))
        return true;
    if (!Fed(check, AutoClavicleLimbRests()) ||
        !SameCount(CheckElements(check, AutoClavicleLimbRests()), 3))
        return Fail(error, "AutoClavicle", "needs three limb rest frames");
    return true;
}

bool
AutoClavicleOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const struct {
        rigExec::RigExecAutoClavicleConstantsView constants;
        bool hasLimb;
    } p = {ConstantsAt(batch, k), FlagAt(batch, kHasLimb, k)};
    const Mat4d &target = *In<Mat4d>(io, AutoClavicleTarget());
    const auto &fkDefault =
        *In<RigExecArray<Mat4d>>(io, AutoClavicleFkDefault());
    if (fkDefault.Size() != 3)
        return Fail(error, "AutoClavicle", "needs three FK default frames");
    rigExec::RigExecAutoClavicleFrames f;
    f.targetPosed = Rows(&target);
    f.pivotPosed = Rows(In<Mat4d>(io, AutoClaviclePivot()));
    f.anchorPosed = Rows(In<Mat4d>(io, AutoClavicleAnchor()));
    f.anchorDefault = Rows(In<Mat4d>(io, AutoClavicleAnchorDefault()));
    f.fkPosed = Rows(In<Mat4d>(io, AutoClavicleFkPosed()));
    for (size_t i = 0; i < 3; ++i)
        f.fkDefault[i] = Rows(&fkDefault[i]);
    f.ikTargetPosed = Rows(In<Mat4d>(io, AutoClavicleIkTarget()));
    f.polePosed = Rows(In<Mat4d>(io, AutoClaviclePole()));
    f.ikBlend = *In<double>(io, AutoClavicleIkBlend());
    f.amount = *In<double>(io, AutoClavicleAmount());
    f.hasLimb = p.hasLimb;
    if (p.hasLimb) {
        const auto *rests =
            In<RigExecArray<Mat4d>>(io, AutoClavicleLimbRests());
        if (!rests || rests->Size() != 3)
            return Fail(error, "AutoClavicle", "needs three limb rest frames");
        f.limb.stretch = InOr(io, AutoClavicleStretch(), 1.0);
        f.limb.pin = InOr(io, AutoClaviclePin(), 0.0);
        f.limb.upperScale = InOr(io, AutoClavicleUpperScale(), 1.0);
        f.limb.lowerScale = InOr(io, AutoClavicleLowerScale(), 1.0);
        f.limb.softDistance = InOr(io, AutoClavicleSoftDistance(), 0.0);
        f.limb.scaleCalibration = InOr(io, AutoClavicleScaleCalibration(), 0.0);
        f.twistRadians =
            InOr(io, AutoClavicleTwist(), 0.0) * std::acos(-1.0) / 180;
        f.limbRestUpper = OriginDistance((*rests)[0], (*rests)[1]) +
                          InOr(io, AutoClavicleUpperOffset(), 0.0);
        f.limbRestLower = OriginDistance((*rests)[1], (*rests)[2]) +
                          InOr(io, AutoClavicleLowerOffset(), 0.0);
    }
    double delta[3];
    rigExec::RigExecAutoClavicleShift(p.constants, f, delta);
    Out<Vec3d>(io, AutoClavicleDelta()) = Vec3d(delta[0], delta[1], delta[2]);
    Mat4d &out = Out<Mat4d>(io, AutoClavicleOut());
    out = target;
    for (int k = 0; k < 3; ++k)
        out._mtx[3][k] = target._mtx[3][k] + delta[k];
    return true;
}

bool
RunAutoClavicle(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return AutoClavicleOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&AutoClavicleTarget(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "target"},
    {&AutoClaviclePivot(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "pivot"},
    {&AutoClavicleAnchor(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "anchor"},
    {&AutoClavicleAnchorDefault(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "anchorDefault"},
    {&AutoClavicleFkPosed(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "fkPosed"},
    {&AutoClavicleFkDefault(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "fkDefault"},
    {&AutoClavicleIkTarget(), &rigExecStd::Mat4dType(), kOptional, "ikTarget"},
    {&AutoClaviclePole(), &rigExecStd::Mat4dType(), kOptional, "pole"},
    {&AutoClavicleIkBlend(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "ikBlend"},
    {&AutoClavicleAmount(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "amount"},
    {&AutoClavicleLimbRests(), &rigExecStd::Mat4dArray(), kOptional,
     "limbRests"},
    {&AutoClavicleStretch(), &rigExecStd::DoubleType(), kOptional, "stretch"},
    {&AutoClaviclePin(), &rigExecStd::DoubleType(), kOptional, "pin"},
    {&AutoClavicleUpperScale(), &rigExecStd::DoubleType(), kOptional,
     "upperScale"},
    {&AutoClavicleLowerScale(), &rigExecStd::DoubleType(), kOptional,
     "lowerScale"},
    {&AutoClavicleSoftDistance(), &rigExecStd::DoubleType(), kOptional,
     "softDistance"},
    {&AutoClavicleScaleCalibration(), &rigExecStd::DoubleType(), kOptional,
     "scaleCalibration"},
    {&AutoClavicleTwist(), &rigExecStd::DoubleType(), kOptional, "twist"},
    {&AutoClavicleUpperOffset(), &rigExecStd::DoubleType(), kOptional,
     "upperLengthOffset"},
    {&AutoClavicleLowerOffset(), &rigExecStd::DoubleType(), kOptional,
     "lowerLengthOffset"},
    {&AutoClavicleDelta(), &rigExecStd::Vec3dType(), kRigExecPortOutput,
     "delta"},
    {&AutoClavicleOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

void
ClavicleValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const AutoClavicleOpPayload *>(payload);
    const rigExec::RigExecAutoClavicleConstants &c = p.constants;
    out.Set(kBasis, c.basis, 9);
    out.Set(kIkValue, c.ikValue);
    out.Set(kGain, c.gain);
    out.Set(kKernel, int32_t(c.kernel));
    SetFlag(out, kNormalize, c.normalize);
    SetList(out, kSwingCount, c.swings);
    SetList(out, kWidthCount, c.widths);
    SetList(out, kGainCount, c.gains);
    SetList(out, kWeightCount, c.weights);
    SetFlag(out, kHasLimb, p.hasLimb);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Fixed(Value("basis", rigExecStd::DoubleType()), 9),
    Value("ikValue", rigExecStd::DoubleType()),
    Value("gain", rigExecStd::DoubleType()),
    Value("kernel", rigExecStd::IntType()),
    Flag("normalize"),
    Count("swingCount"),
    Items("swings", rigExecStd::DoubleType(), kSwingCount),
    Count("widthCount"),
    Items("widths", rigExecStd::DoubleType(), kWidthCount),
    Count("gainCount"),
    Items("gains", rigExecStd::DoubleType(), kGainCount),
    Count("weightCount"),
    Items("weights", rigExecStd::DoubleType(), kWeightCount),
    Flag("hasLimb"),
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "AutoClavicle";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunAutoClavicle;
    d.validate = &CheckAutoClavicle;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &ClavicleValues;
    return d;
}();

} // namespace

void
RegisterAutoClavicleOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(AutoClavicle(), kDesc);
}

} // namespace rigExecOps
