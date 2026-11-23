#include "rigExecOps/constraint/autoClavicleOps.h"

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

using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using namespace opIo;

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
RunAutoClavicle(const void *payload, void *, const RigExecOpIo &io,
                std::string *error)
{
    const auto &p = *static_cast<const AutoClavicleOpPayload *>(payload);
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

} // namespace

void
RegisterAutoClavicleOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &m = rigExecStd::Mat4dType();
    const auto &ms = rigExecStd::Mat4dArray();
    const auto &d = rigExecStd::DoubleType();

    static const rigExec::RigExecPortDesc ports[] = {
        {&AutoClavicleTarget(), &m, kRigExecPortInput, "target"},
        {&AutoClaviclePivot(), &m, kRigExecPortInput, "pivot"},
        {&AutoClavicleAnchor(), &m, kRigExecPortInput, "anchor"},
        {&AutoClavicleAnchorDefault(), &m, kRigExecPortInput, "anchorDefault"},
        {&AutoClavicleFkPosed(), &m, kRigExecPortInput, "fkPosed"},
        {&AutoClavicleFkDefault(), &ms, kRigExecPortInput, "fkDefault"},
        {&AutoClavicleIkTarget(), &m, kOptional, "ikTarget"},
        {&AutoClaviclePole(), &m, kOptional, "pole"},
        {&AutoClavicleIkBlend(), &d, kRigExecPortInput, "ikBlend"},
        {&AutoClavicleAmount(), &d, kRigExecPortInput, "amount"},
        {&AutoClavicleLimbRests(), &ms, kOptional, "limbRests"},
        {&AutoClavicleStretch(), &d, kOptional, "stretch"},
        {&AutoClaviclePin(), &d, kOptional, "pin"},
        {&AutoClavicleUpperScale(), &d, kOptional, "upperScale"},
        {&AutoClavicleLowerScale(), &d, kOptional, "lowerScale"},
        {&AutoClavicleSoftDistance(), &d, kOptional, "softDistance"},
        {&AutoClavicleScaleCalibration(), &d, kOptional, "scaleCalibration"},
        {&AutoClavicleTwist(), &d, kOptional, "twist"},
        {&AutoClavicleUpperOffset(), &d, kOptional, "upperLengthOffset"},
        {&AutoClavicleLowerOffset(), &d, kOptional, "lowerLengthOffset"},
        {&AutoClavicleDelta(), &rigExecStd::Vec3dType(), kRigExecPortOutput,
         "delta"},
        {&AutoClavicleOut(), &m, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "AutoClavicle";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunAutoClavicle;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<AutoClavicleOpPayload>();
        return d;
    }();
    registrar.Add(AutoClavicle(), desc);
}

} // namespace rigExecOps
