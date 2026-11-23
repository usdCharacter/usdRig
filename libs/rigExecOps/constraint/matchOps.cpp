#include "rigExecOps/constraint/matchOps.h"

#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
#include "rigExecRuntime/switchMath.h"

#include <cmath>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(IkFkMatch)

RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchWeight)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSwitch)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchStretch)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSoftDistance)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchUpperScale)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchLowerScale)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchPin)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchTwist)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSources)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchRests)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchChannels)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchHasSpace)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSpaceRest)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSpacePosed)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchPoleMode)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchParentPosed)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchParentDefault)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSwitchSpaceDefault)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSwitchSpacePosed)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSourceDefaults)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchSourcePosed)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchActive)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchIkValues)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchFkValues)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchValid)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchShown)
RIGEXEC_DEFINE_PORT_HANDLE(IkFkMatchShownMask)

RIGEXEC_DEFINE_OP_HANDLE(SpaceMatch)

RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchTarget)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchChannels)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchPosedDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchParentDefaultInverse)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchParentPosed)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchParentDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchSpaceDefault)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchSpacePosed)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchSourceDefaults)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchSourcePosed)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchValues)
RIGEXEC_DEFINE_PORT_HANDLE(SpaceMatchValid)

namespace {

using rigExec::RigExecIkFkMatchChannels;
using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using namespace opIo;

using SwitchRecord = rigExec::RigExecSpaceSwitchRecordT<Vec3d>;
using SwitchInputs = rigExec::RigExecSpaceSwitchInputsT<Mat4d>;

struct SwitchMath {
    using Matrix = Mat4d;
    static Matrix RoundTrip(const Matrix &m)
    {
        return rigExec::RrSwitchRoundTrip(m);
    }
    static Matrix Filter(const Matrix &m, const Vec3d &axis, int filter)
    {
        return rigExec::RrSwitchFilter(m, axis, filter);
    }
    static Matrix Blend(const Matrix &a, const Matrix &b, double t)
    {
        return rigExec::RrSwitchBlend(a, b, t);
    }
    static Matrix Mask(const Matrix &m, const bool *t, const bool *r,
                       const bool *s)
    {
        return rigExec::RrSwitchMask(m, t, r, s);
    }
};

const double *
Rows(const Mat4d &m)
{
    return &m._mtx[0][0];
}

Mat4d
FromRows(const double *rows)
{
    Mat4d m;
    std::copy(rows, rows + 16, &m._mtx[0][0]);
    return m;
}

SwitchRecord
RecordOf(const SpaceSwitchSetup &setup)
{
    SwitchRecord record;
    record.filters = setup.filters;
    record.twistAxis = setup.twistAxis;
    record.affectTranslation = setup.affectTranslation;
    record.affectRotation = setup.affectRotation;
    record.affectScale = setup.affectScale;
    return record;
}

/// RigExecRunSpaceSwitch, then its frame's matrix; \p usable refuses a
/// frame with a non-finite landmark, as the IK/FK match does and the baked
/// space match does not.
bool
RunSwitch(const SwitchRecord &record, const SwitchInputs &input, bool usable,
          Mat4d *out)
{
    Mat4d composed(1.0);
    if (!rigExec::RigExecRunSpaceSwitchArithmetic<SwitchMath>(record, input,
                                                              &composed))
        return false;
    std::array<Vec3d, 4> points;
    const bool finite = rigExec::RrSwitchLandmarks(composed, &points);
    return (finite || !usable) &&
           rigExec::RrDualQuatPointsToMatrix(
               rigExec::kRrSwitchIdentityLandmarks, points, out);
}

/// The switch's inputs from its ports; false when the sources and the setup
/// disagree.
template <class Port>
bool
SwitchInputsOf(const RigExecOpIo &io, const SpaceSwitchSetup &setup,
               const Port &port, SwitchInputs *in)
{
    in->posedDefault = *In<Mat4d>(io, port.posedDefault);
    in->parentDefaultInverse = *In<Mat4d>(io, port.parentDefaultInverse);
    in->parentPosed = *In<Mat4d>(io, port.parentPosed);
    in->parentDefault = *In<Mat4d>(io, port.parentDefault);
    in->hasCarry = setup.hasCarry;
    if (setup.hasCarry) {
        const Mat4d *spaceDefault = In<Mat4d>(io, port.spaceDefault);
        const Mat4d *spacePosed = In<Mat4d>(io, port.spacePosed);
        if (!spaceDefault || !spacePosed)
            return false;
        in->spaceDefault = *spaceDefault;
        in->spacePosed = *spacePosed;
    }
    const auto &defaults = *In<RigExecArray<Mat4d>>(io, port.sourceDefaults);
    const auto &posed = *In<RigExecArray<Mat4d>>(io, port.sourcePosed);
    const size_t count = setup.world.size();
    if (defaults.Size() != count || posed.Size() != count)
        return false;
    in->sources.resize(count);
    for (size_t k = 0; k < count; ++k) {
        in->sources[k].world = setup.world[k] != 0;
        in->sources[k].defaultSpace = defaults[k];
        in->sources[k].posedSpace = posed[k];
    }
    return true;
}

RigExecIkFkMatchChannels
ChannelsAt(const RigExecArray<double> &values, size_t control)
{
    const double *v = values.Data() + control * kMatchChannelStride;
    RigExecIkFkMatchChannels ch;
    for (int k = 0; k < 3; ++k) {
        ch.t[k] = v[k];
        ch.s[k] = v[3 + k];
        ch.r[k] = v[6 + k];
        ch.sign[k] = v[10 + k];
        ch.order[k] = char(v[15 + k]);
    }
    ch.spin = v[9];
    ch.units = v[13];
    return ch;
}

struct IkFkSwitchPorts {
    const rigExec::RigExecPortHandle &posedDefault = IkFkMatchPosedDefault();
    const rigExec::RigExecPortHandle &parentDefaultInverse =
        IkFkMatchParentDefaultInverse();
    const rigExec::RigExecPortHandle &parentPosed = IkFkMatchParentPosed();
    const rigExec::RigExecPortHandle &parentDefault = IkFkMatchParentDefault();
    const rigExec::RigExecPortHandle &spaceDefault =
        IkFkMatchSwitchSpaceDefault();
    const rigExec::RigExecPortHandle &spacePosed = IkFkMatchSwitchSpacePosed();
    const rigExec::RigExecPortHandle &sourceDefaults =
        IkFkMatchSourceDefaults();
    const rigExec::RigExecPortHandle &sourcePosed = IkFkMatchSourcePosed();
};

struct SpaceSwitchPorts {
    const rigExec::RigExecPortHandle &posedDefault = SpaceMatchPosedDefault();
    const rigExec::RigExecPortHandle &parentDefaultInverse =
        SpaceMatchParentDefaultInverse();
    const rigExec::RigExecPortHandle &parentPosed = SpaceMatchParentPosed();
    const rigExec::RigExecPortHandle &parentDefault = SpaceMatchParentDefault();
    const rigExec::RigExecPortHandle &spaceDefault = SpaceMatchSpaceDefault();
    const rigExec::RigExecPortHandle &spacePosed = SpaceMatchSpacePosed();
    const rigExec::RigExecPortHandle &sourceDefaults =
        SpaceMatchSourceDefaults();
    const rigExec::RigExecPortHandle &sourcePosed = SpaceMatchSourcePosed();
};

// RigExecIkFkMatchConstraint's arithmetic, as the scene program's body runs
// it over Gf: the IK half, the pole's parent once the IK control has moved,
// the FK half, and the half presented on the joints.
bool
RunIkFkMatch(const void *payload, void *, const RigExecOpIo &io,
             std::string *error)
{
    const auto &p = *static_cast<const IkFkMatchPayload *>(payload);
    auto &ikValues = Out<RigExecArray<double>>(io, IkFkMatchIkValues());
    auto &fkValues = Out<RigExecArray<double>>(io, IkFkMatchFkValues());
    uint8_t &valid = Out<uint8_t>(io, IkFkMatchValid());
    auto &shown = Out<RigExecArray<Mat4d>>(io, IkFkMatchShown());
    uint32_t &shownMask = Out<uint32_t>(io, IkFkMatchShownMask());
    ikValues.Resize(11);
    fkValues.Resize(20);
    for (size_t k = 0; k < 11; ++k)
        ikValues[k] = 0.0;
    for (size_t k = 0; k < 20; ++k)
        fkValues[k] = 0.0;
    shown.Resize(5);
    for (size_t k = 0; k < 5; ++k)
        shown[k] = Mat4d(1.0);
    valid = 0;
    shownMask = 0;
    if (!*In<uint8_t>(io, IkFkMatchEnabled()))
        return true;
    const auto &sources = *In<RigExecArray<Mat4d>>(io, IkFkMatchSources());
    const auto &rests = *In<RigExecArray<Mat4d>>(io, IkFkMatchRests());
    const auto &channelValues =
        *In<RigExecArray<double>>(io, IkFkMatchChannels());
    if (sources.Size() != 10 || rests.Size() != 8 ||
        channelValues.Size() != 5 * kMatchChannelStride)
        return Fail(error, "IkFkMatch",
                    "needs 10 sources, 8 rests and 5 "
                    "controls' channels");
    const Mat4d *input = sources.Data();
    const Mat4d *rest = rests.Data();
    const Mat4d *control = input + 5;
    RigExecIkFkMatchChannels channels[5];
    for (size_t i = 0; i < 5; ++i)
        channels[i] = ChannelsAt(channelValues, i);
    rigExec::RigExecLimbStretch limb;
    limb.stretch = *In<double>(io, IkFkMatchStretch());
    limb.softDistance = *In<double>(io, IkFkMatchSoftDistance());
    limb.upperScale = *In<double>(io, IkFkMatchUpperScale());
    limb.lowerScale = *In<double>(io, IkFkMatchLowerScale());
    limb.pin = *In<double>(io, IkFkMatchPin());
    limb.scaleCalibration = p.scaleCalibration;
    // The IK solve's space carries its rest lengths and soft distance.
    double space[16];
    const double *spaceMatrix = nullptr;
    if (*In<uint8_t>(io, IkFkMatchHasSpace())) {
        const Mat4d carry = In<Mat4d>(io, IkFkMatchSpaceRest())->GetInverse() *
                            *In<Mat4d>(io, IkFkMatchSpacePosed());
        std::copy(Rows(carry), Rows(carry) + 16, space);
        spaceMatrix = space;
    }
    double restOrigins[3][3], restUpper = 0, restLower = 0, spaceFactor = 1;
    for (size_t i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            restOrigins[i][k] = rest[i]._mtx[3][k];
    rigExec::RigExecIkFkMatchRestLengths(
        restOrigins, spaceMatrix, p.upperLengthOffset, p.lowerLengthOffset,
        &restUpper, &restLower, &spaceFactor);
    limb.softDistance *= spaceFactor;
    rigExec::RigExecIkFkMatchIkInputs ikIn;
    for (size_t i = 0; i < 3; ++i)
        ikIn.joints[i] = Rows(input[i]);
    ikIn.root = Rows(input[3]);
    ikIn.effectorPosed = Rows(input[4]);
    ikIn.ikPosed = Rows(control[0]);
    ikIn.ik = channels[0];
    ikIn.twistRadians =
        *In<double>(io, IkFkMatchTwist()) * std::acos(-1.0) / 180.0;
    ikIn.poleDistance =
        (rest[4].ExtractTranslation() - rest[1].ExtractTranslation())
            .GetLength() *
        spaceFactor;
    ikIn.polePosed = Rows(control[1]);
    ikIn.hasStretch = p.stretchLimb;
    ikIn.limb = limb;
    ikIn.restUpper = restUpper;
    ikIn.restLower = restLower;
    rigExec::RigExecIkFkMatchIkResult ik;
    rigExec::RigExecIkFkMatchIk(ikIn, &ik);
    // The pole's parent once the IK control has moved: its own space switch
    // re-run with every source that rides the IK control carried by the
    // move, or its namespace parent carried the same way.
    RigExecIkFkMatchChannels pole = channels[1];
    double poleFrame[16];
    bool poleOk = false;
    if (ik.ok) {
        const Mat4d moved = control[0].GetInverse() * FromRows(ik.ikFrame);
        const auto composed = rigExec::ikFkMatchKernel::Compose(channels[1]);
        Mat4d parent = FromRows(composed.data()).GetInverse() * control[1];
        const auto mode = IkFkPoleMode(*In<uint32_t>(io, IkFkMatchPoleMode()));
        if (mode == IkFkPoleMode::Switch) {
            SwitchInputs inputs;
            if (!SwitchInputsOf(io, p.poleSwitch, IkFkSwitchPorts(), &inputs))
                return Fail(error, "IkFkMatch",
                            "the pole switch's sources do not match it");
            inputs.active = *In<double>(io, IkFkMatchActive());
            for (size_t k = 0;
                 k < inputs.sources.size() && k < p.poleSourceRides.size(); ++k)
                if (p.poleSourceRides[k] && !inputs.sources[k].world)
                    inputs.sources[k].posedSpace =
                        inputs.sources[k].posedSpace * moved;
            Mat4d switched(1.0);
            if (RunSwitch(RecordOf(p.poleSwitch), inputs, true, &switched))
                parent = switched;
        } else if (mode == IkFkPoleMode::Ride) {
            parent = parent * moved;
        }
        poleOk = rigExec::RigExecIkFkMatchPole(ik.poleTarget, Rows(parent),
                                               &pole, poleFrame);
    }
    rigExec::RigExecIkFkMatchFkInputs fkIn;
    Mat4d offsets[3];
    for (size_t i = 0; i < 3; ++i) {
        fkIn.joints[i] = Rows(input[i]);
        fkIn.fkPosed[i] = Rows(control[2 + i]);
        fkIn.fk[i] = channels[2 + i];
        offsets[i] = rest[5 + i] * rest[i].GetInverse();
        fkIn.offsets[i] = Rows(offsets[i]);
    }
    fkIn.hasStretch = p.stretchLimb;
    fkIn.upperScale = limb.upperScale;
    fkIn.lowerScale = limb.lowerScale;
    fkIn.restUpper = restUpper;
    fkIn.restLower = restLower;
    rigExec::RigExecIkFkMatchFkResult fk;
    rigExec::RigExecIkFkMatchFk(fkIn, &fk);
    if (ik.ok && poleOk && fk.ok) {
        rigExec::RigExecIkFkMatchPack(ik, pole, fk, ikValues.Data(),
                                      fkValues.Data());
        valid = 1;
    }
    const auto show = [&](size_t k, const double *frame) {
        shown[k] = FromRows(frame);
        shownMask |= 1u << k;
    };
    switch (rigExec::RigExecIkFkMatchShownSide(
        *In<double>(io, IkFkMatchSwitch()), p.ikValue,
        *In<double>(io, IkFkMatchWeight()))) {
    case rigExec::RigExecIkFkMatchShown::IkSide:
        if (ik.ok && poleOk) {
            show(0, ik.ikFrame);
            show(1, poleFrame);
        }
        break;
    case rigExec::RigExecIkFkMatchShown::FkSide:
        if (fk.ok)
            for (size_t i = 0; i < 3; ++i)
                show(2 + i, fk.frames[i]);
        break;
    case rigExec::RigExecIkFkMatchShown::None:
        break;
    }
    return true;
}

// The space match of a switched control: per space, the channels that keep
// it on Target with that space selected (RigExecSpaceMatchSolveAll).
bool
RunSpaceMatch(const void *payload, void *, const RigExecOpIo &io,
              std::string *error)
{
    const auto &p = *static_cast<const SpaceMatchPayload *>(payload);
    auto &values = Out<RigExecArray<double>>(io, SpaceMatchValues());
    uint8_t &valid = Out<uint8_t>(io, SpaceMatchValid());
    valid = 0;
    SwitchInputs trial;
    if (!SwitchInputsOf(io, p.space, SpaceSwitchPorts(), &trial))
        return Fail(error, "SpaceMatch", "its sources do not match it");
    const auto &channelValues =
        *In<RigExecArray<double>>(io, SpaceMatchChannels());
    if (channelValues.Size() != kMatchChannelStride)
        return Fail(error, "SpaceMatch", "needs one control's channels");
    const size_t spaces = trial.sources.size();
    values.Resize(spaces * rigExec::RigExecSpaceMatchStride);
    if (!spaces)
        return true;
    const SwitchRecord record = RecordOf(p.space);
    const auto run = [&](const double *avars, double active, double *out) {
        trial.avars = FromRows(avars);
        trial.active = active;
        Mat4d matrix;
        if (!RunSwitch(record, trial, false, &matrix))
            return false;
        std::copy(Rows(matrix), Rows(matrix) + 16, out);
        return true;
    };
    valid = rigExec::RigExecSpaceMatchSolveAll(
                Rows(*In<Mat4d>(io, SpaceMatchTarget())),
                ChannelsAt(channelValues, 0), spaces, run, values.Data())
                ? 1
                : 0;
    return true;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kIn = kRigExecPortInput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
constexpr uint32_t kOut = kRigExecPortOutput;

const rigExec::RigExecPortDesc kIkFkPorts[] = {
    {&IkFkMatchEnabled(), &rigExecStd::BoolType(), kIn, "enabled"},
    {&IkFkMatchWeight(), &rigExecStd::DoubleType(), kIn, "weight"},
    {&IkFkMatchSwitch(), &rigExecStd::DoubleType(), kIn, "switch"},
    {&IkFkMatchStretch(), &rigExecStd::DoubleType(), kIn, "stretch"},
    {&IkFkMatchSoftDistance(), &rigExecStd::DoubleType(), kIn, "softDistance"},
    {&IkFkMatchUpperScale(), &rigExecStd::DoubleType(), kIn, "upperScale"},
    {&IkFkMatchLowerScale(), &rigExecStd::DoubleType(), kIn, "lowerScale"},
    {&IkFkMatchPin(), &rigExecStd::DoubleType(), kIn, "pin"},
    {&IkFkMatchTwist(), &rigExecStd::DoubleType(), kIn, "twist"},
    {&IkFkMatchSources(), &rigExecStd::Mat4dArray(), kIn, "sources"},
    {&IkFkMatchRests(), &rigExecStd::Mat4dArray(), kIn, "rests"},
    {&IkFkMatchChannels(), &rigExecStd::DoubleArray(), kIn, "channels"},
    {&IkFkMatchHasSpace(), &rigExecStd::BoolType(), kIn, "hasSpace"},
    {&IkFkMatchSpaceRest(), &rigExecStd::Mat4dType(), kIn, "spaceRest"},
    {&IkFkMatchSpacePosed(), &rigExecStd::Mat4dType(), kIn, "spacePosed"},
    {&IkFkMatchPoleMode(), &rigExecStd::UIntType(), kIn, "poleMode"},
    {&IkFkMatchPosedDefault(), &rigExecStd::Mat4dType(), kIn,
     "polePosedDefault"},
    {&IkFkMatchParentDefaultInverse(), &rigExecStd::Mat4dType(), kIn,
     "poleParentDefaultInverse"},
    {&IkFkMatchParentPosed(), &rigExecStd::Mat4dType(), kIn, "poleParentPosed"},
    {&IkFkMatchParentDefault(), &rigExecStd::Mat4dType(), kIn,
     "poleParentDefault"},
    {&IkFkMatchSwitchSpaceDefault(), &rigExecStd::Mat4dType(), kOptional,
     "poleSpaceDefault"},
    {&IkFkMatchSwitchSpacePosed(), &rigExecStd::Mat4dType(), kOptional,
     "poleSpacePosed"},
    {&IkFkMatchSourceDefaults(), &rigExecStd::Mat4dArray(), kIn,
     "poleSourceDefaults"},
    {&IkFkMatchSourcePosed(), &rigExecStd::Mat4dArray(), kIn,
     "poleSourcePosed"},
    {&IkFkMatchActive(), &rigExecStd::DoubleType(), kIn, "poleActive"},
    {&IkFkMatchIkValues(), &rigExecStd::DoubleArray(), kOut, "ikValues"},
    {&IkFkMatchFkValues(), &rigExecStd::DoubleArray(), kOut, "fkValues"},
    {&IkFkMatchValid(), &rigExecStd::BoolType(), kOut, "valid"},
    {&IkFkMatchShown(), &rigExecStd::Mat4dArray(), kOut, "shown"},
    {&IkFkMatchShownMask(), &rigExecStd::UIntType(), kOut, "shownMask"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
PutSwitch(Writer &w, const SpaceSwitchSetup &s)
{
    payloadIo::PutList(w, s.filters);
    payloadIo::Put(w, s.twistAxis);
    payloadIo::PutFlags(w, s.affectTranslation);
    payloadIo::PutFlags(w, s.affectRotation);
    payloadIo::PutFlags(w, s.affectScale);
    w.Bool(s.hasCarry);
    payloadIo::PutList(w, s.world);
}

void
GetSwitch(Reader &r, SpaceSwitchSetup &s)
{
    payloadIo::GetList(r, s.filters);
    payloadIo::Get(r, s.twistAxis);
    payloadIo::GetFlags(r, s.affectTranslation);
    payloadIo::GetFlags(r, s.affectRotation);
    payloadIo::GetFlags(r, s.affectScale);
    s.hasCarry = r.Bool();
    payloadIo::GetList(r, s.world);
}

void
Save(Writer &w, const IkFkMatchPayload &p)
{
    w.Version(1);
    PutSwitch(w, p.poleSwitch);
    payloadIo::PutList(w, p.poleSourceRides);
    w.F64(p.ikValue);
    w.F64(p.scaleCalibration);
    w.F64(p.upperLengthOffset);
    w.F64(p.lowerLengthOffset);
    w.Bool(p.stretchLimb);
}

void
Load(Reader &r, IkFkMatchPayload &p)
{
    r.Version(1);
    GetSwitch(r, p.poleSwitch);
    payloadIo::GetList(r, p.poleSourceRides);
    p.ikValue = r.F64();
    p.scaleCalibration = r.F64();
    p.upperLengthOffset = r.F64();
    p.lowerLengthOffset = r.F64();
    p.stretchLimb = r.Bool();
}

void
Save(Writer &w, const SpaceMatchPayload &p)
{
    w.Version(1);
    PutSwitch(w, p.space);
}

void
Load(Reader &r, SpaceMatchPayload &p)
{
    r.Version(1);
    GetSwitch(r, p.space);
}

const rigExec::RigExecOpDesc kIkFk = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "IkFkMatch";
    op.ports = kIkFkPorts;
    op.portCount = std::size(kIkFkPorts);
    op.run = &RunIkFkMatch;
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payload =
        rigExec::RigExecPayloadHandlingFor<IkFkMatchPayload, &Save, &Load>();
    return op;
}();

const rigExec::RigExecPortDesc kSpacePorts[] = {
    {&SpaceMatchTarget(), &rigExecStd::Mat4dType(), kIn, "target"},
    {&SpaceMatchChannels(), &rigExecStd::DoubleArray(), kIn, "channels"},
    {&SpaceMatchPosedDefault(), &rigExecStd::Mat4dType(), kIn, "posedDefault"},
    {&SpaceMatchParentDefaultInverse(), &rigExecStd::Mat4dType(), kIn,
     "parentDefaultInverse"},
    {&SpaceMatchParentPosed(), &rigExecStd::Mat4dType(), kIn, "parentPosed"},
    {&SpaceMatchParentDefault(), &rigExecStd::Mat4dType(), kIn,
     "parentDefault"},
    {&SpaceMatchSpaceDefault(), &rigExecStd::Mat4dType(), kOptional,
     "spaceDefault"},
    {&SpaceMatchSpacePosed(), &rigExecStd::Mat4dType(), kOptional,
     "spacePosed"},
    {&SpaceMatchSourceDefaults(), &rigExecStd::Mat4dArray(), kIn,
     "sourceDefaults"},
    {&SpaceMatchSourcePosed(), &rigExecStd::Mat4dArray(), kIn, "sourcePosed"},
    {&SpaceMatchValues(), &rigExecStd::DoubleArray(), kOut, "values"},
    {&SpaceMatchValid(), &rigExecStd::BoolType(), kOut, "valid"},
};

const rigExec::RigExecOpDesc kSpace = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "SpaceMatch";
    op.ports = kSpacePorts;
    op.portCount = std::size(kSpacePorts);
    op.run = &RunSpaceMatch;
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payload =
        rigExec::RigExecPayloadHandlingFor<SpaceMatchPayload, &Save, &Load>();
    return op;
}();

} // namespace

void
RegisterMatchOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(IkFkMatch(), kIkFk);
    registrar.Add(SpaceMatch(), kSpace);
}

} // namespace rigExecOps
