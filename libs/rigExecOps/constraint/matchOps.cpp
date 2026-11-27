#include "rigExecOps/constraint/matchOps.h"

#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/constraint/spaceSwitchFields.h"
#include "rigExecOps/opIo.h"

#include <array>
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

namespace {

using rigExec::RigExecIkFkMatchChannels;
using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using namespace opIo;
using namespace fieldIo;
using namespace spaceSwitchFields;
using rigExec::RigExecOpBatch;

/// The match payloads' declared fields, in saved order: the space switch,
/// then IkFkMatch's own.
enum MatchField : uint32_t {
    kRideCount = spaceSwitchFields::kSpaceSwitchFieldCount,
    kRides,
    kIkValue,
    kScaleCalibration,
    kUpperLengthOffset,
    kLowerLengthOffset,
    kStretchLimb,
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

/// What IkFkMatch's pole switch works in, kept so a steady frame allocates
/// nothing.
struct IkFkMatchState {
    SwitchInputs inputs;
    SwitchRecord record;
};

/// RigExecRunSpaceSwitch, then its frame's matrix; false for a frame the
/// pose walk would refuse (RigExecReadLandmarkMatrixT).
bool
RunSwitch(const SwitchRecord &record, const SwitchInputs &input, Mat4d *out)
{
    Mat4d composed(1.0);
    return rigExec::RigExecRunSpaceSwitchArithmetic<SwitchMath>(record, input,
                                                                &composed) &&
           rigExec::RigExecReadLandmarkMatrixT<rigExec::RrMat3d, Vec3d>(
               composed, out) == rigExec::RigExecLandmarkRead::Read;
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

/// A space switch's sources and carry agree with its setup where known.
template <class Port>
bool
CheckSwitch(const rigExec::RigExecOpCheck &check, const Port &port,
            const char *op, std::string *error)
{
    const SwitchSettings setup = SwitchAt(check.batch, check.k);
    if (setup.hasCarry &&
        (!Fed(check, port.spaceDefault) || !Fed(check, port.spacePosed)))
        return Fail(error, op, "its switch's carry space is not fed");
    const size_t count = setup.world.Size();
    if (!SameCount(CheckElements(check, port.sourceDefaults), count) ||
        !SameCount(CheckElements(check, port.sourcePosed), count))
        return Fail(error, op, "its switch's sources do not match it");
    return true;
}

bool
CheckIkFkMatch(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, IkFkMatchSources()), 10) ||
        !SameCount(CheckElements(check, IkFkMatchRests()), 8) ||
        !SameCount(CheckElements(check, IkFkMatchChannels()),
                   5 * kMatchChannelStride))
        return Fail(error, "IkFkMatch",
                    "needs 10 sources, 8 rests and 5 controls' channels");
    if (SwitchAt(check.batch, check.k).hasCarry &&
        (!Fed(check, IkFkMatchSwitchSpaceDefault()) ||
         !Fed(check, IkFkMatchSwitchSpacePosed())))
        return Fail(error, "IkFkMatch", "its switch's carry space is not fed");
    return true;
}

bool
CheckSpaceMatch(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SwitchAt(check.batch, check.k).world.Size())
        return Fail(error, "SpaceMatch", "needs at least one space");
    if (!SameCount(CheckElements(check, SpaceMatchChannels()),
                   kMatchChannelStride))
        return Fail(error, "SpaceMatch", "needs one control's channels");
    return CheckSwitch(check, SpaceSwitchPorts(), "SpaceMatch", error);
}

// RigExecIkFkMatchConstraint's arithmetic, as the evaluator runs it over
// Gf: the IK half, the pole's parent once the IK control has moved,
// the FK half, and the half presented on the joints.
bool
IkFkMatchOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    struct {
        SwitchSettings poleSwitch;
        rigExec::RigExecFieldView<const uint8_t> poleSourceRides;
        double ikValue, scaleCalibration, upperLengthOffset, lowerLengthOffset;
        bool stretchLimb;
    } const p = {SwitchAt(batch, k),
                 ListAt<uint8_t>(batch, kRides, k),
                 ValueAt<double>(batch, kIkValue, k),
                 ValueAt<double>(batch, kScaleCalibration, k),
                 ValueAt<double>(batch, kUpperLengthOffset, k),
                 ValueAt<double>(batch, kLowerLengthOffset, k),
                 FlagAt(batch, kStretchLimb, k)};
    auto &ikValues = Out<RigExecArray<double>>(io, IkFkMatchIkValues());
    auto &fkValues = Out<RigExecArray<double>>(io, IkFkMatchFkValues());
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
            IkFkMatchState &state = *static_cast<IkFkMatchState *>(
                rigExec::RigExecBatchState(batch, k));
            SwitchInputs &inputs = state.inputs;
            if (!SwitchInputsOf(io, p.poleSwitch, IkFkSwitchPorts(), &inputs))
                return Fail(error, "IkFkMatch",
                            "the pole switch's sources do not match it");
            inputs.active = *In<double>(io, IkFkMatchActive());
            for (size_t k = 0;
                 k < inputs.sources.size() && k < p.poleSourceRides.Size(); ++k)
                if (p.poleSourceRides[k] && !inputs.sources[k].world)
                    inputs.sources[k].posedSpace =
                        inputs.sources[k].posedSpace * moved;
            SwitchRecord &record = state.record;
            record.filters.assign(p.poleSwitch.filters.begin(),
                                  p.poleSwitch.filters.end());
            record.twistAxis = p.poleSwitch.twistAxis;
            record.affectTranslation = p.poleSwitch.affectTranslation;
            record.affectRotation = p.poleSwitch.affectRotation;
            record.affectScale = p.poleSwitch.affectScale;
            Mat4d switched(1.0);
            if (RunSwitch(record, inputs, &switched))
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
        shownMask = kIkFkMatchMatched;
    }
    const auto show = [&](size_t k, const double *frame) {
        shown[k] = FromRows(frame);
        shownMask |= 1u << k;
    };
    const double weight = *In<double>(io, IkFkMatchWeight());
    switch (rigExec::RigExecIkFkMatchShownSide(
        *In<double>(io, IkFkMatchSwitch()), p.ikValue,
        std::isfinite(weight) ? weight : 0.0)) {
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

bool
RunIkFkMatch(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(
        batch, failed, [&](size_t k) { return IkFkMatchOne(batch, k, error); });
}

// The space match of a switched control: per space, the channels that keep
// it on Target with that space selected (RigExecSpaceMatchSolveAll).
bool
SpaceMatchOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const struct {
        SwitchSettings space;
    } p = {SwitchAt(batch, k)};
    auto &values = Out<RigExecArray<double>>(io, SpaceMatchValues());
    SwitchInputs trial;
    if (!SwitchInputsOf(io, p.space, SpaceSwitchPorts(), &trial))
        return Fail(error, "SpaceMatch", "its sources do not match it");
    const auto &channelValues =
        *In<RigExecArray<double>>(io, SpaceMatchChannels());
    if (channelValues.Size() != kMatchChannelStride)
        return Fail(error, "SpaceMatch", "needs one control's channels");
    const size_t spaces = trial.sources.size();
    values.Resize(spaces * rigExec::RigExecSpaceMatchStride);
    const SwitchRecord record = RecordOf(p.space);
    const auto compose = [&](const double *avars, double active,
                             Mat4d *composed) {
        trial.avars = FromRows(avars);
        trial.active = active;
        return rigExec::RigExecRunSpaceSwitchArithmetic<SwitchMath>(
            record, trial, composed);
    };
    switch (rigExec::RigExecSpaceMatchT<rigExec::RrMat3d, Vec3d>(
        *In<Mat4d>(io, SpaceMatchTarget()), ChannelsAt(channelValues, 0),
        spaces, compose, values.Data())) {
    case rigExec::RigExecSpaceMatchOutcome::Matched:
        return true;
    case rigExec::RigExecSpaceMatchOutcome::NotFinite:
        return Fail(error, "SpaceMatch",
                    rigExec::kRigExecSpaceMatchNotFinite + 1);
    case rigExec::RigExecSpaceMatchOutcome::Unsolved:
        break;
    }
    return Fail(error, "SpaceMatch", "could not match its spaces");
}

bool
RunSpaceMatch(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SpaceMatchOne(batch, k, error);
    });
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
    {&IkFkMatchIkValues(), &rigExecStd::DoubleArray(), kOut, "ikValues",
     rigExec::RigExecOutputBound::Fixed, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 11},
    {&IkFkMatchFkValues(), &rigExecStd::DoubleArray(), kOut, "fkValues",
     rigExec::RigExecOutputBound::Fixed, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 20},
    {&IkFkMatchShown(), &rigExecStd::Mat4dArray(), kOut, "shown",
     rigExec::RigExecOutputBound::Fixed, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 5},
    {&IkFkMatchShownMask(), &rigExecStd::UIntType(), kOut, "shownMask"},
};

void
IkFkValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const IkFkMatchPayload *>(payload);
    SetSwitch(out, p.poleSwitch);
    SetList(out, kRideCount, p.poleSourceRides);
    out.Set(kIkValue, p.ikValue);
    out.Set(kScaleCalibration, p.scaleCalibration);
    out.Set(kUpperLengthOffset, p.upperLengthOffset);
    out.Set(kLowerLengthOffset, p.lowerLengthOffset);
    SetFlag(out, kStretchLimb, p.stretchLimb);
}

void
SpaceValues(const void *payload, RigExecFieldValues &out)
{
    SetSwitch(out, static_cast<const SpaceMatchPayload *>(payload)->space);
}

const rigExec::RigExecFieldDesc kIkFkFields[] = {
    Count("filterCount"),
    Items("filters", rigExecStd::IntType(), kFilterCount),
    Value("twistAxis", rigExecStd::Vec3dType()),
    Fixed(Flag("affectTranslation"), 3),
    Fixed(Flag("affectRotation"), 3),
    Fixed(Flag("affectScale"), 3),
    Flag("hasCarry"),
    Count("worldCount"),
    Items("world", rigExecStd::BoolType(), kWorldCount),
    Count("rideCount"),
    Items("poleSourceRides", rigExecStd::BoolType(), kRideCount),
    Value("ikValue", rigExecStd::DoubleType()),
    Value("scaleCalibration", rigExecStd::DoubleType()),
    Value("upperLengthOffset", rigExecStd::DoubleType()),
    Value("lowerLengthOffset", rigExecStd::DoubleType()),
    Flag("stretchLimb"),
};

/// SpaceMatch saves the space switch alone: the first fields above.
constexpr size_t kSpaceFieldCount = kRideCount;

const rigExec::RigExecOpDesc kIkFk = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "IkFkMatch";
    op.ports = kIkFkPorts;
    op.portCount = std::size(kIkFkPorts);
    op.runBatch = &RunIkFkMatch;
    op.validate = &CheckIkFkMatch;
    op.state = opIo::StateOf<IkFkMatchState>();
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payloadFields = kIkFkFields;
    op.payloadFieldCount = std::size(kIkFkFields);
    op.payloadVersion = 1;
    op.payloadValues = &IkFkValues;
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
    {&SpaceMatchValues(), &rigExecStd::DoubleArray(), kOut, "values",
     rigExec::RigExecOutputBound::Input, &SpaceMatchSourceDefaults(), nullptr,
     rigExec::kRigExecInvalidIndex, rigExec::RigExecSpaceMatchStride},
};

const rigExec::RigExecOpDesc kSpace = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "SpaceMatch";
    op.ports = kSpacePorts;
    op.portCount = std::size(kSpacePorts);
    op.runBatch = &RunSpaceMatch;
    op.validate = &CheckSpaceMatch;
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payloadFields = kIkFkFields;
    op.payloadFieldCount = kSpaceFieldCount;
    op.payloadVersion = 1;
    op.payloadValues = &SpaceValues;
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
