#include "rigExecOps/constraint/harmonicsOps.h"

#include "rigExecGraph/windowHistory.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
#include <cstring>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Harmonics)

RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsIn)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsSpace)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsWeight)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsAmplitude)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsDecay)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsTermination)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsAxisAmplitude)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsOut)
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsStatus)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExec::RigExecWindowPacket;
using rigExec::RigExecWindowPacketPtr;
using rigExec::RigExecWindowRing;
using rigExecRig::PointFrame;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using namespace opIo;
using namespace fieldIo;

/// The harmonics payload's declared fields, in saved order.
enum HarmonicsField : uint32_t {
    kWaves,
    kWaveLength,
    kFrequency,
    kIgnoreFirstFrame,
    kMatchVelocity,
    kHasSpace,
    kFrames,
};

bool
Usable(const PointFrame &frame)
{
    if (!(frame.flags & rigExecRig::kPointFrameValid) ||
        (frame.flags & rigExecRig::kPointFrameDegenerate))
        return false;
    for (const Vec3d &point : frame.points)
        for (size_t i = 0; i < 3; ++i)
            if (!std::isfinite(point[i]))
                return false;
    return true;
}

/// The solve's positions, kept so a steady frame allocates none.
struct HarmonicsState {
    std::vector<double> positions;
};

bool
Finite(const PointFrame &frame)
{
    for (const Vec3d &point : frame.points)
        for (size_t i = 0; i < 3; ++i)
            if (!std::isfinite(point[i]))
                return false;
    return true;
}

bool
CheckHarmonics(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (FlagAt(check.batch, kHasSpace, check.k) != Fed(check, HarmonicsSpace()))
        return Fail(
            error, "Harmonics",
            "its space is fed exactly when its settings measure in one");
    return true;
}

bool
HarmonicsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    HarmonicsPayload p;
    p.settings.waves = ValueAt<double>(batch, kWaves, k);
    p.settings.waveLength = ValueAt<double>(batch, kWaveLength, k);
    p.settings.frequency = ValueAt<double>(batch, kFrequency, k);
    p.settings.ignoreFirstFrame = FlagAt(batch, kIgnoreFirstFrame, k);
    p.matchVelocity = FlagAt(batch, kMatchVelocity, k);
    p.hasSpace = FlagAt(batch, kHasSpace, k);
    const PointFrame &in = *In<PointFrame>(io, HarmonicsIn());
    PointFrame &out = Out<PointFrame>(io, HarmonicsOut());
    uint32_t &status = Out<uint32_t>(io, HarmonicsStatus());
    status = uint32_t(HarmonicsResult::Passed);
    const auto passThrough = [&] {
        out = in;
        return true;
    };
    if (!io.window)
        return Fail(error, "Harmonics", "runs only with a window");
    RigExecWindowPacketPtr &capture = *io.window->capture;
    const bool usable = Usable(in);

    // The capture is the incoming origin in the space, taken before any gate.
    Mat4d space(1.0), toSpace(1.0);
    bool spaceUsable = true;
    if (p.hasSpace) {
        space = *In<Mat4d>(io, HarmonicsSpace());
        const double det = space.GetDeterminant();
        spaceUsable = std::isfinite(det) && det != 0.0;
        if (spaceUsable)
            toSpace = space.GetInverse();
    }
    // A capture equal to the one held is kept, so a steady frame allocates
    // none; the ring may share it.
    Vec3d origin(0.0, 0.0, 0.0);
    if (usable && spaceUsable)
        origin = p.hasSpace ? toSpace.Transform(in.points[0]) : in.points[0];
    if (usable && spaceUsable && std::isfinite(origin[0]) &&
        std::isfinite(origin[1]) && std::isfinite(origin[2])) {
        if (!capture || capture->scalars.size() != 3 ||
            capture->scalars[0] != origin[0] ||
            capture->scalars[1] != origin[1] ||
            capture->scalars[2] != origin[2]) {
            auto packet = std::make_shared<RigExecWindowPacket>();
            packet->scalars = {origin[0], origin[1], origin[2]};
            capture = std::move(packet);
        }
    } else {
        capture.reset();
    }
    if (io.window->captureOnly)
        return passThrough();
    if (!*In<uint8_t>(io, HarmonicsEnabled()))
        return passThrough();
    const double weight = *In<double>(io, HarmonicsWeight());
    if (weight <= 0.0)
        return passThrough();
    if ((in.flags & rigExecRig::kPointFrameValid) && !Finite(in))
        return Fail(error, "Harmonics", "its incoming frame is not finite");
    if (!capture) {
        status = uint32_t(spaceUsable
                              ? HarmonicsResult::Unmeasured
                              : HarmonicsResult::UnmeasuredInDegenerateSpace);
        return passThrough();
    }

    // The usable tail of the ring, then this tick, as one position array.
    const RigExecWindowRing &ring = *io.window->ring;
    size_t begin = ring.size();
    while (begin > 0 && ring[begin - 1] && ring[begin - 1]->scalars.size() == 3)
        --begin;
    std::vector<double> &positions =
        static_cast<HarmonicsState *>(rigExec::RigExecBatchState(batch, k))
            ->positions;
    positions.clear();
    for (size_t k = begin; k < ring.size(); ++k)
        positions.insert(positions.end(), ring[k]->scalars.begin(),
                         ring[k]->scalars.end());
    positions.insert(positions.end(), capture->scalars.begin(),
                     capture->scalars.end());

    rigExec::RigExecHarmonicsParameters params;
    params.amplitude = *In<double>(io, HarmonicsAmplitude());
    params.decay = *In<double>(io, HarmonicsDecay());
    params.termination = *In<double>(io, HarmonicsTermination());
    const Vec3d &axis = *In<Vec3d>(io, HarmonicsAxisAmplitude());
    for (size_t a = 0; a < 3; ++a)
        params.axisAmplitude[a] = axis[a];
    params.matchVelocity = p.matchVelocity;
    if (!std::isfinite(params.amplitude) || !std::isfinite(params.decay) ||
        !std::isfinite(params.termination) || !std::isfinite(axis[0]) ||
        !std::isfinite(axis[1]) || !std::isfinite(axis[2]))
        return Fail(error, "Harmonics", "a parameter is not finite");
    double ring3[3];
    // Fewer than two samples: no history yet, nothing rings.
    if (!rigExec::RigExecHarmonicsOffset(
            p.settings, params,
            reinterpret_cast<const double(*)[3]>(positions.data()),
            positions.size() / 3, ring3))
        return passThrough();
    Vec3d offset(ring3[0], ring3[1], ring3[2]);
    if (p.hasSpace)
        offset = space.TransformDir(offset);
    for (size_t a = 0; a < 3; ++a)
        offset[a] *= weight;
    PointFrame candidate = in;
    for (Vec3d &q : candidate.points)
        for (size_t a = 0; a < 3; ++a)
            q[a] += offset[a];
    if (!Usable(candidate))
        return Fail(error, "Harmonics", "its result is not a usable frame");
    out = candidate;
    status = uint32_t(HarmonicsResult::Solved);
    return true;
}

bool
RunHarmonics(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(
        batch, failed, [&](size_t k) { return HarmonicsOne(batch, k, error); });
}

uint32_t
HarmonicsFrames(const void *values)
{
    if (!values)
        return 0;
    uint32_t frames = 0;
    std::memcpy(&frames,
                static_cast<const RigExecFieldValues *>(values)->Data(kFrames),
                sizeof(frames));
    return frames;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&HarmonicsIn(), &rigExecRig::PointFrameType(), kRigExecPortInput, "in"},
    {&HarmonicsSpace(), &rigExecStd::Mat4dType(),
     kRigExecPortInput | kRigExecPortOptional, "space"},
    {&HarmonicsEnabled(), &rigExecStd::BoolType(), kRigExecPortInput,
     "enabled"},
    {&HarmonicsWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&HarmonicsAmplitude(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "amplitude"},
    {&HarmonicsDecay(), &rigExecStd::DoubleType(), kRigExecPortInput, "decay"},
    {&HarmonicsTermination(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "termination"},
    {&HarmonicsAxisAmplitude(), &rigExecStd::Vec3dType(), kRigExecPortInput,
     "axisAmplitude"},
    {&HarmonicsOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput, "out"},
    {&HarmonicsStatus(), &rigExecStd::UIntType(), kRigExecPortOutput, "status"},
};

void
HarmonicsValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const HarmonicsPayload *>(payload);
    out.Set(kWaves, p.settings.waves);
    out.Set(kWaveLength, p.settings.waveLength);
    out.Set(kFrequency, p.settings.frequency);
    SetFlag(out, kIgnoreFirstFrame, p.settings.ignoreFirstFrame);
    SetFlag(out, kMatchVelocity, p.matchVelocity);
    SetFlag(out, kHasSpace, p.hasSpace);
    out.Set(kFrames, p.frames);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("waves", rigExecStd::DoubleType()),
    Value("waveLength", rigExecStd::DoubleType()),
    Value("frequency", rigExecStd::DoubleType()),
    Flag("ignoreFirstFrame"),
    Flag("matchVelocity"),
    Flag("hasSpace"),
    Value("frames", rigExecStd::UIntType()),
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "Harmonics";
    op.ports = kPorts;
    op.portCount = std::size(kPorts);
    op.runBatch = &RunHarmonics;
    op.validate = &CheckHarmonics;
    op.threading = rigExec::RigExecOpThreading::Windowed;
    op.windowFrames = &HarmonicsFrames;
    op.state = opIo::StateOf<HarmonicsState>();
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payloadFields = kFields;
    op.payloadFieldCount = std::size(kFields);
    op.payloadVersion = 1;
    op.payloadValues = &HarmonicsValues;
    return op;
}();

} // namespace

void
RegisterHarmonicsOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Harmonics(), kDesc);
}

} // namespace rigExecOps
