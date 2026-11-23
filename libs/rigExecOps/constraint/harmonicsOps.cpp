#include "rigExecOps/constraint/harmonicsOps.h"

#include "rigExecGraph/windowHistory.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
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
RIGEXEC_DEFINE_PORT_HANDLE(HarmonicsStatusOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecWindowPacket;
using rigExec::RigExecWindowPacketPtr;
using rigExec::RigExecWindowRing;
using rigExecRig::PointFrame;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using namespace opIo;

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

bool
RunHarmonics(const void *payload, void *, const RigExecOpIo &io,
             std::string *error)
{
    const auto &p = *static_cast<const HarmonicsPayload *>(payload);
    const PointFrame &in = *In<PointFrame>(io, HarmonicsIn());
    PointFrame &out = Out<PointFrame>(io, HarmonicsOut());
    uint32_t &status = Out<uint32_t>(io, HarmonicsStatusOut());
    const auto finish = [&](HarmonicsStatus why) {
        out = in;
        status = uint32_t(why);
        return true;
    };
    if (!io.window)
        return Fail(error, "Harmonics", "runs only with a window");
    RigExecWindowPacketPtr &capture = *io.window->capture;
    capture.reset();
    if (!Usable(in))
        return finish(HarmonicsStatus::Unusable);

    // The capture is the incoming origin in the space, taken before any gate.
    Mat4d space(1.0), toSpace(1.0);
    bool spaceUsable = true;
    if (p.hasSpace) {
        const Mat4d *fed = In<Mat4d>(io, HarmonicsSpace());
        if (!fed)
            return Fail(error, "Harmonics", "its space is not fed");
        space = *fed;
        const double det = space.GetDeterminant();
        spaceUsable = std::isfinite(det) && det != 0.0;
        if (spaceUsable)
            toSpace = space.GetInverse();
    }
    if (spaceUsable) {
        const Vec3d origin =
            p.hasSpace ? toSpace.Transform(in.points[0]) : in.points[0];
        if (std::isfinite(origin[0]) && std::isfinite(origin[1]) &&
            std::isfinite(origin[2])) {
            auto packet = std::make_shared<RigExecWindowPacket>();
            packet->scalars = {origin[0], origin[1], origin[2]};
            capture = std::move(packet);
        }
    }
    if (io.window->captureOnly)
        return finish(HarmonicsStatus::PassedThrough);
    if (!*In<uint8_t>(io, HarmonicsEnabled()))
        return finish(HarmonicsStatus::PassedThrough);
    const double weight = *In<double>(io, HarmonicsWeight());
    if (weight <= 0.0)
        return finish(HarmonicsStatus::PassedThrough);
    if (!capture)
        return finish(spaceUsable ? HarmonicsStatus::NoMeasure
                                  : HarmonicsStatus::NoMeasureDegenerateSpace);

    // The usable tail of the ring, then this tick, as one position array.
    const RigExecWindowRing &ring = *io.window->ring;
    size_t begin = ring.size();
    while (begin > 0 && ring[begin - 1] && ring[begin - 1]->scalars.size() == 3)
        --begin;
    std::vector<double> positions;
    positions.reserve(3 * (ring.size() - begin + 1));
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
        return finish(HarmonicsStatus::NonFinite);
    double ring3[3];
    // Fewer than two samples: no history yet, nothing rings.
    if (!rigExec::RigExecHarmonicsOffset(
            p.settings, params,
            reinterpret_cast<const double(*)[3]>(positions.data()),
            positions.size() / 3, ring3))
        return finish(HarmonicsStatus::PassedThrough);
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
        return finish(HarmonicsStatus::InvalidResult);
    out = candidate;
    status = uint32_t(HarmonicsStatus::Applied);
    return true;
}

uint32_t
HarmonicsFrames(const void *payload)
{
    return payload ? static_cast<const HarmonicsPayload *>(payload)->frames : 0;
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
    {&HarmonicsStatusOut(), &rigExecStd::UIntType(), kRigExecPortOutput,
     "status"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const HarmonicsPayload &p)
{
    w.Version(1);
    w.F64(p.settings.waves);
    w.F64(p.settings.waveLength);
    w.F64(p.settings.frequency);
    w.Bool(p.settings.ignoreFirstFrame);
    w.Bool(p.matchVelocity);
    w.Bool(p.hasSpace);
    w.U32(p.frames);
}

void
Load(Reader &r, HarmonicsPayload &p)
{
    r.Version(1);
    p.settings.waves = r.F64();
    p.settings.waveLength = r.F64();
    p.settings.frequency = r.F64();
    p.settings.ignoreFirstFrame = r.Bool();
    p.matchVelocity = r.Bool();
    p.hasSpace = r.Bool();
    p.frames = r.U32();
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc op;
    op.nameSpace = "rig";
    op.name = "Harmonics";
    op.ports = kPorts;
    op.portCount = std::size(kPorts);
    op.run = &RunHarmonics;
    op.threading = rigExec::RigExecOpThreading::Windowed;
    op.windowFrames = &HarmonicsFrames;
    op.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    op.payload =
        rigExec::RigExecPayloadHandlingFor<HarmonicsPayload, &Save, &Load>();
    return op;
}();

} // namespace

void
RegisterHarmonicsOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Harmonics(), kDesc);
}

} // namespace rigExecOps
