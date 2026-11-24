#include "rigExecOps/deform/clothOp.h"

#include "rigExecGraph/windowHistory.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Cloth)

RIGEXEC_DEFINE_PORT_HANDLE(ClothPoints)
RIGEXEC_DEFINE_PORT_HANDLE(ClothParameters)
RIGEXEC_DEFINE_PORT_HANDLE(ClothColliders)
RIGEXEC_DEFINE_PORT_HANDLE(ClothEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(ClothOut)
RIGEXEC_DEFINE_PORT_HANDLE(ClothStatusOut)

namespace {

using namespace opIo;
using rigExec::RigExecClothPrepared;
using rigExec::RigExecWindowPacket;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

/// A node's prepared compiled rig, built on its first solve from the
/// payload, which does not change for the node's life.
struct ClothState {
    std::shared_ptr<const RigExecClothPrepared> prepared;
    bool unusable = false;
};

const RigExecClothPrepared *
PreparedOf(const ClothPayload &p, ClothState &state)
{
    if (!state.prepared && !state.unusable) {
        auto prepared = std::make_shared<RigExecClothPrepared>();
        if (rigExec::RigExecPrepareCloth(p.compiledRig, prepared.get(),
                                         nullptr))
            state.prepared = std::move(prepared);
        else
            state.unusable = true;
    }
    return state.prepared.get();
}

bool
Run(const void *payload, void *state, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const ClothPayload &p = *static_cast<const ClothPayload *>(payload);
    const Points &in = *In<Points>(io, ClothPoints());
    Points &out = Out<Points>(io, ClothOut());
    uint32_t &status = Out<uint32_t>(io, ClothStatusOut());
    const auto finish = [&](ClothStatus why) {
        out = in;
        status = uint32_t(why);
        return true;
    };
    if (!io.window)
        return Fail(error, "Cloth", "runs only with a window");
    rigExec::RigExecWindowPacketPtr &capture = *io.window->capture;
    capture.reset();

    const Envelope &parameters = *In<Envelope>(io, ClothParameters());
    if (parameters.Size() != rigExec::RigExecClothParameterCount)
        return finish(ClothStatus::NotRecorded);
    const InList<Points> colliders(io, ClothColliders());
    std::vector<const float *> colliderPoints(colliders.Size());
    std::vector<size_t> colliderCounts(colliders.Size());
    for (size_t k = 0; k < colliders.Size(); ++k) {
        colliderPoints[k] =
            reinterpret_cast<const float *>(colliders[k].Data());
        colliderCounts[k] = colliders[k].Size();
    }
    auto packet = std::make_shared<RigExecWindowPacket>();
    rigExec::RigExecClothCapturePacket(
        reinterpret_cast<const float *>(in.Data()), in.Size(),
        colliderPoints.data(), colliderCounts.data(), colliderPoints.size(),
        parameters.Data(), packet.get());
    capture = std::move(packet);
    if (io.window->captureOnly)
        return finish(ClothStatus::PassedThrough);

    const RigExecClothPrepared *prepared =
        PreparedOf(p, *static_cast<ClothState *>(state));
    if (!prepared || in.Size() != prepared->compiledRig.pointCount)
        return finish(ClothStatus::Failed);
    std::vector<const RigExecWindowPacket *> ticks;
    rigExec::RigExecWindowUsableTail(*io.window->ring, capture.get(), &ticks);
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecClothSolvePackets(
            *prepared, ticks.data(), ticks.size(),
            reinterpret_cast<float *>(points.data()), nullptr) ||
        !AllFinite(points))
        return finish(ClothStatus::Failed);
    status = uint32_t(ClothStatus::Solved);
    return WritePoints(io, ClothOut(), in, points,
                       In<Envelope>(io, ClothEnvelope()), "Cloth", error);
}

uint32_t
WindowFrames(const void *payload)
{
    return payload ? uint32_t(rigExec::RigExecClothHistoryFrames(
                         static_cast<const ClothPayload *>(payload)
                             ->compiledRig.settings))
                   : 0;
}

} // namespace

namespace {

using rigExec::kRigExecPortArray;
using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&ClothPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&ClothParameters(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "parameters"},
    {&ClothColliders(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | kRigExecPortArray | kRigExecPortOptional, "colliders"},
    {&ClothEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&ClothOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &ClothPoints()},
    {&ClothStatusOut(), &rigExecStd::UIntType(), kRigExecPortOutput, "status"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const ClothPayload &p)
{
    w.Version(1);
    std::vector<uint8_t> bytes;
    rigExec::RigExecEncodeClothCompiledData(p.compiledRig, &bytes);
    payloadIo::PutList(w, bytes);
}

void
Load(Reader &r, ClothPayload &p)
{
    r.Version(1);
    std::vector<uint8_t> bytes;
    payloadIo::GetList(r, bytes);
    if (!r.Ok())
        return;
    bool legacy = false;
    if (!rigExec::RigExecDecodeClothCompiledData(
            bytes.data(), bytes.size(), &p.compiledRig, nullptr, &legacy) ||
        legacy)
        r.Fail("not a cloth compiled rig");
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Cloth";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.threading = rigExec::RigExecOpThreading::Windowed;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<ClothPayload, &Save, &Load>();
    d.state.size = sizeof(ClothState);
    d.state.align = alignof(ClothState);
    d.state.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<ClothState *>(dst), 1);
    };
    d.state.copy = [](void *dst, const void *src) {
        std::uninitialized_copy_n(static_cast<const ClothState *>(src), 1,
                                  static_cast<ClothState *>(dst));
    };
    d.state.reset = [](void *dst) {
        *static_cast<ClothState *>(dst) = ClothState();
    };
    d.state.destroy = [](void *dst) {
        std::destroy_at(static_cast<ClothState *>(dst));
    };
    d.windowFrames = &WindowFrames;
    return d;
}();

} // namespace

void
RegisterClothOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Cloth(), kDesc);
}

} // namespace rigExecOps
