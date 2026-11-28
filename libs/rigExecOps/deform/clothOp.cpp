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

namespace {

using namespace opIo;
using rigExec::RigExecClothPrepared;
using rigExec::RigExecWindowPacket;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

constexpr char kParameterCount[] =
    "needs RigExecClothParameterCount parameters in packet order";
constexpr char kPointCount[] = "its points differ from its compiled rig's";
constexpr char kUnusable[] = "its compiled rig is unusable";

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
    if (!io.window)
        return Fail(error, "Cloth", "runs only with a window");
    rigExec::RigExecWindowPacketPtr &capture = *io.window->capture;
    capture.reset();

    const Envelope &parameters = *In<Envelope>(io, ClothParameters());
    if (parameters.Size() != rigExec::RigExecClothParameterCount)
        return Fail(error, "Cloth", kParameterCount);
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
    if (io.window->captureOnly) {
        Out<Points>(io, ClothOut()) = in;
        return true;
    }

    const RigExecClothPrepared *prepared =
        PreparedOf(p, *static_cast<ClothState *>(state));
    if (!prepared)
        return Fail(error, "Cloth", kUnusable);
    if (in.Size() != prepared->compiledRig.pointCount)
        return Fail(error, "Cloth", kPointCount);
    std::vector<const RigExecWindowPacket *> ticks;
    rigExec::RigExecWindowUsableTail(*io.window->ring, capture.get(), &ticks);
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecClothSolvePackets(
            *prepared, ticks.data(), ticks.size(),
            reinterpret_cast<float *>(points.data()), nullptr) ||
        !AllFinite(points))
        return Fail(error, "Cloth",
                    "the solve failed or gave a point that is not finite");
    return WritePoints(io, ClothOut(), in, points,
                       In<Envelope>(io, ClothEnvelope()), "Cloth", error);
}

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const auto &p = *static_cast<const ClothPayload *>(
        rigExec::RigExecBatchPayload(check.batch, check.k));
    if (!SameCount(CheckElements(check, ClothParameters()),
                   rigExec::RigExecClothParameterCount))
        return Fail(error, "Cloth", kParameterCount);
    if (!SameCount(CheckElements(check, ClothPoints()),
                   p.compiledRig.pointCount))
        return Fail(error, "Cloth", kPointCount);
    if (ClothState *state = CheckState<ClothState>(check);
        state && !PreparedOf(p, *state))
        return Fail(error, "Cloth", kUnusable);
    return EnvelopeFits(check, ClothPoints(), ClothEnvelope(), "Cloth", error);
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
    d.validate = &Check;
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
