#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecMath/deltaMushKernel.h"
#include "rigExecMath/envelope.h"

#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(DeltaMush)

RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushPoints)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushRest)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushCounts)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushIndices)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushComputationToTarget)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;

template <class T>
const T *
In(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return static_cast<const T *>(io.inputs[port.Index()]);
}

template <class T>
std::vector<T>
ToVector(const RigExecArray<T> &a)
{
    return std::vector<T>(a.begin(), a.end());
}

bool
Fail(std::string *error, const char *why)
{
    if (error)
        *error = std::string("DeltaMush: ") + why;
    return false;
}

bool
Run(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const DeltaMushPayload &p = *static_cast<const DeltaMushPayload *>(payload);
    const Points &in = *In<Points>(io, DeltaMushPoints());
    const auto *envelope = In<RigExecArray<float>>(io, DeltaMushEnvelope());
    if (envelope && envelope->Size() != in.Size())
        return Fail(error, "envelope length differs from the points");
    const Mat4d *map = In<Mat4d>(io, DeltaMushComputationToTarget());
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplyDeltaMushInSpaceKernel<Vec3f, rigExecStd::Vec3d,
                                                     Mat4d>(
            &points, ToVector(*In<Points>(io, DeltaMushRest())),
            ToVector(*In<RigExecArray<int>>(io, DeltaMushCounts())),
            ToVector(*In<RigExecArray<int>>(io, DeltaMushIndices())),
            p.iterations, p.step, p.pinBorders, p.distanceWeight,
            p.displacement, p.settings, map ? *map : Mat4d(1.0)))
        return Fail(error, "unusable topology, rest or computation map");
    Points &out = *static_cast<Points *>(io.outputs[DeltaMushOut().Index()]);
    out = Points(points.data(), points.size());
    if (envelope)
        for (size_t i = 0; i < in.Size(); ++i)
            out[i] =
                rigExec::RigExecBlendEnvelope(in[i], out[i], (*envelope)[i]);
    return true;
}

void
Construct(void *dst)
{
    std::uninitialized_value_construct_n(static_cast<DeltaMushPayload *>(dst),
                                         1);
}

void
Copy(void *dst, const void *src)
{
    std::uninitialized_copy_n(static_cast<const DeltaMushPayload *>(src), 1,
                              static_cast<DeltaMushPayload *>(dst));
}

void
Destroy(void *dst)
{
    std::destroy_at(static_cast<DeltaMushPayload *>(dst));
}

} // namespace

void
RegisterDeltaMushOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    static const rigExec::RigExecPortDesc ports[] = {
        {&DeltaMushPoints(), &points, kRigExecPortInput, "points"},
        {&DeltaMushRest(), &points, kRigExecPortInput, "restPoints"},
        {&DeltaMushCounts(), &ints, kRigExecPortInput, "faceVertexCounts"},
        {&DeltaMushIndices(), &ints, kRigExecPortInput, "faceVertexIndices"},
        {&DeltaMushComputationToTarget(), &rigExecStd::Mat4dType(), kOptional,
         "computationToTarget"},
        {&DeltaMushEnvelope(), &rigExecStd::FloatArray(), kOptional,
         "envelope"},
        {&DeltaMushOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "DeltaMush";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload.size = sizeof(DeltaMushPayload);
        d.payload.align = alignof(DeltaMushPayload);
        d.payload.construct = &Construct;
        d.payload.copy = &Copy;
        d.payload.destroy = &Destroy;
        return d;
    }();
    registrar.Add(DeltaMush(), desc);
}

} // namespace rigExecOps
