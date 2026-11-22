#include "rigExecOps/deform/latticeOp.h"

#include "rigExecMath/envelope.h"

#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Lattice)

RIGEXEC_DEFINE_PORT_HANDLE(LatticePoints)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeRestPoints)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeRestCage)
RIGEXEC_DEFINE_PORT_HANDLE(LatticePosedCage)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeTargetToLattice)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeLatticeToTarget)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeCageToLattice)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(LatticeOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;

template <class T>
const T *
In(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return static_cast<const T *>(io.inputs[port.Index()]);
}

Mat4d
MatrixOr1(const Mat4d *m)
{
    return m ? *m : Mat4d(1.0);
}

bool
Fail(std::string *error, const char *why)
{
    if (error)
        *error = std::string("Lattice: ") + why;
    return false;
}

bool
Run(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const LatticePayload &p = *static_cast<const LatticePayload *>(payload);
    const Points &in = *In<Points>(io, LatticePoints());
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    const auto *envelope = In<RigExecArray<float>>(io, LatticeEnvelope());
    if (envelope && envelope->Size() != in.Size())
        return Fail(error, "envelope length differs from the points");
    std::vector<Vec3f> points(in.begin(), in.end());
    if (p.settings.regularGrid) {
        if (!rigExec::RigExecApplyLatticeGridKernel<Vec3f, Vec3d>(
                &points, std::vector<Vec3f>(posedCage.begin(), posedCage.end()),
                p.divisions, p.settings,
                MatrixOr1(In<Mat4d>(io, LatticeTargetToLattice())),
                MatrixOr1(In<Mat4d>(io, LatticeLatticeToTarget())),
                MatrixOr1(In<Mat4d>(io, LatticeCageToLattice()))))
            return Fail(error, "the grid's settings, cage or spaces are "
                               "unusable");
    } else {
        const Points *rest = In<Points>(io, LatticeRestPoints());
        const Points *restCage = In<Points>(io, LatticeRestCage());
        if (!rest || !restCage)
            return Fail(error, "a bound lattice needs rest points and a "
                               "rest cage");
        if (rest->Size() != in.Size())
            return Fail(error, "rest points differ in count from the points");
        // No retained bind factors yet: each run streams them.
        rigExec::RigExecApplyLatticeKernelRange(
            &points, 0, points.size(), rest->Data(), rest->Size(),
            restCage->Data(), restCage->Size(), posedCage.Data(),
            posedCage.Size(), p.divisions[0], p.divisions[1], p.divisions[2],
            nullptr);
    }
    Points &out = *static_cast<Points *>(io.outputs[LatticeOut().Index()]);
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
    std::uninitialized_value_construct_n(static_cast<LatticePayload *>(dst), 1);
}

void
Copy(void *dst, const void *src)
{
    std::uninitialized_copy_n(static_cast<const LatticePayload *>(src), 1,
                              static_cast<LatticePayload *>(dst));
}

void
Destroy(void *dst)
{
    std::destroy_at(static_cast<LatticePayload *>(dst));
}

} // namespace

void
RegisterLatticeOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &m = rigExecStd::Mat4dType();
    static const rigExec::RigExecPortDesc ports[] = {
        {&LatticePoints(), &points, kRigExecPortInput, "points"},
        {&LatticeRestPoints(), &points, kOptional, "restPoints"},
        {&LatticeRestCage(), &points, kOptional, "restCage"},
        {&LatticePosedCage(), &points, kRigExecPortInput, "posedCage"},
        {&LatticeTargetToLattice(), &m, kOptional, "targetToLattice"},
        {&LatticeLatticeToTarget(), &m, kOptional, "latticeToTarget"},
        {&LatticeCageToLattice(), &m, kOptional, "cageToLattice"},
        {&LatticeEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
        {&LatticeOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "Lattice";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload.size = sizeof(LatticePayload);
        d.payload.align = alignof(LatticePayload);
        d.payload.construct = &Construct;
        d.payload.copy = &Copy;
        d.payload.destroy = &Destroy;
        return d;
    }();
    registrar.Add(Lattice(), desc);
}

} // namespace rigExecOps
