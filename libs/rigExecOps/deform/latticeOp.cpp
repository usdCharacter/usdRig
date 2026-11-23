#include "rigExecOps/deform/latticeOp.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

#include <iterator>
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
using namespace opIo;

Mat4d
MatrixOr1(const Mat4d *m)
{
    return m ? *m : Mat4d(1.0);
}

bool
Run(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const LatticePayload &p = *static_cast<const LatticePayload *>(payload);
    const Points &in = *In<Points>(io, LatticePoints());
    const Points &posedCage = *In<Points>(io, LatticePosedCage());
    std::vector<Vec3f> points(in.begin(), in.end());
    if (p.settings.regularGrid) {
        if (!rigExec::RigExecApplyLatticeGridKernel<Vec3f, Vec3d>(
                &points, std::vector<Vec3f>(posedCage.begin(), posedCage.end()),
                p.divisions, p.settings,
                MatrixOr1(In<Mat4d>(io, LatticeTargetToLattice())),
                MatrixOr1(In<Mat4d>(io, LatticeLatticeToTarget())),
                MatrixOr1(In<Mat4d>(io, LatticeCageToLattice()))))
            return Fail(error, "Lattice",
                        "the grid's settings, cage or spaces are "
                        "unusable");
    } else {
        const Points *rest = In<Points>(io, LatticeRestPoints());
        const Points *restCage = In<Points>(io, LatticeRestCage());
        if (!rest || !restCage)
            return Fail(error, "Lattice",
                        "a bound lattice needs rest points and a "
                        "rest cage");
        if (rest->Size() != in.Size())
            return Fail(error, "Lattice",
                        "rest points differ in count from the points");
        // No retained bind factors yet: each run streams them.
        rigExec::RigExecApplyLatticeKernelRange(
            &points, 0, points.size(), rest->Data(), rest->Size(),
            restCage->Data(), restCage->Size(), posedCage.Data(),
            posedCage.Size(), p.divisions[0], p.divisions[1], p.divisions[2],
            nullptr);
    }
    return WritePoints(io, LatticeOut(), in, points,
                       In<Envelope>(io, LatticeEnvelope()), "Lattice", error);
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&LatticePoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&LatticeRestPoints(), &rigExecStd::Vec3fArray(), kOptional, "restPoints"},
    {&LatticeRestCage(), &rigExecStd::Vec3fArray(), kOptional, "restCage"},
    {&LatticePosedCage(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "posedCage"},
    {&LatticeTargetToLattice(), &rigExecStd::Mat4dType(), kOptional,
     "targetToLattice"},
    {&LatticeLatticeToTarget(), &rigExecStd::Mat4dType(), kOptional,
     "latticeToTarget"},
    {&LatticeCageToLattice(), &rigExecStd::Mat4dType(), kOptional,
     "cageToLattice"},
    {&LatticeEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
    {&LatticeOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const LatticePayload &p)
{
    w.Version(1);
    for (int d : p.divisions)
        w.I32(d);
    const rigExec::RigExecLatticeSettings &s = p.settings;
    w.Bool(s.regularGrid);
    for (rigExec::RigExecLatticeInterpolation i : s.interpolation)
        w.Enum(i);
    for (float v : s.origin)
        w.F32(v);
    for (float v : s.spacing)
        w.F32(v);
    w.F32(s.strength);
    payloadIo::PutList(w, s.mask);
}

void
Load(Reader &r, LatticePayload &p)
{
    r.Version(1);
    for (int &d : p.divisions)
        d = r.I32();
    rigExec::RigExecLatticeSettings &s = p.settings;
    s.regularGrid = r.Bool();
    for (rigExec::RigExecLatticeInterpolation &i : s.interpolation)
        i = r.Enum(rigExec::RigExecLatticeInterpolation::CatmullRom);
    for (float &v : s.origin)
        v = r.F32();
    for (float &v : s.spacing)
        v = r.F32();
    s.strength = r.F32();
    payloadIo::GetList(r, s.mask);
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Lattice";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
        rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<LatticePayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterLatticeOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Lattice(), kDesc);
}

} // namespace rigExecOps
