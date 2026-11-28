// The fixture mover as an op plugin: the Quadratic op deforms with the
// mover's own kernel (kernel.h) as a whole range member, and its binder maps
// the mover's declared inputs onto the op's inputs at Build.
#include "kernel.h"
#include "rigExec/api/opPorts.h"
#include "rigExec/api/registry.h"
#include "rigExecScene/externalOps.h"
#include "rigExecStdTypes/stdTypes.h"

#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace {

using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;

RIGEXEC_DEFINE_OP_HANDLE(Quadratic)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticPoints)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticGain)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticReference)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticBase)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticOut)

bool
Fail(std::string *error, const char *why)
{
    if (error)
        *error = std::string("Quadratic: ") + why;
    return false;
}

/// The points gathered from their groups, kept from frame to frame.
struct State {
    std::vector<Vec3f> points;
};

/// The gain, when the host writes it: the mover's payload sentinel for a
/// result of another point count, or a negative gain.
bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const auto *gain = static_cast<const float *>(
        check.batch
            .inputs[check.k * check.batch.portCount + QuadraticGain().Index()]);
    if (gain && *gain == -2.0f)
        return Fail(error, "the result changes the point count");
    if (gain && *gain < 0.0f)
        return Fail(error, "a negative gain");
    return true;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &, size_t)
{
    return rigExec::RigExecRangePlan::Whole;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
        std::string *error)
{
    using rigExec::opPorts::In;
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    if (In<Points>(io, QuadraticBase())->Size() == 0)
        return Fail(error, "no base points");
    const Points *reference = In<Points>(io, QuadraticReference());
    if (reference && reference->Size() == 0)
        return Fail(error, "an empty reference");
    return true;
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t k,
    const rigExec::RigExecOpRange &range, std::string *error)
{
    using rigExec::opPorts::In;
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    State &state = *static_cast<State *>(rigExec::RigExecBatchState(batch, k));
    const auto &groups = *static_cast<const rigExec::RigExecOpGroups *>(
        range.at[QuadraticPoints().Index()]);
    state.points.resize(range.count);
    for (size_t h = 0; h < groups.groups; ++h) {
        const auto *from = static_cast<const Vec3f *>(groups.points[h]);
        for (int i = groups.bounds[h]; i < groups.bounds[h + 1]; ++i)
            state.points[size_t(i)] = from[i - groups.bounds[h]];
    }
    const float gain = *In<float>(io, QuadraticGain());
    if (gain == -2.0f)
        return Fail(error, "the result changes the point count");
    const Points *reference = In<Points>(io, QuadraticReference());
    if (!rigExec::TestQuadraticDefinition::Deform(
            gain, reference ? (*reference)[0][1] : 0.0f,
            state.points.empty() ? nullptr : &state.points[0][0],
            state.points.size()))
        return Fail(error, "a negative gain");
    auto *out = static_cast<Vec3f *>(range.at[QuadraticOut().Index()]);
    for (size_t i = range.begin; i < range.end; ++i)
        out[i - range.begin] = state.points[i];
    return true;
}

constexpr uint32_t kIn = rigExec::kRigExecPortInput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&QuadraticPoints(), &rigExecStd::Vec3fArray(),
     kIn | rigExec::kRigExecPortNeighbourhood, "points"},
    {&QuadraticGain(), &rigExecStd::FloatType(), kIn, "gain"},
    {&QuadraticReference(), &rigExecStd::Vec3fArray(),
     kIn | rigExec::kRigExecPortOptional, "reference"},
    {&QuadraticBase(), &rigExecStd::Vec3fArray(), kIn, "base"},
    {&QuadraticOut(), &rigExecStd::Vec3fArray(),
     rigExec::kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &QuadraticPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "test";
    d.name = "Quadratic";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.capabilities = rigExec::kRigExecOpRanged;
    d.validate = &Validate;
    d.range.plan = &Plan;
    d.range.prepare = &Prepare;
    d.range.run = &Run;
    d.state.size = sizeof(State);
    d.state.align = alignof(State);
    d.state.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<State *>(dst), 1);
    };
    d.state.copy = [](void *dst, const void *src) {
        std::uninitialized_copy_n(static_cast<const State *>(src), 1,
                                  static_cast<State *>(dst));
    };
    d.state.reset = [](void *dst) { *static_cast<State *>(dst) = State(); };
    d.state.destroy = [](void *dst) {
        std::destroy_at(static_cast<State *>(dst));
    };
    return d;
}();

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Quadratic(), kDesc);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "testQuadraticOps";
    d.registerOps = &Register;
    return d;
}();

/// The mover's declared inputs are its gain, then an optional reference
/// points array; it measures against its target's base points.
bool
Build(const rigExec::RigExecRevisionBinding &binding,
      rigExec::RigExecRigDescBuilder &builder,
      rigExec::RigExecExternalOpPlan &plan, std::string *)
{
    using Source = rigExec::RigExecExternalOpSource;
    plan.node = builder.AddOp(Quadratic());
    plan.out = &QuadraticOut();
    plan.feeds = {{&QuadraticPoints(), Source::Points, 0},
                  {&QuadraticGain(), Source::Input, 0},
                  {&QuadraticBase(), Source::BasePoints, 0}};
    if (binding.externalInputs.size() == 2)
        plan.feeds.push_back({&QuadraticReference(), Source::Input, 1});
    return true;
}

const rigExec::RigExecExternalOpBinder kBinders[] = {
    {sizeof(rigExec::RigExecExternalOpBinder), "ExternalQuadraticMover",
     &Quadratic, &Build},
};

const rigExec::RigExecExternalOpBinderTable kBinderTable = {
    rigExec::kRigExecExternalOpBinderAbiVersion, kBinders, std::size(kBinders)};

} // namespace

RIGEXEC_OP_LIBRARY_ENTRY(testQuadraticOps)
{
    return &kLibrary;
}

RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY(testQuadraticOps)
{
    return &kBinderTable;
}
