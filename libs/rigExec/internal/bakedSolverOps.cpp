#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"

#include <string>
#include <vector>

namespace rigExec {

namespace {

using Solver = RigExecBakedProgramImpl::Solver;
using Landmarks = rigExecStd::RigExecArray<rigExecRig::PointLandmarks>;
using Read = eval::OpSolverRead;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

bool
Is(const RigExecTypeDesc &type, const RigExecTypeHandle &handle)
{
    return RigExecRegistry::Get().TypeAt(handle.Index()).desc == &type;
}

rigExecRig::PointLandmarks
ToOp(const std::array<GfVec3d, 4> &points)
{
    rigExecRig::PointLandmarks out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = rigExecStd::Vec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

rigExecRig::PointFrame
ToOp(const RigExecPointFrame &f)
{
    rigExecRig::PointFrame out;
    out.points = ToOp(f.points);
    out.flags = f.flags;
    return out;
}

std::array<GfVec3d, 4>
FromOp(const rigExecRig::PointLandmarks &points)
{
    std::array<GfVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = GfVec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

RigExecPointFrame
FromOp(const rigExecRig::PointFrame &f)
{
    RigExecPointFrame out;
    out.points = FromOp(f.points);
    out.flags = f.flags;
    return out;
}

/// The frames \p s's op output leads with that are no joint of its
/// aggregate: an FK chain's start frame.
size_t
LeadingFrames(const Solver &s)
{
    return s.type == SchemaTypeName::RigExecFkChain && s.start >= 0 ? 1 : 0;
}

} // namespace

int
RigExecBakedSolverOf(const RigExecBakedProgramImpl &B,
                     const RigExecBakedStep &step)
{
    if (step.kind == RigExecBakedStepKind::Solve)
        return step.object;
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::Solver
               ? origin->object
               : -1;
}

size_t
RigExecBakedSolverNodeCount(const RigExecBakedProgramImpl::Solver &s)
{
    if (s.type != SchemaTypeName::RigExecFkChain)
        return 0;
    return s.jointGroups.empty() ? 1 : 2;
}

bool
RigExecBakedAddSolverOp(const RigExecBakedProgramImpl &B,
                        RigExecRigDescBuilder *builder,
                        eval::OpStepLayout *layout, size_t o,
                        std::string *error)
{
    using namespace rigExecOps;
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    const uint32_t si = uint32_t(origin.object);
    const Solver &s = B.solvers[si];
    const auto look = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          const eval::OpLookup &lookup) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(lookup);
    };
    const auto read = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          Read part) {
        look(
            node, port,
            {uint8_t(eval::OpLookupSource::SolverInput), si, int32_t(part), 0});
    };
    const auto publish = [&](RigExecNodeId node,
                             const RigExecPortHandle &port) {
        builder->BindOutput(node, port);
        layout->hostOutputs.push_back(
            {node, port.Index(), uint8_t(eval::OpOutputTarget::SolverFrames),
             si});
    };
    const auto placed = [&](RigExecNodeId node) {
        return node == o ||
               Fail(error, s.pathText + ": its op node " + std::to_string(o) +
                               " was added as node " + std::to_string(node));
    };
    const bool start = s.start >= 0;
    const size_t nodes = RigExecBakedSolverNodeCount(s);
    if ((origin.part >= 0 ? size_t(origin.part) : nodes - 1) == 0) {
        FkChainPayload p;
        const int base = start ? 1 : 0;
        if (base)
            p.parentIndices.push_back(-1);
        for (size_t k = 0; k < s.controls.size(); ++k)
            p.parentIndices.push_back(s.parentRelative ? base - 1
                                                       : int(k) + base - 1);
        p.scaleSegments = s.scaleSegments;
        const RigExecNodeId node = builder->AddOp(FkChain(), &p);
        if (!placed(node))
            return false;
        read(node, FkChainControls(), Read::FkControls);
        read(node, FkChainOutRests(), Read::FkOutRests);
        if (s.jointGroups.empty())
            publish(node, FkChainOut());
        return true;
    }
    FkJointGroupsPayload p;
    p.jointBegin = start ? 1 : 0;
    p.parents = s.jointGroupParents;
    p.groups = s.jointGroups;
    p.members = s.jointGroupMembers;
    const RigExecNodeId node = builder->AddOp(FkJointGroups(), &p);
    if (!placed(node))
        return false;
    builder->Connect(node - 1, FkChainOut(), node, FkJointGroupsIn());
    read(node, FkJointGroupsSelectors(), Read::GroupSelectors);
    if (start) {
        look(node, FkJointGroupsStart(),
             {uint8_t(eval::OpLookupSource::PoseVersion), s.startRead, 0, 0});
        read(node, FkJointGroupsStartRest(), Read::StartRest);
    }
    publish(node, FkJointGroupsOut());
    return true;
}

bool
RigExecBakedLookUpSolver(const RigExecBakedProgramImpl &B,
                         const eval::OpLookup &lookup,
                         const RigExecTypeDesc &type, void *value,
                         std::string *error)
{
    if (lookup.object >= B.solvers.size())
        return Fail(error, "solver " + std::to_string(lookup.object) +
                               " is no solver of the program");
    const Solver &s = B.solvers[lookup.object];
    const size_t base = s.start >= 0 ? 1 : 0, n = s.controls.size();
    const auto rest = [&](int slot, rigExecRig::PointLandmarks *out) {
        if (slot < 0 || size_t(slot) >= B.restPts.size())
            return Fail(error, s.pathText + " names no rest slot " +
                                   std::to_string(slot));
        *out = ToOp(B.restPts[size_t(slot)]);
        return true;
    };
    const auto frame = [&](uint32_t version, rigExecRig::PointFrame *out) {
        if (version >= B.fin.size())
            return Fail(error, "pose version " + std::to_string(version) +
                                   " is no pose frame of the program");
        *out = ToOp(B.fin[version]);
        return true;
    };
    switch (Read(lookup.part)) {
    case Read::FkControls: {
        if (!Is(type, rigExecRig::PointFrameArrayType()) ||
            s.controlReads.size() != n)
            break;
        auto &out = *static_cast<rigExecRig::PointFrameArray *>(value);
        out.frames.Resize(n + base);
        out.rests.Resize(n + base);
        if (base && (!frame(s.startRead, &out.frames[0]) ||
                     !rest(s.start, &out.rests[0])))
            return false;
        for (size_t k = 0; k < n; ++k)
            if (!frame(s.controlReads[k], &out.frames[k + base]) ||
                !rest(s.controls[k], &out.rests[k + base]))
                return false;
        return true;
    }
    case Read::FkOutRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(n + base);
        if (base && !rest(s.start, &out[0]))
            return false;
        const bool jointBasis = s.jointRests.size() == n;
        for (size_t k = 0; k < n; ++k) {
            if (!jointBasis || k >= s.restIsLive.size() || !s.restIsLive[k]) {
                if (!rest(s.controls[k], &out[k + base]))
                    return false;
                continue;
            }
            if (k >= s.restReads.size() || s.restReads[k] >= B.fin.size())
                return Fail(error, s.pathText + " names no live rest " +
                                       std::to_string(k));
            out[k + base] = ToOp(B.fin[s.restReads[k]].points);
        }
        return true;
    }
    case Read::StartRest:
        if (!Is(type, rigExecRig::PointLandmarksType()))
            break;
        return rest(s.start, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::GroupSelectors: {
        if (!Is(type, rigExecStd::DoubleArray()))
            break;
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(s.jointGroupSelectors.size());
        for (size_t g = 0; g < s.jointGroupSelectors.size(); ++g)
            out[g] = RigExecBakedLeafRead(B, s.jointGroupSelectors[g]);
        return true;
    }
    }
    return Fail(error, s.pathText + " has no input " +
                           std::to_string(lookup.part) + " of that type");
}

bool
RigExecBakedStoreSolver(RigExecBakedProgramImpl &B,
                        const eval::OpHostOutput &output, const void *value,
                        std::string *error)
{
    if (output.object >= B.solvers.size() ||
        output.object >= B.aggregates.size())
        return Fail(error, "solver " + std::to_string(output.object) +
                               " is no solver of the program");
    Solver &s = B.solvers[output.object];
    const auto &out = *static_cast<const rigExecRig::PointFrameArray *>(value);
    const size_t first = LeadingFrames(s);
    RigExecPointFrameArray &aggregate = B.aggregates[output.object];
    const size_t frames =
        out.frames.Size() > first ? out.frames.Size() - first : 0;
    const size_t rests =
        out.rests.Size() > first ? out.rests.Size() - first : 0;
    aggregate.frames.resize(frames);
    aggregate.rests.resize(rests);
    for (size_t k = 0; k < frames; ++k)
        aggregate.frames[k] = FromOp(out.frames[k + first]);
    for (size_t k = 0; k < rests; ++k)
        aggregate.rests[k] = FromOp(out.rests[k + first]);
    s.fallbackSlots.clear();
    for (size_t k = 0; k < s.outputs.size(); ++k) {
        const auto &[slot, element] = s.outputs[k];
        if (element < 0 || size_t(element) >= aggregate.GetSize()) {
            s.outPresent[k] = 0;
            s.fallbackSlots.push_back(slot);
            continue;
        }
        s.outFrames[k] =
            RigExecExtractElementFrame(&aggregate, size_t(element));
        s.outPresent[k] = 1;
    }
    return true;
}

std::string
RigExecBakedSolverFailure(const RigExecBakedProgramImpl &B, int solver,
                          const std::string &error)
{
    if (solver < 0 || size_t(solver) >= B.solvers.size())
        return error;
    return B.solvers[size_t(solver)].pathText + ": " + error;
}

} // namespace rigExec
