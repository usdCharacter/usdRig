#include "rigExec/internal/bakedProgramImpl.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/opFields.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/compile/rigDescData.h"
#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecEvalCore/propertyLookups.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecStdTypes/array.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <tuple>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

using D = RigExecBakedSlotDomain;
using K = RigExecBakedStepKind;

std::function<bool(const RigExecBakedProgramImpl &, RigExecBakedOpInjection *)>
    gOpInjector;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// What the op steps read from the baked program and store in it.
struct BakedOpHost {
    RigExecBakedProgramImpl &B;
    RigExecBakedStep *step = nullptr;
    mutable eval::ConstraintStepCache constraintCache;

    const void *Matrix(eval::OpHostSource source, uint32_t slot) const
    {
        const std::vector<GfMatrix4d> &matrices =
            source == eval::OpHostSource::FinalMatrix ? B.finalMatrix
                                                      : B.baseMatrix;
        return slot < matrices.size() ? matrices[slot].data() : nullptr;
    }
    const void *PresentMatrix(const eval::OpLookup &lookup) const
    {
        const bool record =
            lookup.source == uint8_t(eval::OpLookupSource::FrameRecord);
        const std::vector<GfMatrix4d> &matrices =
            record ? B.frameMatrix : B.deltaValues;
        const std::vector<char> &present =
            record ? B.frameMatrixValid : B.deltaPresent;
        return lookup.object < matrices.size() &&
                       lookup.object < present.size() && present[lookup.object]
                   ? matrices[lookup.object].data()
                   : nullptr;
    }
    bool RevisionLeaf(uint32_t id, bool *enabled, float *weight, bool *radial,
                      std::string *error) const
    {
        return RigExecBakedRevisionLeaf(B, id, enabled, weight, radial, error);
    }
    bool LookUpRevision(uint32_t index, const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *value,
                        std::string *error) const
    {
        return RigExecBakedLookUpRevision(B, index, lookup, type, value, error);
    }
    bool StoreRevisionEnabled(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RigExecBakedStoreRevisionEnabled(
            B, output.object, *static_cast<const uint8_t *>(value) != 0, error);
    }
    const void *ChainGroupPoints(uint32_t chain, uint32_t g) const
    {
        if (chain >= B.chains.size())
            return nullptr;
        const RigExecBakedProgramImpl::GeomChain &c = B.chains[chain];
        return RigExecBakedGroupAt(c, c.revisions.size(), g).data;
    }
    const void *ChainVersionGroupPoints(const eval::OpLookup &lookup,
                                        uint32_t g) const
    {
        if (lookup.object >= B.chains.size() || lookup.part < 0 ||
            size_t(lookup.part) > B.chains[lookup.object].revisions.size())
            return nullptr;
        return RigExecBakedGroupAt(B.chains[lookup.object], size_t(lookup.part),
                                   g)
            .data;
    }
    bool LookUpProperty(uint32_t index, const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *value,
                        std::string *error) const
    {
        return RigExecBakedLookUpProperty(B, lookup, index, type, value, error);
    }
    bool StorePropertyVersion(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RigExecBakedStorePropertyVersion(B, output, value, error);
    }
    bool LookUpPoseVersion(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RigExecBakedLookUpPoseVersion(B, lookup, type, value, error);
    }
    bool StoreVolumePlacement(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RigExecBakedStoreVolumePlacement(B, output, value, error);
    }
    bool LookUpWeight(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RigExecBakedLookUpWeight(B, lookup, type, value, error);
    }
    bool StoreWeightPacket(const eval::OpHostOutput &output, const void *value,
                           std::string *error) const
    {
        return RigExecBakedStoreWeightPacket(B, output, value, error);
    }
    bool LookUpChainVersion(uint32_t index, const eval::OpLookup &lookup,
                            const RigExecTypeDesc &type, void *value,
                            std::string *error) const
    {
        return RigExecBakedLookUpChainVersion(B, index, lookup, type, value,
                                              error);
    }
    bool LookUpWeightField(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RigExecBakedLookUpWeightField(B, lookup, type, value, error);
    }
    bool StoreWeightField(const eval::OpHostOutput &output, const void *value,
                          std::string *error) const
    {
        return RigExecBakedStoreWeightField(B, output, value, error);
    }
    bool LookUpSpace(uint32_t, const eval::OpLookup &lookup,
                     const RigExecTypeDesc &type, void *value,
                     std::string *error) const
    {
        return RigExecBakedLookUpSpace(B, lookup, type, value, error);
    }
    bool StoreSpace(const eval::OpHostOutput &output, const void *value,
                    std::string *error) const
    {
        return RigExecBakedStoreSpace(B, step, output, value, error);
    }
    bool StoreGroup(const eval::OpHostOutput &output, uint32_t g,
                    const void *elements, size_t count,
                    std::string *error) const
    {
        return RigExecBakedStoreChainGroup(B, output, g, elements, count,
                                           error);
    }
    bool LookUpCommit(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RigExecBakedLookUpCommit(B, lookup, type, value, error);
    }
    bool StoreCommit(const eval::OpHostOutput &output, const void *value,
                     std::string *error) const
    {
        return RigExecBakedStoreCommit(B, output, value, error);
    }
    bool LookUpConstraint(uint32_t, const eval::OpLookup &lookup,
                          const RigExecTypeDesc &type, void *value,
                          std::string *error) const
    {
        return RigExecBakedLookUpConstraint(B, constraintCache, lookup, type,
                                            value, error);
    }
    bool LendWindow(RigExecRunState &run, uint32_t node,
                    std::string *error) const
    {
        return RigExecBakedLendConstraintWindow(B, run, node, error);
    }
    bool StoreConstraint(const eval::OpHostOutput &output, const void *value,
                         std::string *error) const
    {
        return RigExecBakedStoreConstraint(B, step, constraintCache, output,
                                           value, error);
    }
    bool LookUpCompose(uint32_t, const eval::OpLookup &lookup,
                       const RigExecTypeDesc &type, void *value,
                       std::string *error) const
    {
        return RigExecBakedLookUpCompose(B, lookup, type, value, error);
    }
    bool StoreCompose(const eval::OpHostOutput &output, const void *value,
                      std::string *error) const
    {
        return RigExecBakedStoreCompose(B, output, value, error);
    }
};

template <class T>
void
Bytes(const T *data, size_t count, std::vector<uint8_t> *out)
{
    const auto *bytes = reinterpret_cast<const uint8_t *>(data);
    out->assign(bytes, bytes + count * sizeof(T));
}

/// \p value's bytes as an input of standard type \p type reads them; false
/// when \p value holds another type.
bool
ConstantBytes(const VtValue &value, const RigExecTypeDesc &type,
              std::vector<uint8_t> *out)
{
    const std::string name =
        std::string(type.nameSpace) == "std" ? type.name : "";
    if (name == "Float" && value.IsHolding<float>()) {
        Bytes(&value.UncheckedGet<float>(), 1, out);
    } else if (name == "Double" && value.IsHolding<double>()) {
        Bytes(&value.UncheckedGet<double>(), 1, out);
    } else if (name == "Int" && value.IsHolding<int>()) {
        const int32_t v = value.UncheckedGet<int>();
        Bytes(&v, 1, out);
    } else if (name == "Bool" && value.IsHolding<bool>()) {
        const uint8_t v = value.UncheckedGet<bool>() ? 1 : 0;
        Bytes(&v, 1, out);
    } else if (name == "Mat4d" && value.IsHolding<GfMatrix4d>()) {
        Bytes(value.UncheckedGet<GfMatrix4d>().data(), 16, out);
    } else if (name == "IntArray" && value.IsHolding<VtIntArray>()) {
        const VtIntArray &v = value.UncheckedGet<VtIntArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "FloatArray" && value.IsHolding<VtFloatArray>()) {
        const VtFloatArray &v = value.UncheckedGet<VtFloatArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "DoubleArray" && value.IsHolding<VtDoubleArray>()) {
        const VtDoubleArray &v = value.UncheckedGet<VtDoubleArray>();
        Bytes(v.cdata(), v.size(), out);
    } else if (name == "Vec3fArray" && value.IsHolding<VtVec3fArray>()) {
        const VtVec3fArray &v = value.UncheckedGet<VtVec3fArray>();
        Bytes(v.cdata(), v.size(), out);
    } else {
        return false;
    }
    return true;
}

/// Which op step reads: a node's, a chain's head, or a range step over
/// one vertex group or, for a member that runs whole, all of them.
struct OpReader {
    uint32_t chain = kRigExecInvalidIndex;
    uint32_t segment = kRigExecInvalidIndex;
    uint32_t group = kRigExecInvalidIndex;
    bool head = false;
    bool whole = false;
};

/// Per node, its hostInputs entries in layout order: entries start[n] to
/// start[n + 1].
struct HostInputsByNode {
    std::vector<uint32_t> start, entries;
};

HostInputsByNode
IndexHostInputs(const eval::OpStepLayout &layout)
{
    HostInputsByNode index;
    index.start.assign(layout.nodes.size() + 1, 0);
    for (const eval::OpHostInput &input : layout.hostInputs)
        ++index.start[input.node + 1];
    for (size_t n = 0; n < layout.nodes.size(); ++n)
        index.start[n + 1] += index.start[n];
    index.entries.resize(layout.hostInputs.size());
    std::vector<uint32_t> next(index.start.begin(), index.start.end() - 1);
    for (uint32_t i = 0; i < layout.hostInputs.size(); ++i)
        index.entries[next[layout.hostInputs[i].node]++] = i;
    return index;
}

/// What port \p p of node \p n reads in \p reader's step.
void
DeclarePortReads(const RigExecBakedProgramImpl &B,
                 const RigExecBakedOpProgram &op,
                 const HostInputsByNode &inputs, uint32_t n, uint32_t p,
                 const OpReader &reader,
                 std::vector<RigExecBakedSlotRange> *reads)
{
    const eval::OpStepLayout &layout = op.layout;
    const uint32_t chain = reader.chain, segment = reader.segment,
                   group = reader.group;
    const bool whole = reader.whole;
    const RigExecNodeDesc &d = layout.nodes[n];
    const uint32_t flags = d.op->ports[p].flags;
    const auto value = [&](uint32_t v, bool listed) {
        if (v == kRigExecInvalidIndex)
            return;
        const uint32_t producer = layout.producerOf[v];
        if (producer == kRigExecInvalidIndex)
            return;
        const uint32_t from = layout.chainOf[producer];
        if (from == kRigExecInvalidIndex) {
            reads->push_back(RigExecBakedOne(D::OpValue, int(v)));
            return;
        }
        const uint32_t base = layout.groupBase[v];
        if (from == chain) {
            if (reader.head ||
                layout.chainDescs[chain].segments[layout.memberOf[producer]] ==
                    segment)
                return;
            if (!listed && !whole && (flags & kRigExecPortPointwise)) {
                reads->push_back(
                    RigExecBakedOne(D::OpGroupValue, int(base + group)));
                return;
            }
            if (!listed && !whole && (flags & kRigExecPortNeighbourhood)) {
                for (const uint32_t h :
                     op.halo[chain][layout.memberOf[n]][group])
                    reads->push_back(
                        RigExecBakedOne(D::OpGroupValue, int(base + h)));
                return;
            }
        }
        reads->push_back(
            RigExecBakedRange(D::OpGroupValue, int(base),
                              int(base + layout.chains[from].Groups())));
    };
    value(d.values[p], false);
    for (const uint32_t element : d.lists[p])
        value(element, true);
    const auto matrix = [&](const eval::OpLookup &lookup) {
        if (lookup.source == uint8_t(eval::OpLookupSource::Matrix))
            reads->push_back(
                RigExecBakedOne(lookup.part ? D::BaseMatrix : D::FinalMatrix,
                                int(lookup.object)));
        else if (lookup.source == uint8_t(eval::OpLookupSource::FrameRecord))
            reads->push_back(
                RigExecBakedOne(D::FrameMatrix, int(lookup.object)));
        else if (lookup.source ==
                 uint8_t(eval::OpLookupSource::ConstraintDelta))
            reads->push_back(
                RigExecBakedOne(D::ConstraintDelta, int(lookup.object)));
    };
    const auto matrices = [&](const eval::OpLookup &lookup) {
        matrix(lookup);
        if (lookup.source == uint8_t(eval::OpLookupSource::Selected))
            for (int32_t e = 0; e < lookup.part; ++e)
                matrix(layout.lookups[lookup.object + uint32_t(e)]);
    };
    for (uint32_t k = inputs.start[n]; k < inputs.start[n + 1]; ++k) {
        const uint32_t i = inputs.entries[k];
        const eval::OpHostInput &input = layout.hostInputs[i];
        if (input.port != p)
            continue;
        if (input.source == eval::OpHostSource::FinalMatrix) {
            reads->push_back(
                RigExecBakedOne(D::FinalMatrix, int(input.object)));
        } else if (input.source == eval::OpHostSource::BaseMatrix) {
            reads->push_back(RigExecBakedOne(D::BaseMatrix, int(input.object)));
        } else if (layout.grouped[i]) {
            if (reader.head)
                continue;
            const bool latest = input.source == eval::OpHostSource::ChainPoints;
            const uint32_t from =
                latest ? input.object : layout.lookups[input.object].object;
            const size_t version =
                latest ? B.chains[from].revisions.size()
                       : size_t(layout.lookups[input.object].part);
            if (whole) {
                for (uint32_t h = 0; h < layout.chains[chain].Groups(); ++h)
                    RigExecBakedDeclareGroupRead(B, int(from), version, h,
                                                 reads);
                continue;
            }
            RigExecBakedDeclareGroupRead(B, int(from), version, group, reads);
            if (layout.grouped[i] == 2)
                for (const uint32_t h :
                     op.halo[chain][layout.memberOf[n]][group])
                    if (h != group)
                        RigExecBakedDeclareGroupRead(B, int(from), version, h,
                                                     reads);
        } else if (input.source == eval::OpHostSource::Lookup &&
                   (reader.head || chain == kRigExecInvalidIndex)) {
            const eval::OpLookup &lookup = layout.lookups[input.object];
            matrices(lookup);
            if (lookup.source == uint8_t(eval::OpLookupSource::RevisionLeaf) &&
                lookup.read == uint8_t(eval::OpRevisionRead::ChainBase) &&
                lookup.object < B.revisionIndex.size())
                reads->push_back(RigExecBakedOne(
                    D::ChainBase, B.revisionIndex[lookup.object].first));
            if (lookup.source == uint8_t(eval::OpLookupSource::List))
                for (int32_t e = 0; e < lookup.part; ++e)
                    matrices(layout.lookups[lookup.object + uint32_t(e)]);
        }
    }
}

void
DeclareNodeReads(const RigExecBakedProgramImpl &B,
                 const RigExecBakedOpProgram &op,
                 const HostInputsByNode &inputs, uint32_t n,
                 const OpReader &reader,
                 std::vector<RigExecBakedSlotRange> *reads)
{
    const RigExecNodeDesc &d = op.layout.nodes[n];
    for (uint32_t p = 0; p < d.values.size(); ++p)
        if (d.op->ports[p].flags & kRigExecPortInput)
            DeclarePortReads(B, op, inputs, n, p, reader, reads);
}

/// Per chain, member reading an earlier segment through a neighbourhood
/// input, and vertex group: the groups that group reads, from the
/// member's reads after a head over the values Build has; every group when
/// the head cannot run before the first frame.
void
FindHaloGroups(RigExecBakedOpProgram *op, RigExecRunState &run)
{
    const eval::OpStepLayout &layout = op->layout;
    op->halo.assign(layout.chains.size(),
                    std::vector<std::vector<std::vector<uint32_t>>>());
    for (uint32_t c = 0; c < layout.chains.size(); ++c) {
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const std::vector<int> &bounds = layout.chains[c].bounds;
        const uint32_t groups = layout.chains[c].Groups();
        std::string why;
        const bool known = run.RunChainHead(c, bounds.data(), groups, &why);
        op->halo[c].assign(desc.members.size(),
                           std::vector<std::vector<uint32_t>>());
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            bool halo = false;
            for (const RigExecChainDesc::Feed &feed : desc.feeds[m])
                halo = halo || feed.neighbourhood;
            for (const uint32_t i : layout.groupedOf[c])
                halo = halo || (layout.grouped[i] == 2 &&
                                layout.hostInputs[i].node == desc.members[m]);
            if (!halo)
                continue;
            op->halo[c][m].assign(groups, std::vector<uint32_t>());
            for (uint32_t g = 0; g < groups; ++g) {
                size_t lo = 0, hi = SIZE_MAX;
                if (known)
                    run.ChainReads(c, m, g, &lo, &hi);
                for (uint32_t h = 0; h < groups; ++h)
                    if (size_t(bounds[h]) < hi && size_t(bounds[h + 1]) > lo)
                        op->halo[c][m][g].push_back(h);
            }
        }
    }
}

/// Removes the RevisionChunk steps of the revisions \p stored names, whose
/// groups op chains store instead, keeping B's step indices current.
void
DropRevisionChunks(RigExecBakedProgramImpl *B, std::vector<int> stored)
{
    std::sort(stored.begin(), stored.end());
    const auto dropped = [&](const RigExecBakedStep &step) {
        return step.kind == K::RevisionChunk &&
               std::binary_search(stored.begin(), stored.end(), step.object);
    };
    std::vector<int> moved(B->steps.size(), -1);
    int kept = 0;
    for (size_t i = 0; i < B->steps.size(); ++i)
        if (!dropped(B->steps[i]))
            moved[i] = kept++;
    B->steps.erase(std::remove_if(B->steps.begin(), B->steps.end(), dropped),
                   B->steps.end());
    for (int &step : B->revisionFuseStep)
        if (step >= 0)
            step = moved[size_t(step)];
}

/// A member fed node to node by the op revision before it: the two nodes
/// and the member's revision.
struct PointJoin {
    RigExecNodeId from = 0;
    RigExecNodeId to = 0;
    int revision = -1;
};

/// Per Range matrix (GeomRevision::opMatrix): a
/// MoverTransform node fed the matrices FoldInfluences would fold, the
/// transform the first present of a frame record, the constraint delta and
/// the provider table, and a MatrixDeform range chain member; per Range
/// smooth (opSmooth) a Smooth member over the compiled topology; per Range
/// delta mush (opMush) a DeltaMush member over the compiled topology, edges
/// and iteration count, its other inputs looked up per frame and its frame
/// provider selected as a matrix mover's reference; per Range lattice
/// (opLattice) a Lattice member over the divisions read at Build, its rest
/// points the chain's base and its cages looked up per frame. Each member
/// stores its groups and reads the points entering its revision: the
/// output of the op revision before it in its point chain, so the two share
/// one range chain, when that one writes every group or this one reads
/// point by point, and this one's weights do not read the entering points;
/// else the stored version. A revision \p apart names always reads the
/// stored version. \p nodeRevision gets each node's revision id, \p joins
/// each node-to-node feed. False for a matrix with no transform, a delta
/// mush framed by more than one provider or a lattice divided below 2.
bool
AddPointChainOps(const RigExecBakedProgramImpl &B,
                 RigExecRigDescBuilder *builder, eval::OpStepLayout *layout,
                 const std::vector<RigExecBakedOpFeed> &packets,
                 const std::vector<RigExecBakedOpFeed> &fields,
                 const std::vector<int> &apart,
                 std::vector<std::pair<RigExecNodeId, int>> *nodeRevision,
                 std::vector<PointJoin> *joins, std::string *error)
{
    using L = eval::OpLookupSource;
    const auto bind = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          const std::vector<eval::OpLookup> &candidates) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        if (candidates.size() > 1)
            layout->lookups.push_back({uint8_t(L::Selected),
                                       uint32_t(layout->lookups.size() + 1),
                                       int32_t(candidates.size()), 0});
        layout->lookups.insert(layout->lookups.end(), candidates.begin(),
                               candidates.end());
    };
    const auto feed = [](const std::vector<RigExecBakedOpFeed> &feeds, int at) {
        return at >= 0 && size_t(at) < feeds.size() && feeds[size_t(at)].port
                   ? &feeds[size_t(at)]
                   : nullptr;
    };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        RigExecNodeId previous = kRigExecInvalidIndex;
        const RigExecPortHandle *previousOut = nullptr;
        bool previousWritesAll = false;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            if (!revision.OpNodes()) {
                previousOut = nullptr;
                continue;
            }
            const int id = B.chainRevisionBegin[c] + int(r);
            const bool writesAll = std::find(revision.groupWritten.begin(),
                                             revision.groupWritten.end(),
                                             0) == revision.groupWritten.end();
            const auto points = [&](RigExecNodeId node,
                                    const RigExecPortHandle &port,
                                    bool neighbourhood) {
                if (previousOut && (previousWritesAll || !neighbourhood) &&
                    !revision.weightCurrentPhase &&
                    std::find(apart.begin(), apart.end(), id) == apart.end()) {
                    builder->Connect(previous, *previousOut, node, port);
                    joins->push_back({previous, node, id});
                } else
                    bind(node, port,
                         {{uint8_t(L::ChainVersion), uint32_t(c), int32_t(r),
                           0}});
            };
            const auto enabled = [&](RigExecNodeId node,
                                     const RigExecPortHandle &port) {
                bind(node, port,
                     {{uint8_t(L::RevisionLeaf), uint32_t(id), -1,
                       uint8_t(eval::OpRevisionRead::Enabled)}});
                layout->hostOutputs.push_back(
                    {node, port.Index(),
                     uint8_t(eval::OpOutputTarget::RevisionEnabled),
                     uint32_t(id)});
            };
            const auto stores = [&](RigExecNodeId node,
                                    const RigExecPortHandle &out) {
                builder->BindOutput(node, out);
                layout->hostOutputs.push_back(
                    {node, out.Index(),
                     uint8_t(eval::OpOutputTarget::ChainGroup), uint32_t(id)});
                nodeRevision->push_back({node, id});
                previous = node;
                previousOut = &out;
                previousWritesAll = writesAll;
            };
            const int32_t phase = revision.finalPhase ? 0 : 1;
            const bool atPrim = revision.binding.transformPhase.kind ==
                                RigExecReadPhaseKind::AtPrim;
            const auto table = [&](int slot) {
                return eval::OpLookup{uint8_t(L::Matrix), uint32_t(slot), phase,
                                      0};
            };
            const auto phased = [&](const std::vector<int> *records,
                                    eval::OpLookup last) {
                std::vector<eval::OpLookup> candidates;
                if (atPrim && records)
                    for (const int record : *records)
                        candidates.push_back(
                            {uint8_t(L::FrameRecord), uint32_t(record), -1, 0});
                candidates.push_back(last);
                return candidates;
            };
            const auto records = [&](size_t k) {
                return k < revision.influenceRecords.size()
                           ? &revision.influenceRecords[k]
                           : nullptr;
            };
            const auto constant = [&](RigExecNodeId node,
                                      const RigExecPortHandle &port,
                                      const int32_t *ints, size_t count) {
                builder->BindInput(node, port);
                eval::OpConstant bytes;
                bytes.node = node;
                bytes.port = port.Index();
                bytes.bytes.resize(count * sizeof(int32_t));
                if (count)
                    std::memcpy(bytes.bytes.data(), ints, bytes.bytes.size());
                layout->constants.push_back(std::move(bytes));
            };
            // The points the revision writes, as ascending [begin, end) pairs.
            const auto written = [&](RigExecNodeId node,
                                     const RigExecPortHandle &port) {
                std::vector<int32_t> ranges;
                for (size_t g = 0; g < revision.groupWritten.size() &&
                                   g + 1 < chain.groupBounds.size();
                     ++g) {
                    if (!revision.groupWritten[g])
                        continue;
                    if (!ranges.empty() &&
                        ranges.back() == chain.groupBounds[g])
                        ranges.back() = chain.groupBounds[g + 1];
                    else
                        ranges.insert(ranges.end(), {chain.groupBounds[g],
                                                     chain.groupBounds[g + 1]});
                }
                constant(node, port, ranges.data(), ranges.size());
            };
            const auto weighs = [&](RigExecNodeId node,
                                    const RigExecPortHandle &envelope,
                                    const RigExecPortHandle &packetPort,
                                    const RigExecPortHandle &defaultWeight) {
                if (revision.weightCurrentPhase) {
                    const RigExecBakedOpFeed *field =
                        feed(fields, revision.weightField);
                    if (!field)
                        return Fail(error, revision.moverPathText +
                                               ": its weight field makes no "
                                               "weights");
                    builder->Connect(field->node, *field->port, node, envelope);
                } else if (revision.weightObject >= 0) {
                    const RigExecBakedOpFeed *packet =
                        feed(packets, revision.weightObject);
                    if (!packet)
                        return Fail(error, revision.moverPathText +
                                               ": its weight object makes no "
                                               "packet");
                    builder->Connect(packet->node, *packet->port, node,
                                     packetPort);
                } else {
                    bind(node, defaultWeight,
                         {{uint8_t(L::RevisionLeaf), uint32_t(id), -1,
                           uint8_t(eval::OpRevisionRead::DefaultWeight)}});
                }
                return true;
            };
            if (revision.opMush) {
                using Read = eval::OpRevisionRead;
                rigExecOps::DeltaMushPayload payload;
                payload.iterations = revision.opMushIterations;
                payload.settings.edges.assign(revision.opMushEdges.cbegin(),
                                              revision.opMushEdges.cend());
                const RigExecNodeId mush =
                    builder->AddOp(rigExecOps::DeltaMush(), &payload);
                points(mush, rigExecOps::DeltaMushPoints(), true);
                constant(mush, rigExecOps::DeltaMushCounts(),
                         revision.opMushCounts.cdata(),
                         revision.opMushCounts.size());
                constant(mush, rigExecOps::DeltaMushIndices(),
                         revision.opMushIndices.cdata(),
                         revision.opMushIndices.size());
                enabled(mush, rigExecOps::DeltaMushEnabled());
                for (const auto &[port, read] :
                     {std::make_pair(&rigExecOps::DeltaMushRest(),
                                     Read::RestPoints),
                      std::make_pair(&rigExecOps::DeltaMushBase(),
                                     Read::ChainBase),
                      std::make_pair(&rigExecOps::DeltaMushStep(),
                                     Read::MushStep),
                      std::make_pair(&rigExecOps::DeltaMushPinBorders(),
                                     Read::MushPinBorders),
                      std::make_pair(&rigExecOps::DeltaMushDistanceWeight(),
                                     Read::MushDistanceWeight),
                      std::make_pair(&rigExecOps::DeltaMushDisplacement(),
                                     Read::MushDisplacement),
                      std::make_pair(&rigExecOps::DeltaMushSmoothing(),
                                     Read::MushSmoothing),
                      std::make_pair(&rigExecOps::DeltaMushFrameTransport(),
                                     Read::MushFrameTransport),
                      std::make_pair(&rigExecOps::DeltaMushSmoothWeights(),
                                     Read::MushSmoothWeights),
                      std::make_pair(&rigExecOps::DeltaMushOnlySmooth(),
                                     Read::MushOnlySmooth),
                      std::make_pair(
                          &rigExecOps::DeltaMushComputationToTarget(),
                          Read::MushComputationToTarget)})
                    bind(mush, *port,
                         {{uint8_t(L::RevisionLeaf), uint32_t(id), -1,
                           uint8_t(read)}});
                if (revision.influenceSlots.size() > 1)
                    return Fail(error, revision.moverPathText +
                                           ": a delta mush is framed by one "
                                           "provider");
                if (!revision.influenceSlots.empty())
                    bind(mush, rigExecOps::DeltaMushFrame(),
                         phased(records(0), table(revision.influenceSlots[0])));
                if (!weighs(mush, rigExecOps::DeltaMushEnvelope(),
                            rigExecOps::DeltaMushPacket(),
                            rigExecOps::DeltaMushDefaultWeight()))
                    return false;
                if (!writesAll && !revision.weightCurrentPhase &&
                    revision.weightObject >= 0)
                    written(mush, rigExecOps::DeltaMushWritten());
                stores(mush, rigExecOps::DeltaMushOut());
                continue;
            }
            if (revision.opLattice) {
                using Read = eval::OpRevisionRead;
                const std::array<int, 3> &divisions =
                    revision.opLatticeDivisions;
                if (divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2)
                    return Fail(error, revision.moverPathText +
                                           ": a lattice's divisions are "
                                           "below 2");
                rigExecOps::LatticePayload payload;
                payload.divisions = divisions;
                const RigExecNodeId lattice =
                    builder->AddOp(rigExecOps::Lattice(), &payload);
                points(lattice, rigExecOps::LatticePoints(), false);
                enabled(lattice, rigExecOps::LatticeEnabled());
                for (const auto &[port, read] :
                     {std::make_pair(&rigExecOps::LatticeRestPoints(),
                                     Read::ChainBase),
                      std::make_pair(&rigExecOps::LatticeRestCage(),
                                     Read::LatticeRestCage),
                      std::make_pair(&rigExecOps::LatticePosedCage(),
                                     Read::LatticeLiveCage)})
                    bind(lattice, *port,
                         {{uint8_t(L::RevisionLeaf), uint32_t(id), -1,
                           uint8_t(read)}});
                if (!weighs(lattice, rigExecOps::LatticeEnvelope(),
                            rigExecOps::LatticePacket(),
                            rigExecOps::LatticeDefaultWeight()))
                    return false;
                if (!writesAll && !revision.weightCurrentPhase &&
                    revision.weightObject >= 0)
                    written(lattice, rigExecOps::LatticeWritten());
                stores(lattice, rigExecOps::LatticeOut());
                continue;
            }
            if (revision.opSmooth) {
                const RigExecNodeId smooth =
                    builder->AddOp(rigExecOps::Smooth());
                points(smooth, rigExecOps::SmoothPoints(), true);
                constant(smooth, rigExecOps::SmoothCounts(),
                         revision.opSmoothCounts.cdata(),
                         revision.opSmoothCounts.size());
                constant(smooth, rigExecOps::SmoothIndices(),
                         revision.opSmoothIndices.cdata(),
                         revision.opSmoothIndices.size());
                enabled(smooth, rigExecOps::SmoothEnabled());
                if (!weighs(smooth, rigExecOps::SmoothEnvelope(),
                            rigExecOps::SmoothPacket(),
                            rigExecOps::SmoothDefaultWeight()))
                    return false;
                stores(smooth, rigExecOps::SmoothOut());
                continue;
            }
            std::vector<eval::OpLookup> transform;
            if (atPrim)
                for (const int record : revision.transformRecords)
                    transform.push_back(
                        {uint8_t(L::FrameRecord), uint32_t(record), -1, 0});
            if (revision.constraintDelta >= 0)
                transform.push_back({uint8_t(L::ConstraintDelta),
                                     uint32_t(revision.constraintDelta), -1,
                                     0});
            if (revision.transformSlot >= 0)
                transform.push_back(table(revision.transformSlot));
            if (transform.empty())
                return Fail(error, revision.moverPathText +
                                       ": a matrix mover with no transform");
            rigExecOps::MoverTransformPayload fold;
            fold.posedPoints = revision.posedPoints;
            const RigExecNodeId mover =
                builder->AddOp(rigExecOps::MoverTransform(), &fold);
            bind(mover, rigExecOps::MoverTransformTransform(), transform);
            const bool space = revision.transformSpaceSlot >= 0;
            if (space)
                bind(mover, rigExecOps::MoverTransformSpace(),
                     {table(revision.transformSpaceSlot)});
            const std::vector<int> &references = revision.influenceSlots;
            if (!references.empty())
                bind(mover, rigExecOps::MoverTransformReference(),
                     phased(records(0), table(references[0])));
            if (space && references.size() > 1)
                bind(mover, rigExecOps::MoverTransformReferenceSpace(),
                     phased(records(1), table(references[1])));
            if (revision.carrySpaceSlot >= 0)
                bind(mover, rigExecOps::MoverTransformCarry(),
                     {table(revision.carrySpaceSlot)});
            rigExecOps::MatrixDeformPayload payload;
            payload.radial = revision.opMatrixRadial;
            const RigExecNodeId deform =
                builder->AddOp(rigExecOps::MatrixDeform(), &payload);
            builder->Connect(mover, rigExecOps::MoverTransformOut(), deform,
                             rigExecOps::MatrixDeformTransform());
            points(deform, rigExecOps::MatrixDeformPoints(), false);
            enabled(deform, rigExecOps::MatrixDeformEnabled());
            if (revision.opMatrixRadialLeaf)
                bind(deform, rigExecOps::MatrixDeformRadial(),
                     {{uint8_t(L::RevisionLeaf), uint32_t(id), -1,
                       uint8_t(eval::OpRevisionRead::Radial)}});
            if (!writesAll)
                written(deform, rigExecOps::MatrixDeformWritten());
            if (!weighs(deform, rigExecOps::MatrixDeformEnvelope(),
                        rigExecOps::MatrixDeformPacket(),
                        rigExecOps::MatrixDeformDefaultWeight()))
                return false;
            nodeRevision->push_back({mover, id});
            stores(deform, rigExecOps::MatrixDeformOut());
        }
    }
    return true;
}

RigExecBakedStep &
AddOpStep(RigExecBakedProgramImpl *B, K kind, int object, int part)
{
    B->steps.emplace_back();
    RigExecBakedStep &step = B->steps.back();
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 1;
    return step;
}

void
EmitOpSteps(RigExecBakedProgramImpl *B, const RigExecBakedOpProgram &op)
{
    const eval::OpStepLayout &layout = op.layout;
    const HostInputsByNode inputs = IndexHostInputs(layout);
    const auto outputs = [&](uint32_t n, auto &&visit) {
        const RigExecNodeDesc &d = layout.nodes[n];
        for (size_t p = 0; p < d.values.size(); ++p)
            if ((d.op->ports[p].flags & kRigExecPortOutput) &&
                d.values[p] != kRigExecInvalidIndex)
                visit(d.values[p]);
    };
    for (RigExecBakedStep &step : B->steps) {
        const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(*B, step);
        const bool constraint =
            origin &&
            (origin->family == RigExecBakedOpFamily::Constraint ||
             (origin->family == RigExecBakedOpFamily::PoseCommit &&
              RigExecBakedConstraintRunsAsOps(*B, size_t(origin->object))));
        if (!origin || (origin->family != RigExecBakedOpFamily::WeightField &&
                        origin->family != RigExecBakedOpFamily::WeightPacket &&
                        !constraint))
            continue;
        const uint32_t n = uint32_t(step.object);
        if (origin->family == RigExecBakedOpFamily::WeightField || constraint)
            DeclareNodeReads(*B, op, inputs, n, OpReader(), &step.reads);
        outputs(n, [&](uint32_t v) {
            step.writes.push_back(RigExecBakedOne(D::OpValue, int(v)));
        });
    }
    for (uint32_t n = 0; n < layout.nodes.size(); ++n) {
        if (layout.chainOf[n] != kRigExecInvalidIndex ||
            n < B->opNodeOrigins.size())
            continue;
        RigExecBakedStep &step = AddOpStep(B, K::OpNode, int(n), -1);
        DeclareNodeReads(*B, op, inputs, n, OpReader(), &step.reads);
        outputs(n, [&](uint32_t v) {
            step.writes.push_back(RigExecBakedOne(D::OpValue, int(v)));
        });
        if (n < op.nodeRevision.size() && op.nodeRevision[n] >= 0)
            step.writes.push_back(
                RigExecBakedOne(D::RevisionTransforms, op.nodeRevision[n]));
    }
    for (uint32_t c = 0; c < layout.chains.size(); ++c) {
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const uint32_t groups = layout.chains[c].Groups();
        {
            RigExecBakedStep &head = AddOpStep(B, K::OpChainHead, int(c), -1);
            OpReader reader;
            reader.chain = c;
            reader.head = true;
            for (const uint32_t n : desc.members)
                DeclareNodeReads(*B, op, inputs, n, reader, &head.reads);
            head.writes.push_back(
                RigExecBakedOne(D::OpValue, int(layout.PlanSlot(c))));
            int last = -1;
            for (const uint32_t n : desc.members) {
                const int id =
                    n < op.nodeRevision.size() ? op.nodeRevision[n] : -1;
                if (id >= 0 && id != last)
                    head.writes.push_back(
                        RigExecBakedOne(D::RevisionPacket, id));
                last = id >= 0 ? id : last;
            }
        }
        for (uint32_t s = 0; s < desc.members.size(); ++s) {
            if (desc.segments[s] != s)
                continue;
            const bool whole = desc.whole[s] != 0;
            for (uint32_t g = 0; g < (whole ? 1 : groups); ++g) {
                if (!eval::OpSegmentWrites(layout.chains[c], desc, s, g))
                    continue;
                RigExecBakedStep &step =
                    AddOpStep(B, K::OpChainRange, int(c), int(s * groups + g));
                step.reads.push_back(
                    RigExecBakedOne(D::OpValue, int(layout.PlanSlot(c))));
                for (uint32_t m = s;
                     m < desc.members.size() && desc.segments[m] == s; ++m) {
                    const uint32_t n = desc.members[m];
                    OpReader reader;
                    reader.chain = c;
                    reader.segment = s;
                    reader.group = g;
                    reader.whole = whole;
                    DeclareNodeReads(*B, op, inputs, n, reader, &step.reads);
                    outputs(n, [&](uint32_t v) {
                        const uint32_t base = layout.groupBase[v];
                        step.writes.push_back(
                            whole
                                ? RigExecBakedRange(D::OpGroupValue, int(base),
                                                    int(base + groups))
                                : RigExecBakedOne(D::OpGroupValue,
                                                  int(base + g)));
                    });
                    for (const uint32_t o : layout.hostOutputsOf[n]) {
                        const eval::OpHostOutput &output =
                            layout.hostOutputs[o];
                        if (output.target !=
                            uint8_t(eval::OpOutputTarget::ChainGroup))
                            continue;
                        for (uint32_t h = whole ? 0 : g;
                             h < (whole ? groups : g + 1); ++h) {
                            const int slot = RigExecBakedGroupSlot(
                                *B, int(output.object), h);
                            if (slot >= 0)
                                step.writes.push_back(
                                    RigExecBakedOne(D::RevisionOut, slot));
                        }
                    }
                }
                std::sort(step.reads.begin(), step.reads.end(),
                          [](const RigExecBakedSlotRange &a,
                             const RigExecBakedSlotRange &b) {
                              return std::make_tuple(a.domain, a.begin, a.end) <
                                     std::make_tuple(b.domain, b.begin, b.end);
                          });
            }
        }
    }
}

/// The op nodes the pose walk numbered, in origin order: the walk
/// interleaves provider and frame-record matrices with pose commits.
bool
AddPoseWalkOps(const RigExecBakedProgramImpl &B, RigExecRigDescBuilder *builder,
               eval::OpStepLayout *layout, std::string *error)
{
    using F = RigExecBakedOpFamily;
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o) {
        const F family = B.opNodeOrigins[o].family;
        if ((family == F::ProviderMatrix || family == F::FrameMatrix ||
             family == F::ProviderRefresh) &&
            !RigExecBakedAddSpaceOp(B, builder, layout, o, error))
            return false;
        if (family == F::PoseCommit &&
            !RigExecBakedAddPoseCommitOp(B, builder, layout, o, error))
            return false;
        if (family == F::Constraint &&
            !RigExecBakedAddConstraintOp(B, builder, layout, o, error))
            return false;
        if ((family == F::SpaceCheckpoint || family == F::PoseInterpolator) &&
            !RigExecBakedAddComposeOp(B, builder, layout, o, error))
            return false;
    }
    return true;
}

} // namespace

const RigExecBakedOpOrigin *
RigExecBakedOriginOf(const RigExecBakedProgramImpl &B,
                     const RigExecBakedStep &step)
{
    if (step.kind != K::OpNode || step.object < 0 ||
        size_t(step.object) >= B.opNodeOrigins.size())
        return nullptr;
    return &B.opNodeOrigins[size_t(step.object)];
}

RigExecBakedStepIdentity
RigExecBakedIdentityOf(const RigExecBakedProgramImpl &B,
                       const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    if (origin && origin->kind != K::OpNode)
        return {origin->kind, origin->object, origin->part};
    const RigExecBakedOpProgram *op = B.opProgram.get();
    const bool chain =
        step.kind == K::OpChainHead || step.kind == K::OpChainRange;
    if (op && step.object >= 0 && (chain || step.kind == K::OpNode)) {
        const std::vector<int> &revisions =
            chain ? op->chainRevision : op->nodeRevision;
        int id = size_t(step.object) < revisions.size()
                     ? revisions[size_t(step.object)]
                     : -1;
        if (id >= 0 && step.kind == K::OpNode)
            return {K::InfluenceFold, id, -1};
        if (id >= 0 && step.kind == K::OpChainHead)
            return {K::RevisionStatic, id, -1};
        if (id >= 0) {
            // A range step is its segment's first member's revision's.
            const uint32_t groups =
                op->layout.chains[size_t(step.object)].Groups();
            const std::vector<RigExecNodeId> &members =
                op->layout.chainDescs[size_t(step.object)].members;
            const uint32_t segment = uint32_t(step.part) / groups;
            if (segment < members.size() &&
                members[segment] < op->nodeRevision.size() &&
                op->nodeRevision[members[segment]] >= 0)
                id = op->nodeRevision[members[segment]];
            return {K::RevisionChunk, id, int(uint32_t(step.part) % groups)};
        }
    }
    return {step.kind, step.object, step.part};
}

void
RigExecBakedOpStepRevisions(const RigExecBakedProgramImpl &B,
                            const RigExecBakedStep &step, std::vector<int> *ids)
{
    ids->clear();
    const RigExecBakedOpProgram *op = B.opProgram.get();
    if (!op || step.object < 0 ||
        size_t(step.object) >= op->layout.chainDescs.size() ||
        (step.kind != K::OpChainHead && step.kind != K::OpChainRange))
        return;
    const RigExecChainDesc &desc = op->layout.chainDescs[size_t(step.object)];
    uint32_t first = 0, last = uint32_t(desc.members.size());
    if (step.kind == K::OpChainRange) {
        const uint32_t groups = op->layout.chains[size_t(step.object)].Groups();
        first = groups ? uint32_t(step.part) / groups : 0;
        last = first;
        while (last < desc.members.size() && desc.segments[last] == first)
            ++last;
    }
    for (uint32_t m = first; m < last; ++m) {
        const uint32_t n = desc.members[m];
        const int id = n < op->nodeRevision.size() ? op->nodeRevision[n] : -1;
        if (id >= 0 && std::find(ids->begin(), ids->end(), id) == ids->end())
            ids->push_back(id);
    }
}

void
RigExecBakedProgramTesting::SetOpInjector(
    std::function<bool(const RigExecBakedProgramImpl &,
                       RigExecBakedOpInjection *)>
        injector)
{
    gOpInjector = std::move(injector);
}

bool
RigExecBakedBuildOpSteps(RigExecBakedProgramImpl *program, std::string *error)
{
    RigExecBakedProgramImpl &B = *program;
    B.opProgram.reset();
    B.opState.reset();
    auto op = std::make_shared<RigExecBakedOpProgram>();
    eval::OpStepLayout &layout = op->layout;
    bool matrices = false;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains)
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions)
            matrices = matrices || revision.OpNodes();
    if ((!B.opNodeOrigins.empty() || matrices) &&
        !RigExecEnsureBuiltinRegistry(error))
        return false;
    std::vector<RigExecBakedOpFeed> packets, fields;
    std::vector<std::pair<RigExecNodeId, int>> nodeRevision;
    std::vector<PointJoin> joins;
    std::vector<int> apart;
    std::unique_ptr<RigExecRigDescBuilder> builder;
    RigExecBakedOpInjection injection;
    bool injected = false;
    std::shared_ptr<const RigExecRigDesc> desc;
    std::string why;
    // A member the compiler did not put in the range chain of the node
    // feeding it reads its stored entering version instead, and the rig is
    // built again.
    for (;;) {
        builder = std::make_unique<RigExecRigDescBuilder>();
        layout = eval::OpStepLayout();
        packets.clear();
        fields.clear();
        nodeRevision.clear();
        joins.clear();
        if (!RigExecBakedAddPropertyOps(B, builder.get(), &layout, error) ||
            !RigExecBakedAddWeightFieldOps(B, builder.get(), &layout, false,
                                           &fields, error) ||
            !AddPoseWalkOps(B, builder.get(), &layout, error))
            return false;
        RigExecBakedAddVolumePlacementOps(B, builder.get(), &layout);
        if (!RigExecBakedAddWeightPacketOps(B, builder.get(), &layout, &packets,
                                            error) ||
            !RigExecBakedAddWeightFieldOps(B, builder.get(), &layout, true,
                                           &fields, error) ||
            !RigExecBakedAddProviderOps(B, builder.get(), &layout, error) ||
            !RigExecBakedAddComposeOps(B, builder.get(), &layout, error) ||
            !AddPointChainOps(B, builder.get(), &layout, packets, fields, apart,
                              &nodeRevision, &joins, error))
            return false;
        injection = RigExecBakedOpInjection();
        injection.builder = builder.get();
        injected = gOpInjector && gOpInjector(B, &injection);
        if (B.opNodeOrigins.empty() && nodeRevision.empty() && !injected)
            return true;
        desc = builder->Build(&why);
        if (!desc)
            return Fail(error, why);
        RigExecCompileOptions options;
        options.hostOrder = true;
        options.useSimd = B.useSimd;
        layout.program = RigExecCompile(*desc, options, &why);
        if (!layout.program)
            return Fail(error, why);
        std::vector<uint32_t> chainOf(RigExecProgramNodeCount(*layout.program),
                                      kRigExecInvalidIndex);
        for (size_t c = 0; c < RigExecProgramChainCount(*layout.program); ++c) {
            RigExecChainDesc chain;
            RigExecProgramChainAt(*layout.program, c, &chain);
            for (const uint32_t n : chain.members)
                chainOf[n] = uint32_t(c);
        }
        const size_t before = apart.size();
        for (const PointJoin &join : joins)
            if (chainOf[join.from] != chainOf[join.to])
                apart.push_back(join.revision);
        if (apart.size() == before)
            break;
    }
    const uint32_t lookupBase = uint32_t(layout.lookups.size());
    for (eval::OpHostInput input : injection.hostInputs) {
        if (input.source == eval::OpHostSource::Lookup)
            input.object += lookupBase;
        layout.hostInputs.push_back(input);
    }
    for (eval::OpLookup lookup : injection.lookups) {
        if (lookup.source == uint8_t(eval::OpLookupSource::List) ||
            lookup.source == uint8_t(eval::OpLookupSource::Selected))
            lookup.object += lookupBase;
        layout.lookups.push_back(lookup);
    }
    layout.hostOutputs.insert(layout.hostOutputs.end(),
                              injection.hostOutputs.begin(),
                              injection.hostOutputs.end());
    const size_t chains = RigExecProgramChainCount(*layout.program);
    layout.chains.assign(chains, eval::OpChainLayout());
    op->nodeRevision.assign(RigExecProgramNodeCount(*layout.program), -1);
    for (const auto &[node, id] : nodeRevision)
        op->nodeRevision[node] = id;
    op->chainRevision.assign(chains, -1);
    std::vector<uint32_t> chainOfNode(RigExecProgramNodeCount(*layout.program),
                                      kRigExecInvalidIndex);
    std::vector<uint32_t> memberOfNode(chainOfNode.size(),
                                       kRigExecInvalidIndex);
    std::vector<size_t> members(chains, 0);
    for (size_t c = 0; c < chains; ++c) {
        RigExecChainDesc desc;
        RigExecProgramChainAt(*layout.program, c, &desc);
        members[c] = desc.members.size();
        if (!desc.members.empty())
            op->chainRevision[c] = op->nodeRevision[desc.members[0]];
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            chainOfNode[desc.members[m]] = uint32_t(c);
            memberOfNode[desc.members[m]] = m;
        }
    }
    std::vector<char> placed(chains, 0);
    for (const eval::OpHostInput &input : layout.hostInputs) {
        const uint32_t c = input.node < chainOfNode.size()
                               ? chainOfNode[input.node]
                               : kRigExecInvalidIndex;
        uint32_t from = input.object;
        if (input.source == eval::OpHostSource::Lookup) {
            RigExecNodeDesc node;
            if (c == kRigExecInvalidIndex ||
                input.object >= layout.lookups.size() ||
                layout.lookups[input.object].source !=
                    uint8_t(eval::OpLookupSource::ChainVersion) ||
                !RigExecProgramNodeAt(*layout.program, input.node, &node) ||
                input.port >= node.values.size() ||
                !(node.op->ports[input.port].flags &
                  (kRigExecPortPointwise | kRigExecPortNeighbourhood)))
                continue;
            from = layout.lookups[input.object].object;
        } else if (input.source != eval::OpHostSource::ChainPoints) {
            continue;
        }
        if (c == kRigExecInvalidIndex)
            return Fail(error, "node " + std::to_string(input.node) +
                                   " reads chain points outside a range chain");
        if (from >= B.chains.size() || B.chains[from].groupBounds.size() < 2)
            return Fail(error, "point chain " + std::to_string(from) +
                                   " is not cut into vertex groups");
        if (placed[c] && layout.chains[c].pointChain != from)
            return Fail(error, "range chain " + std::to_string(c) +
                                   " reads two point chains");
        layout.chains[c].pointChain = from;
        layout.chains[c].bounds = B.chains[from].groupBounds;
        placed[c] = 1;
    }
    for (size_t c = 0; c < chains; ++c)
        if (!placed[c])
            return Fail(error, "range chain " + std::to_string(c) +
                                   " reads no point chain");
    std::vector<int> stored;
    for (const eval::OpHostOutput &output : layout.hostOutputs) {
        if (output.target != uint8_t(eval::OpOutputTarget::ChainGroup) ||
            output.node >= chainOfNode.size() ||
            chainOfNode[output.node] == kRigExecInvalidIndex)
            continue;
        eval::OpChainLayout &chain = layout.chains[chainOfNode[output.node]];
        const size_t groups = chain.Groups();
        if (output.object >= B.revisionIndex.size() ||
            uint32_t(B.revisionIndex[output.object].first) != chain.pointChain)
            return Fail(error, "node " + std::to_string(output.node) +
                                   " stores into revision " +
                                   std::to_string(output.object) +
                                   ", which is not of its point chain");
        const auto &[pointChain, r] = B.revisionIndex[output.object];
        const std::vector<char> &written =
            B.chains[size_t(pointChain)].revisions[size_t(r)].groupWritten;
        if (written.size() != groups)
            return Fail(error, "revision " + std::to_string(output.object) +
                                   " is cut into " +
                                   std::to_string(written.size()) +
                                   " groups, its range chain into " +
                                   std::to_string(groups));
        if (chain.writes.empty())
            chain.writes.assign(members[chainOfNode[output.node]] * groups, 1);
        const size_t row = size_t(memberOfNode[output.node]) * groups;
        for (size_t g = 0; g < groups; ++g)
            chain.writes[row + g] = chain.writes[row + g] && written[g];
        stored.push_back(int(output.object));
    }
    for (eval::OpChainLayout &chain : layout.chains)
        if (std::find(chain.writes.begin(), chain.writes.end(), 0) ==
            chain.writes.end())
            chain.writes.clear();
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const RigExecBakedOpInjection::Constant &constant :
         injection.constants) {
        RigExecNodeDesc node;
        RigExecProgramNodeAt(*layout.program, constant.node, &node);
        eval::OpConstant bytes;
        bytes.node = constant.node;
        bytes.port = constant.port;
        if (!node.op || constant.port >= node.values.size() ||
            !ConstantBytes(
                constant.value,
                *registry.TypeAt(node.op->ports[constant.port].type->Index())
                     .desc,
                &bytes.bytes))
            return Fail(error, "the value fixed for input " +
                                   std::to_string(constant.port) + " of node " +
                                   std::to_string(constant.node) +
                                   " is not of its type");
        layout.constants.push_back(std::move(bytes));
    }
    if (!eval::LayOutOpSteps(&layout, error))
        return false;
    B.propertyKeyHold.assign(layout.lookups.size(), VtArray<GfVec2f>());
    B.propertyPointHold.assign(layout.lookups.size(), VtVec3fArray());
    B.revisionLeafHold.assign(layout.lookups.size(), VtValue());
    B.chainVersionGather.assign(layout.lookups.size(), {});
    op->outputs = injection.outputs;
    for (const RigExecDescNode &node : desc->Data().nodes) {
        const RigExecOpDesc &desc = *registry.OpAt(node.op).desc;
        std::vector<uint8_t> bytes;
        bool savable = true;
        if (desc.payloadFieldCount)
            RigExecEncodePayloadFields(desc, node.fields, bytes);
        else if (node.hasPayload && desc.payload.encode)
            desc.payload.encode(node.payload.Data(), bytes);
        else
            savable = !node.hasPayload && !desc.payload.size;
        op->payloads.push_back(std::move(bytes));
        op->savable.push_back(savable ? 1 : 0);
    }
    for (const auto &[name, output] : op->outputs) {
        RigExecValueRef ref;
        const RigExecTypeDesc *type =
            RigExecResolveOutput(*layout.program, output, &ref) && ref.type
                ? registry.TypeAt(ref.type->Index()).desc
                : nullptr;
        if (!type || std::string(type->nameSpace) != "std" ||
            std::string(type->name) != "Vec3fArray")
            return Fail(error, "output " + name + " is not a point array");
    }
    auto state = std::make_shared<eval::OpStepState>();
    state->run = RigExecRunState::Create(layout.program, {}, &why);
    if (!state->run)
        return Fail(error, why);
    const BakedOpHost host{B};
    if (!eval::BindOpStepState(layout, host, state.get(), error))
        return false;
    FindHaloGroups(op.get(), *state->run);
    if (!stored.empty())
        DropRevisionChunks(&B, stored);
    EmitOpSteps(&B, *op);
    state->run->ReleaseOwner();
    B.opProgram = std::move(op);
    B.opState = std::move(state);
    return true;
}

void
RigExecBakedRunOpStep(RigExecBakedProgramImpl *program, RigExecBakedStep *step)
{
    RigExecBakedProgramImpl &B = *program;
    step->failed = false;
    step->diagnostics.clear();
    if (!B.opProgram || !B.opState) {
        step->failed = true;
        step->diagnostics.push_back("op nodes: the program holds none");
        return;
    }
    const BakedOpHost host{B, step};
    eval::OpStepState &state = *B.opState;
    std::string error;
    size_t chain = 0, part = 0;
    const bool property = RigExecBakedPropertyPartOf(B, *step, &chain, &part);
    if (property) {
        step->lines.clear();
        RigExecBakedPropertyInputLines(B, chain, part, &step->lines);
    }
    const eval::OpStepKind kind = eval::OpStepKind(uint8_t(step->kind));
    RigExecNodeId failed = kRigExecInvalidIndex;
    if (eval::RunOpStep(B.opProgram->layout, host, &state, kind, step->object,
                        step->part, &error, &failed))
        return;
    step->failed = true;
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, *step);
    const std::vector<int> &nodeRevision = B.opProgram->nodeRevision;
    const int failedRevision =
        failed < nodeRevision.size() ? nodeRevision[failed] : -1;
    if (property)
        step->diagnostics.push_back(
            B.propertyChains[chain].revisions[part - 1].linePrefix + ": " +
            error);
    else if (origin && origin->family == RigExecBakedOpFamily::WeightPacket)
        step->diagnostics.push_back(
            B.weightProgram[size_t(origin->object)].name + ": " + error);
    else if (origin && origin->family == RigExecBakedOpFamily::PoseCommit)
        step->diagnostics.push_back(
            RigExecBakedCommitFailure(B, origin->object, error));
    else if (origin && origin->family == RigExecBakedOpFamily::Constraint)
        step->diagnostics.push_back(
            RigExecBakedConstraintFailure(B, origin->object, error));
    else if (!origin && failedRevision >= 0 &&
             size_t(failedRevision) < B.revisionIndex.size())
        step->diagnostics.push_back(
            B.chains[size_t(B.revisionIndex[size_t(failedRevision)].first)]
                .revisions[size_t(
                    B.revisionIndex[size_t(failedRevision)].second)]
                .moverPathText +
            ": " + error);
    else if (const RigExecBakedStepIdentity id =
                 RigExecBakedIdentityOf(B, *step);
             !origin && id.kind != step->kind &&
             size_t(id.object) < B.revisionIndex.size())
        step->diagnostics.push_back(
            B.chains[size_t(B.revisionIndex[size_t(id.object)].first)]
                .revisions[size_t(B.revisionIndex[size_t(id.object)].second)]
                .moverPathText +
            ": " + error);
    else if (origin && origin->family == RigExecBakedOpFamily::WeightField)
        step->diagnostics.push_back(
            B.weightProgram[size_t(B.weightFields[size_t(origin->object)]
                                       .nodeObjects[size_t(origin->part)])]
                .name +
            ": " + error);
    else
        step->diagnostics.push_back(error);
}

bool
RigExecBakedCloneOpState(const RigExecBakedProgramImpl &source,
                         RigExecBakedProgramImpl *clone, std::string *error)
{
    clone->opProgram = source.opProgram;
    clone->opState.reset();
    if (!source.opProgram || !source.opState)
        return true;
    auto state = std::make_shared<eval::OpStepState>();
    state->run = source.opState->run->Clone();
    state->versions = source.opState->versions;
    state->groupVersions = source.opState->groupVersions;
    state->haloReads = source.opState->haloReads;
    const RigExecBakedOpProgram &op = *source.opProgram;
    const BakedOpHost host{*clone};
    if (!eval::BindOpStepState(op.layout, host, state.get(), error))
        return false;
    eval::CopyOpLookups(*source.opState, state.get());
    state->run->ReleaseOwner();
    clone->opState = std::move(state);
    return true;
}

void
RigExecBakedPublishOpOutputs(const RigExecBakedProgramImpl &B,
                             RigExecRigPose *pose)
{
    if (!B.opProgram || !B.opState)
        return;
    for (const auto &[name, output] : B.opProgram->outputs) {
        RigExecValueRef ref;
        if (!RigExecResolveOutput(*B.opProgram->layout.program, output, &ref))
            continue;
        const auto *points =
            static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                B.opState->run->GetValue(ref));
        if (!points)
            continue;
        VtVec3fArray value(points->Size());
        if (points->Size())
            std::memcpy(value.data(), points->Data(),
                        points->Size() * sizeof(GfVec3f));
        pose->opOutputs[name] = VtValue(value);
    }
}

bool
RigExecBakedOpValueVersion(const RigExecBakedProgramImpl &B,
                           RigExecBakedSlotDomain domain, uint32_t slot,
                           uint64_t *version)
{
    if (!B.opState)
        return false;
    const std::vector<uint64_t> &versions =
        domain == D::OpValue ? B.opState->versions : B.opState->groupVersions;
    if (slot >= versions.size())
        return false;
    *version = versions[slot];
    return true;
}

} // namespace rigExec
