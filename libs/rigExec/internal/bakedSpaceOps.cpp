#include "rigExec/internal/bakedProgramImpl.h"

#include "rigExec/api/rigDesc.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/affineMath.h"
#include "rigExecOps/space/providerOps.h"
#include "rigExecOps/space/spaceOps.h"
#include "rigExecStdTypes/stdTypes.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/array.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <string>
#include <vector>

namespace rigExec {

namespace {

bool
SpaceFail(std::string *error, std::string why)
{
    if (error)
        *error = std::move(why);
    return false;
}

void
PutFrame(const RigExecPointFrame &frame, rigExecRig::PointFrame *out)
{
    for (size_t i = 0; i < 4; ++i)
        out->points[i] = rigExecStd::Vec3d(
            frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    out->flags = frame.flags;
}

RigExecPointFrame
TakeFrame(const rigExecRig::PointFrame &frame)
{
    RigExecPointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] =
            GfVec3d(frame.points[i][0], frame.points[i][1], frame.points[i][2]);
    out.flags = frame.flags;
    return out;
}

using ProviderRead = eval::spaces::ProviderRead;
using ProviderClass = eval::spaces::ProviderInputClass;
using AffineClass = eval::spaces::AffineInputClass;

ProviderClass
ArithmeticClassOf(const RigExecProviderOp &op, size_t p)
{
    return op.inputs[p] == RigExecNoProviderValue
               ? ProviderClass::None
               : eval::spaces::ProviderArithmeticInputClass(op.kind, p);
}

AffineClass
AffineClassOf(const RigExecProviderOp &op, size_t p)
{
    return eval::spaces::AffineFrameInputClass<RrAffineMath>(op.affineKind, p);
}

bool
ItemsClass(AffineClass c)
{
    return c == AffineClass::IntItems || c == AffineClass::DoubleItems ||
           c == AffineClass::MatrixItems;
}

template <class T>
rigExecStd::RigExecArray<T> &
ListOf(void *out)
{
    return *static_cast<rigExecStd::RigExecArray<T> *>(out);
}

const VtValue *
Boxed(const RigExecTypedValueStore &store, RigExecValueId id)
{
    return store.Read<VtValue>(id);
}

template <class T>
const T *
BoxedAs(const RigExecTypedValueStore &store, RigExecValueId id)
{
    const VtValue *value = Boxed(store, id);
    return value && value->IsHolding<T>() ? &value->UncheckedGet<T>() : nullptr;
}

bool
ReadScalar(const RigExecTypedValueStore &store, RigExecValueId id, double *out)
{
    if (const double *value = store.Read<double>(id)) {
        *out = *value;
        return true;
    }
    if (const float *narrow = store.Read<float>(id)) {
        *out = double(*narrow);
        return true;
    }
    return false;
}

void
PutMatrix(const GfMatrix4d *matrix, rigExecStd::Mat4d *out)
{
    static_assert(sizeof(rigExecStd::Mat4d) == sizeof(GfMatrix4d),
                  "a std matrix is sixteen doubles");
    const GfMatrix4d identity(1.0);
    std::memcpy(static_cast<void *>(out), (matrix ? matrix : &identity)->data(),
                sizeof(GfMatrix4d));
}

/// Whether \p id's value reads as arithmetic input class \p c this frame,
/// as GfProviderStore reads it.
bool
ArithmeticReadable(const RigExecTypedValueStore &store, RigExecValueId id,
                   ProviderClass c)
{
    double scalar = 0.0;
    switch (c) {
    case ProviderClass::Matrix:
        return store.Read<GfMatrix4d>(id) != nullptr;
    case ProviderClass::Frame:
        return store.Read<RigExecPointFrame>(id) != nullptr;
    case ProviderClass::Scalar:
        return ReadScalar(store, id, &scalar);
    case ProviderClass::Vector:
        return store.Read<GfVec3d>(id) != nullptr;
    case ProviderClass::Order:
    case ProviderClass::Reset:
    case ProviderClass::None:
        break;
    }
    return true;
}

/// Whether \p id's value reads as affine input class \p c this frame, as
/// the affine sequencing reads it; texts and arrays always do.
bool
AffineReadable(const RigExecTypedValueStore &store, RigExecValueId id,
               AffineClass c)
{
    double scalar = 0.0;
    switch (c) {
    case AffineClass::Matrix:
        return store.Read<GfMatrix4d>(id) != nullptr;
    case AffineClass::Vector:
        return store.Read<GfVec3d>(id) != nullptr;
    case AffineClass::IntVector:
        return BoxedAs<GfVec3i>(store, id) != nullptr;
    case AffineClass::Double:
        return ReadScalar(store, id, &scalar);
    case AffineClass::Bool:
        return BoxedAs<bool>(store, id) != nullptr;
    case AffineClass::Int:
        return BoxedAs<int>(store, id) != nullptr;
    case AffineClass::Frame:
        return store.Read<RigExecPointFrame>(id) != nullptr;
    case AffineClass::Text:
    case AffineClass::IntItems:
    case AffineClass::DoubleItems:
    case AffineClass::MatrixItems:
        break;
    }
    return true;
}

bool
ResetsStack(const RigExecTypedValueStore &store, RigExecValueId id)
{
    if (id >= store.values.size() || !store.values[size_t(id)].initialized ||
        store.values[size_t(id)].blocked)
        return false;
    const VtValue &raw = store.values[size_t(id)].raw;
    if (!raw.IsHolding<VtTokenArray>())
        return false;
    for (const TfToken &token : raw.UncheckedGet<VtTokenArray>())
        if (token == "!resetXformStack!")
            return true;
    return false;
}

/// Fills arithmetic list \p read of \p op, one element per input of its
/// class in input order (ProviderArithmeticPayload's slots).
bool
FillArithmetic(const RigExecTypedValueStore &store, const RigExecProviderOp &op,
               ProviderRead read, void *out, std::string *error)
{
    const size_t n = op.inputs.size();
    const auto count = [&](ProviderClass c) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            k += ArithmeticClassOf(op, p) == c ? 1 : 0;
        return k;
    };
    const auto each = [&](ProviderClass c, auto &&put) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            if (ArithmeticClassOf(op, p) == c)
                put(k++, op.inputs[p]);
    };
    switch (read) {
    case ProviderRead::Present: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(n);
        for (size_t p = 0; p < n; ++p)
            list[p] = ArithmeticReadable(store, op.inputs[p],
                                         ArithmeticClassOf(op, p))
                          ? 1
                          : 0;
        return true;
    }
    case ProviderRead::Matrices: {
        auto &list = ListOf<rigExecStd::Mat4d>(out);
        list.Assign(count(ProviderClass::Matrix));
        each(ProviderClass::Matrix, [&](size_t k, RigExecValueId id) {
            PutMatrix(store.Read<GfMatrix4d>(id), &list[k]);
        });
        return true;
    }
    case ProviderRead::Authoritative: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(count(ProviderClass::Matrix));
        each(ProviderClass::Matrix, [&](size_t k, RigExecValueId id) {
            list[k] = id < store.values.size() &&
                              store.values[size_t(id)].authoritative
                          ? 1
                          : 0;
        });
        return true;
    }
    case ProviderRead::Frames: {
        auto &list = ListOf<rigExecRig::PointFrame>(out);
        list.Assign(count(ProviderClass::Frame));
        each(ProviderClass::Frame, [&](size_t k, RigExecValueId id) {
            if (const auto *frame = store.Read<RigExecPointFrame>(id))
                PutFrame(*frame, &list[k]);
        });
        return true;
    }
    case ProviderRead::Scalars: {
        auto &list = ListOf<double>(out);
        list.Assign(count(ProviderClass::Scalar));
        each(ProviderClass::Scalar, [&](size_t k, RigExecValueId id) {
            ReadScalar(store, id, &list[k]);
        });
        return true;
    }
    case ProviderRead::Vectors: {
        auto &list = ListOf<rigExecStd::Vec3d>(out);
        list.Assign(count(ProviderClass::Vector));
        each(ProviderClass::Vector, [&](size_t k, RigExecValueId id) {
            if (const auto *v = store.Read<GfVec3d>(id))
                list[k] = rigExecStd::Vec3d((*v)[0], (*v)[1], (*v)[2]);
        });
        return true;
    }
    case ProviderRead::Orders: {
        auto &list = ListOf<int32_t>(out);
        list.Assign(count(ProviderClass::Order));
        each(ProviderClass::Order, [&](size_t k, RigExecValueId id) {
            if (const auto *token = store.Read<TfToken>(id))
                list[k] = eval::spaces::ProviderPackOrder(token->GetString());
        });
        return true;
    }
    case ProviderRead::Resets: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(count(ProviderClass::Reset));
        each(ProviderClass::Reset, [&](size_t k, RigExecValueId id) {
            list[k] = ResetsStack(store, id) ? 1 : 0;
        });
        return true;
    }
    default:
        break;
    }
    return SpaceFail(error, "provider arithmetic has no input list " +
                                std::to_string(int(read)));
}

/// Fills affine list \p read of \p op, one element per input of its class
/// in input order (AffineFramePayload's slots); the three item lists share
/// one span per array input.
bool
FillAffine(const RigExecTypedValueStore &store, const RigExecProviderOp &op,
           ProviderRead read, void *out, std::string *error)
{
    const size_t n = op.inputs.size();
    const auto listed = [&](size_t p, AffineClass c) {
        return op.inputs[p] != RigExecNoProviderValue &&
               AffineClassOf(op, p) == c;
    };
    const auto fill = [&](AffineClass c, auto &list, auto &&put) {
        size_t k = 0;
        for (size_t p = 0; p < n; ++p)
            k += listed(p, c) ? 1 : 0;
        list.Assign(k);
        k = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                put(k++, op.inputs[p]);
    };
    const auto items = [&](AffineClass c, auto &list, const auto *sample) {
        using Array = std::decay_t<decltype(*sample)>;
        size_t total = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                if (const auto *array = BoxedAs<Array>(store, op.inputs[p]))
                    total += array->size();
        list.Assign(total);
        size_t at = 0;
        for (size_t p = 0; p < n; ++p)
            if (listed(p, c))
                if (const auto *array = BoxedAs<Array>(store, op.inputs[p]))
                    for (const auto &item : *array)
                        std::memcpy(static_cast<void *>(&list[at++]), &item,
                                    sizeof(item));
    };
    switch (read) {
    case ProviderRead::AffinePresent: {
        auto &list = ListOf<uint8_t>(out);
        list.Assign(n);
        for (size_t p = 0; p < n; ++p)
            list[p] = op.inputs[p] == RigExecNoProviderValue ||
                              AffineReadable(store, op.inputs[p],
                                             AffineClassOf(op, p))
                          ? 1
                          : 0;
        return true;
    }
    case ProviderRead::AffineMatrices:
        fill(AffineClass::Matrix, ListOf<rigExecStd::Mat4d>(out),
             [&](size_t k, RigExecValueId id) {
                 PutMatrix(store.Read<GfMatrix4d>(id),
                           &ListOf<rigExecStd::Mat4d>(out)[k]);
             });
        return true;
    case ProviderRead::AffineDoubles:
        fill(AffineClass::Double, ListOf<double>(out),
             [&](size_t k, RigExecValueId id) {
                 ReadScalar(store, id, &ListOf<double>(out)[k]);
             });
        return true;
    case ProviderRead::AffineBools:
        fill(AffineClass::Bool, ListOf<uint8_t>(out),
             [&](size_t k, RigExecValueId id) {
                 const bool *value = BoxedAs<bool>(store, id);
                 ListOf<uint8_t>(out)[k] = value && *value ? 1 : 0;
             });
        return true;
    case ProviderRead::AffineInts:
        fill(AffineClass::Int, ListOf<int32_t>(out),
             [&](size_t k, RigExecValueId id) {
                 const int *value = BoxedAs<int>(store, id);
                 ListOf<int32_t>(out)[k] = value ? int32_t(*value) : 0;
             });
        return true;
    case ProviderRead::AffineVectors:
        fill(AffineClass::Vector, ListOf<rigExecStd::Vec3d>(out),
             [&](size_t k, RigExecValueId id) {
                 if (const auto *v = store.Read<GfVec3d>(id))
                     ListOf<rigExecStd::Vec3d>(out)[k] =
                         rigExecStd::Vec3d((*v)[0], (*v)[1], (*v)[2]);
             });
        return true;
    case ProviderRead::AffineIntVectors:
        fill(AffineClass::IntVector, ListOf<rigExecStd::Vec3i>(out),
             [&](size_t k, RigExecValueId id) {
                 static_assert(sizeof(rigExecStd::Vec3i) == sizeof(GfVec3i),
                               "a std int vector is three ints");
                 if (const auto *v = BoxedAs<GfVec3i>(store, id))
                     std::memcpy(static_cast<void *>(
                                     &ListOf<rigExecStd::Vec3i>(out)[k]),
                                 v->data(), sizeof(GfVec3i));
             });
        return true;
    case ProviderRead::AffineFrames:
        fill(AffineClass::Frame, ListOf<rigExecRig::PointFrame>(out),
             [&](size_t k, RigExecValueId id) {
                 if (const auto *frame = store.Read<RigExecPointFrame>(id))
                     PutFrame(*frame, &ListOf<rigExecRig::PointFrame>(out)[k]);
             });
        return true;
    case ProviderRead::AffineIntItems:
        items(AffineClass::IntItems, ListOf<int32_t>(out),
              static_cast<const VtIntArray *>(nullptr));
        return true;
    case ProviderRead::AffineDoubleItems:
        items(AffineClass::DoubleItems, ListOf<double>(out),
              static_cast<const VtDoubleArray *>(nullptr));
        return true;
    case ProviderRead::AffineMatrixItems:
        items(AffineClass::MatrixItems, ListOf<rigExecStd::Mat4d>(out),
              static_cast<const VtMatrix4dArray *>(nullptr));
        return true;
    case ProviderRead::AffineItemSpans: {
        size_t spans = 0;
        for (size_t p = 0; p < n; ++p)
            spans += op.inputs[p] != RigExecNoProviderValue &&
                             ItemsClass(AffineClassOf(op, p))
                         ? 1
                         : 0;
        auto &list = ListOf<int32_t>(out);
        list.Assign(2 * spans);
        size_t k = 0, ints = 0, doubles = 0, matrices = 0;
        for (size_t p = 0; p < n; ++p) {
            const AffineClass c = AffineClassOf(op, p);
            if (op.inputs[p] == RigExecNoProviderValue || !ItemsClass(c))
                continue;
            const RigExecValueId id = op.inputs[p];
            size_t size = 0, *offset = &ints;
            if (c == AffineClass::IntItems) {
                const auto *array = BoxedAs<VtIntArray>(store, id);
                size = array ? array->size() : 0;
            } else if (c == AffineClass::DoubleItems) {
                const auto *array = BoxedAs<VtDoubleArray>(store, id);
                size = array ? array->size() : 0;
                offset = &doubles;
            } else {
                const auto *array = BoxedAs<VtMatrix4dArray>(store, id);
                size = array ? array->size() : 0;
                offset = &matrices;
            }
            list[2 * k] = int32_t(*offset);
            list[2 * k + 1] = int32_t(size);
            *offset += size;
            ++k;
        }
        return true;
    }
    default:
        break;
    }
    return SpaceFail(error, "an affine frame has no input list " +
                                std::to_string(int(read)));
}

} // namespace

bool
RigExecBakedAddSpaceOp(const RigExecBakedProgramImpl &B,
                       RigExecRigDescBuilder *builder,
                       eval::OpStepLayout *layout, size_t o, std::string *error)
{
    using namespace rigExecOps;
    using S = eval::OpLookupSource;
    using T = eval::OpOutputTarget;
    const auto lookUp = [&](RigExecNodeId node, const RigExecPortHandle &port,
                            S source, uint32_t object, int32_t part,
                            uint8_t read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(source), object, part, read});
    };
    const auto store = [&](RigExecNodeId node, const RigExecPortHandle &port,
                           T target, uint32_t object) {
        builder->BindOutput(node, port);
        layout->hostOutputs.push_back(
            {node, port.Index(), uint8_t(target), object});
    };
    const auto lookUpList = [&](RigExecNodeId node,
                                const RigExecPortHandle &port,
                                const std::vector<eval::OpLookup> &elements) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(S::List),
                                   uint32_t(layout->lookups.size()) + 1,
                                   int32_t(elements.size()), 0});
        layout->lookups.insert(layout->lookups.end(), elements.begin(),
                               elements.end());
    };
    const auto numbered = [&](RigExecNodeId node, size_t origin) {
        return node == origin ||
               SpaceFail(error, "op node " + std::to_string(origin) +
                                    " was numbered " + std::to_string(node));
    };
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    if (origin.family == RigExecBakedOpFamily::ProviderMatrix) {
        const uint32_t slot = uint32_t(origin.object);
        ProviderMatrixPayload payload;
        payload.posed = origin.part == 1;
        payload.joint = std::find(B.jointSlots.begin(), B.jointSlots.end(),
                                  int(slot)) != B.jointSlots.end();
        const RigExecNodeId node = builder->AddOp(ProviderMatrix(), &payload);
        if (!numbered(node, o))
            return false;
        lookUp(node, ProviderMatrixRest(), S::RestFrame, slot, -1, 0);
        if (payload.posed)
            lookUp(node, ProviderMatrixFinal(), S::PoseVersion,
                   uint32_t(B.finLast[slot]), 0, 0);
        lookUp(node, ProviderMatrixBase(), S::PoseVersion,
               uint32_t(B.baseLast[slot]), 1, 0);
        store(node, ProviderMatrixOut(),
              payload.posed ? T::ProviderFinalMatrix : T::ProviderBaseMatrix,
              slot);
    } else if (origin.family == RigExecBakedOpFamily::FrameMatrix) {
        const RigExecBakedFrameRecord &record =
            B.frameRecords[size_t(origin.object)];
        if (record.commit < 0 || size_t(record.commit) >= B.commits.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(origin.object) +
                                        " names no commit");
        if (record.slot < 0 || size_t(record.slot) >= B.restFrames.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(origin.object) +
                                        " names no slot");
        FrameMatrixPayload payload;
        payload.solverOutput = B.commits[size_t(record.commit)].solverOutput;
        payload.position = record.position;
        payload.target = record.target;
        const RigExecNodeId node = builder->AddOp(FrameMatrix(), &payload);
        if (!numbered(node, o))
            return false;
        const uint32_t commit = uint32_t(record.commit);
        lookUp(node, FrameMatrixRest(), S::RestFrame, uint32_t(record.slot), -1,
               0);
        lookUp(node, FrameMatrixFrame(), S::PoseVersion, record.version, 0, 0);
        if (payload.solverOutput) {
            lookUp(node, FrameMatrixPresent(), S::CommitRecordGate, commit, -1,
                   0);
        } else {
            lookUp(node, FrameMatrixRecordAfter(), S::CommitRecordGate, commit,
                   -1, 1);
            lookUp(node, FrameMatrixRecordEveryTarget(), S::CommitRecordGate,
                   commit, -1, 2);
        }
        store(node, FrameMatrixOut(), T::FrameMatrix, uint32_t(origin.object));
        store(node, FrameMatrixValid(), T::FrameMatrixValid,
              uint32_t(origin.object));
    } else if (origin.family == RigExecBakedOpFamily::ProviderRefresh) {
        const uint32_t index = uint32_t(origin.object);
        const auto &refresh = B.providerRefreshes[index];
        ProviderRefreshPayload payload;
        std::vector<eval::OpLookup> carryBase, carryFinal, parents;
        for (const auto &carry : refresh.carries) {
            payload.blockingCounts.push_back(
                uint32_t(carry.blockingSlots.size()));
            carryBase.push_back(
                {uint8_t(S::PoseVersion), carry.baseRead, 1, 0});
            carryFinal.push_back(
                {uint8_t(S::PoseVersion), carry.finRead, 0, 0});
            for (const int slot : carry.blockingSlots)
                parents.push_back(
                    {uint8_t(S::ProviderParentRaw), uint32_t(slot), -1, 0});
        }
        const RigExecNodeId node = builder->AddOp(ProviderRefresh(), &payload);
        if (!numbered(node, o))
            return false;
        lookUp(node, ProviderRefreshBase(), S::PoseVersion, refresh.baseRead, 1,
               0);
        lookUp(node, ProviderRefreshFinal(), S::PoseVersion, refresh.finRead, 0,
               0);
        lookUp(node, ProviderRefreshRaw(), S::ProviderFrame, index, -1, 0);
        lookUp(node, ProviderRefreshCurrent(), S::ProviderFrame, index, -1, 1);
        if (!refresh.priorConstraints.empty()) {
            std::vector<eval::OpLookup> committed;
            for (const auto &[commit, pos] : refresh.priorConstraints)
                committed.push_back(
                    {uint8_t(S::CommitRecordGate), commit, int32_t(pos), 3});
            lookUpList(node, ProviderRefreshConstrained(), committed);
        }
        lookUpList(node, ProviderRefreshCarryBase(), carryBase);
        lookUpList(node, ProviderRefreshCarryFinal(), carryFinal);
        if (!parents.empty())
            lookUpList(node, ProviderRefreshBlockingParents(), parents);
        store(node, ProviderRefreshBaseOut(), T::RefreshBase, index);
        store(node, ProviderRefreshFinalOut(), T::RefreshFinal, index);
        store(node, ProviderRefreshCarryBaseOut(), T::RefreshCarryBase, index);
        store(node, ProviderRefreshCarryFinalOut(), T::RefreshCarryFinal,
              index);
    }
    return true;
}

int
RigExecBakedProviderRefreshOf(const RigExecBakedProgramImpl &B,
                              const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::ProviderRefresh
               ? int(origin->object)
               : -1;
}

bool
RigExecBakedAddProviderOps(const RigExecBakedProgramImpl &B,
                           RigExecRigDescBuilder *builder,
                           eval::OpStepLayout *layout, std::string *error)
{
    using namespace rigExecOps;
    using S = eval::OpLookupSource;
    using T = eval::OpOutputTarget;
    const RigExecProviderProgram &program = B.GetProviderProgram();
    std::vector<int> producer(program.valueKeys.size(), -1),
        sampled(program.valueKeys.size(), -1);
    for (size_t k = 0; k < program.ops.size(); ++k)
        if (program.ops[k].output < producer.size())
            producer[size_t(program.ops[k].output)] = int(k);
    for (size_t k = 0; k < program.sampled.size(); ++k)
        if (program.sampled[k].value < sampled.size())
            sampled[size_t(program.sampled[k].value)] = int(k);
    const auto lookUp = [&](RigExecNodeId node, const RigExecPortHandle &port,
                            uint32_t op, ProviderRead read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(S::ProviderInputs), op, -1, uint8_t(read)});
    };
    const auto store = [&](RigExecNodeId node, const RigExecPortHandle &port,
                           T target, uint32_t op) {
        builder->BindOutput(node, port);
        layout->hostOutputs.push_back(
            {node, port.Index(), uint8_t(target), op});
    };
    // A text input is the uniform token its attribute authors; 0 where it
    // authors none, -1 where it cannot be known at Build.
    const auto text = [&](RigExecValueId id, const std::string &key,
                          std::string *out) -> int {
        const auto refuse = [&](std::string why) {
            SpaceFail(error, std::move(why));
            return -1;
        };
        const int attribute = id < producer.size() ? producer[size_t(id)] : -1;
        const RigExecProviderOp *op =
            attribute >= 0 ? &program.ops[size_t(attribute)] : nullptr;
        if (!op || op->kind != RigExecProviderOpKind::Attribute ||
            op->inputs.empty())
            return refuse("affine text input " + key +
                          " is not an authored attribute");
        if (op->inputs.size() > 1 && op->inputs[1] != RigExecNoProviderValue)
            return refuse("affine text input " + key + " is connected");
        const RigExecValueId raw = op->inputs[0];
        const int leaf = raw < sampled.size() ? sampled[size_t(raw)] : -1;
        if (leaf < 0 || size_t(leaf) >= B.providerLeaves.values.size())
            return refuse("affine text input " + key +
                          " has no sampled attribute");
        if (size_t(leaf) < B.providerLeaves.varying.size() &&
            B.providerLeaves.varying[size_t(leaf)])
            return refuse("affine text input " + key + " varies over time");
        const VtValue &value = B.providerLeaves.values[size_t(leaf)];
        const bool blocked = size_t(leaf) < B.providerLeafBlocked.size() &&
                             B.providerLeafBlocked[size_t(leaf)];
        if (blocked || !value.IsHolding<TfToken>())
            return 0;
        *out = value.UncheckedGet<TfToken>().GetString();
        return 1;
    };
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o) {
        const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
        if (origin.family != RigExecBakedOpFamily::ProviderOp)
            continue;
        const uint32_t index = uint32_t(origin.object);
        const RigExecProviderOp &op = program.ops[index];
        const size_t n = op.inputs.size();
        if (op.kind == RigExecProviderOpKind::AffineFrame) {
            AffineFramePayload payload;
            payload.kind = op.affineKind;
            payload.targets = op.affineTargets;
            std::vector<int32_t> counts(size_t(AffineClass::Frame) + 1, 0);
            int32_t spans = 0;
            for (size_t p = 0; p < n; ++p) {
                const AffineClass c = AffineClassOf(op, p);
                int32_t slot = -1;
                if (op.inputs[p] == RigExecNoProviderValue) {
                } else if (c == AffineClass::Text) {
                    std::string value;
                    const int found =
                        text(op.inputs[p],
                             program.valueKeys[size_t(op.inputs[p])], &value);
                    if (found < 0)
                        return false;
                    if (found) {
                        slot = int32_t(payload.texts.size());
                        payload.texts.push_back(std::move(value));
                    }
                } else if (ItemsClass(c)) {
                    slot = spans++;
                    ++counts[size_t(c)];
                } else {
                    slot = counts[size_t(c)]++;
                }
                payload.slots.push_back(slot);
            }
            const RigExecNodeId node = builder->AddOp(AffineFrame(), &payload);
            if (node != o)
                return SpaceFail(error, "op node " + std::to_string(o) +
                                            " was numbered " +
                                            std::to_string(node));
            const std::pair<AffineClass, ProviderRead> lists[] = {
                {AffineClass::Matrix, ProviderRead::AffineMatrices},
                {AffineClass::Double, ProviderRead::AffineDoubles},
                {AffineClass::Bool, ProviderRead::AffineBools},
                {AffineClass::Int, ProviderRead::AffineInts},
                {AffineClass::Vector, ProviderRead::AffineVectors},
                {AffineClass::IntVector, ProviderRead::AffineIntVectors},
                {AffineClass::IntItems, ProviderRead::AffineIntItems},
                {AffineClass::DoubleItems, ProviderRead::AffineDoubleItems},
                {AffineClass::MatrixItems, ProviderRead::AffineMatrixItems},
                {AffineClass::Frame, ProviderRead::AffineFrames},
            };
            const RigExecPortHandle *ports[] = {
                &AffineFrameMatrices(),    &AffineFrameDoubles(),
                &AffineFrameBools(),       &AffineFrameInts(),
                &AffineFrameVectors(),     &AffineFrameIntVectors(),
                &AffineFrameIntItems(),    &AffineFrameDoubleItems(),
                &AffineFrameMatrixItems(), &AffineFrameFrames(),
            };
            for (size_t l = 0; l < std::size(lists); ++l)
                if (counts[size_t(lists[l].first)])
                    lookUp(node, *ports[l], index, lists[l].second);
            if (spans)
                lookUp(node, AffineFrameItemSpans(), index,
                       ProviderRead::AffineItemSpans);
            lookUp(node, AffineFramePresent(), index,
                   ProviderRead::AffinePresent);
            store(node, AffineFrameOut(), T::ProviderValueMatrix, index);
            continue;
        }
        ProviderArithmeticPayload payload;
        payload.kind = op.kind;
        payload.scaleAvars = op.scaleAvars;
        std::vector<int32_t> counts(size_t(ProviderClass::Reset) + 1, 0);
        for (size_t p = 0; p < n; ++p) {
            const ProviderClass c = ArithmeticClassOf(op, p);
            payload.slots.push_back(
                c == ProviderClass::None ? -1 : counts[size_t(c)]++);
        }
        const RigExecNodeId node =
            builder->AddOp(ProviderArithmetic(), &payload);
        if (node != o)
            return SpaceFail(error, "op node " + std::to_string(o) +
                                        " was numbered " +
                                        std::to_string(node));
        const std::pair<ProviderClass, ProviderRead> lists[] = {
            {ProviderClass::Matrix, ProviderRead::Matrices},
            {ProviderClass::Matrix, ProviderRead::Authoritative},
            {ProviderClass::Frame, ProviderRead::Frames},
            {ProviderClass::Scalar, ProviderRead::Scalars},
            {ProviderClass::Vector, ProviderRead::Vectors},
            {ProviderClass::Order, ProviderRead::Orders},
            {ProviderClass::Reset, ProviderRead::Resets},
        };
        const RigExecPortHandle *ports[] = {
            &ProviderArithmeticMatrices(), &ProviderArithmeticAuthoritative(),
            &ProviderArithmeticFrames(),   &ProviderArithmeticScalars(),
            &ProviderArithmeticVectors(),  &ProviderArithmeticOrders(),
            &ProviderArithmeticResets(),
        };
        for (size_t l = 0; l < std::size(lists); ++l)
            if (counts[size_t(lists[l].first)])
                lookUp(node, *ports[l], index, lists[l].second);
        lookUp(node, ProviderArithmeticPresent(), index, ProviderRead::Present);
        store(node, ProviderArithmeticUnavailable(),
              T::ProviderValueUnavailable, index);
        store(node, ProviderArithmeticAuthoritativeOut(),
              T::ProviderValueAuthority, index);
        if (eval::spaces::ProviderArithmeticYieldsFrame(op.kind))
            store(node, ProviderArithmeticFrame(), T::ProviderValueFrame,
                  index);
        else
            store(node, ProviderArithmeticMatrix(), T::ProviderValueMatrix,
                  index);
    }
    return true;
}

int
RigExecBakedProviderOpOf(const RigExecBakedProgramImpl &B,
                         const RigExecBakedStep &step)
{
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::ProviderOp
               ? int(origin->object)
               : -1;
}

bool
RigExecBakedLookUpSpace(const RigExecBakedProgramImpl &B,
                        const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *out,
                        std::string *error)
{
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderInputs)) {
        const RigExecProviderProgram &program = B.GetProviderProgram();
        if (lookup.object >= program.ops.size() || !type.element)
            return SpaceFail(error, "no provider operation " +
                                        std::to_string(lookup.object) +
                                        " fills a list");
        const RigExecProviderOp &op = program.ops[lookup.object];
        const ProviderRead read = ProviderRead(lookup.read);
        return op.kind == RigExecProviderOpKind::AffineFrame
                   ? FillAffine(B.providerValues, op, read, out, error)
                   : FillArithmetic(B.providerValues, op, read, out, error);
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::RestFrame)) {
        if (type.size != sizeof(rigExecRig::PointFrame) ||
            lookup.object >= B.restFrames.size())
            return SpaceFail(error, "slot " + std::to_string(lookup.object) +
                                        " has no rest frame");
        PutFrame(B.restFrames[lookup.object],
                 static_cast<rigExecRig::PointFrame *>(out));
        return true;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderFrame)) {
        const RigExecPointFrame *frame =
            type.size == sizeof(rigExecRig::PointFrame) &&
                    lookup.object < B.providerRefreshes.size()
                ? B.providerValues.Read<RigExecPointFrame>(
                      lookup.read
                          ? B.providerRefreshes[lookup.object].currentValue
                          : B.providerRefreshes[lookup.object].baseValue)
                : nullptr;
        if (!frame)
            return SpaceFail(error, lookup.read ? "connected final pose input "
                                                  "incomplete"
                                                : "connected base pose input "
                                                  "incomplete");
        PutFrame(*frame, static_cast<rigExecRig::PointFrame *>(out));
        return true;
    }
    if (lookup.source == uint8_t(eval::OpLookupSource::ProviderParentRaw)) {
        if (type.size != sizeof(GfMatrix4d) ||
            lookup.object >= B.providerParentRawLeaves.size())
            return SpaceFail(error, "slot " + std::to_string(lookup.object) +
                                        " has no parent space");
        const GfMatrix4d raw =
            RigExecBakedProviderParentRaw(B, int(lookup.object));
        std::memcpy(out, raw.data(), sizeof(GfMatrix4d));
        return true;
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::CommitRecordGate) ||
        lookup.object >= B.commits.size() || lookup.read > 3)
        return SpaceFail(error, "commit " + std::to_string(lookup.object) +
                                    " holds no such record gate");
    const RigExecBakedCommit &commit = B.commits[lookup.object];
    if (lookup.read == 3) {
        if (type.size != sizeof(uint8_t) || lookup.part < 0 ||
            size_t(lookup.part) >= commit.present.size())
            return SpaceFail(error, "commit " + std::to_string(lookup.object) +
                                        " has no output " +
                                        std::to_string(lookup.part));
        *static_cast<uint8_t *>(out) =
            !commit.abandoned && commit.present[size_t(lookup.part)] ? 1 : 0;
        return true;
    }
    if (lookup.read == 0) {
        if (!type.borrow)
            return SpaceFail(error, "a commit's presence is an array");
        type.borrow(out, commit.present.data(), commit.present.size());
        return true;
    }
    if (type.size != sizeof(uint8_t))
        return SpaceFail(error, "a commit's record gate is a Bool");
    const bool gate =
        lookup.read == 1 ? commit.recordAfter : commit.recordEveryTarget;
    *static_cast<uint8_t *>(out) = gate ? 1 : 0;
    return true;
}

bool
RigExecBakedStoreSpace(RigExecBakedProgramImpl &B,
                       const eval::OpHostOutput &output, const void *value,
                       std::string *error)
{
    using T = eval::OpOutputTarget;
    const T target = T(output.target);
    if (target == T::ProviderValueMatrix || target == T::ProviderValueFrame ||
        target == T::ProviderValueAuthority ||
        target == T::ProviderValueUnavailable) {
        const RigExecProviderProgram &program = B.GetProviderProgram();
        if (output.object >= program.ops.size() ||
            output.object >= B.providerOpPending.size())
            return SpaceFail(error, "no provider operation " +
                                        std::to_string(output.object));
        auto &pending = B.providerOpPending[output.object];
        if (target == T::ProviderValueAuthority) {
            pending.first = *static_cast<const uint8_t *>(value);
            return true;
        }
        if (target == T::ProviderValueUnavailable) {
            pending.second = *static_cast<const int32_t *>(value);
            return true;
        }
        const RigExecProviderOp &op = program.ops[output.object];
        const auto [authority, unavailable] = pending;
        pending = {0, -1};
        if (unavailable >= 0) {
            const RigExecValueId source = size_t(unavailable) < op.inputs.size()
                                              ? op.inputs[size_t(unavailable)]
                                              : RigExecNoProviderValue;
            const bool blocked =
                source < B.providerValues.values.size() &&
                B.providerValues.values[size_t(source)].blocked;
            B.providerValues.values[size_t(op.output)].raw = VtValue();
            B.providerValues.Publish(op.output, std::monostate(), blocked,
                                     false, 0, "provider input unavailable");
            return true;
        }
        if (target == T::ProviderValueFrame) {
            B.providerValues.Publish(
                op.output,
                TakeFrame(*static_cast<const rigExecRig::PointFrame *>(value)));
            return true;
        }
        GfMatrix4d matrix;
        std::memcpy(matrix.data(), value, sizeof(GfMatrix4d));
        B.providerValues.Publish(op.output, matrix, false, authority != 0);
        return true;
    }
    if (target >= T::RefreshBase) {
        if (output.object >= B.providerRefreshes.size())
            return SpaceFail(error, "no provider refresh " +
                                        std::to_string(output.object));
        const auto &refresh = B.providerRefreshes[output.object];
        if (target == T::RefreshBase || target == T::RefreshFinal) {
            const auto &frame =
                *static_cast<const rigExecRig::PointFrame *>(value);
            (target == T::RefreshBase ? B.base[refresh.baseWrite]
                                      : B.fin[refresh.finWrite]) =
                TakeFrame(frame);
            return true;
        }
        const auto &frames = *static_cast<
            const rigExecStd::RigExecArray<rigExecRig::PointFrame> *>(value);
        if (frames.Size() != refresh.carries.size())
            return SpaceFail(error, "provider refresh " +
                                        std::to_string(output.object) +
                                        " carries " +
                                        std::to_string(refresh.carries.size()) +
                                        " descendants");
        for (size_t k = 0; k < frames.Size(); ++k) {
            const auto &carry = refresh.carries[k];
            (target == T::RefreshCarryBase ? B.base[carry.baseWrite]
                                           : B.fin[carry.finWrite]) =
                TakeFrame(frames[k]);
        }
        return true;
    }
    if (target == T::FrameMatrixValid) {
        if (output.object >= B.frameMatrixValid.size())
            return SpaceFail(error, "frame record " +
                                        std::to_string(output.object) +
                                        " has no slot");
        B.frameMatrixValid[output.object] =
            *static_cast<const uint8_t *>(value) != 0;
        return true;
    }
    std::vector<GfMatrix4d> &matrices =
        target == T::ProviderFinalMatrix  ? B.finalMatrix
        : target == T::ProviderBaseMatrix ? B.baseMatrix
                                          : B.frameMatrix;
    if (output.object >= matrices.size())
        return SpaceFail(error, "no matrix " + std::to_string(output.object) +
                                    " to store");
    std::memcpy(matrices[output.object].data(), value, sizeof(GfMatrix4d));
    return true;
}

} // namespace rigExec
