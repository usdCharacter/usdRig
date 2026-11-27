// The baked program's EvalCore pose Host: program tables and Gf math
// behind the pose.h binding. Internal to RigExec, never an exported API.
#ifndef RIGEXEC_POSE_EVAL_HOST_H
#define RIGEXEC_POSE_EVAL_HOST_H

#include "bakedExecCrossCheckRows.h"
#include "bakedProgramImpl.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/rangeRunner.h"
#include "rigExecScene/poseProgram.h"

#include <algorithm>

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/vec3d.h"

namespace rigExec {

// Decode the sampled USD values; EvalCore owns table size and default policy.
inline void
PrepareConstraintArrays(RigExecBakedProgramImpl::ConstraintArrays *arrays)
{
    auto &a = *arrays;
    a.diagnostics.clear();
    a.poleDiagnostics.clear();
    const auto weights = [&](size_t channel, size_t count) {
        const auto *raw = a.raw[channel].IsHolding<VtFloatArray>()
                              ? &a.raw[channel].UncheckedGet<VtFloatArray>()
                              : nullptr;
        const char *name = channel == 0
                               ? SchemaAttrText::inputsSourceWeights
                               : SchemaAttrText::inputsPoleVectorWeights;
        auto &diagnostics = channel == 0 ? a.diagnostics : a.poleDiagnostics;
        auto &values = channel == 0 ? a.weights : a.poleWeights;
        return eval::NormalizeConstraintArray<double>(
            channel, count, raw ? raw->size() : 0, raw != nullptr,
            [&](size_t k) { return double((*raw)[k]); },
            [&](size_t size, size_t expected) {
                diagnostics.push_back(a.pathText + " " + name + " has " +
                                      std::to_string(size) + " entries for " +
                                      std::to_string(expected) + " sources");
            },
            &values);
    };
    const auto offsets = [&](size_t channel, size_t count, bool enabled) {
        const auto *raw = enabled && a.raw[channel].IsHolding<VtVec3dArray>()
                              ? &a.raw[channel].UncheckedGet<VtVec3dArray>()
                              : nullptr;
        const char *name = channel == 1
                               ? SchemaAttrText::inputsTranslationOffsets
                               : SchemaAttrText::inputsRotationOffsets;
        auto &values = channel == 1 ? a.translationOffsets : a.rotationOffsets;
        return eval::NormalizeConstraintArray<GfVec3d>(
            channel, count, raw ? raw->size() : 0, raw != nullptr,
            [&](size_t k) { return (*raw)[k]; },
            [&](size_t size, size_t expected) {
                a.diagnostics.push_back(a.pathText + " " + name + " has " +
                                        std::to_string(size) + " entries for " +
                                        std::to_string(expected) + " sources");
            },
            &values);
    };
    eval::NormalizeConstraintTables(a.sourceCount, a.poleCount, a.parentOffsets,
                                    a.readPole, weights, offsets, &a.ok,
                                    &a.poleOk);
}

struct BakedPoseMath {
    using Mat4 = GfMatrix4d;
    using Mat3 = GfMatrix3d;
    using Vec3 = GfVec3d;
    using Quat = GfQuatd;
    using Frame = RigExecPointFrame;
    static Mat4 Identity4()
    {
        return Mat4(1.0);
    }
    static Mat3 Identity3()
    {
        return Mat3(1.0);
    }
    static Mat4 Mat4FromElements(double m00, double m01, double m02, double m03,
                                 double m10, double m11, double m12, double m13,
                                 double m20, double m21, double m22, double m23,
                                 double m30, double m31, double m32, double m33)
    {
        return Mat4(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23,
                    m30, m31, m32, m33);
    }
    static Mat3 Mat3FromElements(double m00, double m01, double m02, double m10,
                                 double m11, double m12, double m20, double m21,
                                 double m22)
    {
        return Mat3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
    static Vec3 Vec3Of(double x, double y, double z)
    {
        return Vec3(x, y, z);
    }
    static void SinCos(double v, double *s, double *c)
    {
        GfSinCos(v, s, c);
    }
    static Quat QuatFromComponents(double w, double x, double y, double z)
    {
        return Quat(w, Vec3(x, y, z));
    }
    static void QuatToComponents(const Quat &q, double *w, Vec3 *im)
    {
        *w = q.GetReal();
        *im = q.GetImaginary();
    }
    static Frame MakeFrame(const std::array<Vec3, 4> &points, unsigned flags)
    {
        Frame frame;
        frame.points = points;
        frame.flags = flags;
        return frame;
    }
};

// Borrowed views over the program tables. All borrowed storage
// outlives the step call.
class BakedPoseInputs {
public:
    explicit BakedPoseInputs(const RigExecBakedProgramImpl *program)
        : _program(program)
    {
    }

    size_t avarWriteCount(size_t object) const
    {
        const auto varying = RigExecBakedAvarBindingRange(
            _program->avarBindingBegin, int64_t(object));
        const auto constant = RigExecBakedAvarBindingRange(
            _program->avarConstantBindingBegin, int64_t(object));
        return (varying.second - varying.first) +
               (constant.second - constant.first);
    }

    size_t avarWriteSlot(size_t object, size_t k) const
    {
        const auto varying = RigExecBakedAvarBindingRange(
            _program->avarBindingBegin, int64_t(object));
        const size_t n0 = varying.second - varying.first;
        if (k < n0) {
            return _program->avarBindings[varying.first + k].slot;
        }
        const auto constant = RigExecBakedAvarBindingRange(
            _program->avarConstantBindingBegin, int64_t(object));
        return _program->avarConstantBindings[constant.first + (k - n0)].slot;
    }

    double avarWriteValue(size_t object, size_t k) const
    {
        const auto varying = RigExecBakedAvarBindingRange(
            _program->avarBindingBegin, int64_t(object));
        const size_t n0 = varying.second - varying.first;
        if (k < n0) {
            return RigExecBakedLeafRead(
                *_program, _program->avarBindings[varying.first + k].input);
        }
        const auto constant = RigExecBakedAvarBindingRange(
            _program->avarConstantBindingBegin, int64_t(object));
        return RigExecBakedLeafRead(
            *_program,
            _program->avarConstantBindings[constant.first + (k - n0)].input);
    }

    size_t checkpointObjectCount() const
    {
        return _program->switchFrameContexts.size();
    }

    size_t checkpointRecomposeCount(size_t object) const
    {
        return _program->switchFrameContexts[object].recompose.size();
    }

    bool checkpointSlot(size_t object, size_t k,
                        eval::PoseSlotComposeT<BakedPoseMath> *view) const
    {
        const auto &recompose = _program->switchFrameContexts[object].recompose;
        if (k >= recompose.size() || recompose[k] < 0) {
            return false;
        }
        return slotCompose(size_t(recompose[k]), view);
    }

    int checkpointAnchor(size_t object) const
    {
        return _program->switchFrameContexts[object].anchor;
    }

    bool posedMatrix(size_t slot, GfMatrix4d *out) const
    {
        if (slot >= _program->posedM.size()) {
            return false;
        }
        *out = _program->posedM[slot];
        return true;
    }

    size_t composeGroupCount() const
    {
        return _program->composeGroups.size();
    }

    void composeGroupRange(size_t object, int *begin, int *end) const
    {
        const auto &group = _program->composeGroups[object];
        *begin = group.begin;
        *end = group.end;
    }

    bool isPoseSlot(size_t slot) const
    {
        return slot < _program->slotKind.size() &&
               _program->slotKind[slot] == RigExecBakedSlotKind::FirstFramePose;
    }

    bool composeSlotBounds(size_t slot) const
    {
        return slot < _program->slotKind.size() &&
               slot < _program->base.size() && slot < _program->fin.size() &&
               slot < _program->posedM.size() && slot < _program->restM.size();
    }

    bool providerActive(size_t slot) const
    {
        return _program->providerActive.empty() ||
               (slot < _program->providerActive.size() &&
                _program->providerActive[slot]);
    }

    int parent(size_t slot) const
    {
        return slot < _program->parent.size() ? _program->parent[slot] : -1;
    }

    double restAvar(size_t slot, size_t k) const
    {
        return RigExecBakedLeafRead(*_program,
                                    _program->ladders[slot].restAvars[k]);
    }

    double defaultAvar(size_t slot, size_t k) const
    {
        return RigExecBakedLeafRead(*_program,
                                    _program->ladders[slot].defaultAvars[k]);
    }

    GfMatrix4d ladderSpace(size_t slot, int channel) const
    {
        const auto &L = _program->ladders[slot];
        switch (channel) {
        case eval::kPoseLadderRestSpace:
            return _SpaceMatrix(L, 0,
                                RigExecBakedLeafRead(*_program, L.restSpace));
        case eval::kPoseLadderDefaultSpace:
            return _SpaceMatrix(
                L, 1, RigExecBakedLeafRead(*_program, L.defaultSpace));
        case eval::kPoseLadderPosedSpace:
            return _SpaceMatrix(L, 2,
                                RigExecBakedLeafRead(*_program, L.posedSpace));
        case eval::kPoseLadderParentSpace:
            return _SpaceMatrix(L, 3,
                                RigExecBakedLeafRead(*_program, L.parentSpace));
        case eval::kPoseLadderParentDefaultSpace:
            return _SpaceMatrix(
                L, 4, RigExecBakedLeafRead(*_program, L.parentDefaultSpace));
        case eval::kPoseLadderAvarDefaultSpace:
            return _SpaceMatrix(
                L, 5, RigExecBakedLeafRead(*_program, L.avarDefaultSpace));
        default:
            return _SpaceMatrix(
                L, 6, RigExecBakedLeafRead(*_program, L.posedDefaultSpace));
        }
    }

    GfMatrix4d interveningSpace(size_t slot) const
    {
        return RigExecBakedLeafRead(*_program,
                                    _program->ladders[slot].interveningSpace);
    }

    GfMatrix4d ladderParentSpaceLeaf(size_t slot) const
    {
        return RigExecBakedLeafRead(*_program,
                                    _program->ladders[slot].parentSpace);
    }

    bool ladderChannelReady(size_t slot, int channel) const
    {
        const auto &L = _program->ladders[slot];
        const int id = L.spaceValues[size_t(channel)];
        return id >= 0 &&
               _program->providerValues.Read<GfMatrix4d>(RigExecValueId(id));
    }

    bool ladderAuthoritative(size_t slot, int channel) const
    {
        const auto &L = _program->ladders[slot];
        const RigExecBakedInput<GfMatrix4d> *input = nullptr;
        bool connected = false;
        switch (channel) {
        case eval::kPoseLadderDefaultSpace:
            input = &L.defaultSpace;
            connected = L.defaultSpaceConnected;
            break;
        case eval::kPoseLadderPosedSpace:
            input = &L.posedSpace;
            connected = L.posedSpaceConnected;
            break;
        case eval::kPoseLadderParentSpace:
            input = &L.parentSpace;
            connected = L.parentSpaceConnected;
            break;
        case eval::kPoseLadderParentDefaultSpace:
            input = &L.parentDefaultSpace;
            connected = L.parentDefaultSpaceConnected;
            break;
        case eval::kPoseLadderAvarDefaultSpace:
            input = &L.avarDefaultSpace;
            connected = L.avarDefaultSpaceConnected;
            break;
        case eval::kPoseLadderPosedDefaultSpace:
            input = &L.posedDefaultSpace;
            connected = L.posedDefaultSpaceConnected;
            break;
        default:
            return false;
        }
        const int index = input->overrideIndex;
        return connected ||
               (index >= 0 && ((size_t(index) < _program->overridden.size() &&
                                _program->overridden[size_t(index)]) ||
                               (size_t(index) < _program->upstreamOn.size() &&
                                _program->upstreamOn[size_t(index)])));
    }

    GfVec3d ladderRotationSign(size_t slot) const
    {
        return RigExecBakedLeafRead(*_program,
                                    _program->ladders[slot].rotationSign);
    }

    bool restRoundTrip(size_t slot, GfMatrix4d *out) const
    {
        if (slot >= _program->restRoundTrip.size()) {
            return false;
        }
        *out = _program->restRoundTrip[slot];
        return true;
    }

    bool defaultRoundTrip(size_t slot, GfMatrix4d *out) const
    {
        if (slot >= _program->defaultRoundTrip.size()) {
            return false;
        }
        *out = _program->defaultRoundTrip[slot];
        return true;
    }

    void baseFrame(size_t slot, std::array<GfVec3d, 4> *points,
                   unsigned *flags) const
    {
        const RigExecPointFrame &frame = _program->base[slot];
        *points = frame.points;
        *flags = frame.flags;
    }

    size_t commitObjectCount() const
    {
        return _program->commits.size();
    }

    eval::PoseCommitFacts commitFacts(size_t object) const
    {
        const RigExecBakedCommit &commit = _program->commits[object];
        eval::PoseCommitFacts facts;
        const auto span = [](const std::vector<uint32_t> &v) {
            eval::PoseU32List list;
            list.data = v.data();
            list.count = v.size();
            return list;
        };
        facts.slotReads = span(commit.slotReads);
        facts.slotWrites = span(commit.slotWrites);
        facts.slotBaseWrites = span(commit.slotBaseWrites);
        facts.slotCarry = span(commit.slotCarry);
        facts.slotBaseCarry = span(commit.slotBaseCarry);
        facts.descendantReads = span(commit.descendantReads);
        facts.closestReads = span(commit.closestReads);
        facts.descendantWrites = span(commit.descendantWrites);
        facts.descendantBaseWrites = span(commit.descendantBaseWrites);
        facts.descendantCarry = span(commit.descendantCarry);
        facts.descendantBaseCarry = span(commit.descendantBaseCarry);
        facts.sourceReads = span(commit.sourceReads);
        facts.targetReads = span(commit.targetReads);
        facts.poleReads = span(commit.poleReads);
        facts.worldUpRead = commit.worldUpRead;
        facts.targetRead = commit.targetRead;
        facts.effectorRead = commit.effectorRead;
        facts.solverOutput = commit.solverOutput;
        facts.moverPath = commit.moverPathText;
        return facts;
    }

    size_t commitSlotCount(size_t object) const
    {
        return _program->commits[object].slots.size();
    }

    size_t commitPropagateCount(size_t object) const
    {
        return _program->commits[object].propagate.size();
    }

    bool commitPropagateAt(size_t object, size_t k, int *first,
                           int *second) const
    {
        const auto &propagate = _program->commits[object].propagate;
        if (k >= propagate.size()) {
            return false;
        }
        *first = propagate[k].first;
        *second = propagate[k].second;
        return true;
    }

    bool commitClosestPosAt(size_t object, size_t k, int *pos) const
    {
        const auto &closestPos = _program->commits[object].closestPos;
        if (k >= closestPos.size()) {
            return false;
        }
        *pos = closestPos[k];
        return true;
    }

    bool commitPropParent(int slot, bool *authored, int *next) const
    {
        if (slot < 0) {
            return false;
        }
        const size_t i = size_t(slot);
        if (i >= _program->propParent.size() ||
            i >= _program->parentSpaceAuthored.size()) {
            return false;
        }
        *authored = _program->parentSpaceAuthored[i] != 0;
        *next = _program->propParent[i];
        return true;
    }

    std::string commitPathText(size_t slot) const
    {
        if (slot >= _program->pathTexts->size()) {
            return std::string();
        }
        return (*_program->pathTexts)[slot];
    }

    bool finFrame(uint32_t version, std::array<GfVec3d, 4> *points,
                  unsigned *flags) const
    {
        if (size_t(version) >= _program->fin.size()) {
            return false;
        }
        const RigExecPointFrame &frame = _program->fin[size_t(version)];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool restFrame(size_t slot, std::array<GfVec3d, 4> *points,
                   unsigned *flags) const
    {
        if (slot >= _program->restFrames.size()) {
            return false;
        }
        const RigExecPointFrame &frame = _program->restFrames[slot];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool ikFkFacts(size_t object, eval::PoseIkFkFacts *facts) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        facts->envelope = RigExecBakedLeafRead(*_program, c.defaultWeight);
        for (size_t k = 0; k < 7; ++k) {
            const auto &scalar = c.matchScalars[k];
            facts->scalars[k] =
                scalar.isFloat
                    ? double(RigExecBakedLeafRead(*_program, scalar.narrow))
                    : RigExecBakedLeafRead(*_program, scalar.wide);
        }
        facts->ikValue = c.matchIkValue;
        facts->scaleCalibration = c.matchScaleCalibration;
        facts->upperOffset = c.matchUpperOffset;
        facts->lowerOffset = c.matchLowerOffset;
        facts->stretchLimb = c.matchStretchLimb;
        return true;
    }

    bool matchChannels(size_t slot, RigExecIkFkMatchChannels *out) const
    {
        if (slot * 11 + 11 > _program->avars.size() ||
            slot >= _program->noScaleAvars.size() ||
            slot >= _program->rotOrder.size()) {
            return false;
        }
        RigExecBakedMatchChannelsOf(*_program, int(slot), out);
        return true;
    }

    const RigExecSpaceSwitchInputs *matchPoleSwitch(size_t object,
                                                    uint32_t *mode) const
    {
        const RigExecBakedProgramImpl &B = *_program;
        const RigExecBakedProgramImpl::Constraint &c =
            B.constraints[size_t(B.walkSteps[object].index)];
        const int pole = c.sources.size() > 6 ? c.sources[6] : -1;
        const bool switched =
            c.matchPoleSwitch >= 0 &&
            size_t(c.matchPoleSwitch) < B.spaceSwitches.size();
        *mode = c.matchPoleParentRides ? 2u : 0u;
        if (switched && pole >= 0 && size_t(pole) < B.posedAuthored.size() &&
            !B.posedAuthored[size_t(pole)])
            *mode = 1u;
        return switched
                   ? &B.spaceSwitches[size_t(c.matchPoleSwitch)].kernelInputs
                   : nullptr;
    }

    eval::PoseIkChainView ikChainView(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        eval::PoseIkChainView view;
        view.slots = c.targetSlots.data();
        view.count = c.targetSlots.size();
        view.ikMode = int(c.ikMode);
        view.useAnimatedTs = c.useAnimatedTs;
        view.poleModeObject = c.poleModeObject;
        view.poleObjects = c.poleObjects.size();
        return view;
    }

    bool ikRestLive(size_t object, size_t k) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        return k < c.ikRestLive.size() && c.ikRestLive[k];
    }

    double ikStretch(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        return double(RigExecBakedLeafRead(*_program, c.stretch));
    }

    eval::PoseU32List commitTargetReads(size_t object) const
    {
        const std::vector<uint32_t> &reads =
            _program->commits[object].targetReads;
        eval::PoseU32List list;
        list.data = reads.data();
        list.count = reads.size();
        return list;
    }

    void ikPoleVector(size_t object, GfVec3d *pole, double *twistDegrees) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        *pole = RigExecBakedLeafRead(*_program, c.poleVector);
        *twistDegrees = RigExecBakedLeafRead(*_program, c.twistDegrees);
    }

    bool resolveIkEffector(size_t object, std::array<GfVec3d, 4> *points,
                           unsigned *flags) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        if (c.effector >= 0) {
            if (size_t(commit.effectorRead) >= _program->fin.size()) {
                return false;
            }
            *points = _program->fin[size_t(commit.effectorRead)].points;
            *flags = _program->fin[size_t(commit.effectorRead)].flags;
            return (*flags & RigExecPointFrameValid) != 0;
        }
        if (c.effectorNative < 0 ||
            size_t(c.effectorNative) >= _program->nativeFrameOk.size() ||
            size_t(c.effectorNative) >= _program->nativeFrames.size() ||
            size_t(c.effectorNative) >= _program->nativeSources.size() ||
            !_program->nativeFrameOk[size_t(c.effectorNative)]) {
            return false;
        }
        *points = _program->nativeFrames[size_t(c.effectorNative)].points;
        *flags = _program->nativeFrames[size_t(c.effectorNative)].flags;
        if ((*flags & RigExecPointFrameValid) == 0) {
            return false;
        }
        const auto enumerate = [&](const RigExecPoseFrameVisitor &visit) {
            for (const RigExecBakedCommit::AncestorRead &a :
                 commit.effectorAncestors) {
                if (a.slot < 0 || size_t(a.slot) >= _program->paths.size() ||
                    size_t(a.base) >= _program->base.size() ||
                    size_t(a.fin) >= _program->fin.size()) {
                    continue;
                }
                visit(_program->paths[size_t(a.slot)],
                      _program->base[size_t(a.base)],
                      _program->fin[size_t(a.fin)]);
            }
        };
        const RigExecPoseFrameEnumerator providers(enumerate);
        RigExecPointFrame frame;
        frame.points = *points;
        frame.flags = *flags;
        if (!RigExecApplyRevisedAncestorDelta(
                _program->nativeSources[size_t(c.effectorNative)].path,
                providers, &frame)) {
            return false;
        }
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool ikPoleBounds(size_t object, size_t k) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        return k < c.poleObjects.size() && k < commit.poleReads.size() &&
               k < c.poleObjectNatives.size() &&
               k < commit.poleAncestors.size();
    }

    bool resolveIkPole(size_t object, size_t k, std::array<GfVec3d, 4> *points,
                       unsigned *flags) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        if (k >= c.poleObjects.size() || k >= commit.poleReads.size() ||
            k >= c.poleObjectNatives.size() ||
            k >= commit.poleAncestors.size()) {
            return false;
        }
        if (c.poleObjects[k] >= 0) {
            if (size_t(commit.poleReads[k]) >= _program->fin.size()) {
                return false;
            }
            *points = _program->fin[size_t(commit.poleReads[k])].points;
            *flags = _program->fin[size_t(commit.poleReads[k])].flags;
            return (*flags & RigExecPointFrameValid) != 0;
        }
        const int native = c.poleObjectNatives[k];
        if (native < 0 || size_t(native) >= _program->nativeFrameOk.size() ||
            size_t(native) >= _program->nativeFrames.size() ||
            size_t(native) >= _program->nativeSources.size() ||
            !_program->nativeFrameOk[size_t(native)]) {
            return false;
        }
        *points = _program->nativeFrames[size_t(native)].points;
        *flags = _program->nativeFrames[size_t(native)].flags;
        if ((*flags & RigExecPointFrameValid) == 0) {
            return false;
        }
        const auto enumerate = [&](const RigExecPoseFrameVisitor &visit) {
            for (const RigExecBakedCommit::AncestorRead &a :
                 commit.poleAncestors[k]) {
                if (a.slot < 0 || size_t(a.slot) >= _program->paths.size() ||
                    size_t(a.base) >= _program->base.size() ||
                    size_t(a.fin) >= _program->fin.size()) {
                    continue;
                }
                visit(_program->paths[size_t(a.slot)],
                      _program->base[size_t(a.base)],
                      _program->fin[size_t(a.fin)]);
            }
        };
        const RigExecPoseFrameEnumerator providers(enumerate);
        RigExecPointFrame frame;
        frame.points = *points;
        frame.flags = *flags;
        if (!RigExecApplyRevisedAncestorDelta(
                _program->nativeSources[size_t(native)].path, providers,
                &frame)) {
            return false;
        }
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool commitSlotAt(size_t object, size_t pos, int *slot) const
    {
        if (pos >= _program->commits[object].slots.size()) {
            return false;
        }
        *slot = _program->commits[object].slots[pos];
        return true;
    }

    bool baseFrameAt(uint32_t version, std::array<GfVec3d, 4> *points,
                     unsigned *flags) const
    {
        if (size_t(version) >= _program->base.size()) {
            return false;
        }
        const RigExecPointFrame &frame = _program->base[size_t(version)];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    double harmonicsScalar(size_t object, size_t k) const
    {
        const auto &s =
            _program->constraints[size_t(_program->walkSteps[object].index)]
                .harmonicsScalars[k];
        return s.isFloat ? double(RigExecBakedLeafRead(*_program, s.narrow))
                         : RigExecBakedLeafRead(*_program, s.wide);
    }

    bool historyTick() const
    {
        return _program->historyTick;
    }

    eval::PoseConstraintFactsT<BakedPoseMath>
    constraintFacts(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        eval::PoseConstraintFactsT<BakedPoseMath> facts;
        facts.target = c.target;
        facts.deltaBase = c.deltaBase;
        facts.spaceSlot = c.spaceSlot;
        facts.pathText = c.pathText;
        facts.enabled = RigExecBakedLeafRead(*_program, c.enabled);
        facts.hasWeightObject = !c.weightObject.IsEmpty();
        facts.pointsTarget = !c.pointsTarget.IsEmpty();
        if (facts.hasWeightObject && c.weightField >= 0 &&
            size_t(c.weightField) < _program->weightFields.size()) {
            const RigExecBakedProgramImpl::WeightField &field =
                _program->weightFields[size_t(c.weightField)];
            facts.fieldOk = field.ok;
            facts.fieldCount = field.values.size();
            facts.fieldValue =
                field.values.empty() ? 0.0 : double(field.values[0]);
            facts.fieldError = field.error;
        } else if (facts.hasWeightObject) {
            facts.fieldOk = false;
            facts.fieldCount = 0;
        }
        facts.defaultWeight =
            double(RigExecBakedLeafRead(*_program, c.defaultWeight));
        facts.code = eval::kPoseConstraintUnknown;
        if (c.type == SchemaTypeName::RigExecMatrixMover) {
            facts.code = eval::kPoseConstraintMatrixMover;
        } else if (c.type == SchemaTypeName::RigExecPositionConstraint) {
            facts.code = eval::kPoseConstraintPosition;
        } else if (c.type == SchemaTypeName::RigExecRotationConstraint) {
            facts.code = eval::kPoseConstraintRotation;
        } else if (c.type == SchemaTypeName::RigExecScaleConstraint) {
            facts.code = eval::kPoseConstraintScale;
        } else if (c.type == SchemaTypeName::RigExecParentConstraint) {
            facts.code = eval::kPoseConstraintParent;
        } else if (c.type == SchemaTypeName::RigExecAimConstraint) {
            facts.code = eval::kPoseConstraintAim;
        }
        facts.ikFkMatch = c.ikFkMatch;
        facts.harmonics = c.harmonics;
        facts.radialBlend = c.radialBlend;
        facts.singleChainIk = c.singleChainIk;
        facts.arrays = constraintArrays(object);
        if (c.arrays < 0 ||
            size_t(c.arrays) >= _program->constraintArrays.size()) {
            return facts;
        }
        facts.affectX = RigExecBakedLeafRead(*_program, c.affectX);
        facts.affectY = RigExecBakedLeafRead(*_program, c.affectY);
        facts.affectZ = RigExecBakedLeafRead(*_program, c.affectZ);
        facts.order = static_cast<eval::pose_constraints::PoseEulerOrder>(
            static_cast<int>(c.order));
        facts.offset = RigExecBakedLeafRead(*_program, c.offset);
        facts.blendShear = c.blendShear;
        facts.translationAxes.x = RigExecBakedLeafRead(*_program, c.tX);
        facts.translationAxes.y = RigExecBakedLeafRead(*_program, c.tY);
        facts.translationAxes.z = RigExecBakedLeafRead(*_program, c.tZ);
        facts.rotationAxes.x = RigExecBakedLeafRead(*_program, c.rX);
        facts.rotationAxes.y = RigExecBakedLeafRead(*_program, c.rY);
        facts.rotationAxes.z = RigExecBakedLeafRead(*_program, c.rZ);
        facts.scaleAxes.x = RigExecBakedLeafRead(*_program, c.sX);
        facts.scaleAxes.y = RigExecBakedLeafRead(*_program, c.sY);
        facts.scaleAxes.z = RigExecBakedLeafRead(*_program, c.sZ);
        return facts;
    }

    size_t constraintSourceCount(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        return c.sources.size();
    }

    bool constraintSourceRef(size_t object, size_t k,
                             eval::PoseSourceRef *ref) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        if (k >= c.sources.size() || k >= commit.sourceReads.size() ||
            k >= c.sourceNatives.size() || k >= c.sourcePathTexts.size()) {
            return false;
        }
        ref->slot = c.sources[k];
        ref->finRead = commit.sourceReads[k];
        ref->native = c.sourceNatives[k];
        ref->pathText = c.sourcePathTexts[k];
        return true;
    }

    bool resolveNativeSource(size_t object, size_t k,
                             std::array<GfVec3d, 4> *points,
                             unsigned *flags) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        const int native = c.sourceNatives[k];
        if (native < 0 || size_t(native) >= _program->nativeFrameOk.size() ||
            size_t(native) >= _program->nativeFrames.size() ||
            size_t(native) >= _program->nativeSources.size() ||
            !_program->nativeFrameOk[size_t(native)]) {
            return false;
        }
        if (k >= commit.sourceAncestors.size()) {
            return false;
        }
        *points = _program->nativeFrames[size_t(native)].points;
        *flags = _program->nativeFrames[size_t(native)].flags;
        if ((*flags & RigExecPointFrameValid) == 0) {
            return false;
        }
        const auto &ancestors = commit.sourceAncestors[k];
        const auto enumerate = [&](const RigExecPoseFrameVisitor &visit) {
            for (const RigExecBakedCommit::AncestorRead &a : ancestors) {
                if (a.slot < 0 || size_t(a.slot) >= _program->paths.size() ||
                    size_t(a.base) >= _program->base.size() ||
                    size_t(a.fin) >= _program->fin.size()) {
                    continue;
                }
                visit(_program->paths[size_t(a.slot)],
                      _program->base[size_t(a.base)],
                      _program->fin[size_t(a.fin)]);
            }
        };
        const RigExecPoseFrameEnumerator providers(enumerate);
        RigExecPointFrame frame;
        frame.points = *points;
        frame.flags = *flags;
        if (!RigExecApplyRevisedAncestorDelta(
                _program->nativeSources[size_t(native)].path, providers,
                &frame)) {
            return false;
        }
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool resolveWorldUp(size_t object, std::array<GfVec3d, 4> *points,
                        unsigned *flags) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        if (c.worldUpObject >= 0) {
            if (!finFrame(commit.worldUpRead, points, flags)) {
                return false;
            }
            return (*flags & RigExecPointFrameValid) != 0;
        }
        const int native = c.worldUpNative;
        if (native < 0 || size_t(native) >= _program->nativeFrameOk.size() ||
            size_t(native) >= _program->nativeFrames.size() ||
            size_t(native) >= _program->nativeSources.size() ||
            !_program->nativeFrameOk[size_t(native)]) {
            return false;
        }
        *points = _program->nativeFrames[size_t(native)].points;
        *flags = _program->nativeFrames[size_t(native)].flags;
        if ((*flags & RigExecPointFrameValid) == 0) {
            return false;
        }
        const auto enumerate = [&](const RigExecPoseFrameVisitor &visit) {
            for (const RigExecBakedCommit::AncestorRead &a :
                 commit.worldUpAncestors) {
                if (a.slot < 0 || size_t(a.slot) >= _program->paths.size() ||
                    size_t(a.base) >= _program->base.size() ||
                    size_t(a.fin) >= _program->fin.size()) {
                    continue;
                }
                visit(_program->paths[size_t(a.slot)],
                      _program->base[size_t(a.base)],
                      _program->fin[size_t(a.fin)]);
            }
        };
        const RigExecPoseFrameEnumerator providers(enumerate);
        RigExecPointFrame frame;
        frame.points = *points;
        frame.flags = *flags;
        if (!RigExecApplyRevisedAncestorDelta(
                _program->nativeSources[size_t(native)].path, providers,
                &frame)) {
            return false;
        }
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool constraintGate(size_t object,
                        eval::PoseConstraintGateFacts *facts) const
    {
        const RigExecBakedProgramImpl &P = *_program;
        if (object >= P.walkSteps.size() || object >= P.commits.size() ||
            P.walkSteps[object].solverBatch || P.walkSteps[object].index < 0 ||
            size_t(P.walkSteps[object].index) >= P.constraints.size()) {
            return false;
        }
        const RigExecBakedProgramImpl::Constraint &c =
            P.constraints[size_t(P.walkSteps[object].index)];
        facts->target = c.target;
        facts->deltaBase = c.deltaBase;
        facts->spaceSlot = c.spaceSlot;
        facts->targetRead = P.commits[object].targetRead;
        facts->matrixMover = c.type == SchemaTypeName::RigExecMatrixMover;
        facts->harmonics = c.harmonics;
        facts->singleChainIk = c.singleChainIk;
        facts->ikFkMatch = c.ikFkMatch;
        facts->enabled = RigExecBakedLeafRead(P, c.enabled);
        facts->hasWeightObject = !c.weightObject.IsEmpty();
        facts->pointsTarget = !c.pointsTarget.IsEmpty();
        facts->fieldOk = false;
        facts->fieldCount = 0;
        facts->fieldValue = 0.0;
        if (facts->hasWeightObject && c.weightField >= 0 &&
            size_t(c.weightField) < P.weightFields.size()) {
            const RigExecBakedProgramImpl::WeightField &field =
                P.weightFields[size_t(c.weightField)];
            facts->fieldOk = field.ok;
            facts->fieldCount = field.values.size();
            facts->fieldValue =
                field.values.empty() ? 0.0 : double(field.values[0]);
        }
        facts->defaultWeight = double(RigExecBakedLeafRead(P, c.defaultWeight));
        return true;
    }

    eval::PoseArraysViewT<BakedPoseMath> constraintArrays(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        eval::PoseArraysViewT<BakedPoseMath> view;
        if (c.arrays < 0 ||
            size_t(c.arrays) >= _program->constraintArrays.size()) {
            view.ok = false;
            return view;
        }
        const RigExecBakedProgramImpl::ConstraintArrays &arrays =
            _program->constraintArrays[size_t(c.arrays)];
        view.ok = arrays.ok;
        view.weights = arrays.weights.empty() ? nullptr : arrays.weights.data();
        view.weightCount = arrays.weights.size();
        view.translationOffsets = arrays.translationOffsets.empty()
                                      ? nullptr
                                      : arrays.translationOffsets.data();
        view.rotationOffsets = arrays.rotationOffsets.empty()
                                   ? nullptr
                                   : arrays.rotationOffsets.data();
        view.offsetCount = std::min(arrays.translationOffsets.size(),
                                    arrays.rotationOffsets.size());
        view.diagnostics =
            arrays.diagnostics.empty() ? nullptr : arrays.diagnostics.data();
        view.diagnosticCount = arrays.diagnostics.size();
        view.poleOk = arrays.poleOk;
        view.poleWeights =
            arrays.poleWeights.empty() ? nullptr : arrays.poleWeights.data();
        view.poleWeightCount = arrays.poleWeights.size();
        view.poleDiagnostics = arrays.poleDiagnostics.empty()
                                   ? nullptr
                                   : arrays.poleDiagnostics.data();
        view.poleDiagnosticCount = arrays.poleDiagnostics.size();
        return view;
    }

    // Flag \p i of constraint commit \p object's mover: affect x, y, z,
    // then the translation, rotation and scale axes.
    bool constraintFlag(size_t object, size_t i) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedProgramImpl &P = *_program;
        switch (i) {
        case 0:
            return RigExecBakedLeafRead(P, c.affectX);
        case 1:
            return RigExecBakedLeafRead(P, c.affectY);
        case 2:
            return RigExecBakedLeafRead(P, c.affectZ);
        case 3:
            return RigExecBakedLeafRead(P, c.tX);
        case 4:
            return RigExecBakedLeafRead(P, c.tY);
        case 5:
            return RigExecBakedLeafRead(P, c.tZ);
        case 6:
            return RigExecBakedLeafRead(P, c.rX);
        case 7:
            return RigExecBakedLeafRead(P, c.rY);
        case 8:
            return RigExecBakedLeafRead(P, c.rZ);
        case 9:
            return RigExecBakedLeafRead(P, c.sX);
        case 10:
            return RigExecBakedLeafRead(P, c.sY);
        case 11:
            return RigExecBakedLeafRead(P, c.sZ);
        default:
            return false;
        }
    }

    GfVec3d constraintVector(size_t object, eval::PoseConstraintVector v) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        using V = eval::PoseConstraintVector;
        switch (v) {
        case V::Offset:
            return RigExecBakedLeafRead(*_program, c.offset);
        case V::AimVector:
            return RigExecBakedLeafRead(*_program, c.aimVector);
        case V::UpVector:
            return RigExecBakedLeafRead(*_program, c.upVector);
        case V::RotationOffset:
            return RigExecBakedLeafRead(*_program, c.rotationOffset);
        case V::WorldUpVector:
            return RigExecBakedLeafRead(*_program, c.worldUpVector);
        case V::SceneUp:
            break;
        }
        return c.sceneUp;
    }

    bool constraintSourceRead(size_t object, size_t k, int *slot,
                              uint32_t *finRead) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        const RigExecBakedCommit &commit = _program->commits[object];
        if (k >= c.sources.size() || k >= commit.sourceReads.size()) {
            return false;
        }
        *slot = c.sources[k];
        *finRead = commit.sourceReads[k];
        return true;
    }

    bool worldUpObjectNamed(size_t object) const
    {
        return _program->constraints[size_t(_program->walkSteps[object].index)]
            .worldUpObjectNamed;
    }

    bool worldUpRotationOnly(size_t object) const
    {
        return _program->constraints[size_t(_program->walkSteps[object].index)]
            .worldUpRotationOnly;
    }

    bool deltaBaseValid(int deltaBase) const
    {
        if (deltaBase < 0) {
            return false;
        }
        const size_t b = size_t(deltaBase);
        return b < _program->deltaValues.size() &&
               b < _program->deltaPresent.size() &&
               b < _program->deltaBaseMatrix.size() &&
               b < _program->deltaBaseOk.size();
    }

    void deltaBaseMatrix(int deltaBase, GfMatrix4d *out) const
    {
        *out = _program->deltaBaseMatrix[size_t(deltaBase)];
    }

    bool deltaBaseOk(int deltaBase) const
    {
        return _program->deltaBaseOk[size_t(deltaBase)] != 0;
    }

    std::string deltaBasePath(size_t object) const
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        return c.deltaBasePathText;
    }

    bool pathValid(int slot) const
    {
        return slot >= 0 && size_t(slot) < _program->pathTexts->size();
    }

    bool slotCompose(size_t slot,
                     eval::PoseSlotComposeT<BakedPoseMath> *view) const
    {
        if (slot * 11 + 11 > _program->avars.size() ||
            slot >= _program->noScaleAvars.size() ||
            slot >= _program->rotOrder.size() ||
            slot >= _program->posedD.size() ||
            slot >= _program->parentSpaceM.size() ||
            slot >= _program->parentSpaceAuthored.size() ||
            slot >= _program->parentDinv.size() ||
            slot >= _program->posedAuthored.size() ||
            slot >= _program->posedAuthoredM.size()) {
            return false;
        }
        view->avars11 = &_program->avars[slot * 11];
        const TfToken &order = _program->rotOrder[slot];
        view->rotOrder.assign(order.GetText(), order.size());
        view->rotationSign = slot < _program->rotationSign.size()
                                 ? _program->rotationSign[slot]
                                 : 0u;
        view->noScaleAvars = _program->noScaleAvars[slot] != 0;
        view->posedAuthored = _program->posedAuthored[slot] != 0;
        view->posedAuthoredM = &_program->posedAuthoredM[slot];
        view->posedD = &_program->posedD[slot];
        view->parentDinv = &_program->parentDinv[slot];
        view->parentSpaceM = &_program->parentSpaceM[slot];
        view->parentSpaceAuthored = _program->parentSpaceAuthored[slot] != 0;
        return true;
    }

private:
    GfMatrix4d _SpaceMatrix(const RigExecBakedProgramImpl::Ladder &L,
                            size_t channel, const GfMatrix4d &fallback) const
    {
        const int id = L.spaceValues[channel];
        const auto *value =
            id >= 0
                ? _program->providerValues.Read<GfMatrix4d>(RigExecValueId(id))
                : nullptr;
        return value ? *value : fallback;
    }

    const RigExecBakedProgramImpl *_program;
};

class BakedPoseState {
public:
    BakedPoseState(RigExecBakedProgramImpl *program, RigExecBakedStep *step)
        : _program(program), _step(step)
    {
    }

    void diagnose(const std::string &message)
    {
        _step->diagnostics.push_back(message);
    }

    void writeAvar(size_t slot, double value)
    {
        _program->avars[slot] = value;
    }

    std::vector<RigExecSpaceCheckpointInput> &checkpointInputs(size_t object)
    {
        return _program->switchFrameContexts[object].kernelInputs;
    }

    void writeSwitchFrame(size_t object, const GfMatrix4d &matrix)
    {
        _program->switchFrames[object] = matrix;
    }

    void beforeSlot(eval::StepKind kind, int begin, int i)
    {
        if (_program->execCheckRows) {
            _program->execCheckRows->BeforeSlot(
                *_program, static_cast<RigExecBakedStepKind>(kind), begin, i);
        }
    }

    void writeRest(size_t slot, const GfMatrix4d &restM,
                   const std::array<GfVec3d, 4> &points, unsigned flags,
                   const GfMatrix4d &roundTrip)
    {
        _program->restM[slot] = restM;
        _program->restFrames[slot] = BakedPoseMath::MakeFrame(points, flags);
        _program->restPts[slot] = points;
        _program->restRoundTrip[slot] = roundTrip;
    }

    void writeLadder(size_t slot, const GfMatrix4d &selfD,
                     const GfMatrix4d &parentDinv,
                     const GfMatrix4d &defaultRoundTrip,
                     const GfMatrix4d &posedD, const GfMatrix4d &parentSpaceM,
                     const GfMatrix4d &posedAuthoredM, bool posedAuthored,
                     bool parentSpaceAuthored, unsigned char rotationSign)
    {
        _program->selfD[slot] = selfD;
        _program->parentDinv[slot] = parentDinv;
        _program->defaultRoundTrip[slot] = defaultRoundTrip;
        _program->posedD[slot] = posedD;
        _program->parentSpaceM[slot] = parentSpaceM;
        _program->posedAuthoredM[slot] = posedAuthoredM;
        _program->posedAuthored[slot] = posedAuthored ? 1 : 0;
        _program->parentSpaceAuthored[slot] = parentSpaceAuthored ? 1 : 0;
        _program->rotationSign[slot] = rotationSign;
    }

    void writeLadderUnavailable(size_t slot)
    {
        GfMatrix4d unavailable(1.0);
        unavailable[3][0] = std::numeric_limits<double>::quiet_NaN();
        _program->selfD[slot] = unavailable;
        _program->parentDinv[slot] = unavailable;
        _program->restRoundTrip[slot] = unavailable;
        _program->defaultRoundTrip[slot] = unavailable;
        _program->posedAuthoredM[slot] = unavailable;
        _program->posedD[slot] = unavailable;
        _program->parentSpaceM[slot] = unavailable;
        _program->posedAuthored[slot] = 0;
        _program->parentSpaceAuthored[slot] = 0;
        _program->rotOrder[slot] = _program->xyzToken;
        _program->rotationSign[slot] = 0;
    }

    void writeRotOrder(size_t slot)
    {
        const TfToken order = RigExecBakedLeafRead(
            *_program, _program->ladders[slot].rotationOrder);
        _program->rotOrder[slot] = order.IsEmpty() ? _program->xyzToken : order;
    }

    eval::PoseLadderLastsT<BakedPoseMath> ladderLasts(size_t slot)
    {
        eval::PoseLadderLastsT<BakedPoseMath> lasts;
        lasts.restM = &_program->lastRestM[slot];
        lasts.selfD = &_program->lastSelfD[slot];
        lasts.parentDinv = &_program->lastParentDinv[slot];
        lasts.posedAuthoredM = &_program->lastPosedAuthoredM[slot];
        lasts.posedD = &_program->lastPosedD[slot];
        lasts.parentSpaceM = &_program->lastParentSpaceM[slot];
        lasts.posedAuthored = &_program->lastPosedAuthored[slot];
        lasts.parentSpaceAuthored = &_program->lastParentSpaceAuthored[slot];
        lasts.rotationSign = &_program->lastRotationSign[slot];
        return lasts;
    }

    bool rotOrderMoved(size_t slot)
    {
        return _program->rotOrder[slot] != _program->lastRotOrder[slot];
    }

    void commitRotOrder(size_t slot)
    {
        _program->lastRotOrder[slot] = _program->rotOrder[slot];
    }

    void markRestChanged(size_t slot)
    {
        if (!_program->restChanged[slot]) {
            _program->restChanged[slot] = 1;
        }
    }

    void markLadderChanged(size_t slot)
    {
        if (!_program->ladderChanged[slot]) {
            _program->ladderChanged[slot] = 1;
        }
    }

    void writeBase(size_t slot, const std::array<GfVec3d, 4> &points,
                   unsigned flags)
    {
        _program->base[slot] = BakedPoseMath::MakeFrame(points, flags);
    }

    void writeFin(size_t slot, const std::array<GfVec3d, 4> &points,
                  unsigned flags)
    {
        _program->fin[slot] = BakedPoseMath::MakeFrame(points, flags);
    }

    void writePosedM(size_t slot, const GfMatrix4d &matrix)
    {
        _program->posedM[slot] = matrix;
    }

    eval::PoseCommitState commitState(size_t object)
    {
        RigExecBakedCommit &commit = _program->commits[object];
        eval::PoseCommitState state;
        state.present = commit.present.data();
        state.presentCount = commit.present.size();
        state.deltaOk = commit.deltaOk.data();
        state.deltaOkCount = commit.deltaOk.size();
        state.outcome = commit.outcome.data();
        state.outcomeCount = commit.outcome.size();
        state.abandoned = &commit.abandoned;
        state.framesCount = commit.frames.size();
        state.deltasCount = commit.deltas.size();
        state.stagedCount = commit.staged.size();
        state.sourcesCount = commit.sources.size();
        state.recordAfter = commit.recordAfter;
        state.recordEveryTarget = commit.recordEveryTarget;
        state.recordGatesValid = true;
        return state;
    }

    void commitFrame(size_t object, size_t pos, std::array<GfVec3d, 4> *points,
                     unsigned *flags)
    {
        const RigExecPointFrame &frame = _program->commits[object].frames[pos];
        *points = frame.points;
        *flags = frame.flags;
    }

    void writeCommitDelta(size_t object, size_t pos, const GfMatrix4d &delta)
    {
        _program->commits[object].deltas[pos] = delta;
    }

    void commitDelta(size_t object, size_t pos, GfMatrix4d *delta)
    {
        *delta = _program->commits[object].deltas[pos];
    }

    void commitStagedFrame(size_t object, size_t k,
                           std::array<GfVec3d, 4> *points, unsigned *flags)
    {
        const RigExecPointFrame &frame = _program->commits[object].staged[k];
        *points = frame.points;
        *flags = frame.flags;
    }

    void writeCommitStaged(size_t object, size_t k,
                           const std::array<GfVec3d, 4> &points, unsigned flags)
    {
        _program->commits[object].staged[k] =
            BakedPoseMath::MakeFrame(points, flags);
    }

    bool writeFinVersion(uint32_t version, const std::array<GfVec3d, 4> &points,
                         unsigned flags)
    {
        if (size_t(version) >= _program->fin.size()) {
            return false;
        }
        _program->fin[size_t(version)] =
            BakedPoseMath::MakeFrame(points, flags);
        return true;
    }

    void setMatchValuesValid(size_t object, bool value)
    {
        RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        c.matchValuesValid = value;
    }

    void setMatchShownMask(size_t object, unsigned mask)
    {
        RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        c.matchShownMask = mask;
    }

    void writeMatchShown(size_t object, size_t k,
                         const std::array<GfVec3d, 4> &points, unsigned flags)
    {
        RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        c.matchShown[k] = BakedPoseMath::MakeFrame(points, flags);
        c.matchShownMask |= 1u << k;
    }

    bool matchValueOut(size_t object, double **ik, double **fk)
    {
        RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        *ik = c.matchIkValues.data();
        *fk = c.matchFkValues.data();
        return true;
    }

    bool writeBaseVersion(uint32_t version,
                          const std::array<GfVec3d, 4> &points, unsigned flags)
    {
        if (size_t(version) >= _program->base.size()) {
            return false;
        }
        _program->base[size_t(version)] =
            BakedPoseMath::MakeFrame(points, flags);
        return true;
    }

    void setRecordEveryTarget(size_t object, bool value)
    {
        _program->commits[object].recordEveryTarget = value;
    }

    void recordConstraintEnvelope(size_t object)
    {
        RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        c.weightScratch.clear();
        c.weightError.clear();
        if (c.weightField >= 0 &&
            size_t(c.weightField) < _program->weightFields.size()) {
            const RigExecBakedProgramImpl::WeightField &field =
                _program->weightFields[size_t(c.weightField)];
            c.weightScratch = field.values;
            c.weightError = field.error;
        }
    }

    void writeCommitFrame(size_t object, size_t pos,
                          const std::array<GfVec3d, 4> &points, unsigned flags)
    {
        _program->commits[object].frames[pos] =
            BakedPoseMath::MakeFrame(points, flags);
    }

    bool sampleConstraintArrays(size_t object)
    {
        const RigExecBakedProgramImpl::Constraint &c =
            _program->constraints[size_t(_program->walkSteps[object].index)];
        if (c.arrays < 0 ||
            size_t(c.arrays) >= _program->constraintArrays.size()) {
            return false;
        }
        PrepareConstraintArrays(&_program->constraintArrays[size_t(c.arrays)]);
        return true;
    }

    void setRecordAfter(size_t object, bool value)
    {
        _program->commits[object].recordAfter = value;
    }

    void writeConstraintSource(size_t object, size_t k,
                               const std::array<GfVec3d, 4> &points,
                               unsigned flags, double weight,
                               const GfVec3d &translationOffset,
                               const GfVec3d &rotationOffsetDegrees)
    {
        RigExecConstraintSource &source = _program->commits[object].sources[k];
        source.frame = BakedPoseMath::MakeFrame(points, flags);
        source.normalizedWeight = weight;
        source.translationOffset = translationOffset;
        source.rotationOffsetDegrees = rotationOffsetDegrees;
    }

    bool writeDeltaIdentity(size_t deltaBase)
    {
        if (deltaBase >= _program->deltaValues.size() ||
            deltaBase >= _program->deltaPresent.size()) {
            return false;
        }
        _program->deltaValues[deltaBase] = GfMatrix4d(1.0);
        _program->deltaPresent[deltaBase] = 1;
        return true;
    }

    void writeDeltaMeasured(size_t deltaBase, const GfMatrix4d &delta)
    {
        _program->deltaValues[deltaBase] = delta;
        _program->deltaPresent[deltaBase] = 1;
    }

    void writeDeltaAbsent(size_t deltaBase)
    {
        _program->deltaPresent[deltaBase] = 0;
    }

    void observeCandidate(size_t object, const std::array<GfVec3d, 4> &points,
                          unsigned flags)
    {
        if (_program->execCheckRows) {
            _program->execCheckRows->ObserveConstraintCandidate(
                int(object), BakedPoseMath::MakeFrame(points, flags));
        }
    }

private:
    RigExecBakedProgramImpl *_program;
    RigExecBakedStep *_step;
};

class BakedPoseHost {
public:
    using Math = BakedPoseMath;
    using Inputs = BakedPoseInputs;
    using State = BakedPoseState;
    using EvalValueFamily = eval::PoseValueFamily;

    BakedPoseHost(RigExecBakedProgramImpl *program, RigExecBakedStep *step)
        : _inputs(program), _state(program, step), _program(program)
    {
    }

    const Inputs &inputs() const
    {
        return _inputs;
    }
    State &state()
    {
        return _state;
    }
    const RigExecRangeRunner *ranges() const
    {
        return nullptr;
    }

private:
    Inputs _inputs;
    State _state;
    RigExecBakedProgramImpl *_program;
};

} // namespace rigExec

#endif // RIGEXEC_POSE_EVAL_HOST_H
