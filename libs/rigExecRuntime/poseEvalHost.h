// The runtime's EvalCore pose Host: Rr storage and math behind the
// pose.h binding. Internal to RigExecRuntime, never an exported API.
#ifndef RIGEXEC_RUNTIME_POSE_EVAL_HOST_H
#define RIGEXEC_RUNTIME_POSE_EVAL_HOST_H

#include "poseInternal.h"
#include "poseMath.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/rangeRunner.h"

#include <algorithm>

namespace rigExec {

// RigExecApplyRevisedAncestorDelta (rigEvaluator.cpp): the native
// source rides the delta of the deepest revised ancestor above it.
inline bool
RrApplyRevisedAncestorDelta(
    const RrProgram *program, const std::string &xformPath,
    const std::vector<RigExecWireAncestorRead> &ancestors,
    std::array<RrVec3d, 4> *points, unsigned *flags)
{
    const RrStore &store = program->store;
    const auto elementCount = [](const std::string &path) {
        return size_t(std::count(path.begin(), path.end(), '/'));
    };
    std::string closest;
    std::array<RrVec3d, 4> closestBase = *points;
    std::array<RrVec3d, 4> closestCurrent = *points;
    for (const RigExecWireAncestorRead &a : ancestors) {
        if (a.slot < 0 || size_t(a.slot) >= program->slotMeta->paths.size() ||
            size_t(a.base) >= store.base.size() ||
            size_t(a.fin) >= store.fin.size()) {
            return false;
        }
        const std::string provider =
            program->TextOrEmpty(program->slotMeta->paths[size_t(a.slot)]);
        const RrPointFrame &base = store.base[size_t(a.base)];
        const RrPointFrame &current = store.fin[size_t(a.fin)];
        if (provider == xformPath || xformPath.size() <= provider.size() ||
            xformPath.compare(0, provider.size(), provider) != 0 ||
            xformPath[provider.size()] != '/' ||
            current.points == base.points) {
            continue;
        }
        if (closest.empty() || elementCount(provider) > elementCount(closest)) {
            closest = provider;
            closestBase = base.points;
            closestCurrent = current.points;
        }
    }
    if (!closest.empty()) {
        RrMat4d delta;
        delta.SetIdentity();
        if (!RrPointsToMatrix(closestBase, closestCurrent, &delta)) {
            return false;
        }
        const RrPointFrame moved = RrMatrixToPoints(*points, delta);
        *points = moved.points;
        *flags = moved.flags;
    }
    return (*flags & RrPointFrameValid) != 0;
}

using RrPoseMath = RrPoseMathT<RrPointFrame>;

// Borrowed views over the program's decoded records and scratch. All
// borrowed storage outlives the step call.
class RrPoseInputs {
public:
    explicit RrPoseInputs(const RrProgram *program) : _program(program)
    {
    }

    size_t avarWriteCount(size_t object) const
    {
        const RrInputState &state = _program->inputState;
        if (object + 1 >= state.avarReadBegin.size()) {
            return 0;
        }
        return size_t(state.avarReadBegin[object + 1] -
                      state.avarReadBegin[object]);
    }

    int32_t avarWriteSlot(size_t object, size_t k) const
    {
        const RrInputState &state = _program->inputState;
        const uint32_t read =
            state.avarReads[size_t(state.avarReadBegin[object]) + k];
        return _program->registeredReads[read].avar;
    }

    double avarWriteValue(size_t object, size_t k) const
    {
        const RrInputState &state = _program->inputState;
        const uint32_t read =
            state.avarReads[size_t(state.avarReadBegin[object]) + k];
        return _program->ReadRegistered(int32_t(read)).f64;
    }

    size_t checkpointObjectCount() const
    {
        return _program->poses->spaceCheckpoints.size();
    }

    size_t checkpointRecomposeCount(size_t object) const
    {
        return _program->poses->spaceCheckpoints[object].recompose.size();
    }

    bool checkpointSlot(size_t object, size_t k,
                        eval::PoseSlotComposeT<RrPoseMath> *view) const
    {
        const auto &recompose =
            _program->poses->spaceCheckpoints[object].recompose;
        if (k >= recompose.size() || recompose[k] < 0) {
            return false;
        }
        return slotCompose(size_t(recompose[k]), view);
    }

    int checkpointAnchor(size_t object) const
    {
        return _program->poses->spaceCheckpoints[object].anchor;
    }

    bool posedMatrix(size_t slot, RrMat4d *out) const
    {
        if (slot >= _program->store.posedM.size()) {
            return false;
        }
        *out = _program->store.posedM[slot];
        return true;
    }

    size_t composeGroupCount() const
    {
        return _program->poses->composeGroups.size();
    }

    void composeGroupRange(size_t object, int *begin, int *end) const
    {
        const auto &group = _program->poses->composeGroups[object];
        *begin = group.begin;
        *end = group.end;
    }

    bool isPoseSlot(size_t slot) const
    {
        const auto &meta = *_program->slotMeta;
        return slot < meta.slotKind.size() &&
               meta.slotKind[slot] == RigExecWireSlotKind::FirstFramePose;
    }

    bool composeSlotBounds(size_t slot) const
    {
        const auto &meta = *_program->slotMeta;
        const RrStore &store = _program->store;
        const RrPoseScratch *scratch =
            runtimePoseDetail::_RrScratch(const_cast<RrProgram *>(_program));
        return slot < meta.slotKind.size() && slot < store.base.size() &&
               slot < store.fin.size() && slot < store.posedM.size() &&
               slot < scratch->restM.size();
    }

    bool providerActive(size_t slot) const
    {
        const auto &meta = *_program->slotMeta;
        return meta.providerActive.empty() ||
               (slot < meta.providerActive.size() && meta.providerActive[slot]);
    }

    int parent(size_t slot) const
    {
        const auto &meta = *_program->slotMeta;
        return slot < meta.parent.size() ? meta.parent[slot] : -1;
    }

    double restAvar(size_t slot, size_t k) const
    {
        return _program->ReadLadder(slot, RrLadderRestAvar0 + int(k)).f64;
    }

    double defaultAvar(size_t slot, size_t k) const
    {
        return _program->ReadLadder(slot, RrLadderDefaultAvar0 + int(k)).f64;
    }

    RrMat4d ladderSpace(size_t slot, int channel) const
    {
        return _program->ReadLadder(slot, _LadderField(channel)).matrix;
    }

    RrMat4d interveningSpace(size_t slot) const
    {
        return _program->ReadLadder(slot, RrLadderInterveningSpace).matrix;
    }

    RrMat4d ladderParentSpaceLeaf(size_t slot) const
    {
        return _program
            ->ReadRegistered(
                _program->ladderRead[slot][size_t(RrLadderParentSpace)])
            .matrix;
    }

    bool ladderChannelReady(size_t slot, int channel) const
    {
        const auto &ladder = _program->poses->ladders[slot];
        if (channel < 0 || size_t(channel) >= ladder.spaceValues.size()) {
            return false;
        }
        const int id = ladder.spaceValues[size_t(channel)];
        if (id < 0 || size_t(id) >= _program->store.providerValues.size()) {
            return false;
        }
        const auto &value = _program->store.providerValues[size_t(id)];
        return value.initialized && !value.blocked &&
               std::holds_alternative<std::array<double, 16>>(value.value);
    }

    bool ladderAuthoritative(size_t slot, int channel) const
    {
        const auto &ladder = _program->poses->ladders[slot];
        const RigExecWireInput *input = nullptr;
        bool connected = false;
        switch (channel) {
        case eval::kPoseLadderDefaultSpace:
            input = ladder.defaultSpace.get();
            connected = ladder.defaultSpaceConnected;
            break;
        case eval::kPoseLadderPosedSpace:
            input = ladder.posedSpace.get();
            connected = ladder.posedSpaceConnected;
            break;
        case eval::kPoseLadderParentSpace:
            input = ladder.parentSpace.get();
            connected = ladder.parentSpaceConnected;
            break;
        case eval::kPoseLadderParentDefaultSpace:
            input = ladder.parentDefaultSpace.get();
            connected = ladder.parentDefaultSpaceConnected;
            break;
        case eval::kPoseLadderAvarDefaultSpace:
            input = ladder.avarDefaultSpace.get();
            connected = ladder.avarDefaultSpaceConnected;
            break;
        case eval::kPoseLadderPosedDefaultSpace:
            input = ladder.posedDefaultSpace.get();
            connected = ladder.posedDefaultSpaceConnected;
            break;
        default:
            return false;
        }
        if (!input) {
            return connected;
        }
        const int index = input->overrideIndex;
        return connected ||
               (index >= 0 &&
                size_t(index) < _program->inputState.valueOverridden.size() &&
                _program->inputState.valueOverridden[size_t(index)]);
    }

    RrVec3d ladderRotationSign(size_t slot) const
    {
        return _program->ReadLadder(slot, RrLadderRotationSign).vec;
    }

    bool restRoundTrip(size_t slot, RrMat4d *out) const
    {
        const auto &tables = _RrScratch()->restRoundTrip;
        if (slot >= tables.size()) {
            return false;
        }
        *out = tables[slot];
        return true;
    }

    bool defaultRoundTrip(size_t slot, RrMat4d *out) const
    {
        const auto &tables = _RrScratch()->defaultRoundTrip;
        if (slot >= tables.size()) {
            return false;
        }
        *out = tables[slot];
        return true;
    }

    void baseFrame(size_t slot, std::array<RrVec3d, 4> *points,
                   unsigned *flags) const
    {
        const RrPointFrame &frame = _program->store.base[slot];
        *points = frame.points;
        *flags = frame.flags;
    }

    size_t commitObjectCount() const
    {
        return std::min(_program->poses->commits.size(),
                        _program->store.commits.size());
    }

    eval::PoseCommitFacts commitFacts(size_t object) const
    {
        const RigExecWireCommit &commit = _program->poses->commits[object];
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
        facts.moverPath = _program->TextOrEmpty(commit.moverPath);
        return facts;
    }

    size_t commitSlotCount(size_t object) const
    {
        return _program->poses->commits[object].slots.size();
    }

    size_t commitPropagateCount(size_t object) const
    {
        return _program->poses->commits[object].propagate.size();
    }

    bool commitPropagateAt(size_t object, size_t k, int *first,
                           int *second) const
    {
        const auto &propagate = _program->poses->commits[object].propagate;
        if (k >= propagate.size()) {
            return false;
        }
        *first = propagate[k].first;
        *second = propagate[k].second;
        return true;
    }

    bool commitClosestPosAt(size_t object, size_t k, int *pos) const
    {
        const auto &closestPos = _program->poses->commits[object].closestPos;
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
        const auto &meta = *_program->slotMeta;
        const RrPoseScratch *scratch = _RrScratch();
        if (i >= meta.propParent.size() ||
            i >= scratch->parentSpaceAuthored.size()) {
            return false;
        }
        *authored = scratch->parentSpaceAuthored[i] != 0;
        *next = meta.propParent[i];
        return true;
    }

    std::string commitPathText(size_t slot) const
    {
        const auto &meta = *_program->slotMeta;
        if (slot >= meta.paths.size()) {
            return std::string();
        }
        return _program->TextOrEmpty(meta.paths[slot]);
    }

    bool finFrame(uint32_t version, std::array<RrVec3d, 4> *points,
                  unsigned *flags) const
    {
        if (size_t(version) >= _program->store.fin.size()) {
            return false;
        }
        const RrPointFrame &frame = _program->store.fin[size_t(version)];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool restFrame(size_t slot, std::array<RrVec3d, 4> *points,
                   unsigned *flags) const
    {
        const RrPoseScratch *scratch = _RrScratch();
        if (slot >= scratch->restFrames.size()) {
            return false;
        }
        const RrPointFrame &frame = scratch->restFrames[slot];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool ikFkFacts(size_t object, eval::PoseIkFkFacts *facts) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        if (c.matchConstants.size() != 4) {
            return false;
        }
        facts->envelope =
            double(_program->ReadConstraint(ci, RrConstraintDefaultWeight).f32);
        for (size_t k = 0; k < 7; ++k) {
            const RrInputValue value =
                _program->ReadConstraint(ci, RrConstraintMatchFirst + int(k));
            facts->scalars[k] =
                value.tag == RrInputTag::Float ? double(value.f32) : value.f64;
        }
        facts->ikValue = c.matchConstants[0];
        facts->scaleCalibration = c.matchConstants[1];
        facts->upperOffset = c.matchConstants[2];
        facts->lowerOffset = c.matchConstants[3];
        facts->stretchLimb = c.matchStretchLimb;
        return true;
    }

    bool matchChannels(size_t slot, RigExecIkFkMatchChannels *out) const
    {
        const RrPoseScratch *scratch = _RrScratch();
        if (slot * 11 + 11 > _program->store.avars.size() ||
            slot >= scratch->noScaleAvars.size() ||
            slot >= scratch->rotOrder.size()) {
            return false;
        }
        *out = runtimePoseDetail::_RrMatchChannelsOf(*_program, *scratch, slot);
        return true;
    }

    bool ikChainFacts(size_t object, eval::PoseIkChainFacts *facts) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        facts->targetSlots.assign(c.targetSlots.begin(), c.targetSlots.end());
        facts->effector = c.effector;
        facts->effectorNative = c.effectorNative;
        facts->stretch =
            c.stretch
                ? double(_program->ReadConstraint(ci, RrConstraintStretch).f32)
                : 0.0;
        facts->ikMode = int(c.ikMode);
        facts->useAnimatedTs = c.useAnimatedTs;
        facts->ikRestLive.assign(c.ikRestLive.begin(), c.ikRestLive.end());
        facts->poleModeObject = c.poleModeObject;
        facts->poleObjects.assign(c.poleObjects.begin(), c.poleObjects.end());
        return true;
    }

    void ikPoleVector(size_t object, RrVec3d *pole, double *twistDegrees) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        *pole = _program->ReadConstraint(ci, RrConstraintPoleVector).vec;
        *twistDegrees =
            _program->ReadConstraint(ci, RrConstraintTwistDegrees).f64;
    }

    bool resolveIkEffector(size_t object, std::array<RrVec3d, 4> *points,
                           unsigned *flags) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        if (c.effector < 0) {
            const int native = c.effectorNative;
            if (native < 0 ||
                size_t(native) >= _program->store.nativeFrameOk.size() ||
                size_t(native) >= _program->store.nativeFrames.size() ||
                size_t(native) >= _program->poses->nativeSources.size() ||
                !_program->store.nativeFrameOk[size_t(native)]) {
                return false;
            }
            *points = _program->store.nativeFrames[size_t(native)].points;
            *flags = _program->store.nativeFrames[size_t(native)].flags;
            if ((*flags & RrPointFrameValid) == 0) {
                return false;
            }
            return RrApplyRevisedAncestorDelta(
                _program,
                _program->TextOrEmpty(
                    _program->poses->nativeSources[size_t(native)].path),
                wireCommit.effectorAncestors, points, flags);
        }
        if (size_t(wireCommit.effectorRead) >= _program->store.fin.size()) {
            return false;
        }
        *points = _program->store.fin[size_t(wireCommit.effectorRead)].points;
        *flags = _program->store.fin[size_t(wireCommit.effectorRead)].flags;
        return (*flags & RrPointFrameValid) != 0;
    }

    bool ikPoleBounds(size_t object, size_t k) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        return k < c.poleObjects.size() && k < wireCommit.poleReads.size() &&
               k < wireCommit.poleAncestors.size() &&
               k < c.poleObjectNatives.size();
    }

    bool resolveIkPole(size_t object, size_t k, std::array<RrVec3d, 4> *points,
                       unsigned *flags) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        if (k >= c.poleObjects.size() || k >= wireCommit.poleReads.size() ||
            k >= wireCommit.poleAncestors.size() ||
            k >= c.poleObjectNatives.size()) {
            return false;
        }
        if (c.poleObjects[k] < 0) {
            const int native = c.poleObjectNatives[k];
            if (native < 0 ||
                size_t(native) >= _program->store.nativeFrameOk.size() ||
                size_t(native) >= _program->store.nativeFrames.size() ||
                size_t(native) >= _program->poses->nativeSources.size() ||
                !_program->store.nativeFrameOk[size_t(native)]) {
                return false;
            }
            *points = _program->store.nativeFrames[size_t(native)].points;
            *flags = _program->store.nativeFrames[size_t(native)].flags;
            if ((*flags & RrPointFrameValid) == 0) {
                return false;
            }
            return RrApplyRevisedAncestorDelta(
                _program,
                _program->TextOrEmpty(
                    _program->poses->nativeSources[size_t(native)].path),
                wireCommit.poleAncestors[k].v, points, flags);
        }
        if (size_t(wireCommit.poleReads[k]) >= _program->store.fin.size()) {
            return false;
        }
        *points = _program->store.fin[size_t(wireCommit.poleReads[k])].points;
        *flags = _program->store.fin[size_t(wireCommit.poleReads[k])].flags;
        return (*flags & RrPointFrameValid) != 0;
    }

    bool commitSlotAt(size_t object, size_t pos, int *slot) const
    {
        if (pos >= _program->poses->commits[object].slots.size()) {
            return false;
        }
        *slot = _program->poses->commits[object].slots[pos];
        return true;
    }

    eval::PoseHarmonicsFacts<RrPoseMath> harmonicsFacts(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        eval::PoseHarmonicsFacts<RrPoseMath> facts;
        facts.target = c.target;
        facts.spaceSlot = c.spaceSlot;
        facts.window = c.window;
        facts.pathText = _program->TextOrEmpty(c.path);
        facts.enabled =
            _program->ReadConstraint(ci, RrConstraintEnabled).boolean;
        facts.hasWeightObject = c.weightObject != 0;
        if (facts.hasWeightObject) {
            const RrWeightFieldResult &field =
                _program->store.weightFieldResults[size_t(c.weightField)];
            facts.fieldOk = field.ok;
            facts.fieldCount = field.values.size();
            facts.fieldValue =
                field.values.empty() ? 0.0 : double(field.values[0]);
            facts.fieldError = field.error;
        }
        facts.defaultWeight =
            double(_program->ReadConstraint(ci, RrConstraintDefaultWeight).f32);
        for (int k = 0; k < 3; ++k) {
            const RrInputValue value =
                _program->ReadConstraint(ci, RrConstraintHarmonicsFirst + k);
            facts.scalars[size_t(k)] =
                value.tag == RrInputTag::Float ? double(value.f32) : value.f64;
        }
        facts.axis = _program->ReadConstraint(ci, RrConstraintOffset).vec;
        facts.matchVelocity = c.harmonicsMatchVelocity;
        facts.settingsValid = c.harmonicsConstants.size() == 3;
        if (facts.settingsValid) {
            facts.settings.waves = c.harmonicsConstants[0];
            facts.settings.waveLength = c.harmonicsConstants[1];
            facts.settings.frequency = c.harmonicsConstants[2];
            facts.settings.ignoreFirstFrame = c.harmonicsIgnoreFirstFrame;
        }
        return facts;
    }

    bool windowCaptureValid(int window) const
    {
        return window >= 0 &&
               size_t(window) < _program->store.windowCaptures.size();
    }

    bool windowedOpValid(int window) const
    {
        return window >= 0 && size_t(window) < _program->windowedOps.size();
    }

    size_t windowRingCount(int window) const
    {
        if (window < 0 ||
            size_t(window) >= _program->store.windowRings.size()) {
            return 0;
        }
        return _program->store.windowRings[size_t(window)].size();
    }

    RigExecWindowPacketPtr windowRingPacket(int window, size_t k) const
    {
        if (window < 0 ||
            size_t(window) >= _program->store.windowRings.size()) {
            return RigExecWindowPacketPtr();
        }
        const RigExecWindowRing &ring =
            _program->store.windowRings[size_t(window)];
        return k < ring.size() ? ring[k] : RigExecWindowPacketPtr();
    }

    bool windowFeedsWindow(int window) const
    {
        return _program->windowedOps[size_t(window)].feedsWindow;
    }

    bool historyTick() const
    {
        return _program->store.historyTick;
    }

    eval::PoseConstraintFactsT<RrPoseMath> constraintFacts(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        eval::PoseConstraintFactsT<RrPoseMath> facts;
        facts.target = c.target;
        facts.deltaBase = c.deltaBase;
        facts.spaceSlot = c.spaceSlot;
        facts.pathText = _program->TextOrEmpty(c.path);
        facts.enabled =
            _program->ReadConstraint(ci, RrConstraintEnabled).boolean;
        facts.hasWeightObject = c.weightObject != 0;
        facts.pointsTarget = c.pointsTarget != 0;
        if (facts.hasWeightObject && c.weightField >= 0 &&
            size_t(c.weightField) < _program->store.weightFieldResults.size()) {
            const RrWeightFieldResult &field =
                _program->store.weightFieldResults[size_t(c.weightField)];
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
            double(_program->ReadConstraint(ci, RrConstraintDefaultWeight).f32);
        const std::string ctype = _program->TextOrEmpty(c.type);
        if (ctype == SchemaTypeName::RigExecMatrixMover) {
            facts.code = eval::kPoseConstraintMatrixMover;
        } else if (ctype == SchemaTypeName::RigExecPositionConstraint) {
            facts.code = eval::kPoseConstraintPosition;
        } else if (ctype == SchemaTypeName::RigExecRotationConstraint) {
            facts.code = eval::kPoseConstraintRotation;
        } else if (ctype == SchemaTypeName::RigExecScaleConstraint) {
            facts.code = eval::kPoseConstraintScale;
        } else if (ctype == SchemaTypeName::RigExecParentConstraint) {
            facts.code = eval::kPoseConstraintParent;
        } else if (ctype == SchemaTypeName::RigExecAimConstraint) {
            facts.code = eval::kPoseConstraintAim;
        }
        facts.ikFkMatch = c.ikFkMatch;
        facts.harmonics = c.harmonics;
        facts.radialBlend = (c.flags & RigExecWireConstraintRadialBlend) != 0;
        facts.singleChainIk = c.singleChainIk;
        facts.arrays = constraintArrays(object);
        if (c.arrays < 0 || size_t(c.arrays) >= _program->store.arrays.size()) {
            return facts;
        }
        facts.affectX =
            _program->ReadConstraint(ci, RrConstraintAffectX).boolean;
        facts.affectY =
            _program->ReadConstraint(ci, RrConstraintAffectY).boolean;
        facts.affectZ =
            _program->ReadConstraint(ci, RrConstraintAffectZ).boolean;
        facts.order = eval::pose_constraints::PoseEulerOrder(c.order);
        facts.offset = _program->ReadConstraint(ci, RrConstraintOffset).vec;
        facts.blendShear = (c.flags & RigExecWireConstraintBlendShear) != 0;
        facts.translationAxes.x =
            _program->ReadConstraint(ci, RrConstraintTX).boolean;
        facts.translationAxes.y =
            _program->ReadConstraint(ci, RrConstraintTY).boolean;
        facts.translationAxes.z =
            _program->ReadConstraint(ci, RrConstraintTZ).boolean;
        facts.rotationAxes.x =
            _program->ReadConstraint(ci, RrConstraintRX).boolean;
        facts.rotationAxes.y =
            _program->ReadConstraint(ci, RrConstraintRY).boolean;
        facts.rotationAxes.z =
            _program->ReadConstraint(ci, RrConstraintRZ).boolean;
        facts.scaleAxes.x =
            _program->ReadConstraint(ci, RrConstraintSX).boolean;
        facts.scaleAxes.y =
            _program->ReadConstraint(ci, RrConstraintSY).boolean;
        facts.scaleAxes.z =
            _program->ReadConstraint(ci, RrConstraintSZ).boolean;
        return facts;
    }

    size_t constraintSourceCount(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        return _program->poses->constraints[ci].sources.size();
    }

    bool constraintSourceRef(size_t object, size_t k,
                             eval::PoseSourceRef *ref) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        if (k >= c.sources.size() || k >= wireCommit.sourceReads.size() ||
            k >= c.sourceNatives.size() || k >= c.sourcePaths.size()) {
            return false;
        }
        ref->slot = c.sources[k];
        ref->finRead = wireCommit.sourceReads[k];
        ref->native = c.sourceNatives[k];
        ref->pathText = _program->TextOrEmpty(c.sourcePaths[k]);
        return true;
    }

    bool resolveNativeSource(size_t object, size_t k,
                             std::array<RrVec3d, 4> *points,
                             unsigned *flags) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        const int native = c.sourceNatives[k];
        if (native < 0 ||
            size_t(native) >= _program->store.nativeFrameOk.size() ||
            size_t(native) >= _program->store.nativeFrames.size() ||
            size_t(native) >= _program->poses->nativeSources.size() ||
            !_program->store.nativeFrameOk[size_t(native)]) {
            return false;
        }
        if (k >= wireCommit.sourceAncestors.size()) {
            return false;
        }
        *points = _program->store.nativeFrames[size_t(native)].points;
        *flags = _program->store.nativeFrames[size_t(native)].flags;
        if ((*flags & RrPointFrameValid) == 0) {
            return false;
        }
        return RrApplyRevisedAncestorDelta(
            _program,
            _program->TextOrEmpty(
                _program->poses->nativeSources[size_t(native)].path),
            wireCommit.sourceAncestors[k].v, points, flags);
    }

    bool resolveWorldUp(size_t object, std::array<RrVec3d, 4> *points,
                        unsigned *flags) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &wireCommit = _program->poses->commits[object];
        if (c.worldUpObject >= 0) {
            if (!finFrame(wireCommit.worldUpRead, points, flags)) {
                return false;
            }
            return (*flags & RrPointFrameValid) != 0;
        }
        const int native = c.worldUpNative;
        if (native < 0 ||
            size_t(native) >= _program->store.nativeFrameOk.size() ||
            !_program->store.nativeFrameOk[size_t(native)]) {
            return false;
        }
        if (size_t(native) >= _program->store.nativeFrames.size()) {
            return false;
        }
        *points = _program->store.nativeFrames[size_t(native)].points;
        *flags = _program->store.nativeFrames[size_t(native)].flags;
        return RrApplyRevisedAncestorDelta(
            _program,
            _program->TextOrEmpty(
                _program->poses->nativeSources[size_t(native)].path),
            wireCommit.worldUpAncestors, points, flags);
    }

    bool constraintGate(size_t object,
                        eval::PoseConstraintGateFacts *facts) const
    {
        const auto &poses = *_program->poses;
        if (object >= poses.walkSteps.size() ||
            object >= poses.commits.size() ||
            poses.walkSteps[object].solverBatch ||
            poses.walkSteps[object].index < 0 ||
            size_t(poses.walkSteps[object].index) >= poses.constraints.size()) {
            return false;
        }
        const size_t ci = size_t(poses.walkSteps[object].index);
        const RigExecWireConstraint &c = poses.constraints[ci];
        facts->target = c.target;
        facts->deltaBase = c.deltaBase;
        facts->spaceSlot = c.spaceSlot;
        facts->targetRead = poses.commits[object].targetRead;
        facts->matrixMover =
            _program->TextEquals(c.type, SchemaTypeName::RigExecMatrixMover);
        facts->enabled =
            _program->ReadConstraint(ci, RrConstraintEnabled).boolean;
        facts->hasWeightObject = c.weightObject != 0;
        facts->pointsTarget = c.pointsTarget != 0;
        facts->fieldOk = false;
        facts->fieldCount = 0;
        facts->fieldValue = 0.0;
        if (facts->hasWeightObject && c.weightField >= 0 &&
            size_t(c.weightField) < _program->store.weightFieldResults.size()) {
            const RrWeightFieldResult &field =
                _program->store.weightFieldResults[size_t(c.weightField)];
            facts->fieldOk = field.ok;
            facts->fieldCount = field.values.size();
            facts->fieldValue =
                field.values.empty() ? 0.0 : double(field.values[0]);
        }
        facts->defaultWeight =
            double(_program->ReadConstraint(ci, RrConstraintDefaultWeight).f32);
        return true;
    }

    eval::PoseArraysViewT<RrPoseMath> constraintArrays(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        eval::PoseArraysViewT<RrPoseMath> view;
        if (c.arrays < 0 || size_t(c.arrays) >= _program->store.arrays.size()) {
            view.ok = false;
            return view;
        }
        const RrConstraintArraysLive &arrays =
            _program->store.arrays[size_t(c.arrays)];
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
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        return i < 12 &&
               _program->ReadConstraint(ci, RrConstraintAffectX + int(i))
                   .boolean;
    }

    RrVec3d constraintVector(size_t object, eval::PoseConstraintVector v) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        using V = eval::PoseConstraintVector;
        switch (v) {
        case V::Offset:
            return _program->ReadConstraint(ci, RrConstraintOffset).vec;
        case V::AimVector:
            // Files written before Build bound the aimAxis fallback into
            // aimVector choose it here.
            return c.aimVectorAuthored
                       ? _program->ReadConstraint(ci, RrConstraintAimVector).vec
                       : RrVec3d(c.aimAxisFallback[0], c.aimAxisFallback[1],
                                 c.aimAxisFallback[2]);
        case V::UpVector:
            return _program->ReadConstraint(ci, RrConstraintUpVector).vec;
        case V::RotationOffset:
            return _program->ReadConstraint(ci, RrConstraintRotationOffset).vec;
        case V::WorldUpVector:
            return _program->ReadConstraint(ci, RrConstraintWorldUpVector).vec;
        case V::SceneUp:
            break;
        }
        return RrVec3d(c.sceneUp[0], c.sceneUp[1], c.sceneUp[2]);
    }

    bool constraintSourceRead(size_t object, size_t k, int *slot,
                              uint32_t *finRead) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        const RigExecWireCommit &commit = _program->poses->commits[object];
        if (k >= c.sources.size() || k >= commit.sourceReads.size()) {
            return false;
        }
        *slot = c.sources[k];
        *finRead = commit.sourceReads[k];
        return true;
    }

    bool worldUpObjectNamed(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        return _program->poses->constraints[ci].worldUpObjectNamed;
    }

    bool worldUpRotationOnly(size_t object) const
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        return (_program->poses->constraints[ci].flags &
                RigExecWireConstraintWorldUpRotationOnly) != 0;
    }

    bool deltaBaseValid(int deltaBase) const
    {
        if (deltaBase < 0) {
            return false;
        }
        const size_t b = size_t(deltaBase);
        const RrPoseScratch *scratch = _RrScratch();
        return b < scratch->deltaValues.size() &&
               b < _program->store.deltaValues.size() &&
               b < scratch->deltaPresent.size() &&
               b < _program->store.deltaPresent.size() &&
               b < _program->store.deltaBaseMatrix.size() &&
               b < _program->store.deltaBaseOk.size() && _program->geometry &&
               b < _program->geometry->deltaBasePaths.size();
    }

    void deltaBaseMatrix(int deltaBase, RrMat4d *out) const
    {
        *out = _program->store.deltaBaseMatrix[size_t(deltaBase)];
    }

    bool deltaBaseOk(int deltaBase) const
    {
        return _program->store.deltaBaseOk[size_t(deltaBase)];
    }

    std::string deltaBasePath(size_t object) const
    {
        const RigExecWireConstraint &c =
            _program->poses
                ->constraints[size_t(_program->poses->walkSteps[object].index)];
        const int deltaBase = c.deltaBase;
        if (deltaBase < 0 || !_program->geometry ||
            size_t(deltaBase) >= _program->geometry->deltaBasePaths.size()) {
            return std::string();
        }
        return _program->TextOrEmpty(
            _program->geometry->deltaBasePaths[size_t(deltaBase)]);
    }

    bool pathValid(int slot) const
    {
        return slot >= 0 && size_t(slot) < _program->slotMeta->paths.size();
    }

    bool baseFrameAt(uint32_t version, std::array<RrVec3d, 4> *points,
                     unsigned *flags) const
    {
        if (size_t(version) >= _program->store.base.size()) {
            return false;
        }
        const RrPointFrame &frame = _program->store.base[size_t(version)];
        *points = frame.points;
        *flags = frame.flags;
        return true;
    }

    bool slotCompose(size_t slot,
                     eval::PoseSlotComposeT<RrPoseMath> *view) const
    {
        const RrStore &store = _program->store;
        const RrPoseScratch *scratch = _RrScratch();
        if (slot * 11 + 11 > store.avars.size() ||
            slot >= scratch->noScaleAvars.size() ||
            slot >= scratch->rotOrder.size() ||
            slot >= scratch->posedD.size() ||
            slot >= scratch->parentSpaceM.size() ||
            slot >= scratch->parentSpaceAuthored.size() ||
            slot >= scratch->parentDinv.size() ||
            slot >= scratch->posedAuthored.size() ||
            slot >= scratch->posedAuthoredM.size()) {
            return false;
        }
        view->avars11 = &store.avars[slot * 11];
        view->rotOrder = _program->TextOrEmpty(scratch->rotOrder[slot]);
        view->rotationSign = slot < scratch->rotationSign.size()
                                 ? scratch->rotationSign[slot]
                                 : 0u;
        view->noScaleAvars = scratch->noScaleAvars[slot] != 0;
        view->posedAuthored = scratch->posedAuthored[slot] != 0;
        view->posedAuthoredM = &scratch->posedAuthoredM[slot];
        view->posedD = &scratch->posedD[slot];
        view->parentDinv = &scratch->parentDinv[slot];
        view->parentSpaceM = &scratch->parentSpaceM[slot];
        view->parentSpaceAuthored = scratch->parentSpaceAuthored[slot] != 0;
        return true;
    }

private:
    static int _LadderField(int channel)
    {
        switch (channel) {
        case eval::kPoseLadderRestSpace:
            return RrLadderRestSpace;
        case eval::kPoseLadderDefaultSpace:
            return RrLadderDefaultSpace;
        case eval::kPoseLadderPosedSpace:
            return RrLadderPosedSpace;
        case eval::kPoseLadderParentSpace:
            return RrLadderParentSpace;
        case eval::kPoseLadderParentDefaultSpace:
            return RrLadderParentDefaultSpace;
        case eval::kPoseLadderAvarDefaultSpace:
            return RrLadderAvarDefaultSpace;
        default:
            return RrLadderPosedDefaultSpace;
        }
    }

    const RrPoseScratch *_RrScratch() const
    {
        return runtimePoseDetail::_RrScratch(const_cast<RrProgram *>(_program));
    }

    const RrProgram *_program;
};

class RrPoseState {
public:
    explicit RrPoseState(RrProgram *program) : _program(program)
    {
    }

    void writeAvar(int32_t slot, double value)
    {
        _program->store.avars[size_t(slot)] = value;
    }

    std::vector<RigExecSpaceCheckpointInputT<RrMat4d>> &
    checkpointInputs(size_t object)
    {
        return runtimePoseDetail::_RrScratch(_program)
            ->checkpointInputs[object];
    }

    void writeSwitchFrame(size_t object, const RrMat4d &matrix)
    {
        _program->store.switchFrames[object] = matrix;
    }

    void beforeSlot(eval::StepKind kind, int begin, int i)
    {
        (void)kind;
        (void)begin;
        (void)i;
    }

    void writeRest(size_t slot, const RrMat4d &restM,
                   const std::array<RrVec3d, 4> &points, unsigned flags,
                   const RrMat4d &roundTrip)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        scratch->restM[slot] = restM;
        scratch->restFrames[slot] = RrPoseMath::MakeFrame(points, flags);
        scratch->restPts[slot] = points;
        scratch->restRoundTrip[slot] = roundTrip;
    }

    void writeLadder(size_t slot, const RrMat4d &selfD,
                     const RrMat4d &parentDinv, const RrMat4d &defaultRoundTrip,
                     const RrMat4d &posedD, const RrMat4d &parentSpaceM,
                     const RrMat4d &posedAuthoredM, bool posedAuthored,
                     bool parentSpaceAuthored, unsigned char rotationSign)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        scratch->selfD[slot] = selfD;
        scratch->parentDinv[slot] = parentDinv;
        scratch->defaultRoundTrip[slot] = defaultRoundTrip;
        scratch->posedD[slot] = posedD;
        scratch->parentSpaceM[slot] = parentSpaceM;
        scratch->posedAuthoredM[slot] = posedAuthoredM;
        scratch->posedAuthored[slot] = posedAuthored ? 1 : 0;
        scratch->parentSpaceAuthored[slot] = parentSpaceAuthored ? 1 : 0;
        scratch->rotationSign[slot] = rotationSign;
    }

    void writeLadderUnavailable(size_t slot)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        RrMat4d unavailable(1);
        unavailable[3][0] = std::numeric_limits<double>::quiet_NaN();
        scratch->restRoundTrip[slot] = unavailable;
        scratch->defaultRoundTrip[slot] = unavailable;
        scratch->selfD[slot] = unavailable;
        scratch->parentDinv[slot] = unavailable;
        scratch->posedD[slot] = unavailable;
        scratch->posedAuthoredM[slot] = unavailable;
        scratch->parentSpaceM[slot] = unavailable;
        scratch->posedAuthored[slot] = 0;
        scratch->parentSpaceAuthored[slot] = 0;
        scratch->rotationSign[slot] = 0;
        scratch->rotOrder[slot] = 0;
    }

    void writeRotOrder(size_t slot)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        scratch->rotOrder[slot] =
            _program->ReadLadder(slot, RrLadderRotationOrder).token;
    }

    eval::PoseLadderLastsT<RrPoseMath> ladderLasts(size_t slot)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        eval::PoseLadderLastsT<RrPoseMath> lasts;
        lasts.restM = &scratch->lastRestM[slot];
        lasts.selfD = &scratch->lastSelfD[slot];
        lasts.parentDinv = &scratch->lastParentDinv[slot];
        lasts.posedAuthoredM = &scratch->lastPosedAuthoredM[slot];
        lasts.posedD = &scratch->lastPosedD[slot];
        lasts.parentSpaceM = &scratch->lastParentSpaceM[slot];
        lasts.posedAuthored = &scratch->lastPosedAuthored[slot];
        lasts.parentSpaceAuthored = &scratch->lastParentSpaceAuthored[slot];
        lasts.rotationSign = &scratch->lastRotationSign[slot];
        return lasts;
    }

    bool rotOrderMoved(size_t slot)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        return scratch->rotOrder[slot] != scratch->lastRotOrder[slot];
    }

    void commitRotOrder(size_t slot)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        scratch->lastRotOrder[slot] = scratch->rotOrder[slot];
    }

    void markRestChanged(size_t slot)
    {
        _program->store.restChanged[slot] = 1;
        _movedAny = true;
    }

    void markLadderChanged(size_t slot)
    {
        _program->store.ladderChanged[slot] = 1;
        _movedAny = true;
    }

    bool movedAny() const
    {
        return _movedAny;
    }

    void writeBase(size_t slot, const std::array<RrVec3d, 4> &points,
                   unsigned flags)
    {
        _program->store.base[slot] = RrPoseMath::MakeFrame(points, flags);
    }

    void writeFin(size_t slot, const std::array<RrVec3d, 4> &points,
                  unsigned flags)
    {
        _program->store.fin[slot] = RrPoseMath::MakeFrame(points, flags);
    }

    void writePosedM(size_t slot, const RrMat4d &matrix)
    {
        _program->store.posedM[slot] = matrix;
    }

    eval::PoseCommitState commitState(size_t object)
    {
        RrCommitScratch &commit = _program->store.commits[object];
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
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        if (object < scratch->recordAfter.size() &&
            object < scratch->recordEveryTarget.size()) {
            state.recordAfter = scratch->recordAfter[object] != 0;
            state.recordEveryTarget = scratch->recordEveryTarget[object] != 0;
            state.recordGatesValid = true;
        }
        return state;
    }

    void commitFrame(size_t object, size_t pos, std::array<RrVec3d, 4> *points,
                     unsigned *flags)
    {
        const RrPointFrame &frame = _program->store.commits[object].frames[pos];
        *points = frame.points;
        *flags = frame.flags;
    }

    void writeCommitDelta(size_t object, size_t pos, const RrMat4d &delta)
    {
        _program->store.commits[object].deltas[pos] = delta;
    }

    void commitDelta(size_t object, size_t pos, RrMat4d *delta)
    {
        *delta = _program->store.commits[object].deltas[pos];
    }

    void commitStagedFrame(size_t object, size_t k,
                           std::array<RrVec3d, 4> *points, unsigned *flags)
    {
        const RrPointFrame &frame = _program->store.commits[object].staged[k];
        *points = frame.points;
        *flags = frame.flags;
    }

    void writeCommitStaged(size_t object, size_t k,
                           const std::array<RrVec3d, 4> &points, unsigned flags)
    {
        _program->store.commits[object].staged[k] =
            RrPoseMath::MakeFrame(points, flags);
    }

    bool writeFinVersion(uint32_t version, const std::array<RrVec3d, 4> &points,
                         unsigned flags)
    {
        if (size_t(version) >= _program->store.fin.size()) {
            return false;
        }
        _program->store.fin[size_t(version)] =
            RrPoseMath::MakeFrame(points, flags);
        return true;
    }

    void setMatchValuesValid(size_t object, bool value)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        _program->store.matchValid[ci] = value ? 1 : 0;
    }

    void setMatchShownMask(size_t object, unsigned mask)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        _program->store.matchShownMask[ci] = uint8_t(mask);
    }

    void writeMatchShown(size_t object, size_t k,
                         const std::array<RrVec3d, 4> &points, unsigned flags)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        _program->store.matchShown[ci][k] =
            RrPoseMath::MakeFrame(points, flags);
        _program->store.matchShownMask[ci] |= uint8_t(1u << k);
    }

    bool matchValueOut(size_t object, double **ik, double **fk)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        if (ci >= _program->store.matchValues.size() ||
            _program->store.matchValues[ci].size() < 31 ||
            ci >= _program->store.matchShown.size()) {
            return false;
        }
        *ik = _program->store.matchValues[ci].data();
        *fk = _program->store.matchValues[ci].data() + 11;
        return true;
    }

    void matchPoleParent(size_t object, const RrMat4d &moved, RrMat4d *parent)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        const int poleSlot = c.sources.size() > 6 ? c.sources[6] : -1;
        if (c.matchPoleSwitch >= 0 && poleSlot >= 0 &&
            size_t(poleSlot) < scratch->posedAuthored.size() &&
            !scratch->posedAuthored[size_t(poleSlot)]) {
            RrMat4d switched;
            if (runtimePoseDetail::_RrMatchPoleParent(
                    _program, c.matchPoleSwitch, c.matchPoleSourceRides, moved,
                    &switched)) {
                *parent = switched;
            }
            return;
        }
        if (c.matchPoleParentRides) {
            *parent = eval::pose_ops::Multiply4<RrPoseMath>(*parent, moved);
        }
    }

    // Defined in poseConstraints.cpp: the single-chain IK core lives
    // there until the Solve lift unifies it, and this TU-local call
    // cannot inline into every includer.
    bool solveIkChain(size_t object, const std::vector<RrPointFrame> &chain,
                      const std::vector<RrPointFrame> &rests,
                      const RrPointFrame &effector, int mode, double stretch,
                      double weight, const RrVec3d &pole, double twistDegrees,
                      bool useAnimatedTs, std::vector<RrPointFrame> *solved);

    bool writeBaseVersion(uint32_t version,
                          const std::array<RrVec3d, 4> &points, unsigned flags)
    {
        if (size_t(version) >= _program->store.base.size()) {
            return false;
        }
        _program->store.base[size_t(version)] =
            RrPoseMath::MakeFrame(points, flags);
        return true;
    }

    void diagnose(const std::string &message)
    {
        if (_step < _program->store.stepOutputs.size()) {
            _program->store.stepOutputs[_step].diagnostics.push_back(message);
        }
    }

    void setStep(size_t step)
    {
        _step = step;
    }

    void setRecordEveryTarget(size_t object, bool value)
    {
        runtimePoseDetail::_RrScratch(_program)->recordEveryTarget[object] =
            value ? 1 : 0;
    }

    RigExecWindowPacketPtr &windowCapture(int window)
    {
        return _program->store.windowCaptures[size_t(window)];
    }

    void recordConstraintEnvelope(size_t object)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        std::vector<float> &envelope = scratch->weightScratch[ci];
        std::string &envelopeError = scratch->weightError[ci];
        envelope.clear();
        envelopeError.clear();
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        if (c.weightField >= 0 &&
            size_t(c.weightField) < _program->store.weightFieldResults.size()) {
            const RrWeightFieldResult &field =
                _program->store.weightFieldResults[size_t(c.weightField)];
            envelope = field.values;
            envelopeError = field.error;
        }
    }

    void writeCommitFrame(size_t object, size_t pos,
                          const std::array<RrVec3d, 4> &points, unsigned flags)
    {
        _program->store.commits[object].frames[pos] =
            RrPoseMath::MakeFrame(points, flags);
    }

    bool sampleConstraintArrays(size_t object)
    {
        const size_t ci = size_t(_program->poses->walkSteps[object].index);
        const RigExecWireConstraint &c = _program->poses->constraints[ci];
        RrStore &store = _program->store;
        if (size_t(c.arrays) >= store.arrays.size() ||
            size_t(c.arrays) >= _program->poses->constraintArrays.size()) {
            return false;
        }
        RrConstraintArraysLive &arrays = store.arrays[size_t(c.arrays)];
        const auto &arrayWire =
            _program->poses->constraintArrays[size_t(c.arrays)];
        const auto arrayPath = _program->TextOrEmpty(arrayWire.prim);
        arrays.diagnostics.clear();
        arrays.poleDiagnostics.clear();
        const auto weights = [&](size_t channel, size_t count) {
            const int slot = arrayWire.rawSlots[channel];
            const auto *raw =
                slot >= 0 && RrInputHasValue(_program, uint32_t(slot))
                    ? RrInputArray<float>(_program, uint32_t(slot))
                    : nullptr;
            const char *name = channel == 0
                                   ? SchemaAttrText::inputsSourceWeights
                                   : SchemaAttrText::inputsPoleVectorWeights;
            auto &lines =
                channel == 0 ? arrays.diagnostics : arrays.poleDiagnostics;
            auto &values = channel == 0 ? arrays.weights : arrays.poleWeights;
            return eval::NormalizeConstraintArray<double>(
                channel, count, raw ? raw->size() : 0, raw != nullptr,
                [&](size_t k) { return double((*raw)[k]); },
                [&](size_t size, size_t expected) {
                    lines.push_back(arrayPath + " " + name + " has " +
                                    std::to_string(size) + " entries for " +
                                    std::to_string(expected) + " sources");
                },
                &values);
        };
        const auto offsets = [&](size_t channel, size_t count, bool enabled) {
            const int slot = arrayWire.rawSlots[channel];
            const auto *raw =
                enabled && slot >= 0 &&
                        RrInputHasValue(_program, uint32_t(slot))
                    ? RrInputArray<RigExecWireVec3d>(_program, uint32_t(slot))
                    : nullptr;
            const char *name = channel == 1
                                   ? SchemaAttrText::inputsTranslationOffsets
                                   : SchemaAttrText::inputsRotationOffsets;
            auto &values = channel == 1 ? arrays.translationOffsets
                                        : arrays.rotationOffsets;
            return eval::NormalizeConstraintArray<RrVec3d>(
                channel, count, raw ? raw->size() : 0, raw != nullptr,
                [&](size_t k) {
                    return RrVec3d((*raw)[k][0], (*raw)[k][1], (*raw)[k][2]);
                },
                [&](size_t size, size_t expected) {
                    arrays.diagnostics.push_back(
                        arrayPath + " " + name + " has " +
                        std::to_string(size) + " entries for " +
                        std::to_string(expected) + " sources");
                },
                &values);
        };
        eval::NormalizeConstraintTables(
            size_t(arrayWire.sourceCount), size_t(arrayWire.poleCount),
            arrayWire.parentOffsets, arrayWire.readPole, weights, offsets,
            &arrays.ok, &arrays.poleOk);
        return true;
    }

    void setRecordAfter(size_t object, bool value)
    {
        runtimePoseDetail::_RrScratch(_program)->recordAfter[object] =
            value ? 1 : 0;
    }

    void writeConstraintSource(size_t object, size_t k,
                               const std::array<RrVec3d, 4> &points,
                               unsigned flags, double weight,
                               const RrVec3d &translationOffset,
                               const RrVec3d &rotationOffsetDegrees)
    {
        RrConstraintSource &source = _program->store.commits[object].sources[k];
        source.frame = RrPoseMath::MakeFrame(points, flags);
        source.normalizedWeight = weight;
        source.translationOffset = translationOffset;
        source.rotationOffsetDegrees = rotationOffsetDegrees;
    }

    bool writeDeltaIdentity(size_t deltaBase)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        if (deltaBase >= scratch->deltaValues.size() ||
            deltaBase >= _program->store.deltaValues.size() ||
            deltaBase >= scratch->deltaPresent.size() ||
            deltaBase >= _program->store.deltaPresent.size()) {
            return false;
        }
        RrMat4d identity;
        identity.SetIdentity();
        scratch->deltaValues[deltaBase] = identity;
        _program->store.deltaValues[deltaBase] = identity;
        scratch->deltaPresent[deltaBase] = 1;
        _program->store.deltaPresent[deltaBase] = 1;
        return true;
    }

    void writeDeltaMeasured(size_t deltaBase, const RrMat4d &delta)
    {
        RrPoseScratch *scratch = runtimePoseDetail::_RrScratch(_program);
        scratch->deltaValues[deltaBase] = delta;
        _program->store.deltaValues[deltaBase] = delta;
        scratch->deltaPresent[deltaBase] = 1;
        _program->store.deltaPresent[deltaBase] = 1;
    }

    void writeDeltaAbsent(size_t deltaBase)
    {
        runtimePoseDetail::_RrScratch(_program)->deltaPresent[deltaBase] = 0;
        _program->store.deltaPresent[deltaBase] = 0;
    }

    void observeCandidate(size_t object, const std::array<RrVec3d, 4> &points,
                          unsigned flags)
    {
        (void)object;
        (void)points;
        (void)flags;
    }

private:
    RrProgram *_program;
    size_t _step = size_t(-1);
    bool _movedAny = false;
};

class RrPoseHost {
public:
    using Math = RrPoseMath;
    using Inputs = RrPoseInputs;
    using State = RrPoseState;
    using EvalValueFamily = eval::PoseValueFamily;

    explicit RrPoseHost(RrProgram *program, size_t step = size_t(-1))
        : _inputs(program), _state(program), _program(program)
    {
        _state.setStep(step);
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
    RrProgram *_program;
};

} // namespace rigExec

#endif // RIGEXEC_RUNTIME_POSE_EVAL_HOST_H
