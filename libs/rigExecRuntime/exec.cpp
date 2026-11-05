// Runtime driver: sample sources, execute the common graph, then publish outputs.
#include "rigExecRuntime/runtime.h"
#include "stageArrayInputs.h"

#include "rigExecRuntime/labels.h"
#include "poseInternal.h"

#include <algorithm>

namespace rigExec {

namespace {

template <typename T>
void
_SortByPath(std::vector<T> *out)
{
    std::sort(out->begin(), out->end(),
              [](const T &a, const T &b) { return a.path < b.path; });
}

// A membership change's new rows, built before any output changes.
template<class T>
struct _StagedRows {
    bool changed=false;
    std::vector<T> rows;
    std::vector<size_t> positions;
};

// \p describe fills what a row keeps for its key, beyond its path.
template<class Order,class T,class Enumerate,class Describe>
void _StageRows(Order *order,const std::vector<T> &rows,const RrProgram &program,
                Enumerate enumerate,Describe describe,_StagedRows<T> *staged)
{
    order->nextKeys.clear();
    enumerate([&](uint32_t id,const auto &) {order->nextKeys.push_back(id);});
    staged->changed=order->keys!=order->nextKeys || rows.size()!=order->nextKeys.size();
    if(!staged->changed)return;
    const size_t count=order->nextKeys.size();
    staged->rows.resize(count);
    std::vector<size_t> sorted(count);
    for(size_t i=0;i<count;++i) {
        sorted[i]=i;
        staged->rows[i].path=program.TextOrEmpty(order->nextKeys[i]);
    }
    std::sort(sorted.begin(),sorted.end(),[&](size_t a,size_t b) {
        return staged->rows[a].path<staged->rows[b].path;
    });
    _SortByPath(&staged->rows);
    staged->positions.resize(count);
    for(size_t i=0;i<count;++i)staged->positions[sorted[i]]=i;
    size_t at=0;
    enumerate([&](uint32_t,const auto &value) {
        describe(staged->rows[staged->positions[at++]],value);
    });
}

template<class Order,class T,class Enumerate,class Assign,class Recycle>
void _CommitRows(Order *order,std::vector<T> *rows,_StagedRows<T> *staged,
                 Enumerate enumerate,Assign assign,Recycle recycle)
{
    if(staged->changed) {
        for(T &row:*rows)recycle(row);
        rows->swap(staged->rows);
        order->keys.swap(order->nextKeys);
        order->positions.swap(staged->positions);
    }
    size_t at=0;
    enumerate([&](uint32_t,const auto &value) {
        assign((*rows)[order->positions[at++]],value);
    });
}

// The pool only spares an allocation: one that cannot grow drops the buffer.
template<class T>
void _Recycle(RigExecSharedArray<T> *array) noexcept
{
    try {
        RrRetainedRecycle(array->Release());
    } catch(...) {
    }
}

}  // namespace

bool
RigExecRuntimeStageArrayInputs::CanSample(const RigExecRuntimeReader &reader,
                                         size_t slot)
{
    const auto &slots = reader._program->inputState.stageArraySlots;
    return std::binary_search(slots.begin(), slots.end(), slot);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::Enumerate(const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &file = *reader._program->inputState.file;
    for (size_t slot : RrStageArraySlots(reader._program.get())) {
        result.push_back({slot, reader._program->TextOrEmpty(file.inputs[slot].name()),
                          RrInputTag(uint8_t(file.inputs[slot].type()))});
    }
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetSample(RigExecRuntimeReader &reader,
    size_t slot, const RigExecRuntimeArray &value, std::string *error)
{
    return RrStageArraySet(reader._program.get(), slot, value, error);
}

bool
RigExecRuntimeStageArrayInputs::ClearSample(RigExecRuntimeReader &reader,
    size_t slot, std::string *error)
{
    return RrStageArrayClear(reader._program.get(), slot, error);
}

bool
RigExecRuntimeStageArrayInputs::CanSampleProviderValue(
    const RigExecRuntimeReader &reader, size_t slot)
{
    const auto &sources=reader._program->inputState.slotProviderSource;
    return slot<sources.size() && sources[slot];
}

bool
RigExecRuntimeStageArrayInputs::SetSampleBlocked(
    RigExecRuntimeReader &reader,size_t slot,bool blocked,std::string *error)
{
    return RrStageInputBlockedSet(reader._program.get(),slot,blocked,error);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::EnumerateProviderValues(
    const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &state=reader._program->inputState;
    for (size_t slot=0;slot<state.slotProviderSource.size();++slot) {
        if (!state.slotProviderSource[slot]) continue;
        const auto &input=state.file->inputs[slot];
        const auto tag=RrInputTag(uint8_t(input.type()));
        const bool sourceBacked = (state.slotProviderSource[slot] & 2) != 0;
        if ((!sourceBacked && !(input.flags() & uint8_t(RigExecWireInputSlotFlags::Animated))) ||
            tag==RrInputTag::Token || RrInputTagIsArray(tag)) continue;
        result.push_back({slot,reader._program->TextOrEmpty(input.name()),tag,
                          (input.flags() & uint8_t(RigExecWireInputSlotFlags::Animated)) != 0});
    }
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetScalarSample(
    RigExecRuntimeReader &reader,size_t slot,const RrInputValue &value,std::string *error)
{
    return RrStageScalarSet(reader._program.get(),slot,value,error);
}

bool
RigExecRuntimeStageArrayInputs::ClearScalarSample(
    RigExecRuntimeReader &reader,size_t slot,std::string *error)
{
    return RrStageScalarClear(reader._program.get(),slot,error);
}

bool
RigExecRuntimeStageArrayInputs::CanSampleToken(
    const RigExecRuntimeReader &reader, size_t slot)
{
    const auto &slots = reader._program->inputState.stageTokenSlots;
    return std::binary_search(slots.begin(), slots.end(), slot);
}

std::vector<RigExecStageArrayInputInfo>
RigExecRuntimeStageArrayInputs::EnumerateTokens(const RigExecRuntimeReader &reader)
{
    std::vector<RigExecStageArrayInputInfo> result;
    const auto &state = reader._program->inputState;
    for (size_t slot : state.stageTokenSlots)
        result.push_back({slot, reader._program->TextOrEmpty(state.file->inputs[slot].name()),
                          RrInputTag::Token});
    return result;
}

bool
RigExecRuntimeStageArrayInputs::SetTokenSample(
    RigExecRuntimeReader &reader, size_t slot, const std::string &text,
    std::string *error)
{
    return RrStageTokenSet(reader._program.get(), slot, text, true, error);
}

bool
RigExecRuntimeStageArrayInputs::ClearTokenSample(
    RigExecRuntimeReader &reader, size_t slot, std::string *error)
{
    return RrStageTokenSet(reader._program.get(), slot, {}, false, error);
}

bool
RigExecRuntimeStageArrayInputs::SetTokenArraySample(
    RigExecRuntimeReader &reader, size_t slot,
    const std::vector<std::string> &texts, std::string *error)
{
    return RrStageTokenArraySet(reader._program.get(), slot, texts, error);
}

size_t
RigExecRuntimeReader::GetInputCount() const
{
    return _program->inputState.inputInfo.size();
}

const RigExecRuntimeInputInfo &
RigExecRuntimeReader::GetInputInfo(size_t index) const
{
    const auto &info = _program->inputState.inputInfo;
    return index < info.size() ? info[index] : _noInput;
}

bool
RigExecRuntimeReader::FindInput(const std::string &name, size_t *index) const
{
    const RrInputState &state = _program->inputState;
    const auto found = state.nameIndex.find(name);
    if (found == state.nameIndex.end() ||
        found->second >= state.inputInfo.size()) {
        return false;
    }
    if (index) {
        *index = found->second;
    }
    return true;
}

const RrInputValue &
RigExecRuntimeReader::GetInputValue(size_t index) const
{
    const auto &values = _program->inputState.inputValues;
    return index < values.size() ? values[index] : _noValue;
}

bool
RigExecRuntimeReader::SetInput(const std::string &name,
                               const RrInputValue &value, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSet(_program.get(), index, value, error);
}

bool
RigExecRuntimeReader::SetInput(const std::string &name, double value,
                               std::string *error)
{
    RrInputValue held = _noValue;
    held.tag = RrInputTag::Double;
    held.f64 = value;
    return SetInput(name, held, error);
}

bool
RigExecRuntimeReader::SetInputToken(const std::string &name,
                                    const std::string &text,
                                    std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSetToken(_program.get(), index, text, error);
}

bool
RigExecRuntimeReader::SetInputAt(size_t index, const RrInputValue &value,
                                 std::string *error)
{
    return RrInputsSet(_program.get(), index, value, error);
}

bool
RigExecRuntimeReader::SetSampledInputAt(size_t index,
                                        const RrInputValue &value,
                                        std::string *error)
{
    return RrInputsSet(_program.get(), index, value, error,
                       /*acceptNonFinite=*/true);
}

bool
RigExecRuntimeReader::SetInputArray(const std::string &name,
                                    const RigExecRuntimeArray &value,
                                    std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/true,
                            error);
}

bool
RigExecRuntimeReader::SetInputArrayAt(size_t index,
                                      const RigExecRuntimeArray &value,
                                      std::string *error)
{
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/true,
                            error);
}

bool
RigExecRuntimeReader::SetSampledInputArrayAt(size_t index,
                                             const RigExecRuntimeArray &value,
                                             std::string *error)
{
    return RrInputsSetArray(_program.get(), index, value, /*authored=*/false,
                            error);
}

bool
RigExecRuntimeReader::GetInputArrayAt(size_t index,
                                      RigExecRuntimeArray *out) const
{
    return RrInputsGetArray(_program.get(), index, out);
}

bool
RigExecRuntimeReader::ClearInput(const std::string &name, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    return RrInputsClear(_program.get(), index, error);
}

bool
RigExecRuntimeReader::ClearInputAt(size_t index, std::string *error)
{
    return RrInputsClear(_program.get(), index, error);
}

bool
RigExecRuntimeReader::ResetInput(const std::string &name, std::string *error)
{
    size_t index = 0;
    if (!FindInput(name, &index)) {
        if (error) {
            *error = "no input named " + name;
        }
        return false;
    }
    RrInputsReset(_program.get(), index);
    return true;
}

void
RigExecRuntimeReader::ResetInputs()
{
    for (size_t i = 0; i < GetInputCount(); ++i) {
        RrInputsReset(_program.get(), i);
    }
}

void
RigExecRuntimeReader::TouchAnimatedInputs()
{
    _program->store.animatedTouched = true;
}

double
RigExecRuntimeReader::GetBakeTime() const
{
    return _program->file ? _program->file->bakeTime : 0.0;
}

std::string
RigExecRuntimeReader::GetTokenText(uint32_t token) const
{
    return _program->TextOrEmpty(token);
}

std::string
RigExecRuntimeReader::GetStepLabelForTesting(size_t step) const
{
    if (!_program->file || step >= _program->file->steps.size())
        return std::to_string(step);
    const auto &key = _program->file->steps[step].descriptorKey;
    // Compile appends this suffix to the exact native display label.
    const auto suffix = key.rfind("/category:");
    return suffix == std::string::npos ? key : key.substr(0, suffix);
}

size_t
RigExecRuntimeReader::GetClosedClusterCountForTesting() const
{
    return _program->store.lastClosedClusters;
}

bool
RigExecRuntimeReader::GetParallelSafeForTesting() const
{
    return _program->parallelSafe;
}

void
RigExecRuntimeReader::SetTaskDispatch(
    std::function<void(std::function<void()>)> dispatch,
    std::function<void()> wait)
{
    RrStore &store = _program->store;
    // Both or neither: a dispatch without its join could return from
    // Execute before the bodies finished.
    if (!dispatch || !wait) {
        store.dispatch = nullptr;
        store.wait = nullptr;
        return;
    }
    store.dispatch = std::move(dispatch);
    store.wait = std::move(wait);
}

size_t
RigExecRuntimeReader::GetSlotLeafKeysForTesting() const
{
    return _program->store.slotLeafKeys;
}

size_t
RigExecRuntimeReader::GetSlotLeafCountForTesting(size_t slot) const
{
    const std::vector<uint32_t> &begin = _program->slotLeafBegin;
    return slot + 1 < begin.size() ? begin[slot + 1] - begin[slot] : 0;
}

size_t
RigExecRuntimeReader::GetSourceKeysBuiltForTesting() const
{
    return _program->store.sourceKeysBuilt;
}

bool
RigExecRuntimeReader::GetSimdEnabledForTesting() const
{
    return _program->geoSettings.useSimd;
}

bool
RigExecRuntimeReader::GetPartitionStaleForTesting(
    const std::string &moverPath) const
{
    return RrGeometryPartitionStaleForTesting(_program.get(), moverPath);
}

bool
RigExecRuntimeReader::GetRevisionDecisionForTesting(
    const std::string &moverPath, int *acceptance, bool *chunksOk) const
{
    return acceptance && chunksOk &&
           RrGeometryRevisionDecisionForTesting(_program.get(), moverPath,
                                                acceptance, chunksOk);
}

bool
RigExecRuntimeReader::GetRangeRoleForTesting(const std::string &moverPath,
                                             bool *rangeRole,
                                             bool *ownSource) const
{
    return RrGeometryRangeRoleForTesting(_program.get(), moverPath, rangeRole,
                                         ownSource);
}

bool
RigExecRuntimeReader::GetSkinLayoutIsOpenForTesting(
    const std::string &moverPath) const
{
    return RrGeometrySkinLayoutIsOpenForTesting(_program.get(), moverPath);
}

std::vector<int32_t>
RigExecRuntimeReader::GetLastRunTraceForTesting() const
{
    return _program->store.runTrace;
}

std::vector<std::string>
RigExecRuntimeReader::GetComposeMovesForTesting(bool ladder) const
{
    const RrStore &store = _program->store;
    const std::vector<char> &changed =
        ladder ? store.ladderChanged : store.restChanged;
    const RigExecWireSlotMeta &meta = *_program->slotMeta;
    std::vector<std::string> out;
    for (size_t slot = 0; slot < changed.size() && slot < meta.paths.size();
         ++slot) {
        if (changed[slot]) {
            out.push_back(_program->TextOrEmpty(meta.paths[slot]));
        }
    }
    return out;
}

bool
RigExecRuntimeReader::GetStepRanForTesting(size_t step) const
{
    const RrStore &store = _program->store;
    if (step >= _program->steps->size() || !store.everRan) {
        return false;
    }
    return step < store.opExecution.ran.size() && store.opExecution.ran[step];
}

std::vector<RigExecRuntimeJointMatrix>
RigExecRuntimeReader::GetJointRestMatrices() const
{
    std::vector<RigExecRuntimeJointMatrix> out;
    const std::vector<RrPointFrame> &restFrames =
        RrPoseRestFrames(_program.get());
    const RigExecWireSlotMeta &meta = *_program->slotMeta;
    for (size_t i = 0; i < meta.jointSlots.size(); ++i) {
        const size_t slot = size_t(meta.jointSlots[i]);
        if (slot >= restFrames.size()) {
            continue;
        }
        RrMat4d rest;
        const RrPointFrame &frame = restFrames[slot];
        if (RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &rest)) {
            out.push_back({_program->TextOrEmpty(meta.jointPaths[i]), rest});
        }
    }
    _SortByPath(&out);
    return out;
}

std::vector<RigExecRuntimeJointMatrix>
RigExecRuntimeReader::GetJointPoseMatrices() const
{
    const auto rests = GetJointRestMatrices();
    std::vector<RigExecRuntimeJointMatrix> out;
    for (const auto &delta : _jointMatrices) {
        const auto rest = std::lower_bound(rests.begin(), rests.end(), delta.path,
            [](const RigExecRuntimeJointMatrix &a, const std::string &path) {
                return a.path < path;
            });
        if (rest != rests.end() && rest->path == delta.path) {
            out.push_back({delta.path, rest->matrix * delta.matrix});
        }
    }
    return out;
}

std::vector<RigExecRuntimePropertyValue>
RigExecRuntimeReader::GetPropertyValues() const
{
    std::vector<RigExecRuntimePropertyValue> out;
    out.reserve(_program->store.propertyResults.size());
    for (const auto &entry : _program->store.propertyResults) {
        out.push_back({_program->TextOrEmpty(entry.first), entry.second});
    }
    _SortByPath(&out);
    return out;
}

std::vector<RigExecRuntimeScalarPrimvar>
RigExecRuntimeReader::GetScalarPrimvars() const
{
    std::vector<RigExecRuntimeScalarPrimvar> out;
    const RigExecWireFile *file = _program->inputState.file;
    if (!file) {
        return out;
    }
    out.reserve(file->scalarPrimvarChains.size());
    for (const uint32_t c : file->scalarPrimvarChains) {
        // Open validated the indices (format.cpp).
        const uint32_t target = file->propertyChains[c].target;
        if (target >= file->inputs.size()) {
            continue;
        }
        const uint32_t name = file->inputs[target].name();
        const auto found = _program->store.propertyResults.find(name);
        if (found == _program->store.propertyResults.end()) {
            continue;
        }
        const RrPropertyValue &value = found->second;
        if (value.tag == RrPropertyValue::Tag::Float) {
            out.push_back({_program->TextOrEmpty(name), value.f32});
        } else if (value.tag == RrPropertyValue::Tag::Double) {
            out.push_back({_program->TextOrEmpty(name), float(value.f64)});
        }
    }
    _SortByPath(&out);
    return out;
}

std::vector<RigExecRuntimeIkFkMatch>
RigExecRuntimeReader::GetIkFkMatches() const
{
    std::vector<RigExecRuntimeIkFkMatch> out;
    const auto &constraints = _program->poses->constraints;
    const RrStore &store = _program->store;
    for (size_t i = 0; i < constraints.size() && i < store.matchValid.size();
         ++i) {
        if (!constraints[i].ikFkMatch || !store.matchValid[i]) continue;
        const auto &values = store.matchValues[i];
        out.push_back({_program->TextOrEmpty(constraints[i].path),
                       std::vector<double>(values.begin(), values.begin() + 11),
                       std::vector<double>(values.begin() + 11, values.end())});
    }
    _SortByPath(&out);
    return out;
}

std::vector<RigExecRuntimeSpaceMatch>
RigExecRuntimeReader::GetSpaceMatches() const
{
    std::vector<RigExecRuntimeSpaceMatch> out;
    const auto &switches = _program->poses->spaceSwitches;
    const RrStore &store = _program->store;
    const RigExecWireSlotMeta &meta = *_program->slotMeta;
    for (size_t i = 0; i < switches.size() && i < store.spaceMatchValid.size();
         ++i) {
        const int slot = switches[i].slot;
        if (!store.spaceMatchValid[i] || slot < 0 ||
            size_t(slot) >= meta.paths.size()) {
            continue;
        }
        out.push_back({_program->TextOrEmpty(meta.paths[size_t(slot)]),
                       store.spaceMatchValues[i]});
    }
    _SortByPath(&out);
    return out;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetFinFrames() const
{
    return _program->store.fin;
}

const std::vector<RrPointFrame> &
RigExecRuntimeReader::GetBaseFrames() const
{
    return _program->store.base;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetFinalMatrices() const
{
    return _program->store.finalMatrix;
}

const std::vector<RrMat4d> &
RigExecRuntimeReader::GetBaseMatrices() const
{
    return _program->store.baseMatrix;
}

const std::vector<RrWeightPacket> &
RigExecRuntimeReader::GetWeightPackets() const
{
    return _program->store.weightPackets;
}

bool
RigExecRuntimeReader::SampleGeometryForTesting(std::string *error)
{
    std::vector<std::string> diagnostics;
    return RrPrologueGeometry(_program.get(), &diagnostics, error);
}

size_t
RigExecRuntimeReader::GetWindowedOperationCount() const
{
    return _program->windowedOps.size();
}

size_t
RigExecRuntimeReader::GetHistoryDepth() const
{
    std::vector<uint32_t> frames;
    std::vector<std::vector<uint32_t>> inner;
    for (const RrProgram::WindowedOp &op : _program->windowedOps) {
        frames.push_back(op.frames);
        inner.push_back(op.inner);
    }
    return RigExecWindowHistoryDepth(frames, inner);
}

void
RigExecRuntimeReader::PushHistory()
{
    RrProgram &program = *_program;
    RrStore &store = program.store;
    for (size_t w = 0; w < program.windowedOps.size() &&
                       w < store.windowRings.size() &&
                       w < store.windowCaptures.size();
         ++w) {
        RigExecWindowRing &ring = store.windowRings[w];
        ring.push_back(store.windowCaptures[w]);
        const size_t keep = program.windowedOps[w].frames;
        if (ring.size() > keep)
            ring.erase(ring.begin(), ring.begin() + std::ptrdiff_t(ring.size() - keep));
        store.windowRingChanged[w] = 1;
    }
}

void
RigExecRuntimeReader::ClearHistory()
{
    RrStore &store = _program->store;
    for (size_t w = 0; w < store.windowRings.size(); ++w) {
        if (!store.windowRings[w].empty()) store.windowRingChanged[w] = 1;
        store.windowRings[w].clear();
    }
}

bool
RigExecRuntimeReader::ExecuteHistory(std::string *error)
{
    RrProgram &program = *_program;
    if (program.windowedOps.empty()) return true;
    if (!_history) {
        if (!_historyBytes) {
            if (error) *error = "the reader kept no bytes for its history";
            return false;
        }
        _history = Open(_historyBytes->data(), _historyBytes->size(), error);
        if (!_history) return false;
        _history->_historyBytes.reset();
    }
    RrProgram &tick = *_history->_program;
    // The kernels installed here; their prepared states are immutable.
    for (size_t k = 0; k < program.externals.size() && k < tick.externals.size(); ++k) {
        tick.externals[k].kernel = program.externals[k].kernel;
        tick.externals[k].state = program.externals[k].state;
        tick.externals[k].version = program.externals[k].version;
    }
    RrInputsCopyCurrent(program, &tick);
    tick.store.windowRings = program.store.windowRings;
    std::fill(tick.store.windowRingChanged.begin(),
              tick.store.windowRingChanged.end(), char(1));
    if (!_history->_RunHistoryTick(error)) return false;
    program.store.windowRings = tick.store.windowRings;
    std::fill(program.store.windowRingChanged.begin(),
              program.store.windowRingChanged.end(), char(1));
    return true;
}

// One history tick on this program, which runs nothing else: the masked
// graph at the inputs as set, its captures pushed as the newest tick.
bool
RigExecRuntimeReader::_RunHistoryTick(std::string *error)
{
    RrProgram &program = *_program;
    RrStore &store = program.store;
    store.runTrace.clear();
    RrInputsApplyTouched(&program);
    std::vector<std::string> diagnostics;
    RrProloguePose(&program);
    store.historyTick = true;
    const bool ran = RrPrologueGeometry(&program, &diagnostics, error) &&
                     RrRunSteps(&program, false, error);
    store.historyTick = false;
    RrShareLatticeBinds(&program);
    if (!ran) return false;
    PushHistory();
    return true;
}

bool
RigExecRuntimeReader::Execute(std::string *error)
{
    RrProgram &program = *_program;
    RrStore &store = program.store;
    store.runTrace.clear();
    // The inputs set since the last run. Only an Animated input set, or
    // TouchAnimatedInputs, dirties what a change of time dirties.
    RrInputsApplyTouched(&program);

    std::vector<std::string> poseDiagnostics;
    RrProloguePose(&program);
    if (!RrPrologueGeometry(&program, &poseDiagnostics, error)) return false;
    const bool ran = RrRunSteps(&program, false, error);
    // Joined: the binds this run built share now, not at the next run.
    RrShareLatticeBinds(&program);
    if (!ran) {
        return false;
    }

    if(!program.requiredStageFramesAdmission.admitted) {
        std::vector<size_t> propertySteps;
        for(size_t i=0;i<program.steps->size();++i)
            if((*program.steps)[i].kind==RigExecWireStepKind::PropertyRevision)propertySteps.push_back(i);
        std::sort(propertySteps.begin(),propertySteps.end(),[&](size_t a,size_t b) {
            const auto &left=(*program.steps)[a]; const auto &right=(*program.steps)[b];
            return std::make_pair(left.object,left.part)<std::make_pair(right.object,right.part);
        });
        for(size_t i:propertySteps) {
            poseDiagnostics.insert(poseDiagnostics.end(),store.headLines[i].begin(),store.headLines[i].end());
        }
        const size_t target=size_t(program.requiredStageFramesAdmission.firstBadTarget);
        const size_t slot=size_t(program.slotMeta->xformSlots[target]);
        const std::string refusal="could not resolve constraint target "+
            program.TextOrEmpty(program.slotMeta->paths[slot])+" relative to the asset root";
        poseDiagnostics.push_back(refusal);
        _jointMatrices.clear(); _points.clear(); _matrixPrimvars.clear();
        _weightFrames.clear(); _weightFields.clear(); _providerXforms.clear();
        _diagnostics=std::move(poseDiagnostics);
        _counters=RigExecRuntimeCounters();
        _counters.executedOpCount=store.opExecution.executed;
        if(error)*error=refusal;
        return false;
    }

    const size_t epilogueMismatches = program.epilogue.mismatches;
    if (!RrPublishPose(&program, &poseDiagnostics, error)) {
        return false;
    }
    RrPublishGeometry(&program, &poseDiagnostics);
    // RIGEXEC_VERIFY_EPILOGUE_LISTS: lines the held steps published
    // differently from a sweep of every step fail the run.
    if (program.epilogue.mismatches != epilogueMismatches) {
        if (error) {
            *error = "the epilogue's held steps and a sweep of every "
                     "step published different lines";
        }
        return false;
    }

    for (const auto &loop : program.opGraph.cycles) {
        std::string message = "operation cycle: ";
        for (size_t i = 0; i < loop.size(); ++i) {
            if (i) message += " -> ";
            message += loop[i];
        }
        if (std::find(poseDiagnostics.begin(), poseDiagnostics.end(), message) == poseDiagnostics.end())
            poseDiagnostics.push_back(std::move(message));
    }

    RigExecRuntimeCounters counters;
    counters.executedOpCount=store.opExecution.executed;

    // A plugin mover this runtime has no kernel for is a no-op, said on
    // every frame it is one, beside the lines of the step it sat in.
    for (const auto &[slot, index] : program.externalIndex) {
        const RrProgram::ExternalRevision &state = program.externals[index];
        if (state.state) {
            continue;
        }
        const RigExecWireRevision &wire =
            program.geometry->chains[slot.first].revisions[slot.second];
        poseDiagnostics.push_back(
            "warning: " + program.TextOrEmpty(wire.moverPath) + " is a " +
            state.type + ", which this runtime has no kernel for; its "
            "points pass through");
    }

    // The compile notices a fresh evaluator seeds its first generation
    // with (inert movers, purpose warnings): file order, ahead of
    // every program line, exactly once. Drained here, on the success
    // path only, so a failed Execute keeps both the seed and the
    // previous frame.
    if (!program.compileDiagnostics.empty()) {
        poseDiagnostics.insert(poseDiagnostics.begin(),
                               program.compileDiagnostics.begin(),
                               program.compileDiagnostics.end());
        program.compileDiagnostics.clear();
    }

    // Assemble the outputs only here, at the one exit that published:
    // a failure anywhere above keeps the previous frame. Every output stages
    // its rows before any is committed, so a throw while staging publishes
    // none of this frame.
    const auto entries=[](const auto &map) {
        return [&map](auto visit) {for(const auto &entry:map)visit(entry.first,entry.second);};
    };
    const auto joints=entries(store.jointMatricesFinal);
    const auto moved=entries(store.movedProperties);
    const auto matrices=entries(store.movedMatrices);
    const auto frames=[&](auto visit) {
        const size_t count=std::min({program.slotMeta?program.slotMeta->paths.size():size_t(0),
            store.volumePlaced.size(),store.volumePlacement.size()});
        for(size_t slot=0;slot<count;++slot)if(store.volumePlaced[slot])
            visit(program.slotMeta->paths[slot],store.volumePlacement[slot]);
    };
    const auto fields=entries(store.weightFields);
    const auto providers=[&](auto visit) {
        for(const auto &entry:store.providerXforms) {
            const auto base=store.providerBaseXforms.find(entry.first);
            if(base!=store.providerBaseXforms.end())
                visit(entry.first,std::make_pair(entry.second,base->second));
        }
    };
    _StagedRows<RigExecRuntimeJointMatrix> jointRows;
    _StagedRows<RigExecRuntimePoints> pointRows;
    _StagedRows<RigExecRuntimeMatrixPrimvar> matrixRows;
    _StagedRows<RigExecRuntimeWeightFrame> frameRows;
    _StagedRows<RigExecRuntimeWeightField> fieldRows;
    _StagedRows<RigExecRuntimeProviderXform> providerRows;
    const auto pathOnly=[](auto &,const auto &) {};
    _StageRows(&_jointOrder,_jointMatrices,program,joints,pathOnly,&jointRows);
    _StageRows(&_pointOrder,_points,program,moved,pathOnly,&pointRows);
    _StageRows(&_matrixOrder,_matrixPrimvars,program,matrices,pathOnly,&matrixRows);
    _StageRows(&_weightFrameOrder,_weightFrames,program,frames,pathOnly,&frameRows);
    // A field's target comes from its file record, so it moves only with
    // membership.
    _StageRows(&_weightOrder,_weightFields,program,fields,
        [&](auto &row,const auto &value) {
            if(!program.GetText(value.target,&row.target))row.target.clear();
        },&fieldRows);
    _StageRows(&_providerOrder,_providerXforms,program,providers,pathOnly,&providerRows);
    const auto matrix=[](auto &row,const auto &value) {row.matrix=value;};
    const auto keep=[](auto &) {};
    _CommitRows(&_jointOrder,&_jointMatrices,&jointRows,joints,matrix,keep);
    _CommitRows(&_pointOrder,&_points,&pointRows,moved,
        [](auto &row,const auto &value) {
            _Recycle(&row.points);
            row.points=RigExecSharedArray<RrVec3f>(value.Share());
        },[](auto &row) {_Recycle(&row.points);});
    _CommitRows(&_matrixOrder,&_matrixPrimvars,&matrixRows,matrices,matrix,keep);
    _CommitRows(&_weightFrameOrder,&_weightFrames,&frameRows,frames,matrix,keep);
    _CommitRows(&_weightOrder,&_weightFields,&fieldRows,fields,
        [](auto &row,const auto &value) {
            _Recycle(&row.weights);
            row.weights=RigExecSharedArray<float>(value.weights.Share());
        },[](auto &row) {_Recycle(&row.weights);});
    _CommitRows(&_providerOrder,&_providerXforms,&providerRows,providers,
        [](auto &row,const auto &value) {row.matrix=value.first;row.base=value.second;},keep);
    store.movedProperties.clear();
    store.weightFields.clear();
    _diagnostics = std::move(poseDiagnostics);
    _counters = counters;
    return true;
}

}  // namespace rigExec
