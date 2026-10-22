// The frozen program snapshot's definition. It holds a whole baked program
// clone, so only code that reads a snapshot's fields includes this; code
// that only passes snapshots around needs frozenContext.h alone.
#ifndef RIGEXEC_FROZEN_PROGRAM_H
#define RIGEXEC_FROZEN_PROGRAM_H

#include "bakedProgramImpl.h"
#include "frozenContext.h"

#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"

#include <map>
#include <thread>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Detached edit payload relative to one immutable frozen snapshot.
struct RigExecFrozenAvarPatch {
    const RigExecFrozenProgram *basis = nullptr; ///< identity only; caller pins the base
    struct Avar { double constant, value; bool varying; };
    std::vector<Avar> avars;
    std::vector<double> constants;
    std::vector<size_t> promoted;
    std::vector<uint64_t> editSerial;
    size_t varyingInputs = 0;
    uint64_t valueEditSerial = 0, programStamp = 0;
};

/// A worker-safe snapshot of a baked program tied to its compiled rig.
///
/// Built on the UI thread by RigExecFreezeProgram, which deep-copies the
/// live program's compiled rig tables and per-frame state into `program`, nulls
/// every pointer to live evaluator state (evaluator, stage, caches,
/// evaluator-bound std::functions), and captures the small compiled rig values the
/// frame path reads through those pointers (the joint/solver binding map,
/// whether guide taps stand). Immutable after the freeze: shared across the
/// jobs of its compiled rig, read-only on every thread. A workspace clones the
/// snapshot once into private working state and executes sampled jobs through
/// the shared graph. Concurrent jobs use distinct workspace lanes.
///
/// The clone keeps the live program's USD handles COPIED but DEAD: no frozen
/// code path dereferences them (see the audit on RigExecFreezeProgram in
/// frozenContext.cpp), and the worker nulls every input head/query it patches
/// so a misrouted read fails closed onto the patched constant instead of
/// reaching the stage. A null or mismatched snapshot (or none at all, see
/// RigExecFrozenEvalContext::frozen) declines the job; the frame evaluates
/// live when asked.
struct RigExecFrozenProgram {
    /// The program clone: compiled rig tables plus the per-frame state as it stood
    /// at freeze time (the history the frozen run branches from). Live
    /// pointers nulled; see RigExecFreezeProgram.
    RigExecBakedProgramImpl program;
    /// Copy of the evaluator's joint/solver binding map (compiled rig data the
    /// pose epilogue reads through a live pointer).
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> jointSolverBinding;
    /// Whether the live program had guide taps standing at freeze time.
    /// The frozen epilogue replays the guide publication from the aggregates
    /// when this and the context's guides flag are both set.
    bool solverGuidesPresent = false;
    // Freeze-captured handle identities, plain data for the worker. A
    // worker must not dereference a USD handle -- not even IsValid or
    // GetPath, which reach composed specs and prim data -- so every handle
    // identity the frozen run needs is captured here on the UI thread.
    /// Every patchable input's head path, in _ForEachPatchableInput order
    /// (empty when the head is invalid; the worker declines a varying input
    /// it cannot key).
    std::vector<SdfPath> inputHeadPaths;
    std::vector<VtValue> inputConstants; ///< typed fallbacks, parallel to inputHeadPaths
    /// Per constraintArrays entry, the four operator-array keys (source
    /// weights, translation offsets, rotation offsets, pole weights); empty
    /// where the entry reads no such array.
    std::vector<SdfPath> arrayKeys;
    /// Per chain, whether its base query is valid (haveBase needs it).
    std::vector<char> chainBaseQueryValid;
    /// Per derived target (derivedIndex order), same.
    std::vector<char> derivedBaseQueryValid;
    /// Per solver, whether its ribbon query is valid.
    std::vector<char> ribbonQueryValid;
    /// Per chain revision (revisionIndex order), whether the mover carries
    /// the three per-frame scalar inputs the packet assembly reads.
    std::vector<char> moverHasEnabled;
    std::vector<char> moverHasDefaultWeight;
    std::vector<char> moverHasMethod;
    /// Per chain revision (revisionIndex order), the sample key of the
    /// mover's inputs:defaultWeight, built here so the worker builds no
    /// path.
    std::vector<SdfPath> moverDefaultWeightKeys;
    /// Per weight object, four synthetic sample keys in
    /// frozenDetail::_FrozenWeightArrayRole order, built here with the
    /// sampler's own key function.
    std::vector<SdfPath> weightArrayKeys;
    /// Whether input-replay capture (RIGEXEC_INPUT_REPLAY) was requested at
    /// freeze, and the thread that froze. Capture is a serial owning-thread
    /// log: only that thread records boundaries for this snapshot, and no
    /// worker lane reads the environment or touches the recorder.
    bool inputReplayCapture = false;
    std::thread::id inputReplayOwner;
    bool RecordsInputReplay() const
    {
        return inputReplayCapture &&
               inputReplayOwner == std::this_thread::get_id();
    }
};

}  // namespace rigExec

#endif  // RIGEXEC_FROZEN_PROGRAM_H
