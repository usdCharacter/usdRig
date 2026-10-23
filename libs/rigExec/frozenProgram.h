// Frozen program storage and its captured avar edits. Callers that only
// pass snapshots around need frozenContext.h alone.
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

/// Binding facts captured on the stage owner thread and shared unchanged
/// by every avar revision of the frozen program.
struct RigExecFrozenBindings {
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
};

/// Immutable frozen history plus an optional captured avar revision.
/// Freeze detaches live state and clears live evaluator pointers. Patching
/// shares that history and its bindings; the existing worker input placement
/// applies avarPatch before executing the common graph. A workspace clones
/// the history once into its private arena. Concurrent jobs own distinct arenas.
/// Copied USD handles remain inert; workers consume only captured facts.
struct RigExecFrozenProgram {
    std::shared_ptr<const RigExecBakedProgramImpl> program;
    std::shared_ptr<const RigExecFrozenBindings> bindings;
    std::shared_ptr<const RigExecFrozenAvarPatch> avarPatch;
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
