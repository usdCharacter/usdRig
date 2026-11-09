#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/imagingHandle.py and the RigExec
container's use of it: every imaging call names THIS session's stage.

rigExecImaging's C surface is stage-scoped (docs/multistage-imaging.md):
every per-stage entry point is RigExecImaging_<Name>ForStage(stageCacheId,
...), and the handle's plain C names -- handle.SetTime(frame),
getattr(handle, "RigExecImaging_SetTime") -- are handle ADAPTERS that
dispatch only to the scoped symbol with the handle's cache id prepended
exactly once. There is no fallback to a stage-less spelling: a library
missing a required scoped entry point is refused at load, and no legacy
symbol is ever called. The libraries are doubles here, so this runs with
no rigExecImaging, no display and no usdview.

Two stages opened from the SAME layer -- identical prim paths -- are the
case that matters: a call keyed by path alone could not tell them apart.

Usage: test_imaging_handle.py
"""
import ctypes
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd, UsdUtils  # noqa: E402

import rigExecUsdview  # noqa: E402
import imagingHandle  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Raises(action, message):
    try:
        action()
    except AttributeError:
        return
    raise AssertionError(message)


class _ScopedLibrary(object):
    """A stage-scoped library: every per-stage entry point names its
    stage, and every call records the cache id it was given, so the test
    can say which stage's context a call reached.

    Optional features (the preview, the guide bounds, the repaint-gate
    counters) are absent on purpose: the handle must report THEM absent,
    never route them to some other stage. ContextCount is present to
    prove the directory query still passes through.
    """

    def __init__(self):
        self.calls = []
        self.generations = {}
        self.active = set()

    def RigExecImaging_ActivateForStage(self, cacheId, rigPath, frame):
        self.calls.append(("ActivateForStage", cacheId, rigPath, frame))
        self.active.add(cacheId)
        return 0

    def RigExecImaging_SetTimeForStage(self, cacheId, frame):
        self.calls.append(("SetTimeForStage", cacheId, frame))
        self.generations[cacheId] = self.generations.get(cacheId, 0) + 1
        return 0

    def RigExecImaging_OnIdleForStage(self, cacheId):
        self.calls.append(("OnIdleForStage", cacheId))
        return 0

    def RigExecImaging_OnEditCommittedForStage(self, cacheId):
        self.calls.append(("OnEditCommittedForStage", cacheId))
        return 0

    def RigExecImaging_DeactivateForStage(self, cacheId):
        self.calls.append(("DeactivateForStage", cacheId))
        self.active.discard(cacheId)

    def RigExecImaging_GetGenerationForStage(self, cacheId):
        self.calls.append(("GetGenerationForStage", cacheId))
        return self.generations.get(cacheId, 0)

    def RigExecImaging_GetFrameStatesForStage(
            self, cacheId, rigPath, frames, statesOut, count):
        self.calls.append(("GetFrameStatesForStage", cacheId, bytes(rigPath),
                           count))
        for i in range(count):
            statesOut[i] = 2
        return count

    def RigExecImaging_WarmRangeForStage(self, cacheId, rigPath, frames,
                                         count):
        self.calls.append(("WarmRangeForStage", cacheId, bytes(rigPath),
                           count))
        return 0

    def RigExecImaging_IsActiveForStage(self, cacheId):
        return int(cacheId in self.active)

    def RigExecImaging_ContextCount(self):
        return len(self.active)


class _LegacyTrapLibrary(object):
    """A library from before per-stage contexts: legacy spellings only.

    Every symbol is a tripwire. The library is UNSUPPORTED: BindLibrary
    and ImagingHandle must refuse it, so no tripwire may ever fire.
    """

    def __init__(self):
        self.legacyCalls = []

    def _hit(self, name, *args):
        self.legacyCalls.append((name,) + args)
        raise AssertionError("a legacy imaging symbol was called: "
                             "RigExecImaging_" + name)

    def RigExecImaging_Activate(self, cacheId, rigPath, frame):
        self._hit("Activate", cacheId, rigPath, frame)
        return 0

    def RigExecImaging_SetTime(self, frame):
        self._hit("SetTime", frame)
        return 0

    def RigExecImaging_OnIdle(self):
        self._hit("OnIdle")
        return 0

    def RigExecImaging_OnEditCommitted(self):
        self._hit("OnEditCommitted")
        return 0

    def RigExecImaging_Deactivate(self):
        self._hit("Deactivate")

    def RigExecImaging_GetGeneration(self):
        self._hit("GetGeneration")
        return 0

    def RigExecImaging_WarmRange(self, rigPath, frames, count):
        self._hit("WarmRange", rigPath, frames, count)
        return 0


class _HalfScopedLibrary(_ScopedLibrary):
    """The scoped surface except the frame change: also unsupported.

    A library that cannot name its stage for SetTime could evaluate
    another session's stage the moment the timeline moves.
    """
    RigExecImaging_SetTimeForStage = None


def _TwinStages():
    """Two stages over ONE layer: identical prim paths, distinct stages."""
    layer = Sdf.Layer.CreateAnonymous("shared.usda")
    Usd.Stage.Open(layer).DefinePrim("/Rig", "RigExecRoot")
    first = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionA"))
    second = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionB"))
    return first, second


def _Erase(*stages):
    cache = UsdUtils.StageCache.Get()
    for stage in stages:
        if cache.Contains(stage):
            cache.Erase(stage)


def TestForStageRouting():
    """Each handle reaches its own stage's context, by cache id -- in
    both activation orders."""
    lib = _ScopedLibrary()
    first, second = _TwinStages()
    try:
        a = imagingHandle.ImagingHandle(lib, first, insert=True)
        b = imagingHandle.ImagingHandle(lib, second, insert=True)
        _Check(a.CacheId() >= 0 and b.CacheId() >= 0,
               "insert=True caches both stages")
        _Check(a.CacheId() != b.CacheId(),
               "two stages over one layer are two cache entries")
        _Check(first.GetPrimAtPath("/Rig") and second.GetPrimAtPath("/Rig"),
               "and they do share the prim path")

        # Order one: A activates first.
        _Check(a.Activate(b"", 1.0) == 0 and b.Activate("", 1.0) == 0,
               "both stages activate")
        _Check(("ActivateForStage", a.CacheId(), b"", 1.0) in lib.calls and
               ("ActivateForStage", b.CacheId(), b"", 1.0) in lib.calls,
               "activation named each stage: %r" % lib.calls)

        a.SetTime(3.0)
        a.SetTime(4.0)
        b.SetTime(1.0)
        _Check(("SetTimeForStage", a.CacheId(), 3.0) in lib.calls,
               "A's frame went to A's context: %r" % lib.calls)
        _Check(("SetTimeForStage", b.CacheId(), 1.0) in lib.calls,
               "B's frame went to B's context")
        _Check(a.GetGeneration() == 2 and b.GetGeneration() == 1,
               "each stage's generation moved with its own frames only")

        # Order two: B activates again first, then A; both stay separate.
        b.SetTime(2.0)
        a.SetTime(5.0)
        _Check(a.GetGeneration() == 3 and b.GetGeneration() == 2,
               "reversing the order did not merge the stages")

        # The alias names, looked up by getattr the way cacheStripModel
        # looks them up on what it is handed: adapters, cache id in front
        # exactly once.
        entry = getattr(b, "RigExecImaging_GetFrameStates", None)
        _Check(entry is not None, "the handle answers to the C spelling")
        frames = (ctypes.c_double * 3)(1.0, 2.0, 3.0)
        states = (ctypes.c_int * 3)()
        _Check(entry(b"/Rig", frames, states, 3) == 3,
               "the C-spelled call returns the library's answer")
        _Check(lib.calls[-1] == ("GetFrameStatesForStage", b.CacheId(),
                                 b"/Rig", 3),
               "and was routed to B with B's id, once: %r" % (lib.calls[-1],))
        setTime = getattr(b, "RigExecImaging_SetTime")
        setTime(7.0)
        _Check(lib.calls[-1] == ("SetTimeForStage", b.CacheId(), 7.0),
               "the SetTime adapter prepends the cache id exactly once")

        a.OnEditCommitted()
        a.OnIdle()
        _Check(lib.calls[-2:] == [("OnEditCommittedForStage", a.CacheId()),
                                  ("OnIdleForStage", a.CacheId())],
               "warming triggers reach A only")

        # Warming: present optional feature, reached by its legacy name.
        warmFrames = (ctypes.c_double * 2)(1.0, 2.0)
        _Check(a.WarmRange(b"/Rig", warmFrames, 2) == 0, "warm range")
        _Check(lib.calls[-1] == ("WarmRangeForStage", a.CacheId(), b"/Rig", 2),
               "the warm range names A")

        # Activation already takes the id; deactivation is per stage.
        _Check(a.IsActive() and b.IsActive(), "both are active at once")
        a.Deactivate()
        _Check(lib.calls[-1] == ("DeactivateForStage", a.CacheId()),
               "A's deactivation names A")
        _Check(not a.IsActive() and b.IsActive(),
               "deactivating A leaves B active")

        # Every recorded call is a scoped one: no stage-less spelling ran.
        _Check(all(name.endswith("ForStage")
                   for name, *_rest in lib.calls),
               "a stage-less imaging symbol was called: %r" % lib.calls)

        # The id-taking entry points keep their library signatures.
        count = imagingHandle.ContextCount(lib)
        _Check(count == len(lib.active),
               "ContextCount stays a directory query")
        _Check(getattr(b, "RigExecImaging_ActivateForStage") is not None,
               "the scoped activation name passes through getattr")
    finally:
        _Erase(first, second)
    print("  ok: ForStage routing keeps two same-path stages apart "
          "in both orders")


def TestMissingEntryPoints():
    """No scoped export: AttributeError, like a CDLL, and Has() says so."""
    lib = _ScopedLibrary()
    handle = imagingHandle.ImagingHandle(lib, None)
    _Check(getattr(handle, "RigExecImaging_BeginPreview", None) is None,
           "getattr with a default answers None for a missing entry point")
    _Check(not handle.Has("BeginPreview"), "Has() says so too")
    _Check(handle.Has("SetTime") and handle.Has("RigExecImaging_WarmRange"),
           "Has() accepts both spellings")
    _Check(not handle.Has("Activate"),
           "the stage-less Activate name is not an entry point")
    _Check(handle.Has("ActivateForStage"),
           "activation is bound as ActivateForStage")
    _Check(getattr(handle, "RigExecImaging_Activate", None) is None,
           "getattr reports the legacy Activate name absent")
    _Raises(handle.EndPreview,
            "a missing entry point must raise AttributeError")
    _Raises(lambda: handle.RigExecImaging_NotAThing,
            "an unknown name must raise AttributeError")
    _Check(imagingHandle.ImagingHandle(None, None).lib is None,
           "a handle over no library is inert")
    _Check(imagingHandle.ContextCount(None) is None,
           "no library, no context count")
    print("  ok: missing entry points raise AttributeError")


def TestLazyCacheId():
    """insert=False waits for someone else to cache the stage."""
    lib = _ScopedLibrary()
    stage, _other = _TwinStages()
    try:
        handle = imagingHandle.ImagingHandle(lib, stage)
        _Check(handle.CacheId() == -1, "not cached, not inserted: -1")
        _Check(not UsdUtils.StageCache.Get().Contains(stage),
               "insert=False never inserts")
        cacheId = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
        _Check(handle.CacheId() == cacheId,
               "the id is looked up again once the stage is cached")
        handle.SetTime(2.0)
        _Check(lib.calls[-1] == ("SetTimeForStage", cacheId, 2.0),
               "and calls carry it")
    finally:
        _Erase(stage)
    print("  ok: cache id resolves lazily without inserting")


def TestLegacyLibraryRefused():
    """A pre-scoping library is refused, and none of its symbols runs.

    BindLibrary and every ImagingHandle over it raise AttributeError
    naming the missing scoped entry points. The trap library raises at
    the first legacy call; reaching this test's final assertions at all
    means no tripwire fired."""
    trap = _LegacyTrapLibrary()
    stage, _other = _TwinStages()
    try:
        _Raises(lambda: imagingHandle.BindLibrary(trap),
                "a legacy-only library must be refused at BindLibrary")
        _Raises(lambda: imagingHandle.ImagingHandle(trap, stage),
                "no handle may exist over a legacy-only library")
        half = _HalfScopedLibrary()
        _Raises(lambda: imagingHandle.BindLibrary(half),
                "a library missing a required scoped symbol is refused")
        _Raises(lambda: imagingHandle.ImagingHandle(half, stage),
                "and no handle may exist over it either")
        _Check(not trap.legacyCalls,
               "a legacy imaging symbol ran: %r" % trap.legacyCalls)
    finally:
        _Erase(stage)
    print("  ok: a library without the scoped surface is refused, "
          "never used")


def _BareContainer(lib, stage):
    """A RigExec container with only what activation reads, as the
    testusdview harness builds them (testUsdviewRigExec.py)."""
    dataModel = type("DataModel", (), {})()
    dataModel.stage = stage
    dataModel.currentFrame = Usd.TimeCode(1)
    dataModel.viewSettings = type("ViewSettings", (), {})()
    dataModel.viewSettings.displayGuide = True
    api = type("UsdviewApi", (), {})()
    api.dataModel = dataModel

    container = rigExecUsdview.RigExecUsdviewContainer.__new__(
        rigExecUsdview.RigExecUsdviewContainer)
    container._api = api
    container._lib = lib
    container._imaging = None
    container._active = False
    container._rigPaths = []
    container._cachedStage = None
    container._stageNoticeKey = None
    container._activating = False
    container._warmingCommitPending = False
    return container


def TestContainersNeverTouchEachOther():
    """Two sessions' containers in one process, one scoped-only library."""
    lib = _ScopedLibrary()
    first, second = _TwinStages()
    a = _BareContainer(lib, first)
    b = _BareContainer(lib, second)
    try:
        a._ActivateCurrentStage()
        b._ActivateCurrentStage()
        idA = a._Imaging().CacheId()
        idB = b._Imaging().CacheId()
        _Check(a._active and b._active, "both sessions activated")
        _Check(idA != idB and lib.active == {idA, idB},
               "both stages active at once: %r" % lib.active)
        _Check((("ActivateForStage", idA, b"", 1.0) in lib.calls) and
               (("ActivateForStage", idB, b"", 1.0) in lib.calls),
               "each container activated its own stage: %r" % lib.calls)

        # Frame changes drive only their own stage.
        a._OnFrameChanged(3.0)
        b._OnFrameChanged(1.0)
        _Check(("SetTimeForStage", idA, 3.0) in lib.calls and
               ("SetTimeForStage", idB, 1.0) in lib.calls,
               "each session's frame reached its own stage")
        _Check(lib.generations == {idA: 1, idB: 1},
               "one publication each: %r" % lib.generations)

        # Re-activating B deactivates B only -- the other activation order.
        del lib.calls[:]
        b._ActivateCurrentStage()
        _Check(("DeactivateForStage", idB) in lib.calls,
               "B's reactivation deactivated B first")
        _Check(("DeactivateForStage", idA) not in lib.calls,
               "and never A")
        _Check(idA in lib.active, "A stayed active throughout")

        # B's shutdown leaves A evaluating.
        b._Shutdown()
        _Check(("DeactivateForStage", idB) in lib.calls and
               ("DeactivateForStage", idA) not in lib.calls,
               "B's shutdown deactivated B and not A")
        _Check(lib.active == {idA}, "A is still active: %r" % lib.active)
        a._OnFrameChanged(2.0)
        _Check(lib.generations[idA] == 2, "and A still evaluates")

        # A container that never activated deactivates nothing at all.
        idle = _BareContainer(lib, Usd.Stage.CreateInMemory())
        del lib.calls[:]
        idle._Shutdown()
        _Check(not lib.calls,
               "a never-activated container deactivates nothing: %r"
               % lib.calls)
    finally:
        a._Shutdown()
        _Erase(first, second)
    print("  ok: containers of two sessions never deactivate each other")


def TestUnsupportedLibraryDegradesTheContainer():
    """A container facing an unsupported library warns and stops.

    The load path reports the refusal; the container then runs with no
    imaging at all. It must never activate, evaluate, or deactivate
    anything -- the trap library raises the moment any of its symbols
    is touched."""
    trap = _LegacyTrapLibrary()
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")
    container = _BareContainer(None, stage)

    load = rigExecUsdview._LoadRigExecImaging
    try:
        rigExecUsdview._LoadRigExecImaging = (
            lambda: imagingHandle.BindLibrary(trap))
        try:
            container._OnStageReplaced()
        except Exception as error:
            raise AssertionError(
                "the unsupported library raised through stage replacement "
                "instead of degrading: %r" % error)
        _Check(container._lib is None,
               "the refusal left no library bound")
        _Check(not container._active, "no activation claimed")
        container._OnFrameChanged(2.0)
        _Check(not trap.legacyCalls,
               "the unsupported library was used anyway: %r"
               % trap.legacyCalls)
        container._Shutdown()
        _Check(not trap.legacyCalls,
               "shutdown reached the unsupported library: %r"
               % trap.legacyCalls)
    finally:
        rigExecUsdview._LoadRigExecImaging = load
        container._ReleaseCachedStage()
        _Erase(stage)
    print("  ok: an unsupported library is reported, and the session "
          "runs with no imaging")


def TestFailedActivationEnablesGuides():
    class FailedLibrary(_ScopedLibrary):
        def RigExecImaging_ActivateForStage(self, cacheId, rigPath, frame):
            self.calls.append(("ActivateForStage", cacheId, rigPath, frame))
            return 3

    stage, other = _TwinStages()
    container = _BareContainer(FailedLibrary(), stage)
    container._api.dataModel.viewSettings.displayGuide = False
    try:
        container._ActivateCurrentStage()
        _Check(not container._active, "compile failure stays inactive")
        _Check(container._api.dataModel.viewSettings.displayGuide,
               "authored guides remain enabled after compile failure")
    finally:
        container._Shutdown()
        _Erase(stage, other)
    print("  ok: failed activation leaves authored guides enabled")


def main():
    TestForStageRouting()
    TestMissingEntryPoints()
    TestLazyCacheId()
    TestLegacyLibraryRefused()
    TestContainersNeverTouchEachOther()
    TestUnsupportedLibraryDegradesTheContainer()
    TestFailedActivationEnablesGuides()
    print("IMAGING_HANDLE_OK (7 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
