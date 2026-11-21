#!/usr/bin/env python
"""
Headless test for per-session plugin state: several usdview sessions in ONE
process (usdOrchestrate's shared host) each get their own gizmo, view cube,
editors, preview channel, published-frame reader and container, with no
cross-talk.

usdview gives every window its own AppController and UsdviewApi, but a
module exists once per process, so anything a plugin kept at module scope
used to be shared: the gizmo and the view cube installed themselves into the
first window only. plugin/rigExecUsdview/sessionRegistry.py now files that
state under the session's main window.

Fake apis over real (never shown, never GL) QMainWindows on Qt's offscreen
platform: no usdview, no rigExecImaging, no display. The controllers
themselves are replaced by doubles -- what is under test is which session
gets which, not what a gizmo draws.

Usage: test_session_state.py
"""
from collections import Counter
from contextlib import ExitStack, contextmanager
import gc
import os
import sys
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets  # noqa: E402

import avarDock  # noqa: E402
import gizmoMath  # noqa: E402
import gizmoPreview  # noqa: E402
import gizmoUI  # noqa: E402
import sessionRegistry  # noqa: E402
import stageViewReady  # noqa: E402
import viewAxisUI  # noqa: E402
import viewCubeUI  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
    ["test_session_state"])


class _Api(object):
    """What the plugins read off a UsdviewApi: its window and its stage."""

    def __init__(self, window, stage=None):
        self.qMainWindow = window
        self.stage = stage
        self._UsdviewApi__appController = SimpleNamespace(_stageView=None)


def _Session(stage=None):
    window = QtWidgets.QMainWindow()
    return window, _Api(window, stage)


def _Host(api):
    return api._UsdviewApi__appController


def _Destroy(window):
    """Destroy the window's C++ object now, as closing a session does."""
    try:
        import shiboken6
        shiboken6.delete(window)
        return
    except ImportError:
        pass
    window.deleteLater()
    # Deliver deletion without also dispatching any pending install timers.
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)


class _ActiveWindow(object):
    """Pin sessionRegistry's view of QApplication.activeWindow()."""

    def __init__(self, window):
        self._window = window
        self._saved = None

    def __enter__(self):
        self._saved = sessionRegistry.ActiveWindow
        sessionRegistry.ActiveWindow = lambda: self._window
        return self

    def __exit__(self, *args):
        sessionRegistry.ActiveWindow = self._saved
        return False


class _FakeController(object):
    built = []

    def __init__(self, usdviewApi, *args, **kwargs):
        self.usdviewApi = usdviewApi
        _FakeController.built.append(self)


class _Patched(object):
    """Swap module attributes for the duration of a block."""

    def __init__(self, module, **values):
        self._module = module
        self._values = values
        self._saved = {}

    def __enter__(self):
        for name, value in self._values.items():
            self._saved[name] = getattr(self._module, name)
            setattr(self._module, name, value)
        return self

    def __exit__(self, *args):
        for name, value in self._saved.items():
            setattr(self._module, name, value)
        return False


def TestRegistry():
    """Weak keys, destroyed windows, and no winner by recency."""
    registry = sessionRegistry.SessionRegistry("test")
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    registry.Set(apiA, "A")
    registry.Set(windowB, "B")
    _Check(registry.Get(apiA) == "A" and registry.Get(windowA) == "A",
           "an api and its window are the same session")
    _Check(registry.Get(apiB) == "B", "B is its own session")
    _Check(len(registry) == 2, "two sessions")
    with _ActiveWindow(None):
        _Check(registry.Current() is None,
               "two sessions and no active window is ambiguous: no answer, "
               "and certainly not the most recent one")
    with _ActiveWindow(windowA):
        _Check(registry.Current() == "A", "the active window's session")
    # A panel: a top-level window parented to the main window.
    panel = QtWidgets.QDialog(windowB)
    child = QtWidgets.QLineEdit(panel)
    with _ActiveWindow(panel):
        _Check(registry.Current() == "B",
               "a panel's session is its owner's")
    _Check(registry.SessionOf(child) is windowB,
           "a widget inside a panel belongs to the panel's owner")
    _Check(sessionRegistry.SessionOfEvent(child, windowB) and
           not sessionRegistry.SessionOfEvent(child, windowA),
           "SessionOfEvent follows the same chain")

    _Destroy(windowA)
    _Check(len(registry) == 1 and registry.Get(apiA) is None,
           "destroying A's window dropped A's entry")
    with _ActiveWindow(None):
        _Check(registry.Current() == "B",
               "one session left: it is the answer, as in a plain usdview")

    # A headless key (no window at all) is weakly held.
    class _Headless(object):
        pass
    key = _Headless()
    registry.Set(key, "headless")
    _Check(registry.Get(key) == "headless", "headless keys work")
    del key
    _Check(len(registry) == 1, "a collected headless key drops its entry")
    _Destroy(windowB)
    _Check(len(registry) == 0, "and B goes with its window")

    # A host ending a session without destroying its window forgets it
    # everywhere at once.
    windowC, apiC = _Session()
    other = sessionRegistry.SessionRegistry("test other")
    registry.Set(apiC, "C")
    other.Set(windowC, "C2")
    _Check(sessionRegistry.ForgetSession(apiC) >= 2,
           "ForgetSession reached every registry holding C")
    _Check(registry.Get(apiC) is None and other.Get(apiC) is None,
           "and C is gone from both")
    _Destroy(windowC)
    print("  ok: registry keys, ownership and lifetimes")


def TestGizmoAndViewCubePerSession():
    """Each session installs its own gizmo and its own view cube."""
    _FakeController.built = []
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    _Host(apiA)._stageView = _Host(apiB)._stageView = object()
    with _Patched(gizmoUI, GizmoController=_FakeController), \
            _Patched(viewCubeUI, ViewCubeController=_FakeController):
        gizmoA = gizmoUI.InstallViewportTools(apiA, undoStack=None)
        gizmoB = gizmoUI.InstallViewportTools(apiB, undoStack=None)
        cubeA = viewCubeUI.InstallViewCube(apiA)
        cubeB = viewCubeUI.InstallViewCube(apiB)
        _Check(gizmoA is not None and gizmoB is not None,
               "the SECOND window got viewport tools too")
        _Check(gizmoA is not gizmoB, "and its own controller")
        _Check(gizmoA.usdviewApi is apiA and gizmoB.usdviewApi is apiB,
               "each controller is built on its own session's api")
        _Check(cubeA is not None and cubeB is not None and
               cubeA is not cubeB, "each window has its own view cube")
        _Check(gizmoUI.InstallViewportTools(apiA, None) is gizmoA and
               viewCubeUI.InstallViewCube(apiB) is cubeB,
               "installing again is a no-op per session")
        _Check(len(_FakeController.built) == 4, "four controllers, once")

        # GetController with and without an api.
        _Check(gizmoUI.GetController(apiA) is gizmoA and
               gizmoUI.GetController(apiB) is gizmoB and
               viewCubeUI.GetController(apiB) is cubeB,
               "GetController(api) answers for that session")
        _Check(gizmoUI.GetController(windowB) is gizmoB,
               "a main window names its session too")
        with _ActiveWindow(windowB):
            _Check(gizmoUI.GetController() is gizmoB and
                   viewCubeUI.GetController() is cubeB,
                   "without an api: the active window's session")
            _Check(gizmoUI._controller is gizmoB,
                   "the old module attribute is a view onto the same")
        with _ActiveWindow(None):
            _Check(gizmoUI.GetController() is None,
                   "two sessions, no active window: None, not the newest")

        # Closing one window drops its controllers and leaves the other.
        _Destroy(windowA)
        _Check(gizmoUI.GetController(apiA) is None and
               viewCubeUI.GetController(apiA) is None,
               "A's controllers went with A's window")
        _Check(gizmoUI.GetController(apiB) is gizmoB and
               viewCubeUI.GetController(apiB) is cubeB,
               "B's are untouched")
        with _ActiveWindow(None):
            _Check(gizmoUI.GetController() is gizmoB,
                   "one session left: GetController() is its controller, "
                   "exactly as in a plain usdview")
    _Destroy(windowB)
    _Check(gizmoUI.GetController() is None, "no sessions, no controller")
    print("  ok: gizmo and view cube install once per session")


_INSTALLERS = (
    ("gizmo", gizmoUI, "GizmoController"),
    ("viewCube", viewCubeUI, "ViewCubeController"),
    ("avarDock", avarDock, "AvarDockController"),
    ("viewAxis", viewAxisUI, "ViewAxisController"),
)


def _Requests(api, undo=None, opener=None, retries=stageViewReady.DEFAULT_RETRIES):
    return (
        lambda: gizmoUI.InstallViewportTools(api, undo, retries, opener),
        lambda: viewCubeUI.InstallViewCube(api, retries),
        lambda: avarDock.InstallAvarDock(api, undo, retries),
        lambda: viewAxisUI.InstallViewAxis(api, retries),
    )


@contextmanager
def _ToolDoubles(onProbe=None, onCreate=None):
    probes = Counter()
    built = []
    stageView = stageViewReady.StageView

    def _Probe(api):
        probes[api] += 1
        if onProbe is not None:
            onProbe(api, probes[api])
        return stageView(api)

    def _Constructor(tool):
        def _Create(api, *args, **kwargs):
            # Read the fake host field without incrementing the broker count.
            controller = SimpleNamespace(api=api, view=stageView(api),
                                         args=args, kwargs=kwargs)
            built.append((api, tool, controller))
            if onCreate is not None:
                onCreate(api, tool)
            return controller
        return _Create

    with ExitStack() as patches:
        patches.enter_context(_Patched(stageViewReady, StageView=_Probe))
        for tool, module, constructor in _INSTALLERS:
            patches.enter_context(_Patched(
                module, **{constructor: _Constructor(tool)}))
        yield probes, built


def _Results(api):
    return [stageViewReady.GetResult(api, tool) for tool, _, _ in _INSTALLERS]


def _PumpUntil(predicate):
    for _ in range(2 * stageViewReady.DEFAULT_RETRIES):
        if predicate():
            return
        _APP.processEvents()
    _Check(predicate(), "queued StageView work did not settle")


def _PumpExtraTurns():
    for _ in range(stageViewReady.DEFAULT_RETRIES + 1):
        _APP.processEvents()


def TestAllInstallersImmediate():
    window, api = _Session()
    _Host(api)._stageView = object()
    undo, opener = object(), lambda: None
    with _ToolDoubles() as (probes, built):
        requests = _Requests(api, undo, opener, retries=0)
        controllers = [request() for request in requests]
        _Check(all(c is not None for c in controllers), "all four are ready")
        _Check([request() for request in requests] == controllers and
               len(built) == probes[api] == 4,
               "each new tool probes once; repeated requests do not probe")
        for result, controller in zip(_Results(api), controllers):
            _Check(result == stageViewReady.InstallResult("ready", controller)
                   and result.controller is controller and controller.api is api
                   and controller.view is _Host(api)._stageView,
                   "ready carries the controller built for this session")
        _Check(controllers[0].args == (undo,) and controllers[0].kwargs == {
            "openGraphEditor": opener, "installFilters": True},
            "standalone gizmo installs its own input filters")
        _Check(controllers[2].args == (undo,), "dock gets the session's undo")
    _Destroy(window)
    print("  ok: all four installers are immediate and idempotent")


def TestPendingInstallPerSession():
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    undoA, undoB, ignored = object(), object(), object()
    def _OpenGraphEditor():
        pass

    notified = []
    reentered = []
    delegated = set()

    def _Reenter(api, tool):
        if tool == "gizmo":
            reentered.append((stageViewReady.GetResult(api, tool),
                              gizmoUI.InstallViewportTools(api, ignored)))

    def _Added(api, kind, controller):
        notified.append((api, kind, controller, gizmoUI.GetController(api),
                         stageViewReady.GetResult(api, "gizmo")))

    with _ToolDoubles(onCreate=_Reenter) as (probes, built), \
            _Patched(gizmoUI.editorExtension,
                     OwnsInput=lambda api: api in delegated, EditorAdded=_Added):
        order = (2, 0, 3, 1)
        for api, undo in ((apiA, undoA), (apiB, undoB)):
            requests = _Requests(api, undo, _OpenGraphEditor)
            for i in order:
                _Check(requests[i]() is None, "no view yet: installer queues")
            _Check(all(request() is None for request in _Requests(api, ignored)),
                   "repeated requests join without replacing inputs")
        old = _Results(apiA)
        pending = stageViewReady.InstallResult("pending")
        _Check(old == _Results(apiB) == [pending] * 4 and
               probes == {apiA: 1, apiB: 1}, "one initial probe per session")
        try:
            old[0].status = "ready"
        except AttributeError:
            pass
        else:
            raise AssertionError("install snapshots must be immutable")

        delegated.add(apiA)
        _Host(apiA)._stageView = object()
        _PumpUntil(lambda: all(r.status == "ready" for r in _Results(apiA))
                   and probes[apiB] >= 2)
        _Check(_Results(apiB) == [pending] * 4,
               "B is still waiting after A became ready")
        with _ActiveWindow(windowB):
            for tool, module, _ in _INSTALLERS:
                ready = module.GetController(windowA)
                _Check(ready is not None and module.GetController(apiB) is None
                       and module.GetController(windowB) is None
                       and module.GetController() is ready
                       and module._controller is ready
                       and stageViewReady.Controllers(tool) == [ready],
                       "pending active session does not mask the sole ready tool")

        _Host(apiB)._stageView = object()
        _PumpUntil(lambda: all(r.status == "ready" for r in _Results(apiB)))
        _Check([(api, tool) for api, tool, _ in built] == [
            (api, _INSTALLERS[i][0]) for api in (apiA, apiB) for i in order],
            "all factories run once, in registration order within each session")
        _Check(probes[apiA] == 2 and probes[apiB] > probes[apiA],
               "only one broker probe per session per turn")
        _Check(old == [pending] * 4 and reentered == [(pending, None)] * 2,
               "old snapshots and constructor reentry see pending")
        gizmoA, gizmoB = (gizmoUI.GetController(api) for api in (apiA, apiB))
        _Check(notified == [(apiA, gizmoUI.editorExtension.GIZMO, gizmoA,
                             gizmoA, stageViewReady.GetResult(apiA, "gizmo"))],
               "delegation is evaluated at construction and notified after ready")
        _Check(gizmoA.args == (undoA,) and gizmoB.args == (undoB,) and
               gizmoA.kwargs == {"openGraphEditor": _OpenGraphEditor,
                                 "installFilters": False} and
               gizmoB.kwargs == {"openGraphEditor": _OpenGraphEditor,
                                 "installFilters": True} and
               avarDock.GetController(apiA).args == (undoA,) and
               avarDock.GetController(apiB).args == (undoB,),
               "first undo and opener inputs win for each session")
        for api in (apiA, apiB):
            for request, (_, module, _) in zip(_Requests(api, ignored), _INSTALLERS):
                controller = request()
                _Check(controller is module.GetController(api) and
                       controller.api is api and controller.view is _Host(api)._stageView,
                       "repeat returns its own ready controller and view")
        with _ActiveWindow(None):
            for _, module, _ in _INSTALLERS:
                _Check(module.GetController() is None and module._controller is None,
                       "two ready sessions without active ownership are ambiguous")
        with _ActiveWindow(windowB):
            for _, module, _ in _INSTALLERS:
                _Check(module._controller is module.GetController(apiB) and
                       module._controller is not module.GetController(apiA),
                       "active/legacy lookup selects B, not another session")
        settled = probes.copy()
        _PumpExtraTurns()
        _Check(probes == settled and len(built) == 8, "settled sessions do no work")
    _Destroy(windowA)
    _Destroy(windowB)
    print("  ok: staggered readiness, FIFO, delegation, and ready-only lookups")


def TestInstallProbeBounds():
    windowLast, apiLast = _Session()
    windowSmall, apiSmall = _Session()
    windowZero, apiZero = _Session()
    windowHeadless, apiHeadless = _Session()
    _Host(apiHeadless)._noRender = True

    def _RevealLast(api, count):
        if api is apiLast and count == 3:
            _Host(api)._stageView = object()

    with _ToolDoubles(onProbe=_RevealLast) as (probes, built):
        for api, retries in ((apiLast, 2), (apiSmall, 2), (apiZero, 0),
                             (apiHeadless, stageViewReady.DEFAULT_RETRIES)):
            _Check(_Requests(api, retries=retries)[0]() is None,
                   "no view during first probe")
        unavailable = stageViewReady.InstallResult("unavailable")
        for api in (apiZero, apiHeadless):
            _Check(_Results(api)[0] == unavailable,
                   "zero retries and known no-render settle synchronously")
        old = _Results(apiSmall)[0]
        for api in (apiLast, apiSmall):
            for request in _Requests(api)[1:]:
                request()
        _PumpUntil(lambda: all(r.status != "pending" for r in _Results(apiSmall))
                   and all(r.status != "pending" for r in _Results(apiLast)))
        _Check(all(r.status == "ready" for r in _Results(apiLast)) and
               [api for api, _, _ in built] == [apiLast] * 4,
               "the final allowed probe can still publish all waiting tools")
        _Check(old == stageViewReady.InstallResult("pending"),
               "exhaustion does not change the old snapshot")
        for api in (apiSmall, apiZero, apiHeadless):
            if api is not apiHeadless:
                _Host(api)._stageView = object()
            _Check(all(request() is None for request in _Requests(api)) and
                   _Results(api) == [unavailable] * 4,
                   "unavailability is terminal for repeated and new tools")
        _PumpExtraTurns()
        _Check(probes == {apiLast: 3, apiSmall: 3, apiZero: 1, apiHeadless: 1},
               "late arrivals cannot extend the shared bound or rearm a timer")
    for window in (windowLast, windowSmall, windowZero, windowHeadless):
        _Destroy(window)
    print("  ok: final probe, bounded retries, zero, and no-render outcomes")


def TestInstallFailures():
    timerType = QtCore.QTimer

    class _Timer(timerType):
        def start(self, interval):
            if self.isSingleShot() and getattr(self, "started", False):
                raise RuntimeError("timer rearm error")
            self.started = True
            super().start(interval)

    for failure in ("factory", "probe", "timer"):
        window, api = _Session()
        with ExitStack() as patches:
            def _FailProbe(api, count):
                if failure == "probe" and count == 2:
                    raise RuntimeError("probe error")

            def _FailCube(api, tool):
                if failure == "factory" and tool == "viewCube":
                    raise RuntimeError("factory error")

            probes, built = patches.enter_context(_ToolDoubles(_FailProbe, _FailCube))
            warnings = []
            logger = stageViewReady.logging.getLogger(stageViewReady.__name__)
            patches.enter_context(_Patched(logger, warning=lambda fmt, *args:
                                           warnings.append(fmt % args)))
            if failure == "timer":
                patches.enter_context(_Patched(QtCore, QTimer=_Timer))
            requests = _Requests(api)
            _Check(all(request() is None for request in requests) and
                   all(r.status == "pending" for r in _Results(api)),
                   "the failure must happen after all tools are waiting")
            if failure == "factory":
                _Host(api)._stageView = object()
            _PumpUntil(lambda: all(r.status != "pending" for r in _Results(api)))
            results = _Results(api)
            failed = []
            for (tool, _, _), result, request in zip(_INSTALLERS, results, requests):
                if failure != "factory" or tool == "viewCube":
                    failed.append(tool)
                    _Check(result.status == "failed" and result.controller is None
                           and failure in result.error and "error" in result.error,
                           "affected tools fail with a diagnostic and no controller")
                else:
                    _Check(result.status == "ready" and result.controller is not None
                           and result.error is None, "peers survive a factory error")
                _Check(request() is result.controller, "repeat returns the same outcome")
            _PumpExtraTurns()
            _Check(len(built) == (4 if failure == "factory" else 0) and
                   _Results(api) == results and probes[api] == 2,
                   "failure settles without reconstructing or probing again")
            _Check(len(warnings) == len(failed) and all(
                tool in warning and failure in warning
                for tool, warning in zip(failed, warnings)),
                "warn once per failed tool with its cause")
        _Destroy(window)
    print("  ok: constructor isolation and shared probe/timer failures")


def TestRetirePendingInstalls():
    windowClosed, apiClosed = _Session()
    windowForgotten, apiForgotten = _Session()
    windowRetired, apiRetired = _Session()
    windowSurvivor, apiSurvivor = _Session()
    notified = []
    oldUndo, freshUndo = object(), object()

    def _RetireDuringCreate(api, tool):
        if api is apiRetired and tool == "gizmo":
            stageViewReady.Retire(api)
            stageViewReady.Retire(api)

    with _ToolDoubles(onCreate=_RetireDuringCreate) as (probes, built), \
            _Patched(gizmoUI.editorExtension, OwnsInput=lambda api: True,
                     EditorAdded=lambda api, kind, c: notified.append(api)):
        for api in (apiClosed, apiForgotten, apiRetired, apiSurvivor):
            _Check(all(request() is None for request in _Requests(api, oldUndo)),
                   "pending before retirement")
        old = _Results(apiForgotten)
        _Destroy(windowClosed)
        _Check(sessionRegistry.ForgetSession(windowForgotten) >= 1 and
               _Results(apiForgotten) == [None] * 4, "forget detaches the generation")
        _Check(all(request() is None for request in _Requests(apiForgotten, freshUndo)),
               "the retained window registers a fresh generation before dispatch")
        for api in (apiClosed, apiForgotten, apiRetired, apiSurvivor):
            _Host(api)._stageView = object()
        _PumpUntil(lambda: all(r.status == "ready" for r in _Results(apiSurvivor))
                   and all(r.status == "ready" for r in _Results(apiForgotten))
                   and _Results(apiRetired)[0] is None)
        _PumpExtraTurns()
        _Check(_Results(apiClosed) == _Results(apiRetired) == [None] * 4 and
               old == [stageViewReady.InstallResult("pending")] * 4,
               "retirement clears registrations without mutating snapshots")
        _Check(probes == {apiClosed: 1, apiForgotten: 3, apiRetired: 2, apiSurvivor: 2}
               and Counter(api for api, _, _ in built) == {
                   apiForgotten: 4, apiRetired: 1, apiSurvivor: 4}
               and Counter(notified) == {apiForgotten: 1, apiSurvivor: 1},
               "no stale probes, remaining factories or onReady after retirement")
        _Check(avarDock.GetController(apiForgotten).args == (freshUndo,) and
               avarDock.GetController(apiSurvivor).args == (oldUndo,),
               "fresh generation uses new inputs; survivor is unchanged")
    for window in (windowForgotten, windowRetired, windowSurvivor):
        _Destroy(window)
    print("  ok: close, forget/re-register, and retirement during creation")


class _FilterController(QtCore.QObject):
    """Enough of GizmoController for ViewportHotkeyFilter: the real
    ownership gate, recorded actions."""

    OwnsWindowEvent = gizmoUI.GizmoController.OwnsWindowEvent
    _MainWindow = gizmoUI.GizmoController._MainWindow
    _InMainWindow = gizmoUI.GizmoController._InMainWindow
    HandleHotkey = gizmoUI.GizmoController.HandleHotkey
    _TypingFocus = staticmethod(gizmoUI.GizmoController._TypingFocus)
    _Claim = staticmethod(gizmoUI.GizmoController._Claim)
    _DRAG_KEYS = gizmoUI.GizmoController._DRAG_KEYS

    def __init__(self, usdviewApi):
        super(_FilterController, self).__init__()
        self.usdviewApi = usdviewApi
        self.cleared = 0
        self.flushed = 0
        self.acted = []
        self._visible = True
        self._view = QtWidgets.QWidget(usdviewApi.qMainWindow)
        self._claimedKey = None
        self._drag = None
        self.toolbar = self

    def Sync(self):
        pass

    def FlushReleasedEdit(self):
        self.flushed += 1

    def _ClearHolds(self):
        self.cleared += 1

    def _ReleaseHold(self, event, key):
        self.acted.append(("release", key))
        return False

    def _Act(self, kind, key, handler):
        self.acted.append(("act", key))
        return False

    def _CursorOverView(self):
        return True

    def _ToolKey(self, kind, key):
        return False

    def _DragKey(self, kind, key):
        return False


def TestHotkeyRouting():
    """One window's keys and focus changes never drive another session."""
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    ctrlA = _FilterController(apiA)
    ctrlB = _FilterController(apiB)
    filterA = gizmoUI.ViewportHotkeyFilter(ctrlA)
    filterB = gizmoUI.ViewportHotkeyFilter(ctrlB)
    widgetA = QtWidgets.QWidget(windowA)
    panelB = QtWidgets.QDialog(windowB)
    try:
        deactivate = QtCore.QEvent(QtCore.QEvent.WindowDeactivate)
        for event_filter in (filterA, filterB):
            event_filter.eventFilter(widgetA, deactivate)
        _Check(ctrlA.cleared == 1 and ctrlB.cleared == 0,
               "A's window deactivating clears A's holds only: %d/%d"
               % (ctrlA.cleared, ctrlB.cleared))
        for event_filter in (filterA, filterB):
            event_filter.eventFilter(panelB, deactivate)
        _Check(ctrlA.cleared == 1 and ctrlB.cleared == 1,
               "B's panel deactivating is B's business only")

        press = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_W,
                                QtCore.Qt.NoModifier)
        release = QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W,
                                  QtCore.Qt.NoModifier)
        _Check(filterB.eventFilter(widgetA, press) is False and
               filterB.eventFilter(widgetA, release) is False,
               "B's filter ignores keys in A's window")
        _Check(ctrlB.acted == [],
               "without acting on them, not even a hold release: %r"
               % ctrlB.acted)
        _Check(ctrlB.flushed == 0,
               "B's filter does not drain B's edit for A's input")
        filterA.eventFilter(widgetA, press)
        filterA.eventFilter(widgetA, release)
        _Check(("act", QtCore.Qt.Key_W) in ctrlA.acted and
               ("release", QtCore.Qt.Key_W) in ctrlA.acted,
               "A's filter does act on them: %r" % ctrlA.acted)
        _Check(ctrlA.flushed == 1 and ctrlB.flushed == 0,
               "the owning filter drains before key press, not key release")
        for kind in (QtCore.QEvent.ShortcutOverride,
                     QtCore.QEvent.MouseButtonPress, QtCore.QEvent.Close):
            event = (QtGui.QKeyEvent(kind, QtCore.Qt.Key_W,
                                    QtCore.Qt.NoModifier)
                     if kind == QtCore.QEvent.ShortcutOverride
                     else QtCore.QEvent(kind))
            for event_filter in (filterA, filterB):
                event_filter.eventFilter(panelB, event)
        _Check(ctrlA.flushed == 1 and ctrlB.flushed == 3,
               "B's shortcut, mouse press and close drain only B's edit")
    finally:
        _Destroy(windowA)
        _Destroy(windowB)
    print("  ok: hotkey filters act for their own session only")


class _ShortcutController(object):
    """A controller double with real undo / redo actions and the real
    scoping rule."""

    _ScopeShortcuts = gizmoUI.GizmoController._ScopeShortcuts
    _MainWindow = gizmoUI.GizmoController._MainWindow

    def __init__(self, usdviewApi, *args, **kwargs):
        self.usdviewApi = usdviewApi
        window = usdviewApi.qMainWindow
        self.actions = []
        for name in ("undo", "redo"):
            action = gizmoUI.QtActionWidgets.QAction(name, window)
            action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.actions.append(action)

    def _UndoActions(self):
        return list(self.actions)

    def Contexts(self):
        return set(a.shortcutContext() for a in self.actions)


def TestUndoShortcutScoping():
    """Only the session in front keeps application-wide undo / redo."""
    app = QtCore.Qt.ApplicationShortcut
    win = QtCore.Qt.WindowShortcut
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    _Host(apiA)._stageView = _Host(apiB)._stageView = object()
    with _Patched(gizmoUI, GizmoController=_ShortcutController):
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: windowA)):
            ctrlA = gizmoUI.InstallViewportTools(apiA, None)
            _Check(ctrlA.Contexts() == {app},
                   "alone in the process: application-wide, as ever")
            ctrlB = gizmoUI.InstallViewportTools(apiB, None)
            _Check(ctrlA.Contexts() == {app} and ctrlB.Contexts() == {win},
                   "A in front: A's undo is application-wide, B's is "
                   "window-scoped, so Ctrl+Z is never ambiguous")
        panel = QtWidgets.QDialog(windowB)
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: panel)):
            for controller in (ctrlA, ctrlB):
                controller._ScopeShortcuts()
            _Check(ctrlA.Contexts() == {win} and ctrlB.Contexts() == {app},
                   "focus moved to B's panel: the roles swap")
        unrelated = QtWidgets.QMainWindow()
        with _Patched(QtWidgets.QApplication,
                      activeWindow=staticmethod(lambda: unrelated)):
            for controller in (ctrlA, ctrlB):
                controller._ScopeShortcuts()
            _Check(ctrlA.Contexts() == {win} and ctrlB.Contexts() == {app},
                   "a window no ready gizmo owns changes nothing")
        _Destroy(unrelated)
        _Destroy(windowB)
        ctrlA._ScopeShortcuts()
        _Check(ctrlA.Contexts() == {app},
               "the last session left is application-wide again")
    _Destroy(windowA)
    print("  ok: undo / redo shortcuts scoped to the session in front")


def _TwinStages():
    layer = Sdf.Layer.CreateAnonymous("twin.usda")
    stage = Usd.Stage.Open(layer)
    stage.DefinePrim("/Asset", "Scope")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    prim = stage.DefinePrim("/Asset/Rig/Ctrl", "Xform")
    prim.CreateAttribute("avars:tx", Sdf.ValueTypeNames.Double).Set(0.0)
    first = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionA"))
    second = Usd.Stage.Open(layer, Sdf.Layer.CreateAnonymous("sessionB"))
    return first, second


def TestPublishedReadersPerSession():
    """Each session's reader answers for its own stage, same paths or not."""
    first, second = _TwinStages()
    windowA, apiA = _Session(first)
    windowB, apiB = _Session(second)
    path = Sdf.Path("/Asset/Rig/Ctrl")
    matrixA = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1, 0, 0))
    matrixB = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 2, 0))

    def _Reader(own, matrix):
        return lambda stage, p, time: matrix if stage == own else None

    gizmoMath.SetPublishedControlFrameReader(_Reader(first, matrixA),
                                             session=apiA)
    gizmoMath.SetPublishedControlFrameReader(_Reader(second, matrixB),
                                             session=apiB)
    try:
        time = Usd.TimeCode(1)
        _Check(gizmoMath._ReadPublishedControlFrame(first, path, time)
               == matrixA, "A's stage reads A's published frame")
        _Check(gizmoMath._ReadPublishedControlFrame(second, path, time)
               == matrixB, "B's stage, same path, reads B's")
        _Destroy(windowA)
        _Check(gizmoMath._ReadPublishedControlFrame(first, path, time)
               is None, "A's reader went with A's window")
        _Check(gizmoMath._ReadPublishedControlFrame(second, path, time)
               == matrixB, "B's did not")
    finally:
        gizmoMath.SetPublishedControlFrameReader(None, session=apiB)
        _Destroy(windowB)
    _Check(not gizmoMath._HasPublishedControlFrameReader(),
           "no readers left")
    print("  ok: published-frame readers are per session")


def TestPreviewPerSession():
    """Previews: one channel per session, values per stage."""
    first, second = _TwinStages()
    windowA, apiA = _Session(first)
    windowB, apiB = _Session(second)
    attrPath = Sdf.Path("/Asset/Rig/Ctrl.avars:tx")
    attrA = first.GetAttributeAtPath(attrPath)
    attrB = second.GetAttributeAtPath(attrPath)

    class _Sink(object):
        def __init__(self):
            self.updates = []
            self.ends = 0

        def Begin(self, packed):
            return len(packed.split("\n"))

        def Update(self, values):
            self.updates.append(list(values))
            return True

        def End(self):
            self.ends += 1
            return True

    sinkA, sinkB = _Sink(), _Sink()
    gizmoPreview.SetSink(sinkA, session=apiA)
    gizmoPreview.SetSink(sinkB, session=apiB)
    try:
        writerA = gizmoMath.Writer(first, Usd.TimeCode(1),
                                   gizmoMath.WRITE_ANIMATION)
        writerA.Set(attrA, 5.0)
        _Check(gizmoMath._Previewed(attrA) == 5.0,
               "A's drag previews A's attribute")
        _Check(gizmoMath._Previewed(attrB) is None,
               "and NOT the same path in B's stage")
        _Check(gizmoPreview.Push(writerA.Pending(), session=apiA,
                                 stage=first),
               "A's sample reached A's sink")
        _Check(sinkA.updates == [[5.0]] and sinkB.updates == [],
               "and only A's sink")
        _Check(gizmoPreview.IsPreviewing(apiA) and
               not gizmoPreview.IsPreviewing(apiB),
               "only A is previewing")

        # B previews the same path at the same time.
        gizmoMath.SetPreviewValue(attrPath, 9.0, stage=second)
        _Check(gizmoPreview.Push({attrPath: 9.0}, session=apiB,
                                 stage=second), "B previews too")
        _Check(gizmoMath._Previewed(attrA) == 5.0 and
               gizmoMath._Previewed(attrB) == 9.0,
               "two stages, one path, two values")

        # B's End drops B's values and leaves A's drag alone.
        gizmoPreview.End(session=apiB, stage=second)
        _Check(gizmoMath._Previewed(attrB) is None,
               "B's end dropped B's value")
        _Check(gizmoMath._Previewed(attrA) == 5.0,
               "and A's drag still holds its own")
        _Check(sinkB.ends == 1 and sinkA.ends == 0, "B's sink ended only")
        writerA.Clear()
        gizmoPreview.End(session=apiA, stage=first)
        _Check(gizmoMath.PreviewValues() == {}, "everything is dropped")
    finally:
        gizmoPreview.SetSink(None, session=apiA)
        gizmoPreview.SetSink(None, session=apiB)
        gizmoMath.SetPreviewValues({})
        _Destroy(windowA)
        _Destroy(windowB)
    print("  ok: preview channels per session, values per stage")


def TestMemoScopePerStage():
    """A memo scope opened for one stage never answers another's."""
    first, second = _TwinStages()
    primA = first.GetPrimAtPath("/Asset/Rig/Ctrl")
    primB = second.GetPrimAtPath("/Asset/Rig/Ctrl")
    with gizmoMath.MemoScope(first) as scope:
        _Check(gizmoMath._AmbientScope(primA) is scope,
               "the scope serves its own stage")
        _Check(gizmoMath._AmbientScope(primB) is None,
               "and not the same path in another stage")
        ctxA = gizmoMath._Context(None, Usd.TimeCode(1), primA)
        ctxB = gizmoMath._Context(None, Usd.TimeCode(1), primB)
        _Check(ctxA is not ctxB, "so B gets a context of its own")
    with gizmoMath.MemoScope() as legacy:
        _Check(gizmoMath._AmbientScope(primB) is legacy,
               "a scope with no stage serves any call, as before")
    _Check(gizmoMath._AmbientScope(primA) is None, "scopes close")
    print("  ok: memo scopes are bound to their stage")


def TestPanelInstanceView():
    """`Cls._instance` of a per-session panel class is a view."""

    class _Panel(QtWidgets.QDialog,
                 metaclass=sessionRegistry.PerSessionInstanceMeta(
                     QtWidgets.QDialog)):
        _sessions = sessionRegistry.SessionRegistry("test panels")

        def __init__(self, api):
            super(_Panel, self).__init__(api.qMainWindow)
            self._api = api

        @classmethod
        def GetInstance(cls, api):
            panel = cls._sessions.Get(api)
            if panel is None:
                panel = cls._sessions.Set(api, cls(api))
            return panel

    windowA, apiA = _Session()
    windowB, apiB = _Session()
    panelA = _Panel.GetInstance(apiA)
    _Check(_Panel._instance is panelA, "one session: its panel")
    panelB = _Panel.GetInstance(apiB)
    _Check(panelA is not panelB, "each session has its own panel")
    _Check(_Panel.GetInstance(apiA) is panelA, "and keeps it")
    with _ActiveWindow(panelB):
        _Check(_Panel._instance is panelB,
               "_instance is the active session's panel")
        _Panel._instance = None
        _Check(_Panel._sessions.Get(apiB) is None and
               _Panel._sessions.Get(apiA) is panelA,
               "assigning None forgets the current session's panel only")
    _Destroy(windowA)
    _Check(len(_Panel._sessions) == 0, "A's panel went with its window")
    _Destroy(windowB)
    print("  ok: per-session _instance view")


def TestContainerPerSession():
    """rigExecUsdview files one container per session."""
    import rigExecUsdview

    class _Signal(object):
        def connect(self, slot):
            pass

    class _Registry(object):
        def registerCommandPlugin(self, name, label, callback):
            return (name, callback)

    def _ContainerApi(window):
        api = _Api(window)
        api.dataModel = type("DataModel", (), {})()
        api.dataModel.stage = None
        api.dataModel.signalStageReplaced = _Signal()
        api.dataModel.currentFrameChanged = _Signal()
        return api

    windowA, windowB = QtWidgets.QMainWindow(), QtWidgets.QMainWindow()
    apiA, apiB = _ContainerApi(windowA), _ContainerApi(windowB)
    containerA = rigExecUsdview.RigExecUsdviewContainer()
    containerB = rigExecUsdview.RigExecUsdviewContainer()
    containerA.registerPlugins(_Registry(), apiA)
    containerB.registerPlugins(_Registry(), apiB)
    _Check(rigExecUsdview.ContainerFor(apiA) is containerA and
           rigExecUsdview.ContainerFor(apiB) is containerB,
           "each session finds its own container")
    with _ActiveWindow(windowA):
        _Check(rigExecUsdview._container is containerA,
               "the old module attribute views the active session's")
    with _ActiveWindow(None):
        _Check(rigExecUsdview.ContainerFor() is None,
               "ambiguous without an active window")
    _Destroy(windowA)
    _Check(rigExecUsdview.ContainerFor(apiB) is containerB and
           rigExecUsdview._container is containerB,
           "A's container is dropped with its window; B's stays")
    _Destroy(windowB)
    print("  ok: one RigExec container per session")


def TestRegistryDestroyedSlots():
    """Filing and removing a session repeatedly adds no slots to its window.

    Every Set used to connect a fresh closure to the window's `destroyed`
    signal and Pop never disconnected it, so a panel closed and reopened
    (or an install retried) grew the long-lived window's signal forever.
    """
    class _Signal(object):
        def __init__(self):
            self.slots = []

        def connect(self, slot):
            self.slots.append(slot)

        def disconnect(self, slot):
            self.slots.remove(slot)

        def emit(self):
            for slot in list(self.slots):
                slot()

    class _Window(object):
        def __init__(self):
            self.destroyed = _Signal()

    registry = sessionRegistry.SessionRegistry("test slots")
    window = _Window()
    for _ in range(20):
        registry.Set(window, True)
        registry.Pop(window)
    _Check(len(window.destroyed.slots) == 0 and len(registry) == 0,
           "20 Set/Pop cycles leave no destroyed slot behind (%d)"
           % len(window.destroyed.slots))
    registry.Set(window, "a")
    registry.Set(window, "b")
    _Check(len(window.destroyed.slots) == 1,
           "replacing a live entry reuses its one connection")
    window.destroyed.emit()
    _Check(len(registry) == 0, "and that connection still drops the entry")
    registry.Set(window, "c")
    other = sessionRegistry.SessionRegistry("test slots other")
    other.Set(window, "d")
    _Check(len(window.destroyed.slots) == 3,
           "one slot per live entry (the emitted one stays until Qt drops "
           "it with the dying window)")
    _Check(sessionRegistry.ForgetSession(window) == 2 and
           len(window.destroyed.slots) == 1,
           "ForgetSession disconnects what it forgets")
    registry.Set(window, "e")
    registry.Clear()
    _Check(len(window.destroyed.slots) == 1, "and so does Clear")

    # The same on a real window: its receiver count stays put, and the
    # entry still goes with the window.
    windowQ, apiQ = _Session()
    signature = QtCore.SIGNAL("destroyed(QObject*)")
    # PySide keeps one receiver of its own on an object once any Python
    # slot was connected to it; the baseline is taken after that.
    registry.Set(apiQ, True)
    registry.Pop(apiQ)
    baseline = windowQ.receivers(signature)
    for _ in range(20):
        registry.Set(apiQ, True)
        registry.Pop(apiQ)
    _Check(windowQ.receivers(signature) == baseline,
           "a real window's destroyed signal did not grow (%d -> %d)"
           % (baseline, windowQ.receivers(signature)))
    registry.Set(apiQ, True)
    _Check(windowQ.receivers(signature) == baseline + 1,
           "a filed session is watched once")
    _Destroy(windowQ)
    _Check(len(registry) == 0, "and is dropped with its window")
    print("  ok: registry connections are owned by their entries")


def TestRegistryRemovalHook():
    windowA, apiA = _Session()
    windowB, apiB = _Session()
    windowC, apiC = _Session()
    removed = []

    def _Removed(value):
        removed.append((value, len(registry), registry.Get(apiA)))

    registry = sessionRegistry.SessionRegistry("test removal", _Removed)
    first, replacement = object(), object()
    registry.Set(apiA, first)
    registry.Set(apiA, first)
    _Check(not removed, "setting the identical value must not dispose it")
    registry.Set(apiA, replacement)
    _Check(removed == [(first, 1, replacement)],
           "replacement is visible before the old value is disposed")
    _Check(registry.Pop(apiA) is replacement and
           removed[-1] == (replacement, 0, None) and
           registry.Pop(apiA, "missing") == "missing" and len(removed) == 2,
           "Pop detaches before cleanup, preserves the return, and runs once")

    registry.Set(apiA, "A")
    registry.Set(apiB, "B")
    registry.Clear()
    _Check(set(removed[2:]) == {("A", 0, None), ("B", 0, None)} and
           len(removed) == 4, "Clear detaches all entries before cleanup")
    registry.Set(apiC, "C")
    _Destroy(windowC)
    _Check(removed[4:] == [("C", 0, None)], "destroying a window notifies once")

    key = _Api(None)
    registry.Set(key, "headless")
    del key
    gc.collect()
    _Check(removed[5:] == [("headless", 0, None)], "weak-key collection notifies once")
    registry.Set(apiA, "forgotten")
    _Check(sessionRegistry.ForgetSession(apiA) == 1 and
           removed[6:] == [("forgotten", 0, None)],
           "ForgetSession preserves count and uses the same removal hook")
    _Destroy(windowA)
    _Destroy(windowB)
    _Check(len(removed) == 7, "previously removed entries are not disposed again")
    print("  ok: removal hook for replacement, pop, clear, destruction, and collection")


def TestTouchPoseMarqueeUsesOwnSession():
    """TouchPose's marquee projects through ITS session's gizmo.

    It read gizmoUI's `_controller` view, which answers for the active
    window's session: nothing when that is ambiguous (every control dropped
    from the band), another session's camera when that window is active.
    """
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    touchPoseDir = os.path.join(root, "plugin", "touchPose")
    if touchPoseDir not in sys.path:
        sys.path.insert(0, touchPoseDir)
    import gizmoMarquee
    import touchPoseUI

    stageA = Usd.Stage.CreateInMemory()
    stageB = Usd.Stage.CreateInMemory()
    windowA, apiA = _Session(stageA)
    windowB, apiB = _Session(stageB)
    apiA.frame = apiB.frame = 1.0
    calls = []

    class _Gizmo(object):
        def __init__(self, usdviewApi, *args, **kwargs):
            self.usdviewApi = usdviewApi
            self._solverPosed = ("posed", usdviewApi)

        def _Camera(self):
            return ("camera", self.usdviewApi), ("viewport",), 1.0

    def _ControlsInBand(stage, camera, viewport, frame, band, posed):
        calls.append((stage, camera, posed))
        return ["/ctrl"]

    class _TouchPose(object):
        """Just what _ControlsInBand reads off the controller."""

        def __init__(self, api):
            self._api = api

        def _Stage(self):
            return self._api.stage

    _Host(apiA)._stageView = _Host(apiB)._stageView = object()
    with _Patched(gizmoUI, GizmoController=_Gizmo), \
            _Patched(gizmoMarquee, ControlsInBand=_ControlsInBand):
        gizmoA = gizmoUI.InstallViewportTools(apiA, undoStack=None)
        gizmoB = gizmoUI.InstallViewportTools(apiB, undoStack=None)
        band = touchPoseUI.TouchPoseController._ControlsInBand
        for active in (None, windowA, windowB):
            del calls[:]
            with _ActiveWindow(active):
                picked = band(_TouchPose(apiB), 0, 0, 10, 10)
            _Check(picked == ["/ctrl"],
                   "B's band picks controls whatever window is active "
                   "(%r)" % (active,))
            _Check(calls == [(stageB, ("camera", apiB), ("posed", apiB))],
                   "through B's own camera and posed cache, on B's stage")
        _Check(gizmoA is not gizmoB, "two gizmos")
    _Destroy(windowA)
    _Destroy(windowB)
    print("  ok: TouchPose marquee uses its own session's gizmo")


def main():
    TestRegistry()
    TestGizmoAndViewCubePerSession()
    TestAllInstallersImmediate()
    TestPendingInstallPerSession()
    TestInstallProbeBounds()
    TestInstallFailures()
    TestRetirePendingInstalls()
    TestHotkeyRouting()
    TestUndoShortcutScoping()
    TestPublishedReadersPerSession()
    TestPreviewPerSession()
    TestMemoScopePerStage()
    TestPanelInstanceView()
    TestContainerPerSession()
    TestRegistryDestroyedSlots()
    TestRegistryRemovalHook()
    TestTouchPoseMarqueeUsesOwnSession()
    print("SESSION_STATE_OK (17 groups)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
