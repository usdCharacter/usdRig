#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/editorExtension.py, the one hook a
host application uses to take over input, undo and the document edit target
of the usdview editors.

Covers the do-nothing defaults, Register / Unregister / Current, the guarded
calls (an extension method that raises is warned about once and the caller
gets the fallback), OwnsInput, UndoStack, DocumentEdit, and the routing of
EditorAdded / EditorRemoved / ShowEditor / StageReplaced. Fakes only: no
Qt, no stage, no usdview.

Usage: test_editor_extension.py
"""
import os
import sys
import types

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import editorExtension  # noqa: E402

_PLUGIN = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "plugin")
_TOUCH_POSE = os.path.normpath(os.path.join(_PLUGIN, "touchPose"))
if _TOUCH_POSE not in sys.path:
    sys.path.insert(0, _TOUCH_POSE)

_failures = []
_groups = 0


def _Check(condition, message):
    if not condition:
        _failures.append(message)


class _Recording(editorExtension.EditorExtension):
    """Records every call; answers come from the attributes."""

    def __init__(self):
        self.calls = []
        self.owns = True
        self.stack = object()
        self.edit = None
        self.shown = object()

    def OwnsInput(self, usdviewApi):
        self.calls.append(("OwnsInput", usdviewApi))
        return self.owns

    def UndoStack(self, usdviewApi):
        self.calls.append(("UndoStack", usdviewApi))
        return self.stack

    def DocumentEdit(self, usdviewApi):
        self.calls.append(("DocumentEdit", usdviewApi))
        return self.edit

    def EditorAdded(self, usdviewApi, kind, editor):
        self.calls.append(("EditorAdded", usdviewApi, kind, editor))

    def EditorRemoved(self, usdviewApi, kind, editor):
        self.calls.append(("EditorRemoved", usdviewApi, kind, editor))

    def ShowEditor(self, usdviewApi, kind):
        self.calls.append(("ShowEditor", usdviewApi, kind))
        return self.shown

    def StageReplaced(self, usdviewApi):
        self.calls.append(("StageReplaced", usdviewApi))


class _Raising(editorExtension.EditorExtension):
    def _Boom(self, *args):
        raise ValueError("boom")

    OwnsInput = UndoStack = DocumentEdit = _Boom
    EditorAdded = EditorRemoved = ShowEditor = StageReplaced = _Boom


class _Context(object):
    def __init__(self):
        self.entered = 0
        self.exited = 0

    def __enter__(self):
        self.entered += 1
        return self

    def __exit__(self, *exc):
        self.exited += 1
        return False


class _WarnSink(object):
    """Stands in for pxr.Tf inside editorExtension to count warnings."""

    def __init__(self):
        self.messages = []

    def Warn(self, message):
        self.messages.append(message)


class _BadEnter(object):
    def __enter__(self):
        raise ValueError("enter boom")

    def __exit__(self, *exc):
        raise AssertionError("never entered, never exited")


class _BadExit(object):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        raise ValueError("exit boom")


def _Group(function):
    global _groups
    editorExtension.Unregister()
    function()
    editorExtension.Unregister()
    _groups += 1


def TestDefaults():
    api = object()
    _Check(editorExtension.Current() is None, "Current is None by default")
    _Check(editorExtension.OwnsInput(api) is False, "OwnsInput defaults False")
    _Check(editorExtension.UndoStack(api) is None, "UndoStack defaults None")
    with editorExtension.DocumentEdit(api) as context:
        _Check(context is None, "default DocumentEdit is a null context")
    _Check(editorExtension.ShowEditor(api, editorExtension.GRAPH_EDITOR)
           is None, "ShowEditor defaults None")
    for kind in editorExtension.KINDS:
        _Check(editorExtension.EditorAdded(api, kind, object()) is None,
               "EditorAdded is a no-op for " + kind)
        _Check(editorExtension.EditorRemoved(api, kind, object()) is None,
               "EditorRemoved is a no-op for " + kind)
    _Check(editorExtension.StageReplaced(api) is None,
           "StageReplaced is a no-op")
    base = editorExtension.EditorExtension()
    _Check(base.OwnsInput(api) is False and base.UndoStack(api) is None
           and base.DocumentEdit(api) is None
           and base.ShowEditor(api, editorExtension.GRAPH_EDITOR) is None,
           "the base class answers do-nothing")
    _Check(len(set(editorExtension.KINDS)) == len(editorExtension.KINDS),
           "kinds are distinct")


def TestRegistration():
    first = _Recording()
    editorExtension.Register(first)
    _Check(editorExtension.Current() is first, "Register installs")
    editorExtension.Register(first)
    _Check(editorExtension.Current() is first, "registering twice is a no-op")
    try:
        editorExtension.Register(_Recording())
        _Check(False, "a second extension raises RuntimeError")
    except RuntimeError:
        pass
    _Check(editorExtension.Current() is first, "the first stays registered")

    editorExtension.Unregister(_Recording())
    _Check(editorExtension.Current() is first,
           "Unregister of another object is ignored")
    editorExtension.Unregister(first)
    _Check(editorExtension.Current() is None, "Unregister(obj) removes")

    editorExtension.Register(first)
    editorExtension.Unregister()
    _Check(editorExtension.Current() is None, "Unregister() removes any")
    editorExtension.Unregister()
    _Check(editorExtension.Current() is None, "Unregister() twice is fine")

    incomplete = types.SimpleNamespace(OwnsInput=lambda api: False)
    try:
        editorExtension.Register(incomplete)
        _Check(False, "a missing method raises TypeError")
    except TypeError as error:
        _Check("UndoStack" in str(error), "the TypeError names what is missing")
    _Check(editorExtension.Current() is None, "a rejected object is not kept")

    duck = types.SimpleNamespace(
        **{name: (lambda *a: None) for name in
           editorExtension.EditorExtension.METHODS})
    editorExtension.Register(duck)
    _Check(editorExtension.Current() is duck, "a duck-typed object registers")


def TestOwnsInputUndoDocument():
    api = object()
    ext = _Recording()
    editorExtension.Register(ext)

    _Check(editorExtension.OwnsInput(api) is True, "OwnsInput forwards True")
    ext.owns = 0
    _Check(editorExtension.OwnsInput(api) is False,
           "OwnsInput coerces to bool")
    _Check(editorExtension.UndoStack(api) is ext.stack,
           "UndoStack returns the extension's stack")
    ext.stack = None
    _Check(editorExtension.UndoStack(api) is None,
           "UndoStack None means RigExec's own")

    _Check(ext.calls[0] == ("OwnsInput", api), "OwnsInput gets the api")

    with editorExtension.DocumentEdit(api) as context:
        _Check(context is None, "a None answer becomes a null context")
    ext.edit = _Context()
    with editorExtension.DocumentEdit(api) as context:
        _Check(context is ext.edit and ext.edit.entered == 1,
               "DocumentEdit enters the extension's context")
    _Check(ext.edit.exited == 1, "DocumentEdit exits it")


def TestRouting():
    api = object()
    ext = _Recording()
    editorExtension.Register(ext)
    editor = object()
    editorExtension.EditorAdded(api, editorExtension.GIZMO, editor)
    editorExtension.EditorAdded(api, editorExtension.GIZMO, editor)
    editorExtension.EditorRemoved(api, editorExtension.MARKING_MENU, editor)
    shown = editorExtension.ShowEditor(api, editorExtension.GRAPH_EDITOR)
    editorExtension.StageReplaced(api)
    _Check(shown is ext.shown, "ShowEditor returns the host's widget")
    _Check(ext.calls == [
        ("EditorAdded", api, editorExtension.GIZMO, editor),
        ("EditorAdded", api, editorExtension.GIZMO, editor),
        ("EditorRemoved", api, editorExtension.MARKING_MENU, editor),
        ("ShowEditor", api, editorExtension.GRAPH_EDITOR),
        ("StageReplaced", api)], "calls route with their arguments: %r"
        % (ext.calls,))

    other = object()
    editorExtension.EditorAdded(other, editorExtension.TOUCH_POSE, editor)
    _Check(ext.calls[-1] == ("EditorAdded", other,
                             editorExtension.TOUCH_POSE, editor),
           "the api of each call is passed through")

    editorExtension.Unregister()
    count = len(ext.calls)
    editorExtension.EditorAdded(api, editorExtension.GIZMO, editor)
    editorExtension.StageReplaced(api)
    _Check(len(ext.calls) == count, "nothing routes after Unregister")


def TestGuardedCalls():
    api = object()
    sink = _WarnSink()
    real = editorExtension.Tf
    editorExtension.Tf = sink
    try:
        editorExtension.Register(_Raising())
        for _ in range(3):
            _Check(editorExtension.OwnsInput(api) is False,
                   "a raising OwnsInput falls back to False")
        _Check(len(sink.messages) == 1, "warned once per method and error: %r"
               % (sink.messages,))
        _Check("OwnsInput" in sink.messages[0] and "boom" in sink.messages[0],
               "the warning names the method and the error")
        _Check(editorExtension.UndoStack(api) is None,
               "a raising UndoStack falls back to None")
        with editorExtension.DocumentEdit(api) as context:
            _Check(context is None, "a raising DocumentEdit gives a null context")
        _Check(editorExtension.ShowEditor(api, editorExtension.GRAPH_EDITOR)
               is None, "a raising ShowEditor falls back to None")
        editorExtension.EditorAdded(api, editorExtension.GIZMO, object())
        editorExtension.EditorRemoved(api, editorExtension.GIZMO, object())
        editorExtension.StageReplaced(api)
        _Check(len(sink.messages) == 7,
               "one warning for each of the 7 methods: %d" % len(sink.messages))

        editorExtension.Unregister()
        editorExtension.Register(_Raising())
        editorExtension.OwnsInput(api)
        _Check(len(sink.messages) == 8, "Unregister resets the warned set")

        class _Mixed(_Recording):
            def OwnsInput(self, usdviewApi):
                raise ValueError("first")

        editorExtension.Unregister()
        ext = _Mixed()
        editorExtension.Register(ext)
        before = len(sink.messages)
        editorExtension.OwnsInput(api)
        editorExtension.EditorAdded(api, editorExtension.GIZMO, None)
        _Check(len(sink.messages) == before + 1
               and ext.calls[-1][0] == "EditorAdded",
               "a failing method does not disturb the others")
    finally:
        editorExtension.Tf = real


def TestGuardedContext():
    api = object()
    sink = _WarnSink()
    real = editorExtension.Tf
    editorExtension.Tf = sink
    try:
        class _Ext(_Recording):
            context = None

            def DocumentEdit(self, usdviewApi):
                return self.context

        ext = _Ext()
        editorExtension.Register(ext)
        ext.context = _BadEnter()
        for _ in range(2):
            ran = False
            with editorExtension.DocumentEdit(api) as context:
                ran = True
                _Check(context is None, "a raising __enter__ gives a null context")
            _Check(ran, "the body runs after a raising __enter__")
        _Check(len(sink.messages) == 1 and "__enter__" in sink.messages[0],
               "a raising __enter__ warns once: %r" % (sink.messages,))

        ext.context = _BadExit()
        with editorExtension.DocumentEdit(api):
            pass
        with editorExtension.DocumentEdit(api):
            pass
        _Check(len(sink.messages) == 2 and "__exit__" in sink.messages[1],
               "a raising __exit__ warns once: %r" % (sink.messages,))
        try:
            with editorExtension.DocumentEdit(api):
                raise KeyError("body")
        except KeyError:
            pass
        else:
            _Check(False, "the body's exception propagates past a bad __exit__")
        _Check(len(sink.messages) == 2, "no repeat warning")
    finally:
        editorExtension.Tf = real


def TestFallbackWithoutWarnSink():
    # The real Tf.Warn path must not raise either.
    editorExtension.Register(_Raising())
    _Check(editorExtension.OwnsInput(None) is False,
           "the guarded call survives a real Tf.Warn")


# Public methods the editors expose so a host never reaches a private member.
# One table: a rename shows up here.
_SURFACES = (
    ("gizmoUI", "GizmoController", (
        "InstallsFilters", "HasView", "HasTarget", "ForwardViewEvent",
        "HandlePress", "IsMarqueePending", "HandleMove", "HandleRelease",
        "HandleHover", "HandleKeyRelease", "HandleDragKey", "HandleToolKey",
        "CancelGesture", "ClearHolds", "ReleaseToolbarRow",
        "IsToolbarRowReleased", "IsVisible", "SetVisible", "IsDragging",
        "Tool", "SetTool", "Orientation", "GroupPivot", "Status",
        "FlushReleasedEdit")),
    ("gizmoUI", "ViewportToolbar", (
        "SetToolButtonsVisible", "ToolButtonsVisible", "SetSnapHoldPrefix")),
    ("markingMenuUI", "ControlMarkingMenu", (
        "InstallsFilters", "HasView", "HandleViewEvent", "OpenForClicking",
        "HoveredControls", "SelectedControls", "Targets", "Open", "IsOpen",
        "Cancel")),
    ("pickerHoverUI", "HoverPickerController", (
        "InstallsFilters", "IsShown", "HandleViewEvent", "CancelGesture",
        "ClearHover")),
    ("touchPoseUI", "TouchPoseController", (
        "IsInteractive", "GizmoOwnsAt", "HandleViewEvent", "IsPressPending",
        "CancelGesture", "LeaveView")),
    ("rigExecUsdview", "RigExecUsdviewContainer", (
        "ReadPublishedControlFrame", "ReadPublishedGeneration")),
    ("gizmoUI", "ViewportStatusBar", ("FullText",)),
)


def _Import(module):
    plugin = os.path.normpath(os.path.join(_PLUGIN, "rigExecUsdview"))
    if plugin not in sys.path:
        sys.path.insert(0, plugin)
    return __import__(module)


def TestContainer():
    rigExecUsdview = _Import("rigExecUsdview")
    import rigExecUndo
    Container = rigExecUsdview.RigExecUsdviewContainer
    api = object()
    names = ("RIGEXEC_HISTORY", "RIGEXEC_TOOLMANAGER")
    saved = {name: os.environ.get(name) for name in names}

    def Make():
        container = Container.__new__(Container)
        container._api = api
        container._undoStack = None
        return container

    try:
        for name in names:
            os.environ[name] = "1"
        plain = Make()._UndoStack()
        _Check(type(plain) is rigExecUndo.UndoStack,
               "no extension: a plain rigExecUndo.UndoStack, got %r"
               % (type(plain),))

        ext = _Recording()
        editorExtension.Register(ext)
        container = Make()
        stack = container._UndoStack()
        _Check(stack is ext.stack, "an extension's stack is used")
        _Check(container._UndoStack() is stack, "the stack is created once")
        _Check(ext.calls == [("UndoStack", api)],
               "the extension is asked once with the api: %r" % (ext.calls,))
        editorExtension.Unregister()

        ext = _Recording()
        ext.stack = None
        editorExtension.Register(ext)
        _Check(type(Make()._UndoStack()) is rigExecUndo.UndoStack,
               "an extension answering None leaves RigExec's own stack")
    finally:
        for name, value in saved.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

    container = Make()
    container._active = False
    container._lib = None
    container._cachedStage = None
    _Check(container.ReadPublishedGeneration(object()) is None,
           "ReadPublishedGeneration is None when inactive")
    _Check(container.ReadPublishedControlFrame(object(), "/a", 0.0) is None,
           "ReadPublishedControlFrame is None when inactive")


def TestSurfaces():
    for module, className, methods in _SURFACES:
        cls = getattr(_Import(module), className, None)
        _Check(cls is not None, "%s.%s exists" % (module, className))
        if cls is None:
            continue
        for method in methods:
            _Check(callable(getattr(cls, method, None)),
                   "%s.%s.%s exists" % (module, className, method))
    gizmoUI = _Import("gizmoUI")
    # Attributes set in __init__, not class members.
    import inspect
    _Check("toolChangedHooks" in inspect.getsource(gizmoUI.GizmoController),
           "GizmoController keeps toolChangedHooks")
    _Check("statusListeners" in inspect.getsource(gizmoUI.ViewportStatusBar),
           "ViewportStatusBar keeps statusListeners")
    _Check(callable(getattr(gizmoUI, "GetController", None))
           and callable(getattr(gizmoUI, "StageView", None)),
           "gizmoUI.GetController and StageView exist")
    rigExecUsdview = _Import("rigExecUsdview")
    _Check(callable(rigExecUsdview.ContainerFor)
           and callable(rigExecUsdview.AddToRigExecMenu),
           "ContainerFor and AddToRigExecMenu exist")


def TestDelegation():
    gizmoUI = _Import("gizmoUI")
    markingMenuUI = _Import("markingMenuUI")
    Gizmo = gizmoUI.GizmoController

    calls = []
    standIn = types.SimpleNamespace(
        _keyModifiers=None,
        _ToolKey=lambda key: calls.append(("ToolKey", key, standIn._keyModifiers)))
    result = Gizmo.HandleToolKey(standIn, 81, "MODS")
    _Check(standIn._keyModifiers == "MODS", "HandleToolKey sets the modifiers")
    _Check(calls == [("ToolKey", 81, "MODS")],
           "HandleToolKey calls _ToolKey after setting them: %r" % (calls,))
    _Check(result is False, "HandleToolKey returns bool of the handler")

    released = []
    event = object()
    standIn = types.SimpleNamespace(
        _claimedKey=5,
        _ReleaseHold=lambda e, key: released.append((e, key)) or True)
    _Check(Gizmo.HandleKeyRelease(standIn, event, 7) is True
           and standIn._claimedKey == 5,
           "a release of another key keeps the claim")
    _Check(Gizmo.HandleKeyRelease(standIn, event, 5) is True
           and standIn._claimedKey is None,
           "a release of the claimed key clears it")
    _Check(released == [(event, 7), (event, 5)],
           "both releases reach _ReleaseHold: %r" % (released,))

    log = []
    standIn = types.SimpleNamespace(
        _drag=object(), _marqueeAt=(1, 2), _marqueeBand="b", _marqueeWas=(3,),
        IsDragging=lambda: True,
        _AbortDrag=lambda: log.append("abort"),
        _HideBand=lambda: log.append("hide"))
    Gizmo.CancelGesture(standIn)
    _Check(log == ["abort", "hide"], "CancelGesture aborts then hides: %r" % log)
    _Check(standIn._marqueeAt is None and standIn._marqueeBand is None
           and standIn._marqueeWas == (), "CancelGesture clears the marquee")

    log = []
    standIn.IsDragging = lambda: False
    Gizmo.CancelGesture(standIn)
    _Check(log == [], "CancelGesture with nothing pending does nothing")

    log = []
    standIn = types.SimpleNamespace(
        _marqueeAt=(1, 2), _marqueeBand=None, _marqueeWas=(),
        IsDragging=lambda: False,
        _AbortDrag=lambda: log.append("abort"),
        _HideBand=lambda: log.append("hide"))
    Gizmo.CancelGesture(standIn)
    _Check(log == ["hide"] and standIn._marqueeAt is None,
           "a pending marquee alone is dropped without an abort: %r" % log)

    # ControlMarkingMenu.OpenForClicking
    Menu = markingMenuUI.ControlMarkingMenu
    log = []
    handled = []

    class _Gesture(object):
        def Release(self, x, y):
            log.append(("Release", x, y))
            return "outcome"

    standIn = types.SimpleNamespace(
        _gesture=None, _pressed=True,
        Open=lambda point, controls: log.append(("Open", point, controls)),
        _Handle=lambda outcome: handled.append(outcome),
        IsOpen=lambda: standIn._gesture is not None)
    _Check(Menu.OpenForClicking(standIn, (4, 5), ["c"]) is False
           and standIn._pressed is True and not handled,
           "OpenForClicking leaves a menu that did not open alone")
    log[:] = []
    standIn.Open = lambda point, controls: (
        log.append(("Open", point, controls)),
        setattr(standIn, "_gesture", _Gesture()))
    _Check(Menu.OpenForClicking(standIn, (4, 5), ["c"]) is True,
           "OpenForClicking returns IsOpen")
    _Check(standIn._pressed is False, "the press is marked released")
    _Check(log == [("Open", (4, 5), ["c"]), ("Release", 4, 5)]
           and handled == ["outcome"],
           "the gesture is released at the point and its outcome handled: %r"
           % (log,))


def main():
    for group in (TestDefaults, TestRegistration, TestOwnsInputUndoDocument,
                  TestRouting, TestGuardedCalls, TestGuardedContext,
                  TestFallbackWithoutWarnSink,
                  TestContainer, TestSurfaces, TestDelegation):
        _Group(group)
    if _failures:
        for message in _failures:
            print("FAIL:", message)
        return 1
    print("EDITOR_EXTENSION_OK (%d groups)" % _groups)
    return 0


if __name__ == "__main__":
    sys.exit(main())
