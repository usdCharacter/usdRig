#
# editorExtension -- the one hook through which a host application takes
# over input, undo and the document edit target of RigExec's usdview
# editors (docs/specs/editor-extensions.md).
#
# One extension per process. Without one, every function below returns its
# do-nothing answer and every editor runs on its own exactly as before.
# Calls run on the Qt main thread. Every call into the extension, and the
# DocumentEdit context's __enter__ and __exit__, is guarded: one that
# raises is warned about once per method and error type, and the editor
# falls back to its own behaviour. The exception is input: an editor built
# after OwnsInput answered True has no Qt filters of its own, so if its
# EditorAdded then raises (a warning is posted) that editor stays without
# input. Only the extension's methods are guarded, not that dependence.
#
import contextlib

from pxr import Tf

# Kinds passed to EditorAdded / EditorRemoved, and what `editor` is.
GIZMO = "gizmo"                            # gizmoUI.GizmoController
MARKING_MENU = "markingMenu"               # markingMenuUI.ControlMarkingMenu
HOVER_PICKER = "hoverPicker"               # pickerHoverUI.HoverPickerController
HOVER_PICKER_TOGGLE = "hoverPickerToggle"  # fn(): shows / hides the hover picker
GRAPH_EDITOR = "graphEditor"               # graphEditorUI.GraphEditorPanel
TOUCH_POSE = "touchPose"                   # touchPoseUI.TouchPoseController
TOUCH_POSE_VOICE = "touchPoseVoice"        # touchPoseVoice.TouchPoseVoice
KINDS = (GIZMO, MARKING_MENU, HOVER_PICKER, HOVER_PICKER_TOGGLE,
         GRAPH_EDITOR, TOUCH_POSE, TOUCH_POSE_VOICE)


class EditorExtension(object):
    """The do-nothing answers. Subclass and override what the host takes over."""

    METHODS = ("OwnsInput", "UndoStack", "DocumentEdit", "EditorAdded",
               "EditorRemoved", "ShowEditor", "StageReplaced")

    def OwnsInput(self, usdviewApi):
        """True: the session's editors install no Qt event filters and no
        Undo / Redo shortcuts; the host routes input to their public
        handlers. Must answer the same for a session's lifetime."""
        del usdviewApi
        return False

    def UndoStack(self, usdviewApi):
        """The session's undo stack (rigExecUndo.UndoStack's protocol), or
        None for RigExec's own."""
        del usdviewApi
        return None

    def DocumentEdit(self, usdviewApi):
        """A context manager selecting the layer for an edit RigExec
        authors outside the host's own dispatch (the gizmo's deferred
        commit), or None."""
        del usdviewApi
        return None

    def EditorAdded(self, usdviewApi, kind, editor):
        """`editor` of `kind` exists now. May repeat for one editor."""
        del usdviewApi, kind, editor

    def EditorRemoved(self, usdviewApi, kind, editor):
        del usdviewApi, kind, editor

    def ShowEditor(self, usdviewApi, kind):
        """Show the host's own home of editor `kind`; its widget, or None
        for RigExec's window. Only GRAPH_EDITOR is asked."""
        del usdviewApi, kind
        return None

    def StageReplaced(self, usdviewApi):
        """After RigExec handled a stage replacement (the stage may be None)."""
        del usdviewApi


_extension = None
_warned = set()


def Register(extension):
    """Install `extension` for this process. TypeError when it lacks a
    method of EditorExtension.METHODS; RuntimeError when another is
    registered. Registering the same object again is a no-op. Register
    before the first stage opens (a container's registerPlugins):
    editors consult the hook as they are built."""
    global _extension
    missing = [name for name in EditorExtension.METHODS
               if not callable(getattr(extension, name, None))]
    if missing:
        raise TypeError("an editor extension needs %s" % ", ".join(missing))
    if _extension is not None and _extension is not extension:
        raise RuntimeError("an editor extension is already registered: %r"
                           % (_extension,))
    _extension = extension


def Unregister(extension=None):
    """Remove `extension` (None: whichever is registered)."""
    global _extension
    if extension is None or extension is _extension:
        _extension = None
        _warned.clear()


def Current():
    return _extension


def _Warn(name, error):
    key = (name, type(error).__name__)
    if key not in _warned:
        _warned.add(key)
        Tf.Warn("rigExecUsdview: editor extension %s failed: %s"
                % (name, error))


def _Call(name, fallback, *args):
    extension = _extension
    if extension is None:
        return fallback
    try:
        return getattr(extension, name)(*args)
    except Exception as error:  # noqa: BLE001 -- never into a Qt handler
        _Warn(name, error)
        return fallback


class _GuardedContext(object):
    """Wraps the extension's context: a raising __enter__ becomes a null
    context, a raising __exit__ is warned and never masks the body's own
    exception."""

    def __init__(self, context):
        self._context = context
        self._entered = False

    def __enter__(self):
        try:
            value = self._context.__enter__()
        except Exception as error:  # noqa: BLE001
            _Warn("DocumentEdit.__enter__", error)
            return None
        self._entered = True
        return value

    def __exit__(self, *exc):
        if not self._entered:
            return False
        try:
            return bool(self._context.__exit__(*exc))
        except Exception as error:  # noqa: BLE001
            _Warn("DocumentEdit.__exit__", error)
            return False


def OwnsInput(usdviewApi):
    """THE switch editors consult before installing filters or shortcuts."""
    return bool(_Call("OwnsInput", False, usdviewApi))


def UndoStack(usdviewApi):
    return _Call("UndoStack", None, usdviewApi)


def DocumentEdit(usdviewApi):
    """The extension's document edit context, or a null context."""
    context = _Call("DocumentEdit", None, usdviewApi)
    if context is None:
        return contextlib.nullcontext()
    return _GuardedContext(context)


def EditorAdded(usdviewApi, kind, editor):
    _Call("EditorAdded", None, usdviewApi, kind, editor)


def EditorRemoved(usdviewApi, kind, editor):
    _Call("EditorRemoved", None, usdviewApi, kind, editor)


def ShowEditor(usdviewApi, kind):
    return _Call("ShowEditor", None, usdviewApi, kind)


def StageReplaced(usdviewApi):
    _Call("StageReplaced", None, usdviewApi)
