#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Stage-scoped graph history, owned by the views displaying that stage."""

from weakref import WeakSet, WeakValueDictionary, ref

from pxr import Usd


_histories = WeakValueDictionary()


_history_backend = None


def set_history_backend(fn):
    """Route every stage's graph history through `fn(stage)` -> backend or
    None (None: Noodles' own lists). A backend provides Push(description,
    do_func, undo_func, edit_target, view_ref), Undo(), Redo(), CanUndo(),
    CanRedo(), UndoText(), RedoText(), AddReloadListener(fn) -> token and
    IsReplaying(); fn must return the same backend object while one history
    serves the stage. Passing None restores Noodles' own history."""
    global _history_backend
    _history_backend = fn


def get_history_backend():
    return _history_backend


def _backend_for(stage):
    fn = _history_backend
    if fn is None or stage is None:
        return None
    try:
        return fn(stage)
    except Exception:
        return None


def _guard_during_replay(view, backend):
    """While an external history backend replays an entry (undo / redo), the
    view's USD notice handling waits: a replay restores specs (a renamed prim
    comes back as delete + create, not as a namespace move), and following
    that notice by notice would drop nodes. The graph is reloaded from USD
    after the replay instead (the reload listener). Only a view served by a
    backend is guarded; any other view keeps its handler untouched."""
    if getattr(view, "_historyReplayGuard", False):
        return
    handler = getattr(view, "_handleUsdChanges", None)
    if handler is None:
        return

    def _Guarded(*args, **kwargs):
        try:
            replaying = backend.IsReplaying()
        except Exception:
            replaying = False
        if replaying:
            return None
        return handler(*args, **kwargs)

    view._handleUsdChanges = _Guarded
    view._historyReplayGuard = True


def history_for_stage(stage):
    """Share history by USD stage identity, including its session layer.

    The registry holds histories weakly. Views keep their history alive until
    they close or bind another stage; commands must not capture a view.
    """
    if stage is None:
        return None
    history = _histories.get(stage)
    if history is None:
        history = UndoHistory(stage)
        _histories[stage] = history
    return history


class UndoHistory:
    """UI-thread history whose callbacks receive the view invoking replay."""

    def __init__(self, stage):
        self.stage = stage
        self._undo = []
        self._redo = []
        self._maxDepth = 100
        self._replaying = False
        # External backend state (unused without a backend): the views that
        # pushed or replayed here, reloaded after the backend replays.
        self._views = WeakSet()
        self._listenerBackend = None
        self._listener = None

    def _backend(self, view=None):
        backend = _backend_for(self.stage)
        if backend is None:
            return None
        if view is not None:
            self._views.add(view)
            _guard_during_replay(view, backend)
        if self._listenerBackend is None or \
                self._listenerBackend is not backend:
            views = self._views
            pending = {"posted": False}

            def _ReloadNow():
                pending["posted"] = False
                for live in list(views):
                    reload = getattr(live, "_reloadCurrentGraph", None)
                    if reload is not None:
                        reload()

            def _Reload():
                # One event-loop turn after the backend's replay, so a host
                # that rebuilds its UI after a stage change is settled
                # first. Synchronous without a QApplication (headless).
                try:
                    from pxr.Usdviewq.qt import QtCore, QtWidgets
                    deferred = QtWidgets.QApplication.instance() is not None
                except Exception:
                    deferred = False
                if not deferred:
                    _ReloadNow()
                elif not pending["posted"]:
                    pending["posted"] = True
                    QtCore.QTimer.singleShot(0, _ReloadNow)
            self._listener = backend.AddReloadListener(_Reload)
            self._listenerBackend = backend
        return backend

    def push(self, description, do_func, undo_func, view=None):
        backend = self._backend(view)
        if backend is not None:
            viewRef = ref(view) if view is not None else (lambda: None)
            backend.Push(description, do_func, undo_func,
                        self.stage.GetEditTarget(), viewRef)
            return
        if self._replaying:
            raise RuntimeError("Cannot record a command during undo or redo")
        self._redo.clear()
        self._undo.append(
            (description, do_func, undo_func, self.stage.GetEditTarget()))
        self._trim()

    def _replay(self, view, source, destination, callback_index):
        if view.nodeGraph.getStage() != self.stage:
            raise RuntimeError("Undo history belongs to a different stage")
        if self._replaying:
            raise RuntimeError("Undo and redo cannot be nested")
        if not source:
            return
        command = source[-1]
        self._replaying = True
        try:
            with Usd.EditContext(self.stage, command[3]):
                command[callback_index](view)
        finally:
            self._replaying = False
        # A failed callback retains its entry so it can be retried.
        destination.append(command)
        source.pop()

    def undo(self, view):
        backend = self._backend(view)
        if backend is not None:
            backend.Undo()
            return
        self._replay(view, self._undo, self._redo, 2)

    def redo(self, view):
        backend = self._backend(view)
        if backend is not None:
            backend.Redo()
            return
        self._replay(view, self._redo, self._undo, 1)

    def bridged(self):
        """True while an external history backend serves this stage."""
        return self._backend() is not None

    def canUndo(self):
        backend = self._backend()
        if backend is not None:
            return backend.CanUndo()
        return bool(self._undo)

    def canRedo(self):
        backend = self._backend()
        if backend is not None:
            return backend.CanRedo()
        return bool(self._redo)

    def undoDescription(self):
        backend = self._backend()
        if backend is not None:
            return backend.UndoText()
        return self._undo[-1][0] if self._undo else ""

    def redoDescription(self):
        backend = self._backend()
        if backend is not None:
            return backend.RedoText()
        return self._redo[-1][0] if self._redo else ""

    def clear(self):
        if self._replaying:
            raise RuntimeError("Cannot clear history during undo or redo")
        self._undo.clear()
        self._redo.clear()

    def setMaxStackDepth(self, depth):
        if self._replaying:
            raise RuntimeError("Cannot resize history during undo or redo")
        if depth < 0:
            raise ValueError("Undo depth cannot be negative")
        self._maxDepth = depth
        self._trim()

    def _trim(self):
        excess = len(self._undo) - self._maxDepth
        if excess > 0:
            del self._undo[:excess]
