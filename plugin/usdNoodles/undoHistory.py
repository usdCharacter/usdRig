#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Stage-scoped graph history, owned by the views displaying that stage."""

from weakref import WeakValueDictionary

from pxr import Usd


_histories = WeakValueDictionary()


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

    def push(self, description, do_func, undo_func):
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
        self._replay(view, self._undo, self._redo, 2)

    def redo(self, view):
        self._replay(view, self._redo, self._undo, 1)

    def canUndo(self):
        return bool(self._undo)

    def canRedo(self):
        return bool(self._redo)

    def undoDescription(self):
        return self._undo[-1][0] if self._undo else ""

    def redoDescription(self):
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
