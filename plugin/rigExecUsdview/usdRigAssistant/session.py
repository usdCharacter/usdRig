"""The usdview session the assistant works in: its stage, time, selection,
undo stack and viewport. Every call runs on usdview's main thread (the
toolbox sees to that)."""
import logging
import os

from pxr import Sdf, Usd

from . import host

logger = logging.getLogger(host.LOGGER_NAME)


class GroupEdit(object):
    """Several rigExecUndo edits as one undo step: undone last first."""

    def __init__(self, label, edits):
        self.label = label
        self.edits = list(edits)

    def Undo(self):
        for edit in reversed(self.edits):
            edit.Undo()

    def Redo(self):
        for edit in self.edits:
            edit.Redo()


class LayerSnapshotEdit(object):
    """Undo for code the assistant ran: the whole layer before and after,
    since there is no telling which specs it touched."""

    def __init__(self, label, layer, before, after):
        self.label = label
        self.layer = layer
        self.before = before
        self.after = after

    def _restore(self, source):
        if self.layer is None or self.layer.expired:
            return
        self.layer.TransferContent(source)

    def Undo(self):
        self._restore(self.before)

    def Redo(self):
        self._restore(self.after)


def snapshot(layer):
    copy = Sdf.Layer.CreateAnonymous('assistant_snapshot')
    copy.TransferContent(layer)
    return copy


def same_content(a, b):
    return a.ExportToString() == b.ExportToString()


class Session(object):
    """What the tools reach in usdview, through its usdviewApi."""

    def __init__(self, api, container=None):
        self.api = api
        self.container = container

    # The stage
    @property
    def stage(self):
        return self.api.stage

    def stage_file(self):
        stage = self.stage
        if stage is None:
            return None
        return stage.GetRootLayer().realPath or None

    def stage_folder(self):
        path = self.stage_file()
        return os.path.dirname(path) if path else None

    @property
    def app(self):
        return getattr(self.api, '_UsdviewApi__appController', None)

    # Time
    def time(self):
        frame = self.api.frame
        return frame if isinstance(frame, Usd.TimeCode) else Usd.TimeCode(
            float(frame))

    def set_frame(self, frame):
        app = self.app
        if app is None:
            raise RuntimeError('usdview has no app controller to set the '
                               'frame with')
        app.setFrame(float(frame))
        return self.time()

    # Selection
    def selected_prims(self):
        return [p for p in (self.api.selectedPrims or [])
                if p and p.IsValid() and not p.IsPseudoRoot()]

    def select(self, paths, frame=False):
        stage = self.stage
        prims = [stage.GetPrimAtPath(p) for p in paths]
        prims = [p for p in prims if p]
        selection = self.api.dataModel.selection
        with selection.batchPrimChanges:
            selection.clearPrims()
            for prim in prims:
                selection.addPrim(prim)
        if frame and prims:
            self.frame_selection()
        return [str(p.GetPath()) for p in prims]

    def frame_selection(self):
        from pxr.Usdviewq.qt import QtGui, QtWidgets
        main = self.api.qMainWindow
        action_type = getattr(QtGui, 'QAction', None) or QtWidgets.QAction
        action = main.findChild(action_type, 'actionFrame_Selected')
        if action is not None and action.isEnabled():
            action.trigger()
            return True
        return False

    # Undo
    def undo_stack(self):
        if self.container is not None:
            return self.container._UndoStack()
        return None

    # The viewport
    def grab_viewport(self, path):
        image = self.api.GrabViewportShot()
        if image is None or image.isNull():
            raise RuntimeError('The viewport gave no picture')
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        if not image.save(path):
            raise RuntimeError('Could not save {}'.format(path))
        return image.width(), image.height()

    def update_viewport(self):
        try:
            self.api.UpdateViewport()
        except Exception:
            logger.debug('Viewport update failed', exc_info=True)

    # The live evaluator
    def imaging(self):
        if self.container is None:
            return None
        try:
            return self.container.ImagingHandle()
        except Exception:
            logger.debug('No imaging handle', exc_info=True)
            return None

    def rig_active(self):
        if self.container is None:
            return False
        try:
            return bool(self.container.IsRigActive())
        except Exception:
            return False
