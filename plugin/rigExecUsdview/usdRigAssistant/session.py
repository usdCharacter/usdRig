"""The usdview session the assistant works in: its stage, time, selection,
undo stack and viewport. Every call runs on usdview's main thread (the
toolbox sees to that)."""
import logging
import os

from pxr import Sdf, Usd

import rigExecUndo

from . import host

logger = logging.getLogger(host.LOGGER_NAME)


class GroupEdit(object):
    """Several rigExecUndo edits as one undo step: undone last first.

    The group replays as one host boundary: every layer is checked before
    anything is written, and if a restore still fails the ones already made
    are put back, so neither the stage nor the history cursor moves.
    """

    def __init__(self, label, edits):
        self.label = label
        self.edits = list(edits)

    def _Plan(self, direction):
        useAfter = direction == 'Redo'
        members = self.edits if useAfter else reversed(self.edits)
        plan = []
        for edit in members:
            plan.extend(edit.Snapshots(useAfter))
        return plan

    def Undo(self):
        rigExecUndo.ApplyPlan(self._Plan('Undo'))

    def Redo(self):
        rigExecUndo.ApplyPlan(self._Plan('Redo'))


def snapshot(layer):
    copy = Sdf.Layer.CreateAnonymous('assistant_snapshot')
    copy.TransferContent(layer)
    return copy


def stage_layers(stage):
    """Every layer a stage reads: the layers it uses, its session layer and
    its edit target, each once."""
    if stage is None:
        return []
    try:
        layers = list(stage.GetUsedLayers())
    except Exception:
        logger.debug('Could not list the used layers', exc_info=True)
        layers = []
    for extra in (stage.GetSessionLayer(), stage.GetEditTarget().GetLayer()):
        if extra is not None:
            layers.append(extra)
    seen = {}
    for layer in layers:
        if layer is not None and not layer.expired:
            seen[layer] = None
    return list(seen)


def layer_baseline(stage):
    """A content copy of every layer the stage reads now, keyed by layer.

    Keyed by the layer handle and not its identifier: the copy has to go
    back into this very layer, and an anonymous identifier only carries the
    address of a layer that may already be gone. A layer the code brings in
    afterwards has no entry here, so there is nothing to restore it to."""
    return {layer: snapshot(layer) for layer in stage_layers(stage)}


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
