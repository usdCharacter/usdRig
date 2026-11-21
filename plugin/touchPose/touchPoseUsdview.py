#
# TouchPose's usdview plugin container.
#
# The panel is imported lazily inside the callback, as every panel in
# rigExecUsdview is, so that this module stays importable with no Qt.
#
import os
import sys

from pxr import Tf
from pxr.Usdviewq.plugin import PluginContainer

from usdRigViewMenu import AddToRigExecMenu as _AddToRigExecMenu


# usdview's plugin loader does not retain a container that registers no
# commands, and a garbage-collected container takes its Qt connections
# with it. rigExecUsdview keeps its containers per session for the same
# reason, and this files each session's container in a registry of the
# same kind: several usdview sessions can share this module in one process,
# and each has its own container. The command registered below anchors the
# container too, so a process without rigExecUsdview's registry on the
# module search path still keeps it.
_containers = None


def _Containers():
    """The per-session container registry, or None without rigExecUsdview."""
    global _containers
    if _containers is None:
        try:
            import sessionRegistry
        except ImportError:
            return None
        _containers = sessionRegistry.SessionRegistry("touchPose containers")
    return _containers


def ContainerFor(usdviewApi=None):
    """The TouchPose container of `usdviewApi`'s session, or None."""
    containers = _Containers()
    if containers is None:
        return None
    if usdviewApi is None:
        return containers.Current()
    return containers.Get(usdviewApi)


class TouchPoseContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        self._api = plugCtx

        # Importable from this directory without it being on PYTHONPATH:
        # the plugin is found through PXR_PLUGINPATH_NAME, which says
        # nothing about module search, and `touchPoseUI` imports
        # `touchPoseModel` beside it.
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)

        containers = _Containers()
        if containers is not None:
            containers.Set(plugCtx, self)

        self._touchPose = plugRegistry.registerCommandPlugin(
            "TouchPoseContainer.touchPose",
            "TouchPose",
            lambda api: self._OpenPanel(api))

    def configureView(self, plugRegistry, plugUIBuilder):
        _AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                          self._touchPose, 40)
        self._ActivateByDefault()

    def _ActivateByDefault(self):
        """TouchPose is ON when a rig that has it opens.

        The mode used to require opening the panel, and to switch itself
        off again when the panel closed. Both are gone: the animator's
        rule is that a rig with touch regions is touchable straight
        away, and stays touchable until the toggle says otherwise.

        DEFERRED to the event loop rather than run here. configureView
        runs while usdview is still assembling itself, and the
        controller needs a stage view and a stage to find regions on --
        asked too early it finds neither and quietly answers False.

        SetActive already refuses a stage with no touch regions, which
        is what makes this safe to call unconditionally: on a rig
        without them it is a no-op, not an error.
        """
        try:
            from pxr.Usdviewq.qt import QtCore
        except ImportError:
            return                      # no Qt: nothing to activate into
        QtCore.QTimer.singleShot(0, self._Activate)

    def _Activate(self):
        try:
            import touchPoseUI
            controller = touchPoseUI.TouchPoseController.GetInstance(
                self._api)
            controller.SetActive(True)
        except Exception as error:
            # Never take usdview down over a convenience. A rig with no
            # regions, a stage still loading, or a Qt build without the
            # view all land here and simply leave the mode off.
            Tf.Warn("touchPose: could not activate by default: %s" % error)

    def _OpenPanel(self, usdviewApi=None):
        try:
            import touchPoseUI
        except ImportError as error:
            Tf.Warn("touchPose: panel unavailable: %s" % error)
            return None
        return touchPoseUI.OpenTouchPosePanel(usdviewApi or self._api)


Tf.Type.Define(TouchPoseContainer)
