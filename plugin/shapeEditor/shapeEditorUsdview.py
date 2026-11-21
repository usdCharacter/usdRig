#
# The Shape Editor's usdview plugin container.
import os
import sys

from pxr import Tf
from pxr.Usdviewq.plugin import PluginContainer
from usdRigViewMenu import AddToRigExecMenu as _AddToRigExecMenu


# usdview does not retain a container that registers no commands, and a
# collected container takes its Qt connections with it. One per session,
# filed like rigExecUsdview's (several usdview sessions can share this
# module in one process); the command registered below anchors the
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
        _containers = sessionRegistry.SessionRegistry(
            "shapeEditor containers")
    return _containers


def ContainerFor(usdviewApi=None):
    """The Shape Editor container of `usdviewApi`'s session, or None."""
    containers = _Containers()
    if containers is None:
        return None
    if usdviewApi is None:
        return containers.Current()
    return containers.Get(usdviewApi)


class ShapeEditorContainer(PluginContainer):

    def registerPlugins(self, plugRegistry, plugCtx):
        self._api = plugCtx

        # Found through PXR_PLUGINPATH_NAME, which says nothing about
        # module search -- so this directory has to go on sys.path for
        # `shapeEditorUI` to import `shapeEditorModel` beside it.
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)

        containers = _Containers()
        if containers is not None:
            containers.Set(plugCtx, self)

        self._shapeEditor = plugRegistry.registerCommandPlugin(
            "ShapeEditorContainer.shapeEditor",
            "Shape Editor",
            lambda api: self._OpenPanel(api))

    def configureView(self, plugRegistry, plugUIBuilder):
        _AddToRigExecMenu(plugUIBuilder, "Animation Editors",
                          self._shapeEditor, 20)

    def _OpenPanel(self, usdviewApi=None):
        try:
            import shapeEditorUI
        except ImportError as error:
            Tf.Warn("shapeEditor: panel unavailable: %s" % error)
            return None
        panel = shapeEditorUI.ShapeEditorPanel.GetInstance(
            usdviewApi or self._api)
        panel.show()
        panel.raise_()
        return panel


Tf.Type.Define(ShapeEditorContainer)
