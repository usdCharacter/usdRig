"""Stage-free menu contract using OpenUSD plugin discovery and Qt."""
import itertools
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest

from pxr import Plug
from pxr.Usdviewq.plugin import (PluginContainerTfType, PluginRegistry,
                                 PluginUIBuilder)
from pxr.Usdviewq.qt import QtCore, QtWidgets

_ROOT = Path(__file__).resolve().parents[2]
_TYPES = {
    "rigExecUsdview": "rigExecUsdview.RigExecUsdviewContainer",
    "shapeEditor": "shapeEditorUsdview.ShapeEditorContainer",
    "touchPose": "touchPoseUsdview.TouchPoseContainer",
}
_VIEWPORT = ["Viewport Tools", "View Cube", "View Axis"]
_GENERAL = ["Avar Editor", "Layer Stack", "Layer Opinions", "Execution Stack",
            "Deformer Toggles", "Profiler", "Cache Strip", "AI Assistant"]
_ANIMATION = ["Graph Editor", "Shape Editor", "Control Picker", "Hover Picker",
              "TouchPose", "Volume Weight Editor"]


class _EmptyModel(QtCore.QObject):
    signalStageReplaced = QtCore.Signal()
    currentFrameChanged = QtCore.Signal()
    stage = None


def _labels(menu):
    return [action.text() for action in menu.actions()]


class MenuTest(unittest.TestCase):
    def setUp(self):
        self.window = QtWidgets.QMainWindow()
        self.builder = PluginUIBuilder(self.window)
        self.api = SimpleNamespace(qMainWindow=self.window,
                                   dataModel=_EmptyModel())
        self.registry = PluginRegistry(self.api)

    def _command(self, name, label=None, callback=lambda api: None):
        return self.registry.registerCommandPlugin(name, label or name,
                                                   callback)

    def test_orders_and_standalone_discovery(self):
        for selected in [tuple(_TYPES), *((name,) for name in _TYPES)]:
            with self.subTest(selected=selected):
                paths = [str(_ROOT / "plugin" / name) for name in selected]
                env = dict(os.environ, PXR_PLUGINPATH_NAME=os.pathsep.join(paths))
                plugin_root = _ROOT / "plugin"
                inherited = [path for path in env.get("PYTHONPATH", "").split(os.pathsep)
                             if path and Path(path).resolve().parent != plugin_root]
                env["PYTHONPATH"] = os.pathsep.join(
                    paths + [str(plugin_root / "usdRigViewShared")] + inherited)
                probe = subprocess.run([sys.executable, __file__, *selected],
                                       env=env, capture_output=True, text=True)
                self.assertEqual(probe.returncode, 0, probe.stdout + probe.stderr)

    def test_live_membership(self):
        from usdRigViewMenu import AddToRigExecMenu

        calls = []
        command = self._command("first", "Same Label", lambda api: calls.append(api))
        twin = self._command("second", "Same Label")
        add = AddToRigExecMenu
        action = add(self.builder, "Viewport", command, 10)
        menu = action.parent()
        self.assertIs(add(self.builder, "Viewport", command, 99), action)
        self.assertIsNot(add(self.builder, "Viewport", twin, 10), action)
        self.assertIsNot(add(self.builder, None, command, 10), action)
        self.assertIsNot(add(self.builder, "General Editors", command, 10), action)
        other_window = QtWidgets.QMainWindow()
        self.assertIsNot(add(PluginUIBuilder(other_window), "Viewport", command, 10),
                         action)
        menu.removeAction(action)
        new_action = add(self.builder, "Viewport", command, 5)
        self.assertIsNot(new_action, action)
        self.assertEqual(_labels(menu), ["Same Label", "Same Label"])
        self.assertIs(menu.actions()[0], new_action)
        menu.clear()
        rebuilt = add(self.builder, "Viewport", command, 10)
        self.assertEqual(menu.actions(), [rebuilt])
        self.assertIs(add(self.builder, "Viewport", command, 10), rebuilt)
        rebuilt.trigger()
        self.assertEqual(calls, [self.api])

    def test_ranked_and_unranked_neighbors(self):
        from usdRigViewMenu import AddToRigExecMenu

        add = AddToRigExecMenu
        host = self.builder.findOrCreateMenu("RigExec")
        host.findOrCreateSubmenu("Animation Editors").addItem(self._command("Host"))
        host.findOrCreateSubmenu("Unknown")
        host.findOrCreateSubmenu("General Editors")
        action = add(self.builder, "Viewport", self._command("First"), 10)
        menu = action.parent()
        menu.addAction("Raw")
        add(self.builder, "Viewport", self._command("Last"), 20)
        add(self.builder, "Viewport", self._command("Tie"), 10)
        add(self.builder, "Viewport", self._command("Low"), 5)
        add(self.builder, "Animation Editors", self._command("Editor"), 20)
        self.assertEqual(_labels(menu), ["Low", "First", "Raw", "Tie", "Last"])
        top = self.window.menuBar().actions()[0].menu()
        self.assertEqual(_labels(top),
                         ["Viewport", "Unknown", "General Editors", "Animation Editors"])
        self.assertEqual(_labels(top.actions()[-1].menu()), ["Host", "Editor"])


def _check_discovery(selected):
    types = Plug.Registry.GetAllDerivedTypes(PluginContainerTfType)
    found = {entry.typeName: entry for entry in types}
    assert set(found) == {_TYPES[name] for name in selected}, found
    expected = {}
    if "rigExecUsdview" in selected:
        expected.update({"Viewport": _VIEWPORT, "General Editors": _GENERAL})
    expected["Animation Editors"] = [
        label for label in _ANIMATION
        if ({"Shape Editor": "shapeEditor", "TouchPose": "touchPose"}
            .get(label, "rigExecUsdview")) in selected]
    for order in itertools.permutations(selected):
        case = MenuTest()
        case.setUp()
        containers = []
        for name in order:
            entry = found[_TYPES[name]]
            Plug.Registry().GetPluginForType(entry).Load()
            container = entry.pythonClass()
            container.registerPlugins(case.registry, case.api)
            containers.append(container)
        for _ in range(2):
            for container in containers:
                container.configureView(case.registry, case.builder)
            bar = case.window.menuBar()
            case.assertEqual(_labels(bar), ["RigExec"], order)
            submenus = bar.actions()[0].menu().actions()
            case.assertEqual([action.text() for action in submenus], list(expected), order)
            for action in submenus:
                case.assertEqual(_labels(action.menu()), expected[action.text()], order)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    if len(sys.argv) > 1 and sys.argv[1] in _TYPES:
        _check_discovery(sys.argv[1:])
    else:
        unittest.main()
