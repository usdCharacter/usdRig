"""Editor methods consume published frames and compiler diagnostics."""
import pathlib
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import rigexec_test_env

rigexec_test_env.SetupPluginTest()
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] /
                       "plugin" / "shapeEditor"))

from pxr import Gf, Usd
import execStackUI
import shapeEditorUI


class EditorPublications(unittest.TestCase):
    def test_frames_use_current_publication_and_time(self):
        matrix = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(3, 4, 5))
        imaging = Mock()
        imaging.Has.return_value = True

        def read(path, frame, is_default, output):
            if path == "/missing":
                return 0
            for i, value in enumerate(v for row in matrix for v in row):
                output[i] = value
            return 1

        imaging.GetControlFrameAssetSpace.side_effect = read
        panel = SimpleNamespace(_StageImaging=lambda: imaging,
                                _api=SimpleNamespace(frame=Usd.TimeCode.Default()))
        frames = shapeEditorUI.ShapeEditorPanel._VizFrames(panel)
        self.assertEqual(frames("/driver"), matrix)
        self.assertEqual(frames("/driver"), matrix)
        self.assertIsNone(frames("/missing"))
        self.assertIsNone(frames("/missing"))
        self.assertEqual(imaging.GetControlFrameAssetSpace.call_count, 2)
        self.assertEqual(imaging.GetControlFrameAssetSpace.call_args_list[0].args[:3],
                         ("/driver", 0.0, 1))
        panel._api.frame = Usd.TimeCode(14)
        fresh = shapeEditorUI.ShapeEditorPanel._VizFrames(panel)
        fresh("/driver")
        self.assertEqual(imaging.GetControlFrameAssetSpace.call_args.args[:3],
                         ("/driver", 14.0, 0))

    def test_unavailable_publication_does_not_evaluate_a_rig(self):
        status = Mock()
        panel = SimpleNamespace(_StageImaging=lambda: None, _vizInfo=status)
        self.assertIsNone(shapeEditorUI.ShapeEditorPanel._VizFrames(panel))
        status.setText.assert_called_once()
        pose = SimpleNamespace(weight=1.0)
        panel = SimpleNamespace(
            _ReadFromViewport=lambda: False,
            _interpolators=[SimpleNamespace(poses=[pose])],
            _firingOnly=SimpleNamespace(isChecked=lambda: False),
            _UpdateNumbers=Mock(), _Status=Mock())
        shapeEditorUI.ShapeEditorPanel._RefreshWeights(panel)
        self.assertEqual(pose.weight, 0.0)
        panel._UpdateNumbers.assert_called_once()
        panel._Status.assert_called_once_with("no published weights")

    def test_status_reports_compiler_without_type_order_rules(self):
        panel = SimpleNamespace(_solvers=[], _movers=[
            {"type": "RigExecSkinMover", "ordinal": 0},
            {"type": "RigExecOrientConstraint", "ordinal": 1}],
            _compileError=None, _compileDiagnostics=[])
        self.assertEqual(execStackUI.ExecStackPanel._StatusText(panel, 0, 2),
                         "0 of 0 solvers, 2 of 2 movers shown")
        panel._compileDiagnostics = ["Skipped /Rig/Move: missing input"]
        self.assertIn(panel._compileDiagnostics[0],
                      execStackUI.ExecStackPanel._StatusText(panel, 0, 2))
        panel._compileError = "dependency cycle at /Rig/Move"
        self.assertIn(panel._compileError,
                      execStackUI.ExecStackPanel._StatusText(panel, 0, 0))


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
