"""Measure completed animation paints, including the first playback loop.

Run with testusdview --testScript tests/testUsdviewPlayback.py on an animated
stage after loading bin/_env. The paced runs keep the stage's frame rate;
RIGEXEC_PLAYBACK_MIN_FPS (default 24) is the minimum throughput with playback
requested at 120 fps.
"""
import json
import os
import statistics
import time

from pxr.Usdviewq.qt import QtCore
from pxr.Usdviewq.usdviewApi import UsdviewApi
import rigExecUsdview


def testUsdviewInputFunction(controller):
    container = rigExecUsdview.ContainerFor(UsdviewApi(controller))
    assert container is not None and container._active
    assert container._playbackDriver is not None
    view = controller._stageView
    minimum, maximum = view.minimumSize(), view.maximumSize()
    physical = view._physicalWindowSize
    view.SetPhysicalWindowSize(1280, 720)
    originalPaint = view.paintGL
    rate = controller.framesPerSecond
    samples = []

    def paint():
        originalPaint()
        frame = float(controller._dataModel.currentFrame.GetValue())
        if not samples or samples[-1][1] != frame:
            samples.append((time.perf_counter(), frame))

    view.paintGL = paint
    try:
        for phase, target in (("first", rate), ("steady", rate), ("capacity", 120)):
            samples.clear()
            controller.framesPerSecond = target
            controller._ui.playButton.setChecked(True)
            controller._playClicked()
            loop = QtCore.QEventLoop()
            QtCore.QTimer.singleShot(6000, loop.quit)
            loop.exec()
            controller._ui.playButton.setChecked(False)
            controller._playClicked()
            assert len(samples) > 2
            intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
            fps = 1.0 / statistics.mean(intervals)
            # A frame more than a period late restarts the driver's
            # schedule (an interval over two periods) and its lost time is
            # not made up; the frames between such restarts keep the rate.
            late = [i for i in intervals if i > 2.0 / target]
            paced = [i for i in intervals if i <= 2.0 / target]
            print("RIGEXEC_PLAYBACK " + json.dumps({
                "phase": phase, "target": target, "painted_fps": fps,
                "frames": len(samples), "late": len(late),
                "median_ms": statistics.median(intervals) * 1000,
                "p95_ms": sorted(intervals)[int(.95 * (len(intervals) - 1))] * 1000,
            }), flush=True)
            if phase == "capacity":
                assert fps >= float(os.environ.get("RIGEXEC_PLAYBACK_MIN_FPS", "24"))
            else:
                assert len(late) <= len(intervals) // 20, late
                # A finite timed window can start or end between two paints.
                assert abs(1.0 / statistics.mean(paced) - target) <= 1.0 / 6.0
                frames = list(controller._timeSamples)
                for a, b in zip(samples, samples[1:]):
                    index = controller._findClosestFrameIndex(a[1])
                    expected = frames[(index + 1) % len(frames)]
                    assert abs(b[1] - expected) < 1e-6, (a, b)
    finally:
        controller._ui.playButton.setChecked(False)
        controller._playClicked()
        controller.framesPerSecond = rate
        view.paintGL = originalPaint
        view._physicalWindowSize = physical
        view.setMinimumSize(minimum)
        view.setMaximumSize(maximum)
