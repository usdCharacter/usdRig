"""Pace usdview playback without sleeping on the Qt owner thread."""
import math
import time


class PlaybackDriver:
    def __init__(self, controller):
        from pxr.Usdviewq.qt import QtCore
        self.controller = controller
        self.timer = controller._qtimer
        self.original = controller._advanceFrameForPlayback
        self.interval = self.timer.interval()
        self.timerType = self.timer.timerType()
        self.singleShot = self.timer.isSingleShot()
        self.deadline = None
        self.started = None
        self.restored = False
        self.timer.timeout.disconnect(self.original)
        self.timer.setSingleShot(True)
        self.timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.Tick)

    def Tick(self):
        from pxr.Usdviewq.appController import HUDEntries
        c = self.controller
        if self.restored or not c._dataModel.playing:
            return
        now = time.perf_counter()
        period = 1.0 / max(1.0, c.framesPerSecond)
        if self.deadline is None or now - self.deadline > period:
            self.deadline = now
        self.deadline += period
        try:
            if c._playbackIndex == 0:
                self.started = now
            elif c._playbackIndex == 4 and self.started is not None:
                delta = (now - self.started) / 4.0
                if delta > 0:
                    c._fpsHUDInfo[HUDEntries.PLAYBACK] = "%.2f ms (%.2f FPS)" % (
                        delta * 1000.0, 1.0 / delta)
            c._playbackIndex = (c._playbackIndex + 1) % 5
            c._advanceFrame()
        finally:
            # A frame that raised still re-arms, as usdview's repeating
            # timer would; a frame that restored usdview's timer does not.
            if not self.restored and c._dataModel.playing:
                # Leave Qt free to paint this frame before requesting the next one.
                delay = max(1, math.ceil((self.deadline - time.perf_counter()) * 1000.0))
                self.timer.start(delay)

    def Restore(self):
        """Return usdview's own timer; later calls do nothing."""
        if self.restored:
            return
        self.restored = True
        self.timer.stop()
        try:
            self.timer.timeout.disconnect(self.Tick)
        except (RuntimeError, TypeError):
            pass
        self.timer.timeout.connect(self.original)
        self.timer.setSingleShot(self.singleShot)
        self.timer.setTimerType(self.timerType)
        self.timer.setInterval(self.interval)
        # A Restore inside Tick finds the single shot already spent.
        if self.controller._dataModel.playing:
            self.timer.start()


def Install(api):
    controller = getattr(api, "_UsdviewApi__appController", None)
    if controller is None or not hasattr(controller, "_qtimer"):
        return None
    return PlaybackDriver(controller)
