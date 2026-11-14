#
# RigExec usdview playblast: renders the viewport frame by frame to images
# or a movie, with Maya's Playblast Options window. playblastModel.py
# holds the options and the file and size rules.
#
# Every frame is rendered by usdview's own StageView into a framebuffer at
# the display size, so what is drawn -- renderer, render mode, lights,
# camera and rig pose -- is the viewport's; below full Scale it is filtered
# down. Render offscreen keeps the visible viewport still while frames
# render; otherwise it follows them. Manipulators and other overlay widgets
# are never part of the frame.
#
# Capture is not paced by the display: Qt events, the progress window and
# the visible viewport are serviced at most every _UI_INTERVAL_S, since each
# window swap waits for vertical sync. Movie frames stream to ffmpeg's
# standard input while the next frame renders.
#
import collections
import contextlib
import os
import shutil
import subprocess
import tempfile
import time

from pxr import Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    from pxr.Usdviewq.qt import QtOpenGL
except ImportError:                                  # PySide2 builds
    QtOpenGL = None

import playblastModel as model

_SETTINGS_ORG = "RigExec"
_SETTINGS_APP = "Playblast"
_SETTINGS_KEY = "options"

# A progressive renderer is polled until it converges, up to this long a
# frame. Storm converges in one pass.
_CONVERGE_TIMEOUT_S = 60.0

_UI_INTERVAL_S = 1.0 / 30.0

# View settings that draw ornaments into the frame; DrawAxis has none.
_ORNAMENT_SETTINGS = ("showHUD", "showMask_Outline", "showReticles_Inside",
                      "showReticles_Outside", "displayCameraOracles")

_QT_WRITERS = {"jpg": "JPG", "png": "PNG", "tif": "TIF", "bmp": "BMP",
               "webp": "WEBP"}


def _Controller(usdviewApi):
    return getattr(usdviewApi, "_UsdviewApi__appController", None)


def ImageEncodings():
    """The image encodings this Qt can write, in Maya's order."""
    supported = {bytes(name).decode("ascii", "ignore").lower()
                 for name in QtGui.QImageWriter.supportedImageFormats()}
    return tuple(encoding for encoding in model.IMAGE_ENCODINGS
                 if encoding in supported)


def LoadOptions():
    settings = QtCore.QSettings(_SETTINGS_ORG, _SETTINGS_APP)
    options = model.PlayblastOptions.FromJson(
        settings.value(_SETTINGS_KEY, ""))
    return _Available(options)


def SaveOptions(options):
    settings = QtCore.QSettings(_SETTINGS_ORG, _SETTINGS_APP)
    settings.setValue(_SETTINGS_KEY, options.ToJson())


def _Available(options):
    """Options this machine can run: without ffmpeg a movie format falls
    back to an image sequence."""
    encodings = ImageEncodings() or ("png",)
    if (model.IsMovie(options.format) and
            model.FindFfmpeg() is None):
        options = options.Copy(format=model.FORMAT_IMAGE,
                               encoding=model.DefaultEncoding(
                                   model.FORMAT_IMAGE, encodings))
    return options.Normalized(encodings)


def _FramebufferClasses():
    for module in (QtOpenGL, QtGui):
        fbo = getattr(module, "QOpenGLFramebufferObject", None)
        if fbo is not None:
            return fbo, getattr(module, "QOpenGLFramebufferObjectFormat")
    return None, None


@contextlib.contextmanager
def _ViewSize(view, renderSize, outputSize):
    # usdview sizes the render from _physicalWindowSize, places its HUD from
    # width() and height(), and its mask and reticles from those times
    # devicePixelRatioF(). Logical size is the output's, so ornaments keep
    # their size in a frame that is filtered down; physical size is the
    # render's.
    ratio = view.devicePixelRatioF() or 1.0
    factor = float(renderSize[0]) / outputSize[0]
    saved = view._physicalWindowSize
    view._physicalWindowSize = tuple(renderSize)
    view.width = lambda: outputSize[0] / ratio
    view.height = lambda: outputSize[1] / ratio
    view.devicePixelRatioF = lambda: ratio * factor
    try:
        yield
    finally:
        del view.width
        del view.height
        del view.devicePixelRatioF
        view._physicalWindowSize = saved


class OffscreenTarget(object):
    """A framebuffer of its own that the StageView renders into, at any
    size and without drawing to the screen; frames come back at
    `outputSize`. Close() before discarding."""

    def __init__(self, view, renderSize, outputSize=None):
        self._view = view
        self._size = (int(renderSize[0]), int(renderSize[1]))
        self._outputSize = (self._size if outputSize is None else
                            (int(outputSize[0]), int(outputSize[1])))
        self._fbo = None

    def _Ensure(self):
        if self._fbo is not None:
            return self._fbo
        fboClass, formatClass = _FramebufferClasses()
        if fboClass is None:
            raise RuntimeError("Qt has no QOpenGLFramebufferObject")
        attachment = getattr(fboClass, "Attachment", fboClass)
        fmt = formatClass()
        fmt.setAttachment(attachment.CombinedDepthStencil)
        fmt.setSamples(max(0, self._view.format().samples()))
        fbo = fboClass(self._size[0], self._size[1], fmt)
        if not fbo.isValid():
            raise RuntimeError("cannot allocate a %dx%d framebuffer"
                               % self._size)
        self._fbo = fbo
        return fbo

    def Render(self):
        """The current frame as an opaque Format_RGB32 QImage."""
        view = self._view
        view.makeCurrent()
        try:
            fbo = self._Ensure()
            if not fbo.bind():
                raise RuntimeError("cannot bind the offscreen framebuffer")
            try:
                with _ViewSize(view, self._size, self._outputSize):
                    deadline = time.monotonic() + _CONVERGE_TIMEOUT_S
                    while True:
                        view.paintGL()
                        renderer = view._getRenderer()
                        if (renderer is None or renderer.IsConverged() or
                                time.monotonic() > deadline):
                            break
                        time.sleep(0.01)
                image = fbo.toImage()
            finally:
                fbo.release()
        finally:
            view.doneCurrent()
        image = image.convertToFormat(QtGui.QImage.Format_RGB32)
        if self._outputSize != self._size:
            image = image.scaled(self._outputSize[0], self._outputSize[1],
                                 QtCore.Qt.IgnoreAspectRatio,
                                 QtCore.Qt.SmoothTransformation)
        return image

    def Close(self):
        if self._fbo is None:
            return
        # Deleted with the context current, which owns its GL objects.
        self._view.makeCurrent()
        try:
            self._fbo = None
        finally:
            self._view.doneCurrent()


@contextlib.contextmanager
def _Ornaments(view, viewSettings, show):
    if show:
        yield
        return
    saved = [(name, getattr(viewSettings, name))
             for name in _ORNAMENT_SETTINGS if hasattr(viewSettings, name)]
    for name, _ in saved:
        setattr(viewSettings, name, False)
    view.DrawAxis = lambda *args: None
    try:
        yield
    finally:
        del view.DrawAxis
        for name, value in saved:
            setattr(viewSettings, name, value)


def _ShowFrame(controller, frame):
    """Move usdview to `frame`, which may lie off its slider."""
    dataModel = controller._dataModel
    timeCode = Usd.TimeCode(frame)
    if dataModel.currentFrame != timeCode:
        # The rig evaluates on currentFrameChanged, before this returns.
        dataModel.currentFrame = timeCode
    samples = controller._timeSamples
    if samples:
        index = controller._findClosestFrameIndex(frame)
        if abs(samples[index] - frame) < 1e-9:
            slider = controller._ui.frameSlider
            blocked = slider.blockSignals(True)
            slider.setValue(index)
            slider.blockSignals(blocked)
    controller.setFrameField(frame)


@contextlib.contextmanager
def _PlayblastState(controller, offScreen):
    dataModel = controller._dataModel
    view = controller._stageView
    if dataModel.playing:
        controller._ui.playButton.click()
    frame = dataModel.currentFrame
    # Frames step as they do in playback: no property-view refresh per
    # frame, and the rig warms no idle frames between them.
    dataModel.playing = True
    if offScreen:
        view.setUpdatesEnabled(False)
    try:
        yield
    finally:
        dataModel.playing = False
        if offScreen:
            view.setUpdatesEnabled(True)
        if not frame.IsDefault():
            _ShowFrame(controller, frame.GetValue())
        controller._updateOnFrameChange()


def _TempRoot():
    location = QtCore.QStandardPaths.writableLocation(
        QtCore.QStandardPaths.TempLocation) or tempfile.gettempdir()
    root = os.path.join(location, "rigExecPlayblast")
    os.makedirs(root, exist_ok=True)
    return root


def DefaultMovieDirectory():
    location = QtCore.QStandardPaths.writableLocation(
        QtCore.QStandardPaths.MoviesLocation)
    return location or os.path.expanduser("~")


def _StageStem(stage):
    return model.StageStem(stage.GetRootLayer().identifier)


class PlayblastResult(object):
    def __init__(self, paths, movie, numbers, rate):
        self.paths = paths
        self.movie = movie
        self.numbers = numbers
        self.rate = rate


class _Canceled(Exception):
    pass


def _Warn(parent, text):
    QtWidgets.QMessageBox.warning(parent, "Playblast", text)


def _ConfirmOverwrite(parent, paths):
    existing = [path for path in paths if os.path.exists(path)]
    if not existing:
        return True
    shown = existing[0] if len(existing) == 1 else (
        "%s and %d more" % (existing[0], len(existing) - 1))
    answer = QtWidgets.QMessageBox.question(
        parent, "Playblast", "%s already exists. Overwrite?" % shown,
        QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        QtWidgets.QMessageBox.No)
    return answer == QtWidgets.QMessageBox.Yes


class _Encoder(object):
    """ffmpeg encoding raw frames written to its standard input. Close()
    before discarding."""

    def __init__(self, argv, logPath):
        self._logPath = logPath
        self._log = open(logPath, "wb")
        kwargs = {}
        if os.name == "nt":
            kwargs["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW",
                                              0)
        try:
            self._process = subprocess.Popen(
                argv, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=self._log, **kwargs)
        except OSError:
            self._log.close()
            raise

    def Write(self, image):
        try:
            self._process.stdin.write(image.constBits())
        except OSError:
            self._process.wait()
            raise self._Failure()

    def Finish(self, progress):
        try:
            self._process.stdin.close()
        except OSError:
            pass
        while self._process.poll() is None:
            QtWidgets.QApplication.processEvents()
            if progress.wasCanceled():
                raise _Canceled()
            time.sleep(0.02)
        if self._process.returncode != 0:
            raise self._Failure()

    def Close(self):
        if self._process.poll() is None:
            self._process.kill()
            self._process.wait()
        try:
            self._process.stdin.close()
        except OSError:
            pass
        self._log.close()

    def _Failure(self):
        self._log.flush()
        with open(self._logPath, "rb") as log:
            detail = log.read().decode("utf-8", "replace").strip()
        return RuntimeError("ffmpeg failed (%s): %s"
                            % (self._process.returncode, detail[-800:]))


def Playblast(usdviewApi, options, parent=None):
    """Run one playblast. Returns a PlayblastResult, or None when it was
    canceled or could not run (the reason is shown)."""
    controller = _Controller(usdviewApi)
    parent = parent or usdviewApi.qMainWindow
    if controller is None or getattr(controller, "_stageView", None) is None:
        _Warn(parent, "There is no viewport to playblast.")
        return None
    if getattr(controller, "_rigExecPlayblasting", False):
        return None
    stage = controller._dataModel.stage
    if not stage:
        _Warn(parent, "There is no stage to playblast.")
        return None
    encodings = ImageEncodings() or ("png",)
    options = options.Normalized(encodings)
    movie = model.IsMovie(options.format)
    ffmpeg = model.FindFfmpeg() if movie else None
    if movie and ffmpeg is None:
        _Warn(parent, "The %s format needs ffmpeg. Put ffmpeg on PATH or "
              "set %s to its path, or choose the image format."
              % (options.format, model.FFMPEG_ENV))
        return None

    view = controller._stageView
    frames = model.PlayblastFrames(
        options, controller._timeSamples,
        controller._dataModel.currentFrame.GetValue(), controller.step)
    if not frames:
        _Warn(parent, "The time range holds no frames.")
        return None
    renderSettings = None
    if options.displaySize == model.SIZE_RENDER_SETTINGS:
        renderSettings = model.RenderSettingsSize(stage)
        if renderSettings is None:
            _Warn(parent, "The stage has no render settings resolution; "
                  "using the window size.")
    windowSize = view.GetPhysicalWindowSize()
    width, height = model.OutputSize(options, windowSize, renderSettings)
    renderSize = model.RenderSize(options, windowSize, renderSettings)
    rate = model.FrameRate(stage.GetTimeCodesPerSecond(), controller.step,
                           controller.framesPerSecond)
    numbers = model.FrameNumbers(frames)

    stem = _StageStem(stage)
    try:
        tempDirectory = "" if options.saveToFile else tempfile.mkdtemp(
            prefix=stem + "_", dir=_TempRoot())
    except OSError as error:
        _Warn(parent, "Cannot create a temporary folder: %s" % error)
        return None
    base = model.OutputBase(options, stem, tempDirectory,
                            DefaultMovieDirectory())
    if movie:
        outputs = [model.MoviePath(base, options.format)]
    else:
        outputs = [model.ImagePath(base, number, options.framePadding,
                                   options.encoding) for number in numbers]
    if options.saveToFile and not _ConfirmOverwrite(parent, outputs):
        return None
    try:
        os.makedirs(os.path.dirname(os.path.abspath(base)), exist_ok=True)
        workDirectory = tempfile.mkdtemp(
            prefix=stem + "_frames_", dir=_TempRoot()) if movie else None
    except OSError as error:
        _Warn(parent, "Cannot create the playblast folder: %s" % error)
        return None
    keepFrames = not movie or not options.removeTemporaryFiles
    if movie:
        frameBase = os.path.join(workDirectory, stem)
        frameEncoding = model.INTERMEDIATE_ENCODING
        frameNumbers = list(range(len(frames)))
    else:
        frameBase = base
        frameEncoding = options.encoding
        frameNumbers = numbers

    writer = _QT_WRITERS.get(frameEncoding, frameEncoding.upper())
    quality = options.quality if model.UsesQuality(frameEncoding) else -1
    progress = QtWidgets.QProgressDialog(
        "Playblasting...", "Cancel", 0, len(frames) + (1 if movie else 0),
        parent)
    progress.setWindowTitle("Playblast")
    progress.setWindowModality(QtCore.Qt.WindowModal)
    progress.setMinimumDuration(0)
    progress.setValue(0)
    target = OffscreenTarget(view, renderSize, (width, height))
    encoder = None
    encoded = False
    controller._rigExecPlayblasting = True
    framePaths = []
    try:
        if movie:
            encoder = _Encoder(
                model.FfmpegCommand(ffmpeg, (width, height), rate,
                                    options.format, options.encoding,
                                    options.quality, outputs[0]),
                os.path.join(workDirectory, "ffmpeg.log"))
        with _PlayblastState(controller, options.offScreen), \
                _Ornaments(view, controller._dataModel.viewSettings,
                           options.showOrnaments):
            serviced = None
            for index, frame in enumerate(frames):
                if progress.wasCanceled():
                    raise _Canceled()
                _ShowFrame(controller, frame)
                image = target.Render()
                if encoder is not None:
                    encoder.Write(image)
                if keepFrames:
                    path = model.ImagePath(frameBase, frameNumbers[index],
                                           options.framePadding,
                                           frameEncoding)
                    if not image.save(path, writer, quality):
                        raise RuntimeError("cannot write %s" % path)
                    framePaths.append(path)
                now = time.monotonic()
                if (index + 1 == len(frames) or serviced is None or
                        now - serviced >= _UI_INTERVAL_S):
                    progress.setLabelText(
                        "Playblasting frame %s (%d of %d)"
                        % (model.FormatNumber(frame), index + 1,
                           len(frames)))
                    if not options.offScreen:
                        view.update()
                    progress.setValue(index + 1)
                    QtWidgets.QApplication.processEvents()
                    serviced = time.monotonic()
        if encoder is not None:
            progress.setLabelText("Encoding %s..." % outputs[0])
            encoder.Finish(progress)
            progress.setValue(len(frames) + 1)
            encoded = True
    except _Canceled:
        usdviewApi.PrintStatus("Playblast canceled.")
        return None
    except (RuntimeError, OSError, Tf.ErrorException) as error:
        _Warn(parent, "Playblast failed: %s" % error)
        return None
    finally:
        controller._rigExecPlayblasting = False
        if encoder is not None:
            encoder.Close()
            if not encoded:
                try:
                    os.remove(outputs[0])
                except OSError:
                    pass
        target.Close()
        progress.close()
        if movie and workDirectory and options.removeTemporaryFiles:
            shutil.rmtree(workDirectory, ignore_errors=True)
        view.update()

    result = PlayblastResult(outputs if movie else framePaths, movie,
                             numbers, rate)
    usdviewApi.PrintStatus("Playblast: %d frames, %dx%d, written to %s"
                           % (len(frames), width, height,
                              outputs[0] if movie else
                              os.path.dirname(os.path.abspath(base))))
    if options.view:
        ShowResult(result, parent, stem)
    return result


def ShowResult(result, parent, title):
    if result.movie:
        QtGui.QDesktopServices.openUrl(
            QtCore.QUrl.fromLocalFile(result.paths[0]))
        return None
    viewer = FlipbookViewer(result.paths, result.numbers, result.rate,
                            "Playblast - %s" % title, parent)
    viewer.show()
    viewer.raise_()
    return viewer


def RunPlayblast(usdviewApi):
    """The Playblast button: run with the saved options."""
    return Playblast(usdviewApi, LoadOptions())


def ShowOptions(usdviewApi):
    """The option box: Maya's Playblast Options window, one per session."""
    window = usdviewApi.qMainWindow
    dialog = getattr(window, "_rigExecPlayblastOptions", None)
    if dialog is None:
        dialog = PlayblastOptionsWindow(usdviewApi, window)
        window._rigExecPlayblastOptions = dialog
    dialog.SetOptions(LoadOptions())
    dialog.show()
    dialog.raise_()
    dialog.activateWindow()
    return dialog


class _SliderField(QtWidgets.QWidget):
    """A number field with a slider beside it, as in Maya option boxes."""

    def __init__(self, low, high, decimals=0, step=1, parent=None):
        super(_SliderField, self).__init__(parent)
        self._factor = 10 ** decimals
        if decimals:
            self.field = QtWidgets.QDoubleSpinBox(self)
            self.field.setDecimals(decimals)
        else:
            self.field = QtWidgets.QSpinBox(self)
        self.field.setRange(low, high)
        self.field.setSingleStep(step)
        self.field.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.field.setFixedWidth(60)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.slider.setRange(int(round(low * self._factor)),
                             int(round(high * self._factor)))
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.field)
        layout.addWidget(self.slider, 1)
        self.field.valueChanged.connect(self._FieldChanged)
        self.slider.valueChanged.connect(self._SliderChanged)

    def _FieldChanged(self, value):
        blocked = self.slider.blockSignals(True)
        self.slider.setValue(int(round(value * self._factor)))
        self.slider.blockSignals(blocked)

    def _SliderChanged(self, value):
        self.field.setValue(value / float(self._factor)
                            if self._factor != 1 else value)

    def Value(self):
        return self.field.value()

    def SetValue(self, value):
        self.field.setValue(value)
        self._FieldChanged(self.field.value())


_FORMAT_LABELS = {model.FORMAT_QT: "qt", model.FORMAT_AVI: "avi",
                  model.FORMAT_IMAGE: "image"}
_SIZE_LABELS = ((model.SIZE_WINDOW, "From Window"),
                (model.SIZE_RENDER_SETTINGS, "From Render Settings"),
                (model.SIZE_CUSTOM, "Custom"))


class PlayblastOptionsWindow(QtWidgets.QDialog):
    """Maya's Playblast Options window."""

    def __init__(self, usdviewApi, parent=None):
        super(PlayblastOptionsWindow, self).__init__(parent)
        self.setObjectName("rigExecPlayblastOptions")
        self.setWindowTitle("Playblast Options")
        self._api = usdviewApi
        self._imageEncodings = ImageEncodings() or ("png",)
        self._ffmpeg = model.FindFfmpeg()
        defaults = model.PlayblastOptions()
        self._customSize = (defaults.width, defaults.height)
        self._BuildMenu()
        self._BuildFields()
        self._BuildButtons()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setMenuBar(self._menuBar)
        outer.addLayout(self._grid)
        outer.addStretch(1)
        outer.addLayout(self._buttons)
        self.SetOptions(LoadOptions())

    def _BuildMenu(self):
        self._menuBar = QtWidgets.QMenuBar(self)
        edit = self._menuBar.addMenu("Edit")
        edit.addAction("Save Settings", self._SaveSettings)
        edit.addAction("Reset Settings", self._ResetSettings)

    def _Row(self, label, widget):
        row = self._grid.rowCount()
        if label:
            text = QtWidgets.QLabel(label, self)
            text.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self._grid.addWidget(text, row, 0)
        self._grid.addWidget(widget, row, 1)
        return widget

    def _BuildFields(self):
        self._grid = QtWidgets.QGridLayout()
        self._grid.setColumnStretch(1, 1)
        self.view = self._Row("", QtWidgets.QCheckBox("View", self))
        self.showOrnaments = self._Row(
            "", QtWidgets.QCheckBox("Show ornaments", self))
        self.offScreen = self._Row(
            "", QtWidgets.QCheckBox("Render offscreen", self))

        rangeBox = QtWidgets.QWidget(self)
        rangeLayout = QtWidgets.QHBoxLayout(rangeBox)
        rangeLayout.setContentsMargins(0, 0, 0, 0)
        self.timeSlider = QtWidgets.QRadioButton("Time Slider", rangeBox)
        self.startEnd = QtWidgets.QRadioButton("Start/End", rangeBox)
        rangeLayout.addWidget(self.timeSlider)
        rangeLayout.addWidget(self.startEnd)
        rangeLayout.addStretch(1)
        self._Row("Time range:", rangeBox)
        self.startTime = self._Row("Start time:", self._FrameField())
        self.endTime = self._Row("End time:", self._FrameField())

        self.format = self._Row("Format:", QtWidgets.QComboBox(self))
        for token in model.FORMATS:
            self.format.addItem(_FORMAT_LABELS[token], token)
            if model.IsMovie(token) and self._ffmpeg is None:
                item = self.format.model().item(self.format.count() - 1)
                item.setEnabled(False)
                item.setToolTip("Needs ffmpeg on PATH or %s"
                                % model.FFMPEG_ENV)
        self.encoding = self._Row("Encoding:", QtWidgets.QComboBox(self))
        self.quality = self._Row("Quality:", _SliderField(0, 100))

        self.displaySize = self._Row("Display size:",
                                     QtWidgets.QComboBox(self))
        for token, label in _SIZE_LABELS:
            self.displaySize.addItem(label, token)
        sizeBox = QtWidgets.QWidget(self)
        sizeLayout = QtWidgets.QHBoxLayout(sizeBox)
        sizeLayout.setContentsMargins(0, 0, 0, 0)
        self.width_ = QtWidgets.QSpinBox(sizeBox)
        self.height_ = QtWidgets.QSpinBox(sizeBox)
        for field in (self.width_, self.height_):
            field.setRange(1, model.SIZE_LIMIT)
            field.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            field.setFixedWidth(70)
            sizeLayout.addWidget(field)
        sizeLayout.addStretch(1)
        self._Row("", sizeBox)
        self.scale = self._Row("Scale:", _SliderField(
            model.SCALE_RANGE[0], model.SCALE_RANGE[1], decimals=2,
            step=0.05))
        self.framePadding = self._Row("Frame padding:", _SliderField(
            *model.PADDING_RANGE))
        self.removeTemporaryFiles = self._Row(
            "", QtWidgets.QCheckBox("Remove temporary files", self))
        self.saveToFile = self._Row(
            "", QtWidgets.QCheckBox("Save to file", self))
        fileBox = QtWidgets.QWidget(self)
        fileLayout = QtWidgets.QHBoxLayout(fileBox)
        fileLayout.setContentsMargins(0, 0, 0, 0)
        self.movieFile = QtWidgets.QLineEdit(fileBox)
        self.movieFile.setMinimumWidth(260)
        self.movieFile.setPlaceholderText(QtCore.QDir.toNativeSeparators(
            os.path.join(DefaultMovieDirectory(), "<stage name>")))
        self.browse = QtWidgets.QPushButton("Browse...", fileBox)
        fileLayout.addWidget(self.movieFile, 1)
        fileLayout.addWidget(self.browse)
        self._Row("Movie file:", fileBox)

        self.timeSlider.toggled.connect(self._TimeRangeChanged)
        self.format.currentIndexChanged.connect(self._FormatChanged)
        self.encoding.currentIndexChanged.connect(self._UpdateEnabled)
        self.displaySize.currentIndexChanged.connect(self._SizeChanged)
        self.width_.valueChanged.connect(self._CustomSizeEdited)
        self.height_.valueChanged.connect(self._CustomSizeEdited)
        self.saveToFile.toggled.connect(self._UpdateEnabled)
        self.browse.clicked.connect(self._Browse)

    def _FrameField(self):
        field = QtWidgets.QDoubleSpinBox(self)
        field.setRange(-1e7, 1e7)
        field.setDecimals(2)
        field.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        field.setFixedWidth(90)
        return field

    def _BuildButtons(self):
        self._buttons = QtWidgets.QHBoxLayout()
        for label, slot in (("Playblast", self._Playblast),
                            ("Apply", self._Apply),
                            ("Close", self.close)):
            button = QtWidgets.QPushButton(label, self)
            button.setAutoDefault(False)
            button.clicked.connect(slot)
            self._buttons.addWidget(button, 1)

    def _Formats(self):
        return self.format.currentData() or model.FORMAT_IMAGE

    def _FormatChanged(self, *args):
        format = self._Formats()
        previous = self.encoding.currentText()
        blocked = self.encoding.blockSignals(True)
        self.encoding.clear()
        for encoding in model.Encodings(format, self._imageEncodings):
            self.encoding.addItem(encoding)
        index = self.encoding.findText(previous)
        if index < 0:
            index = self.encoding.findText(
                model.DefaultEncoding(format, self._imageEncodings))
        self.encoding.setCurrentIndex(max(0, index))
        self.encoding.blockSignals(blocked)
        self._UpdateEnabled()

    def _SliderRange(self):
        controller = _Controller(self._api)
        samples = getattr(controller, "_timeSamples", None)
        if samples:
            return (samples[0], samples[-1])
        if controller is not None:
            frame = controller._dataModel.currentFrame
            if not frame.IsDefault():
                return (frame.GetValue(), frame.GetValue())
        return None

    def _TimeRangeChanged(self, *args):
        # As in Maya, Start/End shows the time slider's range until it is
        # chosen, so choosing it starts from the range being played.
        if self.timeSlider.isChecked():
            shown = self._SliderRange()
            if shown is not None:
                self.startTime.setValue(shown[0])
                self.endTime.setValue(shown[1])
        self._UpdateEnabled()

    def _CustomSizeEdited(self, *args):
        if self.displaySize.currentData() == model.SIZE_CUSTOM:
            self._customSize = (self.width_.value(), self.height_.value())

    def _SizeChanged(self, *args):
        size = self.displaySize.currentData()
        view = getattr(_Controller(self._api), "_stageView", None)
        shown = None
        if size == model.SIZE_CUSTOM:
            shown = self._customSize
        elif size == model.SIZE_RENDER_SETTINGS and self._api.stage:
            shown = model.RenderSettingsSize(self._api.stage)
        if shown is None and view is not None:
            shown = view.GetPhysicalWindowSize()
        if shown is not None:
            custom = self._customSize
            self.width_.setValue(int(shown[0]))
            self.height_.setValue(int(shown[1]))
            self._customSize = custom
        self._UpdateEnabled()

    def _UpdateEnabled(self, *args):
        startEnd = self.startEnd.isChecked()
        self.startTime.setEnabled(startEnd)
        self.endTime.setEnabled(startEnd)
        format = self._Formats()
        self.quality.setEnabled(model.UsesQuality(self.encoding.currentText()))
        custom = self.displaySize.currentData() == model.SIZE_CUSTOM
        self.width_.setEnabled(custom)
        self.height_.setEnabled(custom)
        self.removeTemporaryFiles.setEnabled(model.IsMovie(format))
        save = self.saveToFile.isChecked()
        self.movieFile.setEnabled(save)
        self.browse.setEnabled(save)

    def _Browse(self):
        format = self._Formats()
        if model.IsMovie(format):
            extension = model.MOVIE_EXTENSIONS[format]
            pattern = "Movies (*%s)" % extension
        else:
            extension = "." + self.encoding.currentText()
            pattern = "Images (*%s)" % extension
        start = self.movieFile.text().strip() or os.path.join(
            DefaultMovieDirectory(), "playblast")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Movie file", start, pattern)
        if path:
            self.movieFile.setText(path)

    def SetOptions(self, options):
        options = options.Normalized(self._imageEncodings)
        self.view.setChecked(options.view)
        self.showOrnaments.setChecked(options.showOrnaments)
        self.offScreen.setChecked(options.offScreen)
        self.timeSlider.setChecked(options.timeRange == model.TIME_SLIDER)
        self.startEnd.setChecked(options.timeRange == model.START_END)
        self.startTime.setValue(options.startTime)
        self.endTime.setValue(options.endTime)
        self._TimeRangeChanged()
        self.format.setCurrentIndex(max(0, self.format.findData(
            options.format)))
        self._FormatChanged()
        self.encoding.setCurrentIndex(max(0, self.encoding.findText(
            options.encoding)))
        self.quality.SetValue(options.quality)
        self._customSize = (options.width, options.height)
        blocked = self.displaySize.blockSignals(True)
        self.displaySize.setCurrentIndex(max(0, self.displaySize.findData(
            options.displaySize)))
        self.displaySize.blockSignals(blocked)
        self._SizeChanged()
        self.scale.SetValue(options.scale)
        self.framePadding.SetValue(options.framePadding)
        self.removeTemporaryFiles.setChecked(options.removeTemporaryFiles)
        self.saveToFile.setChecked(options.saveToFile)
        self.movieFile.setText(options.movieFile)
        self._UpdateEnabled()

    def Options(self):
        return model.PlayblastOptions(
            view=self.view.isChecked(),
            showOrnaments=self.showOrnaments.isChecked(),
            offScreen=self.offScreen.isChecked(),
            timeRange=(model.START_END if self.startEnd.isChecked()
                       else model.TIME_SLIDER),
            startTime=self.startTime.value(),
            endTime=self.endTime.value(),
            format=self._Formats(),
            encoding=self.encoding.currentText(),
            quality=self.quality.Value(),
            displaySize=self.displaySize.currentData(),
            width=self._customSize[0],
            height=self._customSize[1],
            scale=self.scale.Value(),
            framePadding=self.framePadding.Value(),
            removeTemporaryFiles=self.removeTemporaryFiles.isChecked(),
            saveToFile=self.saveToFile.isChecked(),
            movieFile=self.movieFile.text().strip(),
        ).Normalized(self._imageEncodings)

    def _SaveSettings(self):
        SaveOptions(self.Options())

    def _ResetSettings(self):
        self.SetOptions(_Available(model.PlayblastOptions()))

    def _Apply(self):
        options = self.Options()
        SaveOptions(options)
        return Playblast(self._api, options, self.parentWidget())

    def _Playblast(self):
        options = self.Options()
        SaveOptions(options)
        self.close()
        return Playblast(self._api, options, self.parentWidget())


class FlipbookViewer(QtWidgets.QWidget):
    """Plays a written image sequence at its frame rate."""

    _CACHE_FRAMES = 96

    def __init__(self, paths, numbers, rate, title, parent=None):
        super(FlipbookViewer, self).__init__(parent, QtCore.Qt.Window)
        self.setObjectName("rigExecPlayblastViewer")
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle(title)
        self._paths = list(paths)
        self._numbers = list(numbers)
        self._cache = collections.OrderedDict()
        self._index = 0

        self._picture = QtWidgets.QLabel(self)
        self._picture.setAlignment(QtCore.Qt.AlignCenter)
        self._picture.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                    QtWidgets.QSizePolicy.Ignored)
        self._picture.setStyleSheet("background-color: black;")
        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self._slider.setRange(0, max(0, len(self._paths) - 1))
        self._frame = QtWidgets.QLabel(self)
        self._play = QtWidgets.QPushButton("Play", self)
        self._play.setCheckable(True)
        self._loop = QtWidgets.QCheckBox("Loop", self)
        self._loop.setChecked(True)
        self._timer = QtCore.QTimer(self)
        self._timer.setTimerType(QtCore.Qt.PreciseTimer)
        self._timer.setInterval(max(1, int(round(1000.0 / max(rate, 1e-3)))))

        controls = QtWidgets.QHBoxLayout()
        for text, step in (("|<", None), ("<", -1)):
            controls.addWidget(self._StepButton(text, step))
        controls.addWidget(self._play)
        for text, step in ((">", 1), (">|", None)):
            controls.addWidget(self._StepButton(text, step, last=True))
        controls.addWidget(self._slider, 1)
        controls.addWidget(self._frame)
        controls.addWidget(self._loop)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addWidget(self._picture, 1)
        layout.addLayout(controls)

        self._slider.valueChanged.connect(self._Show)
        self._play.toggled.connect(self._Toggle)
        self._timer.timeout.connect(self._Advance)
        self._FitWindow()
        self._Show(0)
        if len(self._paths) > 1:
            self._play.setChecked(True)

    def _StepButton(self, text, step, last=False):
        button = QtWidgets.QToolButton(self)
        button.setText(text)
        if step is None:
            target = (lambda: len(self._paths) - 1) if last else (lambda: 0)
            button.clicked.connect(lambda: self._Seek(target()))
        else:
            button.clicked.connect(lambda: self._Seek(self._index + step))
        return button

    def _FitWindow(self):
        pixmap = self._Pixmap(0)
        if pixmap is None or pixmap.isNull():
            self.resize(640, 400)
            return
        screen = self.screen() if hasattr(self, "screen") else None
        available = (screen.availableGeometry().size() if screen is not None
                     else QtCore.QSize(1920, 1080))
        width = min(pixmap.width() + 8, int(available.width() * 0.9))
        height = min(pixmap.height() + 48, int(available.height() * 0.9))
        self.resize(width, height)

    def _Pixmap(self, index):
        pixmap = self._cache.get(index)
        if pixmap is None:
            pixmap = QtGui.QPixmap(self._paths[index])
            self._cache[index] = pixmap
            while len(self._cache) > self._CACHE_FRAMES:
                self._cache.popitem(last=False)
        else:
            self._cache.move_to_end(index)
        return pixmap

    def _Show(self, index):
        if not self._paths:
            return
        self._index = max(0, min(index, len(self._paths) - 1))
        blocked = self._slider.blockSignals(True)
        self._slider.setValue(self._index)
        self._slider.blockSignals(blocked)
        self._frame.setText("%d" % self._numbers[self._index])
        self._Paint()

    def _Paint(self):
        pixmap = self._Pixmap(self._index)
        if pixmap is None or pixmap.isNull():
            self._picture.clear()
            return
        size = self._picture.size()
        if pixmap.width() > size.width() or pixmap.height() > size.height():
            pixmap = pixmap.scaled(size, QtCore.Qt.KeepAspectRatio,
                                   QtCore.Qt.SmoothTransformation)
        self._picture.setPixmap(pixmap)

    def _Seek(self, index):
        self._play.setChecked(False)
        self._Show(index)

    def _Toggle(self, playing):
        self._play.setText("Pause" if playing else "Play")
        if playing:
            if self._index >= len(self._paths) - 1:
                self._Show(0)
            self._timer.start()
        else:
            self._timer.stop()

    def _Advance(self):
        index = self._index + 1
        if index >= len(self._paths):
            if not self._loop.isChecked():
                self._play.setChecked(False)
                return
            index = 0
        self._Show(index)

    def resizeEvent(self, event):
        super(FlipbookViewer, self).resizeEvent(event)
        self._Paint()

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_Space:
            self._play.toggle()
        elif key == QtCore.Qt.Key_Left:
            self._Seek(self._index - 1)
        elif key == QtCore.Qt.Key_Right:
            self._Seek(self._index + 1)
        elif key == QtCore.Qt.Key_Home:
            self._Seek(0)
        elif key == QtCore.Qt.Key_End:
            self._Seek(len(self._paths) - 1)
        else:
            super(FlipbookViewer, self).keyPressEvent(event)
