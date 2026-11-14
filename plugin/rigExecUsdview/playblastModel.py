#
# RigExec usdview playblast: the options behind the Playblast Options
# window and the pure parts of a run -- which frames, what size, which
# files, and the ffmpeg command that turns frames into a movie. Qt-free;
# playblastUI.py renders, writes and shows.
#
# The options follow Maya's Playblast Options window field for field.
# Multi-camera output and Use sequence time have no usdview counterpart and
# are left out. Movie formats are encoded by ffmpeg (RIGEXEC_FFMPEG, else
# ffmpeg on PATH); image sequences need nothing but Qt.
#
import json
import math
import os
import re
import shutil
import sys

FORMAT_QT = "qt"
FORMAT_AVI = "avi"
FORMAT_IMAGE = "image"
FORMATS = (FORMAT_QT, FORMAT_AVI, FORMAT_IMAGE)
MOVIE_FORMATS = (FORMAT_QT, FORMAT_AVI)

TIME_SLIDER = "timeSlider"
START_END = "startEnd"
TIME_RANGES = (TIME_SLIDER, START_END)

SIZE_WINDOW = "window"
SIZE_RENDER_SETTINGS = "renderSettings"
SIZE_CUSTOM = "custom"
DISPLAY_SIZES = (SIZE_WINDOW, SIZE_RENDER_SETTINGS, SIZE_CUSTOM)

H264 = "H.264"
PRORES_422 = "Apple ProRes 422"
PRORES_4444 = "Apple ProRes 4444"
PHOTO_JPEG = "Photo - JPEG"
PNG_MOVIE = "PNG"
NONE_AVI = "none"
MJPEG = "MJPEG"

MOVIE_ENCODINGS = {
    FORMAT_QT: (H264, PRORES_422, PRORES_4444, PHOTO_JPEG, PNG_MOVIE),
    FORMAT_AVI: (NONE_AVI, MJPEG, H264),
}
# In Maya's order; the UI offers those Qt can write.
IMAGE_ENCODINGS = ("jpg", "png", "tif", "bmp", "webp")
MOVIE_EXTENSIONS = {FORMAT_QT: ".mov", FORMAT_AVI: ".avi"}

# Encodings whose output quality the Quality field controls.
QUALITY_ENCODINGS = frozenset((H264, PHOTO_JPEG, MJPEG, "jpg", "webp"))

# Frames a movie was encoded from, kept when Remove temporary files is off.
INTERMEDIATE_ENCODING = "png"

SCALE_RANGE = (0.1, 1.0)
PADDING_RANGE = (0, 10)
SIZE_LIMIT = 16384

FFMPEG_ENV = "RIGEXEC_FFMPEG"


def _Clamp(value, low, high):
    return max(low, min(high, value))


def DefaultEncoding(format, imageEncodings=IMAGE_ENCODINGS):
    if format == FORMAT_IMAGE:
        return "png" if "png" in imageEncodings else imageEncodings[0]
    return MOVIE_ENCODINGS[format][0]


def Encodings(format, imageEncodings=IMAGE_ENCODINGS):
    if format == FORMAT_IMAGE:
        return tuple(imageEncodings)
    return MOVIE_ENCODINGS[format]


class PlayblastOptions(object):
    """One playblast's settings, Maya's field names in camelCase."""

    _DEFAULTS = (
        ("view", True),
        ("showOrnaments", True),
        ("offScreen", False),
        ("timeRange", TIME_SLIDER),
        ("startTime", 1.0),
        ("endTime", 24.0),
        ("format", FORMAT_QT),
        ("encoding", H264),
        ("quality", 70),
        ("displaySize", SIZE_WINDOW),
        ("width", 1920),
        ("height", 1080),
        ("scale", 0.5),
        ("framePadding", 4),
        ("removeTemporaryFiles", True),
        ("saveToFile", False),
        ("movieFile", ""),
    )
    FIELDS = tuple(name for name, _ in _DEFAULTS)

    def __init__(self, **values):
        for name, default in self._DEFAULTS:
            setattr(self, name, default)
        for name, value in values.items():
            if name not in self.FIELDS:
                raise TypeError("unknown playblast option %r" % name)
            setattr(self, name, value)

    def Copy(self, **values):
        fields = self.ToDict()
        fields.update(values)
        return PlayblastOptions(**fields)

    def ToDict(self):
        return {name: getattr(self, name) for name in self.FIELDS}

    def ToJson(self):
        return json.dumps(self.ToDict(), sort_keys=True)

    @classmethod
    def FromJson(cls, text):
        """Saved options; unknown keys are ignored, and anything unreadable
        gives the defaults."""
        try:
            data = json.loads(text) if text else {}
        except (TypeError, ValueError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        fields = {name: data[name] for name in cls.FIELDS if name in data}
        try:
            return cls(**fields).Normalized()
        except (TypeError, ValueError):
            return cls()

    def Normalized(self, imageEncodings=IMAGE_ENCODINGS):
        """A copy with every field coerced into its valid range."""
        defaults = PlayblastOptions()
        out = PlayblastOptions()
        for name in ("view", "showOrnaments", "offScreen",
                     "removeTemporaryFiles", "saveToFile"):
            setattr(out, name, bool(getattr(self, name)))
        out.timeRange = (self.timeRange if self.timeRange in TIME_RANGES
                         else defaults.timeRange)
        for name in ("startTime", "endTime"):
            value = float(getattr(self, name))
            setattr(out, name, value if math.isfinite(value)
                    else getattr(defaults, name))
        if out.endTime < out.startTime:
            out.endTime = out.startTime
        out.format = self.format if self.format in FORMATS else defaults.format
        encodings = Encodings(out.format, imageEncodings)
        out.encoding = (self.encoding if self.encoding in encodings
                        else DefaultEncoding(out.format, imageEncodings))
        out.quality = int(_Clamp(int(round(float(self.quality))), 0, 100))
        out.displaySize = (self.displaySize if self.displaySize in
                           DISPLAY_SIZES else defaults.displaySize)
        out.width = int(_Clamp(int(self.width), 1, SIZE_LIMIT))
        out.height = int(_Clamp(int(self.height), 1, SIZE_LIMIT))
        scale = float(self.scale)
        out.scale = _Clamp(scale if math.isfinite(scale) else defaults.scale,
                           *SCALE_RANGE)
        out.framePadding = int(_Clamp(int(self.framePadding),
                                      *PADDING_RANGE))
        out.movieFile = str(self.movieFile or "")
        return out


def IsMovie(format):
    return format in MOVIE_FORMATS


def UsesQuality(encoding):
    return encoding in QUALITY_ENCODINGS


def FrameList(start, end, step):
    """usdview's own frame list (Usdviewq.common.Drange), so frames on the
    slider's grid compare equal to its time samples."""
    if end < start:
        return []
    if step <= 0:
        return [start]
    frames = [start]
    n = 1
    while start + n * step <= end:
        frames.append(start + n * step)
        n += 1
    return frames


def PlayblastFrames(options, sliderFrames, currentFrame, step):
    """The frames a playblast renders.

    Time Slider takes usdview's time samples (its playback range), or the
    current frame on a stage without one; Start/End steps through the
    typed range at usdview's step.
    """
    if options.timeRange == START_END:
        return FrameList(options.startTime, options.endTime, step)
    frames = list(sliderFrames)
    return frames if frames else [currentFrame]


def FrameRate(timeCodesPerSecond, step, fallback):
    """Frames per second of the written frames: one frame every `step`
    time codes."""
    if timeCodesPerSecond > 0 and step > 0:
        return timeCodesPerSecond / step
    return fallback if fallback > 0 else 24.0


def BaseSize(options, windowSize, renderSettingsSize=None):
    """Display size before Scale, in pixels."""
    if options.displaySize == SIZE_CUSTOM:
        return (options.width, options.height)
    if (options.displaySize == SIZE_RENDER_SETTINGS and
            renderSettingsSize is not None):
        return tuple(renderSettingsSize)
    return tuple(windowSize)


def OutputSize(options, windowSize, renderSettingsSize=None):
    """The rendered image size: display size times Scale.

    Movie encoders subsample chroma, so a movie's size is rounded down to
    even numbers.
    """
    width, height = BaseSize(options, windowSize, renderSettingsSize)
    width = int(_Clamp(int(round(width * options.scale)), 1, SIZE_LIMIT))
    height = int(_Clamp(int(round(height * options.scale)), 1, SIZE_LIMIT))
    if IsMovie(options.format):
        width = max(2, width - width % 2)
        height = max(2, height - height % 2)
    return (width, height)


def RenderSize(options, windowSize, renderSettingsSize=None):
    """The size frames render at before they are filtered to OutputSize.

    Below full Scale, frames render at the display size and are filtered
    down, so a smaller playblast is the viewport's picture made smaller
    rather than a coarser render.
    """
    if options.scale >= SCALE_RANGE[1]:
        return OutputSize(options, windowSize, renderSettingsSize)
    width, height = BaseSize(options, windowSize, renderSettingsSize)
    return (int(_Clamp(int(width), 1, SIZE_LIMIT)),
            int(_Clamp(int(height), 1, SIZE_LIMIT)))


def FrameNumbers(frames):
    """The number each frame's file carries: the frame itself when every
    frame is whole, else its position (subframe steps)."""
    if all(abs(frame - round(frame)) < 1e-6 for frame in frames):
        return [int(round(frame)) for frame in frames]
    return list(range(len(frames)))


def ImagePath(base, number, padding, extension):
    """Maya's naming: base.0001.ext"""
    return "%s.%0*d.%s" % (base, padding, number, extension)


_UNSAFE = re.compile(r"[^A-Za-z0-9_.-]+")


def SafeName(text, fallback="playblast"):
    name = _UNSAFE.sub("_", text or "").strip("._")
    return name or fallback


def StageStem(identifier):
    """A file-name stem from a root layer identifier."""
    name = os.path.basename(str(identifier or "").replace("\\", "/"))
    stem = name.split(".")[0] if not name.startswith("anon:") else ""
    return SafeName(stem)


def _StripExtension(path, extensions):
    root, extension = os.path.splitext(path)
    return root if extension.lower() in extensions else path


def OutputBase(options, stem, tempDirectory, defaultDirectory):
    """Where the result goes, without frame number or extension.

    Save to file writes the Movie file path (or the default directory when
    it is empty); otherwise the run goes to `tempDirectory`.
    """
    if not options.saveToFile:
        return os.path.join(tempDirectory, stem)
    path = os.path.expanduser(os.path.expandvars(options.movieFile.strip()))
    if not path:
        return os.path.join(defaultDirectory, stem)
    known = set(MOVIE_EXTENSIONS.values())
    known.update("." + encoding for encoding in IMAGE_ENCODINGS)
    known.update((".jpeg", ".tiff"))
    return _StripExtension(path, known)


def MoviePath(base, format):
    return base + MOVIE_EXTENSIONS[format]


def _QualityScale(quality, best, worst):
    """Map 0-100 onto an encoder scale where `best` is quality 100."""
    return int(round(worst + (best - worst) * _Clamp(quality, 0, 100) / 100.0))


_COLORSPACE_TAGS = {"bt709": "bt709", "bt601": "bt470bg"}


def _Yuv(pixelFormat, matrix, colorRange):
    """Convert the sRGB frames to YUV with `matrix` and tag it. Untagged
    H.264 and ProRes are decoded with BT.601 or BT.709 by frame size; JPEG
    is always decoded as BT.601 (JFIF)."""
    return ["-vf", "scale=out_color_matrix=%s:out_range=%s:"
            "flags=accurate_rnd+full_chroma_int" % (matrix, colorRange),
            "-pix_fmt", pixelFormat, "-color_range", colorRange,
            "-colorspace", _COLORSPACE_TAGS[matrix],
            "-color_primaries", "bt709", "-color_trc", "iec61966-2-1"]


def CodecArguments(format, encoding, quality):
    """ffmpeg output arguments for one movie encoding."""
    if encoding == H264:
        args = (["-c:v", "libx264", "-preset", "medium",
                 "-crf", str(_QualityScale(quality, 12, 40))]
                + _Yuv("yuv420p", "bt709", "tv"))
    elif encoding == PRORES_422:
        args = (["-c:v", "prores_ks", "-profile:v", "2"]
                + _Yuv("yuv422p10le", "bt709", "tv"))
    elif encoding == PRORES_4444:
        args = (["-c:v", "prores_ks", "-profile:v", "4"]
                + _Yuv("yuv444p10le", "bt709", "tv"))
    elif encoding in (PHOTO_JPEG, MJPEG):
        args = (["-c:v", "mjpeg", "-q:v", str(_QualityScale(quality, 2, 31))]
                + _Yuv("yuvj420p", "bt601", "pc"))
    elif encoding == PNG_MOVIE:
        args = ["-c:v", "png", "-pix_fmt", "rgb24"]
    elif encoding == NONE_AVI:
        args = ["-c:v", "rawvideo", "-pix_fmt", "bgr24"]
    else:
        raise ValueError("unknown movie encoding %r" % encoding)
    if format == FORMAT_QT:
        args += ["-movflags", "+faststart"]
    return args


def FormatNumber(value):
    """A frame or rate as text: whole numbers without a decimal point."""
    if abs(value - round(value)) < 1e-6:
        return str(int(round(value)))
    return "%.6g" % value


def RawPixelFormat(byteorder=sys.byteorder):
    """ffmpeg's name for a 32-bit 0xffRRGGBB QImage (Format_RGB32) in
    memory."""
    return "bgr0" if byteorder == "little" else "0rgb"


def FfmpegCommand(ffmpeg, size, rate, format, encoding, quality, output):
    """The ffmpeg command that encodes raw Format_RGB32 frames of `size`,
    written to its standard input, into `output`."""
    return ([ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", RawPixelFormat(),
             "-video_size", "%dx%d" % tuple(size),
             "-framerate", FormatNumber(rate), "-i", "-"]
            + CodecArguments(format, encoding, quality) + [output])


def FindFfmpeg(environ=None, which=shutil.which):
    """The ffmpeg executable: RIGEXEC_FFMPEG when it names a file, else
    ffmpeg on PATH, else None."""
    environ = os.environ if environ is None else environ
    explicit = environ.get(FFMPEG_ENV)
    if explicit:
        return explicit if os.path.isfile(explicit) else None
    return which("ffmpeg")


def RenderSettingsSize(stage):
    """The resolution of the stage's render settings, or None.

    The stage's renderSettingsPrimPath when it names one, else the first
    RenderSettings prim on the stage.
    """
    from pxr import Usd, UsdRender
    settings = UsdRender.Settings.GetStageRenderSettings(stage)
    if not settings:
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdRender.Settings):
                settings = UsdRender.Settings(prim)
                break
    if not settings:
        return None
    resolution = settings.GetResolutionAttr().Get()
    if resolution is None:
        return None
    width, height = int(resolution[0]), int(resolution[1])
    if width < 1 or height < 1:
        return None
    return (width, height)
