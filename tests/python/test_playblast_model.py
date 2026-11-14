#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/playblastModel.py: the Playblast
Options fields, the frames and size of a run, Maya's file naming, and the
ffmpeg command for each movie encoding.

No Qt and no usdview; the render settings lookup runs on an in-memory
stage.

Usage: test_playblast_model.py
"""
import os

# Sibling module: this script's own directory is sys.path[0].
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Usd, UsdRender  # noqa: E402

import playblastModel as model  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Drange(start, stop, step):
    # Usdviewq.common.Drange, copied so this test needs no Qt.
    frames = [start]
    n = 1
    while start + n * step <= stop:
        frames.append(start + n * step)
        n += 1
    return frames


def TestOptions():
    options = model.PlayblastOptions()
    _Check(options.view and options.showOrnaments and not options.offScreen,
           "Maya's check box defaults")
    _Check(options.quality == 70 and options.scale == 0.5 and
           options.framePadding == 4, "Maya's number defaults")
    round_trip = model.PlayblastOptions.FromJson(
        options.Copy(format=model.FORMAT_IMAGE, encoding="jpg",
                     quality=90).ToJson())
    _Check(round_trip.ToDict() == options.Copy(
        format=model.FORMAT_IMAGE, encoding="jpg", quality=90).ToDict(),
        "options survive JSON")
    _Check(model.PlayblastOptions.FromJson("not json").ToDict() ==
           options.ToDict(), "unreadable settings give the defaults")
    _Check(model.PlayblastOptions.FromJson('{"bogus": 1, "quality": 5}')
           .quality == 5, "unknown keys are ignored")
    clamped = options.Copy(quality=250, scale=4, framePadding=-2,
                           width=0, format="mpeg", timeRange="x",
                           startTime=30, endTime=10).Normalized()
    _Check(clamped.quality == 100 and clamped.scale == 1.0 and
           clamped.framePadding == 0 and clamped.width == 1,
           "numbers clamp into range")
    _Check(clamped.format == model.FORMAT_QT and
           clamped.timeRange == model.TIME_SLIDER,
           "unknown choices fall back")
    _Check(clamped.endTime == 30, "end never precedes start")
    swapped = options.Copy(format=model.FORMAT_AVI,
                           encoding=model.PRORES_422).Normalized()
    _Check(swapped.encoding == model.NONE_AVI,
           "an encoding the format lacks takes the format's first")
    images = options.Copy(format=model.FORMAT_IMAGE,
                          encoding="webp").Normalized(("jpg", "png"))
    _Check(images.encoding == "png",
           "an image encoding Qt cannot write is replaced")


def TestFrames():
    for start, end, step in ((1, 24, 1), (1, 3, 0.3), (-5, 5, 2.5)):
        _Check(model.FrameList(start, end, step) ==
               _Drange(start, end, step), "matches usdview's Drange")
    _Check(model.FrameList(5, 1, 1) == [], "empty range")
    slider = [1.0, 2.0, 3.0]
    timeSlider = model.PlayblastOptions()
    _Check(model.PlayblastFrames(timeSlider, slider, 2.0, 1.0) == slider,
           "Time Slider takes usdview's samples")
    _Check(model.PlayblastFrames(timeSlider, [], 7.0, 1.0) == [7.0],
           "a stage without samples blasts the current frame")
    startEnd = timeSlider.Copy(timeRange=model.START_END, startTime=10,
                               endTime=12)
    _Check(model.PlayblastFrames(startEnd, slider, 2.0, 0.5) ==
           [10, 10.5, 11, 11.5, 12], "Start/End at usdview's step")
    _Check(model.FrameNumbers([1.0, 2.0, 3.0]) == [1, 2, 3],
           "whole frames keep their numbers")
    _Check(model.FrameNumbers([1.0, 1.5, 2.0]) == [0, 1, 2],
           "subframes number by position")
    _Check(model.FrameRate(24, 1, 30) == 24 and
           model.FrameRate(24, 2, 30) == 12, "rate from step")


def TestSize():
    options = model.PlayblastOptions()
    _Check(model.OutputSize(options, (1201, 801)) == (600, 400),
           "window size times scale, even for a movie")
    image = options.Copy(format=model.FORMAT_IMAGE, encoding="png")
    _Check(model.OutputSize(image, (1201, 801)) == (600, 400),
           "image size times scale")
    full = image.Copy(scale=1.0)
    _Check(model.OutputSize(full, (1201, 801)) == (1201, 801),
           "odd sizes stay for images")
    custom = full.Copy(displaySize=model.SIZE_CUSTOM, width=640, height=360)
    _Check(model.OutputSize(custom, (1201, 801)) == (640, 360), "custom")
    settings = full.Copy(displaySize=model.SIZE_RENDER_SETTINGS)
    _Check(model.OutputSize(settings, (100, 50), (2048, 858)) ==
           (2048, 858), "render settings")
    _Check(model.OutputSize(settings, (100, 50), None) == (100, 50),
           "no render settings falls back to the window")
    _Check(model.RenderSize(options, (1201, 801)) == (1201, 801),
           "below full scale, frames render at the display size")
    _Check(model.RenderSize(full, (1201, 801)) == (1201, 801) and
           model.RenderSize(full.Copy(format=model.FORMAT_QT,
                                      encoding=model.H264),
                            (1201, 801)) == (1200, 800),
           "at full scale, frames render at the output size")


def TestNames():
    _Check(model.ImagePath("/a/blast", 7, 4, "png") == "/a/blast.0007.png",
           "Maya's name.####.ext")
    _Check(model.ImagePath("b", -3, 4, "jpg") == "b.-003.jpg",
           "negative frames")
    _Check(model.StageStem("C:\\shots\\sh010.anim.usda") == "sh010",
           "stem from a layer path")
    _Check(model.StageStem("anon:0x1234:tmp.usda") == "playblast",
           "anonymous layers")
    options = model.PlayblastOptions()
    _Check(model.OutputBase(options, "sh", "/tmp/run", "/movies") ==
           os.path.join("/tmp/run", "sh"), "temp when not saving")
    saving = options.Copy(saveToFile=True)
    _Check(model.OutputBase(saving, "sh", "/tmp/run", "/movies") ==
           os.path.join("/movies", "sh"), "default movie directory")
    named = saving.Copy(movieFile="/out/final.mov")
    _Check(model.OutputBase(named, "sh", "/tmp", "/m") == "/out/final",
           "a typed extension is replaced by the format's")
    _Check(model.MoviePath("/out/final", model.FORMAT_AVI) ==
           "/out/final.avi", "avi extension")


def TestFfmpeg():
    for format in model.MOVIE_FORMATS:
        for encoding in model.MOVIE_ENCODINGS[format]:
            args = model.CodecArguments(format, encoding, 70)
            _Check(args[0] == "-c:v", "codec first for %s" % encoding)
    best = model.CodecArguments(model.FORMAT_QT, model.H264, 100)
    worst = model.CodecArguments(model.FORMAT_QT, model.H264, 0)
    _Check(best[best.index("-crf") + 1] == "12" and
           worst[worst.index("-crf") + 1] == "40",
           "quality maps onto the crf scale")
    _Check("-movflags" in best, "qt movies are streamable")
    _Check("-movflags" not in model.CodecArguments(
        model.FORMAT_AVI, model.H264, 70), "avi takes no mov flags")
    for encoding, matrix, tag in ((model.H264, "bt709", "bt709"),
                                  (model.PRORES_422, "bt709", "bt709"),
                                  (model.MJPEG, "bt601", "bt470bg")):
        args = model.CodecArguments(model.FORMAT_QT, encoding, 70)
        _Check(args[args.index("-colorspace") + 1] == tag and
               "out_color_matrix=" + matrix in args[args.index("-vf") + 1],
               "%s converts with and tags %s" % (encoding, matrix))
    _Check("-colorspace" not in model.CodecArguments(
        model.FORMAT_QT, model.PNG_MOVIE, 70), "RGB encodings stay RGB")
    command = model.FfmpegCommand("ffmpeg", (640, 360), 23.976,
                                  model.FORMAT_QT, model.H264, 70,
                                  "/o/x.mov")
    _Check(command[0] == "ffmpeg" and command[-1] == "/o/x.mov",
           "program and output")
    _Check(command[command.index("-framerate") + 1] == "23.976",
           "fractional rate")
    _Check(command[command.index("-video_size") + 1] == "640x360" and
           command[command.index("-f") + 1] == "rawvideo" and
           command[command.index("-i") + 1] == "-",
           "raw frames on standard input")
    _Check(model.RawPixelFormat("little") == "bgr0" and
           model.RawPixelFormat("big") == "0rgb",
           "Format_RGB32 byte order")
    _Check(model.FindFfmpeg({model.FFMPEG_ENV: "/no/such/ffmpeg"},
                            lambda name: "/usr/bin/ffmpeg") is None,
           "an explicit path that is missing is not replaced by PATH")
    _Check(model.FindFfmpeg({}, lambda name: "/usr/bin/ffmpeg") ==
           "/usr/bin/ffmpeg", "PATH")


def TestRenderSettings():
    stage = Usd.Stage.CreateInMemory()
    _Check(model.RenderSettingsSize(stage) is None, "no settings")
    settings = UsdRender.Settings.Define(stage, "/Render/Settings")
    settings.CreateResolutionAttr(Gf.Vec2i(1280, 536))
    _Check(model.RenderSettingsSize(stage) == (1280, 536),
           "found by type when the stage names none")
    other = UsdRender.Settings.Define(stage, "/Other")
    other.CreateResolutionAttr(Gf.Vec2i(320, 240))
    stage.SetMetadata("renderSettingsPrimPath", "/Other")
    _Check(model.RenderSettingsSize(stage) == (320, 240),
           "the stage's renderSettingsPrimPath wins")


def main():
    TestOptions()
    TestFrames()
    TestSize()
    TestNames()
    TestFfmpeg()
    TestRenderSettings()
    print("test_playblast_model: OK")


if __name__ == "__main__":
    main()
