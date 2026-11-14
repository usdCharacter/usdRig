---
title: Frame range and playblast
---

# Frame range and playblast

RigExec adds two things to usdview's timeline: start and end fields for the
stage's frame range, and a **Playblast** button beside **Play**.

## Frame range

The timeline row reads like Maya's range slider:

| Field | What it is |
|---|---|
| Outer left | Start of the stage's frame range (`startTimeCode`). |
| Inner left | Start of usdview's playback range. |
| Inner right | End of usdview's playback range. |
| Outer right | End of the stage's frame range (`endTimeCode`). |

Type a frame into an outer field and press ++enter++. The stage's start and
end are written as one undoable edit, both into the same layer: the session
layer when it already holds a range, otherwise the edit target if it is the
session or root layer, otherwise the root layer. Typing a start after the end,
or an end before the start, moves the other end with it.

usdview follows at once. Playback range ends that sat on the old stage ends
move with them, and any other end is kept and clamped into the new range. The
slider, the range fields and the current frame update, and playback turns on
or off with the number of frames. A stage with no authored range gets one
from the fields, which makes it playable.

The same update happens whenever the stage's start, end or frame rate changes
by any route: undo and redo, a Python script, or a layer reload.

## Playblast

Click **Playblast** to render the playback range with the saved options.
++shift++-click it, or click the option box beside it, for **Playblast
Options**. As in Maya, **Playblast** saves the options, runs and closes the
window, **Apply** saves and runs, and **Edit ▸ Save Settings** and **Reset
Settings** store or restore them without running.

Each frame is rendered by the viewport itself, with its renderer, render
mode, lights, camera and the evaluated rig pose, at the output size.
Manipulators, the view cube and other panels over the viewport are never in
the frame. Frames step as they do in playback, and the frame you were on is
restored afterwards. **Cancel** in the progress window stops the run.

| Option | Behaviour |
|---|---|
| View | Shows the result when the run ends: image sequences in a flipbook (++space++ plays, arrow keys step), movies in the system's player. |
| Show ornaments | Keeps usdview's HUD, origin axis, camera mask outline, reticles and camera guides in the frames. |
| Render offscreen | Leaves the viewport still while frames render. Off, the viewport shows each frame. Either way frames can be larger than the viewport. |
| Time range | **Time Slider** renders usdview's playback range. **Start/End** renders the typed range at usdview's step size. |
| Format | **qt** (`.mov`), **avi**, or **image** for an image sequence. |
| Encoding | qt: H.264, Apple ProRes 422, Apple ProRes 4444, Photo - JPEG, PNG. avi: none, MJPEG, H.264. image: the formats Qt can write, such as jpg, png, tif. |
| Quality | 0 to 100, for H.264, JPEG and WebP. |
| Display size | **From Window** (the viewport's pixels), **From Render Settings** (the resolution of the stage's render settings) or **Custom**. |
| Scale | 0.10 to 1.00 times the display size. |
| Frame padding | Digits in image numbers: `name.0001.png`. |
| Remove temporary files | Deletes the frames a movie was encoded from. |
| Save to file | Writes to **Movie file**. Off, the result goes to a temporary folder. An empty **Movie file** uses the system's Videos folder and the stage's name. |

Whole frames are numbered by frame; subframe steps are numbered from 0.
Movie sizes are rounded down to even numbers, which the encoders need.

The movie formats need [ffmpeg](https://ffmpeg.org): set `RIGEXEC_FFMPEG` to
its path, or put it on `PATH`. Without it, **qt** and **avi** are unavailable
and saved movie options fall back to an image sequence. Maya's
**Multi-camera output** and **Use sequence time** have no usdview
counterpart and are left out.
