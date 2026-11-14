---
title: Frame range
---

# Frame range

RigExec adds start and end fields for the stage's frame range to usdview's
timeline.

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
