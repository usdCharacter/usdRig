# Graph editor

Open **RigExec → Animation Editors → Graph Editor** to edit scalar `Ts`
animation splines on the selected prims or properties. The editor supports
`double`, `float`, and `half` curves. Vector attributes use time samples and
do not appear as editable scalar splines.

## Editing

Select curves in the list to isolate them. Drag keys to change time and value.
Drag a tangent handle to move its end: the angle sets the slope and the length
sets the width. Unified tangents share the angle; each side keeps its own
length. Turn **Weighted** off to lock the length so a drag only rotates the
handle. Hermite curves fix the width, so their handles always only rotate.
Moved keys retain their order. Insert Key adds a knot without replacing the rest of the curve.
New keys use Auto tangents; editing an existing key preserves its tangent setup.

A drag authors nothing while it is in flight. The editor draws the edited
curves and sends their values at the current frame to Hydra, so the viewport
follows the drag; other frames update at the commit. Releasing the drag queues
one edit for the next UI event-loop turn, authored to the edit target that was
active at the press; Escape during the drag cancels it. The preview stays
visible until the queued commit completes. Undo, another edit, a key press in
the editor, a frame change, a selection change, and closing the editor commit
a queued edit first. Buttons and numeric fields author immediately.

Edits use the same undo stack as the viewport tools. They are authored to the
active edit target. Broken tangent state is stored in
the knot's `rigExec` custom data because equal slopes do not imply unified
tangents. Unify copies the outgoing slope to the incoming side.

## Time and navigation

The frame ruler runs along the top of the plot. The current time is a line
through the plot with a tab in the ruler that shows the frame number. Press or
drag anywhere in the ruler to scrub; the cursor changes over it. Hold `K` and
drag with the left button to scrub from anywhere in the plot without touching
keys. Hold `K` and drag with the middle button to move the editor's time marker
alone; the viewport moves to that frame when the button is released.
Scrubbing snaps to whole frames inside the stage's frame range, because the
viewer's timeline only holds those frames.

`Alt` + middle drag pans and `Alt` + right drag zooms, about the pressed
point. Adding `Shift` holds either one to the axis the drag first moves along.
The wheel zooms both axes about the cursor. Frames outside the stage range are
shaded.

## Extrapolation

The pre/post Infinity controls apply to the visible curves.

| Setting | USD mode | Result |
|---|---|---|
| Constant | `TsExtrapHeld` | Hold the endpoint value |
| Linear | `TsExtrapLinear` | Extend the endpoint slope |
| Cycle | `TsExtrapLoopReset` | Repeat the curve without a value offset |
| Cycle with Offset | `TsExtrapLoopRepeat` | Offset each repetition to join endpoints |
| Oscillate | `TsExtrapLoopOscillate` | Alternate forward and backward repetitions |

A loop on a single-key spline behaves as Held. Non-finite samples are not
drawn. Frame All fits the keys; a tangent overshoot may extend outside that view.

## Shortcuts

| Key | Action |
|---|---|
| `A` / `F` | Frame all / selected keys |
| `Home` | Frame the stage range |
| `K` + left drag | Scrub |
| `K` + middle drag | Move the time marker; the frame follows on release |
| `I` | Insert key |
| `Delete` / `Backspace` | Delete selected keys |
| `Escape` | Cancel a drag or clear selection |
| `Ctrl+Z` / `Ctrl+Shift+Z`, `Shift+Z`, `Ctrl+Y` | Undo / redo |

Shortcuts apply to the editor window and yield to text-entry widgets.
Clamped/Plateau tangents, audio, retiming tools, and Euler filtering are not
implemented. The Qt-free model is tested by `test_graph_model` and
`test_graph_screen`; `bin/test/run_testusdview_graph.sh` or `.bat` checks the UI.
