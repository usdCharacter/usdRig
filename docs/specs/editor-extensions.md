# Editor extensions

A host application can take over input routing, undo, and the document edit
target of the RigExec `usdview` editors through one hook module,
`plugin/rigExecUsdview/editorExtension.py`. The hook is generic: it names no
host and reads no environment setting.

## The one-switch rule

The single switch is "an extension is registered and answers `OwnsInput(api)`
with True". When it does, the session's editors install no Qt event filters
and no Undo or Redo shortcuts. The host forwards input to the editors' public
handlers (below). With no extension registered, every function returns its
do-nothing answer and every editor runs its own input handling unchanged.

Editors call the module functions, never the extension object. Each call is
guarded: an extension method that raises is reported once per method and
error type through `Tf.Warn`, and the editor falls back to its own behavior.
The context manager `DocumentEdit` returns is guarded too: a raising
`__enter__` is warned and replaced by a null context, so the commit goes to
the stage's edit target as without an extension; a raising `__exit__` is
warned and never masks an exception from the body.

Only the extension's methods are guarded. Once `OwnsInput` answers True an
editor is built without its own Qt filters and relies on the extension for
input. If `EditorAdded` then raises, a warning is posted and that editor is
left without input. All calls run on the Qt main thread.

## Registration and timing

`usdview` imports every plugin container, runs every `registerPlugins` (in
sorted plugin-name order), then every `configureView`, then opens the stage.
Register in the host's `registerPlugins`, before the first stage opens:
editors consult the hook as they are built.

```python
import editorExtension
editorExtension.Register(MyExtension())
```

- `Register(extension)` raises `TypeError` when the object lacks a method of
  `EditorExtension.METHODS`, and `RuntimeError` when a different extension is
  registered. Registering the same object again does nothing.
- `Unregister(extension=None)` removes the given extension, or whichever is
  registered.
- `Current()` returns the registered extension or None.

RigExec resolves its undo stack on first use, after every `registerPlugins`.
It builds the gizmo, marking menu, and hover picker when a stage is replaced,
the touch-pose controller from a timer started in `configureView`, and the
graph editor on demand. Its own retries (waiting for the stage view) stay in
RigExec and announce the editor through `EditorAdded` when it finally exists.
The host's own readiness is the host's concern; it can run it from
`StageReplaced`.

## EditorExtension

Subclass `editorExtension.EditorExtension` and override what the host takes
over. The base class holds the do-nothing answers. Each module-level function
of the same name forwards to the extension (a null context for
`DocumentEdit`).

| Method | Base answer | Meaning |
|---|---|---|
| `OwnsInput(usdviewApi)` | `False` | True: editors install no filters or Undo/Redo shortcuts. Must answer the same for a session's lifetime. |
| `UndoStack(usdviewApi)` | `None` | The session's undo stack, or None for RigExec's own. |
| `DocumentEdit(usdviewApi)` | `None` | A context manager selecting the layer for an edit RigExec authors outside the host's dispatch, or None. |
| `EditorAdded(usdviewApi, kind, editor)` | no-op | `editor` of `kind` exists now. May repeat for one editor. |
| `EditorRemoved(usdviewApi, kind, editor)` | no-op | The editor is going away. |
| `ShowEditor(usdviewApi, kind)` | `None` | Show the host's own home of editor `kind` and return its widget, or None for RigExec's window. Only `GRAPH_EDITOR` is asked. |
| `StageReplaced(usdviewApi)` | no-op | After RigExec handled a stage replacement; the stage may be None. |

## Kinds and input surfaces

`EditorAdded` and `EditorRemoved` pass one of these kinds. `editor` is the
object named below. Handlers are public methods; each runs the same body the
editor's own filter runs, so forwarded events behave identically.

| Kind | `editor` | Added when |
|---|---|---|
| `GIZMO` | `gizmoUI.GizmoController` | The viewport tools are installed. |
| `MARKING_MENU` | `markingMenuUI.ControlMarkingMenu` | The menu is built. |
| `HOVER_PICKER` | `pickerHoverUI.HoverPickerController` | The picker is built. |
| `HOVER_PICKER_TOGGLE` | `fn()` that shows or hides the hover picker | The hotkey is installed; the host binds its own key. |
| `GRAPH_EDITOR` | `graphEditorUI.GraphEditorPanel` | The panel is constructed. |
| `TOUCH_POSE` | `touchPoseUI.TouchPoseController` | The controller installs on the stage view; `EditorRemoved` on uninstall. |
| `TOUCH_POSE_VOICE` | `touchPoseVoice.TouchPoseVoice` | Voice input installs; the host binds the push-to-talk key (`PUSH_TO_TALK_KEY`) to its `Press` and `Release`. |

### Gizmo controller

Constructed with `installFilters=False` when input is yielded.

- `InstallsFilters()`, `HasView()`, `HasTarget()`
- `ForwardViewEvent(event)`: the stage-view filter for one event; True when consumed.
- `HandlePress(event)`: a viewport press; ends a text edit first, as a click does.
- `IsMarqueePending()`
- `HandleMove(event)`, `HandleRelease(event)`
- `HandleHover(point)`: pre-selection at physical pixel `point`; None means the pointer left.
- `HandleKeyRelease(event, key)`: `event` is a `QKeyEvent` or any object with `isAutoRepeat()`, `modifiers()`, and `key()`.
- `HandleDragKey(key)`
- `HandleToolKey(key, modifiers)`: `modifiers` are the Qt modifiers the key acts with.
- `CancelGesture()`: aborts a drag and drops a pending marquee.
- `ClearHolds()`

Also public: `toolbar`, `statusBar`, `usdviewApi`, `toolChangedHooks`,
`ReleaseToolbarRow()`, `IsToolbarRowReleased()`,
`ViewportToolbar.SetToolButtonsVisible()` and `ToolButtonsVisible()`,
`SetSnapHoldPrefix(prefix)`, `ViewportStatusBar.statusListeners`,
`FullText()`, `IsVisible()`, `IsDragging()`, `Tool()`, `SetTool()`,
`Orientation()`, `GroupPivot()`, `Status()`, `FlushReleasedEdit()`, and the
module functions `StageView(api)` and `GetController(api)`. See
[viewport gizmos](viewport-gizmos.md).

### Marking menu

Constructed with `installFilters=False` when input is yielded.

- `InstallsFilters()`, `HasView()`
- `HandleViewEvent(event)`
- `OpenForClicking(point, controls)`: opens at `point` with the press already released, so the menu stays up for a click; returns True when open.
- Existing: `HoveredControls`, `SelectedControls`, `Targets`, `Open`, `IsOpen`, `Cancel`, and `GetMenu(api)`.

### Hover picker

`InstallHotkey` registers no key when input is yielded; it announces
`HOVER_PICKER_TOGGLE` instead. The controller is built with
`installFilters=False`.

- `InstallsFilters()`, `IsShown()`
- `HandleViewEvent(event)`
- `CancelGesture()`, `ClearHover()`

### Touch pose

- `IsInteractive()`, `IsPressPending()`
- `GizmoOwnsAt(x, y)`: whether the gizmo owns the pixel.
- `HandleViewEvent(event)`
- `CancelGesture()`: ends a live stroke and clears the press and band.
- `LeaveView()`: sends the controller a pointer-left event.

### Graph editor

When input is yielded the panel installs no hotkey filter and no Undo/Redo
window shortcuts. The host routes the window's keys to
`GraphEditorPanel.HandleKey(key, modifiers=None)` and the release of a held
key (`K`) to `HandleKeyRelease(key)`. Also public: `HOTKEYS`,
`OwnsEvent`, `GetInstance`, and the embedding surface `SetEditHook`,
`EditHook`, `IsPreviewing`, `KeySplines`, `AuthoredSpline`, `DrawSplines`.
A drag released on the canvas commits on the next event-loop turn, so the
hook's `FinishEdit` follows the release by one turn. `HasPendingCommit()`
reports a queued commit and `FlushPendingCommit()` runs it at once;
`EndGesture()` commits a live drag synchronously.
See the [graph editor](graph-editor.md). `ShowEditor(api, GRAPH_EDITOR)`
lets the host present the editor in its own container.

## Container reads

The `rigExecUsdview` container offers two public reads for a host:

- `ReadPublishedControlFrame(stage, path, time)`: this session's published asset-space frame of `path` at `time` (`Gf.Matrix4d`), or None. It answers for this session's stage only.
- `ReadPublishedGeneration(stage)`: the imaging generation of this session's published snapshot, or None.

Hosts reach the container with `rigExecUsdview.ContainerFor(api)` and add
menu entries with `rigExecUsdview.AddToRigExecMenu(plugUIBuilder, submenu,
commandPlugin, rank)`.

## Undo stack protocol

`UndoStack(api)` replaces the container's stack for every RigExec editor of
the session: gizmo, marking menu, picker, graph editor, avar editor, layer
editors, and touch pose (through the gizmo's stack). The reference
implementation is `rigExecUndo.UndoStack`. The protocol:

- `Push(edit)`: `edit` has `label`, `Undo()`, and `Redo()`.
- `Undo() -> bool`, `Redo() -> bool`
- `CanUndo()`, `CanRedo()`
- `UndoText()`, `RedoText()`
- `Clear()`
- `AddListener(fn)`: `fn` takes no arguments.
- `AddFlusher(fn)` (optional): `fn` takes no arguments and runs before every
  `Undo()` and `Redo()`. The graph editor registers its queued commit here.
- `LIMIT`

With an input owner, RigExec registers no Undo/Redo shortcuts (gizmo toolbar,
gizmo window shortcuts, graph editor window). Its Undo and Redo buttons still
act on the stack.

`DocumentEdit(api)` wraps only the deferred commits of the gizmo and the
graph editor, the RigExec authoring steps that happen outside the host's own
event dispatch. Return a
context manager that selects the edit target layer for its duration.

## Node-graph history backend

The bundled node editor accepts a neutral history backend:

```python
undoHistory.set_history_backend(fn)   # fn(stage) -> backend or None
undoHistory.get_history_backend()
```

`fn` must return the same backend object while one history serves the stage;
None for a stage keeps the editor's own lists. A backend provides
`Push(description, do_func, undo_func, edit_target, view_ref)`, `Undo()`,
`Redo()`, `CanUndo()`, `CanRedo()`, `UndoText()`, `RedoText()`,
`AddReloadListener(fn)` returning a token, and `IsReplaying()`.
`set_history_backend(None)` restores the editor's own history.

## Without an extension

`OwnsInput` is False, `UndoStack` and `ShowEditor` answer None,
`DocumentEdit` is a null context, and the notification calls do nothing.
Editors install their filters and shortcuts and use `rigExecUndo.UndoStack`.

## Host rules

- Call everything on the owner (Qt main) thread.
- Evaluation, caching, playback, and imaging author nothing to the stage;
  an extension's `DocumentEdit` covers only the gizmo's commit.
- Answer `OwnsInput` the same way for the whole session.
