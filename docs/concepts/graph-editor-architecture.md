# usdNoodles graph editor architecture

usdNoodles is the node editor bundled with usdRig. It displays and edits USD
prims, attribute connections, and relationships; it is not the RigExec
engine's evaluation graph. The [RigExec Graph Editor](../specs/graph-editor.md)
is a separate tool for editing animation splines in `usdview`.
[Editor extensions](../specs/editor-extensions.md) describes the host input,
undo, and edit-target hooks in `plugin/rigExecUsdview/editorExtension.py`,
not the usdNoodles command registry or replay contract.

## Layer owners

The editor is split into the following modules in `plugin/usdNoodles/graphview/`:

| Module | Responsibility |
| --- | --- |
| `view.py` | Composes `GraphView` on `GraphWidgetBase`, wires the registry, settings and host callbacks, and delivers `nodeGraphReplaced`. |
| `stage_sync.py` | Binds the USD notice receiver, guards stage replacement with an epoch, and reconciles stage changes into the graph. |
| `model.py` | Owns graph deltas, revisions, and subscriptions through `GraphModelState`, over the existing graph collections and spatial index. |
| `selection.py` | Owns selection by node ID and `LinkKey`, remapping, and prim-tree synchronization. Model selection flags are projections of that state. |
| `intents.py` | Defines frozen requests and dispatches them by exact type through `CommandRegistry`. |
| `editing.py` | Executes authored edits, journals local USD opinions, and bridges to the existing per-stage history. |
| `interaction.py`, `interaction_events.py` | Own the Qt widget boundary, picking and hover, gesture state, and the translation of events, shortcuts, and menus into commands. |
| `frame.py` | Captures detached records and defines picking against a captured `Frame`. |
| `resources.py` | Owns the GL managers, atlas, buffers, and shaders. Allocation and disposal require the owning context to be current. |
| `render.py` | Orchestrates paint and owns the renderer's detached native graph mirror. |
| `render_nodes.py`, `render_links.py` | Draw node bodies, ports, icons, and stickers; and complete, dangling, and temporary links, respectively. |
| `render_text.py`, `render_overlays.py`, `render_hotbox.py` | Draw text and inline values/editors, status and marquee overlays, and the node creation hotbox, respectively. |

The legacy `plugin/usdNoodles/graphView.py` shim exports the same `GraphView`
class as `graphview/view.py`, not a subclass or a second implementation.
`compat.py` binds existing method names to their owners; `legacy.py` resolves
patchable shim symbols at call time. Existing callers such as `_qt_editor.py`
can keep the legacy import and signal connections.

## From a USD notice to one change set

`StageSync` checks the receiver's epoch before reconciling: a queued callback
from a replaced stage must not change the new graph. A notice arriving off the
widget's owner thread is posted to that thread. The reconciliation pass reads
USD there, following namespace moves before checking for removed or newly
in-scope prims. For example, remapping, removing, and adopting nodes submit
ordered `GraphDelta` operations through `GraphModelState.apply`.

The pass runs in one model transaction. Synchronous reconciliation within an
open transaction joins that transaction instead of publishing separately.
`GraphModelState` journals graph changes and refreshes native adjacency and the
spatial index before publishing one immutable `ChangeSet` at the outer commit.
The record identifies changed and removed nodes and links, remaps, and the
current revisions. The topology revision tracks membership and endpoint
identity, not ordinary value refreshes. A failed transaction rolls back its
journaled changes; neither failure nor a no-op publishes a change. On graph
replacement the old state publishes a final replacement record and retires its
subscriptions; the new graph gets its own state.

## Replay targets the original authoring site

An authored command records an `EditSite`: a reference to the stage instance,
the authoring layer identifier, and the edit-target mapping from scene paths to
spec paths. Undo and redo callbacks receive the invoking view from history and
resolve the recorded sites against that view's stage. A second view of the same
stage can replay the command even if its current edit target is different. A
new stage opened from the same file is not the same stage. A missing, muted,
replaced, or non-editable layer, or a mapping that no longer resolves as recorded,
causes replay to fail rather than write to the current edit target.

The journal records local before-images and the command retains the actual
forward opinions. For example, replay restores a connection's local list op,
not a list reconstructed from the composed stage. Compound replay resolves its
sites before writing, applies steps forward for redo and in reverse for undo,
and rolls back partial writes on failure. Scoped edit contexts restore the
caller's target on exit. `HistoryBridge` uses the existing per-stage history
rather than maintaining a second undo stack.

## Frame capture, picking, and drawing

`FrameSource.capture` runs on the owner thread. It advances animations (or uses
state already advanced by the paint callback), then pins a model revision,
graph identity generation, viewport, animation offsets, node and port geometry,
value and glyph inputs, links, overlays, and hit candidates. Model change
notifications mark records for refresh; unaffected records can be shared across
captures. Panning changes the viewport, not the graph. Graph replacement changes
the identity generation, and retained frames remain readable without live USD
handles or mutable native model objects.

`Frame.hit(point)` and the frame-based draw primitives use the same captured
geometry and coordinate transforms. For example, the hotbox is tested in screen
space, while picking a port uses the frame viewport to convert the pointer to
world space. `FrameSource.hit` can capture on demand if there is no frame yet.
The widget paint callback captures a frame for links, status, marquee, and the
hotbox. Other passes still use view-based adapters, and some Qt event handlers
still pick through live helpers in `interaction.py`.

`Renderer.draw(frame)` builds or reuses a native `GraphModel` from the captured
records using render-owned `NodeData` and `LinkData`. The renderer populates
nodes before links, builds adjacency, and validates identities, ports, and edge
direction before swapping in the candidate. Explicit dangling links remain
visual records rather than complete edges. If validation fails, the previous
mirror is retained and the bad frame is not drawn with it. Unchanged records
can reuse render copies on idle draws and pans; drawing a retained older frame
uses a mirror for that frame rather than assuming the latest one matches.

The mirror is a rendering cache, not a second editable graph. The frame-based
node and text passes share render transforms for placement; neither a transient
drag offset nor a glyph refresh writes back to the live model. The frame can
outlive a draw or GL context, while the resources and renderer caches belong to
the rendering side of the widget.
