#!/usr/bin/env python
"""
Headless test for the AI Assistant in plugin/rigExecUsdview/usdRigAssistant:
the tools on an in-memory stage (looking, posing, changing the stage,
running Python, and one undo step per reply), the action blocks of models
without tool calling, and the MCP server Claude Code connects to. A fake
usdviewApi stands in for usdview; no provider is called.

Usage: test_assistant.py
"""
import json
import os
import sys
import tempfile
import urllib.error
import urllib.request

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["USDRIG_USER_DIR"] = tempfile.mkdtemp(prefix="usdrig_assistant_")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtWidgets  # noqa: E402

import rigExecUndo  # noqa: E402
import layerOpinionsModel  # noqa: E402
from usdRigAssistant import chats, mcp_server, session  # noqa: E402
from usdRigAssistant import stage_actions, tools  # noqa: E402

_failures = []


def _Check(condition, message):
    if not condition:
        _failures.append(message)


class _Selection(object):
    def __init__(self, api):
        self.api = api
        self.batchPrimChanges = self

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def clearPrims(self):
        self.api.selectedPrims = []

    def addPrim(self, prim):
        self.api.selectedPrims.append(prim)


class _DataModel(object):
    def __init__(self, api):
        self.selection = _Selection(api)


class _Api(object):
    def __init__(self, stage):
        self.stage = stage
        self.frame = Usd.TimeCode(1)
        self.selectedPrims = []
        self.dataModel = _DataModel(self)
        self.qMainWindow = None

    def UpdateViewport(self):
        pass


class _Container(object):
    def __init__(self):
        self.stack = rigExecUndo.UndoStack()

    def _UndoStack(self):
        return self.stack

    def IsRigActive(self):
        return False

    def ImagingHandle(self):
        return None


class _CountingStack(rigExecUndo.UndoStack):
    """A real stack that keeps what was pushed, to count the steps."""

    def __init__(self):
        rigExecUndo.UndoStack.__init__(self)
        self.pushed = []

    def Push(self, edit):
        self.pushed.append(edit)
        rigExecUndo.UndoStack.Push(self, edit)


class _CountingContainer(_Container):
    def __init__(self):
        _Container.__init__(self)
        self.stack = _CountingStack()


class _StacklessContainer(_Container):
    """A host with no undo stack to record on."""

    def _UndoStack(self):
        return None


class _SwappingContainer(_Container):
    """A host that swaps its undo stack partway through a reply."""

    def __init__(self):
        _Container.__init__(self)
        self.replacement = rigExecUndo.UndoStack()
        self._swapped = False

    def swap(self):
        self._swapped = True

    def _UndoStack(self):
        return self.replacement if self._swapped else self.stack


_LAYER = """#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 24
)
def Xform "Char"
{
    def Xform "Ctl"
    {
        custom double avars:rx = 0
        custom double avars:ty = 0
    }
    def Scope "Geo"
    {
    }
}
"""


def _Stage():
    layer = Sdf.Layer.CreateAnonymous("assistant.usda")
    layer.ImportFromString(_LAYER)
    return Usd.Stage.Open(layer)


def _Toolbox(stage, container, **kwargs):
    api = _Api(stage)
    return tools.Toolbox(session.Session(api, container), **kwargs), api


def TestLooking():
    stage = _Stage()
    box, api = _Toolbox(stage, _Container())
    names = {t.name for t in box.available()}
    _Check("get_prim" in names and "change_stage" not in names and
           "run_python" not in names,
           "looking tools only without Edit and Run: %s" % sorted(names))
    text, failed = box.call("get_stage_overview", {})
    info = json.loads(text)
    _Check(not failed and info["end_frame"] == 24,
           "overview reads the time range: %s" % text)
    text, failed = box.call("list_prims", {"path": "/Char", "depth": 2})
    _Check(not failed and "/Char/Ctl" in text and "/Char/Geo" in text,
           "list_prims outlines the children: %s" % text)
    text, failed = box.call("get_prim", {"path": "Char/Ctl"})
    _Check(not failed and "avars:rx" in json.loads(text)["attributes"],
           "get_prim takes a path without its leading slash: %s" % text)
    text, failed = box.call("find_prims", {"attribute": "avars:ty"})
    _Check(not failed and text.startswith("/Char/Ctl"),
           "find_prims by attribute: %s" % text)
    text, failed = box.call("get_prim", {"path": "/Nope"})
    _Check(failed and "no prim" in text, "a missing prim fails: %s" % text)
    text, failed = box.call("change_stage", {"changes": []})
    _Check(failed and "Edit stage" in text,
           "editing is refused while Edit stage is off: %s" % text)
    text, failed = box.call("select_prims", {"paths": ["/Char/Ctl"]})
    _Check(not failed and [str(p.GetPath()) for p in api.selectedPrims] ==
           ["/Char/Ctl"], "select_prims selects in usdview: %s" % text)


def TestChangesAreOneUndoStep():
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/Extra",
         "type": "Xform"},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:tz",
         "type": "double", "value": 2.5, "time": 5},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 30},
        {"action": "nonsense"},
    ]})
    _Check(not failed and "Could not: There is no action nonsense" in text,
           "a bad action is reported, the rest are made: %s" % text)
    tz = stage.GetAttributeAtPath("/Char/Ctl.avars:tz")
    _Check(stage.GetPrimAtPath("/Char/Rig/Extra").IsValid() and
           tz.Get(5) == 2.5 and
           stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 30,
           "change_stage defines, keys and sets")
    text, failed = box.call("change_stage", {"changes": [
        {"action": "rename_prim", "path": "/Char/Geo", "name": "Mesh"}]})
    renamed = stage.GetPrimAtPath("/Char/Mesh").IsValid()
    _Check(failed or renamed, "rename_prim renames: %s" % text)
    box.finish()
    _Check(container.stack.CanUndo() and
           container.stack.UndoText() == "AI Assistant changes",
           "the reply's changes are one undo step")
    container.stack.Undo()
    _Check(not stage.GetPrimAtPath("/Char/Rig").IsValid(),
           "undo removes the new prims and their new parents")
    _Check(not stage.GetPrimAtPath("/Char/Ctl").HasAttribute("avars:tz"),
           "undo removes the new attribute")
    _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
           "undo puts the old value back")
    _Check(stage.GetPrimAtPath("/Char/Geo").IsValid() and
           not stage.GetPrimAtPath("/Char/Mesh").IsValid(),
           "undo takes back the rename")
    container.stack.Redo()
    _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 30 and
           stage.GetPrimAtPath("/Char/Rig/Extra").IsValid(),
           "redo makes the changes again")


def TestPosing():
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("list_avars", {"path": "/Char/Ctl"})
    found = {a["name"] for a in json.loads(text).get("avars", [])} \
        if not failed else set()
    _Check({"avars:rx", "avars:ty"} <= found,
           "list_avars lists the channels: %s" % text)
    text, failed = box.call("set_avars", {"values": [
        {"path": "/Char/Ctl", "name": "avars:rx", "value": 45},
        {"path": "/Char/Ctl", "name": "ty", "value": 1.5},
        {"path": "/Char/Ctl", "name": "avars:nope", "value": 1}],
        "time": 10})
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    _Check(not failed and abs(rx.Get(10) - 45) < 1e-9,
           "set_avars keys the value at the frame: %s" % text)
    _Check(abs(stage.GetAttributeAtPath("/Char/Ctl.avars:ty").Get(10) -
               1.5) < 1e-9, "a short avar name is found")
    _Check("Could not" in text and "avars:nope" in text,
           "an unknown avar is reported: %s" % text)
    box.finish()
    container.stack.Undo()
    _Check(abs(rx.Get(10)) < 1e-9, "undo takes the key away: %s" %
           rx.Get(10))


def TestRunPython():
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_run=True)
    text, failed = box.call("run_python", {"code": (
        "prim = stage.DefinePrim('/Char/Made', 'Xform')\n"
        "print('made', prim.GetPath())\n"
        "1 + 2")})
    result = json.loads(text)
    _Check(not failed and "made /Char/Made" in result.get("printed", "") and
           result.get("value") == "3",
           "run_python prints and gives back the last value: %s" % text)
    text, failed = box.call("run_python", {"code": "raise ValueError('x')"})
    _Check("ValueError" in json.loads(text).get("error", ""),
           "an error in the code is given back: %s" % text)
    box.finish()
    container.stack.Undo()
    _Check(not stage.GetPrimAtPath("/Char/Made").IsValid(),
           "undo takes back what the Python changed on the edit target")


def _OneStepPerReply():
    """Several tool calls, one host undo step, and finish() only once."""
    stage = _Stage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    box.call("set_avars", {"values": [
        {"path": "/Char/Ctl", "name": "avars:rx", "value": 45}]})
    box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/Extra",
         "type": "Xform"}]})
    box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:tz",
         "type": "double", "value": 1.0}]})
    box.finish()
    box.finish()
    pushed = container.stack.pushed
    _Check(len(pushed) == 1,
           "the calls of a reply are one step on the host stack: %s" % pushed)
    group = pushed[0] if pushed else None
    _Check(isinstance(group, session.GroupEdit) and len(group.edits) == 3,
           "the three edits are one group: %s" % (
               getattr(group, "edits", None),))
    _Check(getattr(group, "label", "") == stage_actions.UNDO_LABEL,
           "the step carries the assistant's label: %s" %
           getattr(group, "label", ""))
    box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rz",
         "type": "double", "value": 1.0}]})
    box.finish()
    _Check(len(container.stack.pushed) == 1,
           "finish is terminal: a later call does not open a new step")
    container.stack.Undo()
    ctl = stage.GetPrimAtPath("/Char/Ctl")
    _Check(abs(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get()) < 1e-9
           and not stage.GetPrimAtPath("/Char/Rig").IsValid()
           and not ctl.HasAttribute("avars:tz"),
           "one undo takes the whole reply back")
    _Check(ctl.HasAttribute("avars:rz"),
           "the change made after finish was never part of the step")


def _OverlappingWrites():
    stage = _Stage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 30},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 50}]})
    text2, failed2 = box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/New", "type": "Xform"},
        {"action": "set_attr", "path": "/Char/Rig/New", "name": "x",
         "type": "double", "value": 3.0}]})
    _Check(not failed and not failed2,
           "writes over the same spec are made: %s %s" % (text, text2))
    _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 50,
           "the last write to a spec wins")
    box.finish()
    _Check(len(container.stack.pushed) == 1,
           "overlapping writes are still one step: %s" %
           container.stack.pushed)
    container.stack.Undo()
    _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
           "one undo puts the original value back through both writes")
    _Check(not stage.GetPrimAtPath("/Char/Rig").IsValid(),
           "undo takes the new prim, its attribute and its new parents")
    container.stack.Redo()
    _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 50 and
           stage.GetPrimAtPath("/Char/Rig/New").IsValid() and
           stage.GetAttributeAtPath("/Char/Rig/New.x").Get() == 3.0,
           "redo makes the writes again in their order")


def _CrossLayerReply():
    root = _Layer("reply_root.usda", _CAPTURE_USDA)
    over = _Layer("reply_over.usda")
    root.subLayerPaths.append(over.identifier)
    stage = Usd.Stage.Open(root)
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 2.0},
        {"action": "set_edit_target", "layer": over.identifier},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "y",
         "type": "double", "value": 7.0}]})
    _Check(not failed, "one reply writes two layers: %s" % text)
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 2.0 and
           over.GetAttributeAtPath("/Char/Ctl.y").default == 7.0,
           "each write went to the layer that was the edit target")
    box.finish()
    pushed = container.stack.pushed
    _Check(len(pushed) == 1, "the cross-layer reply is one step: %s" % pushed)
    edit = pushed[0] if pushed else None
    edits = ([edit] if hasattr(edit, "entries")
             else list(getattr(edit, "edits", [])))
    layers = {entry.layer.identifier
              for e in edits for entry in e.entries}
    _Check(layers == {root.identifier, over.identifier},
           "one step carries both layers: %s" % (layers,))
    container.stack.Undo()
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 1.0 and
           over.GetAttributeAtPath("/Char/Ctl.y") is None,
           "one undo puts both layers back")
    _Check(any(e.startswith("Edits now go to") for e in box.effects),
           "the edit target switch is an effect: %s" % box.effects)


def _EffectsAreNotUndoable():
    stage = _Stage()
    container = _CountingContainer()
    box, api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("select_prims", {"paths": ["/Char/Ctl"]})
    _Check(not failed and any(e.startswith("Selected /Char/Ctl")
                              for e in box.effects),
           "a selection is an effect: %s" % box.effects)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "select", "paths": ["/Char/Geo"]}]})
    _Check(not failed and "Also: Selected /Char/Geo" in text,
           "a select action reports what it did: %s" % text)
    box.finish()
    _Check(not container.stack.pushed and not container.stack.CanUndo(),
           "a select-only reply leaves the undo history untouched")
    _Check([str(p.GetPath()) for p in api.selectedPrims] == ["/Char/Geo"],
           "the selection stands: %s" % box.effects)


def _SaveOnlyReply():
    folder = tempfile.mkdtemp(prefix="usdrig_reply_")
    path = os.path.join(folder, "reply.usda")
    with open(path, "w") as handle:
        handle.write(_LAYER)
    stage = Usd.Stage.Open(path)
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Set(4)
    text, failed = box.call("save_layers", {"all": True})
    _Check(not failed and "Saved" in text,
           "save_layers saves the dirty layer: %s" % text)
    _Check(any(e.startswith("Saved ") for e in box.effects),
           "a save is an effect, not an undoable change: %s" % box.effects)
    box.finish()
    _Check(not container.stack.pushed and not container.stack.CanUndo(),
           "a save-only reply leaves the undo history untouched")
    with open(path) as handle:
        _Check("avars:rx = 4" in handle.read(),
               "the save wrote through the undo step")


def _MissingUndoStack():
    stage = _Stage()
    container = _StacklessContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/Extra",
         "type": "Xform"}]})
    _Check(not failed and stage.GetPrimAtPath("/Char/Rig/Extra").IsValid(),
           "the change is still made without a stack: %s" % text)
    _Check("undo stack" in text and "cannot be taken back" in text,
           "the answer says there is no undo: %s" % text)
    _Check(any("undo stack" in f for f in box.failed),
           "the toolbox says it too: %s" % box.failed)
    text, failed = box.call("set_avars", {"values": [
        {"path": "/Char/Ctl", "name": "avars:rx", "value": 45}]})
    _Check("undo stack" in text, "posing says the same: %s" % text)
    box.finish()
    _Check(stage.GetPrimAtPath("/Char/Rig/Extra").IsDefined() and
           stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get(1) == 45,
           "finish with no stack does not raise and leaves the work in place")


def _ReplacedUndoStack():
    stage = _Stage()
    container = _SwappingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 30}]})
    pinned = container.stack
    container.swap()
    box.finish()
    _Check(pinned.CanUndo() and
           pinned.UndoText() == stage_actions.UNDO_LABEL,
           "the step went to the stack the reply was recorded against")
    _Check(not container.replacement.CanUndo(),
           "the new stack did not get the reply's changes")
    _Check(any("undo stack" in f for f in box.failed),
           "the toolbox says the stack went away: %s" % box.failed)


class _ThrowingStack(rigExecUndo.UndoStack):
    def Push(self, edit):
        raise RuntimeError("the host stack is closed")


def _PushThatThrows():
    stage = _Stage()
    container = _Container()
    container.stack = _ThrowingStack()
    box, _api = _Toolbox(stage, container, can_edit=True)
    box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/Extra",
         "type": "Xform"}]})
    box.finish()
    _Check(any("closed" in f for f in box.failed),
           "a throwing Push is reported on the toolbox: %s" % box.failed)
    text, failed = box.call("get_prim", {"path": "/Char/Ctl"})
    _Check(not failed and "avars:rx" in json.loads(text)["attributes"],
           "the reply keeps working after a failed Push: %s" % text)


def _ChatKeepsEffects():
    """The reply report survives reopening the chat: a chat keeps a fixed
    set of message keys, so "effects" has to be one of them."""
    store = chats.ChatStore(folder=None)
    chat_id = chats.ChatStore.new_id()
    store.save({"id": chat_id, "messages": [
        {"role": "user", "content": "save it"},
        {"role": "assistant", "content": "Saved.",
         "done": ["Created /World/Box"],
         "effects": ["Saved /tmp/reply.usda"]}]})
    kept = store.load(chat_id)["messages"][1]
    _Check(kept.get("effects") == ["Saved /tmp/reply.usda"],
           "the effect did not survive the chat: %s" % kept)
    _Check(kept.get("done") == ["Created /World/Box"],
           "the change did not survive the chat: %s" % kept)


def TestReplyTransaction():
    """One reply is one step on the host's undo stack, and nothing else."""
    _OneStepPerReply()
    _OverlappingWrites()
    _CrossLayerReply()
    _EffectsAreNotUndoable()
    _SaveOnlyReply()
    _MissingUndoStack()
    _ReplacedUndoStack()
    _PushThatThrows()
    _ChatKeepsEffects()


_CAPTURE_USDA = """#usda 1.0
(
    timeCodesPerSecond = 30
)
def Xform "Char"
{
    def Xform "Ctl"
    {
        rel bindings = </Char/Geo>
        custom double x = 1
    }
    def Scope "Geo"
    {
    }
    def Scope "Tail"
    {
    }
}
"""


_AIMER_USDA = """#usda 1.0
def Xform "Char"
{
    def Xform "Rig"
    {
        rel aim = </Char/Geo>
    }
}
"""


def _Layer(name, usda=""):
    layer = Sdf.Layer.CreateAnonymous(name)
    if usda:
        layer.ImportFromString(usda)
    return layer


def _RecordingApplier(stage, stack):
    """An Applier that hands every edit it records to a real undo stack."""
    edits = []

    def record(edit):
        edits.append(edit)
        stack.Push(edit)

    return stage_actions.Applier(stage, record=record), edits


def _SublayerCapture():
    root = _Layer("capture_root.usda", _CAPTURE_USDA)
    near = _Layer("capture_near.usda")
    root.subLayerPaths.append(near.identifier)
    root.subLayerOffsets[0] = Sdf.LayerOffset(0, 5)
    paths = [str(p) for p in root.subLayerPaths]
    offsets = [(o.scale, o.offset) for o in root.subLayerOffsets]
    _Check(offsets != [(1.0, 0.0)],
           "the neighbour is retimed, not identity: %s" % (offsets,))
    new = _Layer("capture_new.usda")
    stack = rigExecUndo.UndoStack()
    applier, edits = _RecordingApplier(Usd.Stage.Open(root), stack)
    done, failed = applier.apply([{"action": "add_sublayer",
                                   "asset": new.identifier, "position": 0}])
    _Check(not failed and len(edits) == 1 and len(edits[0].entries) == 1,
           "a layer's sublayers are one entry: %s %s" % (done, failed))
    entry = edits[0].entries[0] if edits and edits[0].entries else None
    _Check(entry is not None
           and entry.layer.identifier == root.identifier
           and entry.specPath == Sdf.Path.absoluteRootPath
           and isinstance(entry.before,
                          layerOpinionsModel.SublayerSnapshot),
           "the entry names the layer whose sublayers changed")
    _Check([str(p) for p in root.subLayerPaths]
           == [new.identifier] + paths, "the sublayer is inserted first")
    stack.Undo()
    _Check([str(p) for p in root.subLayerPaths] == paths
           and [(o.scale, o.offset) for o in root.subLayerOffsets] == offsets,
           "one undo restores the sublayers and their offsets")
    _Check(root.pseudoRoot.GetInfo("timeCodesPerSecond") == 30,
           "an unrelated layer field survives the undo")
    stack.Redo()
    _Check([str(p) for p in root.subLayerPaths]
           == [new.identifier] + paths
           and [(o.scale, o.offset) for o in root.subLayerOffsets]
           == [(1.0, 0.0), offsets[0]],
           "one redo inserts it again without resetting the offset")


def _RootFieldCapture():
    root = _Layer("capture_meta_root.usda", _CAPTURE_USDA)
    target = _Layer("capture_meta_target.usda")
    root.subLayerPaths.append(target.identifier)
    stage = Usd.Stage.Open(root)
    stage.SetEditTarget(Usd.EditTarget(target))
    stack = rigExecUndo.UndoStack()
    applier, edits = _RecordingApplier(stage, stack)
    done, failed = applier.apply([{"action": "set_metadata", "path": "/",
                                   "key": "metersPerUnit", "value": 0.5}])
    _Check(not failed and len(edits) == 1,
           "a layer field is recorded: %s %s" % (done, failed))
    entry = edits[0].entries[0] if edits and edits[0].entries else None
    _Check(edits and edits[0].label == "AI Assistant changes",
           "the edit carries the reply's label: %s" % (edits,))
    _Check(entry is not None
           and isinstance(entry.before, layerOpinionsModel.InfoSnapshot)
           and entry.before.had is False
           and entry.layer.identifier == target.identifier
           and entry.specPath == Sdf.Path.absoluteRootPath,
           "the field is an info snapshot on the edit target, not a prim")
    _Check(target.pseudoRoot.GetInfo("metersPerUnit") == 0.5
           and not root.pseudoRoot.HasInfo("metersPerUnit"),
           "the field lands on the edit target layer")
    stack.Undo()
    _Check(not target.pseudoRoot.HasInfo("metersPerUnit"),
           "one undo leaves a field that was absent before absent")
    stack.Redo()
    _Check(target.pseudoRoot.GetInfo("metersPerUnit") == 0.5,
           "one redo puts the field back")


def _TargetSwitchCapture():
    root = _Layer("capture_switch_root.usda", _CAPTURE_USDA)
    over = _Layer("capture_switch_over.usda")
    root.subLayerPaths.append(over.identifier)
    stage = Usd.Stage.Open(root)
    stack = rigExecUndo.UndoStack()
    seen, edits = [], []

    def record(edit):
        edits.append(edit)
        stack.Push(edit)

    applier = stage_actions.Applier(stage, record=record,
                                    select=lambda paths: None,
                                    effect=seen.append)
    done, failed = applier.apply([
        {"action": "set_edit_target", "layer": over.identifier},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 2.0},
    ])
    _Check(not failed and len(done) == 1 and len(edits) == 1,
           "the switch reports an effect, the edit one entry: %s %s %s"
           % (done, failed, edits))
    _Check(len(seen) == 1 and "Edits now go to" in seen[0],
           "the switch is reported through the effect callback: %s" % seen)
    entry = edits[0].entries[0] if edits and edits[0].entries else None
    _Check(entry is not None
           and entry.layer.identifier == over.identifier,
           "the entry is bound to the layer the edit went to")
    _Check(over.GetAttributeAtPath("/Char/Ctl.x").default == 2.0,
           "the over carries the new value")
    stack.Undo()
    _Check(over.GetPrimAtPath("/Char") is None
           and over.GetAttributeAtPath("/Char/Ctl.x") is None,
           "one undo leaves no overs behind in the second layer")
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 1.0,
           "the first layer still reads its own value")
    _Check(stage.GetEditTarget().GetLayer().identifier == over.identifier,
           "the edit target stays where the action put it")
    stack.Redo()
    _Check(over.GetAttributeAtPath("/Char/Ctl.x").default == 2.0,
           "one redo makes the edit again")
    done, failed = applier.apply([{"action": "set_attr", "path": "/Char/Ctl",
                                   "name": "x", "value": 2.0}])
    _Check(done and not failed and len(edits) == 1,
           "a change that changes nothing records nothing: %s %s"
           % (done, failed))


class _BlindCaptureApplier(stage_actions.Applier):
    """An Applier whose capture refuses, to check the write is refused too."""

    def _capture_spec(self, scene_path):
        raise stage_actions.ActionError("the snapshot failed")


class _RaisingAfterWriteApplier(stage_actions.Applier):
    """An Applier whose action raises after it wrote."""

    def do_define_prim(self, action):
        stage_actions.Applier.do_define_prim(self, action)
        raise stage_actions.ActionError("stopped after writing")


def _RecordFailure():
    root = _Layer("capture_fail_root.usda", _CAPTURE_USDA)
    stage = Usd.Stage.Open(root)

    def record(edit):
        raise RuntimeError("the undo stack is gone")

    applier = stage_actions.Applier(stage, record=record)
    done, failed = applier.apply([{"action": "set_attr", "path": "/Char/Ctl",
                                   "name": "x", "value": 3.0}])
    _Check(done and any("set_attr" in f and "undo" in f for f in failed),
           "a record that raises is reported, not hidden: %s %s"
           % (done, failed))
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 3.0,
           "a failed record does not roll the change back")
    blind = _BlindCaptureApplier(stage, record=lambda edit: None)
    done, failed = blind.apply([{"action": "define_prim",
                                 "path": "/Char/Rig/New", "type": "Xform"}])
    _Check(not done and any("define_prim" in f for f in failed),
           "a capture that fails is reported: %s" % (failed,))
    _Check(root.GetPrimAtPath("/Char/Rig") is None,
           "a capture that fails before the write leaves the stage alone")


def _WriteBeforeRaise():
    """A write made before an action raised still needs its undo entry."""
    root = _Layer("capture_later_root.usda", _CAPTURE_USDA)
    stack = rigExecUndo.UndoStack()
    edits = []

    def record(edit):
        edits.append(edit)
        stack.Push(edit)

    applier = _RaisingAfterWriteApplier(Usd.Stage.Open(root), record=record)
    done, failed = applier.apply([{"action": "define_prim",
                                   "path": "/Char/Rig/Gone",
                                   "type": "Xform"}])
    _Check(not done and any("define_prim" in f for f in failed)
           and len(edits) == 1,
           "an action that raised still hands over what it wrote: %s %s"
           % (done, failed))
    _Check(stack.Undo() and root.GetPrimAtPath("/Char/Rig") is None,
           "the write made before the raise is undone too")


def _MoveCapture():
    root = _Layer("capture_move_root.usda")
    owner = _Layer("capture_move_owner.usda", _CAPTURE_USDA)
    elsewhere = _Layer("capture_move_elsewhere.usda")
    root.subLayerPaths.append(owner.identifier)
    root.subLayerPaths.append(elsewhere.identifier)
    stage = Usd.Stage.Open(root)
    stage.SetEditTarget(Usd.EditTarget(elsewhere))

    def siblings():
        return list(owner.GetPrimAtPath("/Char").nameChildren.keys())

    def targets():
        spec = owner.GetRelationshipAtPath("/Char/Ctl.bindings")
        return [str(t) for t in spec.GetInfo("targetPaths").explicitItems]

    _Check(siblings() == ["Ctl", "Geo", "Tail"]
           and targets() == ["/Char/Geo"],
           "the rig starts as written: %s %s" % (siblings(), targets()))
    stack = rigExecUndo.UndoStack()
    applier, edits = _RecordingApplier(stage, stack)
    done, failed = applier.apply([{"action": "rename_prim",
                                   "path": "/Char/Geo", "name": "Mesh"}])
    _Check(not failed and done and len(edits) == 1,
           "the rename is applied and recorded: %s %s" % (done, failed))
    _Check(not elsewhere.rootPrims,
           "a move writes the layer that owns the opinions, not the target")
    _Check(siblings() == ["Ctl", "Mesh", "Tail"]
           and targets() == ["/Char/Mesh"],
           "the spec and the relationship that named it moved: %s %s"
           % (siblings(), targets()))
    entries = edits[0].entries if edits else []
    _Check(entries and all(e.before.layer.identifier == owner.identifier
                           and e.after.layer.identifier == owner.identifier
                           for e in entries),
           "every entry is bound to the owning layer")
    stack.Undo()
    _Check(owner.GetPrimAtPath("/Char/Geo") is not None
           and owner.GetPrimAtPath("/Char/Mesh") is None,
           "one undo brings back the old name")
    _Check(siblings() == ["Ctl", "Geo", "Tail"],
           "undo puts the spec back among its siblings: %s" % (siblings(),))
    _Check(targets() == ["/Char/Geo"],
           "undo points the relationship back at the old name")
    stack.Redo()
    _Check(owner.GetPrimAtPath("/Char/Mesh") is not None
           and siblings() == ["Ctl", "Mesh", "Tail"]
           and targets() == ["/Char/Mesh"],
           "one redo makes the rename and the repair again")


def _NewPropertyCapture():
    """A new property is authored by the action itself, so its snapshot has
    to come first: a spec captured after the create cannot be taken away."""
    root = _Layer("capture_prop_root.usda", _CAPTURE_USDA)
    stack = rigExecUndo.UndoStack()
    applier, edits = _RecordingApplier(Usd.Stage.Open(root), stack)
    done, failed = applier.apply([
        {"action": "set_relationship", "path": "/Char/Ctl", "name": "links",
         "targets": ["/Char/Geo"]},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "z",
         "type": "double", "value": 3.0},
    ])
    _Check(not failed and len(edits) == 2,
           "each new property is one edit: %s %s" % (done, failed))
    _Check(root.GetPropertyAtPath("/Char/Ctl.links") is not None
           and root.GetPropertyAtPath("/Char/Ctl.z") is not None,
           "both properties are authored")
    stack.Undo()
    _Check(root.GetPropertyAtPath("/Char/Ctl.z") is None,
           "one undo takes the new attribute away")
    stack.Undo()
    _Check(root.GetPropertyAtPath("/Char/Ctl.links") is None,
           "one undo takes the new relationship away")
    _Check([str(t) for t in root.GetRelationshipAtPath("/Char/Ctl.bindings")
            .GetInfo("targetPaths").explicitItems] == ["/Char/Geo"],
           "the relationship that was already there is untouched")


def _MoveRepairsAnotherLayer():
    """A rename retargets the relationships that named the prim, in
    whatever layer authors them, and one undo puts those back too."""
    root = _Layer("capture_repair_root.usda")
    owner = _Layer("capture_repair_owner.usda", _CAPTURE_USDA)
    reader = _Layer("capture_repair_reader.usda", _AIMER_USDA)
    elsewhere = _Layer("capture_repair_target.usda")
    for layer in (owner, reader, elsewhere):
        root.subLayerPaths.append(layer.identifier)
    stage = Usd.Stage.Open(root)
    stage.SetEditTarget(Usd.EditTarget(elsewhere))

    def aimed():
        return [str(t) for t in reader.GetRelationshipAtPath("/Char/Rig.aim")
                .GetInfo("targetPaths").explicitItems]

    stack = rigExecUndo.UndoStack()
    applier, edits = _RecordingApplier(stage, stack)
    done, failed = applier.apply([{"action": "rename_prim",
                                   "path": "/Char/Geo", "name": "Mesh"}])
    _Check(not failed and len(edits) == 1,
           "the rename stays one edit: %s %s" % (done, failed))
    _Check(aimed() == ["/Char/Mesh"],
           "the move retargets the relationship where it is authored: %s"
           % (aimed(),))
    entries = edits[0].entries if edits else []
    _Check(any(e.layer.identifier == reader.identifier
               and e.specPath == Sdf.Path("/Char/Rig.aim")
               for e in entries),
           "the layer the editor did not name is on the undo stack too")
    stack.Undo()
    _Check(aimed() == ["/Char/Geo"],
           "one undo puts the other layer's target back")
    stack.Redo()
    _Check(aimed() == ["/Char/Mesh"], "one redo retargets it again")


def TestActionCapture():
    """Every action records the layer it actually wrote, so one host undo
    step puts that layer back exactly."""
    _SublayerCapture()
    _RootFieldCapture()
    _TargetSwitchCapture()
    _NewPropertyCapture()
    _MoveCapture()
    _MoveRepairsAnotherLayer()
    _RecordFailure()
    _WriteBeforeRaise()


def TestActionBlocks():
    reply = ("Done.\n\n```usdrig-actions\n"
             '[{"action": "set_active", "path": "/Char/Geo", '
             '"value": false}]\n```\n')
    actions, problems = stage_actions.find_actions(reply)
    _Check(len(actions) == 1 and not problems,
           "an action block is read: %s %s" % (actions, problems))
    _Check(stage_actions.without_actions(reply) == "Done.",
           "the block is left out of what the chat shows")
    _, problems = stage_actions.find_actions(
        "```usdrig-actions\nnot json\n```")
    _Check(problems, "a broken block is reported")
    stage = _Stage()
    described = stage_actions.describe_stage(stage)
    _Check("/Char/Ctl" in described and "(edit target)" in described,
           "describe_stage outlines the stage: %s" % described)


def _TwoLayerStage(usda=_LAYER):
    """A root layer with an anonymous over above it, so a reply can write
    on two layers and each one can be read back on its own."""
    root = _Layer('py_root.usda', usda)
    over = _Layer('py_over.usda')
    root.subLayerPaths.append(over.identifier)
    return root, over, Usd.Stage.Open(root)


def _LayerOf(stage, name):
    """The code the assistant runs has only the stage to reach its layers
    with, so the tests pick them out the same way. The name is matched at
    the end: the session layer's identifier carries the root layer's."""
    return ("[l for l in stage.GetUsedLayers() if "
            "l.identifier.endswith(%r)][0]" % name)


def _PythonWritesTwoLayers():
    """Python that writes on two layers is recorded on both, and one undo
    step puts both back."""
    root, over, stage = _TwoLayerStage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_run=True)
    text, failed = box.call("run_python", {"code": (
        "stage.SetEditTarget(Usd.EditTarget(%s))\n"
        "stage.GetAttributeAtPath('/Char/Ctl.avars:rx').Set(9)\n"
        "stage.SetEditTarget(Usd.EditTarget(%s))\n"
        "stage.DefinePrim('/Char/Wrote', 'Xform')\n"
        % (_LayerOf(stage, "py_root.usda"), _LayerOf(stage, "py_over.usda")))})
    lines = [d for d in box.done if d.endswith("with Python")]
    _Check(not failed and len(lines) == 2,
           "both changed layers are reported: %s %s" % (text, box.done))
    _Check(root.GetAttributeAtPath("/Char/Ctl.avars:rx").default == 9 and
           over.GetPrimAtPath("/Char/Wrote") is not None,
           "the two layers were written: %s" % text)
    box.finish()
    pushed = container.stack.pushed
    _Check(len(pushed) == 1, "the reply is one step: %s" % pushed)
    group = pushed[0] if pushed else None
    _Check(isinstance(group, session.GroupEdit) and len(group.edits) == 2 and
           all(isinstance(e, session.LayerSnapshotEdit) for e in group.edits)
           and all(e.label == "AI Assistant Python" for e in group.edits),
           "the whole-layer edits are one group: %s" % (
               getattr(group, "edits", None),))
    _Check(group is not None and
           {e.layer.identifier for e in group.edits} ==
           {root.identifier, over.identifier},
           "the group carries both layers: %s" % (
               getattr(group, "edits", None),))
    container.stack.Undo()
    _Check(root.GetAttributeAtPath("/Char/Ctl.avars:rx").default == 0 and
           over.GetPrimAtPath("/Char/Wrote") is None and
           not stage.GetPrimAtPath("/Char/Wrote").IsValid(),
           "one undo puts both layers back")
    container.stack.Redo()
    _Check(root.GetAttributeAtPath("/Char/Ctl.avars:rx").default == 9 and
           over.GetPrimAtPath("/Char/Wrote") is not None,
           "one redo writes both layers again")


def _PythonRaisesAfterAnEdit():
    """Code that raises after authoring still gets its change recorded."""
    root, _over, stage = _TwoLayerStage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_run=True)
    text, failed = box.call("run_python", {"code": (
        "stage.GetAttributeAtPath('/Char/Ctl.avars:rx').Set(6)\n"
        "raise ValueError('halfway')\n")})
    result = json.loads(text)
    _Check("halfway" in result.get("error", ""),
           "the error is still given back: %s" % text)
    _Check(root.GetAttributeAtPath("/Char/Ctl.avars:rx").default == 6,
           "the change the code made before raising stands")
    _Check(any(d.endswith("with Python") for d in box.done),
           "the change is recorded even though the code raised: %s"
           % box.done)
    box.finish()
    _Check(len(container.stack.pushed) == 1,
           "the change of a raising run is one step: %s" %
           container.stack.pushed)
    container.stack.Undo()
    _Check(root.GetAttributeAtPath("/Char/Ctl.avars:rx").default == 0 and
           stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
           "undo takes back the change made before the error")


def _PythonOpensALayer():
    """A layer the code brings in has no baseline, so it is reported as
    outside what undo covers rather than guessed at."""
    root, over, stage = _TwoLayerStage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_run=True)
    text, failed = box.call("run_python", {"code": (
        "fresh = Sdf.Layer.CreateAnonymous('py_fresh.usda')\n"
        "%s.subLayerPaths.append(fresh.identifier)\n"
        "stage.SetEditTarget(Usd.EditTarget(fresh))\n"
        "stage.DefinePrim('/Char/Fresh', 'Xform')\n"
        % _LayerOf(stage, "py_root.usda"))})
    _Check(stage.GetPrimAtPath("/Char/Fresh").IsValid(),
           "the code's layer is in the stage: %s" % text)
    _Check(any("not covered by undo" in f and "py_fresh" in f
               for f in box.failed),
           "the layer the code opened is reported as uncovered: %s"
           % box.failed)
    _Check("not covered by undo" in json.loads(text).get("undo", ""),
           "the run says so too: %s" % text)
    layers = [d for d in box.done if d.endswith("with Python")]
    _Check(any("py_root.usda" in d for d in layers),
           "the layer it started with is still covered: %s" % box.done)
    box.finish()
    _Check(len(container.stack.pushed) == 1,
           "the reply is one step whatever it touched: %s" %
           container.stack.pushed)
    container.stack.Undo()
    _Check(list(root.subLayerPaths) == [over.identifier] and
           not stage.GetPrimAtPath("/Char/Fresh").IsValid(),
           "undo puts the starting layer back, and with it the stage")


def _MixedPoseChangeAndPython():
    """Posing, change_stage and Python in one reply: one step, one undo
    puts every layer it touched back."""
    root, over, stage = _TwoLayerStage()
    container = _CountingContainer()
    box, _api = _Toolbox(stage, container, can_edit=True, can_run=True)
    box.call("set_avars", {"values": [
        {"path": "/Char/Ctl", "name": "avars:rx", "value": 45}]})
    box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": "/Char/Rig/Extra", "type":
         "Xform"}]})
    text, failed = box.call("run_python", {"code": (
        "stage.SetEditTarget(Usd.EditTarget(%s))\n"
        "stage.DefinePrim('/Char/Wrote', 'Xform')\n"
        % _LayerOf(stage, "py_over.usda"))})
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    _Check(not failed and rx.Get(1) == 45 and
           stage.GetPrimAtPath("/Char/Rig/Extra").IsValid() and
           stage.GetPrimAtPath("/Char/Wrote").IsValid(),
           "the reply made all three kinds of change: %s" % text)
    box.finish()
    pushed = container.stack.pushed
    _Check(len(pushed) == 1,
           "pose, action and Python are one step: %s" % pushed)
    group = pushed[0] if pushed else None
    _Check(isinstance(group, session.GroupEdit) and len(group.edits) == 3 and
           container.stack.UndoText() == stage_actions.UNDO_LABEL,
           "the step groups all three: %s" % (getattr(group, "edits", None),))
    container.stack.Undo()
    _Check(abs(rx.Get(1)) < 1e-9 and
           root.GetPrimAtPath("/Char/Rig") is None and
           over.GetPrimAtPath("/Char/Wrote") is None and
           not stage.GetPrimAtPath("/Char/Wrote").IsValid(),
           "one undo puts every affected layer back")
    container.stack.Redo()
    _Check(rx.Get(1) == 45 and stage.GetPrimAtPath("/Char/Rig/Extra").IsValid()
           and stage.GetPrimAtPath("/Char/Wrote").IsValid(),
           "one redo makes every change again")


def TestPythonUndoCoverage():
    """What a run_python changed on every layer the reply started with goes
    on the undo step; a layer the code opened is reported instead."""
    _PythonWritesTwoLayers()
    _PythonRaisesAfterAnEdit()
    _PythonOpensALayer()
    _MixedPoseChangeAndPython()


def TestMcpServer():
    stage = _Stage()
    box, _api = _Toolbox(stage, _Container())
    server = mcp_server.McpServer()
    token = server.open(box)
    url = server.url

    def post(message, key=token):
        request = urllib.request.Request(
            url, data=json.dumps(message).encode("utf-8"),
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + key})
        try:
            with urllib.request.urlopen(request, timeout=10) as answer:
                return answer.status, json.loads(answer.read() or b"null")
        except urllib.error.HTTPError as err:
            return err.code, None

    try:
        status, answer = post({"jsonrpc": "2.0", "id": 1,
                               "method": "initialize", "params": {}})
        _Check(status == 200 and
               answer["result"]["serverInfo"]["name"] == "usdrig",
               "initialize names the usdrig server: %s" % answer)
        status, answer = post({"jsonrpc": "2.0", "id": 2,
                               "method": "tools/list"})
        listed = {t["name"] for t in answer["result"]["tools"]}
        _Check("get_stage_overview" in listed and "run_python" not in
               listed, "tools/list offers what is turned on: %s" % listed)
        status, _ = post({"jsonrpc": "2.0", "id": 3, "method": "ping"},
                         key="wrong")
        _Check(status == 401, "a wrong key is refused: %s" % status)
        server.close(token)
        status, _ = post({"jsonrpc": "2.0", "id": 4, "method": "ping"})
        _Check(status == 401, "a closed reply's key is refused")
        token2 = server.open(box)
        # tools/call runs on the main thread, so the call goes from a
        # worker while the main thread turns the event loop.
        import threading
        out = {}

        def call():
            out["answer"] = post({"jsonrpc": "2.0", "id": 5,
                                  "method": "tools/call",
                                  "params": {"name": "list_prims",
                                             "arguments": {"path": "/"}}},
                                 key=token2)

        thread = threading.Thread(target=call)
        thread.start()
        app = QtWidgets.QApplication.instance()
        while thread.is_alive():
            app.processEvents()
            thread.join(0.01)
        status, answer = out["answer"]
        content = answer["result"]["content"]
        _Check(status == 200 and not answer["result"]["isError"] and
               "/Char" in content[0]["text"],
               "tools/call runs a tool on the main thread: %s" % answer)
    finally:
        server.stop()


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _ = app
    for test in (TestLooking, TestChangesAreOneUndoStep, TestPosing,
                 TestRunPython, TestActionCapture, TestActionBlocks,
                 TestReplyTransaction, TestPythonUndoCoverage,
                 TestMcpServer):
        try:
            test()
        except Exception as err:
            import traceback
            traceback.print_exc()
            _failures.append("%s raised %s" % (test.__name__, err))
    if _failures:
        for failure in _failures:
            print("FAIL: " + failure)
        return 1
    print("ASSISTANT_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
