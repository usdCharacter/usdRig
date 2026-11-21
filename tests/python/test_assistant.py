#!/usr/bin/env python
"""
Headless test for the AI Assistant in plugin/rigExecUsdview/usdRigAssistant:
the tools on an in-memory stage (looking, posing, changing the stage,
running Python, and one undo step per reply), the action blocks of models
without tool calling, and the MCP server Claude Code connects to. A fake
usdviewApi stands in for usdview, and replies use a local stub provider.

Usage: test_assistant.py
"""
import json
import os
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["USDRIG_USER_DIR"] = tempfile.mkdtemp(prefix="usdrig_assistant_")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Tf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtCore, QtWidgets  # noqa: E402

import rigExecUndo  # noqa: E402
from usdRigAssistant import mcp_server, session, stage_actions  # noqa: E402
from usdRigAssistant import panel, providers, tools  # noqa: E402

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
    _Check(container.stack.CanUndo() and
           container.stack.UndoText() == "AI Assistant changes",
           "the reply's changes are one undo step, recorded as they are made")
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


def _SetRotation(box, value):
    return box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": value}]})


_PENDING_EXPIRY = ("The tool expired before it started; it did not run.", True)
_RUNNING_EXPIRY = ("The tool already started; its outcome is unknown. "
                   "Do not retry automatically.", True)
_QUEUE_BOUND = 3


def _PumpToolQueue(seconds, until=lambda: False):
    app = QtWidgets.QApplication.instance()
    end = time.monotonic() + seconds
    while not until() and time.monotonic() < end:
        app.processEvents()
        time.sleep(0.001)


def _DrainToolQueue():
    # Leave time for the 30 ms busy retry as well as posted deliveries.
    _PumpToolQueue(0.06)


@contextmanager
def _QueuedCalls(box):
    workers = []
    emitted = threading.Event()

    signal = box._invoke

    class ObservedSignal:
        def emit(self, job):
            signal.emit(job)
            emitted.set()

    def submit(fn):
        result = []
        emitted.clear()
        worker = threading.Thread(target=lambda: result.append(fn()), daemon=True)
        workers.append(worker)
        worker.start()
        if not emitted.wait(_QUEUE_BOUND):
            raise AssertionError("worker did not post its queued tool")
        return worker, result

    with patch.object(box, "_invoke", ObservedSignal()):
        try:
            yield submit
        finally:
            box.cancel()
            try:
                _DrainToolQueue()
            finally:
                for worker in workers:
                    worker.join(_QUEUE_BOUND)
                    _Check(not worker.is_alive(), "queued caller did not stop")


def TestQueuedEditExpiresBeforeDelivery():
    stage = _Stage()
    box, _ = _Toolbox(stage, _Container(), can_edit=True)
    with patch.object(tools, "CALL_TIMEOUT", 0.25), _QueuedCalls(box) as submit:
        worker, result = submit(lambda: _SetRotation(box, 20))
        worker.join(_QUEUE_BOUND)  # No Qt delivery before the caller returns.
        _Check(not worker.is_alive() and result == [_PENDING_EXPIRY],
               "undelivered edit expires pending: %s" % result)
        _DrainToolQueue()
        _Check(stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
               "late initial delivery does not author the expired edit")


def TestQueuedEditExpiresBeforeBusyRetry():
    stage = _Stage()
    entered = []
    retried = threading.Event()
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "note.txt")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write("a note to read")

        def ask_access(_folder, _reason):
            entered.append(True)
            single_shot = QtCore.QTimer.singleShot
            retries = []

            def busy_retry(delay, delivery):
                def retry():
                    delivery()
                    retried.set()

                retries.append(True)
                single_shot(delay, retry)

            with patch.object(tools, "CALL_TIMEOUT", 0.5):
                worker, result = submit(lambda: _SetRotation(box, 20))
                # Stop delivering Qt events at the first retry scheduling.
                with patch.object(QtCore.QTimer, "singleShot", busy_retry):
                    _PumpToolQueue(_QUEUE_BOUND, until=lambda: bool(retries))
                worker.join(_QUEUE_BOUND)
                _Check(retries == [True] and not worker.is_alive() and
                       result == [_PENDING_EXPIRY],
                       "second edit expires pending after busy delivery: %s" % result)
            return tools.ONCE

        box, _ = _Toolbox(stage, _Container(), can_edit=True,
                          memory=tools.Memory(prefs={}), ask_access=ask_access)
        with patch.object(tools, "CALL_TIMEOUT", 10), _QueuedCalls(box) as submit:
            first, result = submit(lambda: box.call("read_file", {"path": path}))
            _DrainToolQueue()
            first.join(_QUEUE_BOUND)
            _Check(entered == [True] and result and not result[0][1] and
                   "a note to read" in result[0][0], "the first tool completes its prompt")
            _PumpToolQueue(_QUEUE_BOUND, until=retried.is_set)
            _Check(retried.is_set() and
                   stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
                   "the delivered busy retry does not author the expired edit")


def TestCancelWakesQueuedEdit():
    stage = _Stage()
    box, _ = _Toolbox(stage, _Container(), can_edit=True)
    with patch.object(tools, "CALL_TIMEOUT", 10), _QueuedCalls(box) as submit:
        worker, result = submit(lambda: _SetRotation(box, 20))
        box.cancel()
        worker.join(_QUEUE_BOUND)  # Neither queue delivery nor deadline can wake it.
        _Check(not worker.is_alive() and result == [("The user stopped the reply.", True)],
               "cancel wakes the pending caller with a stopped error: %s" % result)
        _DrainToolQueue()
        _Check(_SetRotation(box, 30) == ("The user stopped the reply.", True) and
               stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
               "cancelled pending and subsequent edits are never authored")


def TestRunningEditCanCompleteAfterCallerDeadline():
    stage = _Stage()
    box, _ = _Toolbox(stage, _Container(), can_edit=True)
    tool = box.tools["change_stage"]
    handler = tool.handler
    entered = threading.Event()
    release = threading.Event()
    completions = []
    activity = []

    def gated_handler(args):
        entered.set()
        if not release.wait(_QUEUE_BOUND):
            raise AssertionError("running caller did not release the handler")
        result = handler(args)
        completions.append(result)
        return result

    tool.handler = gated_handler
    box.activity.connect(lambda event: activity.append(event["state"]))

    def request():
        try:
            result = _SetRotation(box, 20)
            _Check(entered.is_set(), "caller timed out after actual handler entry")
            return result
        finally:
            release.set()

    with patch.object(tools, "CALL_TIMEOUT", 0.5), _QueuedCalls(box) as submit:
        try:
            worker, result = submit(request)
            _DrainToolQueue()
            worker.join(_QUEUE_BOUND)
            _Check(not worker.is_alive() and result == [_RUNNING_EXPIRY],
                   "running edit reports unknown outcome: %s" % result)
            _DrainToolQueue()
            _Check(len(completions) == 1 and activity == ["running", "done"] and
                   stage.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 20,
                   "the running edit completes and authors exactly once")
        finally:
            release.set()


def TestToolboxRejectsSilentStageReplacement():
    a, b = _Stage(), _Stage()
    container = _Container()
    fresh, _ = _Toolbox(b, container, can_edit=True)
    text, failed = _SetRotation(fresh, 10)
    _Check(not failed and container.stack.UndoText() == "AI Assistant changes",
           "B can author an edit: %s" % text)

    box, api = _Toolbox(a, container, can_edit=True)
    text, failed = _SetRotation(box, 20)
    _Check(not failed, "A can author an edit: %s" % text)
    a_before = a.GetRootLayer().ExportToString()
    b_before = b.GetRootLayer().ExportToString()
    api.stage = b
    api.selectedPrims = [b.GetPrimAtPath("/Char/Geo")]
    text, failed = _SetRotation(box, 30)
    _Check(failed, "old toolbox cannot author at the matching path: %s" % text)
    text, failed = box.call("select_prims", {"paths": ["/Char/Ctl"]})
    _Check(failed and api.selectedPrims == [b.GetPrimAtPath("/Char/Geo")],
           "old toolbox leaves B's selection alone: %s" % text)
    _Check(a.GetRootLayer().ExportToString() == a_before and
           b.GetRootLayer().ExportToString() == b_before,
           "rejected edit leaves both authored layers unchanged")
    container.stack.Undo()
    _Check(a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0 and
           b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10,
           "one undo takes back A's edit and leaves B's alone")
    container.stack.Undo()
    _Check(b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0 and
           not container.stack.CanUndo(),
           "the old toolbox pushed no step of its own")
    container.stack.Redo()
    container.stack.Redo()
    _Check(a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 20 and
           b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10,
           "each step affects only the stage it was opened for")
    api.stage = a
    text, failed = _SetRotation(box, 40)
    # The toolbox answers for the stage object it was opened for. A host
    # that silently shows that very object again makes it current once more;
    # only the panel's document generation ends a reply for good
    # (TestToolboxGenerationAndRetirement, and the A→B→A reopen in
    # TestProviderSilentSwapAndReopen).
    _Check(not failed and box.is_current() and
           a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 40 and
           b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10,
           "A is current again, so the toolbox bound to A works on A: %s" % text)
    container.stack.Undo()
    _Check(a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0 and
           b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10 and
           _UndoState(container.stack) == (True, "AI Assistant changes", True,
                                 "AI Assistant changes"),
           "A's reply is one step, and B's step is untouched beside it")


def TestToolboxGenerationAndRetirement():
    stage = _Stage()
    generation = 0
    container = _Container()
    box, api = _Toolbox(stage, container, can_edit=True,
                        document_current=lambda: generation == 0)
    text, failed = _SetRotation(box, 10)
    _Check(not failed and box.is_current(), "the current generation can edit: %s" % text)
    text, failed = _SetRotation(box, 20)
    _Check(not failed, "later calls keep the same step: %s" % text)
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    _Check(rx.Get() == 20 and container.stack.UndoText() == "AI Assistant changes",
           "each change is recorded as it is authored")
    container.stack.Undo()
    _Check(rx.Get() == 0 and not container.stack.CanUndo(),
           "both changes are one step, pushed once")
    generation += 1
    text, failed = _SetRotation(box, 30)
    _Check(failed and api.stage is box.stage and rx.Get() == 0,
           "generation revokes authority even on the same stage: %s" % text)

    live, _ = _Toolbox(stage, container, can_edit=True)
    text, failed = _SetRotation(live, 15)
    _Check(not failed, "the current toolbox can edit: %s" % text)
    live.cancel()
    live.cancel()
    content, failed = live.call_content("select_prims", {"paths": ["/Char/Ctl"]})
    _Check(live.is_current() and failed and content[0]["type"] == "text" and
           content[0]["text"], "cancel closes admission, not document identity")
    container.stack.Undo()
    _Check(rx.Get() == 0, "a cancelled reply's edit is still undone")


class _ArtistEdit(object):
    """A change the artist made, as the editor would record it."""

    label = "Artist rotation"

    def __init__(self, attribute, value):
        self._attribute = attribute
        self._before = attribute.Get()
        self._after = value

    def Undo(self):
        self._attribute.Set(self._before)

    def Redo(self):
        self._attribute.Set(self._after)


def TestArtistCommitStartsANewStep():
    stage = _Stage()
    container = _Container()
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    box, _api = _Toolbox(stage, container, can_edit=True)
    _SetRotation(box, 10)
    _SetRotation(box, 20)
    _Check(rx.Get() == 20 and container.stack.UndoText() == "AI Assistant changes",
           "the reply's edits are one step")
    container.stack.Undo()
    _Check(rx.Get() == 0 and container.stack.Redo(), "both edits undo together")

    container.stack.Push(_ArtistEdit(rx, 5))
    rx.Set(5)
    _Check(rx.Get() == 5, "the artist turns the control")
    text, failed = _SetRotation(box, 30)
    _Check(not failed and rx.Get() == 30,
           "the same reply can edit again: %s" % text)
    container.stack.Undo()
    _Check(rx.Get() == 5 and container.stack.UndoText() == "Artist rotation",
           "the artist's change is the newest step, not the reply's")
    container.stack.Undo()
    _Check(rx.Get() == 20 and container.stack.UndoText() == "AI Assistant changes",
           "the artist's change comes off before the reply's")
    container.stack.Undo()
    _Check(rx.Get() == 0 and not container.stack.CanUndo(),
           "the reply's two changes are still one step")


def TestEditsUndoInTheOrderTheyWereMade():
    """The history undoes and redoes in the order the changes were made,
    even when the editor commits one of its own while a change the reply
    is still authoring is in flight."""
    stage = _Stage()
    container = _Container()
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    box, _api = _Toolbox(stage, container, can_edit=True)
    _SetRotation(box, 11)
    _Check(rx.Get() == 11 and
           _UndoState(container.stack) == (True, "AI Assistant changes",
                                          False, ""),
           "the reply keys the control and has one step")
    made = []

    def select_with_an_artist_commit(paths):
        # The editor commits a change of its own in the middle of the
        # reply's batch, as a queued slider commit does.
        made.append(rx.Get())
        container.stack.Push(_ArtistEdit(rx, 23))
        rx.Set(23)

    box._select = select_with_an_artist_commit
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:ty",
         "value": 5},
        {"action": "select", "paths": ["/Char/Ctl"]},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 30},
    ]})
    _Check(not failed and made == [11] and _Values(stage) == (30.0, 5.0),
           "the artist commit lands between the reply's changes: %s" % text)
    _Check(_UndoState(container.stack) == (True, "AI Assistant changes",
                                          False, ""),
           "the step written after the artist commit is the newest")
    container.stack.Undo()
    _Check(_Values(stage) == (23.0, 5.0) and
           container.stack.UndoText() == "Artist rotation",
           "the first undo takes back the change made after the artist")
    container.stack.Undo()
    _Check(_Values(stage) == (11.0, 5.0) and
           container.stack.UndoText() == "AI Assistant changes",
           "the next undo takes back the artist's commit")
    container.stack.Undo()
    _Check(_Values(stage) == (0.0, 0.0) and not container.stack.CanUndo(),
           "the last undo takes back what the reply made first")
    for wanted in ((11.0, 5.0), (23.0, 5.0), (30.0, 5.0)):
        _Check(container.stack.Redo() and _Values(stage) == wanted,
               "redo puts the changes back in the order they were made")
    _Check(not container.stack.CanRedo(), "redo stops at the newest change")
    container.stack.Undo()
    _Check(container.stack.CanRedo(), "undo leaves a redo branch")
    _SetRotation(box, 40)
    _Check(not container.stack.CanRedo(),
           "a new write after undo discards the stale redo branch")


def TestACommitFromAStageNoticeKeepsTheAuthoredOrder():
    """A change the editor commits from a USD notice the reply's own write
    fires is recorded after that write, so the history still undoes in the
    order the changes were made."""
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    channel = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    _Check(channel.Get() == 0, 'the notice test starts at zero')
    made = []

    def artist_releases_the_slider(notice, sender):
        # The editor commits the artist's slider release from the notice
        # the reply's change fires, before that change is recorded.
        if made or channel.Get() != 11:
            return
        made.append(channel.Get())
        container.stack.Push(_ArtistEdit(channel, 23))
        channel.Set(23)

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                  artist_releases_the_slider, stage)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 11}]})
    listener.Revoke()
    _Check(not failed and made == [11] and channel.Get() == 23,
           "the artist commit lands while the reply writes: %s" % text)
    _Check(_UndoState(container.stack) == (True, "Artist rotation", False, ""),
           "the reply's step is on the history before the artist commit")
    container.stack.Undo()
    _Check(channel.Get() == 11 and
           container.stack.UndoText() == "AI Assistant changes",
           "the first undo takes back the change made last")
    container.stack.Undo()
    _Check(channel.Get() == 0 and not container.stack.CanUndo(),
           "the next undo takes back the change that began first")
    for wanted in (11, 23):
        _Check(container.stack.Redo() and channel.Get() == wanted,
               "redo puts the changes back in the order they were made")
    _Check(not container.stack.CanRedo(), "redo stops at the newest change")


def TestAWriteAfterAnUndoDoesNotGrowAnOlderStep():
    """An undo that took a later change off the history closes that step to
    the reply that wrote before it: the next change is a step of its own
    and discards what the undo left to be redone."""
    stage = _Stage()
    container = _Container()
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    box, _api = _Toolbox(stage, container, can_edit=True)
    _SetRotation(box, 11)
    container.stack.Push(_ArtistEdit(rx, 23))
    rx.Set(23)
    _Check(_UndoState(container.stack) == (True, "Artist rotation", False, ""),
           "the artist change is the newest step")
    container.stack.Undo()
    _Check(rx.Get() == 11 and
           _UndoState(container.stack) == (True, "AI Assistant changes",
                                          True, "Artist rotation"),
           "undoing the artist change leaves it waiting to be redone")
    text, failed = _SetRotation(box, 31)
    _Check(not failed and rx.Get() == 31,
           "the same reply writes again: %s" % text)
    _Check(not container.stack.CanRedo(),
           "the new write discards the redo branch the undo left")
    _Check(_UndoState(container.stack) == (True, "AI Assistant changes",
                                          False, ""),
           "the new write is a step of its own, not a late addition")
    container.stack.Undo()
    _Check(rx.Get() == 11, "the first undo takes back the newest change")
    container.stack.Undo()
    _Check(rx.Get() == 0 and not container.stack.CanUndo(),
           "the older change is still undone on its own")


def _CommitFromAPoseNotice(pose, label):
    """The reply poses a control to 11, and the artist's slider release
    arrives from the USD notice that write fires. The reply's step is
    already on the history when the notice runs, so Undo takes 23 back
    first and 11 after it, with no empty step between them, and Redo
    puts them back in the order they were made."""
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    channel = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    made = []

    def artist_releases_the_slider(notice, sender):
        if made or channel.Get() != 11:
            return
        made.append(channel.Get())
        container.stack.Push(_ArtistEdit(channel, 23))
        channel.Set(23)

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                  artist_releases_the_slider, stage)
    text, failed = pose(box)
    listener.Revoke()
    _Check(not failed and made == [11] and channel.Get() == 23,
           'the artist commit came from the notice the pose fired: %s' % text)
    _Check(len(container.stack._undo) == 2 and
           _UndoState(container.stack) == (True, 'Artist rotation', False,
                                           ''),
           'the pose is on the history before the artist commit, and '
           'nothing empty is left between them')
    _Check(container.stack.Undo() and channel.Get() == 11 and
           container.stack.UndoText() == label,
           'the first undo takes back the change made last')
    _Check(container.stack.Undo() and channel.Get() == 0 and
           not container.stack.CanUndo(),
           'the next undo takes back the change that began first, and the '
           'history stops there')
    for wanted in (11, 23):
        _Check(container.stack.Redo() and channel.Get() == wanted,
               'redo puts the changes back in the order they were made')
    _Check(not container.stack.CanRedo(), 'redo stops at the newest change')


def TestAPoseCommitFromAStageNoticeKeepsTheAuthoredOrder():
    _CommitFromAPoseNotice(
        lambda box: box.call('set_avars', {
            'values': [{'path': '/Char/Ctl', 'name': 'avars:rx',
                        'value': 11}], 'key': False}),
        'AI Assistant pose')


def _LayeredStage(base_rotation):
    """The rig authored in a base layer under a blank root, so clearing a
    value the reply wrote resolves back to the base instead of nothing."""
    base = Sdf.Layer.CreateAnonymous('assistant_base.usda')
    base.ImportFromString(
        _LAYER.replace('avars:rx = 0', 'avars:rx = %g' % base_rotation))
    root = Sdf.Layer.CreateAnonymous('assistant_root.usda')
    root.subLayerPaths.append(base.identifier)
    return Usd.Stage.Open(root)


def TestAResetCommitFromAStageNoticeKeepsTheAuthoredOrder():
    """The reset producer is sealed the same way as the pose one: the
    artist commit its notice carries follows the reset in history."""
    stage = _LayeredStage(4)
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    channel = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    channel.Set(11)  # the control sits at 11 already
    _Check(channel.Get() == 11, 'the reset starts from eleven')
    made = []

    def artist_pushes_it_back(notice, sender):
        if made or channel.Get() != 4:
            return
        made.append(channel.Get())
        container.stack.Push(_ArtistEdit(channel, 23))
        channel.Set(23)

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                  artist_pushes_it_back, stage)
    text, failed = box.call('reset_avars', {'paths': ['/Char/Ctl'],
                                            'names': ['avars:rx'],
                                            'key': False})
    listener.Revoke()
    _Check(not failed and made == [4] and channel.Get() == 23,
           'the artist commit came from the notice the reset fired: %s'
           % text)
    _Check([step.label for step in container.stack._undo] ==
           ['AI Assistant reset', 'Artist rotation'],
           'the reset is on the history before the artist commit, with no '
           'empty step between them')
    _Check(container.stack.Undo() and channel.Get() == 4 and
           container.stack.UndoText() == 'AI Assistant reset',
           'the first undo takes back the change made last')
    _Check(container.stack.Undo() and channel.Get() == 11 and
           not container.stack.CanUndo(),
           'the next undo takes back the reset that began first, and the '
           'history stops there')
    for wanted in (4, 23):
        _Check(container.stack.Redo() and channel.Get() == wanted,
               'redo puts the changes back in the order they were made')
    _Check(not container.stack.CanRedo(), 'redo stops at the newest change')


def TestAClearCommitFromAStageNoticeKeepsTheAuthoredOrder():
    """Clearing is sealed the same way as the pose and reset writes: the
    value the base layer reveals is recorded, and the artist commit the
    clear's notice carries follows it in history."""
    stage = _LayeredStage(4)
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True)
    channel = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    channel.Set(11)  # the control sits at 11 already
    _Check(channel.Get() == 11, 'the clear starts from eleven')
    made = []

    def artist_writes_after_the_clear(notice, sender):
        if made or channel.Get() != 4:
            return
        made.append(channel.Get())
        container.stack.Push(_ArtistEdit(channel, 23))
        channel.Set(23)

    listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged,
                                  artist_writes_after_the_clear, stage)
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'clear_attr', 'path': '/Char/Ctl',
         'name': 'avars:rx'}]})
    listener.Revoke()
    _Check(not failed and made == [4] and channel.Get() == 23,
           'the artist commit came from the notice the clear fired: %s'
           % text)
    _Check([step.label for step in container.stack._undo] ==
           ['AI Assistant changes', 'Artist rotation'],
           'the clear is on the history before the artist commit, with no '
           'empty step between them')
    _Check(container.stack.Undo() and channel.Get() == 4 and
           container.stack.UndoText() == 'AI Assistant changes',
           'the first undo takes back the change made last')
    _Check(container.stack.Undo() and channel.Get() == 11 and
           not container.stack.CanUndo(),
           'the next undo takes back the clear that began first, and the '
           'history stops there')
    for wanted in (4, 23):
        _Check(container.stack.Redo() and channel.Get() == wanted,
               'redo puts the changes back in the order they were made')
    _Check(not container.stack.CanRedo(), 'redo stops at the newest change')


def TestACallThatChangesNothingLeavesHistoryAlone():
    """A read-only call, a change that cannot be made and a write of the
    value that is already there record no undo step: the redo branch an
    undo left is still there, and so is every step under it."""
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True, can_run=True)
    _SetRotation(box, 11)
    container.stack.Undo()
    _Check(_UndoState(container.stack) == (False, '', True,
                                          'AI Assistant changes') and
           _Values(stage) == (0.0, 0.0),
           'undo leaves the change waiting to be redone')
    text, failed = box.call('run_python', {
        'code': "print(stage.GetPrimAtPath('/Char/Ctl').GetPath())"})
    _Check(not failed and _UndoState(container.stack) ==
           (False, '', True, 'AI Assistant changes'),
           'a read-only call leaves the history as it was: %s' % text)
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Nope', 'name': 'avars:rx',
         'value': 3}]})
    _Check(text.startswith('Could not') and container.stack.CanRedo(),
           'a change that could not be made leaves the redo branch: %s'
           % text)
    text, failed = _SetRotation(box, 0)
    _Check(not failed and container.stack.CanRedo() and
           not container.stack.CanUndo(),
           'writing the value that is already there records nothing: %s'
           % text)
    _Check(container.stack.Redo() and _Values(stage) == (11.0, 0.0) and
           not container.stack.CanRedo(),
           'the redo branch still holds the change the reply made')


class _ProtocolStack(object):
    """An undo stack that implements the documented host protocol and
    nothing more, so it cannot say which edit is newest."""

    def __init__(self):
        self._undo = []
        self._redo = []

    def Push(self, edit):
        self._undo.append(edit)
        self._redo = []

    def Undo(self):
        if not self._undo:
            return False
        edit = self._undo.pop()
        edit.Undo()
        self._redo.append(edit)
        return True

    def Redo(self):
        if not self._redo:
            return False
        edit = self._redo.pop()
        edit.Redo()
        self._undo.append(edit)
        return True

    def CanUndo(self):
        return bool(self._undo)

    def CanRedo(self):
        return bool(self._redo)

    def UndoText(self):
        return self._undo[-1].label if self._undo else ""

    def RedoText(self):
        return self._redo[-1].label if self._redo else ""

    def Clear(self):
        self._undo = []
        self._redo = []

    def AddListener(self, fn):
        pass

    def AddFlusher(self, fn):
        pass


class _ProtocolContainer(object):
    def __init__(self):
        self.stack = _ProtocolStack()

    def _UndoStack(self):
        return self.stack

    def IsRigActive(self):
        return False

    def ImagingHandle(self):
        return None


def TestHostUndoStackWithoutLastEdit():
    """A host stack following the documented protocol, which does not
    include LastEdit, still has every change the assistant makes."""
    stage = _Stage()
    container = _ProtocolContainer()
    box, _api = _Toolbox(stage, container, can_edit=True)
    _SetRotation(box, 11)
    _Check(box.is_current() and _Values(stage) == (11.0, 0.0) and
           len(container.stack._undo) == 1 and
           container.stack.UndoText() == "AI Assistant changes",
           "the first change is pushed on its own")
    _SetRotation(box, 23)
    _Check(_Values(stage) == (23.0, 0.0) and
           len(container.stack._undo) == 2 and
           container.stack.UndoText() == "AI Assistant changes",
           "the second change is a separate edit, not a lost one")
    container.stack.Undo()
    _Check(_Values(stage) == (11.0, 0.0) and container.stack.CanUndo(),
           "the first undo takes back the second write")
    container.stack.Undo()
    _Check(_Values(stage) == (0.0, 0.0) and not container.stack.CanUndo() and
           container.stack.CanRedo(),
           "the second undo takes back the first write")
    _Check(container.stack.Redo() and _Values(stage) == (11.0, 0.0) and
           container.stack.Redo() and _Values(stage) == (23.0, 0.0),
           "both writes come back on redo")


class _GroupingHostStack(_ProtocolStack):
    """A host stack that can also say which step is newest, and offers no
    way to take a step back: the assistant can only record what it
    actually changed."""

    def LastEdit(self):
        return self._undo[-1] if self._undo else None


class _GroupingHostContainer(_ProtocolContainer):
    def __init__(self):
        self.stack = _GroupingHostStack()


def TestHostUndoStackWithoutWithdraw():
    """A host stack with the newest-step identity and nothing to take a
    step back still gets no empty step from a call that changes nothing."""
    stage = _Stage()
    container = _GroupingHostContainer()
    box, _api = _Toolbox(stage, container, can_edit=True, can_run=True)
    channel = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    _SetRotation(box, 11)
    container.stack.Push(_ArtistEdit(channel, 30))
    channel.Set(30)
    _Check(len(container.stack._undo) == 2 and
           container.stack.UndoText() == 'Artist rotation',
           'the reply and the artist each have their own step')
    _SetRotation(box, 30)
    box.call('run_python', {'code': "stage.GetPrimAtPath('/Char/Ctl')"})
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Nope', 'name': 'avars:rx',
         'value': 5}]})
    _Check(len(container.stack._undo) == 2 and
           text.startswith('Could not') and
           container.stack.UndoText() == 'Artist rotation',
           'the calls that changed nothing left no step behind: %s' % text)
    for wanted in (11.0, 0.0):
        _Check(container.stack.Undo() and _Values(stage) == (wanted, 0.0),
               'each step undoes on its own')
    _Check(not container.stack.CanUndo() and container.stack.CanRedo(),
           'the redo branch holds the two steps')
    for wanted in (11.0, 30.0):
        _Check(container.stack.Redo() and _Values(stage) == (wanted, 0.0),
               'and redo puts them back in order')
    _Check(not container.stack.CanRedo(), 'redo stops at the newest step')


def TestANoChangeAtTheHistoryLimitKeepsTheOldestStep():
    """A full history keeps its oldest step through a call that records
    nothing: there is no empty step to push and drop it."""
    stage = _Stage()
    container = _Container()
    box, _api = _Toolbox(stage, container, can_edit=True, can_run=True)
    rx = stage.GetAttributeAtPath('/Char/Ctl.avars:rx')
    limit = rigExecUndo.UndoStack.LIMIT
    oldest = _ArtistEdit(rx, 1)
    container.stack.Push(oldest)
    for _ in range(limit - 1):
        container.stack.Push(_ArtistEdit(rx, 2))
    _Check(len(container.stack._undo) == limit, 'the history is full')
    _SetRotation(box, 0)
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Nope', 'name': 'avars:rx',
         'value': 5}]})
    _Check(len(container.stack._undo) == limit and
           container.stack._undo[0] is oldest,
           'a call that changed nothing pushed nothing at the limit: %s'
           % text)
    _Check(container.stack.Undo() and rx.Get() == 0 and
           container.stack.Redo() and rx.Get() == 2,
           'the full history still undoes and redoes')


def TestStageReplacementMidBatchStopsTheRest():
    a, b = _Stage(), _Stage()
    container = _Container()
    box, live = _Toolbox(a, container, can_edit=True)

    class _SelectionOpensAnotherStage(_Selection):
        def addPrim(self, prim):
            _Selection.addPrim(self, prim)
            live.stage = b

    live.dataModel.selection = _SelectionOpensAnotherStage(live)
    b_before = b.GetRootLayer().ExportToString()
    text, _failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 10},
        {"action": "select", "paths": ["/Char/Ctl"]},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:ty",
         "value": 20}]})
    _Check("Could not: The document is no longer current: 1 of the changes "
           "were not made" in text and
           a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10 and
           a.GetAttributeAtPath("/Char/Ctl.avars:ty").Get() == 0 and
           b.GetRootLayer().ExportToString() == b_before,
           "the batch stops where the document stops being current: %s" % text)
    container.stack.Undo()
    _Check(a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0 and
           live.selectedPrims,
           "the changes the batch made stay undoable, and the selection is the new one")


def TestSharedLayerReplacementMidBatchStopsTheRest():
    a = _Stage()
    b = Usd.Stage.Open(a.GetRootLayer())  # another document over the same layer
    assert a is not b and a.GetRootLayer() == b.GetRootLayer()
    container = _Container()
    box, live = _Toolbox(a, container, can_edit=True)

    class _SelectionOpensAnotherStage(_Selection):
        def addPrim(self, prim):
            _Selection.addPrim(self, prim)
            live.stage = b

    live.dataModel.selection = _SelectionOpensAnotherStage(live)
    text, _failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:rx",
         "value": 10},
        {"action": "select", "paths": ["/Char/Ctl"]},
        {"action": "set_attr", "path": "/Char/Ctl", "name": "avars:ty",
         "value": 22}]})
    _Check("The document is no longer current" in text and
           a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10 and
           b.GetAttributeAtPath("/Char/Ctl.avars:ty").Get() == 0,
           "the batch stops at the replacement although the layer is shared: %s" % text)
    container.stack.Undo()
    _Check(a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0 and
           b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 0,
           "the undo reaches back through the shared layer")


def TestToolboxSwapDuringActivity():
    a, b = _Stage(), _Stage()
    box, api = _Toolbox(a, _Container(), can_edit=True)
    a_before = a.GetRootLayer().ExportToString()
    b_before = b.GetRootLayer().ExportToString()

    def on_activity(event):
        if event["state"] == "running":
            api.stage = b

    box.activity.connect(on_activity)
    try:
        text, failed = _SetRotation(box, 30)
        _Check(failed and api.stage is b and
               a.GetRootLayer().ExportToString() == a_before and
               b.GetRootLayer().ExportToString() == b_before,
               "swap in running notification prevents authoring: %s" % text)
    finally:
        box.activity.disconnect(on_activity)


def TestToolboxFolderGrantAfterPrompt():
    stage, replacement = _Stage(), _Stage()
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "note.txt")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write("a note to read")
        for answer in (tools.ONCE, tools.ALWAYS):
            for swap in (True, False):
                memory = tools.Memory(prefs={})
                chat_folders = set()
                prompts = []

                def ask_access(requested, reason):
                    prompts.append(requested)
                    if swap:
                        api.stage = replacement
                    return answer

                box, api = _Toolbox(stage, _Container(), memory=memory,
                                    chat_folders=chat_folders,
                                    ask_access=ask_access)
                text, failed = box.call("read_file", {"path": path})
                granted = (folder in chat_folders if answer == tools.ONCE
                           else memory.allowed(path))
                _Check(prompts == [folder] and failed == swap and
                       ("a note to read" in text) != swap and
                       granted != swap and
                       (not swap or not chat_folders and not memory.folders()),
                       "%s grants access only while current: %s" % (answer, text))


def TestToolboxNoStageWithoutApp():
    box, api = _Toolbox(None, _Container())
    text, failed = box.call("get_output_log", {})
    _Check(not failed, "stage-free tools work on owner without an app: %s" % text)
    results = []
    worker = threading.Thread(target=lambda: results.append(
        box.call("get_output_log", {})))
    worker.start()
    worker.join(2)
    _Check(not worker.is_alive() and results and results[0][1] and results[0][0],
           "off-owner calls without an app get a textual error")
    api.stage = _Stage()
    schemas = (box.anthropic_tools(), box.responses_tools(),
               [t["function"] for t in box.chat_completions_tools()], box.mcp_tools())
    for schema in schemas:
        names = {t["name"] for t in schema}
        _Check("get_output_log" in names and "list_prims" not in names,
               "schemas use the captured absence of a stage")
    text, failed = box.call("get_output_log", {})
    _Check(failed, "even a stage-free tool rejects a stale reply: %s" % text)


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
    container.stack.Undo()
    _Check(not stage.GetPrimAtPath("/Char/Made").IsValid(),
           "undo takes back what the Python changed on the edit target")


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


_CTL = '/Char/Ctl'
_NATIVE_ACTIONS = [
    {'action': 'set_attr', 'path': _CTL, 'name': 'avars:rx', 'value': 30},
    {'action': 'bad_native_action'},
]
_TEXT_ACTIONS = [
    {'action': 'set_attr', 'path': _CTL, 'name': 'avars:ty', 'value': 12},
    {'action': 'bad_text_action'},
]
_ACTION_REPLY = ('Here is the translation.\n```usdrig-actions\n' +
                 json.dumps(_TEXT_ACTIONS) + '\n```')
_FRESH_TEXT = 'This is the replacement document reply.'
_PLAIN_TEXT = 'Noted.'


def _PumpUntil(predicate, message):
    app = QtWidgets.QApplication.instance()
    deadline = time.monotonic() + _QUEUE_BOUND
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError(message)
        app.processEvents()


class _LocalReply:
    def __init__(self, route, gated=False):
        self.route = route
        self.posted = threading.Event()
        self.release = threading.Event()
        if not gated:
            self.release.set()
        self.passes = []
        self.sent = None
        self.job = None
        self.native_result = None

    def wait_for_release(self):
        if not self.release.wait(3 * _QUEUE_BOUND):
            raise AssertionError('provider was not released')


class _LocalProvider(providers.Provider):
    id = 'local-test'
    label = 'Local test provider'

    def __init__(self, replies, prefs):
        super().__init__(prefs=prefs)
        self.replies = replies

    def status(self):
        return 'Ready locally.'

    def cached_models(self):
        return ['test-model']

    def stream_chat(self, messages, system, model=None, tools=None):
        reply = self.replies[messages[-1]['content']]
        reply.passes.append((model, tools is not None))
        reply.sent = [message['content'] for message in messages]
        if reply.route == 'fallback' and tools is not None:
            raise providers.ToolsUnsupported('Try without native tools')
        if reply.route in ('native', 'mixed'):
            if tools is None:
                raise AssertionError('expected a native toolbox')
            signal = tools._invoke

            class ObservedSignal:
                def emit(self, job):
                    signal.emit(job)
                    reply.job = (job.name, job.args)
                    reply.posted.set()

            with patch.object(tools, '_invoke', ObservedSignal()):
                reply.native_result = tools.call(
                    'change_stage', {'changes': _NATIVE_ACTIONS})
            reply.wait_for_release()
        if reply.route == 'fresh':
            yield _FRESH_TEXT
        elif reply.route == 'native':
            yield 'Here is the rotation.'
        elif reply.route == 'plain':
            yield _PLAIN_TEXT
        else:
            yield _ACTION_REPLY
        reply.wait_for_release()


@contextmanager
def _ProviderPanel(api, container, replies, edit=True):
    prefs = {}
    provider = _LocalProvider(replies, prefs)
    with patch.object(providers, 'get_providers', return_value=[provider]), \
            patch.object(providers, 'get_provider', return_value=provider):
        ui = panel.AiAssistant(session.Session(api, container), prefs=prefs)
        try:
            ui.include_check.setChecked(False)
            ui.edit_check.setChecked(edit)
            yield ui
        finally:
            for reply in replies.values():
                reply.release.set()
            ui.stop()
            for worker in [ui.thread] + list(ui.left_threads):
                if worker is not None:
                    _Check(worker.wait(_QUEUE_BOUND * 1000),
                           'local provider worker did not stop')
            ui.close()
            _PumpUntil(lambda: not ui.left_threads,
                       'local provider completion deliveries were not drained')


def _ProviderFinished(ui):
    _PumpUntil(lambda: not ui.busy and not ui.left_threads,
               'local provider reply did not complete')


def _Values(stage):
    return tuple(stage.GetAttributeAtPath(_CTL + '.' + name).Get()
                 for name in ('avars:rx', 'avars:ty'))


def _UndoState(stack):
    return (stack.CanUndo(), stack.UndoText(), stack.CanRedo(), stack.RedoText())


def _DocumentState(stage):
    return (stage.GetEditTarget(), tuple(
        (layer.identifier, layer.ExportToString())
        for layer in stage.GetLayerStack()))


def TestProviderCompletion():
    for route, values, done, failed in (
            ('native', (30, 0), ['avars:rx'], ['bad_native_action']),
            ('text', (0, 12), ['avars:ty'], ['bad_text_action']),
            ('fallback', (0, 12), ['avars:ty'], ['bad_text_action']),
            ('mixed', (30, 12), ['avars:rx', 'avars:ty'],
             ['bad_native_action', 'bad_text_action'])):
        stage = _Stage()
        baseline = _DocumentState(stage)
        api, container = _Api(stage), _Container()
        reply = _LocalReply(route)
        with _ProviderPanel(api, container, {'edit': reply}) as ui:
            ui.send('edit')
            box = ui.toolbox
            _ProviderFinished(ui)
            _Check(_Values(stage) == values, route + ': authored values')
            assert [m['role'] for m in ui.messages] == ['user', 'assistant'], route
            message = ui.messages[-1]
            expected_done = [f'Set {_CTL}.{name} to {value}'
                             for name, value in zip(('avars:rx', 'avars:ty'), values)
                             if name in done]
            _Check(message.get('done') == expected_done and
                   message.get('failed') == ['There is no action ' + name
                                             for name in failed],
                   route + ': each action outcome is recorded once: ' + str(message))
            passes = [('test-model', True)]
            if route == 'fallback':
                passes.append(('test-model', False))
            _Check(reply.passes == passes,
                   route + ': actual provider passes: ' + str(reply.passes))
            if route in ('native', 'mixed'):
                _Check(reply.native_result is not None and not reply.native_result[1],
                       route + ': native tool returned successfully')
            if route == 'fallback':
                _Check([entry['state'] for entry in message['tools']] == ['note', 'done'],
                       'fallback records the unsupported-tools note and final dispatch')
            stack = container.stack
            _Check(stack.UndoText() == 'AI Assistant changes', route + ': undo label')
            stack.Undo()
            _Check(_Values(stage) == (0, 0) and _DocumentState(stage) == baseline and
                   not stack.CanUndo() and stack.CanRedo(),
                   route + ': a single undo restores all authored edits')
            _Check(_SetRotation(box, 99)[1] and _DocumentState(stage) == baseline,
                   route + ': a retained toolbox cannot edit after normal completion')


def _WaitForAttempt(ui, reply):
    if reply.route == 'native':
        # Do not pump: emit must have posted a real change_stage job to Qt.
        posted = reply.posted.wait(_QUEUE_BOUND)
        assert posted, 'native edit was not queued'
        assert reply.job == ('change_stage', {'changes': _NATIVE_ACTIONS})
        assert reply.native_result is None, 'queued edit returned before owner delivery'
    else:
        _PumpUntil(lambda: ui.reply == _ACTION_REPLY, 'action chunk was not delivered')
        assert ui.thread.isRunning(), 'provider finished before the swap barrier'
        assert stage_actions.find_actions(ui.reply) == (_TEXT_ACTIONS, [])


def _TestProviderSwap(route, replacement):
    a, b = _Stage(), _Stage()
    assert a is not b and a.GetRootLayer() != b.GetRootLayer()
    assert _Values(a) == _Values(b) == (0, 0), 'both documents have the target attributes'
    api, container = _Api(a), _Container()
    seed, _ = _Toolbox(b, container, can_edit=True)
    text, failed = _SetRotation(seed, 10)
    assert not failed, text
    container.stack.Undo()
    b.SetEditTarget(Usd.EditTarget(b.GetSessionLayer()))
    baseline_a, baseline_b = _DocumentState(a), _DocumentState(b)
    stack = container.stack
    baseline_undo = _UndoState(stack)
    assert baseline_undo == (False, '', True, 'AI Assistant changes')
    old = _LocalReply(route, gated=True)
    fresh = _LocalReply('fresh', gated=True)
    with _ProviderPanel(api, container, {'old': old, 'fresh': fresh}) as ui:
        ui.send('old')
        old_worker, old_box = ui.thread, ui.toolbox
        _WaitForAttempt(ui, old)
        assert _DocumentState(a) == baseline_a, 'old edit ran before replacement'
        api.stage = b
        if replacement != 'silent':
            ui.set_session(session.Session(api, container))
            if replacement == 'reopen':
                api.stage = a
                ui.set_session(session.Session(api, container))
                assert old_box.stage is api.stage, 'A→B→A must use the same A object'
            assert not old_box.is_current(), 'old authority survived replacement'
            assert old_worker in ui.left_threads, 'old completion is still outstanding'
            ui.send('fresh')
            assert ui.thread is not None and ui.thread is not old_worker
            assert ui.send_button.text() == 'Stop', 'fresh Send is active'
            status = ui.status_label.text()
            assert status.endswith('is answering...')
        else:
            assert ui.thread is old_worker and not old_box.cancelled.is_set()
            assert not old_box.is_current(), 'live stage identity changed without notification'

        old.release.set()
        if replacement != 'silent' or route != 'native':
            # Let finished queue up without allowing owner delivery yet.
            joined = old_worker.wait(_QUEUE_BOUND * 1000)
            assert joined, 'old provider did not return'
        if replacement != 'silent':
            _PumpUntil(lambda: ui.reply == _FRESH_TEXT and not ui.left_threads,
                       'old completion did not drain while fresh reply was active')
            assert ui.busy and ui.status_label.text() == status
            assert [m['shown'] for m in ui.messages] == ['old', 'fresh']
            fresh.release.set()
        _ProviderFinished(ui)
        _Check(old.sent == ['old'] and
               (replacement == 'silent' or fresh.sent == ['fresh']),
               replacement + ': the canceled turn is not sent to the model again')
        if route == 'native':
            _Check(old.native_result is not None and old.native_result[1] and
                   old.native_result not in (_PENDING_EXPIRY, _RUNNING_EXPIRY),
                   replacement + ': stale native edit was refused, not timed out')
        elif route == 'fallback':
            _Check(old.passes == [('test-model', True), ('test-model', False)],
                   'delayed text was delivered by the tools-unsupported retry')
        if replacement != 'silent':
            assert [(m['role'], m.get('shown', m['content'])) for m in ui.messages] == [
                ('user', 'old'), ('user', 'fresh'), ('assistant', _FRESH_TEXT)]
            _Check(ui.chat_store.load(ui.chat_id)['messages'] == ui.messages and
                   ui.status_label.text() == '' and not ui.status_label.property('error') and
                   ui.send_button.text() == 'Send', 'fresh reply history and status survive')
        else:
            _Check([m['role'] for m in ui.messages] == ['user'],
                   'silent swap ignores the old assistant reply')
        _Check(_DocumentState(a) == baseline_a and _DocumentState(b) == baseline_b and
               _UndoState(stack) == baseline_undo,
               f'{replacement}/{route}: A and B layers/edit targets and undo are unchanged')
        stack.Redo()
        _Check(_Values(a) == (0, 0) and _Values(b) == (10, 0),
               'the preexisting redo still targets only B')


def TestProviderSwapNative():
    _TestProviderSwap('native', 'notify')


def TestProviderSwapFinalText():
    _TestProviderSwap('fallback', 'notify')


def TestProviderSilentSwapAndReopen():
    for replacement in ('silent', 'reopen'):
        for route in ('native', 'text'):
            _TestProviderSwap(route, replacement)


def TestProviderFinalTextPermissionsAndStop():
    for send_edit, finish_edit, stop in ((False, True, False),
                                         (True, False, False),
                                         (True, True, True)):
        stage = _Stage()
        baseline = _DocumentState(stage)
        api, container = _Api(stage), _Container()
        reply = _LocalReply('text', gated=True)
        with _ProviderPanel(api, container, {'edit': reply}, edit=send_edit) as ui:
            ui.send('edit')
            _WaitForAttempt(ui, reply)
            ui.edit_check.setChecked(finish_edit)
            if stop:
                ui.stop()
            reply.release.set()
            _ProviderFinished(ui)
            message = ui.messages[-1]
            _Check(message['role'] == 'assistant' and message['content'] == _ACTION_REPLY and
                   not message.get('done') and not message.get('failed'),
                   'disallowed final text stays in history without tool outcomes')
            if not stop:
                note = 'was off when you sent' if not send_edit else 'to allow them'
                _Check(note in message.get('note', ''), 'show why the edit was disallowed')
            _Check(_DocumentState(stage) == baseline and not container.stack.CanUndo(),
                   'permission changes or Stop leave final text inert')


def TestEditToggleRevokesAPendingNativeEdit():
    stage = _Stage()
    baseline = _DocumentState(stage)
    api, container = _Api(stage), _Container()
    reply = _LocalReply('native', gated=True)
    with _ProviderPanel(api, container, {'edit': reply}) as ui:
        ui.send('edit')
        _WaitForAttempt(ui, reply)
        ui.edit_check.setChecked(False)
        reply.release.set()
        _ProviderFinished(ui)
        message = ui.messages[-1]
        _Check(_DocumentState(stage) == baseline and not container.stack.CanUndo(),
               'turning Edit stage off revokes the change still in the queue')
        _Check(message['role'] == 'assistant' and
               message['content'] == 'Here is the rotation.' and
               any(entry['state'] == 'failed' for entry in message.get('tools', [])) and
               any('turned off' in problem for problem in message.get('failed', [])),
               'the reply says the change was not made: %s' % message)
    offline, _ = _Toolbox(stage, container, can_edit=True,
                          edit_allowed=lambda: False)
    text, failed = _SetRotation(offline, 40)
    _Check(failed and 'turned off' in text and _DocumentState(stage) == baseline,
           'the consent check sits in dispatch, which is where MCP calls go: %s' % text)


def TestReopenedChatExcludesTheCanceledTurn():
    a, b = _Stage(), _Stage()
    api, container = _Api(a), _Container()
    old = _LocalReply('native', gated=True)
    fresh = _LocalReply('plain', gated=True)
    with _ProviderPanel(api, container, {'old': old, 'fresh': fresh}) as ui:
        ui.send('old')
        _WaitForAttempt(ui, old)
        api.stage = b
        ui.set_session(session.Session(api, container))
        old.release.set()
        _ProviderFinished(ui)
        saved = ui.chat_store.load(ui.chat_id)
        _Check(any(m['role'] == 'user' and m['content'] == 'old' and m.get('canceled')
                   for m in saved['messages']),
               'the canceled turn is marked in the saved chat')
        ui.open_chat(ui.chat_id)
        _Check([m.get('shown', m['content']) for m in ui.messages] == ['old'],
               'the reopened chat shows the canceled request')
        ui.send('fresh')
        fresh.release.set()
        _ProviderFinished(ui)
        _Check(fresh.sent == ['fresh'],
               'a reopened chat sends only the new question to the model')
        _Check([m.get('shown', m['content']) for m in ui.messages] ==
               ['old', 'fresh', _PLAIN_TEXT],
               'the transcript still shows the canceled request')


def TestSendRollsBackWhenTheChatCannotBeSaved():
    stage = _Stage()
    api, container = _Api(stage), _Container()
    lost = _LocalReply('plain', gated=True)
    again = _LocalReply('plain', gated=False)
    with _ProviderPanel(api, container, {'hello': lost, 'again': again}) as ui:
        with patch.object(ui.chat_store, 'save',
                          side_effect=OSError('the disk is full')):
            ui.input.setPlainText('hello')
            ui.send()
        _Check(not ui.busy and ui.toolbox is None and ui.thread is None and
               not ui._pending_turn and not ui._settling and ui.messages == [] and
               ui.input.toPlainText() == 'hello' and
               'could not be saved' in ui.status_label.text() and
               bool(ui.status_label.property('error')) and
               ui.send_button.text() == 'Send',
               'a chat that cannot be saved leaves no reply in flight')
        ui.send('again')
        _ProviderFinished(ui)
        message = ui.messages[-1]
        _Check([m.get("shown", m["content"]) for m in ui.messages] ==
               ["again", _PLAIN_TEXT] and
               message['role'] == 'assistant' and message['content'] == _PLAIN_TEXT and
               not message.get('failed'),
               'the next Send works normally after a failed save')


def TestSendRollsBackWhenTheRequestCannotStart():
    stage = _Stage()
    api, container = _Api(stage), _Container()
    again = _LocalReply('plain', gated=False)
    with _ProviderPanel(api, container, {'start me': again}) as ui:
        attempts = []

        def failing_init(self, *args, **kwargs):
            attempts.append(1)
            raise RuntimeError('no thread left')

        with patch.object(panel.ChatThread, '__init__', failing_init):
            ui.input.setPlainText('start me')
            ui.send()
        _Check(len(attempts) == 1 and not ui.busy and ui.thread is None and
               not ui._pending_turn and not ui._settling and
               ui.messages == [] and
               ui.input.toPlainText() == 'start me' and
               'could not start' in ui.status_label.text() and
               bool(ui.status_label.property('error')) and
               ui.send_button.text() == 'Send',
               'a request that cannot start leaves nothing in flight')
        saved = ui.chat_store.load(ui.chat_id)
        _Check(saved is None or
               all(message['content'] != 'start me'
                   for message in saved['messages']),
               'the rolled back turn is not kept in the saved chat')
        ui.send('start me')
        _ProviderFinished(ui)
        _Check(again.sent == ['start me'],
               'the first retry starts exactly one request')


def TestSendRollsBackWhenTheWorkerCannotStart():
    stage = _Stage()
    api, container = _Api(stage), _Container()
    again = _LocalReply('plain', gated=False)
    with _ProviderPanel(api, container, {'start me': again}) as ui:
        attempts = []

        def failing_start(self):
            attempts.append(1)
            raise RuntimeError('no threads left')

        with patch.object(panel.ChatThread, 'start', failing_start):
            ui.input.setPlainText('start me')
            ui.send()
        _Check(len(attempts) == 1 and not ui.busy and ui.thread is None and
               not ui._pending_turn and not ui._settling and ui.messages == [] and
               ui.input.toPlainText() == 'start me' and
               'could not start' in ui.status_label.text() and
               bool(ui.status_label.property('error')) and
               ui.send_button.text() == 'Send',
               'a worker that cannot start leaves no reply in flight')
        QtWidgets.QApplication.processEvents()
        _Check(not ui.findChildren(panel.ChatThread),
               'the worker that never started is not kept')
        saved = ui.chat_store.load(ui.chat_id)
        _Check(saved is None or
               all(message['content'] != 'start me'
                   for message in saved['messages']),
               'the turn that never reached the model is not in the chat')
        ui.send('start me')
        _ProviderFinished(ui)
        _Check(again.sent == ['start me'],
               'the first retry starts exactly one request')


def TestSendRollsBackAfterAnotherSendTookThePanel():
    """A chat that cannot be saved, for a Send another Send has already
    taken the place of, gives back only its own question: the reply now in
    flight keeps what it holds."""
    stage = _Stage()
    api, container = _Api(stage), _Container()
    lost = _LocalReply('plain', gated=True)
    instead = _LocalReply('plain', gated=True)
    with _ProviderPanel(api, container,
                        {'hello': lost, 'instead': instead}) as ui:
        store = ui.chat_store
        write = store.save
        moved = []

        def save_that_moves_the_panel(record):
            if not moved:
                # The artist stops this Send and sends another question
                # while the first Send is still writing its chat.
                moved.append(1)
                ui.stop()
                ui.send('instead')
                raise OSError('the disk is full')
            write(record)

        with patch.object(store, 'save', save_that_moves_the_panel):
            ui.input.setPlainText('hello')
            ui.send()
        _Check(ui.busy and ui.toolbox is not None and ui.thread is not None and
               ui._pending_turn is None and not ui.left_threads and
               ui.findChildren(panel.ChatThread) == [ui.thread] and
               [m.get('shown', m['content']) for m in ui.messages] ==
               ['instead'] and ui.send_button.text() == 'Stop',
               'the Send that took the panel is left alone')
        _Check('could not be saved' in ui.status_label.text() and
               bool(ui.status_label.property('error')) and
               ui.input.toPlainText() == 'hello',
               'the failed Send says so and keeps its question to send again')
        saved = ui.chat_store.load(ui.chat_id)
        _Check(saved is not None and
               [m['content'] for m in saved['messages']] == ['instead'],
               'reloaded storage contains only the replacement question')
        instead.release.set()
        _ProviderFinished(ui)
        _Check(lost.sent is None and instead.sent == ['instead'],
               'the rolled back question never reached the model, and the '
               'other Send asked once')
        saved = ui.chat_store.load(ui.chat_id)
        _Check(saved is not None and
               all(message['content'] != 'hello'
                   for message in saved['messages']),
               'the rolled back turn is not kept in the saved chat')
        _Check([m.get('shown', m['content']) for m in ui.messages] ==
               ['instead', _PLAIN_TEXT],
               'the reply in flight is recorded once')
        ui.send('hello')
        lost.release.set()
        _ProviderFinished(ui)
        _Check(len(lost.passes) == 1 and lost.sent[-1] == 'hello',
               'the failed question reaches the provider exactly once on retry')


def TestMcpEditLosesConsentBeforeDispatch():
    """An MCP tools/call queued while Edit stage is on is refused, and
    authors nothing, when consent goes off before the owner thread runs it."""
    stage = _Stage()
    baseline = _DocumentState(stage)
    api, container = _Api(stage), _Container()
    held = _LocalReply('plain', gated=True)
    server = mcp_server.McpServer()
    try:
        with _ProviderPanel(api, container, {'hold': held}) as ui:
            ui.send('hold')
            _PumpUntil(lambda: held.sent is not None,
                       'the reply never started')
            box = ui.toolbox
            token = server.open(box)
            answers = {}

            def post():
                call = {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                        'params': {'name': 'change_stage', 'arguments': {
                            'changes': [{'action': 'set_attr', 'path': _CTL,
                                         'name': 'avars:rx',
                                         'value': 30}]}}}
                request = urllib.request.Request(
                    server.url,
                    data=json.dumps(call).encode('utf-8'),
                    headers={'Content-Type': 'application/json',
                             'Authorization': 'Bearer ' + token})
                try:
                    with urllib.request.urlopen(request, timeout=10) as answer:
                        raw = answer.read()
                    answers['response'] = json.loads(raw or b'null')
                except urllib.error.HTTPError as err:
                    answers['status'] = err.code

            worker = threading.Thread(target=post, daemon=True)
            worker.start()
            deadline = time.monotonic() + _QUEUE_BOUND
            while box.outstanding_calls() == 0 and time.monotonic() < deadline:
                time.sleep(0.01)
            _Check(box.outstanding_calls() == 1,
                   'the MCP edit waits for the owner thread')
            ui.edit_check.setChecked(False)
            _PumpUntil(lambda: 'response' in answers,
                       'the MCP edit never answered')
            worker.join(_QUEUE_BOUND)
            result = (answers.get('response') or {}).get('result') or {}
            texts = [item.get('text', '')
                     for item in result.get('content') or []]
            _Check(result.get('isError') and
                   any('turned off' in text for text in texts),
                   'the queued MCP edit is refused: %s' % answers)
            _Check(_DocumentState(stage) == baseline and
                   not container.stack.CanUndo(),
                   'the refused MCP edit authored nothing and recorded '
                   'nothing')
            held.release.set()
            _ProviderFinished(ui)
            _Check(_Values(stage) == (0.0, 0.0) and
                   not container.stack.CanUndo(),
                   'the reply closes with the stage untouched')
    finally:
        server.stop()


def TestReplyWaitsForItsQueuedCall():
    stage = _Stage()
    api, container = _Api(stage), _Container()
    reply = _LocalReply('native', gated=True)
    with patch.object(tools, 'CALL_TIMEOUT', 0.5), \
            _ProviderPanel(api, container, {'edit': reply}) as ui:
        ui.send('edit')
        _WaitForAttempt(ui, reply)
        box = ui.toolbox
        tool = box.tools['change_stage']
        author = tool.handler

        def slow_handler(args):
            result = author(args)
            time.sleep(1.2)  # past the caller's deadline, so the model moves on
            QtWidgets.QApplication.processEvents()
            waiting.append('Writing the changes' in ui.status_label.text()
                           and ui._settling is box and ui.busy)
            return result

        tool.handler = slow_handler
        waiting = []
        reply.release.set()
        _PumpUntil(lambda: reply.native_result is not None,
                   'the model never moved on without its change')
        _Check(waiting == [True] and reply.native_result == _RUNNING_EXPIRY,
               'the reply waited while its own change was still being authored')
        _ProviderFinished(ui)
        answers = [m for m in ui.messages if m['role'] == 'assistant']
        _Check(len(answers) == 1 and answers[0]['content'] == 'Here is the rotation.' and
               answers[0].get('done') == ['Set ' + _CTL + '.avars:rx to 30'],
               'the reply is recorded once, with the outcome of its queued call')
        _Check([entry['state'] for entry in answers[0].get('tools', [])] == ['done'],
               'the saved activity does not stay running: %s' % answers[0])
        _Check(_Values(stage) == (30.0, 0.0) and
               container.stack.UndoText() == 'AI Assistant changes',
               'the queued change was authored as one step')
        container.stack.Undo()
        _Check(_Values(stage) == (0, 0) and not container.stack.CanUndo(),
               'one undo takes back the reply that settled late')
        _Check(ui._settling is None and not ui.busy and
               ui.send_button.text() == 'Send', 'the settled reply leaves the panel idle')


def main():
    app = QtWidgets.QApplication.instance()
    for test in (TestToolboxNoStageWithoutApp, TestLooking,
                 TestChangesAreOneUndoStep, TestToolboxRejectsSilentStageReplacement,
                 TestToolboxGenerationAndRetirement,
                 TestArtistCommitStartsANewStep,
                 TestEditsUndoInTheOrderTheyWereMade,
                 TestACommitFromAStageNoticeKeepsTheAuthoredOrder,
                 TestAPoseCommitFromAStageNoticeKeepsTheAuthoredOrder,
                 TestAResetCommitFromAStageNoticeKeepsTheAuthoredOrder,
                 TestAClearCommitFromAStageNoticeKeepsTheAuthoredOrder,
                 TestACallThatChangesNothingLeavesHistoryAlone,
                 TestAWriteAfterAnUndoDoesNotGrowAnOlderStep,
                 TestHostUndoStackWithoutLastEdit,
                 TestHostUndoStackWithoutWithdraw,
                 TestANoChangeAtTheHistoryLimitKeepsTheOldestStep,
                 TestStageReplacementMidBatchStopsTheRest,
                 TestSharedLayerReplacementMidBatchStopsTheRest,
                 TestToolboxSwapDuringActivity,
                 TestToolboxFolderGrantAfterPrompt, TestPosing, TestRunPython,
                 TestActionBlocks, TestQueuedEditExpiresBeforeDelivery,
                 TestQueuedEditExpiresBeforeBusyRetry, TestCancelWakesQueuedEdit,
                 TestRunningEditCanCompleteAfterCallerDeadline, TestMcpServer,
                 TestProviderCompletion, TestProviderSwapNative,
                 TestProviderSwapFinalText, TestProviderSilentSwapAndReopen,
                 TestProviderFinalTextPermissionsAndStop,
                 TestEditToggleRevokesAPendingNativeEdit,
                 TestReopenedChatExcludesTheCanceledTurn,
                 TestReplyWaitsForItsQueuedCall,
                 TestSendRollsBackWhenTheChatCannotBeSaved,
                 TestSendRollsBackWhenTheRequestCannotStart,
                 TestSendRollsBackWhenTheWorkerCannotStart,
                 TestSendRollsBackAfterAnotherSendTookThePanel,
                 TestMcpEditLosesConsentBeforeDispatch):
        if app is None and test is not TestToolboxNoStageWithoutApp:
            app = QtWidgets.QApplication([])
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
