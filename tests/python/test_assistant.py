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

from pxr import Sdf, Usd  # noqa: E402
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
    _Check(not failed, "B can author an edit: %s" % text)
    fresh.finish()
    container.stack.Undo()

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
    box.finish()
    _Check(a.GetRootLayer().ExportToString() == a_before and
           b.GetRootLayer().ExportToString() == b_before,
           "rejected edit leaves both authored layers unchanged")
    _Check(not container.stack.CanUndo() and container.stack.CanRedo(),
           "stale finish does not push or clear B's redo")
    api.stage = a
    box.finish()
    _Check(not container.stack.CanUndo() and container.stack.CanRedo(),
           "stale finish drained the old edits without rolling them back")
    container.stack.Redo()
    _Check(b.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 10 and
           a.GetAttributeAtPath("/Char/Ctl.avars:rx").Get() == 20,
           "B's existing redo still affects only B")


def TestToolboxGenerationAndRetirement():
    stage = _Stage()
    generation = 0
    container = _Container()
    box, api = _Toolbox(stage, container, can_edit=True,
                        document_current=lambda: generation == 0)
    box.finish()
    text, failed = _SetRotation(box, 10)
    _Check(not failed and box.is_current(), "finish allows later calls: %s" % text)
    box.finish()
    box.finish()
    container.stack.Undo()
    rx = stage.GetAttributeAtPath("/Char/Ctl.avars:rx")
    _Check(rx.Get() == 0 and not container.stack.CanUndo(),
           "finish pushes each edit only once")
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
    live.finish()
    container.stack.Undo()
    _Check(rx.Get() == 0, "cancel still lets finish undo a current edit")


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

            # Observe actual submission, not merely arrival at stream_chat.
            with patch.object(tools, '_invoke', ObservedSignal()):
                reply.native_result = tools.call(
                    'change_stage', {'changes': _NATIVE_ACTIONS})
            reply.wait_for_release()
        if reply.route == 'fresh':
            yield _FRESH_TEXT
        elif reply.route == 'native':
            yield 'Here is the rotation.'
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
    seed.finish()
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


def main():
    app = QtWidgets.QApplication.instance()
    for test in (TestToolboxNoStageWithoutApp, TestLooking,
                 TestChangesAreOneUndoStep, TestToolboxRejectsSilentStageReplacement,
                 TestToolboxGenerationAndRetirement, TestToolboxSwapDuringActivity,
                 TestToolboxFolderGrantAfterPrompt, TestPosing, TestRunPython,
                 TestActionBlocks, TestQueuedEditExpiresBeforeDelivery,
                 TestQueuedEditExpiresBeforeBusyRetry, TestCancelWakesQueuedEdit,
                 TestRunningEditCanCompleteAfterCallerDeadline, TestMcpServer,
                 TestProviderCompletion, TestProviderSwapNative,
                 TestProviderSwapFinalText, TestProviderSilentSwapAndReopen,
                 TestProviderFinalTextPermissionsAndStop):
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
