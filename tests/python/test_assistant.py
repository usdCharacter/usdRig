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
from usdRigAssistant import mcp_server, session, stage_actions  # noqa: E402
from usdRigAssistant import tools  # noqa: E402

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


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _ = app
    for test in (TestLooking, TestChangesAreOneUndoStep, TestPosing,
                 TestRunPython, TestActionBlocks, TestMcpServer):
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
