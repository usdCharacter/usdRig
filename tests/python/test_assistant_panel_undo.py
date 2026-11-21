#!/usr/bin/env python3
"""Qt-free tests for the AI Assistant panel's reply transaction.

Every reply is one transaction: native tool calls and the fenced action
blocks of the same reply share one Toolbox, and closing it pushes exactly
one undo step. A reply that was stopped, or whose fences are not allowed,
finalizes only the work already executed. What the reply could not do, and
what no undo takes back, are reported, not swallowed.

Run: bin/test/run_python_tests.sh test_assistant_panel_undo
Requires: OpenUSD 26.08 with Usdviewq available.
Prints ASSISTANT_UNDO_OK on success.
"""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["USDRIG_USER_DIR"] = tempfile.mkdtemp(prefix="usdrig_assistant_")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rigexec_test_env  # noqa: E402

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtWidgets  # noqa: E402

import rigExecUndo  # noqa: E402
from usdRigAssistant import chat_render, panel  # noqa: E402
from usdRigAssistant import session as assistant_session  # noqa: E402
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


class _ThrowingStack(rigExecUndo.UndoStack):
    def Push(self, edit):
        raise RuntimeError("the host stack refused the edit")


_CAPTURE_USDA = """#usda 1.0
(
    timeCodesPerSecond = 30
)
def Xform "Char"
{
    def Xform "Ctl"
    {
        custom double x = 1
    }
    def Scope "Geo"
    {
    }
}
"""


def _Layer(name, usda=""):
    layer = Sdf.Layer.CreateAnonymous(name)
    if usda:
        layer.ImportFromString(usda)
    return layer


def _Stage():
    return Usd.Stage.Open(_Layer("assistant_undo.usda", _CAPTURE_USDA))


def _Panel(stage, container):
    """A real panel on a fake usdview host, with no chat store on disk."""
    assistant = panel.AiAssistant(prefs={})
    assistant.set_session(assistant_session.Session(_Api(stage), container))
    return assistant


def _ReplyToolbox(assistant):
    """The one Toolbox a reply runs on, as send() builds it."""
    return tools.Toolbox(assistant.session, can_edit=assistant.can_edit,
                         can_run=assistant.can_run, parent=assistant,
                         memory=assistant.memory,
                         chat_folders=assistant.chat_folders,
                         ask_access=assistant.ask_access)


def _Reply(assistant, reply, toolbox):
    """Set up a reply that has come back, ready to be finished."""
    assistant.toolbox = toolbox
    assistant.reply = reply
    assistant.reply_time = 1.0
    assistant.reply_tools = []
    return toolbox


def _Fence(actions):
    return "```" + stage_actions.FENCE + "\n" + actions + "```\n"


def _Sentinel(stage, container, layer):
    """An older step on the host stack the reply has nothing to do with."""
    path = Sdf.Path("/Char/Ctl")
    before = rigExecUndo.SpecSnapshot.Capture(layer, path)
    stage.GetPrimAtPath(path).CreateAttribute(
        "sentinel", Sdf.ValueTypeNames.Double).Set(1.0)
    after = rigExecUndo.SpecSnapshot.Capture(layer, path)
    container.stack.Push(rigExecUndo.Edit(
        "Sentinel change",
        [rigExecUndo.EditEntry(layer, path, before, after)]))


def TestMixedReplyIsOneUndoStep():
    """Native tool edits and the reply's fences are one step, and one undo
    takes all of them back without touching the older step."""
    root = _Layer("mixed_root.usda", _CAPTURE_USDA)
    over = _Layer("mixed_over.usda")
    root.subLayerPaths.append(over.identifier)
    stage = Usd.Stage.Open(root)
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)

    _Sentinel(stage, container, root)

    reply = ("Here is the pose, and the change on the other layer.\n" +
             _Fence('[{"action": "set_edit_target", "layer": "' +
                    over.identifier + '"},\n'
                    ' {"action": "set_attr", "path": "/Char/Ctl", '
                    '"name": "y", "type": "double", "value": 7.0}]\n'))
    _Reply(assistant, reply, toolbox)
    # A native tool call, then the reply's fences: the same reply, so the
    # same Toolbox, so one step.
    text, failed = toolbox.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 2.0}]})
    _Check(not failed, "the native tool edit is made: %s" % text)
    assistant.on_finished()

    _Check(assistant.toolbox is None, "the reply left its toolbox open")
    pushed = container.stack.pushed
    _Check(len(pushed) == 2,
           "the reply pushed once, after the sentinel: %s" % (pushed,))
    step = pushed[1] if len(pushed) == 2 else None
    _Check(isinstance(step, assistant_session.GroupEdit),
           "the two edits are one group: %s" % (step,))
    _Check(getattr(step, "label", "") == stage_actions.UNDO_LABEL,
           "the step carries the assistant's label: %s" %
           getattr(step, "label", ""))
    _Check(len(getattr(step, "edits", [])) == 2,
           "both layers are in the one step: %s" %
           getattr(step, "edits", None))

    ctl = stage.GetPrimAtPath("/Char/Ctl")
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 2.0 and
           over.GetAttributeAtPath("/Char/Ctl.y").default == 7.0,
           "each write went to the layer that was the edit target")
    _Check(ctl.GetAttribute("x").Get() == 2.0 and
           ctl.GetAttribute("y").Get() == 7.0,
           "both changes read back composed")

    # One undo: every layer the reply touched is back, and the step that
    # was there before the reply is still reachable, not consumed.
    container.stack.Undo()
    _Check(root.GetAttributeAtPath("/Char/Ctl.x").default == 1.0 and
           over.GetAttributeAtPath("/Char/Ctl.y") is None,
           "one undo puts both layers back")
    _Check(ctl.GetAttribute("x").Get() == 1.0 and
           ctl.GetAttribute("y").Get() is None,
           "one undo takes both composed values back")
    _Check(container.stack.CanUndo(), "the sentinel was consumed")
    _Check(container.stack.UndoText() == "Sentinel change",
           "the sentinel is exposed: %s" % container.stack.UndoText())
    _Check(ctl.GetAttribute("sentinel").Get() == 1.0,
           "the sentinel's own change is untouched")

    # One redo brings both changes back.
    container.stack.Redo()
    _Check(ctl.GetAttribute("x").Get() == 2.0 and
           ctl.GetAttribute("y").Get() == 7.0,
           "one redo makes both changes again")

    message = assistant.messages[-1]
    _Check(len(message["done"]) == 2,
           "the reply reports both changes: %s" % (message.get("done"),))
    _Check(message["failed"] == [],
           "nothing failed: %s" % (message.get("failed"),))
    _Check(any(e.startswith("Edits now go to")
               for e in message["effects"]),
           "the edit target switch is an effect: %s" %
           (message.get("effects"),))

    # Finishing again changes nothing: the reply is already closed.
    assistant.end_tools()
    _Check(len(container.stack.pushed) == 2,
           "a second finish pushed again: %s" % (container.stack.pushed,))


def TestStoppedReplyKeepsOnlyExecutedWork():
    """A reply stopped mid-flight never runs its fences; what its tools
    already did is the undo step."""
    stage = _Stage()
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant,
           "I was going to add a joint.\n" +
           _Fence('[{"action": "define_prim", "path": "/Char/Ctl/Extra"}]\n'),
           toolbox)
    text, failed = toolbox.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 3.0}]})
    _Check(not failed, "the tool edit before the stop is made: %s" % text)
    toolbox.cancel()
    assistant.on_finished()

    ctl = stage.GetPrimAtPath("/Char/Ctl")
    _Check(not stage.GetPrimAtPath("/Char/Ctl/Extra").IsValid(),
           "a stopped reply ran its fences")
    _Check(len(container.stack.pushed) == 1,
           "the executed work is one step: %s" % (container.stack.pushed,))
    _Check(container.stack.UndoText() == stage_actions.UNDO_LABEL,
           "the step carries the assistant's label")
    container.stack.Undo()
    _Check(ctl.GetAttribute("x").Get() == 1.0,
           "undo takes the executed work back")
    container.stack.Redo()
    _Check(ctl.GetAttribute("x").Get() == 3.0,
           "redo makes the executed work again")
    message = assistant.messages[-1]
    _Check(len(message["done"]) == 1,
           "the reply reports its executed work: %s" % (message.get("done"),))
    _Check(message.get("failed", []) == [],
           "nothing failed: %s" % (message.get("failed"),))


def TestUnreadableFenceIsReportedNotApplied():
    """A fence the reply garbled is a problem, not a half-applied set of
    changes."""
    stage = _Stage()
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant,
           "Adding the joints:\n```" + stage_actions.FENCE + "\n"
           '[{"action": "define_prim", "path": "/Char/Ctl/Extra"\n```\n',
           toolbox)
    assistant.on_finished()

    _Check(not stage.GetPrimAtPath("/Char/Ctl/Extra").IsValid(),
           "an unreadable fence was applied")
    message = assistant.messages[-1]
    _Check(bool(message.get("failed")),
           "the garbled fence is reported: %s" % (message,))
    _Check(message.get("done", []) == [],
           "nothing claims to be done: %s" % (message.get("done"),))
    _Check(container.stack.pushed == [] and not container.stack.CanUndo(),
           "a garbled fence leaves the undo history untouched")


def TestRejectedFenceIsReportedNotApplied():
    """With "Edit stage" off, a reply's fences are not run and the reply
    says so; its already-executed work is still one step."""
    stage = _Stage()
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant,
           "I can move the arm if you allow it.\n" +
           _Fence('[{"action": "define_prim", "path": "/Char/Ctl/Extra"}]\n'),
           toolbox)
    text, failed = toolbox.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 4.0}]})
    _Check(not failed, "the tool edit before the toggle is made: %s" % text)
    # The user turns Edit stage off while the reply is coming.
    assistant.edit_check.setChecked(False)
    assistant.on_finished()

    _Check(not stage.GetPrimAtPath("/Char/Ctl/Extra").IsValid(),
           "a rejected fence was applied")
    _Check(len(container.stack.pushed) == 1,
           "the executed work is still one step: %s" %
           (container.stack.pushed,))
    container.stack.Undo()
    _Check(stage.GetPrimAtPath("/Char/Ctl").GetAttribute("x").Get() == 1.0,
           "undo takes the executed work back")
    message = assistant.messages[-1]
    _Check("Edit stage" in message.get("note", ""),
           "the reply says its fences were not allowed: %s" % (message,))
    _Check(len(message["done"]) == 1,
           "the reply reports its executed work: %s" % (message.get("done"),))


def TestMissingUndoStackIsVisible():
    """A host with no undo stack reports it instead of silently claiming
    the reply is undoable."""
    stage = _Stage()
    container = _StacklessContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant,
           "Posed the arm.\n" +
           _Fence('[{"action": "set_attr", "path": "/Char/Ctl", '
                  '"name": "x", "type": "double", "value": 5.0}]\n'),
           toolbox)
    assistant.on_finished()

    ctl = stage.GetPrimAtPath("/Char/Ctl")
    _Check(ctl.GetAttribute("x").Get() == 5.0,
           "the change is still made without a stack")
    message = assistant.messages[-1]
    _Check(bool(message.get("failed")),
           "the missing stack is reported: %s" % (message,))
    _Check(any("undo" in problem.lower()
               for problem in message["failed"]),
           "the report names undo: %s" % (message.get("failed"),))
    _Check(bool(assistant.status_label.property("error")),
           "no visible failure")
    _Check(assistant.session.undo_stack() is None,
           "the session still has no stack")


def TestRefusedPushIsVisible():
    """A host stack that refuses the push is a failure, not a silent pass."""
    stage = _Stage()
    container = _CountingContainer()
    container.stack = _ThrowingStack()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant, "Posed the arm.\n", toolbox)
    text, failed = toolbox.call("change_stage", {"changes": [
        {"action": "set_attr", "path": "/Char/Ctl", "name": "x",
         "value": 6.0}]})
    _Check(not failed, "the tool edit is made: %s" % text)
    assistant.on_finished()

    ctl = stage.GetPrimAtPath("/Char/Ctl")
    _Check(ctl.GetAttribute("x").Get() == 6.0,
           "the change stands after the refused push")
    message = assistant.messages[-1]
    _Check(bool(message.get("failed")),
           "the refused push is reported: %s" % (message,))
    _Check(any("undo" in problem.lower()
               for problem in message["failed"]),
           "the report names undo: %s" % (message.get("failed"),))
    _Check(bool(assistant.status_label.property("error")),
           "no visible failure")
    _Check(not container.stack.CanUndo(),
           "the reply claims an undo step")


def TestSaveOnlyReplyHasEffectsAndNoStep():
    """A reply that only saves has nothing to undo and says what it did."""
    folder = tempfile.mkdtemp(prefix="usdrig_reply_")
    path = os.path.join(folder, "saved.usda")
    with open(path, "w") as handle:
        handle.write(_CAPTURE_USDA)
    stage = Usd.Stage.Open(path)
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    toolbox = _ReplyToolbox(assistant)
    _Reply(assistant, "Saved the stage for you.\n", toolbox)
    stage.GetAttributeAtPath("/Char/Ctl.x").Set(9.0)
    text, failed = toolbox.call("save_layers", {"all": True})
    _Check(not failed and "Saved" in text,
           "save_layers saves the dirty layer: %s" % text)
    assistant.on_finished()

    message = assistant.messages[-1]
    _Check(bool(message.get("effects")),
           "the save is reported: %s" % (message,))
    _Check(any(e.startswith("Saved ") for e in message["effects"]),
           "the save is an effect: %s" % (message.get("effects"),))
    _Check(message.get("done", []) == [],
           "nothing claims to be done: %s" % (message.get("done"),))
    _Check(container.stack.pushed == [] and not container.stack.CanUndo(),
           "a save-only reply leaves the undo history untouched")
    with open(path) as handle:
        _Check("x = 9" in handle.read(),
               "the save wrote through to the file")


def TestChangeBlocksAreTitledApart():
    """Changed, not undoable, and could not are three separate blocks."""
    html = chat_render.changes_html({
        "role": "assistant", "content": "Done.",
        "done": ["Posed /Char/Ctl"],
        "effects": ["Saved the stage"],
        "failed": ["Could not delete /Char/Old"]})
    _Check(">Changed<" in html, "no Changed block: %s" % html)
    _Check(">Not undoable<" in html, "no Not undoable block: %s" % html)
    _Check(">Could not<" in html, "no Could not block: %s" % html)
    _Check(html.index(">Changed<") < html.index(">Not undoable<") <
           html.index(">Could not<"),
           "the blocks are out of order: %s" % html)
    _Check("Posed /Char/Ctl" in
           html[html.index(">Changed<"):html.index(">Not undoable<")],
           "the change is under the wrong block: %s" % html)
    _Check("Saved the stage" in
           html[html.index(">Not undoable<"):html.index(">Could not<")],
           "the effect is under the wrong block: %s" % html)

    # A chat saved before effects existed still renders.
    old = chat_render.changes_html({"role": "assistant", "content": "Done.",
                                    "done": ["Posed /Char/Ctl"],
                                    "failed": ["Could not find /Char/Ctl"]})
    _Check(">Changed<" in old and ">Could not<" in old,
           "an old message lost a block: %s" % old)
    _Check("Not undoable" not in old,
           "an old message grew an effects block: %s" % old)


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    _ = app
    for test in (TestMixedReplyIsOneUndoStep,
                 TestStoppedReplyKeepsOnlyExecutedWork,
                 TestUnreadableFenceIsReportedNotApplied,
                 TestRejectedFenceIsReportedNotApplied,
                 TestMissingUndoStackIsVisible,
                 TestRefusedPushIsVisible,
                 TestSaveOnlyReplyHasEffectsAndNoStep,
                 TestChangeBlocksAreTitledApart):
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
    print("ASSISTANT_UNDO_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
