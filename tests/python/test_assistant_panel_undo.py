#!/usr/bin/env python3
"""Offscreen Qt tests for the AI Assistant panel's reply transaction.

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

from pxr import Sdf, Tf, Usd  # noqa: E402
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
    stage, generation = assistant.stage, assistant._document_generation
    return tools.Toolbox(assistant.session, can_edit=assistant.can_edit,
                         can_run=assistant.can_run, parent=assistant,
                         memory=assistant.memory,
                         chat_folders=assistant.chat_folders,
                         ask_access=assistant.ask_access,
                         document_current=lambda: (
                             assistant._document_generation == generation
                             and assistant.stage is stage))


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


def TestStageReplacementRejectsLateWrites():
    a, b = _Stage(), _Stage()
    b.GetAttributeAtPath('/Char/Ctl.x').Set(10.0)
    container = _CountingContainer()
    assistant = _Panel(a, container)
    box = _ReplyToolbox(assistant)
    _Reply(assistant, _Fence('[{"action":"set_attr","path":"/Char/Ctl",'
                              '"name":"x","value":30}]'), box)
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x', 'value': 2.0}]})
    _Check(not failed, 'A did not get the first edit: %s' % text)
    assistant.session.api.stage = b
    for name, args in (('change_stage', {'changes': [
            {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
             'value': 20.0}]}),
                       ('run_python', {'code': 'stage.GetAttributeAtPath('
                                     '"/Char/Ctl.x").Set(20)' })):
        text, failed = box.call(name, args)
        _Check(failed and 'stage changed' in text,
               'late %s was accepted: %s' % (name, text))
    assistant.on_finished()
    _Check(b.GetAttributeAtPath('/Char/Ctl.x').Get() == 10.0,
           'B changed from A reply')
    _Check(len(container.stack.pushed) == 1, 'A did not get one undo entry')
    container.stack.Undo()
    _Check(a.GetAttributeAtPath('/Char/Ctl.x').Get() == 1.0 and
           b.GetAttributeAtPath('/Char/Ctl.x').Get() == 10.0,
           'A undo modified B or missed A')


def TestGroupFailsAtomically():
    root = _Layer('group_root.usda', _CAPTURE_USDA)
    over = _Layer('group_over.usda')
    root.subLayerPaths.append(over.identifier)
    stage = Usd.Stage.Open(root)
    container = _CountingContainer()
    box = tools.Toolbox(assistant_session.Session(_Api(stage), container),
                        can_edit=True)
    box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x', 'value': 2.0},
        {'action': 'set_edit_target', 'layer': over.identifier},
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'y',
         'type': 'double', 'value': 7.0}]})
    box.close()
    _Check(len(container.stack.pushed) == 1, 'expected one compound step')
    root.SetPermissionToEdit(False)
    try:
        try:
            container.stack.Undo()
            _Check(False, 'undo accepted a locked later member')
        except RuntimeError:
            pass
        _Check(root.GetAttributeAtPath('/Char/Ctl.x').default == 2.0 and
               over.GetAttributeAtPath('/Char/Ctl.y').default == 7.0 and
               container.stack.CanUndo() and not container.stack.CanRedo(),
               'failed undo changed layers or cursor')
    finally:
        root.SetPermissionToEdit(True)
    container.stack.Undo()
    over.SetPermissionToEdit(False)
    try:
        try:
            container.stack.Redo()
            _Check(False, 'redo accepted a locked later member')
        except RuntimeError:
            pass
        _Check(root.GetAttributeAtPath('/Char/Ctl.x').default == 1.0 and
               over.GetAttributeAtPath('/Char/Ctl.y') is None and
               container.stack.CanRedo() and not container.stack.CanUndo(),
               'failed redo changed layers or cursor')
    finally:
        over.SetPermissionToEdit(True)
    container.stack.Redo()


def TestSharedLayerPythonLeavesIndependentField():
    shared = _Layer('shared.usda', _CAPTURE_USDA.replace('custom double x = 1',
        'custom double x = 1\n        custom double y = 10'))
    a, b = Usd.Stage.Open(shared), Usd.Stage.Open(shared)
    container = _CountingContainer()
    box = tools.Toolbox(assistant_session.Session(_Api(a), container),
                        can_run=True)
    text, failed = box.call('run_python', {'code':
        'stage.GetAttributeAtPath("/Char/Ctl.x").Set(2.0)'})
    _Check(not failed, 'Python write failed: %s' % text)
    box.close()
    b.GetAttributeAtPath('/Char/Ctl.y').Set(20.0)
    container.stack.Undo()
    _Check(shared.GetAttributeAtPath('/Char/Ctl.x').default == 1.0 and
           shared.GetAttributeAtPath('/Char/Ctl.y').default == 20.0,
           'undo removed independent y')
    container.stack.Redo()
    _Check(shared.GetAttributeAtPath('/Char/Ctl.x').default == 2.0 and
           shared.GetAttributeAtPath('/Char/Ctl.y').default == 20.0,
           'redo removed independent y')


def TestInterleavedEditorHistory():
    stage = _Stage()
    root = stage.GetRootLayer()
    container = _CountingContainer()
    box = tools.Toolbox(assistant_session.Session(_Api(stage), container),
                        can_edit=True)
    box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
         'value': 2.0}]})
    path = Sdf.Path('/Char/Ctl.x')
    before = rigExecUndo.SpecSnapshot.Capture(root, path)
    stage.GetAttributeAtPath(path).Set(3.0)
    after = rigExecUndo.SpecSnapshot.Capture(root, path)
    container.stack.Push(rigExecUndo.Edit('Editor', [
        rigExecUndo.EditEntry(root, path, before, after)]))
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
         'value': 4.0}]})
    _Check(failed and len(container.stack.pushed) == 2,
           'interleaved editor write did not close reply: %s' % text)
    container.stack.Undo()
    _Check(stage.GetAttributeAtPath(path).Get() == 2.0,
           'undo editor did not restore Assistant value')
    container.stack.Undo()
    _Check(stage.GetAttributeAtPath(path).Get() == 1.0,
           'undo Assistant did not restore original value')


def TestReferencedMoves():
    for action in ('rename_prim', 'reparent_prim'):
        for local_over in (False, True):
            source = _Layer('reference.usda', '#usda 1.0\n'
                            'def Xform "Asset"\n{\n    def Xform "Part"\n'
                            '    {\n    }\n}\n')
            root = _Layer('reference_root.usda')
            stage = Usd.Stage.Open(root)
            world = stage.DefinePrim('/World', 'Xform')
            world.GetReferences().AddReference(source.identifier, '/Asset')
            stage.DefinePrim('/NewParent', 'Xform')
            link = stage.DefinePrim('/Link', 'Xform').CreateRelationship('target')
            link.SetTargets(['/World/Part'])
            if local_over:
                stage.OverridePrim('/World/Part').SetMetadata('kind', 'component')
            container = _CountingContainer()
            box = tools.Toolbox(assistant_session.Session(_Api(stage), container),
                                can_edit=True)
            destination = ('/World/Moved' if action == 'rename_prim' else
                           '/NewParent/Part')
            args = {'action': action, 'path': '/World/Part'}
            args['name' if action == 'rename_prim' else 'parent'] = (
                'Moved' if action == 'rename_prim' else '/NewParent')
            text, failed = box.call('change_stage', {'changes': [args]})
            _Check(not failed, '%s over=%s failed: %s' %
                   (action, local_over, text))
            box.close()
            if failed:
                continue

            def check(moved):
                expected = destination if moved else '/World/Part'
                _Check(stage.GetPrimAtPath(expected).IsValid() and
                       not stage.GetPrimAtPath(
                           '/World/Part' if moved else destination).IsValid(),
                       '%s over=%s composed path wrong (%s)' %
                       (action, local_over, moved))
                _Check([str(p) for p in link.GetTargets()] == [expected],
                       '%s over=%s relationship wrong (%s)' %
                       (action, local_over, moved))
                _Check(bool(root.relocates) == moved,
                       '%s over=%s relocate wrong (%s): %s' %
                       (action, local_over, moved, root.relocates))

            check(True)
            _Check(len(container.stack.pushed) == 1,
                   'referenced move missing undo entry')
            container.stack.Undo()
            check(False)
            container.stack.Redo()
            check(True)


def TestCloseDuringWrite():
    stage = _Stage()
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    box = _ReplyToolbox(assistant)
    _Reply(assistant, 'Set x.', box)
    closed = [False]

    def on_change(_notice, _sender):
        if not closed[0] and stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 2:
            closed[0] = True
            assistant.close()

    key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_change, stage)
    try:
        text, failed = box.call('change_stage', {'changes': [
            {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
             'value': 2.0}]})
    finally:
        key.Revoke()
    _Check(not failed and closed[0], 'close notice did not fire: %s' % text)
    _Check(stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 2.0 and
           len(container.stack.pushed) == 1 and
           assistant.messages and assistant.messages[-1]['done'],
           'close lost the change, undo step, or saved outcome')
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x', 'value': 3.0}]})
    _Check(failed and stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 2.0,
           'closed toolbox authored another change: %s' % text)
    container.stack.Undo()
    _Check(stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 1.0,
           'the close step is not usable')


def TestDocumentGenerationRejectsLateWrites():
    """A reply belongs to the document it was submitted against: opening
    another stage, then the same stage with another history, is a new
    document, and the reply keeps no authority over it."""
    a, b = _Stage(), _Stage()
    a.GetAttributeAtPath('/Char/Ctl.x').Set(10.0)
    started, other, reloaded = _CountingContainer(), _CountingContainer(), \
        _CountingContainer()
    assistant = _Panel(a, started)
    box = _ReplyToolbox(assistant)
    _Reply(assistant, _Fence('[{"action":"set_attr","path":"/Char/Ctl",'
                             '"name":"y","type":"double","value":7.0}]'), box)
    text, failed = box.call('change_stage', {'changes': [
        {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
         'value': 2.0}]})
    _Check(not failed, 'the reply authored on its own document: %s' % text)
    assistant.set_session(assistant_session.Session(_Api(b), other))
    assistant.set_session(assistant_session.Session(_Api(a), reloaded))
    for name, args in (('change_stage', {'changes': [
            {'action': 'set_attr', 'path': '/Char/Ctl', 'name': 'x',
             'value': 20.0}]}),
                       ('run_python', {'code': 'stage.GetAttributeAtPath('
                                     '"/Char/Ctl.x").Set(20)'})):
        text, failed = box.call(name, args)
        _Check(failed and 'stage changed' in text,
               'late %s was accepted on a new document: %s' % (name, text))
    assistant.on_finished()
    _Check(not a.GetAttributeAtPath('/Char/Ctl.y').IsValid(),
           'the fence authored on the reloaded document')
    _Check(a.GetAttributeAtPath('/Char/Ctl.x').Get() == 2.0,
           'the reply wrote after its document changed')
    _Check(len(started.stack.pushed) == 1,
           'the reply did not settle on the history it started on: %s' %
           (started.stack.pushed,))
    _Check(not other.stack.pushed and not reloaded.stack.pushed,
           'the reply pushed a step on another document\'s history')
    started.stack.Undo()
    _Check(a.GetAttributeAtPath('/Char/Ctl.x').Get() == 10.0 and
           b.GetAttributeAtPath('/Char/Ctl.x').Get() == 1.0,
           'the reply step is not undoable, or reached another stage')


def TestPythonFieldUndoKeepsForeignValue():
    """A Python edit records the field it wrote, not the whole spec, so an
    undo leaves a value another editor put on the same attribute alone."""
    shared = _Layer('shared_field.usda', _CAPTURE_USDA)
    a, b = Usd.Stage.Open(shared), Usd.Stage.Open(shared)
    container = _CountingContainer()
    box = tools.Toolbox(assistant_session.Session(_Api(a), container),
                        can_run=True)
    text, failed = box.call('run_python', {'code':
        'stage.GetRootLayer().GetAttributeAtPath("/Char/Ctl.x")'
        '.SetInfo("documentation", "posed by the assistant")'})
    _Check(not failed, 'the metadata write failed: %s' % text)
    box.close()
    _Check(len(container.stack.pushed) == 1,
           'a metadata-only Python edit made no undo step: %s' %
           (container.stack.pushed,))
    b.GetAttributeAtPath('/Char/Ctl.x').Set(9.0)
    container.stack.Undo()
    _Check(shared.GetAttributeAtPath('/Char/Ctl.x').default == 9.0 and
           not shared.GetAttributeAtPath('/Char/Ctl.x').HasInfo(
               'documentation'),
           'undoing the documentation took the other write with it')
    container.stack.Redo()
    _Check(shared.GetAttributeAtPath('/Char/Ctl.x').default == 9.0 and
           shared.GetAttributeAtPath('/Char/Ctl.x').GetInfo(
               'documentation') == 'posed by the assistant',
           'redoing the documentation took the other write with it')


def TestPythonLayerFieldUndo():
    """A layer field a Python edit sets on the pseudo root is recorded and
    taken back, without touching the prims below it."""
    root = _Layer('pseudo_field.usda', _CAPTURE_USDA)
    stage = Usd.Stage.Open(root)
    container = _CountingContainer()
    box = tools.Toolbox(assistant_session.Session(_Api(stage), container),
                        can_run=True)
    text, failed = box.call('run_python', {'code': 'stage.SetStartTimeCode(5)\n'
                                               'stage.SetEndTimeCode(12)'})
    _Check(not failed, 'the layer field write failed: %s' % text)
    box.close()
    _Check(len(container.stack.pushed) == 1,
           'a pseudo root edit made no undo step: %s' %
           (container.stack.pushed,))
    _Check(root.pseudoRoot.GetInfo('startTimeCode') == 5.0 and
           root.pseudoRoot.GetInfo('endTimeCode') == 12.0,
           'the reply did not set the layer fields')
    container.stack.Undo()
    _Check(not root.pseudoRoot.HasInfo('startTimeCode') and
           not root.pseudoRoot.HasInfo('endTimeCode'),
           'undo left the layer fields set')
    _Check(root.GetAttributeAtPath('/Char/Ctl.x').default == 1.0,
           'undoing the layer fields changed a prim')
    container.stack.Redo()
    _Check(root.pseudoRoot.GetInfo('startTimeCode') == 5.0 and
           root.pseudoRoot.GetInfo('endTimeCode') == 12.0,
           'redo did not set the layer fields back')


def TestCloseDuringFence():
    """A close that happens while a reply's fences are running stops the
    actions that have not run yet: the reply is still the active writer."""
    stage = _Stage()
    container = _CountingContainer()
    assistant = _Panel(stage, container)
    box = _ReplyToolbox(assistant)
    _Reply(assistant, _Fence('[{"action": "set_attr", "path": "/Char/Ctl", '
                             '"name": "x", "value": 2.0},\n'
                             '{"action": "set_attr", "path": "/Char/Ctl", '
                             '"name": "y", "type": "double", "value": 7.0}]'),
           box)
    closed = [False]

    def on_change(_notice, _sender):
        if not closed[0] and stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 2:
            closed[0] = True
            assistant.close()

    key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, on_change, stage)
    try:
        assistant.on_finished()
    finally:
        key.Revoke()
    _Check(closed[0], 'the close notice never fired')
    _Check(stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 2.0 and
           not stage.GetAttributeAtPath('/Char/Ctl.y').IsValid(),
           'the fence kept authoring after the panel closed')
    _Check(len(container.stack.pushed) == 1,
           'the close did not settle one step: %s' % (container.stack.pushed,))
    _Check(any('closed' in problem.lower()
               for message in assistant.messages
               for problem in message.get('failed', [])),
           'the stopped fence is not reported: %s' %
           ([m.get('failed') for m in assistant.messages],))
    container.stack.Undo()
    _Check(stage.GetAttributeAtPath('/Char/Ctl.x').Get() == 1.0,
           'the step the close settled is not usable')


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
                 TestChangeBlocksAreTitledApart,
                 TestStageReplacementRejectsLateWrites,
                 TestDocumentGenerationRejectsLateWrites,
                 TestGroupFailsAtomically,
                 TestSharedLayerPythonLeavesIndependentField,
                 TestPythonFieldUndoKeepsForeignValue,
                 TestPythonLayerFieldUndo,
                 TestReferencedMoves,
                 TestInterleavedEditorHistory,
                 TestCloseDuringWrite,
                 TestCloseDuringFence):
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
