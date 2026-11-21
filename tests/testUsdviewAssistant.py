#
# THE AI ASSISTANT, end to end in usdview.
#
# tests/python/test_assistant.py covers the tools, undo and the MCP server
# on an in-memory stage. This covers what only a live usdview has: the
# RigExec menu item, the panel docking on the main window, the tools on
# the biped (its rig, a control's avars, a keyed pose that moves the
# evaluated rig and comes back off the shared undo stack, authoring spread
# over several layers and a save, the live evaluator, an independent
# evaluation) and a viewport picture reaching an MCP client as image
# content. No provider is called. Edits land in the session layer or in a
# scratch layer the test creates itself; the example file is never written.
#
import json
import os
import shutil
import tempfile
import threading
import urllib.request

os.environ.setdefault("USDRIG_USER_DIR",
                      tempfile.mkdtemp(prefix="usdrig_assistant_gui_"))

from pxr import Sdf, Usd  # noqa: E402

RIG = "/Biped/Rig"
CONTROLS = RIG + "/Main/Shot/Aux/Controls"
ARM_L_ROOT = CONTROLS + "/L_Arm"
FK_ARM = CONTROLS + "/M_Body/M_Torso/M_Chest/M_ChestTop/L_Shldr/L_UpArmSwing"
FK_SHOULDER = FK_ARM + "/L_UpArm"

# A plain Xform outside the rig, and one double on it, that the layer cases
# below author into; the biped reads neither.
PROBE_PRIM = "/AssistantUndoProbe"
PROBE_ATTR = "assistantUndoProbe"
PROBE_SPEC = Sdf.Path(PROBE_PRIM).AppendProperty(PROBE_ATTR)
# A step put on the host undo stack before a reply, so one Undo() shows
# whether the reply went on as one entry or as several.
SENTINEL = "an earlier change"
FENCED = """Setting the probe on the other layer too.

```usdrig-actions
[
  {{"action": "set_edit_target", "layer": "{layer}"}},
  {{"action": "set_attr", "path": "{path}", "name": "{name}", "value": 5.0}}
]
```"""


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Menus(appController):
    from pxr.Usdviewq.qt import QtWidgets
    menus = {}
    for child in appController._mainWindow.menuBar().children():
        if isinstance(child, QtWidgets.QMenu):
            menus[str(child.title()).replace("&", "")] = child
    return menus


def _Submenu(menu, title):
    for action in menu.actions():
        if action.menu() is not None and action.text() == title:
            return action.menu()
    return None


def testUsdviewInputFunction(appController):
    from pxr.Usdviewq.qt import QtCore, QtWidgets

    from usdRigAssistant import host, mcp_server

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    session = stage.GetSessionLayer()
    if stage.GetEditTarget().GetLayer() != session:
        stage.SetEditTarget(Usd.EditTarget(session))

    # --- the menu item ----------------------------------------------------
    reg = appController._plugRegistry
    command = reg.getCommandPlugin("RigExecUsdviewContainer.assistant")
    _Check(command is not None, "RigExec -> AI Assistant is registered")
    editors = _Submenu(_Menus(appController)["RigExec"], "General Editors")
    _Check("AI Assistant" in [a.text() for a in editors.actions()],
           "RigExec -> General Editors has an AI Assistant item")

    # --- off: the settings open instead; on: the panel docks -------------
    host.user_prefs[host.USER_PREF.AI_ENABLED] = False
    command.run()
    appController._processEvents()
    import assistantUI
    dialog = assistantUI._dialogs.Get(api)
    _Check(dialog is not None and dialog.isVisible(),
           "turned off, the item opens the settings")
    dialog.enable_check.setChecked(True)
    appController._processEvents()
    panel = assistantUI._panels.Get(api)
    _Check(panel is not None and panel.isVisible(),
           "turning it on opens the panel")
    _Check(appController._mainWindow.dockWidgetArea(panel) ==
           QtCore.Qt.RightDockWidgetArea, "the panel docks on the right")
    shot = os.environ.get("RIGEXEC_ASSISTANT_SHOT")
    if shot:
        dialog.resize(620, 820)
        appController._processEvents()
        dialog.grab().save(shot + "_settings.png")
    dialog.close()

    # --- the tools on the live session ------------------------------------
    from usdRigAssistant import tools
    box = tools.Toolbox(panel.session, can_edit=True, can_run=True,
                        parent=panel)
    text, failed = box.call("list_rigs", {})
    _Check(not failed and RIG in text, "list_rigs finds the biped: %s"
           % text)
    text, failed = box.call("list_avars", {"path": FK_SHOULDER})
    avars = json.loads(text) if not failed else {}
    names = {a["name"] for a in avars.get("avars", [])}
    _Check("avars:rz" in names, "the FK shoulder offers avars:rz: %s"
           % text)

    def elbow():
        import rigExecUsdview
        container = rigExecUsdview.ContainerFor(api)
        frame = api.frame
        _ = container
        text, failed = box.call("evaluate_rig", {
            "path": RIG, "time": frame.GetValue(),
            "joints": ["elbow_l_def"]})
        pose = json.loads(text)
        _Check(not failed and pose["valid"], "the rig evaluates: %s"
               % text[:400])
        for path, frame in pose["joint_frames"].items():
            if path.endswith("/elbow_l_def"):
                return frame["origin"]
        raise AssertionError("no left elbow in the pose")

    before = elbow()
    frame = api.frame.GetValue()
    text, failed = box.call("set_avars", {
        "values": [{"path": FK_SHOULDER, "name": "avars:rz",
                    "value": 30}], "time": frame})
    _Check(not failed, "set_avars keys the shoulder: %s" % text)
    after = elbow()
    moved = sum((a - b) ** 2 for a, b in zip(after, before)) ** 0.5
    _Check(moved > 1.0, "the pose moved the evaluated elbow: %.3f" % moved)
    import rigExecUsdview
    stack = rigExecUsdview.ContainerFor(api)._UndoStack()
    _Check(stack.UndoText() == "AI Assistant pose",
           "the pose is on the shared undo stack: %r" % stack.UndoText())
    stack.Undo()
    back = elbow()
    error = sum((a - b) ** 2 for a, b in zip(back, before)) ** 0.5
    _Check(error < 1e-6, "undo puts the elbow back: %.6f" % error)

    # --- one reply, one undo step, across layers -------------------------
    #
    # The pose above keys one value in one layer. A reply can also spread
    # its changes over several layers, or mix a tool edit with a fenced
    # action, or save a layer, which is no edit at all. Each of those has
    # to come back off the host stack in as many steps as the user saw
    # replies.
    _UndoAcrossLayers(panel, stack, tempfile.mkdtemp(prefix="usdrig_undo_"))

    text, failed = box.call("get_live_evaluator", {})
    _Check(not failed and text, "the live evaluator answers: %s"
           % text[:300])
    text, failed = box.call("check_rig", {"path": RIG})
    _Check(not failed and json.loads(text)["compiled"],
           "check_rig compiles the biped: %s" % text[:300])
    text, failed = box.call("run_python", {"code": "stage.GetPrimAtPath("
                                           "'%s').GetTypeName()" % RIG})
    _Check(not failed and "RigExecRoot" in json.loads(text).get("value", ""),
           "run_python sees the stage: %s" % text)

    # --- a viewport picture over MCP --------------------------------------
    server = mcp_server.McpServer()
    token = server.open(box)
    out = {}

    def call():
        request = urllib.request.Request(
            server.url, data=json.dumps({
                "jsonrpc": "2.0", "id": 1, "method": "tools/call",
                "params": {"name": "capture_viewport",
                           "arguments": {}}}).encode("utf-8"),
            headers={"Content-Type": "application/json",
                     "Authorization": "Bearer " + token})
        with urllib.request.urlopen(request, timeout=60) as answer:
            out["answer"] = json.loads(answer.read())

    thread = threading.Thread(target=call)
    thread.start()
    while thread.is_alive():
        QtWidgets.QApplication.processEvents()
        thread.join(0.01)
    server.stop()
    content = out["answer"]["result"]["content"]
    kinds = [c["type"] for c in content]
    _Check(kinds == ["text", "image"] and len(content[1]["data"]) > 1000,
           "capture_viewport reaches MCP as a picture: %s" % kinds)

    if shot:
        appController._mainWindow.grab().save(shot + "_window.png")
    panel.close()
    print("RIGEXEC_ASSISTANT_OK")


# --- authoring spread over layers, one reply to one undo step -----------
#
# The example is read through an anonymous session layer over its anchored
# files, and one reply can write in more than one of them at once. However
# the changes spread out, one reply is one step for the user, so one
# Undo() has to take all of it back. What each layer wrote is read from
# that layer's own spec rather than from the composed stage, where the
# session layer hides a weaker layer's opinion.


def _WriteLayer(folder, name):
    """A real, empty layer file in folder, and its path."""
    path = os.path.join(folder, name)
    _Check(Sdf.Layer.CreateNew(path) is not None,
           "the test can write %s" % name)
    return path


def _Layer(stage, identifier):
    """The live layer in stage with this identifier."""
    for layer in stage.GetLayerStack():
        if layer.identifier == identifier:
            return layer
    raise AssertionError("no %s in the layer stack" % identifier)


def _Authored(layer, spec_path):
    """The default value this one layer authors at spec_path, or None."""
    spec = layer.GetAttributeAtPath(spec_path)
    if spec is None or not spec.HasDefaultValue():
        return None
    return spec.default


def _FromDisk(path, spec_path, tag):
    """The default value the bytes on disk author, read through a copy of
    the file so USD cannot answer out of the live layer."""
    copy = "%s-readback-%s.usda" % (os.path.splitext(path)[0], tag)
    shutil.copyfile(path, copy)
    layer = Sdf.Layer.FindOrOpen(copy)
    _Check(layer is not None, "the test can read back %s" % tag)
    return _Authored(layer, spec_path)


def _Arcs(layer):
    """layer's sublayer arcs: their paths and the offsets beside them."""
    return (list(layer.subLayerPaths),
            [Sdf.LayerOffset(offset) for offset in layer.subLayerOffsets])


def _Spelled(offsets):
    return [(offset.offset, offset.scale) for offset in offsets]


def _UndoAcrossLayers(panel, stack, folder):
    """Four replies against the live biped stage, each one step on the
    host undo stack the pose above already used."""
    session = panel.session
    stage = session.stage
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    root = stage.GetRootLayer()
    root_paths = list(root.subLayerPaths)
    # Saving reaches only a layer that is in the stack and has a file of
    # its own, so the scratch layer is added as the example's weakest
    # sublayer. The arc comes off again at the end.
    nested = _WriteLayer(folder, "nested.usda")
    root.subLayerPaths.append(nested)
    _SublayerArcStep(session, panel, stack, folder)
    _TwoLayerStep(session, panel, stack, nested)
    _ToolAndFenceStep(session, panel, stack, nested)
    _SavingIsNoEdit(session, panel, stack, nested)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    stage.RemovePrim(PROBE_PRIM)
    root.subLayerPaths = root_paths
    stage.GetSessionLayer().customLayerData = {}
    _Check(not stage.GetPrimAtPath(PROBE_PRIM).IsValid(),
           "the probe leaves no prim in the stage")
    _Check(list(root.subLayerPaths) == root_paths,
           "the example keeps the sublayers it came with")


def _SublayerArcStep(session, panel, stack, folder):
    """An add_sublayer beside a retimed sublayer, one step each way."""
    import rigExecUndo
    from usdRigAssistant import tools
    layer = session.stage.GetSessionLayer()
    _Check(session.stage.GetEditTarget().GetLayer() is layer,
           "the session layer is the layer under edit")
    _Check(_Arcs(layer) == ([], []),
           "the session layer starts with no sublayers")
    retime = _WriteLayer(folder, "retime.usda")
    added = _WriteLayer(folder, "added.usda")
    layer.subLayerPaths = [retime]
    layer.subLayerOffsets[0] = Sdf.LayerOffset(2.0, 6.0)
    layer.customLayerData = {"usdRigUndoProbe": "kept"}
    before = _Arcs(layer)
    _Check(before[1] != [Sdf.LayerOffset()],
           "the sublayer beside the new one is retimed: %s"
           % _Spelled(before[1]))
    stack.Push(rigExecUndo.Edit(SENTINEL, []))
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("change_stage", {
        "changes": [{"action": "add_sublayer", "asset": added,
                     "position": 0}]})
    _Check(not failed, "add_sublayer applied: %s" % text)
    box.finish()
    arcs = _Arcs(layer)
    _Check(arcs[0] == [added, retime],
           "the new sublayer is the strongest: %s" % arcs[0])
    _Check(arcs[1] == [Sdf.LayerOffset(), Sdf.LayerOffset(2.0, 6.0)],
           "the new arc leaves the old offset on the old arc: %s"
           % _Spelled(arcs[1]))
    # The old arc is retimed after the reply, so what undo gives back can
    # only come from what the reply recorded.
    layer.subLayerOffsets[1] = Sdf.LayerOffset(9.0, 1.0)
    stack.Undo()
    _Check(_Arcs(layer) == before,
           "one undo brings back the arc and the offsets as the reply "
           "found them: %s" % _Spelled(_Arcs(layer)[1]))
    _Check(stack.UndoText() == SENTINEL,
           "the reply was one undo step: %r" % stack.UndoText())
    stack.Redo()
    _Check(_Arcs(layer) == arcs,
           "one redo puts the reply's own arcs back, offsets and all")
    _Check(list(layer.customLayerData) == ["usdRigUndoProbe"],
           "an unrelated field on the layer survives")
    stack.Undo()
    layer.subLayerPaths = []
    _Check(_Arcs(layer) == ([], []), "the session layer is empty again")


def _TwoLayerStep(session, panel, stack, nested):
    """One call that defines a prim, sets a property in two layers, and
    switches the edit target between them."""
    import rigExecUndo
    from usdRigAssistant import tools
    stage = session.stage
    first = stage.GetSessionLayer()
    _Check(any(layer.identifier == nested
               for layer in stage.GetLayerStack(includeSessionLayers=False)),
           "the second layer is in the layer stack")
    stage.SetEditTarget(Usd.EditTarget(first))
    stack.Push(rigExecUndo.Edit(SENTINEL, []))
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "define_prim", "path": PROBE_PRIM, "type": "Xform"},
        {"action": "set_attr", "path": PROBE_PRIM, "name": PROBE_ATTR,
         "type": "double", "value": 7.0},
        {"action": "set_edit_target", "layer": nested},
        {"action": "set_attr", "path": PROBE_PRIM, "name": PROBE_ATTR,
         "type": "double", "value": 3.0}]})
    _Check(not failed, "one change_stage covers two layers: %s" % text)
    box.finish()
    second = _Layer(stage, nested)
    _Check(stage.GetEditTarget().GetLayer() is second,
           "the reply left the edit target on the second layer")
    _Check(_Authored(first, PROBE_SPEC) == 7.0,
           "the first layer authors its own value")
    _Check(_Authored(second, PROBE_SPEC) == 3.0,
           "the second authors the other one")
    stack.Undo()
    _Check(_Authored(first, PROBE_SPEC) is None
           and _Authored(second, PROBE_SPEC) is None,
           "one undo took the property off every layer it wrote")
    _Check(not stage.GetPrimAtPath(PROBE_PRIM).IsValid(),
           "and the prim it set the property on")
    _Check(stage.GetEditTarget().GetLayer() is second,
           "undo leaves the edit target where the reply put it")
    _Check(stack.UndoText() == SENTINEL,
           "the reply was one undo step: %r" % stack.UndoText())
    stack.Redo()
    _Check(_Authored(first, PROBE_SPEC) == 7.0
           and _Authored(second, PROBE_SPEC) == 3.0,
           "one redo brings both layers back")


def _ToolAndFenceStep(session, panel, stack, nested):
    """A tool edit and the same reply's fenced action, closed once."""
    import rigExecUndo
    from usdRigAssistant import stage_actions, tools
    stage = session.stage
    first = stage.GetSessionLayer()
    second = _Layer(stage, nested)
    stage.SetEditTarget(Usd.EditTarget(first))
    _Check(_Authored(first, PROBE_SPEC) == 7.0
           and _Authored(second, PROBE_SPEC) == 3.0,
           "the layers start where the last reply left them")
    stack.Push(rigExecUndo.Edit(SENTINEL, []))
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": PROBE_PRIM, "name": PROBE_ATTR,
         "value": 11.0}]})
    _Check(not failed, "the tool edit applied: %s" % text)
    block = FENCED.format(layer=nested, path=PROBE_PRIM, name=PROBE_ATTR)
    actions, problems = stage_actions.find_actions(block)
    _Check(not problems and len(actions) == 2,
           "the reply's fenced block reads: %s" % problems)
    text, failed = box.call("change_stage", {"changes": actions})
    _Check(not failed, "the fenced action applied: %s" % text)
    box.finish()
    _Check(_Authored(first, PROBE_SPEC) == 11.0,
           "the tool half wrote the first layer")
    _Check(_Authored(second, PROBE_SPEC) == 5.0,
           "the fenced half wrote the second")
    stack.Undo()
    _Check(_Authored(first, PROBE_SPEC) == 7.0,
           "undo puts the first layer's original value back")
    _Check(_Authored(second, PROBE_SPEC) == 3.0,
           "and the second layer's original value")
    _Check(stack.UndoText() == SENTINEL,
           "both halves were one undo step: %r" % stack.UndoText())
    stack.Redo()
    _Check(_Authored(first, PROBE_SPEC) == 11.0
           and _Authored(second, PROBE_SPEC) == 5.0,
           "one redo brings both halves back")


def _SavingIsNoEdit(session, panel, stack, nested):
    """A save is an effect, not an edit: Undo() does not reach the file."""
    import rigExecUndo
    from usdRigAssistant import tools
    stage = session.stage
    first = stage.GetSessionLayer()
    second = _Layer(stage, nested)
    stage.SetEditTarget(Usd.EditTarget(first))
    _Check(_Authored(first, PROBE_SPEC) == 11.0
           and _Authored(second, PROBE_SPEC) == 5.0,
           "the layers start where the last reply's redo left them")
    stack.Push(rigExecUndo.Edit(SENTINEL, []))
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("save_layers", {"layers": [first.identifier]})
    _Check(not failed and "Nothing needed saving" in text,
           "the anonymous session layer is never saved: %s" % text)
    _Check(stack.UndoText() == SENTINEL,
           "a save opens no undo step: %r" % stack.UndoText())
    stage.SetEditTarget(Usd.EditTarget(second))
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("change_stage", {"changes": [
        {"action": "set_attr", "path": PROBE_PRIM, "name": PROBE_ATTR,
         "value": 9.0}]})
    _Check(not failed, "the file layer is edited: %s" % text)
    box.finish()
    _Check(stack.UndoText() == "AI Assistant changes",
           "the edit is on the shared undo stack: %r" % stack.UndoText())
    box = tools.Toolbox(session, can_edit=True, parent=panel)
    text, failed = box.call("save_layers", {"layers": [nested]})
    _Check(not failed and nested in text, "the named layer is saved: %s"
           % text)
    _Check(_FromDisk(nested, PROBE_SPEC, "saved") == 9.0,
           "the file on disk holds the new value")
    stack.Undo()
    _Check(_Authored(second, PROBE_SPEC) == 5.0,
           "undo puts back the value the save had written over")
    _Check(stack.UndoText() == SENTINEL,
           "the save added no step of its own: %r" % stack.UndoText())
    _Check(_FromDisk(nested, PROBE_SPEC, "undone") == 9.0,
           "and the file on disk keeps its value")
