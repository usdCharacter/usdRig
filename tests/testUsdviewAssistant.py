#
# THE AI ASSISTANT, end to end in usdview.
#
# tests/python/test_assistant.py covers the tools, undo and the MCP server
# on an in-memory stage. This covers what only a live usdview has: the
# RigExec menu item, the panel docking on the main window, the tools on
# the biped (its rig, a control's avars, a keyed pose that moves the
# evaluated rig and comes back off the shared undo stack, the live
# evaluator, an independent evaluation) and a viewport picture reaching an
# MCP client as image content. No provider is called. Every edit lands in
# the session layer; the file is never written.
#
import json
import os
import tempfile
import threading
import urllib.request

os.environ.setdefault("USDRIG_USER_DIR",
                      tempfile.mkdtemp(prefix="usdrig_assistant_gui_"))

from pxr import Usd  # noqa: E402

RIG = "/Biped/Rig"
CONTROLS = RIG + "/Main/Shot/Aux/Controls"
ARM_L_ROOT = CONTROLS + "/L_Arm"
FK_ARM = CONTROLS + "/M_Body/M_Torso/M_Chest/M_ChestTop/L_Shldr/L_UpArmSwing"
FK_SHOULDER = FK_ARM + "/L_UpArm"


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
    box.finish()
    import rigExecUsdview
    stack = rigExecUsdview.ContainerFor(api)._UndoStack()
    _Check(stack.UndoText() == "AI Assistant pose",
           "the pose is on the shared undo stack: %r" % stack.UndoText())
    stack.Undo()
    back = elbow()
    error = sum((a - b) ** 2 for a, b in zip(back, before)) ** 0.5
    _Check(error < 1e-6, "undo puts the elbow back: %.6f" % error)

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
