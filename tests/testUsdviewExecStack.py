"""Execution Stack uses the viewer stage and displays its compiled mover order."""
import os

RIG = "/Biped/Rig"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RigExecMenu(appController):
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


def _Trigger(menu, title):
    for action in menu.actions():
        if action.text() == title:
            action.trigger()
            return True
    return False


def _ShownMovers(panel):
    """How many mover rows the current filter leaves visible."""
    return sum(1 for e in panel._movers
               if panel._Keep(e["type"], e["path"].rsplit("/", 1)[-1],
                              e.get("targets") or []))


def testUsdviewInputFunction(appController):
    import execStackUI
    import rigexec

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage

    # --- 1. registration, in the RigExec menu only ----------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin(
        "RigExecUsdviewContainer.execStack") is not None,
        "RigExec -> Execution Stack command is registered")
    menus = _RigExecMenu(appController)
    _Check("RigExec" in menus, "there is a RigExec menu: %s" % sorted(menus))
    rigMenu = _Submenu(menus["RigExec"], "General Editors")
    _Check(rigMenu is not None, "RigExec has a General Editors submenu")
    titles = [a.text() for a in rigMenu.actions()]
    title = next((t for t in titles if "Execution Stack" in t), None)
    _Check(title is not None,
           "RigExec -> General Editors has an Execution Stack item: %s"
           % titles)
    window = menus.get("Window")
    if window is not None:
        _Check(not any("Execution Stack" in a.text()
                       for a in window.actions()),
               "the panel is in the RigExec menu ONLY, not Window")

    # --- 2. opening it populates the ORDERED list -----------------------
    _Check(_Trigger(rigMenu, title), "the menu item triggered")
    appController._processEvents()
    panel = execStackUI.ExecStackPanel.GetInstance(api)
    _Check(panel is not None, "the panel was created")
    appController._processEvents()

    _Check(getattr(panel, "_compileError", None) is None,
           "the rig compiled inside usdview, but: %s"
           % getattr(panel, "_compileError", None))
    movers = list(panel._movers)
    _Check(len(movers) > 10,
           "the ordered mover list is populated, got %d" % len(movers))
    _Check(panel._solvers, "solvers were listed too")

    # --- 3. the order matches a script-side compile ---------------------
    rig = rigexec.Rig(stage, RIG)
    rig.compile()
    order = rig.mover_order
    if callable(order):
        order = order()
    expected = [str(e["path"]) for e in order]
    got = [str(e["path"]) for e in movers]
    _Check(got == expected,
           "panel order differs from a script compile:\n  panel  %s\n  "
           "script %s" % (got[:6], expected[:6]))

    # The compiler's diagnostics are displayed without type-order heuristics.
    status = panel._StatusText(len(panel._solvers), len(movers))
    for diagnostic in panel._compileDiagnostics:
        _Check(diagnostic in status, "compiler diagnostic is displayed")

    # --- 5. filtering narrows rather than empties ------------------------
    box = panel._filterBox
    counts = {}
    for i in range(box.count()):
        box.setCurrentIndex(i)
        appController._processEvents()
        counts[str(box.itemText(i))] = _ShownMovers(panel)
    _Check(any(v > 0 for v in counts.values()),
           "at least one filter shows mover rows: %s" % counts)
    _Check(max(counts.values()) == len(movers),
           "some filter shows every mover (the 'all' case): %s" % counts)
    _Check(min(counts.values()) < len(movers),
           "some filter actually narrows the list: %s" % counts)
    box.setCurrentIndex(0)
    appController._processEvents()

    shot = os.getenv("RIGEXEC_EXECSTACK_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)

    print("RIGEXEC_EXEC_STACK_OK compiled in usdview, %d solvers, %d movers, "
          "order matches a script compile, filters %s"
          % (len(panel._solvers), len(movers), counts))
