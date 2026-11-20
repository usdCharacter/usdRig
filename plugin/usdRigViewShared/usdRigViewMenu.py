"""Shared ranked insertion for the viewer's RigExec menu."""

_SUBMENU_RANKS = {"Viewport": 10, "General Editors": 20,
                  "Animation Editors": 30}
_MENU_RANK = "rigExecMenuRank"


def _PlaceByRank(qMenu, action, rank):
    action.setProperty(_MENU_RANK, rank)
    for other in qMenu.actions():
        if other == action:
            return
        otherRank = other.property(_MENU_RANK)
        if otherRank is not None and otherRank > rank:
            qMenu.removeAction(action)
            qMenu.insertAction(other, action)
            return


def AddToRigExecMenu(plugUIBuilder, submenu, commandPlugin, rank):
    """Return the command's action in RigExec or its named submenu.

    Use None for a direct child. Ranks apply only on insertion; ties keep
    insertion order, and adding the same command again returns its action.
    """
    menu = plugUIBuilder.findOrCreateMenu("RigExec")
    if submenu is not None:
        menu = menu.findOrCreateSubmenu(submenu)

    actions = getattr(menu, "_usdRigCommandActions", None)
    if actions is None:
        actions = menu._usdRigCommandActions = {}
    if commandPlugin in actions:
        return actions[commandPlugin]

    action = menu.addItem(commandPlugin)
    qMenu = action.parent()
    _PlaceByRank(qMenu, action, rank)
    submenuRank = _SUBMENU_RANKS.get(submenu)
    if submenuRank is not None:
        _PlaceByRank(qMenu.parent(), qMenu.menuAction(), submenuRank)
    actions[commandPlugin] = action
    return action
