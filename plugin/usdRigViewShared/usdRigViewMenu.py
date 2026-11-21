"""Shared ranked insertion for the viewer's RigExec menu."""

_SUBMENU_RANKS = {"Viewport": 10, "General Editors": 20,
                  "Animation Editors": 30}
_MENU_RANK = "rigExecMenuRank"
_COMMAND = "usdRigCommandPlugin"


def _Rank(action):
    submenu = action.menu()
    if submenu is not None and submenu.title() in _SUBMENU_RANKS:
        rank = _SUBMENU_RANKS[submenu.title()]
        action.setProperty(_MENU_RANK, rank)
        return rank
    return action.property(_MENU_RANK)


def _OrderTopLevel(qMenu):
    actions = qMenu.actions()
    ranks = [_Rank(action) for action in actions]
    ranked = sorted((action for action, rank in zip(actions, ranks)
                     if rank is not None), key=_Rank)
    # Keep unranked actions in their slots while sorting ranked siblings.
    slots = iter(ranked)
    ordered = [action if rank is None else next(slots)
               for action, rank in zip(actions, ranks)]
    if ordered == actions:
        return
    before = None
    for action in reversed(ordered):
        if _Rank(action) is not None:
            qMenu.removeAction(action)
            qMenu.insertAction(before, action)
        before = action


def AddToRigExecMenu(plugUIBuilder, submenu, commandPlugin, rank):
    """Return the live command action in RigExec or its named submenu.

    Use None for a direct child. Command ranks apply only on insertion;
    ties keep insertion order.
    """
    top = plugUIBuilder.findOrCreateMenu("RigExec")
    menu = top if submenu is None else top.findOrCreateSubmenu(submenu)
    qMenu = menu._qMenu
    for action in qMenu.actions():
        if action.property(_COMMAND) is commandPlugin:
            _OrderTopLevel(top._qMenu)
            return action

    action = menu.addItem(commandPlugin)
    action.setProperty(_COMMAND, commandPlugin)
    action.setProperty(_MENU_RANK, rank)
    for other in qMenu.actions():
        if other == action:
            break
        otherRank = _Rank(other)
        if otherRank is not None and otherRank > rank:
            qMenu.removeAction(action)
            qMenu.insertAction(other, action)
            break
    _OrderTopLevel(top._qMenu)
    return action
