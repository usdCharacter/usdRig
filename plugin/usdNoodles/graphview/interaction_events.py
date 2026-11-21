#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Qt event overrides, shortcuts, menus and hotbox: the interaction owner acts.

These are the ``interaction``-tagged bodies that own a QWidget: raw event
dispatch, keyboard and wheel handling, shortcut-conflict migration and
widget actions, the context menu, container opening, the preferences
dialog, hotbox content, popups, and value-cell/token-press dispatch.
``GraphWidgetBase`` in ``interaction.py`` binds every function here by
explicit assignment (never a forwarding body); the composing
``GraphView`` inherits those bindings.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``. Names the suite patches on the legacy module
(``QtWidgets`` here, plus ``NoodlesConfig`` in the shortcut helpers)
resolve at call time through ``legacy_symbol``; everything else is
imported directly. ``super()`` chains spell the Qt base explicitly as
``QGLWidget.<method>(view, event)`` (the next implementation after the
base in the view's MRO, exactly what the original ``super()`` reached).
"""

from pxr import Tf
from pxr.Usdviewq.qt import QGLWidget, QtCore, QtGui

from UsdNoodles.core import LinkSelectionMode

from .. import noodlesValues
from ..noodlesPreferences import NoodlesPreferences
from ..utils import M, MenuBuilder
from ..widgets.selectableList import SelectableListItem
from . import intents as _intents
from . import interaction as _gestures
from . import stage_adapter
from .legacy import legacy_symbol


def _updateNodeCreationHotbox(view):
    """
    Update the node creation hotbox configuration from enabled libraries.

    This method iterates through all enabled node libraries, collects their
    families and node types, and configures the hotbox to display them.
    Library references are embedded in the node data for later use.

    The applied API schemas registered in this build are appended after the
    libraries. They are not node types - picking one applies it to the
    selected prims rather than creating a prim.
    """
    # Collect all node types with library references from enabled libraries
    node_items = []

    for library in view._nodeLibraries:
        if not library.enabled:
            continue

        # Get families from this library
        families = library.get_families()

        for family in families:
            # Get node types for this family
            node_types = library.get_node_types(family)

            if not node_types:
                continue

            # Create node items with embedded library references
            for node_type in node_types:
                node_item = {
                    "name": node_type["name"],
                    "family": family,
                    "identifier": node_type["identifier"],
                    "library": library,  # Reference to the library for creation
                    "kind": "prim",
                }
                node_items.append(node_item)

    node_items.extend(view._apiSchemaHotboxItems())

    # Configure the hotbox with the collected node items
    view.nodeCreationHotbox.configure_from_libraries(node_items)


def _apiSchemaHotboxItems():
    """Build hotbox entries for every applied API schema in this build.

    Each entry is suffixed "*api" and grouped under the plugin that
    defines it, so the list reads as schemas to apply rather than prims to
    create. The marker doubles as a filter term: the list filters on a
    plain substring of the display name, and no prim type contains "*", so
    typing it narrows the hotbox to schemas alone. It stays out of the
    "identifier" field, which is what ApplyAPI receives.

    get_api_schemas() returns them sorted by plugin, which the list widget
    requires: it emits a group header whenever the group changes, so a
    non-contiguous group would repeat its header.
    """
    from ..apiSchemas import API_SCHEMA_KIND, get_api_schemas

    return [
        {
            "name": f"{schema.identifier} *api",
            "family": f"API Schemas: {schema.plugin}",
            "identifier": schema.identifier,
            "library": None,
            "kind": API_SCHEMA_KIND,
        }
        for schema in get_api_schemas()
    ]


def _showHotkeyPopup(view, keyText, direction):
    if direction == "left":
        message = f"<- {keyText}"
    else:
        message = f"{keyText} ->"

    cursorPos = view.mapFromGlobal(QtGui.QCursor.pos())
    view.hotkeyPopup.show(message, cursorPos)
    view.update()


def _showPopupMessage(view, message, duration_ms=None):
    cursorPos = view.mapFromGlobal(QtGui.QCursor.pos())
    view.hotkeyPopup.show(message, cursorPos, duration_ms=duration_ms)
    view.update()


def _showWarningPopup(view, message):
    view._showPopupMessage(message, duration_ms=view._WARNING_POPUP_DURATION_MS)


def timerEvent(view, event):
    if view.hotkeyPopup.handle_timer_event(event.timerId()):
        view.update()


def closeEvent(view, event):
    stage_adapter.closeStageLifecycle(view)
    QGLWidget.closeEvent(view, event)


def enterEvent(view, event):
    app = legacy_symbol("QtWidgets").QApplication.instance()
    if app:
        view._previousActiveWindow = app.activeWindow()

    topLevel = view.window()
    if topLevel:
        topLevel.activateWindow()
    view.setFocus(QtCore.Qt.MouseFocusReason)

    QGLWidget.enterEvent(view, event)


def leaveEvent(view, event):
    if view._previousActiveWindow and view._previousActiveWindow.isVisible():
        view._previousActiveWindow.activateWindow()
    view._previousActiveWindow = None

    view._clearPropertyTooltip()

    # The resize cursor belongs to a value cell the pointer has now left.
    if view._valueCursorSet is not None:
        view.setCursor(QtCore.Qt.ArrowCursor)
        view._valueCursorSet = None
    view._valueHover = None
    # A drag whose button release happened outside the widget would
    # otherwise leave _mungState live and the notice handler disabled.
    if view._mungState is not None and not (
        legacy_symbol("QtWidgets").QApplication.mouseButtons() & QtCore.Qt.LeftButton
    ):
        view._cancelMung()

    QGLWidget.leaveEvent(view, event)


def _resolveShortcutConflicts(view, shortcuts):
    """Reset stored shortcuts that collide with a later-defined action.

    When a default shortcut is reassigned between releases (e.g. "D" moved
    from shortcutToggleLinkDimming to shortcutRemoveFromGraph), a user's
    persisted state can still bind the key to the original action. The
    manual dispatch in keyPressEvent iterates _shortcutActions in insertion
    order and triggers the first match, which means the older action wins
    and the new one is silently masked.

    Walk the shortcuts list and, for each duplicate normalized key
    sequence, reset every earlier configKey to its schema default so the
    latest definition takes effect.
    """
    settings = legacy_symbol("NoodlesConfig").settings()
    if settings is None:
        return

    # Collect the configured key string for each shortcut in declaration
    # order. We compare on QKeySequence.toString() to normalize formatting
    # (e.g. "ctrl+s" vs "Ctrl+S").
    loaded = []
    for _name, configKey, _checkable, _checkedGetter, _handler in shortcuts:
        shortcutStr = legacy_symbol("NoodlesConfig").get(configKey, "")
        normalized = (
            QtGui.QKeySequence(shortcutStr).toString() if shortcutStr else ""
        )
        loaded.append((configKey, normalized))

    # For each later entry, blank out any earlier entry that holds the
    # same key. Walking from the end keeps the latest-defined action
    # authoritative without re-iterating.
    seen = {}
    for configKey, normalized in reversed(loaded):
        if not normalized:
            continue
        if normalized in seen:
            # An entry defined LATER already claims this key, so reset
            # this earlier entry to its schema default to break the tie.
            info = settings.getSettingInfo(configKey)
            if not info:
                continue
            defaultStr = info.get("default", "")
            defaultNormalized = (
                QtGui.QKeySequence(defaultStr).toString() if defaultStr else ""
            )
            # Only rewrite when the default would actually free the key,
            # otherwise we would loop the same conflict on the next launch.
            if defaultNormalized == normalized:
                continue
            Tf.Status(
                f"Resetting shortcut {configKey!r} from {normalized!r} to "
                f"default {defaultStr!r} (conflicts with {seen[normalized]!r})"
            )
            legacy_symbol("NoodlesConfig").set(configKey, defaultStr)
        else:
            seen[normalized] = configKey


def _setupShortcuts(view):
    """Setup shortcuts from configuration."""
    shortcuts = [
        # (name, configKey, checkable, initialCheckedGetter, handler)
        (
            "Toggle Links",
            "shortcutToggleLinks",
            True,
            lambda: view.drawLinks,
            view._toggleDrawLinks,
        ),
        (
            "Toggle Nodes",
            "shortcutToggleNodes",
            True,
            lambda: view.drawNodes,
            view._toggleDrawNodes,
        ),
        (
            "Toggle Text",
            "shortcutToggleText",
            True,
            lambda: view.drawText,
            view._toggleDrawText,
        ),
        (
            "Toggle Link Dimming",
            "shortcutToggleLinkDimming",
            True,
            lambda: view.linkDimming.target > 0.0,
            view._toggleLinkDimming,
        ),
        ("Select All", "shortcutSelectAll", False, None, view.selectAll),
        (
            "Navigate Backward",
            "shortcutNavigateBackward",
            False,
            None,
            view.navigateBackward,
        ),
        (
            "Navigate Forward",
            "shortcutNavigateForward",
            False,
            None,
            view.navigateForward,
        ),
        ("Frame Selection", "shortcutFrameSelection", False, None, view.frameScene),
        ("Frame All", "shortcutFrameAll", False, None, view.frameAll),
        (
            "Frame with Connections",
            "shortcutFrameWithConnections",
            False,
            None,
            view.frameSelectionWithConnections,
        ),
        (
            "Remove from Graph",
            "shortcutRemoveFromGraph",
            False,
            None,
            view.removeSelectedNodes,
        ),
        (
            "Add from Prim Tree",
            "shortcutAddFromPrimTree",
            False,
            None,
            view.addNodesFromPrimTreeSelection,
        ),
    ]

    # Migration: resolve shortcut conflicts. When a default shortcut moves
    # from one action to another (e.g. "D" moved from shortcutToggleLinkDimming
    # to shortcutRemoveFromGraph), a user's persisted value can still claim
    # the key, masking the newer action. Reset any earlier action whose
    # current value collides with a later action's value so the later
    # definition wins.
    view._resolveShortcutConflicts(shortcuts)

    # Store actions for dynamic updates
    view._shortcutActions = {}

    for name, configKey, checkable, checkedGetter, handler in shortcuts:
        # Get shortcut from config (with default fallback already in schema)
        shortcutStr = legacy_symbol("NoodlesConfig").get(configKey, "")

        action = QtGui.QAction(name, view)

        # Only set shortcut if not empty
        if shortcutStr:
            action.setShortcut(QtGui.QKeySequence(shortcutStr))

        action.setShortcutContext(QtCore.Qt.WidgetShortcut)

        if checkable:
            action.setCheckable(True)
            action.setChecked(checkedGetter())

        action.triggered.connect(handler)
        view.addAction(action)

        # Store for dynamic updates
        view._shortcutActions[configKey] = action

    view._saveAction = QtGui.QAction("Save Stage", view)
    view._saveAction.setShortcut(QtGui.QKeySequence.Save)
    view._saveAction.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
    view._saveAction.triggered.connect(view._saveStage)
    view.addAction(view._saveAction)


def updateShortcuts(view):
    """Update all shortcuts from current configuration.
    Called when preferences are applied."""
    if not hasattr(view, "_shortcutActions"):
        return

    for configKey, action in view._shortcutActions.items():
        shortcutStr = legacy_symbol("NoodlesConfig").get(configKey, "")
        if shortcutStr:
            action.setShortcut(QtGui.QKeySequence(shortcutStr))
        else:
            # Clear shortcut if empty string
            action.setShortcut(QtGui.QKeySequence())


def _getContextMenuConf(view):
    return [
        (
            M.SUBMENU,
            "View",
            [
                (
                    M.CHECK,
                    "Show Links",
                    None,
                    view._toggleDrawLinks,
                    view.drawLinks,
                ),
                (
                    M.CHECK,
                    "Show Nodes",
                    None,
                    view._toggleDrawNodes,
                    view.drawNodes,
                ),
                (M.CHECK, "Show Text", None, view._toggleDrawText, view.drawText),
                (
                    M.CHECK,
                    "Show Attribute Values",
                    None,
                    view._toggleShowAttributeValues,
                    view._cachedShowAttributeValues,
                ),
                (M.SEPARATOR,),
                (
                    M.CHECK,
                    "Link Dimming",
                    None,
                    view._toggleLinkDimming,
                    view.linkDimming.target > 0.0,
                ),
            ],
        ),
        (
            M.SUBMENU,
            "Write Values",
            [
                (
                    M.CHECK,
                    "Auto (key only if already animated)",
                    None,
                    lambda: view._setValueWriteMode("auto"),
                    view._cachedValueWriteMode == "auto",
                ),
                (
                    M.CHECK,
                    "Default (the rest value)",
                    None,
                    lambda: view._setValueWriteMode("default"),
                    view._cachedValueWriteMode == "default",
                ),
                (
                    M.CHECK,
                    "Animation (key at current frame)",
                    None,
                    lambda: view._setValueWriteMode("animation"),
                    view._cachedValueWriteMode == "animation",
                ),
            ],
        ),
        (
            M.SUBMENU,
            "Zoom",
            [
                (M.ACTION, "100%", None, lambda: view._setZoom(1.0)),
                (M.ACTION, "50%", None, lambda: view._setZoom(0.5)),
                (M.ACTION, "25%", None, lambda: view._setZoom(0.25)),
                (M.SEPARATOR,),
                (M.ACTION, "Zoom In", None, lambda: view._setZoom(view.zoom * 1.5)),
                (
                    M.ACTION,
                    "Zoom Out",
                    None,
                    lambda: view._setZoom(view.zoom / 1.5),
                ),
            ],
        ),
        (M.SEPARATOR,),
        (M.ACTION, "Frame All (A)", None, view.frameAll),
        (M.ACTION, "Frame Selection (F)", None, view.frameScene),
        (
            M.ACTION,
            "Frame Selection with Connections (Shift+F)",
            None,
            view.frameSelectionWithConnections,
        ),
        (M.ACTION, "Auto Layout (L)", None, view._autoLayoutNodes),
        (M.SEPARATOR,),
        (M.ACTION, "Select All (Ctrl+A)", None, view.selectAll),
        (M.ACTION, "Clear Selection", None, view._clearSelectionAndSync),
        (
            M.ACTION,
            "Delete Selected Nodes (Delete)",
            None,
            view.deleteSelectedNodes,
        ),
        (M.SEPARATOR,),
        (
            M.SUBMENU,
            "Link Selection Mode",
            [
                (
                    M.CHECK,
                    "Nodes Only",
                    None,
                    lambda: view._setLinkSelectionMode(
                        LinkSelectionMode.NODES_ONLY
                    ),
                    view._linkSelectionMode == LinkSelectionMode.NODES_ONLY,
                ),
                (
                    M.CHECK,
                    "With All Links",
                    None,
                    lambda: view._setLinkSelectionMode(
                        LinkSelectionMode.WITH_ALL_LINKS
                    ),
                    view._linkSelectionMode == LinkSelectionMode.WITH_ALL_LINKS,
                ),
                (
                    M.CHECK,
                    "With Input Links",
                    None,
                    lambda: view._setLinkSelectionMode(
                        LinkSelectionMode.WITH_INPUTS
                    ),
                    view._linkSelectionMode == LinkSelectionMode.WITH_INPUTS,
                ),
                (
                    M.CHECK,
                    "With Output Links",
                    None,
                    lambda: view._setLinkSelectionMode(
                        LinkSelectionMode.WITH_OUTPUTS
                    ),
                    view._linkSelectionMode == LinkSelectionMode.WITH_OUTPUTS,
                ),
            ],
        ),
        (M.SEPARATOR,),
        (M.ACTION, "Navigate Backward ([)", None, view.navigateBackward),
        (M.ACTION, "Navigate Forward (])", None, view.navigateForward),
        (M.SEPARATOR,),
        (
            M.CHECK,
            "Frame in Prim Tree on Select",
            None,
            view._toggleFrameInPrimTree,
            view._frameInPrimTreeOnSelect,
        ),
        (M.SEPARATOR,),
        (M.ACTION, "Open Container", None, view._openSelectedContainer),
    ]


def _initContextMenu(view):
    menuConf = view._getContextMenuConf()
    builder = MenuBuilder(view)
    view._contextMenu, view._contextMenuActions = (
        builder.buildContextMenuWithActions(menuConf)
    )

    # Pre-warm the menu to force Qt to cache layout/rendering data
    # This significantly improves the first-time display performance
    # by triggering Qt's internal menu preparation once during initialization
    view._contextMenu.popup(QtCore.QPoint(-10000, -10000))
    view._contextMenu.hide()


def _updateContextMenuState(view):
    actions = view._contextMenuActions

    hasSelection = len(view._selectedNodes) > 0
    hasLinkSelection = len(view._selectedLinks) > 0

    actions["Show Links"].setChecked(view.drawLinks)
    actions["Show Nodes"].setChecked(view.drawNodes)
    actions["Show Text"].setChecked(view.drawText)
    actions["Show Attribute Values"].setChecked(view._cachedShowAttributeValues)
    actions["Auto (key only if already animated)"].setChecked(
        view._cachedValueWriteMode == "auto"
    )
    actions["Default (the rest value)"].setChecked(
        view._cachedValueWriteMode == "default"
    )
    actions["Animation (key at current frame)"].setChecked(
        view._cachedValueWriteMode == "animation"
    )
    actions["Link Dimming"].setChecked(view.linkDimming.target > 0.0)

    actions["Frame Selection (F)"].setEnabled(hasSelection)
    actions["Frame Selection with Connections (Shift+F)"].setEnabled(hasSelection)
    actions["Clear Selection"].setEnabled(hasSelection or hasLinkSelection)

    actions["Nodes Only"].setChecked(
        view._linkSelectionMode == LinkSelectionMode.NODES_ONLY
    )
    actions["With All Links"].setChecked(
        view._linkSelectionMode == LinkSelectionMode.WITH_ALL_LINKS
    )
    actions["With Input Links"].setChecked(
        view._linkSelectionMode == LinkSelectionMode.WITH_INPUTS
    )
    actions["With Output Links"].setChecked(
        view._linkSelectionMode == LinkSelectionMode.WITH_OUTPUTS
    )

    hasLinkOrNodeSelection = hasLinkSelection or hasSelection
    actions["Navigate Backward ([)"].setEnabled(hasLinkOrNodeSelection)
    actions["Navigate Forward (])"].setEnabled(hasLinkOrNodeSelection)

    actions["Frame in Prim Tree on Select"].setChecked(
        view._frameInPrimTreeOnSelect
    )

    # Enable "Open Container" only if exactly one node is selected and it's a container
    canOpenContainer = view._canOpenSelectedContainer()
    actions["Open Container"].setEnabled(canOpenContainer)


def contextMenuEvent(view, event):
    view._updateContextMenuState()
    view._contextMenu.exec_(event.globalPos())


def _openSelectedContainer(view):
    """Open the selected container node in a new Noodles editor window."""
    if not view._canOpenSelectedContainer():
        Tf.Warn("Cannot open container: no single container node selected")
        return

    selectedNodeId = next(iter(view._selectedNodes))
    node = view.nodes[selectedNodeId]

    # Use the node's ID as the prim path (it should be a valid USD path)
    containerPath = node.id

    # Skip virtual nodes (they have # in their path)
    if "#" in containerPath:
        Tf.Warn(f"Cannot open virtual node as container: {containerPath}")
        return

    if not view._usdviewApi:
        Tf.Warn("Cannot open container: usdviewApi not available")
        return

    # Import here to avoid circular dependency
    from UsdNoodles import OpenNewNoodlesEditorForContainer

    try:
        OpenNewNoodlesEditorForContainer(view._usdviewApi, containerPath)
    except Exception as e:
        Tf.Warn(f"Error opening container: {e}")
        import traceback

        traceback.print_exc()


def _openPreferences(view):
    """Open the preferences dialog."""
    if view._preferencesDlg is None:
        view._preferencesDlg = NoodlesPreferences(view, view)
        # Clean up when dialog closes
        view._preferencesDlg.finished.connect(view._onPreferencesClosed)

    view._preferencesDlg.show()
    view._preferencesDlg.raise_()
    view._preferencesDlg.activateWindow()


def _onPreferencesClosed(view):
    """Called when preferences dialog closes."""
    # Keep instance for reuse, but could set to None to recreate each time
    pass


def wheelEvent(view, event):
    # If hotbox is visible, scroll the list instead of zooming
    if view.nodeCreationHotbox.is_visible:
        # Pixels first, degrees second.
        #
        # A macOS trackpad sends high-resolution pixelDelta, and the OS
        # keeps sending events after the fingers lift -- that IS the
        # momentum, so honouring the pixels gives native inertia for
        # free. The old `-angleDelta // 120` threw all of it away: a
        # trackpad's angleDelta arrives in single digits, integer-divided
        # by a 120-unit wheel notch to exactly 0, so nothing moved until
        # enough events had been discarded to cross a notch and the list
        # jumped a whole row. That is the choppiness.
        pixels = event.pixelDelta().y()
        if pixels:
            view.nodeCreationHotbox.handle_scroll_pixels(-float(pixels))
        else:
            # A real wheel (or any device without pixel resolution):
            # 120 units is one notch, three rows, as before.
            degrees = event.angleDelta().y()
            view.nodeCreationHotbox.handle_scroll_pixels(
                -float(degrees) / 120.0 * 3.0
                * view.nodeCreationHotbox.row_height())
        view.update()
        return

    mousePos = event.position()
    delta = event.angleDelta().y()
    # Use multiplicative zoom factor for smoother, bounded zooming
    # Each wheel notch (typically 120 units) zooms by this factor
    zoomFactor = 1.15  # 15% zoom per wheel notch
    steps = delta / 120.0  # Normalize to wheel notches
    newZoom = view.zoom * pow(zoomFactor, steps)
    view._zoomAroundPoint(newZoom, (mousePos.x(), mousePos.y()))
    view.update()


def keyPressEvent(view, event):
    # Value interaction is checked FIRST, ahead of even the Tab/hotbox
    # branch below: Tab moves to the next editable cell while an editor is
    # open, and Esc abandons a drag. Both keys already mean something else
    # in this widget, so whoever is in the middle of an interaction has to
    # be asked before the global meanings get a turn.
    #
    # Esc during a value drag restores the value the drag started from and
    # leaves the undo stack exactly as if the drag had never happened.
    if view._mungState is not None:
        if event.key() == QtCore.Qt.Key_Escape:
            view._cancelMung()
            view.update()
            return

    # The token popup owns the keyboard while it is open.
    if view._tokenPopup:
        if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            view._commitTokenPopup()
        elif event.key() == QtCore.Qt.Key_Escape:
            view._closeTokenPopup()
        elif event.key() == QtCore.Qt.Key_Up:
            view._tokenPopupList.move_selection(-1)
        elif event.key() == QtCore.Qt.Key_Down:
            view._tokenPopupList.move_selection(1)
        view.update()
        return

    # A value editor owns ALL of the keyboard, for the same reason the
    # rename below does: the shortcuts further down are single letters,
    # and typing 0.5 into a cell must not toggle link dimming.
    if view._valueEditTarget:
        key = event.key()
        if key in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            view._commitValueEdit()
        elif key == QtCore.Qt.Key_Escape:
            view._cancelValueEdit()
        elif key in (QtCore.Qt.Key_Tab, QtCore.Qt.Key_Backtab):
            backwards = key == QtCore.Qt.Key_Backtab or bool(
                event.modifiers() & QtCore.Qt.ShiftModifier
            )
            view._advanceValueEdit(backwards)
        else:
            if (
                view._valueEditFresh
                and event.text()
                and event.text().isprintable()
            ):
                # Emulates the select-all a real line edit does on focus.
                view._valueEditInput.clear()
            view._valueEditFresh = False
            view._valueEditInput.handle_key(event)
        view.update()
        return

    # Tab key - toggle hotbox
    if event.key() == QtCore.Qt.Key_Tab:
        if view.nodeCreationHotbox.is_showing:
            view.nodeCreationHotbox.hide()
        else:
            view.nodeCreationHotbox.show(view.lastMousePos)
        view.update()
        return

    # When hotbox is showing, delegate key handling to it
    if view.nodeCreationHotbox.is_showing:
        if view.nodeCreationHotbox.handle_key(event):
            view.update()
            return

    # A rename in progress owns the keyboard, and owns ALL of it: the
    # shortcuts below are single letters, so letting an unhandled key fall
    # through would make typing a name called Z undo the graph.
    if view._renamingNodeId:
        if event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            view._commitRename()
        elif event.key() == QtCore.Qt.Key_Escape:
            view._cancelRename()
        else:
            view._renameInput.handle_key(event)
        view.update()
        return

    if event.key() == QtCore.Qt.Key_Space:
        view.spacePressed = True
        view.setCursor(QtCore.Qt.OpenHandCursor)
    elif event.key() == QtCore.Qt.Key_Z:
        modifiers = event.modifiers()
        if modifiers & QtCore.Qt.ControlModifier:
            if modifiers & QtCore.Qt.ShiftModifier:
                view._performRedo()
            else:
                view._performUndo()
            return
    elif event.key() == QtCore.Qt.Key_Y:
        if event.modifiers() & QtCore.Qt.ControlModifier:
            view._performRedo()
            return
    # 1-9 for selecting links by pin index
    # Shift+number reverses the direction (inputs instead of outputs, or vice versa)
    elif event.key() >= QtCore.Qt.Key_1 and event.key() <= QtCore.Qt.Key_9:
        pinIndex = event.key() - QtCore.Qt.Key_1  # 0-based index
        reverseDirection = bool(event.modifiers() & QtCore.Qt.ShiftModifier)
        view.selectLinkByPinIndex(pinIndex, reverseDirection)

    # Delete or Backspace: Delete all selected items at once
    elif event.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
        hasLinks = bool(view._selectedLinks)
        hasNodes = bool(view._selectedNodes)
        if hasLinks or hasNodes:
            registry = getattr(view, "_commandRegistry", None)
            if registry is None:
                view._deleteAllSelected()
            else:
                registry.dispatch(_intents.DeleteSelection())
        return

    # L: auto-layout the whole graph (plain key, no Ctrl/Alt/Meta).
    elif event.key() == QtCore.Qt.Key_L and not (
        event.modifiers()
        & (
            QtCore.Qt.ControlModifier
            | QtCore.Qt.AltModifier
            | QtCore.Qt.MetaModifier
        )
    ):
        view._autoLayoutNodes()
        event.accept()
        return

    # Fallback: dispatch to configurable shortcut QActions.
    # _shouldInterceptShortcut intercepts ShortcutOverride for all keys
    # that match a configured shortcut, which bypasses Qt's shortcut
    # system. QAction shortcuts don't fire in that case, so we manually
    # match and trigger them here.
    if hasattr(view, "_shortcutActions"):
        event_key = int(event.key()) | (
            event.modifiers().value & ~QtCore.Qt.KeypadModifier.value
        )
        for _configKey, action in view._shortcutActions.items():
            shortcut = action.shortcut()
            if shortcut.isEmpty():
                continue
            combo = shortcut[0]
            combo_int = (
                combo.toCombined() if hasattr(combo, "toCombined") else int(combo)
            )
            if combo_int == event_key:
                action.trigger()
                event.accept()
                return


def keyReleaseEvent(view, event):
    if view._renamingNodeId:
        return
    if event.key() == QtCore.Qt.Key_Space:
        view.spacePressed = False
        view.setCursor(QtCore.Qt.ArrowCursor)


def _shouldInterceptShortcut(view, event):
    """Check if a ShortcutOverride event should be intercepted by the graph view.

    Prevents usdview's ApplicationShortcut-scoped actions (e.g. 'J' for
    Toggle Framed View) from firing while Noodles has focus.
    """
    # When the hotbox is showing, intercept ALL keys so they reach
    # the text input instead of triggering usdview shortcuts
    if view.nodeCreationHotbox.is_showing:
        return True

    # Tab is always intercepted (toggles the hotbox). Tab is not a
    # printable character so the check below won't catch it.
    if event.key() == QtCore.Qt.Key_Tab:
        return True

    # An interaction that owns the keyboard owns its NON-PRINTABLE keys
    # too. Without this, Qt takes the ShortcutOverride we declined as
    # permission to route the key elsewhere -- Escape goes to the
    # enclosing QDockWidget and the KeyPress never reaches
    # keyPressEvent at all, so "Esc cancels" silently does nothing. The
    # printable-key branch below cannot cover these: none of them
    # produce text.
    if (
        view._renamingNodeId
        or view._valueEditTarget
        or view._mungState is not None
        or view._tokenPopup
    ):
        if event.key() in (
            QtCore.Qt.Key_Escape,
            QtCore.Qt.Key_Return,
            QtCore.Qt.Key_Enter,
            QtCore.Qt.Key_Backtab,
            QtCore.Qt.Key_Backspace,
            QtCore.Qt.Key_Delete,
            QtCore.Qt.Key_Up,
            QtCore.Qt.Key_Down,
            QtCore.Qt.Key_Left,
            QtCore.Qt.Key_Right,
            QtCore.Qt.Key_Home,
            QtCore.Qt.Key_End,
        ):
            return True

    # Intercept unmodified (or Shift-only) printable keys to prevent
    # usdview's ApplicationShortcut-scoped actions from firing while
    # Noodles has focus. Modifier combos (Ctrl+X, Alt+X) are left
    # alone so they can still reach usdview or QAction shortcuts,
    # UNLESS they match a configured Noodles shortcut.
    modifiers = event.modifiers() & ~QtCore.Qt.KeypadModifier
    if modifiers in (QtCore.Qt.NoModifier, QtCore.Qt.ShiftModifier):
        text = event.text()
        if text and text.isprintable():
            return True

    # Intercept keys that match configurable Noodles shortcuts (including
    # Ctrl/Alt combos) so they reach keyPressEvent instead of being
    # captured by usdview's ApplicationShortcut-scoped QActions.
    if hasattr(view, "_shortcutActions"):
        event_key = int(event.key()) | (
            event.modifiers().value & ~QtCore.Qt.KeypadModifier.value
        )
        for action in view._shortcutActions.values():
            shortcut = action.shortcut()
            if shortcut.isEmpty():
                continue
            combo = shortcut[0]
            combo_int = (
                combo.toCombined() if hasattr(combo, "toCombined") else int(combo)
            )
            if combo_int == event_key:
                return True

    return False


def event(view, event):
    if event.type() == QtCore.QEvent.Paint:
        if view._isStageViewMidTeardown():
            return True
    if event.type() == QtCore.QEvent.ShortcutOverride:
        if view._shouldInterceptShortcut(event):
            event.accept()
            return True
    if event.type() == QtCore.QEvent.HoverMove:
        view.lastMousePos = event.position().toPoint()
        view._updateNodeUnderCursor(view.lastMousePos)
        if not view.hasFocus():
            view.setFocus()
        view.update()
    try:
        return QGLWidget.event(view, event)
    except RuntimeError as e:
        Tf.Warn(f"RuntimeError in event handling: {e}")
        return False


def _showPinSelectionMenu(view, targetNodeId, pins, targetIsOutput):
    """Show a context menu to select which pin to connect to on a collapsed node."""
    menu = legacy_symbol("QtWidgets").QMenu(view)
    menu.setTitle("Select Pin")
    for pin in pins:
        action = menu.addAction(pin)
        action.setData(pin)

    def onPinSelected(action):
        selectedPin = action.data()
        if targetIsOutput:
            outputNodeId, outputPortName = targetNodeId, selectedPin
            inputNodeId, inputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
        else:
            inputNodeId, inputPortName = targetNodeId, selectedPin
            outputNodeId, outputPortName = (
                view._dragLinkSourceNode,
                view._dragLinkSourcePort,
            )
        if view._reconnectingLink and (
            outputNodeId == view._reconnectOldSourceNodeId
            and outputPortName == view._reconnectOldSourcePort
            and inputNodeId == view._reconnectOldTargetNodeId
            and inputPortName == view._reconnectOldTargetPort
        ):
            view._cancelLinkReconnect()
            return
        view._createConnection(
            outputNodeId, outputPortName, inputNodeId, inputPortName
        )

    menu.triggered.connect(onPinSelected)
    if view._reconnectingLink:
        from PySide6 import QtCore

        menu.aboutToHide.connect(
            lambda: QtCore.QTimer.singleShot(
                0,
                lambda: (
                    view._finishReconnectDisconnect()
                    if view._reconnectingLink
                    else None
                ),
            )
        )
    menu.popup(view.mapToGlobal(view.lastMousePos))


def _onValueCellPressed(view, hit, event):
    """A left press landed on a value cell. Decide what it means."""
    row = hit.row
    if not row.editable:
        # Read-only is answered, not ignored: silence here reads as a
        # broken cell. It also must NOT fall through to a connection drag.
        view._showPopupMessage("%s: %s" % (row.property_name, row.reason))
        return
    if row.kind == noodlesValues.KIND_BOOL:
        view._toggleBoolValue(hit)
        return
    if row.kind == noodlesValues.KIND_TOKEN_ENUM:
        view._openTokenPopup(hit)
        return

    # Numeric or text: nothing is authored yet. The release decides
    # whether this was a drag (mung) or a click (keyboard editor).
    mungable = bool(row.mungable)
    base = row.components[hit.comp] if mungable else 0.0
    view._mungState = {
        "nodeId": hit.nodeId,
        "primPath": hit.primPath,
        "prop": hit.propertyName,
        "comp": hit.comp,
        "baseScreenX": float(event.position().x()),
        "original": tuple(row.components),
        "current": tuple(row.components),
        "base": float(base) if mungable else 0.0,
        "step": (
            noodlesValues.ladder_step(base, row.type_name) if mungable else 0.0
        ),
        "typeName": row.type_name,
        "time": view._currentTimeCode,
        "mungable": mungable,
    }
    view._mungActive = False


def _openTokenPopup(view, hit):
    """A token with ``allowedTokens``: pick from exactly those, nothing else."""
    tokens = list(hit.row.tokens)
    if not tokens:
        view._beginValueEdit(hit.nodeId, hit.propertyName, 0)
        return
    # The popup wears the node's clothes: same fill, same accent for the
    # current value, same text colours. A native menu here would be the
    # one part of the interaction that did not belong to the graph.
    theme = view._cachedValueTheme
    view._tokenPopupList.font_size = 15.0
    view._tokenPopupList.background_color = theme.popupBg
    view._tokenPopupList.item_color = theme.textNormal
    view._tokenPopupList.selected_color = theme.popupRow
    view._tokenPopupList.set_items(
        [SelectableListItem(token) for token in tokens]
    )
    current = hit.row.components[0] if hit.row.components else ""
    view._tokenPopupList.selected_index = (
        tokens.index(current) if current in tokens else 0
    )
    view._tokenPopupList.scroll_selection_into_view()
    view._tokenPopup = {
        "nodeId": hit.nodeId,
        "primPath": hit.primPath,
        "prop": hit.propertyName,
        "tokens": tuple(tokens),
        "rect": hit.rect,
        "time": view._currentTimeCode,
        "original": tuple(hit.row.components),
    }
    view.update()


def _closeTokenPopup(view):
    view._tokenPopup = None
    view.update()


def _handleTokenPopupPress(view, screenPos):
    """Inside the popup picks a token; anywhere else just closes it."""
    bounds = view._tokenPopupList.bounds
    if bounds:
        x, y, width, height = bounds
        if x <= screenPos.x() <= x + width and y <= screenPos.y() <= y + height:
            index = view._tokenPopupList.item_index_at_pixel(screenPos.y() - y)
            if index is not None:
                view._tokenPopupList.selected_index = index
                view._commitTokenPopup()
                return
    view._closeTokenPopup()


def focusNextPrevChild(view, nextChild):
    """Let keyPressEvent use Tab for creation and inline value editing."""
    return False


def mouseDoubleClickEvent(self, event):
    """Handle double-click events.

    Double-clicking on a dangling link or on a connected port whose
    connected node is missing adds the missing node to the graph.
    Double-clicking a non-dangling relationship link jumps to the
    endpoint farthest from the cursor so users can follow links to
    connected nodes that lie outside the current viewport.
    """
    if event.button() != QtCore.Qt.LeftButton:
        return
    if _gestures.double_click_gesture(self, event):
        return
    # Default behavior for double-click on non-dangling elements
    QGLWidget.mouseDoubleClickEvent(self, event)


def mousePressEvent(self, event):
    if _gestures.press_intercept_overlays(self, event):
        return

    if event.buttons() & QtCore.Qt.LeftButton and not self.spacePressed:
        if _gestures.press_title_caret_toggle(self, event):
            return
        if _gestures.press_group_fold_toggle(self, event):
            return
        if _gestures.press_value_cell(self, event):
            return
        if _gestures.press_hovered_port_link_start(self, event):
            return
        if _gestures.press_arrowhead_reconnect(self, event):
            return
        if _gestures.press_row_edge_link_start(self, event):
            return
        _gestures.press_node_or_empty(self, event)

    self.update()


def mouseMoveEvent(self, event):
    if _gestures.move_intercept_overlays(self, event):
        return

    # During an interactive pan or pan-zoom drag (middle-button, or
    # space + left-button), skip all hover/hit-testing entirely. The cursor
    # is not hovering content during a viewport drag, and these per-move
    # scans (node-under-cursor, hovered port, property tooltip, and link
    # hit-testing) are the dominant cost on large scenes -- they are why
    # panning feels heavier than zooming. The pan/zoom itself is handled
    # below; this mirrors the node-drag and link-drag branches, which also
    # bypass hover updates.
    isViewportDrag = bool(event.buttons() & QtCore.Qt.MiddleButton) or (
        self.spacePressed and bool(event.buttons() & QtCore.Qt.LeftButton)
    )

    if _gestures.move_value_drag(self, event):
        return

    _gestures.move_update_hover(self, event, isViewportDrag)
    if _gestures.move_link_drag(self, event):
        return
    if _gestures.move_viewport_drag(self, event):
        return
    if _gestures.move_left_drag(self, event):
        return

    self.lastMousePos = event.position().toPoint()
    self.update()


def mouseReleaseEvent(self, event):
    if _gestures.release_intercept_overlays(self, event):
        return
    if _gestures.release_link_drag(self, event):
        return
    if _gestures.release_minimap(self, event):
        return
    _gestures.release_node_drag_and_marquee(self, event)
