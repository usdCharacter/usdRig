#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Unit tests for GraphView._shouldInterceptShortcut.

Tests every branch of the shortcut interception logic that prevents
usdview's ApplicationShortcut-scoped actions (e.g. 'J' for Toggle
Framed View) from firing while Noodles has focus.

Production methods run against lightweight view and settings doubles.
"""

import unittest
from unittest.mock import MagicMock, patch

from pxr.Usdviewq.qt import QtCore
from UsdNoodles.graphView import GraphView

_KEY_TAB = QtCore.Qt.Key_Tab
_KEY_A = QtCore.Qt.Key_A
_KEY_J = QtCore.Qt.Key_J
_KEY_X = QtCore.Qt.Key_X
_KEY_F = QtCore.Qt.Key_F
_KEY_5 = QtCore.Qt.Key_5
_KEY_F1 = QtCore.Qt.Key_F1
_KEY_ESCAPE = QtCore.Qt.Key_Escape
_KEY_RETURN = QtCore.Qt.Key_Return

_NO_MODIFIER = QtCore.Qt.NoModifier
_SHIFT_MODIFIER = QtCore.Qt.ShiftModifier
_CTRL_MODIFIER = QtCore.Qt.ControlModifier
_ALT_MODIFIER = QtCore.Qt.AltModifier
_KEYPAD_MODIFIER = QtCore.Qt.KeypadModifier

_should_intercept_shortcut = GraphView._shouldInterceptShortcut


def _make_event(key, modifiers=_NO_MODIFIER, text=""):
    """Create a mock QKeyEvent with the given key, modifiers, and text."""
    event = MagicMock()
    event.key.return_value = key
    event.modifiers.return_value = modifiers
    event.text.return_value = text
    return event


def _make_view(hotbox_showing=False):
    """Create a minimal mock standing in for GraphView."""
    view = MagicMock(spec=[])
    view.nodeCreationHotbox = MagicMock()
    view.nodeCreationHotbox.is_showing = hotbox_showing
    view._renamingNodeId = ""
    view._valueEditTarget = None
    view._mungState = None
    view._tokenPopup = None
    view._shortcutActions = {}
    return view


class TestShouldInterceptShortcut(unittest.TestCase):
    """Tests for GraphView._shouldInterceptShortcut branch coverage.

    Branch map (graphView.py:3745-3771):
      1. hotbox showing  → True
      2. Tab key          → True
      3. no-mod printable → True
      4. Shift+printable  → True
      5. Ctrl/Alt combo   → False
      6. non-printable    → False
      7. KeypadModifier   → stripped, then re-evaluated
    """

    # -- Branch 1: hotbox showing → intercept ALL keys --

    def test_hotbox_showing_intercepts_printable_key(self):
        view = _make_view(hotbox_showing=True)
        event = _make_event(_KEY_J, text="j")
        self.assertTrue(_should_intercept_shortcut(view, event))

    def test_hotbox_showing_intercepts_modifier_combo(self):
        view = _make_view(hotbox_showing=True)
        event = _make_event(_KEY_X, _CTRL_MODIFIER)
        self.assertTrue(_should_intercept_shortcut(view, event))

    # -- Branch 2: Tab always intercepted --

    def test_tab_intercepted_when_hotbox_hidden(self):
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_TAB)
        self.assertTrue(_should_intercept_shortcut(view, event))

    # -- Branch 3: unmodified printable key intercepted --

    def test_unmodified_printable_key_intercepted(self):
        """'j' with no modifier — prevents usdview 'J' shortcut."""
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_J, _NO_MODIFIER, text="j")
        self.assertTrue(_should_intercept_shortcut(view, event))

    # -- Branch 4: Shift + printable key intercepted --

    def test_shift_printable_key_intercepted(self):
        """Shift+A producing 'A' is intercepted."""
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_A, _SHIFT_MODIFIER, text="A")
        self.assertTrue(_should_intercept_shortcut(view, event))

    # -- Branch 5: Ctrl / Alt combos NOT intercepted --

    def test_ctrl_key_not_intercepted(self):
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_X, _CTRL_MODIFIER)
        self.assertFalse(_should_intercept_shortcut(view, event))

    def test_alt_key_not_intercepted(self):
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_F, _ALT_MODIFIER)
        self.assertFalse(_should_intercept_shortcut(view, event))

    def test_ctrl_shift_not_intercepted(self):
        """Ctrl+Shift+A must pass through to usdview."""
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_A, _CTRL_MODIFIER | _SHIFT_MODIFIER)
        self.assertFalse(_should_intercept_shortcut(view, event))

    # -- Branch 6: non-printable keys without modifiers NOT intercepted --

    def test_escape_not_intercepted(self):
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_ESCAPE)
        self.assertFalse(_should_intercept_shortcut(view, event))

    def test_function_key_not_intercepted(self):
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_F1)
        self.assertFalse(_should_intercept_shortcut(view, event))

    def test_return_key_not_intercepted(self):
        r"""Return key text ('\r') is not printable — returns False."""
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_RETURN, _NO_MODIFIER, text="\r")
        self.assertFalse(_should_intercept_shortcut(view, event))

    # -- Branch 7: KeypadModifier stripped before check --

    def test_keypad_modifier_stripped_printable_intercepted(self):
        """Numpad '5' has KeypadModifier but should still be intercepted."""
        view = _make_view(hotbox_showing=False)
        event = _make_event(_KEY_5, _KEYPAD_MODIFIER, text="5")
        self.assertTrue(_should_intercept_shortcut(view, event))


def _resolve_shortcut_conflicts(shortcuts, settings):
    entries = [(name, key, False, None, None) for name, key in shortcuts]
    with patch("UsdNoodles.graphView.NoodlesConfig") as config:
        config.settings.return_value = settings
        if settings is not None:
            config.get.side_effect = settings.get
            config.set.side_effect = settings.set
        GraphView._resolveShortcutConflicts(_make_view(), entries)


class _FakeSettings:
    """Minimal fake of NoodlesConfig that records set() calls."""

    def __init__(self, values, defaults):
        self._values = dict(values)
        self._defaults = dict(defaults)
        self.set_calls = []

    def get(self, key, fallback=""):
        return self._values.get(key, fallback)

    def set(self, key, value):
        self._values[key] = value
        self.set_calls.append((key, value))

    def getSettingInfo(self, key):
        if key not in self._defaults:
            return None
        return {"name": key, "default": self._defaults[key]}



class TestResolveShortcutConflicts(unittest.TestCase):
    """Tests for GraphView._resolveShortcutConflicts migration logic."""

    def test_no_settings_is_noop(self):
        # Should not raise when settings is None.
        _resolve_shortcut_conflicts([("Toggle", "shortcutToggle")], None)

    def test_no_conflict_leaves_values_alone(self):
        settings = _FakeSettings(
            values={"shortcutA": "D", "shortcutB": "G"},
            defaults={"shortcutA": "D", "shortcutB": "G"},
        )
        shortcuts = [("A", "shortcutA"), ("B", "shortcutB")]
        _resolve_shortcut_conflicts(shortcuts, settings)
        self.assertEqual(settings.set_calls, [])

    def test_conflict_resets_earlier_to_default(self):
        # User has shortcutToggle = "D" but the new shortcutRemove default is
        # also "D". The earlier entry should be reset to its new default "G".
        settings = _FakeSettings(
            values={"shortcutToggle": "D", "shortcutRemove": "D"},
            defaults={"shortcutToggle": "G", "shortcutRemove": "D"},
        )
        shortcuts = [
            ("Toggle Link Dimming", "shortcutToggle"),
            ("Remove from Graph", "shortcutRemove"),
        ]
        _resolve_shortcut_conflicts(shortcuts, settings)
        self.assertEqual(settings.set_calls, [("shortcutToggle", "G")])

    def test_conflict_skipped_when_default_would_not_free_key(self):
        # If resetting to default would keep the same key, skip to avoid
        # looping the same conflict on every launch.
        settings = _FakeSettings(
            values={"shortcutA": "D", "shortcutB": "D"},
            defaults={"shortcutA": "D", "shortcutB": "D"},
        )
        shortcuts = [("A", "shortcutA"), ("B", "shortcutB")]
        _resolve_shortcut_conflicts(shortcuts, settings)
        self.assertEqual(settings.set_calls, [])

    def test_empty_shortcut_is_ignored(self):
        # Empty stored value (user cleared the shortcut) is never a conflict.
        settings = _FakeSettings(
            values={"shortcutA": "", "shortcutB": "D"},
            defaults={"shortcutA": "G", "shortcutB": "D"},
        )
        shortcuts = [("A", "shortcutA"), ("B", "shortcutB")]
        _resolve_shortcut_conflicts(shortcuts, settings)
        self.assertEqual(settings.set_calls, [])

    def test_latest_definition_wins_across_three_entries(self):
        # Three entries all claim "D"; only the latest should keep it, the
        # other two get reset to their respective defaults.
        settings = _FakeSettings(
            values={"shortcutA": "D", "shortcutB": "D", "shortcutC": "D"},
            defaults={"shortcutA": "X", "shortcutB": "Y", "shortcutC": "D"},
        )
        shortcuts = [("A", "shortcutA"), ("B", "shortcutB"), ("C", "shortcutC")]
        _resolve_shortcut_conflicts(shortcuts, settings)
        self.assertEqual(
            sorted(settings.set_calls),
            sorted([("shortcutA", "X"), ("shortcutB", "Y")]),
        )
