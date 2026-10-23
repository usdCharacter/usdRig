#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Unit tests for KeyBindingDialog conflict detection.

Tests the find_shortcut_conflicts logic that detects duplicate key sequences
across remappable shortcuts and hardcoded non-remappable keys.

Calls the production conflict detector without constructing the dialog.
"""

import unittest


from UsdNoodles.widgets.keyBindingDialog import (
    NON_REMAPPABLE_KEYS as _NON_REMAPPABLE_KEYS,
    find_shortcut_conflicts as _find_shortcut_conflicts,
)


class TestFindShortcutConflicts(unittest.TestCase):
    def test_no_conflicts(self):
        bindings = [
            ("k1", "Toggle Links", "Ctrl+Shift+L"),
            ("k2", "Toggle Nodes", "Ctrl+Shift+N"),
        ]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertEqual(result, {})

    def test_two_actions_same_key(self):
        bindings = [
            ("k1", "Toggle Links", "Ctrl+Shift+L"),
            ("k2", "Toggle Nodes", "Ctrl+Shift+L"),
        ]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertIn("Ctrl+Shift+L", result)
        self.assertEqual(len(result["Ctrl+Shift+L"]), 2)

    def test_conflict_with_hardcoded_tab(self):
        bindings = [("k1", "Toggle Links", "Tab")]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertIn("Tab", result)
        self.assertIn("(hardcoded: Tab)", result["Tab"])

    def test_empty_key_ignored(self):
        bindings = [("k1", "Toggle Links", ""), ("k2", "Toggle Nodes", "")]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertEqual(result, {})

    def test_conflict_with_delete(self):
        bindings = [("k1", "Select All", "Delete")]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertIn("Delete", result)

    def test_conflict_with_number_key(self):
        bindings = [("k1", "Frame Selection", "5")]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertIn("5", result)

    def test_three_way_conflict(self):
        bindings = [
            ("k1", "Action A", "F"),
            ("k2", "Action B", "F"),
            ("k3", "Action C", "F"),
        ]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertEqual(len(result["F"]), 3)

    def test_no_conflict_different_keys(self):
        bindings = [
            ("k1", "Action A", "A"),
            ("k2", "Action B", "B"),
            ("k3", "Action C", "C"),
        ]
        result = _find_shortcut_conflicts(bindings, _NON_REMAPPABLE_KEYS)
        self.assertEqual(result, {})
