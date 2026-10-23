#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Test production popup wrapping and timer behavior without a GL context."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock


from UsdNoodles.widgets.hotkeyPopup import HotkeyPopup


# --- Test helpers ---------------------------------------------------------


def _char_width(width_per_char):
    """Width function: every character is *width_per_char* px wide."""

    def _f(text, _font_size):
        return len(text) * width_per_char

    return _f


def _make_renderer(width_func):
    renderer = MagicMock()
    renderer.calculateTextWidth.side_effect = width_func
    return renderer


# --- Tests ----------------------------------------------------------------


class WrapMessageTest(unittest.TestCase):
    """Exercise the static greedy word-wrap helper directly."""

    # (label, message, max_width, expected_lines)
    # All cases use _char_width(10) so each character is 10 px.
    CASES = [
        # Single line that fits — no wrapping.
        ("single_line_fits", "hello", 1000, ["hello"]),
        # 7-char "aaa bbb" exceeds 50px, so each 3-char word emits alone.
        ("wraps_on_word_boundary", "aaa bbb ccc", 50, ["aaa", "bbb", "ccc"]),
        # Greedy packing: "aa bb cc" fits at 80 px, "dd" overflows.
        ("greedy_packs_multiple_words", "aa bb cc dd", 80, ["aa bb cc", "dd"]),
        # Embedded \n MUST split regardless of fit.
        ("explicit_newline_splits", "line1\nline2", 10000, ["line1", "line2"]),
        # \n\n splits to two empty wrapped lines.
        ("only_newlines_emits_empty_lines", "\n\n", 1000, ["", ""]),
        # Oversized word: 20-char "supercalifragilistic" (200 px) blows the
        # 50 px cap. Contract: words too wide to fit are emitted on their own
        # line rather than dropped, so no content is lost.
        (
            "oversized_word_on_own_line",
            "tiny supercalifragilistic word",
            50,
            ["tiny", "supercalifragilistic", "word"],
        ),
    ]

    def test_table_driven_wrapping(self):
        for label, message, max_width, expected in self.CASES:
            with self.subTest(case=label):
                renderer = _make_renderer(_char_width(10))
                lines = HotkeyPopup._wrap_message(
                    message, renderer, font_size=12, max_width=max_width
                )
                self.assertEqual(lines, expected)

    def test_empty_message_returns_single_empty_line_without_measuring(self):
        # Empty message must NOT return [] — render() relies on a non-empty
        # list to decide whether to draw. The contract is "one empty line"
        # for an empty message. Width measurement must NOT be called because
        # there's nothing to measure.
        renderer = _make_renderer(_char_width(10))
        lines = HotkeyPopup._wrap_message("", renderer, font_size=12, max_width=1000)
        self.assertEqual(lines, [""])
        renderer.calculateTextWidth.assert_not_called()


class ShowDurationMsTest(unittest.TestCase):
    """Verify the new ``duration_ms`` override on ``HotkeyPopup.show()``.

    The warning-popup path passes a longer timer (1400 ms) so users have
    time to read the toast; navigation popups still use the default 400 ms.
    """

    def _make_popup(self):
        parent = MagicMock()
        parent.startTimer.return_value = 42
        popup = HotkeyPopup(MagicMock())
        popup.set_parent_widget(parent)
        cursor_pos = MagicMock()
        return popup, parent, cursor_pos

    # Each case calls popup.show(...) once and asserts the timer duration
    # passed to startTimer.
    # (label, kwargs_for_show, expected_timer_duration)
    DURATION_CASES = [
        # No argument → falls back to popup._display_duration_ms (400 ms).
        ("default_when_omitted", {}, 400),
        # Warning path → 1400 ms toast.
        ("custom_overrides_default", {"duration_ms": 1400}, 1400),
        # Regression guard: implementation uses ``is not None`` rather than
        # truthiness, so a literal 0 must be passed through. A naive ``or``
        # refactor would break this case.
        ("explicit_zero_honored", {"duration_ms": 0}, 0),
    ]

    def test_table_driven_duration_selection(self):
        for label, kwargs, expected_duration in self.DURATION_CASES:
            with self.subTest(case=label):
                popup, parent, cursor_pos = self._make_popup()

                popup.show("msg", cursor_pos, **kwargs)

                parent.startTimer.assert_called_once_with(expected_duration)

    def test_existing_timer_killed_before_new_timer_starts(self):
        # show() landing while a previous timer is alive must kill the old
        # timer so the popup doesn't get a stale dismiss event from the
        # previous show().
        popup, parent, cursor_pos = self._make_popup()

        # First show — installs timer id 42.
        popup.show("first", cursor_pos)
        parent.startTimer.return_value = 99  # next startTimer() call.

        # Second show — must killTimer(42) before startTimer(...) again.
        popup.show("second", cursor_pos, duration_ms=1400)

        parent.killTimer.assert_called_once_with(42)
        self.assertEqual(parent.startTimer.call_count, 2)
        self.assertEqual(parent.startTimer.call_args_list[-1][0][0], 1400)
