#!/usr/bin/env python3
"""Tests for vfx_profile.py (stdlib unittest; run directly)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vfx_profile  # noqa: E402

BUILD_USD = '''
ONETBB_URL = "https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.12.0.zip"
OPENSUBDIV_URL = "https://github.com/PixarAnimationStudios/OpenSubdiv/archive/v3_6_1.zip"
'''

REQUIRED = {
    "cxx_standard", "python", "pyside6", "numpy", "onetbb_tag",
    "opensubdiv_tag", "macos_deployment_target", "linux_image",
    "linux_glibc_max", "linux_glibcxx_max", "linux_baseline",
    "msvc_toolset_min", "windows_sdk_min",
}


class ProfileTest(unittest.TestCase):
    def setUp(self):
        self.data = vfx_profile.load()

    def test_every_year_has_every_pin(self):
        for year, profile in self.data["years"].items():
            missing = REQUIRED - set(profile["build"])
            self.assertFalse(missing, f"CY{year} lacks {sorted(missing)}")

    def test_outputs_accept_prefixes_and_hash_pins(self):
        a = vfx_profile.outputs(self.data, "2026")
        b = vfx_profile.outputs(self.data, "cy2026")
        self.assertEqual(a, b)
        self.assertEqual(a["year"], "2026")
        self.assertEqual(len(a["profile_hash"]), 12)
        self.assertNotEqual(a["profile_hash"],
                            vfx_profile.outputs(self.data, "2025")["profile_hash"])

    def test_unknown_year_fails(self):
        with self.assertRaises(SystemExit):
            vfx_profile.outputs(self.data, "1999")

    def test_pin_build_usd(self):
        text = vfx_profile.pin_build_usd(BUILD_USD, "v2023.2.0", "v3_8_0")
        self.assertIn("uxlfoundation/oneTBB/archive/refs/tags/v2023.2.0.zip",
                      text)
        self.assertIn("OpenSubdiv/archive/v3_8_0.zip", text)
        self.assertNotIn("v2021.12.0", text)

    def test_pin_build_usd_fails_when_layout_changes(self):
        with self.assertRaises(SystemExit):
            vfx_profile.pin_build_usd("ONETBB_URL = 'x'\n", "v1", "v2")

    def test_notes_table(self):
        notes = vfx_profile.notes(self.data)
        years = sorted(self.data["years"])
        self.assertIn("| | " + " | ".join(f"CY{y}" for y in years) + " |",
                      notes)
        self.assertIn("| OpenUSD |", notes)


if __name__ == "__main__":
    unittest.main()
