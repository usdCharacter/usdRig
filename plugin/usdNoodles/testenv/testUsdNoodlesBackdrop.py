#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Test GroupSticker.from_prim against real authored backdrop attributes."""

import unittest

from pxr import Gf, Usd, UsdUI
from UsdNoodles.widgets.groupSticker import (
    GroupSticker, DEFAULT_COLOR, DEFAULT_CORNER_RADIUS, DEFAULT_INNER_STROKE,
)


class TestGroupStickerFromPrim(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.backdrop = UsdUI.Backdrop.Define(self.stage, "/Backdrop")
        self.prim = self.backdrop.GetPrim()
        self.api = UsdUI.NodeGraphNodeAPI.Apply(self.prim)

    def test_position_and_size_scale(self):
        self.api.CreatePosAttr().Set(Gf.Vec2f(1.5, 2.0))
        self.api.CreateSizeAttr().Set(Gf.Vec2f(0.5, 0.3))
        sticker = GroupSticker.from_prim(self.prim)
        self.assertEqual(sticker.position, Gf.Vec2d(1500, 2000))
        self.assertAlmostEqual(sticker.size[0], 500, places=3)
        self.assertAlmostEqual(sticker.size[1], 300, places=3)

    def test_description_and_color(self):
        self.backdrop.CreateDescriptionAttr().Set("My Sticker Group")
        self.api.CreateDisplayColorAttr().Set(Gf.Vec3f(0.5, 0.3, 0.8))
        sticker = GroupSticker.from_prim(self.prim)
        self.assertEqual(sticker.name, "My Sticker Group")
        for actual, expected in zip(sticker.color, (0.5, 0.3, 0.8, 1.0)):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertEqual(sticker._prim, self.prim)

    def test_unauthored_defaults(self):
        sticker = GroupSticker.from_prim(self.prim)
        self.assertEqual(sticker.name, "")
        self.assertEqual(sticker.position, Gf.Vec2d(0, 0))
        self.assertEqual(sticker.size, Gf.Vec2d(100, 100))
        self.assertEqual(sticker.color, DEFAULT_COLOR)
        self.assertEqual(sticker.cornerRadius, DEFAULT_CORNER_RADIUS)
        self.assertEqual(sticker.innerStroke, DEFAULT_INNER_STROKE)

    def test_custom_display_properties(self):
        for name, value in (("cornerRadius", 15.0), ("innerStroke", 3.0), ("alpha", 120)):
            self.prim.SetCustomDataByKey(name, value)
        sticker = GroupSticker.from_prim(self.prim)
        self.assertEqual(sticker.cornerRadius, 15.0)
        self.assertEqual(sticker.innerStroke, 3.0)
        self.assertEqual(sticker.alpha, 120)


if __name__ == "__main__":
    unittest.main()
