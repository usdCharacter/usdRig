#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""UI-free layers extracted from the noodle editor's graphView.py.

The package imports nothing: submodules stay individually importable
headless (no Qt, no OpenGL, no pxr.Usdviewq), so each layer can be tested
directly against stub views. The physical spelling ``graphview`` is
deliberately distinct from the legacy module ``graphView.py``; see the
import-identity case in testenv/testUsdNoodlesStageSync.py.
"""
