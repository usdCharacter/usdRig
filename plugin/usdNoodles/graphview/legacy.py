#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""Late binding to the legacy ``UsdNoodles.graphView`` module.

Every layer extracted from ``graphView.py`` resolves the names the suite
patches at module scope (``collect_links_for_prim``, ``_push_undo_command``,
``QtWidgets``, ``positionWriteGeneration``, ``layout_graph``, ``NodeVertex``,
``NoodlesConfig``, ``warn_if_non_persistent_edit_target``) through
:func:`legacy_symbol` at call time, never through an import-time copy: a
``patch("UsdNoodles.graphView.<name>")`` context replaces the binding the
moved code reads. Names the suite never patches are imported directly by
the owning layer instead.
"""

from importlib import import_module
import sys


_LEGACY_MODULE = "UsdNoodles.graphView"


def legacy_symbol(name):
    """Return the current ``UsdNoodles.graphView.<name>`` binding.

    Looks the legacy module up in ``sys.modules`` on every call so a
    patched binding is what the caller sees. Import the shim lazily if
    a consumer entered through the canonical module.
    """
    module = sys.modules.get(_LEGACY_MODULE) or import_module(_LEGACY_MODULE)
    return getattr(module, name)
