#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#
"""Compatibility imports for the canonical noodle editor widget."""
from pxr import Tf
from pxr.Usdviewq.qt import QtCore, QtWidgets

from .core import positionWriteGeneration
from .render import NodeVertex
from .nodeGraph import warn_if_non_persistent_edit_target
from .nodeLayout import layout_graph
from .noodlesConfig import NoodlesConfig
from .pinUtils import collect_links_for_prim
from .graphview.editing import (
    _make_connection_edit, _make_reconnect_edits, _make_rename_undo,
    _push_undo_command,
)
from .graphview.interaction import _ValueHit, guards_qt_boundary as _guardsQtBoundary
from .graphview.render import _ProfileSection
from .graphview.stage_adapter import _makeStageSync
from .graphview.stage_sync import _isPathRelated, _nodeIdPrimPath
from .graphview.view import GraphView
