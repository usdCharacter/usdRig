#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""GraphView overlays primitives and legacy adapters."""

from types import SimpleNamespace

from pxr import Gf, Tf

from ..constants import MAX_RENDER_DEPTH
from ..widgets.statusOverlay import StatusOverlay
from ..widgets.valueCell import Rect, append_quad
from .frame import find_node
from .render_text import _Projection, _submit_quads, _vertex_class


def paintStatusOverlay(view):
    if not view.nodeIdUnderCursor or not view.shaderLibrary or not view.fontAtlas:
        return

    if view.nodeIdUnderCursor not in view.nodes:
        return

    node = view.nodes[view.nodeIdUnderCursor]
    projection = view._screenSpaceProjectionMatrix()

    view.statusOverlay.render(
        node,
        view.textRenderer,
        _vertex_class(),
        view._drawNodeVertices,
        projection,
        view.height(),
        MAX_RENDER_DEPTH,
    )


def paintMarquee(view):
    if not view.marqueeActive or not view.shaderLibrary:
        return

    NodeVertex = _vertex_class()
    startPt = Gf.Vec2d(view.marqueeStart.x(), view.marqueeStart.y())
    currentPt = Gf.Vec2d(view.marqueeCurrent.x(), view.marqueeCurrent.y())
    marqueeBounds = Gf.Range2d.GetUnion(
        Gf.Range2d(startPt, startPt), Gf.Range2d(currentPt, currentPt)
    )

    size = marqueeBounds.GetSize()
    if size[0] < 1.0 or size[1] < 1.0:
        return  # Too small to render

    minPt = marqueeBounds.GetMin()
    maxPt = marqueeBounds.GetMax()
    w, h = size[0], size[1]
    depth = MAX_RENDER_DEPTH - 1.0  # Near the front

    r, g, b, a = 100, 150, 255, 80  # Semi-transparent blue

    marqueeVertexData = [
        NodeVertex(
            minPt[0],
            minPt[1],
            depth,
            0,
            0,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
        NodeVertex(
            maxPt[0],
            minPt[1],
            depth,
            w,
            0,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
        NodeVertex(
            minPt[0],
            maxPt[1],
            depth,
            0,
            h,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
        NodeVertex(
            maxPt[0],
            minPt[1],
            depth,
            w,
            0,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
        NodeVertex(
            maxPt[0],
            maxPt[1],
            depth,
            w,
            h,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
        NodeVertex(
            minPt[0],
            maxPt[1],
            depth,
            0,
            h,
            w,
            h,
            r,
            g,
            b,
            a,
            2.0,
            0,
            0,
            0,
            255,
            0.0,
        ),
    ]

    try:
        projection = view._worldSpaceProjectionMatrix()
        strokeColor = (150 / 255.0, 200 / 255.0, 1.0, 180 / 255.0)
        # Always force upload for marquee since it changes every frame during drag
        view._drawNodeVertices(
            marqueeVertexData,
            projection,
            0.0,
            strokeColor,
            generation=0,
            selectionChanged=True,
        )

    except Exception as e:
        Tf.Warn(f"Error rendering marquee: {e}")
        import traceback

        traceback.print_exc()


def draw(frame, resources, mirror, transforms):
    draw_status(frame, resources, transforms)
    draw_marquee(frame, resources, transforms)


def draw_status(frame, resources, transforms):
    text = getattr(resources, "text_renderer", None)
    if text is None or not frame.overlay.status_node_id:
        return
    node = find_node(frame, frame.overlay.status_node_id)
    if node is None:
        return
    StatusOverlay().render(
        SimpleNamespace(name=node.title, type=node.node_type), text, _vertex_class(),
        lambda vertices, projection, radius, stroke, **_: _submit_quads(
            resources, vertices, projection, radius, stroke),
        _Projection(transforms.screen), frame.viewport.height, MAX_RENDER_DEPTH,
    )


def draw_marquee(frame, resources, transforms):
    rect = frame.overlay.marquee_rect
    if rect is None or rect[2] - rect[0] < 1.0 or rect[3] - rect[1] < 1.0:
        return
    vertices = []
    append_quad(vertices, _vertex_class(), Rect(*rect), MAX_RENDER_DEPTH - 1.0,
                (100 / 255, 150 / 255, 1.0, 80 / 255))
    _submit_quads(resources, vertices, _Projection(transforms.world), 0.0,
                  (150 / 255, 200 / 255, 1.0, 180 / 255))
