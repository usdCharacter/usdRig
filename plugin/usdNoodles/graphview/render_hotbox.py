#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#

"""GraphView hotbox primitives and legacy adapters."""

from OpenGL import GL

from ..constants import MAX_RENDER_DEPTH
from ..widgets.valueCell import Rect, append_quad
from .render_text import _Projection, _submit_quads, _vertex_class


def _paintNodeCreationHotbox(view):
    """Render the node creation hotbox overlay."""
    projection = view._screenSpaceProjectionMatrix()
    view.nodeCreationHotbox.render(
        view.textRenderer,
        _vertex_class(),
        view._drawNodeVertices,
        projection,
        view.width(),
        view.height(),
        MAX_RENDER_DEPTH,
    )


def draw(frame, resources, mirror, transforms):
    box = frame.hotbox
    if not box.visible or box.rect is None or box.alpha < 0.01:
        return
    projection = _Projection(transforms.screen)
    depth = MAX_RENDER_DEPTH - 0.2
    text = getattr(resources, 'text_renderer', None)
    use_gl = getattr(resources, 'shader_library', None) is not None

    def quad(rect, color, radius=0.0, stroke=(0.0, 0.0, 0.0, 0.0)):
        vertices = []
        append_quad(vertices, _vertex_class(), Rect(*rect), depth, color, box.alpha)
        _submit_quads(resources, vertices, projection, radius, stroke)

    def label(spec):
        string, x, baseline, size, color = spec
        if text is not None and text.font_atlas:
            vertices = []
            text.generateTextVertices(string, x, baseline, depth, size, vertices, node_index=-1.0)
            if vertices:
                text.drawTextVertices(vertices, projection,
                                      (*color[:3], color[3] * box.alpha), disable_depth=False)

    if use_gl:
        depth_test = GL.glIsEnabled(GL.GL_DEPTH_TEST)
        depth_mask = GL.glGetBooleanv(GL.GL_DEPTH_WRITEMASK)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDepthMask(GL.GL_FALSE)
    try:
        quad(box.rect, (0.1, 0.1, 0.12, 0.95), 6.0, (0.3, 0.4, 0.5, 0.6 * box.alpha))
        if box.input_rect:
            quad(box.input_rect, (0.15, 0.15, 0.18, 1.0), 2.0,
                 (0.3, 0.3, 0.35, 0.6 * box.alpha))
        if box.input_label:
            label(box.input_label)
        if box.cursor_rect:
            quad(box.cursor_rect, (0.95, 0.95, 0.95, 1.0))
        clipped = use_gl and box.list_clip is not None
        if clipped:
            scissor = GL.glIsEnabled(GL.GL_SCISSOR_TEST)
            old_rect = GL.glGetIntegerv(GL.GL_SCISSOR_BOX)
            vx, vy, vw, vh = GL.glGetIntegerv(GL.GL_VIEWPORT)
            sx = vw / max(frame.viewport.width, 1)
            sy = vh / max(frame.viewport.height, 1)
            x0, y0, x1, y1 = box.list_clip
            GL.glEnable(GL.GL_SCISSOR_TEST)
            GL.glScissor(int(vx + x0 * sx), int(vy + vh - y1 * sy),
                         int((x1 - x0) * sx), int((y1 - y0) * sy))
        try:
            for index, rect in box.rows:
                if index < 0:
                    quad(rect, (0.08, 0.08, 0.10, 0.95))
                elif index == box.selected_index:
                    quad(rect, (0.3, 0.5, 0.8, 1.0))
            for spec in box.labels:
                label(spec)
            if box.scrollbar:
                x, y, w, h, thumb_y, thumb_h, active = box.scrollbar
                quad((x, y, x + w, y + h), (0.16, 0.16, 0.19, 0.95))
                quad((x, thumb_y, x + w, thumb_y + thumb_h),
                     (0.55, 0.62, 0.78, 1.0) if active else (0.42, 0.44, 0.50, 1.0))
        finally:
            if clipped:
                GL.glScissor(*old_rect)
                if not scissor:
                    GL.glDisable(GL.GL_SCISSOR_TEST)
    finally:
        if use_gl:
            GL.glDepthMask(depth_mask)
            if depth_test:
                GL.glEnable(GL.GL_DEPTH_TEST)
