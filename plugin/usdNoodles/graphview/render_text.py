#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# in plugin/usdNoodles/ in this repository.
#


"""Text, value cells and in-place editors: the text-side draw owner.

``draw(frame, resources, mirror, transforms)`` paints node text and inline
value cells from a detached frame, the render-owned native mirror, and the
one shared ``NodeTransformFrame`` -- never from the live model, and it
authors nothing. The frame, mirror and transform inputs arrive from
``render.Renderer.draw``; the text renderer, atlas and shaders arrive on
the ``resources`` bundle. Any missing piece (no resources, no text
renderer, no atlas, no mirror, no nodes) skips its submit, so
resource-less draws still sync the mirror.

Node text goes through the existing ``TextRenderer`` against the mirror:
``renderNodeText`` with the frame's ``content_changed`` flag. The native
text manager retains its glyph ranges across draws and rewrites them on a
model or value change -- draw invalidates by passing that flag through,
never by re-measuring per paint, and it never clears a live
``textChanged`` flag the way the legacy paint path does. Title-bar icons
stay with ``render_nodes``: the legacy ``paintText`` drew them first, but
in a draw they are the node primitive's submit, not this one.

Value cells are a Python pass by necessity (as the moved ``_paintValueCells``
docstring explains, node text is generated entirely in C++ with no hook
for a value string), rebuilt here from frame records: each record's
``value_inputs`` plus its pinned layout (row slots/kinds, port geometry,
cell width) through the same ``widgets/valueCell.py`` geometry the legacy
pass uses. State colors resolve from the record's ``state``/``editable``
(``value_colors_for_state``), fitted display texts from its
``type_name``/``components``, and glyph metrics from its ``glyph_inputs``:
nothing is read from a live node. Cells ride the node's own z-order band
and add the frame's transient drag offset, so they move with the quads,
the C++ text and the icons on one shared transform frame.

Draw keeps no second cache on the model: fitted texts, glyph vertices,
digit advances, axis widths and text bands live on the render-owned
``resources`` bundle, keyed by what each entry depends on. A value change
misses and recomputes; an identical redraw retains. The cache is dropped
when the atlas object is replaced, matching the legacy atlas-regeneration
clear. Hover, the edit target, the mung target and the commit flash arrive
time-resolved in the frame, so draw performs none of the legacy pass's
paint-time model writes: no flash-expiry clearing, no ``update()`` pumps.

The screen-space editor widgets -- the rename input, the token popup
list, the mung readout and the hotkey popups -- still paint through the
moved legacy methods on the view. The frame pins their rects and entries,
but their renderers (``SelectableList``, ``TextInputWidget``) are
view-owned; like ``render_nodes`` with group stickers, ``draw`` covers
what the frame fully describes and documents the rest instead of
redesigning it mid-move.

Moved verbatim from ``GraphView`` as module-level functions whose first
parameter is ``view``: ``paintText``, ``_paintValueCells``,
``_paintValueEditor``, ``_emitValueText``, ``_valueTextBand``,
``_valueGlyph``, ``_valueBaselineY``, ``_appendBoolSwitch``,
``_appendChevron``, ``_paintNodeRename``, ``_paintMungLadder``,
``_paintTokenPopup``, ``_activeValueFlash``, ``_startValueFlash``,
``paintHotkeyPopup``, ``paintNavigationHotkeyPopup`` and
``_paintHotkeyPopups``. The suite-patched ``NodeVertex``,
``NoodlesConfig`` and ``QtWidgets`` resolve at call time through
``legacy_symbol``, never through an import-time copy.
"""

from __future__ import annotations

import time
from collections import defaultdict
from types import SimpleNamespace

from pxr import Gf, Tf
from pxr.Usdviewq.qt import QtCore, QtGui

from .. import noodlesValues
from ..constants import MAX_RENDER_DEPTH, node_content_depth
from ..widgets import valueCell
from . import render_nodes
from .legacy import legacy_symbol


_CACHES_ATTR = "_render_text_caches"


class _Projection:
    """A projection exposing ``QMatrix4x4.data()`` over plain tuples.

    The text facade and the quad submitter still take a Qt matrix; this
    shim feeds them the draw's world transform without importing Qt into
    the call path.
    """

    def __init__(self, values):
        self._values = list(values)

    def data(self):
        return list(self._values)


# -- draw entry ---------------------------------------------------------------


def draw(frame, resources, mirror, transforms):
    """Paint the text primitives for *frame*.

    Submits node text, then value cells, from the render-owned *mirror*
    and the frame's records. Each submit is skipped when its renderer,
    the mirror, the atlas, or the frame's nodes are missing. A primitive
    failure propagates to ``Renderer.draw`` after the (already coherent)
    mirror swap, exactly as the renderer documents.
    """
    _submit_node_text(frame, resources, mirror, transforms)
    _submit_value_cells(frame, resources, mirror, transforms)


def _submit_node_text(frame, resources, mirror, transforms):
    """Submit node text for the mirror; the native manager retains ranges.

    ``content_changed`` is the only invalidation: set, the native pass
    rewrites its glyph ranges; clear, it redraws the retained ones. No
    live ``textChanged`` flag is touched -- the next capture re-reads it.
    """
    text_renderer = None
    if resources is not None:
        text_renderer = getattr(resources, "text_renderer", None)
    if text_renderer is None or mirror is None:
        return
    if not getattr(frame, "nodes", ()):
        return
    if getattr(resources, "font_atlas", None) is None:
        return
    if getattr(text_renderer, "font_atlas", None) is None:
        return
    viewport = frame.viewport
    text_renderer.renderNodeText(
        mirror,
        _Projection(transforms.world),
        viewport.zoom,
        bool(getattr(frame, "content_changed", False)),
        viewport.pan_x,
        viewport.pan_y,
        float(viewport.width),
        float(viewport.height),
        True,
    )


def _submit_value_cells(frame, resources, mirror, transforms):
    """Submit value cells for the frame's records, batched like the legacy.

    One quad submit per (corner radius, stroke) bucket and one text submit
    per color bucket, so the cost is O(visible value rows) with no
    per-row GL state changes. Geometry, colors, formats and glyphs all
    resolve from records; the live model is never read and never written.
    """
    del mirror
    text_renderer = None
    if resources is not None:
        text_renderer = getattr(resources, "text_renderer", None)
    if text_renderer is None:
        return
    atlas = getattr(resources, "font_atlas", None)
    if atlas is None:
        return
    if getattr(text_renderer, "font_atlas", None) is None:
        return
    if not getattr(frame, "nodes", ()):
        return
    viewport = frame.viewport
    zoom = viewport.zoom if viewport.zoom else 1.0
    world = viewport.world_bounds
    theme = _value_theme()
    caches = _caches_for(resources, atlas)
    vertex_class = _vertex_class()
    projection = _Projection(transforms.world)
    hover = getattr(frame, "value_hover", None)
    overlay = frame.overlay
    editor = overlay.value_editor
    mung_target = overlay.mung_target
    flash = overlay.flash

    quad_batches = defaultdict(list)  # (radius, strokeColor) -> vertices
    text_batches = defaultdict(list)  # rgba -> glyph floats
    caret_vertices = []

    for record in frame.nodes:
        glyphs = record.glyph_inputs
        if not glyphs.show_values:
            continue
        font_size = float(glyphs.font_size)
        if font_size <= 0.0:
            continue
        row_height = float(record.port_line_height)
        if row_height <= 0.0:
            continue
        # The LOD gate, and it FADES rather than pops: value text is
        # 0.9x the pin font, so it turns to noise a little before the
        # labels do, and a cell that vanished between one wheel notch
        # and the next would read as a bug.
        pixels = row_height * zoom
        min_pixels = float(glyphs.min_pixel_height)
        if pixels < min_pixels:
            continue
        lod_alpha = min(1.0, (pixels - min_pixels) / max(1e-6, min_pixels))
        lod_alpha = max(0.0, lod_alpha)
        if lod_alpha <= 0.01:
            continue
        dx, dy = frame.transform_inputs.offset_for(record.node_id)
        x, y = record.position
        w, h = record.size
        if not _overlaps(
            (x + dx, y + dy, x + dx + w, y + dy + h), world
        ):
            continue

        def measure(text, _size=font_size):
            return text_renderer.calculateTextWidth(text, _size)

        # Cells ride the node's own z-order band, exactly as its text and
        # icons do, so they raise together when a node is brought forward.
        depth = node_content_depth(record.z_order)
        band = _text_band(caches, row_height, font_size, atlas)
        radius = None
        node_id = record.node_id
        for value, rect, subs, h in _record_rows(record, band, dx, dy):
            if radius is None:
                radius = round(valueCell.corner_radius(h), 3)
            inner_pad = valueCell.INNER_PAD_RATIO * h
            row_hovered = hover is not None and hover[0] == node_id and (
                hover[1] == value.property_name
            )
            row_active = (
                editor is not None
                and editor.node_id == node_id
                and editor.property_name == value.property_name
            ) or (
                mung_target is not None
                and mung_target[0] == node_id
                and mung_target[1] == value.property_name
            )
            fill, stroke, text_color, italic = value_colors_for_state(
                value.state, value.editable, theme, row_hovered, row_active
            )
            if flash is not None and flash.node_id == node_id and (
                flash.property_name == value.property_name
            ):
                fill = theme.fillError if flash.kind == "error" else theme.fillAccent
                stroke = theme.error if flash.kind == "error" else theme.accent

            if fill is not None:
                valueCell.append_quad(
                    quad_batches[(radius, stroke)],
                    vertex_class,
                    rect,
                    depth,
                    fill,
                    lod_alpha,
                )
            elif stroke is not None:
                valueCell.append_quad(
                    quad_batches[(radius, stroke)],
                    vertex_class,
                    rect,
                    depth,
                    (0.0, 0.0, 0.0, 0.0),
                    lod_alpha,
                )

            # A time-sampled value carries the animation tint down its
            # leading edge: present at a glance, not another word to read.
            if value.state == noodlesValues.STATE_ANIMATED:
                valueCell.append_quad(
                    quad_batches[(0.0, None)],
                    vertex_class,
                    valueCell.Rect(
                        rect.left, rect.top, rect.left + 0.05 * h, rect.bottom
                    ),
                    depth,
                    theme.textAnimated,
                    lod_alpha,
                )
            if value.has_swatch and len(value.components) >= 3:
                valueCell.append_quad(
                    quad_batches[(round(radius * 0.7, 3), theme.hairline)],
                    vertex_class,
                    valueCell.swatch_rect(rect, h),
                    depth,
                    (
                        float(value.components[0]),
                        float(value.components[1]),
                        float(value.components[2]),
                        1.0,
                    ),
                    lod_alpha,
                )

            baseline_y = _baseline_y(rect, font_size, atlas)
            tabular = value.kind in (
                noodlesValues.KIND_FLOAT,
                noodlesValues.KIND_INT,
            )
            # One measure for the whole row, and a TABULAR one for
            # numbers, so the fitted format matches the widths the
            # glyphs will actually be set at.
            if tabular:
                def row_measure(text, _size=font_size):
                    return _text_width(
                        caches, text_renderer, text, _size, True
                    )
            else:
                row_measure = measure
            axis_labels = (
                valueCell.COLOR_AXIS_LABELS
                if value.has_swatch
                else valueCell.AXIS_LABELS
            )
            hair_width = max(0.6 / max(zoom, 1e-3), 0.012 * h)

            # Fit ONCE for the whole value, so every component of a
            # vector is shown to the same number of decimals -- and once
            # per value, not once per frame (see _fitted_texts).
            slot_width = valueCell.text_max_width(subs[0], h)
            if value.kind == noodlesValues.KIND_TOKEN_ENUM:
                slot_width -= 0.20 * h * 2.2
            if value.count > 1:
                slot_width -= max(
                    _axis_width(caches, text_renderer, label, font_size * 0.8)
                    for label in axis_labels[: value.count]
                ) + inner_pad * 0.5
            texts = _fitted_texts(
                caches, node_id, value, slot_width, font_size, row_measure
            )

            for index, sub in enumerate(subs):
                component_hovered = row_hovered and hover[2] == index
                # Components of one value live in ONE pill, divided by
                # hairlines rather than by gaps, so a vector reads as a
                # single value with parts.
                if index > 0:
                    valueCell.append_quad(
                        quad_batches[(0.0, None)],
                        vertex_class,
                        valueCell.Rect(
                            sub.left - hair_width * 0.5,
                            rect.top + 0.12 * (rect.bottom - rect.top),
                            sub.left + hair_width * 0.5,
                            rect.bottom - 0.12 * (rect.bottom - rect.top),
                        ),
                        depth,
                        theme.hairline,
                        lod_alpha,
                    )
                if component_hovered and fill is not None and not row_active:
                    valueCell.append_quad(
                        quad_batches[(radius, None)],
                        vertex_class,
                        sub,
                        depth,
                        theme.fillHover,
                        lod_alpha * 0.8,
                    )

                if value.kind == noodlesValues.KIND_BOOL:
                    _bool_switch(
                        quad_batches,
                        vertex_class,
                        sub,
                        h,
                        depth,
                        value,
                        theme,
                        lod_alpha,
                    )
                    continue

                available = valueCell.text_max_width(sub, h)
                caret_size = 0.20 * h
                if value.kind == noodlesValues.KIND_TOKEN_ENUM:
                    available -= caret_size * 2.2
                axis_width = 0.0
                if value.count > 1:
                    axis_width = (
                        _axis_width(
                            caches,
                            text_renderer,
                            axis_labels[min(index, 3)],
                            font_size * 0.8,
                        )
                        + inner_pad * 0.5
                    )
                    available -= axis_width
                if available <= 0.0:
                    continue
                if (
                    editor is not None
                    and editor.node_id == node_id
                    and editor.property_name == value.property_name
                    and editor.component == index
                ):
                    continue  # the editor is drawing this one

                if value.count > 1:
                    axis_color = valueCell.AXIS_COLORS[min(index, 3)]
                    key = (axis_color[0], axis_color[1], axis_color[2],
                           0.85 * lod_alpha)
                    _emit_text(
                        caches,
                        text_renderer,
                        axis_labels[min(index, 3)],
                        sub.left + inner_pad,
                        baseline_y,
                        depth,
                        font_size * 0.8,
                        text_batches[key],
                    )

                text = texts[index] if index < len(texts) else ""
                if not text:
                    continue
                if value.kind in (
                    noodlesValues.KIND_TEXT,
                    noodlesValues.KIND_TOKEN_ENUM,
                ):
                    text_x = sub.left + inner_pad
                else:
                    text_x = (
                        sub.right
                        - inner_pad
                        - _text_width(
                            caches, text_renderer, text, font_size, tabular
                        )
                    )
                key = (text_color[0], text_color[1], text_color[2],
                       text_color[3] * lod_alpha)
                _emit_text(
                    caches,
                    text_renderer,
                    text,
                    text_x,
                    baseline_y,
                    depth,
                    font_size,
                    text_batches[key],
                    tabular=tabular,
                    italic=italic,
                )
                if value.kind == noodlesValues.KIND_TOKEN_ENUM:
                    cx = sub.right - inner_pad - caret_size * 0.6
                    cy = (rect.top + rect.bottom) * 0.5
                    _chevron(
                        caret_vertices,
                        vertex_class,
                        cx,
                        cy,
                        caret_size,
                        depth,
                        text_color,
                        lod_alpha,
                    )

    for (radius, stroke), vertices in quad_batches.items():
        if not vertices:
            continue
        _submit_quads(
            resources,
            vertices,
            projection,
            radius,
            stroke if stroke is not None else (0.0, 0.0, 0.0, 0.0),
        )
    if caret_vertices:
        _submit_quads(
            resources, caret_vertices, projection, 0.0, (0.0, 0.0, 0.0, 0.0)
        )
    for color, vertices in text_batches.items():
        if vertices:
            text_renderer.drawTextVertices(vertices, projection, color)


# -- draw helpers -------------------------------------------------------------
#
# Record-driven twins of the view-side value geometry: the same formulas
# as the moved methods and the model value helpers, fed by frame records
# and cached on the render-owned resources bundle instead of the view.


def _overlaps(bounds, world):
    """Whether a world rect overlaps the visible world rect at all."""
    return not (
        bounds[2] < world[0]
        or bounds[0] > world[2]
        or bounds[3] < world[1]
        or bounds[1] > world[3]
    )


def _caches_for(resources, atlas):
    """The draw's private caches, held on the resources bundle.

    Glyph vertices, digit advances, fitted texts, axis widths and text
    bands, each keyed by what its entry depends on. A value change misses
    and recomputes; an identical redraw retains. The whole set is dropped
    when the atlas object is replaced, matching the legacy
    atlas-regeneration clear.
    """
    caches = getattr(resources, _CACHES_ATTR, None)
    if caches is None or caches.get("atlas") is not atlas:
        caches = {
            "atlas": atlas,
            "glyphs": {},
            "digits": {},
            "fits": {},
            "axes": {},
            "bands": {},
        }
        try:
            setattr(resources, _CACHES_ATTR, caches)
        except Exception:
            pass
    return caches


def _config_value(key, default):
    """The ``NoodlesConfig`` value for *key*, resolved at call time.

    Reads through the legacy module so a patched ``NoodlesConfig``
    binding is honored; when the legacy module was never imported
    (resource-less renderer draws), reads the config module directly.
    Either way the lookup happens per call, never as an import-time
    copy.
    """
    try:
        config = legacy_symbol("NoodlesConfig")
    except (KeyError, AttributeError):
        from ..noodlesConfig import NoodlesConfig as config
    get = getattr(config, "get", None)
    if get is None:
        return default
    try:
        return get(key, default)
    except Exception:
        return default


def _vertex_class():
    """The ``NodeVertex`` binding, resolved at call time like the above."""
    try:
        return legacy_symbol("NodeVertex")
    except (KeyError, AttributeError):
        from ..render import NodeVertex

        return NodeVertex


def _value_theme():
    """Every value-cell color, derived from the node theme per draw.

    The same keys and defaults the view caches as ``_cachedValueTheme``:
    a change to the node background or the selection accent carries
    through to the cells without a second palette to maintain.
    """
    return valueCell.build_theme(
        _config_value("nodeBgHigh", 90),
        _config_value("nodeBgLow", 86),
        _config_value("selectedNodeStrokeColor", [1.0, 1.0, 0.117, 1.0]),
        _config_value("portConnectedFillColor", [0.7, 0.8, 0.0, 1.0]),
        _config_value(
            "portDisconnectedFillColor", [0.235, 0.235, 0.235, 1.0]
        ),
        _config_value("valueConnectedColor", [0.45, 0.62, 0.85, 1.0]),
        _config_value("valueAnimatedColor", [0.95, 0.72, 0.25, 1.0]),
        _config_value("valueAuthoredColor", [0.93, 0.93, 0.95, 1.0]),
        _config_value("valueFallbackColor", [0.93, 0.93, 0.95, 0.42]),
    )


def value_colors_for_state(state, editable, theme, hovered=False, active=False):
    """``(fill, stroke, textColor, italicShear)`` for one row's pill.

    The same state table as the ``_valueStateColors`` geometry helper,
    fed by a frame record's ``state``/``editable`` instead of a live row.
    Fill is None where the state asks for no pill at all: a schema
    fallback is information the user did not author, so it is shown as
    dim text on the bare row rather than as a filled field inviting a
    click.
    """
    fill = theme.fillAuthored
    stroke = theme.hairline
    text = theme.textNormal
    italic = 0.0
    if state == noodlesValues.STATE_CONNECTED or not editable:
        fill = None
        stroke = None
        if state == noodlesValues.STATE_CONNECTED:
            text = theme.textConnected
        else:
            text = theme.textDim
        italic = 0.20
    elif state == noodlesValues.STATE_FALLBACK:
        fill = None
        stroke = None
        text = theme.textDim
    elif state == noodlesValues.STATE_ANIMATED:
        text = theme.textAnimated
    if hovered and fill is not None:
        fill = theme.fillHover
    if active:
        fill = theme.fillAccent
        stroke = theme.accent
    return fill, stroke, text, italic


def text_band_for(row_height, size, atlas, pin_font, type_font):
    """``(top offset, height)`` of the value pill within a row band.

    Pure twin of ``_valueTextBand``'s formula: the pill hangs off the row
    label's baseline (see that method for why), never taller than its row.
    """
    row_height = float(row_height)
    line_height = float(atlas.lineHeight)
    block = (pin_font + type_font) * line_height
    baseline = (row_height - block) * 0.5 + float(atlas.ascender) * pin_font
    pad = 0.07 * row_height
    top = baseline - float(atlas.ascender) * size - pad
    height = (
        float(atlas.ascender) - float(atlas.descender)
    ) * size + 2.0 * pad
    # Never taller than the row it belongs to.
    if height > row_height:
        height = row_height
    if top < 0.0:
        top = 0.0
    if top + height > row_height:
        top = max(0.0, row_height - height)
    return (top, height)


def _text_band(caches, row_height, size, atlas):
    """The cached text band: pure function of row height, fonts, metrics."""
    if atlas is None:
        return (
            valueCell.CELL_TOP_RATIO * row_height,
            valueCell.CELL_HEIGHT_RATIO * row_height,
        )
    scale = _config_value("valueFontScale", 0.9) or 0.9
    pin_font = float(size) / float(scale)
    if _config_value("showPinTypeLabels", True):
        type_font = float(_config_value("nodePinTypeFontSize", 26.0))
    else:
        type_font = 0.0
    key = (
        round(float(row_height), 3),
        pin_font,
        type_font,
        size,
        float(atlas.lineHeight),
        float(atlas.ascender),
        float(atlas.descender),
    )
    band = caches["bands"].get(key)
    if band is None:
        band = text_band_for(float(row_height), size, atlas, pin_font, type_font)
        caches["bands"][key] = band
    return band


def _glyph(caches, text_renderer, ch, size):
    """``(vertices, width)`` for one character, generated at the origin.

    A value is the same dozen glyphs every frame, so each character is
    generated once per size at (0, 0, 0) and translated by the caller.
    """
    key = (ch, size)
    cached = caches["glyphs"].get(key)
    if cached is None:
        glyph = []
        text_renderer.generateTextVertices(
            ch, 0.0, 0.0, 0.0, size, glyph, node_index=-1.0
        )
        cached = (tuple(glyph), text_renderer.calculateTextWidth(ch, size))
        caches["glyphs"][key] = cached
    return cached


def _digit_advance(caches, text_renderer, size):
    """The widest digit at *size*: the cell every digit is set in."""
    cached = caches["digits"].get(size)
    if cached is None:
        cached = max(
            text_renderer.calculateTextWidth(d, size) for d in "0123456789"
        )
        caches["digits"][size] = cached
    return cached


def _text_width(caches, text_renderer, text, size, tabular):
    """The width *text* is set at: tabular digits ride one advance."""
    if not tabular:
        return text_renderer.calculateTextWidth(text, size)
    advance = _digit_advance(caches, text_renderer, size)
    total = 0.0
    for ch in text:
        if ch.isdigit():
            total += advance
        else:
            total += _glyph(caches, text_renderer, ch, size)[1]
    return total


def _emit_text(
    caches,
    text_renderer,
    text,
    x,
    baseline_y,
    depth,
    size,
    out,
    tabular=False,
    italic=0.0,
):
    """Append one value string's glyphs to *out*, starting at *x*.

    ``tabular`` sets digits on the widest digit's advance, so a number
    does not shuffle sideways while it is being dragged. ``italic``
    shears the glyphs around the baseline -- the atlas has no italic
    face, and a read-only value has to look different from an editable
    one at a glance.
    """
    if not text:
        return 0.0
    if not tabular and italic <= 0.0:
        text_renderer.generateTextVertices(
            text, x, baseline_y, depth, size, out, node_index=-1.0
        )
        return text_renderer.calculateTextWidth(text, size)
    advance = _digit_advance(caches, text_renderer, size) if tabular else 0.0
    cursor = x
    for ch in text:
        verts, natural = _glyph(caches, text_renderer, ch, size)
        step = advance if (tabular and ch.isdigit()) else natural
        ox = cursor + (step - natural) * 0.5
        oy = baseline_y
        # Glyph vertices are flat floats, 6 per vertex (x, y, z, u, v,
        # nodeIndex); the cached glyph is translated into place, and an
        # italic lean is two lines about the baseline.
        if italic > 0.0:
            for i in range(0, len(verts), 6):
                gx = verts[i] + ox
                gy = verts[i + 1] + oy
                out.append(gx + (baseline_y - gy) * italic)
                out.append(gy)
                out.append(verts[i + 2] + depth)
                out.append(verts[i + 3])
                out.append(verts[i + 4])
                out.append(verts[i + 5])
        else:
            for i in range(0, len(verts), 6):
                out.append(verts[i] + ox)
                out.append(verts[i + 1] + oy)
                out.append(verts[i + 2] + depth)
                out.append(verts[i + 3])
                out.append(verts[i + 4])
                out.append(verts[i + 5])
        cursor += step
    return cursor - x


def _fitted_texts(caches, node_id, value, slot_width, font_size, row_measure):
    """Display texts for one record's components, fitted once per value.

    The fit is pure arithmetic over (components, width, font): a value
    change misses and refits, an identical redraw retains.
    """
    key = (
        value.components,
        round(float(slot_width), 2),
        value.type_name,
        font_size,
    )
    entry = caches["fits"].get((node_id, value.property_name))
    if entry is not None and entry[0] == key:
        return entry[1]
    texts = noodlesValues.fit_components(
        value.components, value.type_name, max(slot_width, 1.0), row_measure
    )
    caches["fits"][(node_id, value.property_name)] = (key, texts)
    return texts


def _axis_width(caches, text_renderer, label, size):
    """Width of one axis hint (``x``/``y``/``z``/``r``/...) at *size*.

    A handful of constant single characters measured once each instead
    of once per vector row per frame.
    """
    key = (label, size)
    width = caches["axes"].get(key)
    if width is None:
        width = text_renderer.calculateTextWidth(label, size)
        caches["axes"][key] = width
    return width


def _baseline_y(rect, size, atlas):
    """Text baseline inside a value pill."""
    block_height = (float(atlas.ascender) - float(atlas.descender)) * size
    top = rect.top + ((rect.bottom - rect.top) - block_height) * 0.5
    return top + float(atlas.ascender) * size


def _bool_switch(
    quad_batches, vertex_class, sub, h, depth, value, theme, alpha
):
    """A rounded checkbox drawn in the PIN's colors, not its own.

    On is the connected-port fill, off is the disconnected one, so a
    boolean reads the same way a wired port does two columns to the left.
    """
    inner_pad = valueCell.INNER_PAD_RATIO * h
    side = 0.46 * h
    top = sub.top + ((sub.bottom - sub.top) - side) * 0.5
    box = valueCell.Rect(
        sub.right - inner_pad - side, top, sub.right - inner_pad, top + side
    )
    on = bool(value.components[0])
    radius = round(side * 0.28, 3)
    valueCell.append_quad(
        quad_batches[(radius, theme.hairline)],
        vertex_class,
        box,
        depth,
        theme.boolOn if on else theme.boolOff,
        alpha,
    )
    if on:
        inset = side * 0.28
        valueCell.append_quad(
            quad_batches[(round(radius * 0.6, 3), None)],
            vertex_class,
            valueCell.Rect(
                box.left + inset,
                box.top + inset,
                box.right - inset,
                box.bottom - inset,
            ),
            depth,
            (0.10, 0.11, 0.09, 0.9),
            alpha,
        )


def _append_triangle(out, vertex_class, points, depth, color):
    """One flat triangle in the NodeVertex convention (the chevron's body)."""
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    width = max(max_x - min_x, 1.0)
    height = max(max_y - min_y, 1.0)
    r = int(color[0] * 255)
    g = int(color[1] * 255)
    b = int(color[2] * 255)
    a = int(color[3] * 255)
    for point_x, point_y in points:
        out.append(
            vertex_class(
                float(point_x),
                float(point_y),
                depth,
                float(point_x - min_x),
                float(point_y - min_y),
                float(width),
                float(height),
                r,
                g,
                b,
                a,
                0.0,
                0,
                0,
                0,
                255,
                0.0,
            )
        )


def _chevron(out, vertex_class, cx, cy, size, depth, color, alpha):
    """The small downward chevron that says "this opens a list"."""
    _append_triangle(
        out,
        vertex_class,
        [
            (cx - size * 0.55, cy - size * 0.28),
            (cx + size * 0.55, cy - size * 0.28),
            (cx, cy + size * 0.34),
        ],
        depth,
        (color[0], color[1], color[2], color[3] * alpha),
    )


def _record_rows(record, band, dx=0.0, dy=0.0):
    """Yield ``(value, cellRect, subRects, h)`` for each value-bearing row.

    Record twin of ``_valueRowGeometry``: the same four row-level
    suppressions (collapsed title, group headers, output-occupied slots,
    unreadably narrow vectors), with the transient drag offset applied so
    the cells ride with the quads. Row geometry still comes from the same
    C++ slot/kind layout the hit tests use, pinned on the record.
    """
    if record.title_collapsed:
        return
    if not record.value_inputs:
        return
    cell_w = float(record.value_cell_width or 0.0)
    if cell_w <= 0.0:
        return
    port_width = float(record.port_width or 0.0)
    if port_width <= 0.0:
        return
    nx = float(record.position[0]) + dx
    nw = float(record.size[0])
    if not valueCell.node_is_wide_enough(nw, port_width):
        return
    h = float(record.port_line_height)
    if h <= 0.0:
        return
    base_y = float(record.position[1]) + dy + float(record.port_start_y)
    out_pins = record.output_pins
    out_slots = record.output_row_slots
    # Every slot an output-side row occupies. A real output row's label is
    # right-aligned to the node's right edge -- into exactly the space the
    # cell wants -- so where the two share a slot, the cell gives way.
    occupied = {
        out_slots[i] if i < len(out_slots) else i
        for i in range(len(out_pins))
    }
    by_pin = {}
    for value in record.value_inputs:
        by_pin.setdefault(value.pin_name, value)
    in_pins = record.input_pins
    in_slots = record.input_row_slots
    in_kinds = record.input_row_kinds
    for i, pin_name in enumerate(in_pins):
        value = by_pin.get(pin_name)
        if value is None:
            continue
        kind = in_kinds[i] if i < len(in_kinds) else 0
        if kind in (1, 2):
            continue
        slot = in_slots[i] if i < len(in_slots) else i
        if slot in occupied:
            continue
        row_top = base_y + slot * h
        rect = valueCell.cell_rect(
            nx,
            nw,
            row_top,
            h,
            cell_w,
            top=row_top + band[0],
            height=band[1],
        )
        if rect is None:
            continue
        inset = valueCell.swatch_inset(h) if value.has_swatch else 0.0
        subs = valueCell.sub_rects_fit(rect, value.count, h, inset)
        if not subs:
            continue
        yield value, rect, subs, h


def _submit_quads(resources, vertices, projection, radius, stroke):
    """Submit one quad bucket through the shared node-vertex path."""
    if not vertices:
        return
    shim = SimpleNamespace(
        shaderLibrary=getattr(resources, "shader_library", None)
    )
    render_nodes._drawNodeVertices(
        shim, vertices, projection, radius, stroke, generation=-1
    )


# -- moved GraphView methods (first parameter ``view``) -------------------------


def paintText(view):
    if not view.nodes or not view.shaderLibrary or not view.fontAtlas:
        return

    try:
        projection = view._worldSpaceProjectionMatrix()

        # Render node icons in title bars. The icons are generated + drawn
        # entirely in C++ from the authoritative GraphModel snapshot (no
        # per-frame quad walk across the language boundary, and they ride
        # node drags via the shared transform frame). The C++ cull of
        # off-screen icons is always-on, matching the glyph cull below.
        if (
            hasattr(view, "_cppIconRenderer")
            and view._cppIconRenderer.isInitialized
        ):
            view._cppIconRenderer.renderNodeIcons(
                view.nodeGraph,
                projection,
                view.zoom,
                view.textChanged,
                view.panX,
                view.panY,
                float(view.width()),
                float(view.height()),
                True,
            )

        success = view.textRenderer.renderNodeText(
            view.nodeGraph,
            projection,
            view.zoom,
            view.textChanged,
            view.panX,
            view.panY,
            float(view.width()),
            float(view.height()),
            True,
        )

        # Only clear textChanged flag if rendering succeeded
        if success and view.textChanged:
            view.textChanged = False

    except Exception as e:
        Tf.Warn(f"Error rendering text: {e}")
        import traceback

        traceback.print_exc()


def _paintValueCells(view):
    """Draw every visible value cell in the node's own visual language.

    A Python pass by necessity: node text is generated and drawn entirely
    in C++ by the pinned external noodles renderer, so there is no hook to
    put a value string into a row's own layout. Everything it draws --
    fills, hairlines, text, the bool switch, the token chevron -- is
    derived from the node theme (``_cachedValueTheme``), so the cells read
    as part of the node rather than as a widget stacked on top of it.

    Batched to a handful of draw calls: one per (corner radius, stroke)
    bucket for the quads, one per colour bucket for the glyphs, so the
    cost is O(visible value rows) with no per-row GL state changes.
    """
    if not view.nodes or not view.shaderLibrary or not view.fontAtlas:
        return
    if not view.textRenderer or not view.textRenderer.font_atlas:
        return

    projection = view._worldSpaceProjectionMatrix()
    fontSize = view._valueFontSize()
    if fontSize <= 0.0:
        return
    theme = view._cachedValueTheme
    minPixelHeight = float(view._cachedValueMinPixelHeight)
    editing = view._valueEditTarget
    munging = view._mungState if view._mungActive else None
    hover = view._valueHover
    flash = view._activeValueFlash()

    quadBatches = defaultdict(list)  # (radius, strokeColor) -> vertices
    textBatches = defaultdict(list)  # rgba -> glyph floats
    caretVertices = []

    def measure(text):
        return view.textRenderer.calculateTextWidth(text, fontSize)

    for nodeId, node in view.nodes.items():
        rowHeight = float(node.layoutPortLineHeight)
        # The LOD gate, and it FADES rather than pops: value text is
        # 0.9x the pin font, so it turns to noise a little before the
        # labels do, and a cell that vanished between one wheel notch
        # and the next would read as a bug.
        if rowHeight <= 0.0:
            continue
        pixels = rowHeight * view.zoom
        if pixels < minPixelHeight:
            continue
        lodAlpha = min(1.0, (pixels - minPixelHeight) / max(1e-6, minPixelHeight))
        lodAlpha = max(0.0, lodAlpha)
        if lodAlpha <= 0.01:
            continue
        try:
            position = Gf.Vec2d(node.position)
            bounds = Gf.Range2d(position, position + Gf.Vec2d(node.size))
        except Exception:
            continue
        if not view._isVisible(bounds):
            continue

        # Cells ride the node's own z-order band, exactly as its text and
        # icons do, so they raise together when a node is brought forward.
        depth = node_content_depth(node.zOrder)
        radius = None
        for row, rect, subs, h in view._valueRowGeometry(node):
            if radius is None:
                radius = round(valueCell.corner_radius(h), 3)
            innerPad = valueCell.INNER_PAD_RATIO * h
            rowHovered = hover is not None and hover[0] == nodeId and (
                hover[1] == row.property_name
            )
            rowActive = (
                editing is not None
                and editing[0] == nodeId
                and editing[1] == row.property_name
            ) or (
                munging is not None
                and munging["nodeId"] == nodeId
                and munging["prop"] == row.property_name
            )
            fill, stroke, textColor, italic = view._valueStateColors(
                row, theme, rowHovered, rowActive
            )
            if flash is not None and flash[0] == nodeId and (
                flash[1] == row.property_name
            ):
                fill = theme.fillError if flash[3] == "error" else theme.fillAccent
                stroke = theme.error if flash[3] == "error" else theme.accent

            if fill is not None:
                valueCell.append_quad(
                    quadBatches[(radius, stroke)],
                    legacy_symbol("NodeVertex"),
                    rect,
                    depth,
                    fill,
                    lodAlpha,
                )
            elif stroke is not None:
                valueCell.append_quad(
                    quadBatches[(radius, stroke)],
                    legacy_symbol("NodeVertex"),
                    rect,
                    depth,
                    (0.0, 0.0, 0.0, 0.0),
                    lodAlpha,
                )

            # A time-sampled value carries the animation tint down its
            # leading edge: present at a glance, not another word to read.
            if row.state == noodlesValues.STATE_ANIMATED:
                valueCell.append_quad(
                    quadBatches[(0.0, None)],
                    legacy_symbol("NodeVertex"),
                    valueCell.Rect(
                        rect.left, rect.top, rect.left + 0.05 * h, rect.bottom
                    ),
                    depth,
                    theme.textAnimated,
                    lodAlpha,
                )
            if row.has_swatch and len(row.components) >= 3:
                valueCell.append_quad(
                    quadBatches[(round(radius * 0.7, 3), theme.hairline)],
                    legacy_symbol("NodeVertex"),
                    valueCell.swatch_rect(rect, h),
                    depth,
                    (
                        float(row.components[0]),
                        float(row.components[1]),
                        float(row.components[2]),
                        1.0,
                    ),
                    lodAlpha,
                )

            baselineY = view._valueBaselineY(rect)
            tabular = row.kind in (
                noodlesValues.KIND_FLOAT,
                noodlesValues.KIND_INT,
            )
            # One measure for the whole row, and a TABULAR one for
            # numbers, so the fitted format matches the widths the
            # glyphs will actually be set at.
            if tabular:
                def rowMeasure(text, _size=fontSize):
                    return view._valueTextWidth(text, _size, True)
            else:
                rowMeasure = measure
            axisLabels = (
                valueCell.COLOR_AXIS_LABELS
                if row.has_swatch
                else valueCell.AXIS_LABELS
            )
            hairWidth = max(0.6 / max(view.zoom, 1e-3), 0.012 * h)

            # Fit ONCE for the whole value, so every component of a
            # vector is shown to the same number of decimals -- and once
            # per value, not once per frame (see _valueFittedTexts).
            slotWidth = valueCell.text_max_width(subs[0], h)
            if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                slotWidth -= 0.20 * h * 2.2
            if row.count > 1:
                slotWidth -= max(
                    view._valueAxisWidth(label, fontSize * 0.8)
                    for label in axisLabels[: row.count]
                ) + innerPad * 0.5
            texts = view._valueFittedTexts(
                nodeId, row, slotWidth, fontSize, rowMeasure
            )

            for index, sub in enumerate(subs):
                componentHovered = rowHovered and hover[2] == index
                # Components of one value live in ONE pill, divided by
                # hairlines rather than by gaps, so a vector reads as a
                # single value with parts.
                if index > 0:
                    valueCell.append_quad(
                        quadBatches[(0.0, None)],
                        legacy_symbol("NodeVertex"),
                        valueCell.Rect(
                            sub.left - hairWidth * 0.5,
                            rect.top + 0.12 * (rect.bottom - rect.top),
                            sub.left + hairWidth * 0.5,
                            rect.bottom - 0.12 * (rect.bottom - rect.top),
                        ),
                        depth,
                        theme.hairline,
                        lodAlpha,
                    )
                if componentHovered and fill is not None and not rowActive:
                    valueCell.append_quad(
                        quadBatches[(radius, None)],
                        legacy_symbol("NodeVertex"),
                        sub,
                        depth,
                        theme.fillHover,
                        lodAlpha * 0.8,
                    )

                if row.kind == noodlesValues.KIND_BOOL:
                    view._appendBoolSwitch(
                        quadBatches, sub, h, depth, row, theme, lodAlpha
                    )
                    continue

                available = valueCell.text_max_width(sub, h)
                caretSize = 0.20 * h
                if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                    available -= caretSize * 2.2
                axisWidth = 0.0
                if row.count > 1:
                    axisWidth = (
                        view._valueAxisWidth(
                            axisLabels[min(index, 3)], fontSize * 0.8
                        )
                        + innerPad * 0.5
                    )
                    available -= axisWidth
                if available <= 0.0:
                    continue
                if (
                    editing is not None
                    and editing[0] == nodeId
                    and editing[1] == row.property_name
                    and editing[2] == index
                ):
                    continue  # the editor is drawing this one

                if row.count > 1:
                    axisColor = valueCell.AXIS_COLORS[min(index, 3)]
                    key = (axisColor[0], axisColor[1], axisColor[2],
                           0.85 * lodAlpha)
                    view._emitValueText(
                        axisLabels[min(index, 3)],
                        sub.left + innerPad,
                        baselineY,
                        depth,
                        fontSize * 0.8,
                        textBatches[key],
                    )

                text = texts[index] if index < len(texts) else ""
                if not text:
                    continue
                if row.kind in (
                    noodlesValues.KIND_TEXT,
                    noodlesValues.KIND_TOKEN_ENUM,
                ):
                    textX = sub.left + innerPad
                else:
                    textX = (
                        sub.right
                        - innerPad
                        - view._valueTextWidth(text, fontSize, tabular)
                    )
                key = (textColor[0], textColor[1], textColor[2],
                       textColor[3] * lodAlpha)
                view._emitValueText(
                    text,
                    textX,
                    baselineY,
                    depth,
                    fontSize,
                    textBatches[key],
                    tabular=tabular,
                    italic=italic,
                )
                if row.kind == noodlesValues.KIND_TOKEN_ENUM:
                    cx = sub.right - innerPad - caretSize * 0.6
                    cy = (rect.top + rect.bottom) * 0.5
                    view._appendChevron(
                        caretVertices, cx, cy, caretSize, depth,
                        textColor, lodAlpha
                    )

    for (radius, stroke), vertices in quadBatches.items():
        if not vertices:
            continue
        view._drawNodeVertices(
            vertices,
            projection,
            radius,
            stroke if stroke is not None else (0.0, 0.0, 0.0, 0.0),
            generation=-1,
        )
    if caretVertices:
        view._drawNodeVertices(
            caretVertices, projection, 0.0, (0.0, 0.0, 0.0, 0.0), generation=-1
        )
    for color, vertices in textBatches.items():
        if vertices:
            view.textRenderer.drawTextVertices(vertices, projection, color)

    if flash is not None:
        # Keep repainting while the flash is running; it is the only
        # animation this pass owns.
        view.update()


def _paintValueEditor(view):
    """Draw the keyboard editor INSIDE the cell it is editing.

    Not the screen-space TextInputWidget the rename uses: an editor that
    appears somewhere other than the value it edits, in a different font
    at a different size, is the thing that makes an inline field feel
    bolted on. This is the same pill, the same radius, the same padding
    and the same baseline as the cell it replaces, with the node's own
    selection accent as its outline -- so entering and leaving edit mode
    moves nothing.
    """
    target = view._valueEditTarget
    if target is None:
        return
    node = view.nodes.get(target[0])
    if node is None:
        return
    rect = view._valueCellRectFor(node, target[1], target[2])
    if rect is None:
        return
    if not view.textRenderer or not view.textRenderer.font_atlas:
        return

    row = view._findValueRow(target[0], target[1])
    theme = view._cachedValueTheme
    projection = view._worldSpaceProjectionMatrix()
    depth = node_content_depth(node.zOrder) + 0.0002
    rowHeight = float(node.layoutPortLineHeight) or 1.0
    innerPad = valueCell.INNER_PAD_RATIO * rowHeight
    fontSize = view._valueFontSize()
    radius = round(valueCell.corner_radius(rowHeight), 3)
    error = view._valueEditError

    quads = []
    valueCell.append_quad(
        quads,
        legacy_symbol("NodeVertex"),
        rect,
        depth,
        theme.fillError if error else theme.fillEdit,
    )
    view._drawNodeVertices(
        quads,
        projection,
        radius,
        theme.error if error else theme.accent,
        generation=-1,
    )

    text = view._valueEditInput.text
    tabular = row is not None and row.kind in (
        noodlesValues.KIND_FLOAT,
        noodlesValues.KIND_INT,
    )
    leftAligned = row is not None and row.kind in (
        noodlesValues.KIND_TEXT,
        noodlesValues.KIND_TOKEN_ENUM,
    )
    width = view._valueTextWidth(text, fontSize, tabular)
    if leftAligned:
        textX = rect.left + innerPad
    else:
        textX = rect.right - innerPad - width
    baselineY = view._valueBaselineY(rect)

    # Select-all on entry, shown the way a text field shows it.
    overlays = []
    if view._valueEditFresh and text:
        valueCell.append_quad(
            overlays,
            legacy_symbol("NodeVertex"),
            valueCell.Rect(
                textX - innerPad * 0.25,
                rect.top + (rect.bottom - rect.top) * 0.14,
                textX + width + innerPad * 0.25,
                rect.bottom - (rect.bottom - rect.top) * 0.14,
            ),
            depth,
            theme.selection,
        )

    cursorText = text[: view._valueEditInput.cursor_position]
    cursorX = textX + view._valueTextWidth(cursorText, fontSize, tabular)
    view._valueEditInput._update_cursor_blink()
    if view._valueEditInput._cursor_visible:
        caretWidth = max(1.2 / max(view.zoom, 1e-3), 0.015 * rowHeight)
        valueCell.append_quad(
            overlays,
            legacy_symbol("NodeVertex"),
            valueCell.Rect(
                cursorX,
                rect.top + (rect.bottom - rect.top) * 0.16,
                cursorX + caretWidth,
                rect.bottom - (rect.bottom - rect.top) * 0.16,
            ),
            depth,
            theme.caret,
        )
    if overlays:
        view._drawNodeVertices(
            overlays, projection, 0.0, (0.0, 0.0, 0.0, 0.0), generation=-1
        )

    glyphs = []
    view._emitValueText(
        text, textX, baselineY, depth, fontSize, glyphs, tabular=tabular
    )
    if glyphs:
        view.textRenderer.drawTextVertices(
            glyphs, projection, theme.textNormal
        )
    # The caret blinks, so this pass owns an animation while it is open.
    view.update()


def _emitValueText(view, text, x, baselineY, depth, size, out,
                   tabular=False, italic=0.0):
    """Append one value string's glyphs to *out*, starting at *x*.

    ``tabular`` sets digits on the widest digit's advance, so a number
    does not shuffle sideways while it is being dragged. ``italic``
    shears the glyphs around the baseline -- the atlas has no italic
    face, and a read-only value has to look different from an editable
    one at a glance.
    """
    if not text:
        return 0.0
    renderer = view.textRenderer
    if not tabular and italic <= 0.0:
        renderer.generateTextVertices(
            text, x, baselineY, depth, size, out, node_index=-1.0
        )
        return renderer.calculateTextWidth(text, size)

    advance = view._valueDigitAdvance(size) if tabular else 0.0
    cursor = x
    for ch in text:
        verts, natural = view._valueGlyph(ch, size)
        step = advance if (tabular and ch.isdigit()) else natural
        ox = cursor + (step - natural) * 0.5
        oy = baselineY
        # Glyph vertices are flat floats, 6 per vertex (x, y, z, u, v,
        # nodeIndex); the cached glyph is translated into place, and an
        # italic lean is two lines about the baseline.
        if italic > 0.0:
            for i in range(0, len(verts), 6):
                gx = verts[i] + ox
                gy = verts[i + 1] + oy
                out.append(gx + (baselineY - gy) * italic)
                out.append(gy)
                out.append(verts[i + 2] + depth)
                out.append(verts[i + 3])
                out.append(verts[i + 4])
                out.append(verts[i + 5])
        else:
            for i in range(0, len(verts), 6):
                out.append(verts[i] + ox)
                out.append(verts[i + 1] + oy)
                out.append(verts[i + 2] + depth)
                out.append(verts[i + 3])
                out.append(verts[i + 4])
                out.append(verts[i + 5])
        cursor += step
    return cursor - x


def _valueTextBand(view, rowHeight):
    """(top offset, height) of the value pill within a row band.

    The pill is hung off the ROW LABEL's baseline, not centred in the
    row: the C++ text pass draws the property name and, under it, the
    ``(type)`` line, and a value floating between the two reads as a
    separate widget. Sharing the name's baseline is what makes the value
    look like the end of the same line.
    """
    atlas = view.textRenderer.font_atlas if view.textRenderer else None
    if atlas is None:
        return (valueCell.CELL_TOP_RATIO * rowHeight,
                valueCell.CELL_HEIGHT_RATIO * rowHeight)
    pinFont = float(view._cachedNodePinFontSize)
    typeFont = (
        float(legacy_symbol("NoodlesConfig").get("nodePinTypeFontSize", 26.0))
        if legacy_symbol("NoodlesConfig").get("showPinTypeLabels", True)
        else 0.0
    )
    size = view._valueFontSize()
    # Pure function of (row height, fonts, atlas metrics): one entry per
    # distinct row height, recomputed only when a dependency changes.
    key = (
        round(float(rowHeight), 3),
        pinFont,
        typeFont,
        size,
        float(atlas.lineHeight),
        float(atlas.ascender),
        float(atlas.descender),
    )
    band = view._valueTextBandCache.get(key)
    if band is not None:
        return band
    lineHeight = float(atlas.lineHeight)
    block = (pinFont + typeFont) * lineHeight
    baseline = (rowHeight - block) * 0.5 + float(atlas.ascender) * pinFont
    pad = 0.07 * rowHeight
    top = baseline - float(atlas.ascender) * size - pad
    height = (float(atlas.ascender) - float(atlas.descender)) * size + 2.0 * pad
    # Never taller than the row it belongs to.
    if height > rowHeight:
        height = rowHeight
    if top < 0.0:
        top = 0.0
    if top + height > rowHeight:
        top = max(0.0, rowHeight - height)
    band = (top, height)
    view._valueTextBandCache[key] = band
    return band


def _valueGlyph(view, ch, size):
    """``(vertices, width)`` for one character, generated at the origin.

    A value is the same dozen glyphs every frame, so each character is
    generated once per size at (0, 0, 0) and translated by the caller --
    the two C++ calls per glyph per frame this replaces were the
    single largest cost in the value paint pass. Cleared with the other
    paint caches when the atlas is regenerated.
    """
    key = (ch, size)
    cached = view._valueGlyphCache.get(key)
    if cached is None:
        renderer = view.textRenderer
        glyph = []
        renderer.generateTextVertices(
            ch, 0.0, 0.0, 0.0, size, glyph, node_index=-1.0
        )
        cached = (tuple(glyph), renderer.calculateTextWidth(ch, size))
        view._valueGlyphCache[key] = cached
    return cached


def _valueBaselineY(view, rect):
    """Text baseline inside a value pill."""
    atlas = view.textRenderer.font_atlas
    size = view._valueFontSize()
    blockHeight = (float(atlas.ascender) - float(atlas.descender)) * size
    top = rect.top + ((rect.bottom - rect.top) - blockHeight) * 0.5
    return top + float(atlas.ascender) * size


def _appendBoolSwitch(view, quadBatches, sub, h, depth, row, theme, alpha):
    """A rounded checkbox drawn in the PIN's colours, not its own.

    On is the connected-port fill, off is the disconnected one, so a
    boolean reads the same way a wired port does two columns to the left.
    """
    innerPad = valueCell.INNER_PAD_RATIO * h
    side = 0.46 * h
    top = sub.top + ((sub.bottom - sub.top) - side) * 0.5
    box = valueCell.Rect(
        sub.right - innerPad - side, top, sub.right - innerPad, top + side
    )
    on = bool(row.components[0])
    radius = round(side * 0.28, 3)
    valueCell.append_quad(
        quadBatches[(radius, theme.hairline)],
        legacy_symbol("NodeVertex"),
        box,
        depth,
        theme.boolOn if on else theme.boolOff,
        alpha,
    )
    if on:
        inset = side * 0.28
        valueCell.append_quad(
            quadBatches[(round(radius * 0.6, 3), None)],
            legacy_symbol("NodeVertex"),
            valueCell.Rect(
                box.left + inset,
                box.top + inset,
                box.right - inset,
                box.bottom - inset,
            ),
            depth,
            (0.10, 0.11, 0.09, 0.9),
            alpha,
        )


def _appendChevron(view, out, cx, cy, size, depth, color, alpha):
    """The small downward chevron that says "this opens a list"."""
    view._appendTriangleVertices(
        out,
        [
            (cx - size * 0.55, cy - size * 0.28),
            (cx + size * 0.55, cy - size * 0.28),
            (cx, cy + size * 0.34),
        ],
        depth,
        (color[0], color[1], color[2], color[3] * alpha),
    )


def _paintNodeRename(view):
    """Draw the inline editor over the node being renamed."""
    node = view.nodes.get(view._renamingNodeId)
    if node is None:
        return

    projection = view._screenSpaceProjectionMatrix()
    x = (float(node.position[0]) - view.panX) * view.zoom
    y = (float(node.position[1]) - view.panY) * view.zoom
    # Never narrower than the text: a zoomed-out node is a few pixels wide
    # and the field has to stay legible to be typed into.
    minWidth, _ = view._renameInput.calculate_size(view.textRenderer)
    width = max(float(node.size[0]) * view.zoom, minWidth)

    view._renameInput.render(
        view.textRenderer,
        x,
        y,
        MAX_RENDER_DEPTH,
        width,
        legacy_symbol("NodeVertex"),
        view._drawNodeVertices,
        projection,
    )


def _paintMungLadder(view):
    """A small readout of the step the drag is currently using.

    ladder without the ladder: the multiplier is the only part
    a user needs while dragging, and it belongs next to the cursor rather
    than in a status bar on the other side of the window.
    """
    state = view._mungState
    if state is None or not view._mungActive or not state["mungable"]:
        return
    if not view.textRenderer or not view.textRenderer.font_atlas:
        return
    modifiers = legacy_symbol("QtWidgets").QApplication.keyboardModifiers()
    multiplier = noodlesValues.mung_multiplier(
        bool(modifiers & QtCore.Qt.ShiftModifier),
        bool(modifiers & QtCore.Qt.ControlModifier),
    )
    step = state["step"] * multiplier
    label = "x%g /px" % step

    theme = view._cachedValueTheme
    projection = view._screenSpaceProjectionMatrix()
    atlas = view.textRenderer.font_atlas
    fontSize = 15.0
    padX, padY = 9.0, 5.0
    width = view.textRenderer.calculateTextWidth(label, fontSize) + padX * 2.0
    height = fontSize * (atlas.ascender - atlas.descender) + padY * 2.0
    # The view's own last mouse position, not QCursor.pos(): the drag is
    # driven by the events this widget received, and a synthetic one (a
    # test, a tablet remap) never moved the OS pointer at all.
    cursor = view.lastMousePos or view.mapFromGlobal(QtGui.QCursor.pos())
    x = min(max(0.0, float(cursor.x()) + 18.0), max(0.0, view.width() - width))
    y = min(max(0.0, float(cursor.y()) - height - 12.0),
            max(0.0, view.height() - height))

    quads = []
    valueCell.append_quad(
        quads,
        legacy_symbol("NodeVertex"),
        valueCell.Rect(x, y, x + width, y + height),
        MAX_RENDER_DEPTH,
        theme.popupBg,
    )
    view._drawNodeVertices(quads, projection, 4.0, theme.hairline, generation=-1)

    glyphs = []
    view.textRenderer.generateTextVertices(
        label,
        x + padX,
        y + padY + atlas.ascender * fontSize,
        MAX_RENDER_DEPTH,
        fontSize,
        glyphs,
        node_index=-1.0,
    )
    if glyphs:
        view.textRenderer.drawTextVertices(
            glyphs, projection, theme.accent, disable_depth=True
        )


def _paintTokenPopup(view):
    """Draw the allowedTokens list under the cell, in the node's style."""
    popup = view._tokenPopup
    if popup is None:
        return
    if not view.textRenderer or not view.textRenderer.font_atlas:
        return

    theme = view._cachedValueTheme
    projection = view._screenSpaceProjectionMatrix()
    rect = popup["rect"]
    # The list is set in the SAME on-screen size as the cell it dropped
    # out of, so the token you are choosing looks like the token you are
    # replacing rather than like an operating-system menu.
    onScreenFont = view._valueFontSize() * view.zoom
    view._tokenPopupList.font_size = max(11.0, min(onScreenFont, 30.0))
    view._tokenPopupList.item_height = (
        view._tokenPopupList.font_size * 1.55
    )
    view._tokenPopupList.padding = view._tokenPopupList.font_size * 0.45
    width, height = view._tokenPopupList.calculate_size(view.textRenderer)
    width = max(width, valueCell.rect_width(rect) * view.zoom)
    x = (rect.left - view.panX) * view.zoom
    y = (rect.bottom - view.panY) * view.zoom + 3.0
    # Clamped into the widget, and flipped above the cell rather than
    # drawn off the bottom edge.
    x = max(0.0, min(x, max(0.0, float(view.width()) - width)))
    if y + height > float(view.height()):
        y = max(0.0, (rect.top - view.panY) * view.zoom - height - 3.0)

    background = []
    valueCell.append_quad(
        background,
        legacy_symbol("NodeVertex"),
        valueCell.Rect(x, y, x + width, y + height),
        MAX_RENDER_DEPTH,
        theme.popupBg,
    )
    view._drawNodeVertices(
        background, projection, 6.0, theme.hairline, generation=-1
    )
    view._tokenPopupList.render(
        view.textRenderer,
        x,
        y,
        MAX_RENDER_DEPTH,
        width,
        legacy_symbol("NodeVertex"),
        view._drawNodeVertices,
        projection,
    )


def _activeValueFlash(view):
    """The running commit/error flash, or None once it has expired."""
    flash = view._valueFlash
    if flash is None:
        return None
    if time.time() >= flash[2]:
        view._valueFlash = None
        return None
    return flash


def _startValueFlash(view, nodeId, propertyName, kind="commit"):
    """A short tint on the cell that just committed (or was rejected)."""
    duration = 0.12 if kind == "commit" else 0.6
    view._valueFlash = (nodeId, propertyName, time.time() + duration, kind)
    view.update()


def paintHotkeyPopup(view):
    if not view.shaderLibrary or not view.fontAtlas:
        return

    projection = view._screenSpaceProjectionMatrix()
    view.hotkeyPopup.render(
        view.textRenderer,
        legacy_symbol("NodeVertex"),
        view._drawNodeVertices,
        projection,
        view.width(),
        view.height(),
        MAX_RENDER_DEPTH,
    )


def paintNavigationHotkeyPopup(view):
    if not view.shaderLibrary or not view.fontAtlas:
        return

    projection = view._screenSpaceProjectionMatrix()
    view.navigationHotkeyPopup.render(
        view.textRenderer,
        legacy_symbol("NodeVertex"),
        view._drawNodeVertices,
        projection,
        view.width(),
        view.height(),
        MAX_RENDER_DEPTH,
    )


def _paintHotkeyPopups(view):
    """Render hotkey popup overlays."""
    if view.hotkeyPopup.is_visible:
        view.paintHotkeyPopup()
    if view.navigationHotkeyPopup.is_visible:
        view.paintNavigationHotkeyPopup()
