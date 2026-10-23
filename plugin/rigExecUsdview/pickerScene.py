"""Read control pickers and manage their controls' session visibility.

The picker used to be a sidecar JSON file found by guessing at the
stage's filename. Now it is scene data: `RigExecPicker` prims composed
into the rig's layer stack, discovered BY TYPE, so a stage carrying three
characters gets three pickers with no configuration anywhere.

What that buys, beyond tidiness:

  * an animator overrides one button with an `over` on one prim, and
    `active = false` removes a button, both in a layer they own;
  * button targets are relationships, so USD remaps them through a
    rename or a reparent, where a baked path string would go dead;
  * liveness is just "did the targets resolve against this stage".

This module turns those prims into the same `Panel` and `Button` objects
`pickerModel` already builds from JSON, so the drawing, hit-testing and
selection code has exactly one path through it.
"""
from pxr import Ar, Sdf, Tf, Usd, UsdGeom

import pickerModel

# Resolved once. `IsA` takes a TfType, and looking it up per prim during
# a traversal of a few thousand is the kind of thing that makes a panel
# feel slow for no reason.
PICKER_TYPE = Tf.Type.FindByName("RigExecPicker")
RIG_TYPE = Tf.Type.FindByName("RigExecRoot")
CONTROL_TYPE = Tf.Type.FindByName("RigExecControl")
JOINT_TYPE = Tf.Type.FindByName("RigExecJoint")
PANEL_TYPE = Tf.Type.FindByName("RigExecPickerPanel")
BUTTON_TYPE = Tf.Type.FindByName("RigExecPickerButton")
# The rig root is a scope, not graph: a picker may sit inside it.
PICKER_ROOT_SAFE = "RigExecRoot"


def _mirror_of(prim):
    """The path of the button this one mirrors, or "".

    Authored from the studio picker's own <mirror> element -- a pairing
    the rig author made, carried across rather than derived from L_/R_ in
    a name, because it also pairs the buttons whose names carry no side.
    """
    rel = prim.GetRelationship("rigExec:picker:mirror")
    targets = rel.GetTargets() if rel else []
    return str(targets[0]) if targets else ""


def _v(prim, name, default=None):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        return default
    value = attr.Get()
    return default if value is None else value


def _rgba(value, default=(128, 128, 128, 255)):
    """Schema colours are 0-1; Button and QColor want 0-255."""
    if value is None:
        return list(default)
    return [int(round(max(0.0, min(1.0, float(c))) * 255.0))
            for c in tuple(value)[:4]]


def _pairs(value):
    return [(float(p[0]), float(p[1])) for p in (value or [])]

def _background_image(panel):
    asset = _v(panel, "ui:backgroundImage")
    if not isinstance(asset, Sdf.AssetPath) or not asset.resolvedPath:
        return b""
    opened = Ar.GetResolver().OpenAsset(Ar.ResolvedPath(asset.resolvedPath))
    return opened.GetBuffer() if opened else b""


def _button_record(prim, panel_id):
    """One RigExecPickerButton as the record `Button` expects."""
    pos = _v(prim, "ui:position", (0.0, 0.0))
    size = _v(prim, "ui:size", (10.0, 10.0))

    controls = prim.GetRelationship("rigExec:picker:controls")
    targets = [str(p) for p in controls.GetTargets()] if controls else []
    # `objects` is what separates a DEAD button from a DECORATION: a
    # backdrop names nothing, while a button whose control was deleted
    # named something that is no longer there. The authored-ness of the
    # relationship is the only thing that tells those apart.
    objects = []
    if controls and controls.HasAuthoredTargets():
        objects = [prim.GetName()]

    attr_target = None
    attr_rel = prim.GetRelationship("rigExec:picker:attribute")
    attr_paths = attr_rel.GetTargets() if attr_rel else []
    if attr_paths:
        path = attr_paths[0]
        attr_target = {
            "path": str(path.GetPrimPath()),
            "attr": path.name,
            # Labels are authored in the rig's own order, so nothing here
            # is inverted; the converter reversed them once at bake time
            # rather than shipping a flag to be re-interpreted.
            "enum": [str(t) for t in
                     (_v(prim, "rigExec:picker:attributeLabels") or [])],
            "invert": False,
        }

    dial = None
    dial_rel = prim.GetRelationship("rigExec:picker:modeDial")
    dial_paths = dial_rel.GetTargets() if dial_rel else []
    if dial_paths:
        dial = str(dial_paths[0])

    # The bezier outline is flattened THREE points to a knot on the way
    # in, because USD has no ragged array. Three, not four: a picker knot
    # is (on-curve, out-tangent, in-tangent), with the following knot's
    # in-tangent completing each span. Regrouping by four silently shears
    # every silhouette, and it still draws, which is what makes it worth
    # saying out loud here.
    flat = _pairs(_v(prim, "ui:bezier"))
    bezier = [flat[i:i + 3] for i in range(0, len(flat) - 2, 3)]

    command = str(_v(prim, "rigExec:picker:command", ""))
    if command:
        kind = "commandButton"
    elif attr_target:
        kind = "attributeButton"
    else:
        kind = "selectButton"

    return {
        "id": prim.GetName(),
        "type": kind,
        "parent": panel_id,
        "z": float(_v(prim, "ui:depth", 0.0)),
        "x": float(pos[0]), "y": float(pos[1]),
        "w": float(size[0]), "h": float(size[1]),
        "shape": str(_v(prim, "ui:shape", "rectangle")),
        "roundness": float(_v(prim, "ui:roundness", 0.0)),
        "direction": str(_v(prim, "ui:direction", "top")),
        "leftSlope": float(_v(prim, "ui:slopeLeft", 0.0)),
        "rightSlope": float(_v(prim, "ui:slopeRight", 0.0)),
        "polygon": _pairs(_v(prim, "ui:polygon")),
        # The button this one mirrors, as a prim path. Authored from the
        # studio picker's own <mirror> element -- a pairing the rig
        # author made, which is why it is carried rather than derived
        # from L_/R_ in a name.
        "mirror": _mirror_of(prim),
        "bezier": bezier,
        "rotation": float(_v(prim, "ui:rotation", 0.0)),
        "alternate": bool(_v(prim, "ui:alternate", False)),
        "fill": _rgba(_v(prim, "ui:fill")),
        "stroke": _rgba(_v(prim, "ui:stroke"), (0, 0, 0, 200)),
        "strokeWidth": float(_v(prim, "ui:strokeWidth", 1.0)),
        "text": (str(_v(prim, "ui:text", ""))
                 or command.replace("_", " ").title()),
        "fontSize": float(_v(prim, "ui:fontSize", 8.0)),
        "bold": bool(_v(prim, "ui:bold", False)),
        "hAlign": str(_v(prim, "ui:textAlign", "center")),
        "textColor": _rgba(_v(prim, "ui:textColor"), (0, 0, 0, 255)),
        "valueColor": _rgba(_v(prim, "ui:valueColor"), (0, 0, 0, 255)),
        "objects": objects,
        "targets": targets,
        "attrTarget": attr_target,
        # A mode with no dial cannot be tested, so it is not a mode.
        "mode": (str(_v(prim, "rigExec:picker:mode", "")) or None
                 if dial else None),
        "dial": dial,
    }


def _order(prim):
    """Sort key: explicit ui:order first, then namespace order."""
    return int(_v(prim, "ui:order", 0) or 0)


def read(picker_prim, live_paths=None):
    """One RigExecPicker prim as a `pickerModel.Picker`."""
    panels, items = [], []
    # GetChildren() filters on the default predicate, which is what
    # makes `active = false` on a panel or a button simply remove it.
    children = [c for c in picker_prim.GetChildren()
                if c.IsA(PANEL_TYPE)]
    for panel in sorted(children, key=_order):
        panel_id = str(panel.GetPath())
        size = _v(panel, "ui:size", (400.0, 600.0))
        panels.append({
            "id": panel_id,
            "label": str(_v(panel, "ui:label", "")) or panel.GetName(),
            "x": 0.0, "y": 0.0,
            "w": float(size[0]), "h": float(size[1]),
            "fill": _rgba(_v(panel, "ui:background"), (41, 41, 41, 255)),
            "backgroundImage": _background_image(panel),
        })
        items.extend(
            _button_record(b, panel_id) for b in panel.GetChildren()
            if b.IsA(BUTTON_TYPE))

    picker = pickerModel.Picker({"panels": panels, "items": items},
                                live_paths)
    picker.name = (str(_v(picker_prim, "ui:label", ""))
                   or picker_prim.GetName())
    picker.path = str(picker_prim.GetPath())
    picker.rig_path = rig_path_for(picker_prim)
    return picker


def find(stage):
    """Every picker on the stage, in tab order.

    BY SCHEMA, never by path or filename. `IsA` rather than a type-name
    string compare, so a studio that subclasses RigExecPicker for its own
    metadata is still found by this.

    A picker may live ANYWHERE: beside the rig, under the asset root, or
    inside the RigExecRoot itself if a studio would rather ship one prim
    that holds the entire rig deliverable. That is a placement decision,
    and this deliberately does not make it for them.

    What the walk does assume is that a picker is never a CHILD of rig
    graph: there is no picker inside a joint, a control, a solver or a
    mover. Pruning there skips the 900-odd prims that make up the graph
    while still finding a picker parented to the rig root or to a scope
    under it. It also does not descend into a picker it has found, since
    everything below one is panels and buttons.
    """
    if stage is None:
        return []
    found = []
    walk = iter(Usd.PrimRange(stage.GetPseudoRoot()))
    for prim in walk:
        if prim.IsA(PICKER_TYPE):
            found.append(prim)
            walk.PruneChildren()
        elif _IsRigGraph(prim):
            walk.PruneChildren()
    return sorted(found, key=_order)


def _IsRigGraph(prim):
    """A rig graph prim: joints, controls, solvers, movers, weights.

    Everything a picker is not, and nothing a picker can be inside of.
    The RigExecRoot itself is NOT one of these -- it is the scope the
    graph lives in, and a picker is allowed to sit in it.
    """
    name = prim.GetTypeName()
    return bool(name) and name != PICKER_ROOT_SAFE and str(name).startswith(
        "RigExec") and not str(name).startswith("RigExecPicker")


def rig_path_for(picker_prim):
    """The authored rig target, including a target that no longer resolves."""
    rel = picker_prim.GetRelationship("rigExec:picker:rig")
    targets = rel.GetTargets() if rel else []
    return str(targets[0]) if targets else ""


def rig_for(picker_prim):
    """The rig a picker names, or None when absent or unresolved."""
    path = rig_path_for(picker_prim)
    if not path:
        return None
    prim = picker_prim.GetStage().GetPrimAtPath(path)
    return prim if prim and prim.IsValid() else None


def control_prims(stage, rig_path=""):
    """Active controls under the named scope; an absent scope means the stage."""
    if stage is None:
        return []
    root = stage.GetPrimAtPath(rig_path) if rig_path else stage.GetPseudoRoot()
    if not root or not root.IsActive():
        return []
    return [prim for prim in Usd.PrimRange(root) if prim.IsA(CONTROL_TYPE)]


def _session_visibility_spec(prim):
    """Whether the session layer holds a visibility value for `prim`.

    A cleared opinion can leave an empty spec behind; that is no opinion.
    """
    session = prim.GetStage().GetSessionLayer()
    path = prim.GetPath().AppendProperty(UsdGeom.Tokens.visibility)
    spec = session.GetAttributeAtPath(path)
    return bool(spec) and (spec.HasDefaultValue() or
                           session.GetNumTimeSamplesForPath(path) > 0)


def rig_hidden(prim, time=Usd.TimeCode.Default()):
    """True for a control the rig itself authors invisible.

    Ctrl Vis toggles only session opinions, so such a control stays hidden
    when shown and does not count against the toggle's tick.
    """
    if _session_visibility_spec(prim):
        return False
    attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
    return attr.Get(time) == UsdGeom.Tokens.invisible


def controls_visible(controls, time=Usd.TimeCode.Default()):
    """True when every control Ctrl Vis governs is effectively visible.

    Controls the rig authors invisible are left out; a collection with
    no governed control is not visible.
    """
    governed = [prim for prim in controls if not rig_hidden(prim, time)]
    return bool(governed) and all(
        UsdGeom.Imageable(prim).ComputeVisibility(time) != UsdGeom.Tokens.invisible
        for prim in governed)


def control_visibility_attributes(controls):
    """Visibility dependencies, including ancestors outside the rig's scope."""
    attributes, seen = [], set()
    for control in controls:
        prim = control
        while prim and not prim.IsPseudoRoot():
            path = prim.GetPath()
            if path in seen:
                break
            seen.add(path)
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                attributes.append(imageable.GetVisibilityAttr())
            prim = prim.GetParent()
    return attributes


def set_controls_visible(stage, controls, visible):
    """Hide with a session opinion on each control; show by clearing it.

    Only the session layer changes, so the rig's own invisible opinions on a
    control or an ancestor still hold after a show.
    """
    attributes = [UsdGeom.Imageable(prim).GetVisibilityAttr() for prim in controls]
    with Usd.EditContext(stage, stage.GetSessionLayer()), Sdf.ChangeBlock():
        for attr in attributes:
            if visible:
                attr.Clear()
            else:
                attr.Set(UsdGeom.Tokens.invisible)


def _live_under(root):
    """Selectable prims inside one rig.

    Controls AND joints: the biped's fingers are RigExecJoints posed
    through their own avars rather than controls, and a button that
    selects one is a working button.
    """
    return set(str(p.GetPath()) for p in Usd.PrimRange(root)
               if p.IsA(CONTROL_TYPE) or p.IsA(JOINT_TYPE))


def load_all(stage):
    """Every picker on the stage, ready to draw. May be empty.

    Each picker resolves its buttons against the rig it NAMES, so on a
    stage with two characters neither one's liveness depends on the
    other being loaded. A picker that names no rig falls back to the
    whole stage, which is the right answer for the single-character case
    and the only answer available without the relationship.
    """
    prims = find(stage)
    if not prims:
        return []
    live_by_rig = {}
    pickers = []
    for prim in prims:
        rig = rig_for(prim)
        key = rig_path_for(prim)
        if key not in live_by_rig:
            live_by_rig[key] = (_live_under(rig) if rig
                                else set() if key
                                else pickerModel.live_control_paths(stage))
        pickers.append(read(prim, live_by_rig[key]))
    return pickers
