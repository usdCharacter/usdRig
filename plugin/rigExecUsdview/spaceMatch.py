"""
Space matching: change a control's space without moving it.

A RigExecSpaceSwitch re-expresses its control's avars in the newly selected
space, so switching mid-pose moves the control -- the schema leaves keeping
it still to tools. The rig does the solving: every evaluation, each switch
publishes `<control>.spaces:matchValues`, the translate and rotate channels
(tx ty tz rx ry rz) that keep its control where it is for every space it
names, in source order (spaceMatchKernel.h). A switch is the space value
plus that space's row, authored as one edit.

Scale is never written, and neither is a channel the rig itself writes or
drives (ikfkMatch._Writable): a rotation-only space keeps its position, so
its translate row is the current one, and a breathing-driven translate stays
the rig's.
"""

from pxr import Sdf

import gizmoMath
import ikfkMatch

MATCH_VALUES = "spaces:matchValues"
_CHANNELS = ikfkMatch._TRANSLATE + ikfkMatch._ROTATE


class Switch(object):
    """One space switch: the attribute selecting its space, the control it
    places, and the channels of that control a match may write."""

    def __init__(self, active, control, writable):
        self.active = active        # Sdf.Path of the selector
        self.control = control      # Sdf.Path
        self.writable = writable    # channel names, in _CHANNELS order


# The last stage's switches, found in one walk: a click must not traverse.
_cache = {"stage": None, "switches": None}


def Switches(stage):
    """{selector path: Switch} for every space switch on `stage`."""
    if _cache["stage"] is stage and _cache["switches"] is not None:
        return _cache["switches"]
    written = set()
    found = []
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:moves")
        if rel:
            written.update(rel.GetTargets())
        if prim.GetTypeName() != "RigExecSpaceSwitch":
            continue
        active = prim.GetRelationship("rigExec:activeSpaceAttribute")
        target = prim.GetRelationship("rigExec:target")
        actives = active.GetTargets() if active else []
        targets = target.GetTargets() if target else []
        if actives and targets:
            found.append((actives, targets[0]))
    switches = {}
    for actives, control in found:
        writable = ikfkMatch._Writable(stage, control, _CHANNELS, written)
        for path in actives:
            switches[path] = Switch(path, control, writable)
    _cache["stage"] = stage
    _cache["switches"] = switches
    return switches


def Forget():
    """Drop the cached switches (after the rig's structure changes)."""
    _cache["stage"] = None
    _cache["switches"] = None


def SwitchTarget(stage, attrPath):
    """The control whose space `attrPath` selects -- the target of the
    RigExecSpaceSwitch reading it -- or None when no switch reads it."""
    switch = Switches(stage).get(Sdf.Path(str(attrPath)))
    return switch.control if switch is not None else None


def Plan(stage, control, spaceAttr, value, time, read=None):
    """{attribute path: value} that sets `spaceAttr` to `value` and keeps
    `control`'s posed frame where it is at `time`: the space value and the
    row the rig published for it. `read(path)` returns published values;
    it defaults to the host session drawing `stage`."""
    control = Sdf.Path(str(control))
    spaceAttr = Sdf.Path(str(spaceAttr))
    if read is None:
        read = ikfkMatch.PublishedValueReader(stage, time)
    if read is None:
        raise RuntimeError("no rig publishes space matches for this stage")
    values = read(control.AppendProperty(MATCH_VALUES))
    index = int(round(float(value)))
    stride = len(_CHANNELS)
    if not values or index < 0 or (index + 1) * stride > len(values):
        raise RuntimeError("%s publishes no match for space %d"
                           % (control.name, index))
    row = dict(zip(_CHANNELS, values[index * stride:(index + 1) * stride]))
    switch = Switches(stage).get(spaceAttr)
    writable = switch.writable if switch is not None else \
        ikfkMatch._Writable(stage, control, _CHANNELS, set())
    plan = {spaceAttr: float(value)}
    for name in writable:
        plan[control.AppendProperty("avars:" + name)] = row[name]
    return plan


def Apply(stage, plan, time, mode, undoStack=None, label="Switch space"):
    """Author `plan` as one edit and one undo entry, landing the way a drag
    does (`mode` is a gizmoMath write mode: keyed in animation mode)."""
    scope = None
    if undoStack is not None:
        import rigExecUndo
        scope = rigExecUndo.SpecScope(stage, list(plan), undoStack, label)
        scope.__enter__()
    try:
        writer = gizmoMath.Writer(stage, time, mode)
        for path, value in plan.items():
            attr = stage.GetAttributeAtPath(path)
            if attr:
                writer.Set(attr, value)
        writer.CommitToStage()
    except Exception as error:
        if scope is not None:
            scope.__exit__(type(error), error, None)
        raise
    if scope is not None:
        scope.__exit__(None, None, None)
