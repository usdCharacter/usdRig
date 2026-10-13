"""
Headless test for plugin/rigExecUsdview/markingMenuUI.py: the right-click
marking menu's own logic, on Qt's offscreen platform with a stage built
here.

  1. Counterparts: a sided name mirrors (L_/R_, _l_/_r_, suffixes) and a
     centre-line name does not; a mirrored control is found beside the
     original, under a mirrored parent, or by name anywhere.
  2. The selection context answers the Selection menu: a control, its
     space list with the active one checked, and nothing it cannot back.
  3. The painted ring: every label sits on its own side of the centre,
     long labels win over the sector they reach into, the overflow
     column hangs below the ring, and a submenu's labels keep clear of
     the circles that go back to its parents.

Usage: test_marking_menu_ui.py
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402
from pxr.Usdviewq.qt import QtCore, QtWidgets  # noqa: E402

import markingMenuDefs as defs  # noqa: E402
import markingMenuModel as mm  # noqa: E402
import markingMenuUI as ui  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


_LAYER = """#usda 1.0
def Xform "Rig" {
    def RigExecControl "M_Body" {
        def RigExecControl "L_Shldr" {
            def RigExecControl "L_UpArm" {}
        }
        def RigExecControl "R_Shldr" {
            def RigExecControl "R_UpArm" {}
        }
    }
    def RigExecControl "L_Arm" {
        custom double spaces:active = 1
    }
    def RigExecControl "R_Arm" {}
    def Scope "Face" {
        def RigExecControl "eye_l_ctrl" {}
    }
    def Scope "FaceR" {
        def RigExecControl "eye_r_ctrl" {}
    }
    def RigExecSpaceSwitch "L_Arm_space" {
        rel rigExec:target = </Rig/L_Arm>
        rel rigExec:activeSpaceAttribute = </Rig/L_Arm.spaces:active>
        uniform token[] rigExec:spaceLabels = ["world", "chest", "hips"]
    }
    def Xform "notAControl" {}
}
"""


def _Stage():
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(_LAYER)
    return Usd.Stage.Open(layer)


class _Menu(object):
    """What SelectionContext asks of the controller."""

    def __init__(self, stage):
        self._stage = stage

    def Stage(self):
        return self._stage

    def Time(self):
        return Usd.TimeCode.Default()

    def KnownLimbs(self):
        return None


def TestCounterparts():
    for name, other in (("L_Arm", "R_Arm"), ("R_UpArm", "L_UpArm"),
                        ("eye_l_ctrl", "eye_r_ctrl"),
                        ("thumb_R", "thumb_L"), ("LeftFoot", "RightFoot")):
        _Check(ui.CounterpartName(name) == other,
               "%s mirrors to %s: %r" % (name, other,
                                          ui.CounterpartName(name)))
    for name in ("M_Body", "Main", "Look", "Lid"):
        _Check(ui.CounterpartName(name) is None,
               "%s is on the centre line" % name)
    stage = _Stage()
    _Check(ui.CounterpartPath(stage, "/Rig/L_Arm") == Sdf.Path("/Rig/R_Arm"),
           "a pair beside each other")
    _Check(ui.CounterpartPath(stage, "/Rig/M_Body/L_Shldr/L_UpArm") ==
           Sdf.Path("/Rig/M_Body/R_Shldr/R_UpArm"),
           "under a mirrored parent")
    _Check(ui.CounterpartPath(stage, "/Rig/Face/eye_l_ctrl") ==
           Sdf.Path("/Rig/FaceR/eye_r_ctrl"),
           "by name anywhere when the parents do not mirror")
    _Check(ui.CounterpartPath(stage, "/Rig/M_Body") is None,
           "nothing for a centre control")
    _Check(ui.ControlOf(stage.GetPrimAtPath("/Rig/notAControl")) is None,
           "a prim outside any control is not one")
    _Check(ui.ControlOf(stage.GetPrimAtPath("/Rig/M_Body/L_Shldr")).GetPath()
           == Sdf.Path("/Rig/M_Body/L_Shldr"), "a control is its own")


def TestContext():
    stage = _Stage()
    context = ui.SelectionContext(_Menu(stage), ["/Rig/L_Arm",
                                                 "/Rig/notAControl"])
    _Check([p.GetName() for p in context.controls] == ["L_Arm"],
           "only controls are acted on")
    _Check(context.Test("control") and context.Test("hasSpaces"),
           "a control with spaces")
    _Check(not context.Test("limb"), "and no limb without the IK/FK finder")
    resolved = mm.Resolve(defs.SELECTION, context, mm.MODE_CLICK)
    space = resolved.Slot("NE")
    _Check(space is not None and space.enabled, "Space is offered")
    child = mm.ResolveChild(space.item, context, mm.MODE_CLICK)
    _Check([r.label for r in child.items] == ["world", "chest", "hips"],
           "the switch's own space names: %r"
           % [r.label for r in child.items])
    _Check([r.label for r in child.items if r.checked] == ["chest"],
           "the active space is checked")
    _Check(not resolved.Slot("NW").enabled,
           "FK/IK is greyed out when nothing is on a limb")
    plain = ui.SelectionContext(_Menu(stage), ["/Rig/R_Arm"])
    _Check(not plain.Test("hasSpaces"), "a control without a switch")


def TestPainter():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    painter = ui.MenuPainter(app.font())
    stage = _Stage()
    context = ui.SelectionContext(_Menu(stage), ["/Rig/L_Arm"])
    menu = mm.Resolve(defs.SELECTION, context, mm.MODE_CLICK)
    centre = (500.0, 400.0)
    rects = painter.ItemRects(menu, centre)
    _Check(len(rects) == len(menu.slots), "one label per ring item")
    sides = {"N": lambda r: r.bottom() <= centre[1],
             "S": lambda r: r.top() >= centre[1],
             "E": lambda r: r.left() >= centre[0],
             "W": lambda r: r.right() <= centre[0],
             "NE": lambda r: r.left() >= centre[0] and r.bottom() <= centre[1],
             "NW": lambda r: r.right() <= centre[0] and
             r.bottom() <= centre[1],
             "SE": lambda r: r.left() >= centre[0] and r.top() >= centre[1],
             "SW": lambda r: r.right() <= centre[0] and r.top() >= centre[1]}
    for direction, resolved in menu.slots.items():
        _Check(sides[direction](rects[resolved.id]),
               "%s sits on its own side of the centre" % direction)
    names = list(rects.items())
    for i, (a, ra) in enumerate(names):
        for b, rb in names[i + 1:]:
            _Check(not ra.intersects(rb), "%s and %s do not overlap"
                   % (a, b))
    # A point on the far right of the NE label is in the E sector, and the
    # label still wins.
    ne = rects["selection.space"]
    far = (ne.right() - 2.0, ne.center().y())
    _Check(mm.SectorFor(far[0] - centre[0], far[1] - centre[1]) == "E",
           "the far end of NE reaches into E's sector")
    _Check(painter.ItemAt(menu, centre, far).id == "selection.space",
           "and the label wins")
    rows, box = painter.ColumnRects(menu, centre)
    _Check(box is not None and box.top() > max(r.bottom()
                                               for r in rects.values()),
           "the overflow column hangs below the ring")
    row = rows["selection.usdviewMenu"]
    _Check(painter.ItemAt(menu, centre, (row.center().x(),
                                         row.center().y())).id ==
           "selection.usdviewMenu", "a column row is hit")
    _Check(painter.ItemAt(menu, centre, centre) is None,
           "the centre is no item")

    # A submenu opened East has its parent's back circle to its West,
    # right where its West label would sit.
    child = mm.Resolve(mm.Menu("child", [
        dict(id="c.w", label="A long way back west", dir="W"),
        dict(id="c.e", label="East", dir="E"),
        dict(id="c.n", label="North", dir="N")]), None, mm.MODE_CLICK)
    parent = (centre[0] - mm.RADIUS_PX, centre[1])
    reach = mm.BACK_RADIUS_PX
    circle = QtCore.QRectF(parent[0] - reach, parent[1] - reach,
                           2 * reach, 2 * reach)
    plain = painter.ItemRects(child, centre)
    _Check(plain["c.w"].intersects(circle),
           "unmoved, the West label would sit on the parent's circle")
    cleared = painter.ItemRects(child, centre, [parent])
    _Check(not cleared["c.w"].intersects(circle),
           "it slides off the circle")
    _Check(cleared["c.w"].right() <= centre[0] and
           mm.SectorFor(cleared["c.w"].center().x() - centre[0],
                        cleared["c.w"].center().y() - centre[1]) == "W",
           "and still reads as West from the centre")
    _Check(cleared["c.e"] == plain["c.e"] and cleared["c.n"] == plain["c.n"],
           "labels nowhere near a circle stay put")


def _Fan(depth, overflow):
    """Submenus in all eight directions, `depth` deep, with long labels
    and, when `overflow`, a column row at every level."""
    label = "A deliberately long submenu label"
    if depth == 0:
        items = [dict(id="leaf.%s" % d, label=label, dir=d)
                 for d in mm.FILL_ORDER]
    else:
        items = [dict(id="sub%d.%s" % (depth, d), label=label, dir=d,
                      kind=mm.SUBMENU, items=_Fan(depth - 1, overflow))
                 for d in mm.FILL_ORDER]
    if overflow:
        items.append(dict(id="row%d" % depth, label="usdview prim menu",
                          overflow=True))
    return items


def TestNothingDrawnOnACircle():
    # Every way down two levels, from every direction, long labels and a
    # column at every level: no label or row as drawn -- without any
    # sliding -- ever covers a back circle, and no circle sits inside the
    # ring.
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    painter = ui.MenuPainter(app.font())
    reach = mm.BACK_RADIUS_PX
    centre0 = (900.0, 600.0)
    checked = 0
    for overflow in (False, True):
        root = mm.Menu("fan", _Fan(2, overflow))
        for first in mm.FILL_ORDER:
            for second in mm.FILL_ORDER:
                now = [0.0]
                gesture = mm.Gesture(root, None, clock=lambda: now[0],
                                     blocker=painter.CoveredDirections)
                gesture.Press(*centre0)
                now[0] = 200.0
                gesture.Tick()
                for direction in (first, second):
                    if gesture.resolved.Slot(direction) is None:
                        break
                    dx, dy = mm.DIRECTION_VECTORS[direction]
                    c = gesture.centre
                    now[0] += 20.0
                    gesture.Move(c[0] + dx * 50.0, c[1] + dy * 50.0)
                    now[0] += 20.0
                    gesture.Move(c[0] + dx * (mm.RADIUS_PX + 1.0),
                                 c[1] + dy * (mm.RADIUS_PX + 1.0))
                    circles = gesture.parentCentres
                    rects = list(painter.ItemRects(gesture.resolved,
                                                   gesture.centre).items())
                    rows, box = painter.ColumnRects(
                        gesture.resolved, gesture.centre, circles)
                    if box is not None:
                        rects.append(("column", box))
                    for x, y in circles:
                        circle = QtCore.QRectF(x - reach, y - reach,
                                               2 * reach, 2 * reach)
                        _Check(mm.Distance((x, y), gesture.centre) >=
                               mm.RADIUS_PX + reach,
                               "%s/%s: a circle inside the ring"
                               % (first, second))
                        for name, rect in rects:
                            _Check(not rect.intersects(circle),
                                   "%s/%s: %s is drawn on a back circle"
                                   % (first, second, name))
                    checked += 1
    _Check(checked > 128, "opened %d menus" % checked)


def main():
    groups = [
        ("counterparts", TestCounterparts),
        ("context", TestContext),
        ("painter", TestPainter),
        ("nothing drawn on a circle", TestNothingDrawnOnACircle),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("MARKING_MENU_UI_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
