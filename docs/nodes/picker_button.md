# ![Picker Button](../../icons/picker_button.png) Picker Button

*One clickable shape: selects controls, flips a switch, or decorates.*

| | |
|---|---|
| **Node type** | `RigExecPickerButton` |
| **Example** | [picker.usda](../examples/picker.usda) |

On this page:

- [Overview](#overview)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Parameters](#parameters)
- [Example](#example)
- [Tips](#tips)
- [See also](#see-also)

## Overview

A button is one shape in a panel and one of four things, decided by
what it names rather than by a flag: a SELECT button lists prims in
`rigExec:picker:controls`, a SWITCH edits the one attribute in
`rigExec:picker:attribute`, a COMMAND runs the named action in
`rigExec:picker:command`, and a button naming none of them is DECORATION —
a backdrop, a silhouette, a label. Everything else on the class is
appearance and visibility: outline, colours, text, draw order, and
which half of an IK/FK pair a button belongs to.

One button. Selects the prims in rigExec:picker:controls,
edits the attribute in rigExec:picker:attribute, runs a
rigExec:picker:command, or none of these -- such a button is a
decoration: the body silhouette, a backdrop, a label.

LIVENESS is the resolve result, not a flag. A button is live when
every prim it targets is a RigExecControl or RigExecJoint of the
picker's rig, and the panel does not draw a button that is not. USD
remaps the targets through renames
and reparents on its own, which a path string in a sidecar file
cannot do, so a control that MOVES keeps its button and only a
control that is DELETED loses one.

## How it works

usdview reads buttons when the panel opens and after any change to
the prims; no rig phase touches them. Liveness is the resolve result, not
a flag: `pickerScene` turns the relationships into target paths and the
model marks a selecting or switching button live only when every target —
or, for a switch, the prim owning the attribute — is a `RigExecControl` or
`RigExecJoint` inside the rig named by `rigExec:picker:rig`, or anywhere on
the stage when the picker names none (`pickerModel.py:107-116`,
`pickerScene.py:345-353`, `:356-378`); a command button the panel
implements is live on its own (`pickerModel.py:107-108`). A button that
resolves to nothing is not drawn at all, though `coverage()` still counts
it for the panel's status line (`pickerModel.py:241-242`, `:344-354`,
`pickerUI.py:1087-1100`). A click selects the targets in order, or steps
the switch's attribute to its next label and writes it back as a float,
matching the pose first when the switch drives an IK/FK limb or a space
switch; a switch with more than two labels opens a menu of them instead
(`pickerUI.py:1102-1160`, `:1302-1338`, `:1439-1464`); the hit test is
the button's bounding box rather than its drawn outline, and `ui:size` is
clamped up to 4 units so a fingertip button is still catchable
(`pickerModel.py:23`, `:57-58`, `:288-310`).

## Wiring

| Relationship | Points to | Required |
|---|---|---|
| `rigExec:picker:controls` | The `RigExecControl` or `RigExecJoint` prims this button selects, in order. | no |
| `rigExec:picker:attribute` | The single attribute a switch edits, as a property path on a control or joint. Mutually exclusive with `rigExec:picker:controls`; on a plain click the attribute wins. | no |
| `rigExec:picker:modeDial` | The attribute `rigExec:picker:mode` is tested against — the limb's IK/FK dial. | only with `rigExec:picker:mode` |

## Parameters

### Node parameters

#### `ui:position`

*Type:* `uniform float2`. *Default:* `(0, 0)`.

Top-left corner in panel units.

#### `ui:size`

*Type:* `uniform float2`. *Default:* `(10, 10)`.

Width and height in panel units, at least 4. Clicks and
marquees test this box, not the drawn outline or ui:rotation.

#### `ui:depth`

*Type:* `uniform float`. *Default:* `0`.

Painter's-algorithm order; higher draws later, on top, and
a click finds the topmost. Equal depths keep prim order, and
widgetControl buttons draw above every other shape whatever their
depth.

#### `ui:rotation`

*Type:* `uniform float`. *Default:* `0`.

Degrees clockwise about the button's own centre.

#### `ui:shape`

*Type:* `uniform token`. *Default:* `"rectangle"`.

Valid values: `rectangle`, `roundedRectangle`, `circle`, `ellipse`, `hexagon`, `triangle`, `trapezoid`, `polygon`, `bezier`, `widgetControl`, `slider`.

Outline. `polygon` reads ui:polygon, `bezier` reads
ui:bezier, and every other token is generated from ui:size;
`circle` fills that box like `ellipse`. `widgetControl` draws as a
rectangle, and `slider` is not drawn.

#### `ui:roundness`

*Type:* `uniform float`. *Default:* `0`.

Corner radius in panel units for rectangle, widgetControl,
polygon and trapezoid outlines. roundedRectangle uses at least 35%
of the shorter side.

#### `ui:polygon`

*Type:* `uniform float2[]`. *Default:* `[]`.

Outline vertices for ui:shape = polygon, in panel units
relative to ui:position.

#### `ui:bezier`

*Type:* `uniform float2[]`. *Default:* `[]`.

Cubic control points for ui:shape = bezier, in panel
units relative to ui:position, THREE per knot -- on-curve point,
out-tangent, in-tangent -- flattened, with each span completed by
the following knot's in-tangent. The last knot closes back to the
first.

#### `ui:fill`

*Type:* `uniform color4f`. *Default:* `(0.5, 0.5, 0.5, 1)`.

Interior colour, drawn as authored with no colour-space
conversion and with straight (not premultiplied) alpha.

#### `ui:stroke`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 0.8)`.

Outline colour.

#### `ui:strokeWidth`

*Type:* `uniform float`. *Default:* `1`.

Outline width in panel units.

#### `ui:text`

*Type:* `uniform string`. *Default:* `""`.

Label drawn inside the button. A command button with no
text is labelled with its command.

#### `ui:fontSize`

*Type:* `uniform float`. *Default:* `8`.

Label size in panel units.

#### `ui:bold`

*Type:* `uniform bool`. *Default:* `false`.

Label weight.

#### `ui:textAlign`

*Type:* `uniform token`. *Default:* `"center"`.

Valid values: `left`, `center`, `right`.

Horizontal label alignment inside the button.

#### `ui:textColor`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 1)`.

Label colour.

#### `ui:valueColor`

*Type:* `uniform color4f`. *Default:* `(0, 0, 0, 1)`.

Colour of the VALUE a switch or dial displays, as
distinct from its label: `Foot` is drawn in ui:textColor and the
`IK` beside it in this.

#### `ui:direction`

*Type:* `uniform token`. *Default:* `"top"`.

Valid values: `top`, `bottom`, `left`, `right`.

Which edge a triangle points at. On a trapezoid, `top` and
`bottom` let the slopes inset the horizontal edges and `left` and
`right` the vertical ones; the slope signs choose which edge.

#### `ui:slopeLeft`

*Type:* `uniform float`. *Default:* `0`.

Trapezoid inset at the left end, in panel units. With
ui:direction `top` or `bottom`, a positive value insets the top
edge and a negative one the bottom edge; with `left` or `right` it
shortens the left edge from the top (positive) or the bottom
(negative).

#### `ui:slopeRight`

*Type:* `uniform float`. *Default:* `0`.

Trapezoid inset at the right end, in panel units. With
ui:direction `top` or `bottom`, a positive value insets the top
edge and a negative one the bottom edge; with `left` or `right` it
shortens the right edge from the top (positive) or the bottom
(negative).

#### `ui:alternate`

*Type:* `uniform bool`. *Default:* `false`.

This button shares a slot with another and is the one
the panel hides. Where rigExec:picker:mode and
rigExec:picker:modeDial are authored they are the authority and
this is ignored: a stacked IK/FK pair is a mode pair, not an
alternate.

#### `rigExec:picker:command`

*Type:* `uniform token`. *Default:* `""`.

A named action this button runs, instead of selecting or
editing. The panel implements a fixed set and ignores the rest, so
a picker may carry actions a given host cannot perform without
those buttons pretending to work. The usdview panel implements
`zero_ctrls`, which clears the authored pose channels of the
selected controls and joints (every control when none is
selected), and `ctrl_vis`, a checkbox that hides or shows the
rig's controls with session-layer visibility opinions.

#### `rigExec:picker:controls`

*Relationship.*

The prims this button selects, in order. Targets may be
RigExecControls or directly-posed RigExecJoints; the picker treats
both as selectable because the avar editor edits both.

#### `rigExec:picker:attribute`

*Relationship.*

The single attribute this button edits, for a switch or a
dial. Mutually exclusive with rigExec:picker:controls.

#### `rigExec:picker:attributeLabels`

*Type:* `uniform token[]`. *Default:* `[]`.

Labels for the values rigExec:picker:attribute takes:
label i names value i, e.g. ["FK", "IK"] on an IK/FK blend. The
button shows the label matching the attribute's CURRENT value,
rounded; it never caches the value it was authored with. A click
writes the next label's value, or opens a menu when there are more
than two.

#### `rigExec:picker:mode`

*Type:* `uniform token`. *Default:* `""`.

Valid values: `, `, `, `.

Show this button only while its limb is in this mode, as
read from rigExec:picker:modeDial: `ik` at 1, `fk` at 0, and both
halves show in between. Empty, or no modeDial target, means always
shown, which is right for anything that is not half of an IK/FK
pair.

#### `rigExec:picker:modeDial`

*Relationship.*

The attribute rigExec:picker:mode is tested against. Only
meaningful when that token is authored.

## Example

`picker.usda` has one of each kind in a single panel: `b_Shoulder`
selects one control, `b_Arm` selects three at once, `b_ArmIkFk` switches
`ArmParams.avars:ikfk` between the labels `FK` and `IK`, `b_Zero` runs the
`zero_ctrls` command, and `backdrop` names nothing and is decoration. The
elbow/wrist pair carries `rigExec:picker:mode = "fk"` and the pole/hand
pair `"ik"`, both dialled off `ArmParams.avars:ikfk`, so half the limb's
buttons swap out when the switch is clicked.

Open it live with:

```bat
bin\launch_usdview.bat docs\examples\picker.usda
```

## Tips

- Leave `ui:text` unauthored on a command button. The panel falls back to the command's own title (`Zero Ctrls`) and then dispatches on that LABEL, so a friendlier caption makes the button inert (`pickerScene.py:154-155`, `pickerModel.py:28`, `:94-97`).
- Author a switch's attribute on a control or a joint — a custom `avars:ikfk` on a params control, connected onward to the blend — because an attribute on a solver prim fails the liveness test and the button is never drawn (`pickerModel.py:109-111`, `:241-242`).
- `rigExec:picker:mode` is ignored without `rigExec:picker:modeDial`, and a dial resting between 0.001 and 0.999 counts as neither mode, so both halves stay reachable mid-handover (`pickerScene.py:164-166`, `pickerUI.py:1239-1245`).

## See also

- [Picker](picker.md)
- [Picker Panel](picker_panel.md)
- [Blend Point Frames](blend_point_frames.md)

---

[UsdRig](../index.md)
