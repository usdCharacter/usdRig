# Biped -- the ported character

Onboarding: [docs/guide/getting-started.md](../../docs/guide/getting-started.md). Spec:
[docs/specs/biped-rig.md](../../docs/specs/biped-rig.md).

**Open `Biped_stack.usda`.** That is the whole character: skeleton, rig,
face, skinned mesh, correctives, picker and touch regions, composed as a
tree of layers. It is self-contained -- clone the repo and open it, nothing
else to build or fetch.

**Nothing else is needed to open it.** The mesh points, the skin weights,
the materials and every rest transform are authored directly into these
files; the only asset paths in the whole stack are the relative sublayers
below, the right side's references onto the left, and the eye shader in
`shaders/`.

    bin\launch_usdview.bat examples\biped\Biped_stack.usda

The stack has two branches, the face over the body. Each branch carries
its own picker panel, correctives and shapes, so the face's live with the
face and the body's with the body. The touch regions of both sit in the
body's touch layer, and the eyes' in the face parts:

    Biped_stack.usda
        Biped_spaces.usda                   the space switches, which span both branches
        Biped_autoclav.usda                 the auto clavicles
        Biped_ikfk_match.usda               each limb's IK/FK match
        Biped_squetch.usda                  the head squetch and the neck slide
        Biped_face_rig.usda                 the face branch
            Biped_lashes.usda               the lash proxies' deformers
            Biped_face_parts.usda           the eye and face-part rig: hair, teeth, gums and tongue skins, mouth clusters, the eyes' touch regions
            Biped_face_picker.usda          the Face panel of the picker
            Biped_face_touch_regions.usda   the face's touch layer; it authors no regions
            Biped_face_psd.usda             face pose interpolators and their drivers
            Biped_face_shapes.usdc          the face's corrective blend shapes
            Biped_cheeks.usda               cheeks, nose and ear clusters
            Biped_brows.usda                the corrugators
            Biped_mouth.usda                the mouth, over the face and eyes
            Biped_eyes.usda                 the eyes, over the face
            Biped_jaw.usda                  the jaw's automatic compression
            Biped_face.usda                 the face's skeleton controls
        Biped_body.usda                     the body branch
            Biped_body_psd_fixes.usda       the right shoulder's pose rotations, rebased
            Biped_body_picker.usda          the Body panel of the picker
            Biped_body_touch_regions.usda   TouchPose regions on body and face controls
            Biped_body_breathing.usda       the chest, belly and shoulder breathing dials
            Biped_body_psd.usda             pose interpolators, one weight per corrective
            Biped_body_shapes.usdc          the blend shapes those weights drive
            Biped_body_bendy.usda           the bendy limbs: bend controls and their wires
            Biped_body_wires.usda           the spine and head wires and their bend controls
            Biped_body_center.usda          spine, neck, hips, chest, the rig root
            Biped_body_left.usda            everything on the left side
            Biped_body_right.usda           the right side, as references onto the left
            Biped_model_parts.usda          the hair, eye, lash, teeth, gum and tongue meshes
            Biped_body_model.usda           the body mesh and the materials

| file | what |
|---|---|
| `Biped_stack.usda` | **the one to open** -- the face and body branches |
| `Biped_body.usda` | the body on its own, with its picker, touch regions and correctives |
| `Biped_face_rig.usda` | the face branch; it needs the body under it |
| `Biped.usda` | the body rig flat, in one file, if you want to read it |
| `Biped_anim.usda` | a 200-frame animated overlay on `Biped.usda`, for timing and for testing the animated evaluate path |
| `Biped_stack_anim.usda` | the same keys over `Biped_stack.usda` |
| `Biped_anim_ikfk.usda` | `Biped_anim.usda` plus one keyed IK/FK blend on the left arm |

The face correctives fire from the face controls the way the source rig's
do: the mouth corners, jaw, inner and main brows, corrugators and lips each
drive a pose interpolator through a hidden driver that copies the control's
channels, measured as a TRANSLATION (or, for the lips, a twist). Moving
`L_Mouth` 3 cm out is exactly the authored wide pose.

Any layer can be muted for a lighter character. Each branch's psd and shapes
layers travel together: the interpolators publish weights nothing reads without
the shapes, and the shapes wait for weights nobody writes without the
interpolators. The face parts expect the ones below them, so drop those
from the top.

`Biped_anim.usda` sublayers `Biped.usda` and keys six controls over frames
1-200. A static stage never re-reads a time sample, so it cannot show what an
animated frame costs or whether the evaluator handles a time change
correctly; that is what this overlay is for:

    build\rigExecPose examples\biped\Biped_anim.usda --frames 1,50,100,150,200 --joints

The body branch's rig and the flat form evaluate identically -- all 259
joint frames of the flat form match to 0.000e+00 cm at rest and under a
test pose.

## What is in it

459 joints (262 in the flat body rig), a skinned 26,276-point body mesh
with 12 UsdPreviewSurface materials and the eye shader (`body_geo`
carries five of them as `GeomSubset`s, since one binding cannot express
five materials over 26,274 faces), and a rig: spline-IK spine and neck,
two-bone IK/FK limbs with a per-limb switch, reverse foot, FK finger
controls that follow the arm in IK as well as FK, twist helpers, and
`M_Body` as the master body control.

The face arrives as its own branch: the jaw group, the jaw compression
that couples the mid-face and the nose to it, the nose's blend between
upper and lower face, and the eye look-ats. The eyes arrive as one more
layer over it: the lids and the blink, the socket stretch and lift, the
lids following the eyes, and the lid and socket curves with a tweak control
on every point of each. The mouth is one more: mouthMain, the lip
clusters, the mouth corners driving lip main, and the lip curve with a
tweak control on every point. Then the brows' corrugators and the cheek,
nose and ear clusters. `Biped_body.usda` is the body alone.

## Driving it

Select a control and open **RigExec → General Editors → Avar Editor**.
`?` below is `L` or `R`. The controls worth starting with:

- `Main`, `Shot` and `Aux` -- the masters at the top; each moves the whole
  character. `Aux` is the "world" of the space switches
- `M_Body` -- the body; the IK hands and feet stay in their own space
- `M_Torso` -- chest, neck, head and arms, not the legs
- `M_HipSwivel` / `M_ChestTop` -- hip swivel and chest; both twist the
  spine, from their own end
- `?_ArmIK` / `?_LegIK` -- the IK hand and foot. Their axes are world
  axes, so the gizmo moves and turns them in world space; the wrist and
  ankle keep their own bone orientation underneath
- `?_Arm` / `?_Leg` -- the small yellow cubes near each wrist
  and ankle. Their `ikfk` channel is the IK/FK switch: **0 = FK, 1 = IK**,
  and the inactive control set fades as you cross. Arms ship in FK, legs in
  IK.
- `?_ArmBend0`..`4` / `?_LegBend0`..`4` -- the bendy limbs, five
  along each arm and leg from the upper joint to the end joint. Move one
  to bend the limb's skin; the two halfway controls ride between their
  neighbours. They bend the skin by their own motion only, so posing the
  limb is unchanged until you touch one
- `M_ChestTop`'s `breathing:chest` and `breathing:belly` dials breathe
  the chest and the belly (their shapes sit at +-10 and +-100), and
  `breathing:shoulders` slides the shoulders out and forward
- `?_Leg`'s `footRoll` and `footRock` dials are the foot roll and bank;
  `?_Heel`, `?_Toe`, `?_BallRoll` and `?_Bank` sit along the IK foot's
  pivots
- the small circles along each finger -- `L_IndexBase` and its 41
  siblings, nested one under the next. They drive their own joint and
  everything below it, and the whole hand rides the wrist in either IK or
  FK
- `M_Skull` on the head, carrying the face. `M_Jaw` at the jaw, with
  `M_UpFace` / `M_LoFace` above it and `M_NoseBridge` and
  `M_Nose` in between. The brows sit on `M_HeadTip`, and each
  `?_Brow` carries its inner brow and its peak (`?_InBrow`, `?_MidBrow`).
  Three dials carry the automatic parts: `face:jawCompression` on `M_Jaw`
  (0 to 1, the jaw pressing up into the middle of the face; closing the
  jaw past rest or pushing it up drives it by itself, and the dial adds on
  top),
  `face:noseFollow` on `M_LoFace` (how much of the lower face the
  nose carries, 1 by default) and `avars:space` on `M_Look`
- `M_Look`, the wide plate in front of the eyes. It is top-level,
  beside `M_Body`; its `avars:space` is `local` (riding the upper face) by
  default, and at `world` a gaze holds while the head turns. `M_LookRot`
  sits between the eyes and turns both together; `L_Eye` and
  `R_Eye` sit on the eyes and turn one each. `M_LookRot`'s `drag`
  (0 to 10, 10 by default) is how much the lids follow the eyes
- `?_UpLid` / `?_LoLid`, in front of each eye. Move them up
  and down to blink and sideways to shift the lid; the lower lid never
  closes past the upper. `sy` on the upper lid stretches the socket and
  `socketLift` lifts it
- the tweak controls along each lid (`?_UpLid1`..`5`, `?_LoLid1`..`5`,
  `?_InLid`, `?_OutLid`) and around each socket (`?_UpSocket1`..`5`,
  `?_LoSocket1`..`5`, `?_InSocket`, `?_OutSocket`, `?_UpSocketLine`,
  `?_LowSocketLine`). They ride the blink and the socket, and move the skin
  only by what you do to them
- `?_Mouth` at the corners of the mouth. Moving one drives the
  whole lip line on its side (lip main); `cornerPinch` presses the lips
  together at the corner
- `M_Mouth` moves the whole mouth; `M_UpLip` / `M_LoLip`
  move and turn each lip, with `M_UpLipCenter` /
  `M_LoLipCenter` riding them and `?_UpLip` /
  `?_LoLip` at the sides
- the lip tweak controls (`M_UpLipMid`, `?_UpLip1`..`3`, `?_OutLip`,
  `?_LoLip1`..`3`, `M_LoLipMid`) ride lip main and shape the lips locally

**RigExec → Viewport → Viewport Tools** gives move/rotate/scale gizmos that
write avars directly, if you would rather drag than type.

## Why it is fast

Rig compilation produces the shared operation graph, reused until a structural edit:
typed steps over dense slots, each bound to the exact producers of its
inputs. A frame reruns only the steps whose inputs changed. usdview,
`rigExecPose`, frame-cache workers and the `.rigexec` runtime all evaluate
that graph; there is no second evaluator to select and no fallback path.
`Biped.usda` and `Biped_body_center.usda` author
`uniform bool rigExec:baked = true` on their `RigExecRoot`; evaluation
ignores that attribute.

### Comparing deformer modes

The launchers take `--deform cpu|gpu|shader` after the stage, for any
asset and any layer:

```
bin\launch_usdview.bat examples\biped\Biped_stack.usda --deform shader
bin/usdview.sh examples/biped/Biped_stack.usda --deform gpu
```

- `cpu` (the default): every deformer runs on the CPU kernels.
- `gpu`: a mesh's whole deformer chain runs as one GPU compute program and
  the points are read back, so everything downstream sees them as before.
- `shader`: the same program runs inside Storm's own compute pass. Points
  never return to the CPU; Storm computes normals and velocities on the
  GPU, and the extent comes from a sampled estimate.

A chain falls back to the CPU kernels when it contains a deformer the
program does not cover, or when another step reads its points (in
`shader` mode, a surface projector or normals step reading the chain).
The mode is the `RIGEXEC_GPU_DEFORM` environment variable (`readback` is
`gpu`'s other name), and `RIGEXEC_GPU_DEFORM_STATS=1` prints per-frame
device timings.

## Touching it

**RigExec → Animation Editors → TouchPose** lets you pick controls by touching the character
instead of hunting for them. Tick the box, hover the body, and the painted
region under the cursor lights up; click it and the control that owns that
region becomes the selection -- so the Avar Editor and the gizmo follow
straight away.

    bin\run_usdview_touchpose.bat

Three colours, three states, all taken from the authored
`touch_sets.touch`: the region under the **cursor** in its own painted
colour, the region you clicked **last** in `leadColor` (a green), and
anything **else selected** in `selectedColor` (a
neutral). Selecting a control anywhere -- the Control Picker, the outliner -- lights its region here too.

### Selecting

The same rules as the Control Picker, so it does not matter which one you
reach for:

| gesture | what it does |
|---|---|
| click / drag | **replace** -- the selection becomes what you picked |
| Shift | **toggle** -- adds what is not selected, removes what is |
| Ctrl | **remove**, always. The one gesture that can never add. |

Dragging on the body marquees, and the same three rules apply to it. The
band catches any region it **touches**, not only ones it fully encloses -- otherwise a finger or a clavicle would be nearly uncatchable -- and only
regions facing you, so a band across the chest does not quietly select the
back as well.

With the panel focused: **T** toggles TouchPose, **P** toggles Paint,
**Ctrl+A** selects every region, **Ctrl+Shift+A** clears, **Ctrl+I**
inverts, **Ctrl+S** saves.

While you are dragging a gizmo handle, TouchPose **stands down** -- no
cast, no hover highlight -- and picks up again on the first mouse move after
you let go. The selection highlight stays lit throughout and follows the
pose being dragged, because it is drawn by the body's own shader.

The highlight **authors nothing**. It is a Storm shader tint: a Hydra scene
index in `rigExecImaging` gives the body a per-face region id and a small
per-region colour table, and wraps its materials so the lit colour is mixed
with `table[region]`. A hover changes one constant primvar; no prim is
created, shown or hidden, and the stage never sees an edit. Picking runs in
C++ against the posed mesh (a BVH refit per pose). Measured on this
character: a hover costs about **0.3 ms** where it cost 4-8 ms, a region
crossing reaches the screen in about **15 ms** where it took 60-90 ms, and
TouchPose adds almost nothing to a selection change where it added
~200 ms (`bin\run_testusdview_touchpose_bench.bat`).

While the box is ticked the **mesh is not selectable**: a click on the skin
belongs to TouchPose, and on unpainted skin it selects nothing rather than
picking `body_geo`. The **viewport gizmo wins** where the two overlap, so
dragging a manipulator handle over a limb never re-selects the limb. Untick
it and usdview's own picking is back exactly as it was. Alt still drives
the camera throughout.

### Painting regions

Tick **Paint**, pick a region in the list, and **dragging** on the body
gives the faces under the brush to that region; **Shift-drag** takes them
away. Paint is a mode and the brush owns the drag while it is on, so the
marquee stands down -- deliberately, because a modifier that painted in one
mode and subtracted in another is how an animator deletes half a selection
by reflex. The regions stay non-overlapping as you paint -- a face joining
one leaves the other in the same step -- and nothing is written until
**Save regions**, which rewrites each region into the touch layer it came
from (on this character, `Biped_body_touch_regions.usda`) and never the
rig.

That last part is not just tidiness. An authored edit inside the rig's
read roots is a stage edit the evaluator must classify and invalidate, and
a structural one recompiles the rig. The paint buffer lives outside the rig
for that reason (and the highlight is not on the stage at all), and the
headless test asserts the rig's generation counter does not move across a
stroke.

### The files

| file | what |
|---|---|
| `Biped_stack.usda` | **the one to open** -- both touch layers over the whole character, so TouchPose and the Control Picker both work on it |
| `Biped_body_touch_regions.usda` | the regions of the body and the face: limbs, fingers, spine, head and neck, jaw, brows, nose, lids, sockets, lips, cheeks and ears |
| `Biped_face_touch_regions.usda` | the face branch's touch layer; it authors no regions |
| `Biped_face_parts.usda` | one region on each eye, on its own scope (`TouchPoseEyeL`, `TouchPoseEyeR`) naming the eye mesh |

The body and face regions share the `/Biped/Rig/TouchPose` scope, so the
tool reads them as one set. Opening a touch layer on its own shows nothing,
because its regions sit under `over`s with no geometry.

The regions come from the studio's `touch_sets.touch`, which ships 247
painted sets. 219 are written here, 110 on the body and 109 on the face: a
set naming a control this port does not have yet is **skipped rather than
authored**, so nothing downstream pays to test, highlight and then refuse a
region it can never act on. The skipped count stays in the import report,
so the naming gap is still visible.

They are **not** `GeomSubset`s under the mesh, which is the obvious place
for them: hdSt collects every face subset whatever family it declares, so
the first 98 of them collided with `body_geo`'s five `materialBind` subsets and cost
16,739 warnings every time the stage opened. On their own scope, with the
faces in a plain `int[]`, the renderer never sees them.

## Everything the rig owns lives under the rig

The picker, the touch regions and the blend shapes are all specific to
one rig, so they sit inside its `RigExecRoot` rather than beside it:

    /Biped/Rig          RigExecRoot
        Main            Main, Shot and Aux, the masters
            Shot/Aux/Joints     the skeleton
            Shot/Aux/Controls   what an animator grabs
        Movers          constraints, deformers, the blend shape movers
        Solvers         FK chains, two-bone and spline IK
        IkFkMatch
        Spaces          the space switches
        AutoClavicles
        Weights
        Curves
        Shapes          UsdSkelBlendShape targets: 114 body, 47 face
        PoseInterpolators
        TouchPose       RigExecTouchRegions, body and face regions
        TouchPoseEyeL, TouchPoseEyeR   one region on each eye
        Uman            RigExecPicker, the control picker

    /Biped/Geom         the mesh
    /Biped/Materials

A rig published on its own now carries its picker, its touch regions and
its correctives with it, and none of them has to be found by guessing at
a path. The cost is that anything under the rig root is walked by the
evaluator's discovery passes: measured at about 14 microseconds per prim
across them.

## The picker is scene data

The picker is `RigExecPicker` prims in the layer stack, found by schema
type rather than by path or filename. A stage carrying two characters
gets two tabs with nothing configured anywhere; each picker names the rig
it drives through `rigExec:picker:rig`, and its buttons resolve inside
it. The panel nests them: one tab per character, its panels inside.

One prim per button is what makes it overridable. To move, recolour,
relabel, retarget or remove a button, write an opinion on that one prim
in a layer you own:

    over "b_L_Hand" {
        float2 ui:position = (338, 148)
        string ui:text = "L Hand"
    }
    over "b_R_Foot" (active = false) { }

Deactivating a panel drops its tab; deactivating the picker removes the
character. The panel follows all of it live, so an override appears as
you author it rather than on the next reopen.

Button targets are relationships, so USD remaps them through a rename or
a reparent: a button goes dead only when its control is really deleted,
and a button that resolves to nothing is not drawn at all.

## TouchPose regions are scene data too

Same rule as the picker, and for the same reason. The regions are
`RigExecTouchRegions` holding one `RigExecTouchRegion` per painted set,
found by schema type. Each scope names the mesh it annotates through
`rigExec:touch:mesh`, so a stage carrying three characters keeps three
sets of regions apart without any of them having to sit anywhere in
particular.

They are NOT `GeomSubset`s. hdSt collects every face GeomSubset under a
mesh whatever family it declares, so the touch sets collided with the
`materialBind` subsets that own the same faces for shading: 16,739
warnings on open, and the renderer doing work for data it never draws.

A save writes back to the scope it was read from rather than to a path
derived from the mesh, so moving the regions does not leave a stage with
two scopes and a reader picking whichever it met first.
