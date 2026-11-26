# Worked handoff: controls, poses, and a rebuild

This walkthrough replays, command by command, what a request such as
“controls, walk, run” must deliver: a control table derived from the composed
rig, one session-layer pose measured on the evaluated mesh, and a rebuild
sequence that works from wherever the character sits. Every command and
diagnostic here is quoted from
[example_control_handoff.py](../../asset_lib/components/example_control_handoff.py);
the prose never states a number or a message the script does not produce.
The skill rules are in [SKILL.md](SKILL.md) under *Hand off controls, poses,
and a rebuild*; the glossary terms (usdRig, RigExec, Control, Mover, Rig
component) are in [CONTEXT.md](../../CONTEXT.md).

The worked character is a small stand-in composed from the `torso`, `spine`
and one `leg` Rig component in namespace `/Demo`, rig root `/Demo/Rig`, and
mesh `/Demo/Geo/body.points`. It uses a `UsdGeom.Points` stand-in so no
character geometry enters the checkout. The same steps apply to a delivered
character; substitute its paths.

## Replay setup

All commands run from the repository root with the environment initialized
once per shell:

```sh
. ./bin/_env.sh
export CHAR=${CHAR:-scratch/handoff/character}
```

`$CHAR` is the character directory relative to the repository root. It may sit
anywhere, and it may be copied or relocated; the rebuild sequence re-derives
the component library path for wherever it ends up. The commands use only
`pxr`, `rigexec`, the standard library, and the composer — no third-party
packages, and no interactive or windowed run is required anywhere.

To materialize the worked character in `$CHAR`, run:

```sh
"$PY" - <<'PY'
import os, sys
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
folder = Path(os.environ.get('CHAR', 'scratch/handoff/character'))
folder.mkdir(parents=True)
nodes, ports, attachments = ex.plan()
ex.write_character(folder, nodes, ports, attachments)
PY
```

The directory then holds the delivery: `demo_model.usda` (entry through the
model), `demo_rig.usda`, `demo_rig.fit.usda`, the generated
`demo_rig.components.usda`, `demo_rig.bindings.json`, and
`demo_animation.usda`.

## 1. Inputs the request declares

A handoff request declares its inputs; everything else is derived, never
asked for. Declared inputs are:

- the character directory and the entry layer to open (`demo_animation.usda`
  for the keyed timeline, `demo_model.usda` for rest);
- the rig root (`/Demo/Rig`) and the output property to measure
  (`/Demo/Geo/body.points`);
- the segment names and their frame ranges — here `walk` frames 1, 5, 9, 13,
  16 and `run` frames 17, 21, 25, 29, 32, keyed as two segments of one
  timeline in `demo_animation.usda`;
- where to write the delivered table (`demo_rig.controls.json`).

What the controls are, which channels each exposes, and the stage units are
never declared by the requester and must not be guessed from a previous
character: read them from the composed stage, as the next section does. A
reply that paraphrases the scope and evaluates one hardcoded frame is what
made the #96 handoff incomplete.

## 2. Derive the table

Open the composition once — one `Usd.Stage.Open` of the model, one
`rigexec.Rig` compiled with `cpu_reference` enabled — and derive every row
from that composed stage.

- **Controls** come from `rig.evaluate(-1).control_paths()` cross-checked
  against every `RigExecControl` prim under the rig root. The two sets must
  agree; a control the stage defines but the evaluated pose does not publish
  is itself a finding, and the example raises rather than shipping a table
  over mismatched sets.
- **Channels** are the `avars:` attributes authored on the composed control
  prim (`prim.GetAuthoredAttributes()`) minus the four configuration channels
  `avars:defaultSpace`, `avars:rotationOrder`, `avars:rotationSign`, and
  `avars:unitScaleFactor`. Cross-check with
  `avarEditorModel.PoseChannels(prim, stage)` so custom channels (a rig's own
  `avars:ikfk`, `foot:` or `face:` channels) are reported rather than missed.
  Never use the schema-declared attribute list: every `RigExecControl`
  exposes all ten schema avars as defaults, so that list would claim channels
  the component never wired.
- **A table that lists only the animated channels is incomplete.** The
  worked character keys three channels (`Control_COG` `avars:ty` and
  `avars:tz`, `Control_FootIK` `avars:tz`); the delivered table has 46 rows
  across 9 controls. An animator who can pose a channel not in the table got
  an incomplete handoff — the #96 finding 1 failure.

Spell the table columns exactly: **prim and property**, **space**, **units**,
**suggested range**, **expected mesh response**, **omitted capability**. The
JSON artifact stores them as `prim`, `property`, `space`, `units`,
`suggested_range`, `enforced_limit`, `expected_mesh_response`, and
`omitted_capability`, plus `version`, `stage`, `rig`, and a `units` object
with the `metersPerUnit` and `upAxis` read from the **composed** stage.
Rotations are degrees; translations use the stage length unit. The `space`
column names the resolved provider prim, or `rig` for a control directly
under the rig root. A control with no authored channels still gets a row with
an empty `property`: `/Demo/Rig/World/COG/Hips/HipRoot` is published but
authored nothing, so its row records that it re-roots the leg IK chain and
offers no channels.

Derive the table and run the coverage and response gates:

```sh
"$PY" - <<'PY'
import os, sys, json
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
folder = Path(os.environ.get('CHAR', 'scratch/handoff/character'))
stage, rig = ex.open_rig(folder, ex.MODEL)
Path(folder, ex.TABLE).write_text(json.dumps(ex.build_table(stage, rig), indent=2))
ex.gate_coverage(folder, stage, rig)
ex.gate_responses(folder, stage, rig)
PY
```

It prints:

```text
PASS coverage: 9 controls including /Demo/Rig/World/KneePole with avars:tx avars:ty avars:tz
PASS responses: 45 isolated property poses moved exactly the points their rows claim at the suggested range
```

The pole control is required, not optional: a leg Rig component publishes
`/Demo/Rig/World/KneePole` with authored `avars:tx`, `avars:ty` and
`avars:tz`, and the gate names it explicitly.

`gate_responses` then proves the rows. It poses each channel alone, on the
session layer, at the end of its suggested range, on a stage at rest, and
requires the points that actually move to be exactly the set the row names;
the points a row leaves at rest stay put to float noise. The claims hold for
the measured fit and value only: in this fit the `FootIK` rotations and the
pole's `avars:tx` and `avars:ty` move no stand-in point at all, and those
rows say so and name the reason in `omitted_capability` rather than leaving
it blank. A response claimed from a control's name instead of a measurement
is the #96 finding 4 failure.

## 3. Suggested ranges versus enforced limits

RigExec puts no numeric bound on a control avar. A value far outside any
suggested range evaluates valid and finite, and it can move geometry — but it
can also saturate: a downstream rig constraint caps the geometric response
while still accepting the value, as the leg IK's `clampWithSoftness` reach
limit does for a knee pole posed far past its range. The `suggested range`
column is authoring guidance; the `enforced_limit` column is null unless the
rig itself authors a limit. What the schema and kernels actually enforce:

- `rigExec:unreachablePolicy = "clampWithSoftness"` on the two-bone IK
  solver: the only legal token value, and a soft limit on the goal's
  reachable space, not on any channel.
- A mover envelope's `inputs:defaultWeight` outside finite [0, 1] does not
  clamp and does not fail: the mover passes the incoming value through and
  reports a diagnostic.
- Weight objects carry `rigExec:rangePolicy` with legal tokens `strict` and
  `clamp`: authored weights default to `strict`, so an out-of-range weight
  invalidates the packet and the mover passes through; generated weight
  fields default to `clamp`, saturating at [0, 1] mid-scrub.
- `avars:sx`, `avars:sy`, `avars:sz` have a signed 1e-4 floor: finite
  magnitudes below 1e-4 are raised to 1e-4 keeping their sign (negative
  scale is a supported reflection), and non-finite values resolve to 1.0.
- `allowedTokens` on token attributes, such as `rigExec:operation`,
  `rigExec:representation`, `rigExec:rangePolicy`, and
  `rigExec:unreachablePolicy`: only the listed spellings are legal.

A range becomes enforced only when the rig authors it — for example a
`RigExecFloatMathMover` with `rigExec:operation = "clamp"` whose
`inputs:min` and `inputs:max` bound a driver the Control feeds. Absent such
rigging, `enforced_limit` stays null.

No table row may imply that an out-of-range pose is prevented. The #96
`spine_twist_past_limit` check drove a spine twist to +90/-90 degrees, far
past a ±45 suggested range, saw it evaluate valid and move geometry, and
scored that as success — the rig never promised the twist would stop. Suggested
ranges in the worked character come from the fit (±0.45 stage lengths for
translations, ±0.6 for the knee pole, ±45 degrees for rotations); state them
as suggestions and measure responses, never as limits.

## 4. One session-layer pose

Pose a channel that no animation key touches — here `/Demo/Rig/World/KneePole`
`avars:tz`, while the keyed timeline touches only the COG and foot goal — so
the pose proves the animator can actually move a control the delivery cannot
currently animate. Author it through the session layer so no delivered layer
changes. The exact recipe:

```python
session = stage.GetSessionLayer()
anon = Sdf.Layer.CreateAnonymous('.usda')
session.subLayerPaths = list(session.subLayerPaths) + [anon.identifier]
stage.SetEditTarget(Usd.EditTarget(anon))
stage.GetPrimAtPath('/Demo/Rig/World/KneePole').GetAttribute('avars:tz').Set(0.45)
```

Then evaluate `rig.evaluate(-1)` and require `pose.valid` and
`pose.reference_mismatches == 0` with `cpu_reference` enabled. Measure on the
delivered output: take `pose.moved_property('/Demo/Geo/body.points')` and
compare against rest at the **documented weighted indices** named in the
table's expected mesh response. For the pole that is indices 3 and 4, which
must move measurably (over 1e-2), while the documented unweighted index 6
must stay bit-identical. Prove nothing was written: take the SHA-256 digest of
every delivered `.usda` before and after the pose step; all digests must be
identical.

```sh
"$PY" - <<'PY'
import os, sys
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
folder = Path(os.environ.get('CHAR', 'scratch/handoff/character'))
stage, rig = ex.open_rig(folder, ex.MODEL)
nodes, _, _ = ex.plan()
keyed = {(nodes[i][role], channel) for i, role, channel, _ in ex.ANIM_KEYS}
digests = ex.layer_digests(folder)
ex.gate_pose(stage, rig, keyed)
ex.gate_no_write(digests, folder)
PY
```

It prints:

```text
PASS pose: session-layer /Demo/Rig/World/KneePole avars:tz=0.45 moved indices 3-4 by 0.268/0.094 and left index 6 bit-identical
PASS no-write: 5 delivered .usda layers unchanged by the pose step
```

The pose gate raises if the posed channel is one the animation keys, if the
posed pose is invalid, if a documented weighted index does not move, or if
the unweighted index moves at all.

## 5. The checkout-relative rebuild sequence

The delivery records a rebuild that runs from wherever the character sits.
Five stages, every command relative to the repository root with
`bin/_env.sh` sourced. Run them from a relocated copy, not the original:

```sh
export MOVED=${MOVED:-scratch/handoff/moved/character}
mkdir -p "$(dirname "$MOVED")"
cp -r "$CHAR" "$MOVED"
rm "$MOVED/demo_rig.components.usda"
```

Export `$MOVED` so the shell and the Python blocks below resolve the same
destination override. The `rm` belongs to preparing a scratch copy: it is
what makes that copy a build from an absent start. The recipe itself never
deletes or overwrites a generated layer — stage 2 refuses one.

**Stage 1 — fit. Stage 2 — composition.** The fit layer and the bindings
JSON are the source of truth and carry the relocated path. Run both stages
through `rebuild`, the guarded sequence the example's gates exercise. It
first refuses while `demo_rig.components.usda` exists — before rewriting the
bindings or composing, so rerunning the recipe over an already-built copy
leaves the bindings and the generated layer byte-identical. It then
re-derives the bindings' `library` field for wherever the copy sits — a path
relative to the bindings JSON, falling back to an absolute POSIX path on
Windows when the library is on another drive — and regenerates the generated
layer through the same `compose_assembly` the `compose.py` CLI runs. The
generated layer is derived data, and the recipe is only proven when it
creates that layer from an absent start:

```sh
"$PY" - <<'PY'
import os, sys
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
ex.rebuild(Path(os.environ['MOVED']))
PY
```

**Stage 3 — Animation.** `demo_animation.usda` sublayers the model and keys
the timeline: `walk` (frames 1, 5, 9, 13, 16) and `run` (frames 17, 21, 25,
29, 32) as two named segments of one timeline, keyed on the COG
`avars:ty`/`avars:tz` and the foot goal `avars:tz`. No rebuild step touches
this layer; reopening it after the composition is regenerated is what proves
the rig underneath survived.

**Stage 4 — Rendering.** `bin/usdview.sh "$MOVED/demo_animation.usda"` is
the checkout-relative host-side viewer. No automated step here opens a
window; the automated gate is numerical, and a viewport pass is a host-side
extra to report honestly.

**Stage 5 — Verification.** Reopen the relocated copy through
`demo_animation.usda` and compare against the delivered build: rest (frame
-1) and the keyed frame 25 must agree bit for bit on
`/Demo/Geo/body.points`. Then run the example's rebuild gate, which also
checks that every keyed frame evaluates valid and finite, that every keyed
frame moves the documented indices, and that the run segment out-strides the
walk segment on the leg-weighted indices:

```sh
"$PY" - <<'PY'
import os, sys
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
src = Path(os.environ.get('CHAR', 'scratch/handoff/character'))
moved = Path(os.environ.get('MOVED', 'scratch/handoff/moved/character'))
_, delivered = ex.open_rig(src, ex.ANIMATION)
_, rebuilt = ex.open_rig(moved, ex.ANIMATION)
for frame in (-1, ex.RUN_FRAME):
    a = ex.bits(ex.points_of(rebuilt.evaluate(frame)))
    b = ex.bits(ex.points_of(delivered.evaluate(frame)))
    assert a == b, 'frame %d differs from the delivered build' % frame
ex.gate_rebuild(src.parent, src)
PY
```

It prints:

```text
PASS rebuild: relocated copy regenerated demo_rig.components.usda; rest and frame 25 are bit-identical, every keyed frame moves indices 0-4
```

Keep every path in the recorded recipe checkout-relative; a recipe with a
hardcoded interpreter or dependency prefix is a recipe for one workstation
only.

## 6. The two failure gates

A handoff is only tested if the checks can fail. Both gates below are
negative gates: they pass only when the expected failure happens with the
control or layer named.

**A missing pole control.** Two probes, both on `/Demo/Rig/World/KneePole`.
Dropping its rows from the table must make the coverage comparison fail and
name the control:

```text
coverage gate: demo_rig.controls.json omits /Demo/Rig/World/KneePole
```

Dropping the pole prim from the fit scaffold — the shape a missing control
takes before composition, since the composition references recreate node
prims — must fail the other direction, a table naming a control the stage
lacks:

```text
coverage gate: demo_rig.controls.json names a control the stage lacks: /Demo/Rig/World/KneePole
```

Deleting the control runs the other way. Pruning the pole prim from the
delivered fit layer is not the removal path: the composition's references
recreate node prims from the bindings, so a rebuild brings the pole back and
coverage still passes. Removing the control's entry from
`demo_rig.bindings.json` is the removal path, and the composer then refuses
the partial node set instead of building a rig without it:

```text
ValueError: Node roles must exactly match the leg manifest
```

**A recipe that only opens the prebuilt stage.** The rebuild gate requires
the generated composition layer to be absent before the recipe and created
by it. Running the recorded sequence over a copy that still has
`demo_rig.components.usda` must refuse before recomputing anything:

```text
rebuild gate: demo_rig.components.usda must be absent before the recipe runs
```

The guard is the recipe's own: the recorded recipe calls
`example_control_handoff.rebuild`, which checks for the generated layer
before it rewrites the bindings or composes, so the refusal leaves the
bindings and the generated layer byte-identical. Opening a prebuilt stage is
not a rebuild; it proves nothing about the fit, bindings, and composer being
able to reproduce the rig. Run both gates:

```sh
"$PY" - <<'PY'
import os, sys
sys.path.insert(0, 'asset_lib/components')
import example_control_handoff as ex
from pathlib import Path
folder = Path(os.environ.get('CHAR', 'scratch/handoff/character'))
stage, rig = ex.open_rig(folder, ex.MODEL)
nodes, ports, attachments = ex.plan()
ex.gate_missing_pole(folder.parent, folder, stage, rig, nodes, attachments)
ex.gate_prebuilt(folder.parent, folder)
PY
```

They print:

```text
PASS missing-pole-control: dropped /Demo/Rig/World/KneePole rows or scaffold prims fail with the control named; pruning only the fit prim returns it after a rebuild, and deleting its bindings entry fails with Node roles must exactly match the leg manifest
PASS prebuilt-only-recipe: the recipe refuses a present demo_rig.components.usda and leaves the bindings and composition byte-identical
```

## 7. Omitted capabilities and checks that actually ran

The table carries an `omitted_capabilities` list, and the delivery repeats
it in prose. For the worked character it reads:

```json
[
 "no arm, hand or finger instances",
 "no eye, gaze or blink instance",
 "the foot goal's rotations reach no stand-in weight: no foot or toe weights are authored"
]
```

For a delivered character, disclose the same class of gap: an `eye`, gaze or
blink instance the rig references but never binds (referencing an eye
component creates no eyeball, lid geometry, material, or shader); authored
face-varying normals that no Mover moves; fingers the component publishes
but the character never modeled. A reader who is not told will assume the
control they see does something, or that the thing they do not see exists.

Report which checks actually ran and which did not. The seven `PASS` lines
below are the automated, numerical record of this walkthrough; a viewport
look or a host render is a separate, host-side observation. If
`bin/usdview.sh` never ran in this environment, say so in the delivery —
never report an unrun render as a pass.

## One-shot proof

The whole sequence, in a throwaway directory, with one command:

```sh
"$PY" asset_lib/components/example_control_handoff.py
```

It prints, in gate order:

```text
PASS coverage: 9 controls including /Demo/Rig/World/KneePole with avars:tx avars:ty avars:tz
PASS responses: 45 isolated property poses moved exactly the points their rows claim at the suggested range
PASS pose: session-layer /Demo/Rig/World/KneePole avars:tz=0.45 moved indices 3-4 by 0.268/0.094 and left index 6 bit-identical
PASS no-write: 5 delivered .usda layers unchanged by the pose step
PASS rebuild: relocated copy regenerated demo_rig.components.usda; rest and frame 25 are bit-identical, every keyed frame moves indices 0-4
PASS missing-pole-control: dropped /Demo/Rig/World/KneePole rows or scaffold prims fail with the control named; pruning only the fit prim returns it after a rebuild, and deleting its bindings entry fails with Node roles must exactly match the leg manifest
PASS prebuilt-only-recipe: the recipe refuses a present demo_rig.components.usda and leaves the bindings and composition byte-identical
```

Exit is non-zero on the first gate that fails, and each gate's diagnostic
names the control or layer at fault.
