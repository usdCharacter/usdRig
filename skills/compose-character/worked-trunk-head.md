# Worked example: bind a requested control chain

The walkthrough for the [compose-character skill](SKILL.md). One illustrative
intake: trunk and head controls, walk and run, weighted deformation with no
skincluster. It teaches how to resolve a request into instances, manifest
ports, node destinations, and provider parents. It is not a delivered
character and its numbers are not a template.

## Map the request to instances

| Control the animator names | Instance | Manifest node roles |
| --- | --- | --- |
| world, COG, hips | `torso` | `Control_World`, `Control_COG`, `Control_Hips` |
| spine, chest | `spine` | `Control_Spine1`, `Control_Spine2`, `Control_Chest` |
| neck, head, face | `head` | `Control_Neck`, `Control_Head`, plus cheek/ear/brow/nose/gaze roles |

That is one possible mapping for this request, not a rule. To resolve another,
walk the component folders in `asset_lib/components` and keep the components
whose `nodes` roles cover the requested controls: a jaw or lip request lands on
`mouth`, a gaze or lid request on `eye`, an IK chain on `arm` or `leg`. Do not
carry this instance set, these role names, or a "no skincluster" choice over to
anatomy the request did not mention. Side labels still do not mirror
transforms or weights, and the head and mouth already contain bilateral facial
controls.

## Read ports from the manifests

The request names what the animator sees, the instance set comes from the
anatomy, and the node role and port names come from the `component.json` of the
component being bound — never from a neighbouring instance or another
character. Each component declares the external prims it consumes; print them
instead of copying a table that goes stale when a manifest changes:

```text
python3 - <<'PY'
import json, pathlib
library = pathlib.Path('asset_lib/components')
for folder in sorted(p for p in library.iterdir() if (p / 'component.json').is_file()):
    ports = json.loads((folder / 'component.json').read_text())['ports']
    print(folder.name, sorted(ports))
PY
```

`Hips` is `leg`'s port, so a `spine` or `head` instance bound through `Hips` is
refused by `bind_component` with `Ports must exactly match the spine manifest`
and `Ports must exactly match the head manifest`.

## Trace one binding end to end

`head`'s `Control_Neck` declares manifest parent `port:Chest`, so the `head`
instance maps port `Chest` to a prim path — usually another instance's node
destination in the same bindings document: the `spine` instance's
`Control_Chest` at `/Demo/Rig/World/COG/Spine1/Spine2/Chest`. That node's own
parent is `node:Control_Spine2`, and the spine chain ends at port
`COG -> /Demo/Rig/World/COG`, the `torso` instance's `Control_COG`, whose root
port is `Rig -> /Demo/Rig`. The assembled chain is
`/Demo/Rig/World/COG/Spine1/Spine2/Chest/Neck/Head`. Grouping scopes may
intervene, but the nearest control, joint, or root ancestor must be the
declared provider. `validate_component` names the offending node and the
provider it wanted: hang `Spine1` under the `torso` instance's `Hips` control
while the `spine` `COG` port still points at `/Demo/Rig/World/COG` and it
rejects the instance with `Provider parent differs for
/Demo/Rig/World/COG/Hips/Spine1; expected /Demo/Rig/World/COG`, because the
port contract, not the incidental ancestor, is the provider. Define every
enclosing scope; an undefined ancestor hides the referenced nodes and fails
with `Missing or incompatible node: …`.

## Where the trunk and head data goes

The fit layer carries the character-specific data: sparse `rigExec:indices` and
`rigExec:values` per `Weight` prim, written for the actual bound mesh topology
as a per-mover influence mask rather than one flat skin-weight set summing to
one; `rest:space` per control in the USD row-vector convention
`childWorld * inverse(parentWorld)`, rig space for a top-level joint; and the
guide fields. Keep reusable component defaults, meshes, texture paths, and
another character's weights out of the fit layer. Follow the layer arrangement
in [*Author the character layers*](SKILL.md#author-the-character-layers) and
the attachment shape in [*Bind and
compose*](SKILL.md#bind-and-compose); this walkthrough only fixes where trunk
and head data goes.

## Poses to check before keying a gait

The animator's interface is control `avars:` channels plus IK goals and poles;
there is no skincluster to key. The minimum set for a delivered character is:
rest; a symmetric lift/check pose; the gait extremes contact, down, passing and
up at walk amplitude and again at run amplitude with trunk flexion/extension,
lateral bend and COG bob; and a trunk twist with neck and head
counter-rotation. For each pose require valid, finite results, motion confined
to the weighted index set, points outside that set unchanged, and
`reference_mismatches == 0` with `cpu_reference` enabled. Author the poses
through an animation or session layer so delivered rest values stay unchanged.

## Run the proof

From the repository root, in the environment initialized by `bin/_env.sh`
(`bin/_env.bat` on Windows):

```text
python3 asset_lib/components/example_trunk_head.py
```

Run it as a script, not with `python -m` and not as an import: its `compose`
import resolves from the script's own folder, which only the script form puts
on the search path. The script replays this walkthrough in a fresh `/Demo`
namespace and ends with a `PASS` line per check; the lines are a smoke signal,
not the contract — an `AssertionError` or traceback is the failure.

It checks the binding and validation rules above (both rejected bindings, a
compiling assembly, motion confined to the weighted indices with no `skel:`
binding on the mesh, and the entry asset reopening on its own), plus rest and
two amplitudes of one trunk, spine and neck pose. It does not key the lift/check
pose, the four gait extremes, walk and run amplitudes as separate poses, or
trunk twist with counter-rotation, and it does not exercise the `eye`, `arm`,
`leg` or `mouth` instances. Its body is a `UsdGeom.Points` stand-in and its fit
values are arbitrary test numbers, so it proves the wiring, not proportions or
weight quality; the pose list above still applies to a delivered character.
