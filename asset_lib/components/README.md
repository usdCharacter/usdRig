# Reusable rig components

These seven character-independent components contain RigExec node definitions,
control channels, solver settings, and symbolic connection recipes. They have
no character geometry, proportions, skin weights, blink offsets, textures, or
absolute asset paths. Arm, leg, and eye each describe **one side**; instantiate
them twice with separate fits and destinations for a bilateral character.

| Component | Contents | Main external inputs |
| --- | --- | --- |
| [leg](leg/component.json) | Hip root, foot IK, knee pole, three joints, two-bone solver, deformation and weight templates | Hips, world, rig root |
| [spine](spine/component.json) | Two spine controls and chest; deformation and weight templates | COG |
| [torso](torso/component.json) | World, COG, hips; deformation and weight templates | Rig root |
| [arm](arm/component.json) | Clavicle, shoulder root, hand IK, elbow pole, three joints, wrist follow, five three-joint fingers; deformation and weight templates | Chest, world, rig root |
| [head](head/component.json) | Neck, head, nose, cheeks, ears, brows and their inner/outer controls; shared gaze/blink control | Chest |
| [eye](eye/component.json) | One eye aim, eye rotation and shader channels, projector, upper/lower lids, three samples per blink, summed/clamped blink channels | Head, shared gaze, eyeball and two lid meshes |
| [mouth](mouth/component.json) | Jaw, lower/upper lip, two corners; deformation and weight templates | Head |

Each folder has `component.usda` (node definitions under `/Component`) and
`component.json` (node roles, provider parents, ports, wiring, and required fit
fields). The USD is a definition library, not a fitted standalone character.
`nodes` are instantiated once per component. `templates` are repeatable
deformation/weight attachments, allowing any number of meshes to share an
influence. A mouth can bind lips, teeth, and an interior independently.

## Composition and fitting

Use [compose.py](compose.py) with the OpenUSD/RigExec environment configured by
the repository's `bin/_env.bat` or `bin/_env.sh`.

```python
from compose import bind_component, validate_component

# stage's edit target is the file-backed composition layer.
# nodes maps manifest roles to destination prim paths; ports maps external
# roles to existing prim paths. Fit values live in a stronger character layer.
bind_component(stage, "leg", nodes, ports, attachments)
# Validate after every component and the character geometry have been loaded.
validate_component(stage, "leg", nodes, ports, attachments)
```

`bind_component` authors USD references to the selected definitions and resolves
symbolic relationships/connections into the supplied namespace. References are
per node so control parenting and mover sibling order remain under the
assembly's control. All destinations must be unique across instances.
Define the enclosing scopes in the character scaffold before binding nodes;
an undefined ancestor hides its descendants from normal USD traversal.
Relationships retain explicit read phases, including the eye's final transform
read. Keep mover child ordering in the fit scaffold: it determines stack order.

Fit each component by authoring the fields listed under `fit` in its manifest:

- `rest:space` is a row-vector matrix relative to the nearest control/joint
  provider; top-level joints use rig space. Adjust both controls and IK joints.
- Guide dimensions, offsets, colors and rotation signs belong to each instance.
  Fit both sides explicitly; changing only a side label does not mirror a rig.
- Sparse weight indices/values belong to the bound mesh topology. Recompute
  them when topology or the intended influence region changes.
- Blink shape offsets and lid guide curves belong to the fitted lids. The eye
  projector's origin, direction and shader matrix belong to the fitted eyeball.

Fit layers should override these fields rather than copy fixed definition
defaults: changes to reusable control channels and solver settings then
propagate through the references. Supply matching guide fields as well as
deformation fields before calling `validate_component`.

## Rebuild an existing assembly

```text
python asset_lib/components/compose.py character.bindings.json character.components.usda
```

The version-1 bindings document has `library` (relative directory), `fit`
(relative layer filename), and `instances` (named objects). Each instance has
`component`, `nodes`, `ports`, and `attachments`. An attachment names a
`template`, destination `path`, and `relationships`; each relationship has
`targets` and optionally `readPhase`. Relative file paths resolve against the
bindings document. Prim paths are absolute within the character stage.

The character entry layer sublayers its fit first, then its composition. The
model provides mesh/material ports and includes that entry layer. Rebuilding
the composition keeps the fit scaffold and its authored child ordering, then
recreates references and wiring without reading a monolithic source rig.

## Verification

```text
python asset_lib/components/test_components.py
```

This builds independently fitted legs at two sizes in a fresh namespace,
evaluates their IK-driven geometry, and checks invalid fits are rejected.
