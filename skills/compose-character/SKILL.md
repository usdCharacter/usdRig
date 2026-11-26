---
name: compose-character
description: Compose, fit, or rebuild a USD character rig using usdRig's reusable leg, spine, torso, arm, head, eye, and mouth components. Use for assembling a new character or adapting an existing rig to asset_lib/components, including fit layers, component bindings, deformation attachments, and validation.
---

# Compose a character with usdRig

Build a character whose rig references reusable component definitions while
keeping its proportions, geometry bindings, and deformation data in character
layers. Use the existing composition API; do not duplicate the component
definitions into a new monolithic rig.

## Read the local contracts

Resolve the repository root as two directories above this skill folder.
Read its [AGENTS.md](../../AGENTS.md), the
[component guide](../../asset_lib/components/README.md), and
[compose.py](../../asset_lib/components/compose.py). Read the `component.json`
files for the components being used; these are the authority for node roles,
external ports, provider parents, required fit fields, and attachment templates.

For a small working authoring/evaluation example, inspect
[test_components.py](../../asset_lib/components/test_components.py). It fits a
leg in an independent namespace at two sizes. Its single-layer test arrangement
does not replace the fit/composition separation for a delivered character.

Run scripts importing `pxr` or `rigexec` in the environment initialized by
`bin/_env.bat` on Windows or `bin/_env.sh` on POSIX. Use that environment's
Python and schema plugins. Resolve paths from the checkout and character
location; do not bake workstation-specific paths into the skill or library.

## Choose instances and namespaces

Inspect the input stage before authoring: its default prim, units, up axis,
geometry paths, topology, materials, existing controls, and layer stack. Use
the supplied character and output location. When converting an existing rig,
retain a baseline for comparison and preserve its public control paths where
possible.

A typical bilateral character has ten instances of seven component types:

| Component | Instances | Responsibility |
| --- | --- | --- |
| `torso` | One | World, COG, hips |
| `spine` | One | Two spine controls and chest |
| `head` | One | Neck/head, cheeks, nose, ears, brows, shared gaze/blink |
| `mouth` | One | Jaw, lower/upper lip, both mouth corners |
| `arm` | One per side | Clavicle, IK chain, wrist follow, hand and fingers |
| `leg` | One per side | Hip root, IK chain, foot goal and knee pole |
| `eye` | One per side | Aim, eye rotation, projector, lids and blink graph |

Choose instances for the requested anatomy. An arm, leg, or eye definition is
one side; the head and mouth already contain bilateral facial controls. Side
labels do not mirror transforms or weights automatically.

Place one `RigExecRoot` around the assembled rig. Map each manifest node role
to a unique absolute prim path in that character's namespace. Respect each
node's `parent` contract; ordinary grouping scopes may intervene, but the
nearest provider ancestor must match. Define enclosing scopes explicitly:
an undefined ancestor can hide referenced nodes from normal USD traversal.
Scope names are conventions, not how RigExec discovers a node's role.

## Author the character layers

Use the character's existing naming conventions. The following names illustrate
the division of responsibility:

```text
character.usda                    Final entry point / optional scene and groom
  references character_model.usda
    references geometry and defines materials
    sublayers character_rig.usda
      sublayers character_rig.fit.usda          Stronger
      sublayers character_rig.components.usda   Weaker
        references asset_lib/components/*/component.usda
```

Keep the rig and geometry paths compatible inside the model's referenced root.
The rig entry layer lists the fit before the composition in `subLayerPaths`.
The model/entry asset should have an appropriate default prim and consistent
units/up-axis metadata.

The **fit layer** owns the rig hierarchy scaffold, its child ordering, and
character-specific values. Define the character/rig roots and grouping scopes;
component nodes can be `over` specs whose definitions come from references.
Author the manifest's required fit attributes with their exact USD types:

- Fit control and joint frames to character landmarks. With USD row-vector
  matrices, local rest is `childWorld * inverse(parentWorld)`; top-level joints
  use rig space. Fit IK root, joint lengths/orientations, goal, and pole
  consistently. Do not fit only the visible controls.
- Fit each side's handedness/rotation signs, guide dimensions, colors, offsets,
  and custom guide curves. All listed guide fit fields are validated too.
- Generate sparse weight indices and values for the actual target topology.
  Matrix-mover stacks use influence masks; do not assume these are a conventional
  flat set of skin weights that must sum to one across the entire stack.
- Generate blink offsets for each lid's vertex ordering and sample activation.
  Fit eye projector origin, direction, and shader offset to the eyeball and its
  compatible shader primvars. Referencing an eye component does not create
  eyeballs, lid geometry, materials, or the shader.

Keep meshes, clothing, teeth, texture paths, vertex arrays, and character-specific
landmarks out of shared definitions. Do not copy reusable default attributes
into the stronger fit layer unless a deliberate character override is needed.
For a new character, derive fit data from its geometry; reusing a template's
weights or blink arrays requires verified matching topology and intended shape.

## Bind and compose

Write `character_rig.bindings.json` with these version-1 fields:

- `version`: `1`.
- `library`: component library directory, preferably relative to this JSON.
- `fit`: path to the character fit layer, relative to this JSON.
- `instances`: a dictionary of named instances. Each value contains
  `component`, `nodes`, `ports`, and optionally `attachments`.

Within each instance, `nodes` must map **every** manifest node role to its
destination prim path, and `ports` must map **every** external port to an
absolute prim path. Follow the selected manifest instead of inferring names
from a different character. Property suffixes in manifest connections are
resolved by the composer; port values themselves are prim paths.

Each attachment contains `template`, `path`, and `relationships`. The template
must exist in that component's `templates` dictionary. Supply exactly its
declared relationship names; each relationship has a `targets` list of absolute
prim/property paths and optionally `readPhase`. Use repeatable deformation and
weight templates to bind influences to as many meshes as required. A points
mover targets the mesh's `.points` property; inspect the template and existing
examples for other endpoint types.

Preserve intended mover sibling ordering in the fit scaffold; it determines
stack application order. Preserve explicit dependency/read-phase metadata,
especially the eye transform's `final` read. Do not alphabetically reorder a
working stack or substitute list order for a required dependency connection.

From the repository root, rebuild the composition:

```text
python asset_lib/components/compose.py character_rig.bindings.json character_rig.components.usda
```

The fit file must already exist. `compose_assembly` copies its hierarchy and
child ordering, removes property opinions from the composition copy, and
authors references and wiring. It does not fit geometry, calculate weights,
create the model/entry layers, or fully validate a character. Keep the fit
scaffold free of unrelated model/groom content and unnecessary sublayers.

For programmatic assembly, load `compose.py` from the repository and use
`bind_component(stage, component, nodes, ports, attachments, library=library)`.
Set the stage edit target to the file-backed composition layer first; keep
fitted values in the stronger fit layer. The composer references individual
node definitions under `/Component`, preserving the chosen character hierarchy.
Do not reference the whole definition library and expect a fitted rig.

Rebuilding a composition replaces that generated layer. Put durable value edits
in the fit and durable wiring edits in the bindings. USD layers are sufficient
at runtime; the JSON and Python composer are authoring inputs. Retain referenced
library files when moving or packaging an unflattened asset.

## Validate and deliver

Open the complete character model, including geometry and material ports.
Check `stage.GetCompositionErrors()`, then call
`validate_component(stage, library=library, **instance)` for every instance.
This checks defined node types, required fit opinions/types, provider parents,
and target existence. It does not establish correct proportions, weight quality,
or deformation behavior.

Compile `rigexec.Rig(stage, rig_path)` and evaluate rest plus representative
poses for the components used: torso/spine/head, both IK limbs and fingers,
jaw/lips/corners, gaze, partial/full blink, and individual lids. Check valid
poses, finite results, intended motion, and unintended influence leakage. Use
session-layer pose edits so checks do not change delivered rest values. When
using `cpu_reference`, check for reference mismatches.

For a conversion, compare original and composed paths, types, values, targets,
connections, child order, and read phases. Compare evaluated control/joint
matrices, moved geometry/properties, and eye projector results in matching
poses. Preserve exact parity when the task is only a decomposition; intentional
refitting to another character needs different geometric expectations.

Open the final entry asset as well as the model to catch broken references or
namespace remapping.

When the request is driven by motion or anatomy references, or names a
deformation limit, run the reference-driven assessment below in addition to
the checks above; a routine decomposition with no reference keeps the checks
above alone. These records are what a later reviewer re-runs, so name the
file, frame, control, and measurement for every claim.

### Record the reference and the mesh axes

Before posing anything, record which reference was used for anatomy, for
walking, and for running separately, each with its citation, the view or time
range used, and how the specimen's pose and proportions differ from this
mesh. Declare the mesh's forward, up, and lateral directions, handedness,
units, and origin, read from stage metadata and geometry rather than
assumed. Record each reference landmark as one row:

`(side, landmark, source view/location, fitted mesh xyz, proportional or pose difference)`

Mark which entries are measured coordinates and which are estimates or
occluded. Never transfer a specimen's angles into fit values without that
difference recorded.

### Match clean silhouettes to labeled solved chains

For both sides, pair a clean full-body silhouette with a labeled solved
chain drawn over the identical camera, frame, and evaluated mesh,
at rest, stance, lift, and swing. Read the chain from the evaluated pose:
`rig.compile()`, `pose = rig.evaluate(frame)`, `pose.valid`,
`pose.joint_paths()` with `pose.joint_matrix(path)`, `pose.control_paths()`
with `pose.control_frame(path)`, and
`pose.moved_property("/<mesh>.points")` for the surface, all from the same
evaluated frame. Label the proximal
articulation, the intermediate bend or bends, the distal articulation, and
the paw, and keep shoulder, elbow, and carpus distinct from hip, stifle, and
hock. Authored control keys or a rest-stage render do not demonstrate
deformation. Discover controls, joints, and movers by schema and wiring,
never by scope or path naming, and read the ports each instance actually
binds from its `component.json`: `leg` binds Hips, World and Rig; `head`
binds Chest; `mouth` binds Head; `spine` binds COG; `torso` binds Rig; `arm`
binds Chest, World and Rig; `eye` binds LookAt, Head, Eye, LidUpper,
LidLower.

### Separate fit, parenting, pole, reach and timing

Compare in one order:
fit, parenting, pole direction, reach, and animation timing. Assess fit
against the recorded landmarks first. Then, with fit held fixed, compare
parenting — whether the shoulder is attached to the scapula and driven by
the thoracic limb while the pelvis is driven by the spine, and how much
inherited motion each limb carries — pole direction in the declared mesh
space, reach through the cycle, and finally animation timing. Report the
findings for each limb on its own line, together with the alternatives that
the evidence does not yet rule out. A solver reserve, a softness clamp, or
four disputed limbs is not evidence of a shared-chain defect; do not name
one cause until the earlier comparisons have been ruled out in that order.

Keep the fix in the right layer. A correction that changes values or weights
belongs in the fit layer; a correction that changes wiring belongs in the
bindings and is followed by the documented compose rebuild. Neither changes
a component port, adds a second evaluator, or reimplements IK outside the rig
in order to draw an overlay. Per-character values stay in fit, wiring stays
in bindings, and component definitions stay reusable.

Deliver links to the final entry point, fit, bindings, and composition, plus the
rebuild command and checks actually run. Keep the component library reusable;
an existing character's legacy monolithic generator is not the new assembly's
rebuild path.
