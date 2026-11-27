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
namespace remapping. Review representative viewport poses when available, and
report separately if only numerical/composition checks were performed.

### Package the delivery as one archive

Build one archive for the delivery, and record the authoring head (commit id)
it was built from and the archive's SHA-256 in the delivery notes. Prove the
member list from the built archive's member listing; a reported file
inventory, such as a `delivery-files.json` list, is not evidence of archive
contents. Derive the members from resolution rather than memory. For each
stage entry that means the layers it actually uses, from
`Stage.GetUsedLayers()`, carrying each referenced
`asset_lib/components/**` layer at the relative position the reference
expects, without flattening or copying component definitions into the
character, so the fit/composition separation and the component ports stay
unchanged. For each packaged entry point it means the import closure of its
package-local modules. Late-added or untracked authoring helpers are what
fall out of the member list.

Every packaged entry point resolves its helpers from its own file, such as
`Path(__file__).resolve().parent / "tools"`, never the authoring directory,
the repository root, the current working directory, or an absolute path.

### Run the headless package check

Run it on the built archive in a fresh directory outside the
authoring checkout, after the in-checkout validation above, as its own record.
Extract that exact archive into an empty directory made by `mktemp -d` and run
with that directory as the working directory. Keep the supported USD/RigExec
environment: the interpreter, `pxr`, and the `rigexec` Python module from
`bin/_env.sh` or `bin\_env.bat` (or an installed usdRig), with
`PXR_PLUGINPATH_NAME` at the generated schema and imaging resources. The
engine and its plugin resources are supported dependencies and may live in the
checkout's build tree or an install; what must be unreachable is the
delivery's own authoring layers and helper modules. Do not put the authoring
delivery directory on `PYTHONPATH` or `sys.path`, and prefer a real barrier for
the authoring sources when one exists. Have the gate assert that every
package-local imported module's `__file__` and every resolved layer identifier
lies under the extracted root, and record which barrier was used.

Then run both packaged entry points as processes from the extracted tree: the
packaged verifier against the character entry, and the packaged animation
checks against the animation entry when the delivery carries one. Capture each
gate's document and exit code separately. Compare the extracted results
bit-for-bit with the authoring-tree run at rest and at one posed or animated
frame. A packaging gate that only re-opens stages in-process does not prove
that the packaged scripts import; the gate itself must run the packaged entry
points from the extracted tree.

```bash
zip=delivery.zip; root=$(mktemp -d)   # "$PY" is the retained interpreter
unzip -q "$zip" -d "$root" && cd "$root"
"$PY" <entry>/verify.py <entry>/character.usda        # character entry
"$PY" <entry>/tools/check_animation.py <entry>/animation.usda  # animation entry
unzip -Z1 "$zip" | grep '\.py$'        # members the archive carries
grep -n 'sys\.path\|^from \|^import ' <entry>/verify.py  # where it looks
```

### Triage an import failure before recommending a fix

Name the failing class with command evidence before changing anything.

- Environment: `No module named 'pxr'`, `'numpy'`, or `'rigexec'`, or
  `Cannot find a valid schema`. The interpreter or plugin path was not
  retained; the package is not at fault.
- Omitted member: the archive listing has no such file.
- Mislocated member: the file is in the archive, but outside the packaged
  entry point's search path.

List the archive's `.py` members, read the packaged entry's `sys.path`
insertions and import lines, and compare both against the pinned authoring
tree's file list, including tracked versus untracked files
(`git status --porcelain`). Then rebuild the archive and re-run the whole
headless package check from scratch; never patch only the extracted copy.

### Report the package gate

Record each gate as pass, fail, or `not run`, naming the blocking module or
missing member for a `not run`. A run that stops at import measured nothing,
so it is never evidence about weights, gait, deformation, or media. Report the
head id, archive digest, environment, and the barrier used, and keep the
package-gate record separate from the independent quality verdict of the rig
(for example an advisory model's medium verdict). An authoring-tree run is not
package evidence.

Deliver links to the final entry point, fit, bindings, and composition, plus the
rebuild command and checks actually run. Keep the component library reusable;
an existing character's legacy monolithic generator is not the new assembly's
rebuild path.
