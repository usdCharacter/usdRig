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
it was built from and the archive's SHA-256 in the delivery notes, together
with the entry points the delivery promises: the character verifier, plus the
animation checks only when the delivery carries an animation entry. The
headless package check selects its gates from that promise, not from the
archive listing. Prove the member list from the built archive's member
listing; a reported file inventory, such as a `delivery-files.json` list, is
not evidence of archive contents. Derive the members from resolution rather
than memory. For each
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
delivery's own authoring layers and helper modules.

Isolate the authoring sources so that no reader can open them — Python or
USD's C++ layer — then prove it with USD itself. Name one authoring *stage* the
packaged entry needs, confirm it exists, and stop if it does not: a guard on a
path that was never there proves nothing, and the gate that should have failed
would pass. For the disposable scratch copy of a delivery this run built, move
it aside to a reserved unique sibling for the run and restore it when the run
exits: nothing can find a directory that is not there, and a fixed sibling name
could already hold another run's tree. Never move, delete, or change permissions on
a live checkout or another job's tree; cover the tree with an empty tmpfs in a
private user and mount namespace (`unshare --user --map-root-user --mount --`,
the helper first and `--` before it), one namespace per gate
process. Then ask USD, in that same gate environment, to open the named stage:
`Usd.Stage.Open` must return nothing and USD must report that it could not open
the layer. A probe that only asks Python to open the path says nothing about the
C++ reads USD performs, and a hook that only denies Python reads is not
isolation. Where neither barrier is available — unprivileged user namespaces are
off, or the mask still let USD open the stage — do not run the extracted gates:
record the package gate as `not run`, with the reason. Record which barrier
proved isolation.

Select the gates once, before anything is extracted, from the entry points
the delivery promises, and run that one selection as processes from the
extracted tree: the packaged verifier against the character entry, and the
packaged animation checks against the animation entry when the delivery
promises one. The same selection prints the statuses and drives the record
comparison: a character-only delivery passes on its character records alone,
and a promised entry whose script the archive lacks fails its gate in the
extracted run instead of vanishing. Run every gate
whatever the previous one exits: clear errexit for the gates and save
each one's stdout, stderr, and exit code to its own record before any triage
command runs, so no status is lost or overwritten. The authoring-tree
baseline is the same function called with the authoring tree, before anything
is isolated, so the two runs differ only in which tree they read.

Prove the extracted run read the delivered data. Every file-backed layer the
stage uses, and every module file this process imported, must resolve inside
the resolved extraction root. The exceptions are the interpreter's own library
directories and the runtime modules the run names: a named package exempts the
directories it is actually loaded from, wherever it is installed, and a named
plain module exempts only its own file, never the directory it sits in. A
directory on `PYTHONPATH` or `sys.path` — the retained `bin/_env.sh`
environment, an installed `asset_lib`, a sibling checkout, or the authoring tree
itself — grants no exception to a delivery layer or a package-local helper, and
a file found there is reported with its resolved path rather than accepted
because it was importable. The interpreter's site-packages grants an exception
to no module on its own: it is exempt only for the names the run wrote down, so
a delivery helper installed there is the same defect one directory deeper. The
subtraction covers the package directories of both the active scheme and the
base installation, since a virtualenv with system site-packages enabled can
import a helper from the base tree nested inside the interpreter library the
exemption covers. That is the class of defect this check exists to catch: the
run succeeds and the delivery is broken. Start the check with the list empty
and export it before any gate runs, so the authoring baseline and the extracted
gates share one list and their records differ only by the tree they read. Its
report names every outside
file this process read, and each entry is one of two things: a module the
delivery must carry, which is the defect, or a module of the retained runtime —
the engine's compiled extension, a virtualenv bootstrap, a `sitecustomize` the
environment injects — which the run then names and prints in its record. Naming
a module the delivery is supposed to carry is not a repair; it hides the gap the
same way `PYTHONPATH` does. A carried `asset_lib/components/**` layer sits
inside the extracted root at the
relative position the reference expects, so it passes this check; one resolved
from the authoring tree or from a search directory does not. Name the ported
prim's resolved path and the child it carries in the record, so the record
itself shows the component reference bound inside the extracted tree rather than
to a same-named layer elsewhere. An anonymous or session layer names no file:
report it separately as not file-backed, never as a violation and never as proof
of isolation. Its identifier embeds a process address, so keep the not-file-backed
count in the compared record and the identifiers in the log; otherwise the two
runs differ by which process ran. Then compare the extracted results
bit-for-bit with the authoring-tree run at rest and at one posed or animated
frame, masking only the not-file-backed identifiers in stderr. Write a package
member's path in a record relative to the delivery root:
an absolute path makes the two runs differ by where they ran, not by what they
read. A packaging gate that only re-opens stages in-process does not prove that
the packaged scripts import; the gate itself must run the packaged entry points
from the extracted tree.

The animation gate asserts the gait resolves on the entry, not in a file. A
default opinion in a layer stronger than the one carrying the time samples
replaces them -- `subLayers` order is strongest first -- and the stage keeps
zero composition errors, so a gate that counts errors alone passes on an entry
that no longer animates.

The packaged entry point calls this with `os`, `sys`, and `sysconfig` imported,
and with the runtime names it was given -- in this environment
`pxr rigexec _rigexec numpy _virtualenv sitecustomize rigexec_test_env`:

```python
def package_paths(stage, package_dir, runtime=()):
    """Report every file this process read that is not inside the delivery.

    package_dir is the resolved extraction root. Every file-backed layer the
    stage uses, and every module file this process imported, must resolve
    inside it. The exceptions are the interpreter's own library directories
    and the runtime modules named by the caller: a named package exempts the
    directories it is actually loaded from wherever it is installed, and a
    named plain module exempts only its own file, never the directory it sits
    in. No directory is exempt for being on the Python search path, and
    site-packages exempts nothing a caller did not name, so a delivery layer
    or package-local helper resolved from a search directory, an installed
    copy, or site-packages is reported with its resolved path. Returns
    (foreign, unfiled); each foreign entry names its kind and path, and
    unfiled lists the layers that have no file.
    """
    root = os.path.realpath(package_dir)

    def real(path):
        return os.path.realpath(path)

    def under(path, base):
        return path == base or path.startswith(base + os.sep)

    def under_any(path, bases):
        return any(under(path, base) for base in bases)

    cfg = sysconfig.get_paths()
    base_cfg = sysconfig.get_paths(vars={"base": sys.base_prefix,
                                          "platbase": sys.base_exec_prefix})
    stdlib = [real(cfg[k]) for k in ("stdlib", "platstdlib") if cfg.get(k)]
    sitepk = [real(d[k]) for d in (cfg, base_cfg)
              for k in ("purelib", "platlib") if d.get(k)]

    def interpreter(path):   # the interpreter's library, not the packages beside it
        return under_any(path, stdlib) and not under_any(path, sitepk)

    named_dirs = []          # the caller's exceptions: whole packages
    named_files = set()      # ... and single modules, never their directory
    for name in runtime:
        mod = sys.modules.get(name)
        dirs = [real(p) for p in getattr(mod, "__path__", [])]
        if dirs:
            named_dirs.extend(dirs)
        elif getattr(mod, "__file__", None):
            named_files.add(real(mod.__file__))

    def exempt(path):
        return (under(path, root) or path in named_files
                or under_any(path, named_dirs) or interpreter(path))

    foreign = []
    for name, mod in sorted(sys.modules.items()):
        path = getattr(mod, "__file__", None)
        if not path:
            continue                # built-in or frozen: no file was read
        path = real(path)
        if exempt(path):
            continue
        foreign.append("module %s: %s" % (name, path))

    unfiled = []
    for layer in stage.GetUsedLayers():
        path = layer.realPath          # absolute file; empty for a layer with no file
        if path:
            if not under(real(path), root):
                foreign.append("layer: " + real(path))
        else:
            unfiled.append(layer.identifier)   # the session layer carries the pose
    return sorted(set(foreign)), unfiled
```

Substitute before running, and only these: `<archive>` the built zip, which the
block names `delivery.zip`; `<entry>` the packaged entry directory, the one
`<entry>/verify.py` and `<entry>/tools/check_animation.py` live under;
`<gates>` the entry points the delivery promises — `verify` alone for a
character-only delivery, `verify animation` when it carries an animation
entry; `<frame>` one representative pose frame;
`$ENTRY_AUTHORING_DIR` the delivery's authoring root; and
`ENTRY_AUTHORING_SCRATCH=1` only when that root is a
disposable copy this run made, which selects the move-aside barrier instead of
the namespace mask. `$PY` and `PXR_PLUGINPATH_NAME` come from the retained
environment; `RIGEXEC_RUNTIME_MODULES` starts empty.

```bash
set -euo pipefail                              # stop before any packaged script
zip=$(realpath delivery.zip)                   # pinned before the cwd changes
authoring=$(realpath "$ENTRY_AUTHORING_DIR")  # the delivery's authoring sources
known=$authoring/<entry>/character.usda        # one authoring stage that entry needs
out=$(mktemp -d); root=$(mktemp -d)            # records dir; fresh extraction
: "${RIGEXEC_RUNTIME_MODULES:=}"      # empty on the first pass: the report names them
export RIGEXEC_RUNTIME_MODULES        # one list, exported before either run
sha256sum "$zip"                    # the digest this record is about

promised="<gates>"                  # verify, or verify animation: the promise
gate_names=()                       # one selection drives runs, status, cmp
for promised_gate in $promised; do  # fixed before anything is extracted
    case $promised_gate in
        verify)    gate_names+=(verify verify-pose) ;;
        animation) gate_names+=(animation animation-pose) ;;
        *) echo "unknown promised gate: $promised_gate" >&2; exit 1 ;;
    esac
done
if [ "${#gate_names[@]}" = 0 ]; then
    echo "the delivery promises no gates to run" >&2
    exit 1
fi

gate() { "$PY" "$@"; }              # the barrier wraps every gate process
gate_spec() {                       # gate name -> script, stage, optional frame
    case $1 in
        verify)         printf '%s\n' <entry>/verify.py \
                                   <entry>/character.usda ;;
        verify-pose)    printf '%s\n' <entry>/verify.py \
                                   <entry>/character.usda <frame> ;;
        animation)      printf '%s\n' <entry>/tools/check_animation.py \
                                   <entry>/animation.usda ;;
        animation-pose) printf '%s\n' <entry>/tools/check_animation.py \
                                   <entry>/animation.usda <frame> ;;
    esac
}
run_gates() {                     # one entry set, one argument set, both trees
    local tree=$1 rec=$2 spec g
    mkdir -p "$rec"
    set +e                            # every gate runs, whatever the last one did
    for g in "${gate_names[@]}"; do
        mapfile -t spec < <(gate_spec "$g")
        gate "$tree/${spec[0]}" "$tree/${spec[1]}" ${spec[2]:+"${spec[2]}"} \
             >"$rec/$g.out" 2>"$rec/$g.err"
        echo $? >"$rec/$g.rc"     # a promised script the archive lacks exits here
    done
    set -e
}
( cd "$authoring" && run_gates "$authoring" "$out/authoring" )   # the baseline

unzip -q "$zip" -d "$root"
cd "$root"                                     # a failed extraction or cd stops

[ -f "$known" ] || { echo "no authoring stage to isolate at $known" >&2; exit 1; }
echo "archive: $zip"                            # the paths a triage needs
echo "extraction: $root"
echo "records: $out"
if [ "${ENTRY_AUTHORING_SCRATCH:-0}" = 1 ]; then
    aside=$(mktemp -d "${authoring}.hidden.XXXXXX")  # a reserved unique sibling
    mv "$authoring" "$aside/tree"    # a copy this run made: now it is gone
    trap 'mv "$aside/tree" "$authoring"; rmdir "$aside"' EXIT  # restore it, drop the reservation
    barrier=moved
else                                    # a checkout that cannot move: cover it
    cat > "$out/hide-authoring.sh" <<'SH'  # kept out of the package
#!/bin/sh
a=$(realpath "$1"); shift                     # the authoring delivery root
mount --bind "$a" "$a" && mount -t tmpfs -o size=65536 tmpfs "$a" || exit 9
exec "$@"                                      # the gate, with the tree hidden
SH
    chmod +x "$out/hide-authoring.sh"
    gate() {   # not -UmR: -R is --root and would swallow the helper path
        unshare --user --map-root-user --mount -- \
            "$out/hide-authoring.sh" "$authoring" "$PY" "$@"; }
    barrier=masked
fi
# the probe is USD's own loader, not Python's open: denying Python reads leaves
# USD's C++ layer reads untouched
gate -c 'import sys; from pxr import Usd; sys.exit(0 if Usd.Stage.Open(sys.argv[1]) else 1)' \
    "$known" >"$out/probe.out" 2>"$out/probe.err" && opened=0 || opened=$?
if [ "$opened" = 0 ]; then
    echo "barrier broken: USD still opens $known" >&2
    cat "$out/probe.out" "$out/probe.err" >&2; exit 1
fi
grep -qiE 'could not open|failed to open' "$out/probe.err" || {
    echo "isolation not established: report the package gate as not run" >&2
    cat "$out/probe.err" >&2; exit 1; }        # no namespace, or the mask failed
echo "barrier: $barrier"

run_gates "$root" "$out/extracted"

status=0
for tree in authoring extracted; do
    printf '%s gates:' "$tree"
    for gate in "${gate_names[@]}"; do
        printf ' %s=%s' "$gate" "$(cat "$out/$tree/$gate.rc")"
        [ "$(cat "$out/$tree/$gate.rc")" = 0 ] || status=1
    done
    echo
done
# each extracted record must match the authoring baseline for that gate, apart
# from the anonymous layer identifiers, which embed a process address
anons='s/^layer with no file: anon:[^ ]*/layer with no file: anon:<id>/'
for gate in "${gate_names[@]}"; do
    cmp -s "$out/extracted/$gate.out" "$out/authoring/$gate.out" \
        || { echo "DIFFER $gate stdout"; status=1; }
    diff <(sed "$anons" "$out/authoring/$gate.err") \
         <(sed "$anons" "$out/extracted/$gate.err") >/dev/null \
        || { echo "DIFFER $gate stderr"; status=1; }
done
exit $status                        # nonzero on any gate failure or record gap
```

The packaged entry point reads `RIGEXEC_RUNTIME_MODULES`, passes it to
`package_paths`, and prints it, so every exception to "everything came from
the extraction" is written down somewhere: the engine, its compiled extension
module, the numerics, and the harness modules the environment injects. A
module the delivery is supposed to carry is never one of them.

### Triage an import failure before recommending a fix

Name the failing class with command evidence, then repair that class only.

The check block exits, so triage the same archive from the paths that record
printed — `archive:`, `extraction:`, `records:` — never a re-extraction or a
rebuild. This block's exit status says only that the evidence was collected.
Completed searches with no match count as collected; missing evidence,
evidence that is not a readable regular file, an archive that fails to list,
a tools listing or git status that fails, or a search that fails to read its
input exits 1. It never restates a gate's verdict, which stays in that gate's
`$out/<tree>/<gate>.rc`.

```bash
zip=$(realpath <archive>)                     # the record's "archive:" line
root=$(realpath <extraction>)                 # ... its "extraction:" line
out=$(realpath <records>)                     # ... and its "records:" line
authoring=$(realpath "$ENTRY_AUTHORING_DIR")  # restored by the check's trap
for required in "$zip" "$root/<entry>/verify.py" "$out/extracted/verify.err"; do
    [ -f "$required" ] && [ -r "$required" ] \
        || { echo "unreadable required evidence: $required" >&2; exit 1; }
done                                          # required evidence, or stop here
members_rc=0                                  # keep the status out of errexit
members=$(unzip -Z1 "$zip") || members_rc=$?  # list members BEFORE filtering
[ "$members_rc" = 0 ] \
    || { echo "cannot list the members of $zip (listing status $members_rc)" >&2
         exit 1; }                            # an unlistable archive proves nothing
printf '%s\n' "$members" | grep '\.py$' \
    || echo "the archive carries no python member"   # a completed empty listing
src_rc=0
grep -n 'sys\.path\|^from \|^import ' "$root/<entry>/verify.py" || src_rc=$?
case $src_rc in                               # where the verifier looks
    0) ;;                                     # source lines printed above
    1) echo "the verifier source names no search path or imports" ;;
    *) echo "the search could not read $root/<entry>/verify.py" >&2
       exit 1 ;;                              # grep's own read failure
esac
tools_rc=0
ls "$authoring/<entry>/tools" || tools_rc=$?  # what the authoring tree has
[ "$tools_rc" = 0 ] \
    || { echo "cannot list the authoring tools at $authoring/<entry>/tools (listing status $tools_rc)" >&2
         exit 1; }                            # missing tools evidence proves nothing
git_rc=0
git -C "$authoring" status --porcelain || git_rc=$?   # untracked helpers are the cause
[ "$git_rc" = 0 ] \
    || { echo "cannot read git status at $authoring (status $git_rc)" >&2
         exit 1; }                            # missing status evidence proves nothing
log_rc=0
grep -n 'not in the package' "$out/extracted/verify.err" || log_rc=$?
case $log_rc in                               # 1 is a plain no-match, not a read error
    0) ;;                                     # matches printed above: foreign-file class
    1) echo "no foreign file in this record" ;;
    *) echo "the search could not read $out/extracted/verify.err" >&2
       exit 1 ;;                              # grep's own read failure
esac
```

- Environment: `No module named 'pxr'`, `'numpy'`, or `'rigexec'`, or
  `Cannot find a valid schema`. The interpreter or plugin path was not
  retained. Re-source `bin/_env.sh` or `bin\_env.bat` and re-run the same
  archive: the package is not at fault, so do not rebuild it.
- Omitted member: the authoring listing has the file and the archive listing
  does not. Add the helper to the package where the entry point's search path
  resolves it, then rebuild the archive and extract the rebuild.
- Mislocated member: the file is in the archive, but outside the packaged
  entry point's search path. Move the member to where the import resolves, or
  fix the entry point's script-relative search path so the two agree, then
  rebuild the archive and extract the rebuild.
- Foreign file: the gate's own assertions passed and the record still names a
  `layer:` or `module ...:` path outside the extraction. That file was found
  outside the delivery -- a `PYTHONPATH` or `sys.path` directory, the
  interpreter's site-packages, an installed copy, or the authoring tree -- so
  the archive is incomplete even though this run worked. Carry the member and
  repackage. Adding the containing directory to `PYTHONPATH`, or installing the
  helper into site-packages, is not a repair; either one is what hid the gap.

Never patch only the extracted copy. A repair that changes the package makes
the recorded digest and member listing stale: re-run the whole headless package
check on a fresh extraction of the new archive.

### Report the package gate

Record each gate as pass, fail, or `not run`, from its own saved exit code and
records, naming the blocking module or missing member for a `not run`, and
whether each extracted record matched its authoring baseline apart from the
anonymous identifiers. A gate whose record is empty because its script never
started, or stopped at an import, is `not run`, not a failure of the check it
contains. A gate whose own assertions passed while its record names a file
outside the extraction is a fail: the run worked because of a file the archive
does not carry. A run that stops at import measured nothing, so it is never
evidence about weights, gait, deformation, or media. A run that could not
isolate the authoring tree is also `not run`: it is neither a pass nor a defect
in the archive. Report the head id, archive digest, environment, the barrier
that proved isolation (`moved` or `masked`), the runtime modules the run named,
and the foreign-file list even when it is empty, and keep the package-gate
record separate from the independent quality verdict of the rig (for example an
advisory model's medium verdict). An authoring-tree run is not package evidence.

Deliver links to the final entry point, fit, bindings, and composition, plus the
rebuild command and checks actually run. Keep the component library reusable;
an existing character's legacy monolithic generator is not the new assembly's
rebuild path.
