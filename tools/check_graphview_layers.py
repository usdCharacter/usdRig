#!/usr/bin/env python3
import argparse
import ast
import json
import pathlib
import subprocess
import sys
import tempfile

GRAPHVIEW_FILE = "plugin/usdNoodles/graphView.py"
PACKAGE_DIR = "plugin/usdNoodles/graphview"
VIEW_FILE = f"{PACKAGE_DIR}/view.py"
INVENTORY = "tools/graphViewLayers.json"
CLASS_NAME = "GraphView"
LAYERS = (
    "model",
    "intents",
    "editing",
    "interaction",
    "selection",
    "render",
    "stage_sync",
    "coordinator",
    "compat",
)
PLUGIN_CEILING = 3000
VIEW_CEILING = 800
MODEL_WRITERS = {"model", "selection", "editing", "view"}
RIG_EDITOR_MODULES = {
    f"plugin/rigExecUsdview/{name}.py" for name in (
        "gizmoFrames", "gizmoRotation", "gizmoTargets", "gizmoWidgets",
        "gizmoModel", "graphCanvas", "graphEditorModel", "volumeWeightModel",
    )
}

QT_ROOTS = {"PySide2", "PySide6", "QtCore", "QtGui", "QtWidgets", "QtOpenGL"}
GL_ROOTS = {"OpenGL"}
USDVIEWQ = "Usdviewq"


def _physical_lines(path):
    with path.open(encoding="utf-8", errors="replace") as source:
        return sum(1 for _ in source)


def _read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _graphview_definitions(root):
    """Return (definitions, direct count, class line), or None if unreadable.

    Definitions are the direct FunctionDef bodies of class GraphView, so a
    nested callback is not a layer to assign. A getter/setter pair shares its
    name and one entry, so the count of definitions and the count of unique
    names differ; the first definition seen supplies the reported line.
    """
    path = root / GRAPHVIEW_FILE
    if not path.is_file():
        return None
    tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    definitions = {}
    total = 0
    class_line = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == CLASS_NAME:
            if not class_line:
                class_line = node.lineno
            for child in node.body:
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    total += 1
                    definitions.setdefault(child.name, child.lineno)
    return list(definitions.items()), total, class_line


def _package_modules(root):
    package = root / PACKAGE_DIR
    if not package.is_dir():
        return []
    return [
        path.relative_to(root).as_posix()
        for path in sorted(package.rglob("*.py"))
        if path.is_file()
    ]


def _plugin_files(root):
    if (root / ".git").exists():
        # Keep tracked files even when ignored; use the repository's ignore
        # rules for untracked files, not a separate architecture allowlist.
        paths = subprocess.check_output(
            ["git", "ls-files", "-z", "--cached", "--others",
             "--exclude-per-directory=.gitignore", "--", ":(glob)plugin/**/*.py"],
            cwd=root,
        )
        return sorted({rel for rel in paths.decode("utf-8", "surrogateescape").split("\0")
                       if rel and (root / rel).is_file()})
    return sorted(path.relative_to(root).as_posix()
                  for path in (root / "plugin").rglob("*.py") if path.is_file())


def _writer_import(node, rel):
    if isinstance(node, ast.Import):
        candidates = [alias.name for alias in node.names]
    else:
        module = node.module or ""
        if node.level:
            parent = pathlib.PurePosixPath(rel).parent.parts[2:]
            if node.level > len(parent) + 1:
                return None
            prefix = parent[:len(parent) + 1 - node.level]
            module = ".".join((*prefix, module)) if module else ".".join(prefix)
        candidates = [module, *(f"{module}.{alias.name}" for alias in node.names)]
    for candidate in candidates:
        parts = candidate.split(".")
        if parts[0].casefold() == "usdnoodles":
            parts = parts[1:]
        if len(parts) >= 2 and parts[0] == "graphview" and parts[1] in MODEL_WRITERS:
            return parts[1]
    return None


def _bound_symbols(body, receiver=None):
    """Collect direct methods and explicit descriptor bindings, not class constants."""
    names = {}
    for node in body:
        if receiver is None and isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.setdefault(node.name, node.lineno)
        if not isinstance(node, ast.Assign) or not isinstance(
            node.value, (ast.Name, ast.Attribute, ast.Call)
        ):
            continue
        for target in node.targets:
            if receiver is None and isinstance(target, ast.Name):
                names.setdefault(target.id, node.lineno)
            elif (isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name)
                  and target.value.id == receiver):
                names.setdefault(target.attr, node.lineno)
    return names


def _composed_symbols(root):
    for module, owner, receiver in (
        ("view", CLASS_NAME, None),
        ("interaction", "GraphWidgetBase", None),
        ("compat", "bind_compat", "cls"),
    ):
        rel = f"{PACKAGE_DIR}/{module}.py"
        path = root / rel
        if not path.is_file():
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue  # R2 reports the parse failure for checked modules.
        for node in tree.body:
            if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name == owner:
                yield rel, node.lineno, _bound_symbols(node.body, receiver)
                break
        else:
            yield rel, 1, {}


def _check_symbols(rel, class_line, names, symbols, *, composed=False):
    violations = []
    for name, line in names.items():
        layer = symbols.get(name)
        if layer is None:
            violations.append(
                f'{rel}:{line}: R1 {name}: no layer assigned; add "{name}": '
                f'"<layer>" to the {rel} symbols map in {INVENTORY}'
            )
        elif layer == "unassigned":
            violations.append(
                f"{rel}:{line}: R1 {name}: layer is still unassigned; "
                f"pick one of {', '.join(LAYERS)}"
            )
        elif layer not in LAYERS:
            violations.append(
                f"{rel}:{line}: R1 {name}: layer {layer!r} is not in the "
                f"vocabulary ({', '.join(LAYERS)})"
            )
        elif composed and layer == "compat" and name != "_loadSampleGraph":
            violations.append(
                f"{rel}:{line}: R1 {name}: record its owning layer, not the compat binding"
            )
    for name in sorted(symbols.keys() - names.keys()):
        violations.append(
            f"{rel}:{class_line}: R1 {name}: the inventory assigns a layer to a "
            f"definition GraphView no longer has; remove the stale entry"
        )
    return violations


def _forbidden_module(dotted):
    """The dotted name itself when it names Qt, OpenGL or pxr.Usdviewq.

    UsdNoodles.core, UsdNoodles.render and UsdNoodles.spatial are native
    re-exports, not UI bindings, so they never match here.
    """
    segments = dotted.split(".") if dotted else []
    if not segments:
        return None
    if segments[0] in QT_ROOTS or segments[0] in GL_ROOTS:
        return dotted
    if USDVIEWQ in segments:
        return dotted
    return None


def _forbidden_import(module, names):
    """Name of the offending Qt/GL/Usdviewq import, or None."""
    for candidate in [module or ""] + list(names):
        offending = _forbidden_module(candidate)
        if offending:
            return offending
    return None


def _check(root, inventory):
    violations = []
    files = inventory.get("files") or {}

    entry = files.get(GRAPHVIEW_FILE)
    shim = (entry or {}).get("layer") == "compat"
    if entry is not None and not shim and entry.get("class") != CLASS_NAME:
        violations.append(
            f"{GRAPHVIEW_FILE}:0: R1 {INVENTORY} records class "
            f"{entry.get('class')!r} for {GRAPHVIEW_FILE}, expected "
            f"{CLASS_NAME}"
        )
    definitions = _graphview_definitions(root)
    if definitions is None:
        violations.append(
            f"{GRAPHVIEW_FILE}:0: R1 {GRAPHVIEW_FILE} is missing; the "
            f"inventory cannot be verified"
        )
    else:
        names, _, class_line = definitions
        if shim:
            legacy_tree = ast.parse((root / GRAPHVIEW_FILE).read_text(encoding="utf-8"))
            reexports = any(
                isinstance(node, ast.ImportFrom)
                and node.level == 1 and node.module == "graphview.view"
                and any(alias.name == CLASS_NAME and alias.asname in (None, CLASS_NAME)
                        for alias in node.names)
                for node in legacy_tree.body
            )
            canonical = root / VIEW_FILE
            owns_class = canonical.is_file() and any(
                isinstance(node, ast.ClassDef) and node.name == CLASS_NAME
                and any(isinstance(base, ast.Name) and base.id == "GraphWidgetBase"
                        for base in node.bases)
                for node in ast.parse(canonical.read_text(encoding="utf-8")).body
            )
            if class_line or not reexports or not owns_class:
                violations.append(
                    f"{GRAPHVIEW_FILE}:1: R1 compatibility shim must re-export "
                    f"{VIEW_FILE}'s {CLASS_NAME}(GraphWidgetBase) without defining it"
                )
        violations.extend(_check_symbols(
            GRAPHVIEW_FILE, class_line, dict(names), (entry or {}).get("symbols", {}),
        ))
        if shim:
            for rel, line, bound in _composed_symbols(root):
                violations.extend(_check_symbols(
                    rel, line, bound, files.get(rel, {}).get("symbols", {}), composed=True,
                ))

    plugin_files = _plugin_files(root)
    modules = set(_package_modules(root)) | RIG_EDITOR_MODULES.intersection(plugin_files)
    for rel in sorted(set(files) - modules - {GRAPHVIEW_FILE}):
        violations.append(f"{rel}:0: R2 {INVENTORY} lists a module outside the source tree")
    for rel in sorted(modules):
        path = root / rel
        module_entry = files.get(rel)
        if module_entry is None:
            violations.append(
                f"{rel}:1: R2 module has no layer entry in {INVENTORY}; "
                f'add "{rel}": {{"layer": "<layer>"}} to the files map'
            )
            continue
        layer = module_entry.get("layer")
        if layer is None:
            violations.append(
                f"{rel}:1: R2 {rel} has no layer value in {INVENTORY}"
            )
        elif layer == "unassigned":
            violations.append(
                f"{rel}:1: R2 {rel}: layer is still unassigned; pick one of "
                f"{', '.join(LAYERS)}"
            )
        elif layer not in LAYERS:
            violations.append(
                f"{rel}:1: R2 {rel}: layer {layer!r} is not in the "
                f"vocabulary ({', '.join(LAYERS)})"
            )
        if layer == "interaction":
            continue
        try:
            tree = ast.parse(
                path.read_text(encoding="utf-8", errors="replace")
            )
        except SyntaxError as error:
            violations.append(f"{rel}:{error.lineno or 1}: R2 cannot parse {rel}")
            continue
        for node in ast.walk(tree):
            if layer == "render":
                if rel.startswith(f"{PACKAGE_DIR}/") and isinstance(node, (ast.Import, ast.ImportFrom)):
                    writer = _writer_import(node, rel)
                    if writer:
                        violations.append(
                            f"{rel}:{node.lineno}: R4 {writer} imported by a render-layer "
                            "module; consume Frame records instead. A static import check "
                            "does not prove read-only behavior; behavioral version tests do."
                        )
                continue
            if isinstance(node, ast.Import):
                offending = _forbidden_import("", [alias.name for alias in node.names])
            elif isinstance(node, ast.ImportFrom):
                offending = _forbidden_import(
                    node.module or "", [alias.name for alias in node.names]
                )
            else:
                continue
            if offending:
                violations.append(
                    f"{rel}:{node.lineno}: R2 {offending} imported by a "
                    f"{layer or 'unassigned'}-layer module; Qt, OpenGL and "
                    f"pxr.Usdviewq belong to interaction and render only"
                )

    counts = {rel: _physical_lines(root / rel) for rel in plugin_files}
    for rel, count in counts.items():
        if count > PLUGIN_CEILING:
            violations.append(
                f"{rel}:{count}: R3 {count} physical lines exceeds the "
                f"{PLUGIN_CEILING}-line ceiling for plugin Python files"
            )

    ceilings = inventory.get("line_ceilings") or {}
    required = (GRAPHVIEW_FILE, VIEW_FILE) if shim or (root / VIEW_FILE).is_file() else ()
    for rel in required:
        ceiling = ceilings.get(rel)
        if type(ceiling) is not int or not 0 < ceiling <= VIEW_CEILING:
            violations.append(
                f"{rel}:0: R3 {INVENTORY} must record an explicit ceiling "
                f"of at most {VIEW_CEILING} lines"
            )

    for rel, ceiling in ceilings.items():
        path = root / rel
        if type(ceiling) is not int or ceiling <= 0:
            violations.append(
                f"{rel}:0: R3 {INVENTORY} records an invalid ceiling "
                f"{ceiling!r} for {rel}"
            )
        elif not path.is_file():
            violations.append(
                f"{rel}:0: R3 {INVENTORY} records a ceiling for {rel}, which "
                f"no longer exists"
            )
        else:
            count = counts[rel] if rel in counts else _physical_lines(path)
            if count > ceiling:
                violations.append(
                    f"{rel}:{count}: R3 {count} physical lines exceeds the "
                    f"ceiling {ceiling} recorded in {INVENTORY}; lower the "
                    f"ceiling after a move, never raise it"
                )
    return violations


def _write_inventory(root, inventory):
    added = []
    files = inventory.setdefault("files", {})
    definitions = _graphview_definitions(root)
    if definitions is not None:
        entry = files.setdefault(
            GRAPHVIEW_FILE, {"class": CLASS_NAME, "symbols": {}}
        )
        symbols = entry.setdefault("symbols", {})
        for name, _ in definitions[0]:
            if name not in symbols:
                symbols[name] = "unassigned"
                added.append(name)
    for rel in sorted(set(_package_modules(root)) | RIG_EDITOR_MODULES.intersection(_plugin_files(root))):
        if rel not in files:
            files[rel] = {"layer": "unassigned"}
            added.append(rel)
    if files.get(GRAPHVIEW_FILE, {}).get("layer") == "compat":
        for rel, _, bound in _composed_symbols(root):
            symbols = files[rel].setdefault("symbols", {})
            for name in sorted(bound.keys() - symbols.keys()):
                symbols[name] = "unassigned"
                added.append(f"{rel}:{name}")
    if added:
        (root / INVENTORY).write_text(
            json.dumps(inventory, indent=2) + "\n", encoding="utf-8", newline="\n"
        )
    return added


_SELF_CASES = [
    (
        "unassigned new method",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n\n    def _planned_probe(self):\n"
            "        pass\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        ["R1 _planned_probe"],
    ),
    (
        "stale assignment",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {
                                "_model_read": "model",
                                "_gone_method": "render",
                            },
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        ["R1 _gone_method"],
    ),
    (
        "unassigned layer value",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "unassigned"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        ["R1 _model_read: layer is still unassigned"],
    ),
    (
        "unknown layer name",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "graphics"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        ["R1 _model_read: layer 'graphics'"],
    ),
    (
        "property pair shares one entry",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    @property\n    def nodeGraph"
            "(self):\n        return self._nodeGraph\n\n    @nodeGraph.setter\n"
            "    def nodeGraph(self, value):\n        self._nodeGraph = value\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"nodeGraph": "model"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        [],
    ),
    (
        "Qt import in a stage_sync module",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/stage_sync.py": {"layer": "stage_sync"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/stage_sync.py": "from pxr.Usdviewq.qt import "
            "QTimer\n",
        },
        ["R2 pxr.Usdviewq"],
    ),
    (
        "Qt import in an interaction module",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/interaction.py": {"layer": "interaction"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/interaction.py": "from PySide6 import QtCore\n",
        },
        [],
    ),
    (
        "GL import in a render module",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/render.py": {"layer": "render"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/render.py": "from OpenGL import GL\n",
        },
        [],
    ),
    (
        "GL import in a model module",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/model.py": {"layer": "model"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/model.py": "import OpenGL.GL as gl\n",
        },
        ["R2 OpenGL.GL"],
    ),
    (
        "Qt import hidden inside a function",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/selection.py": {"layer": "selection"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/selection.py": "def _show_menu():\n"
            "    from PySide2 import QtWidgets\n"
            "    return QtWidgets\n",
        },
        ["R2 PySide2"],
    ),
    (
        "from pxr import Usdviewq",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/coordinator.py": {"layer": "coordinator"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/coordinator.py": "from pxr import Usdviewq\n",
        },
        ["R2 Usdviewq"],
    ),
    (
        "native re-export in a non-UI module",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/stage_sync.py": {"layer": "stage_sync"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/stage_sync.py": "from UsdNoodles.core import "
            "NodeGraphBlueprint\nfrom UsdNoodles.spatial import SpatialIndex\n",
        },
        [],
    ),
    (
        "module over the ceiling",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/model.py": {"layer": "model"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/model.py": "# pad\n" * (PLUGIN_CEILING + 1),
        },
        [f"R3 {PLUGIN_CEILING + 1} physical lines"],
    ),
    (
        "grown ceiling",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/model.py": {"layer": "model"},
                    },
                    "line_ceilings": {f"{PACKAGE_DIR}/model.py": 3},
                }
            ),
            f"{PACKAGE_DIR}/model.py": "a = 1\nb = 2\nc = 3\nd = 4\ne = 5\n",
        },
        ["R3 5 physical lines exceeds the ceiling 3"],
    ),
    (
        "ceiling for a removed file",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        }
                    },
                    "line_ceilings": {f"{PACKAGE_DIR}/gone.py": 800},
                }
            ),
        },
        ["records a ceiling for plugin/usdNoodles/graphview/gone.py"],
    ),
    (
        "module with no inventory entry",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/stage_sync.py": "from UsdNoodles.core import "
            "NodeGraphBlueprint\n",
        },
        ["R2 module has no layer entry"],
    ),
    (
        "module layer still unassigned",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/model.py": {"layer": "unassigned"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/model.py": "def _nodes():\n    return []\n",
        },
        ["R2 plugin/usdNoodles/graphview/model.py: layer is still unassigned"],
    ),
    (
        "dotted pxr.Usdviewq import",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/model.py": {"layer": "model"},
                    },
                    "line_ceilings": {},
                }
            ),
            f"{PACKAGE_DIR}/model.py": "import pxr.Usdviewq\nfrom OpenGL.error "
            "import PyGLError\n",
        },
        ["R2 pxr.Usdviewq", "R2 OpenGL.error"],
    ),
    (
        "inventory names the wrong class",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": "GraphViewLegacy",
                            "symbols": {"_model_read": "model"},
                        }
                    },
                    "line_ceilings": {},
                }
            ),
        },
        ["R1 tools/graphViewLayers.json records class 'GraphViewLegacy'"],
    ),
    (
        "clean tree",
        {
            GRAPHVIEW_FILE: "class GraphView:\n    def _model_read(self):\n"
            "        return self.nodes\n",
            INVENTORY: json.dumps(
                {
                    "version": 1,
                    "files": {
                        GRAPHVIEW_FILE: {
                            "class": CLASS_NAME,
                            "symbols": {"_model_read": "model"},
                        },
                        f"{PACKAGE_DIR}/stage_sync.py": {"layer": "stage_sync"},
                    },
                    "line_ceilings": {GRAPHVIEW_FILE: 3},
                }
            ),
            f"{PACKAGE_DIR}/stage_sync.py": "from UsdNoodles.core import "
            "NodeGraphBlueprint\n",
        },
        [],
    ),
]


_RENDER_PROBE = f"{PACKAGE_DIR}/render_nodes.py"
_PLUGIN_PROBE = "plugin/rigExecUsdview/testenv/testLineLimit.py"

# Start from the smallest composed view; the legacy cases above still exercise R1.
_COMPOSED_SOURCES = {
    GRAPHVIEW_FILE: "from .graphview.view import GraphView\n",
    VIEW_FILE: "class GraphView(GraphWidgetBase):\n    def refresh(self): pass\n",
    _RENDER_PROBE: "from .frame import Frame\nfrom OpenGL import GL\n",
}
_COMPOSED_INVENTORY = {
    "files": {
        GRAPHVIEW_FILE: {"layer": "compat"},
        VIEW_FILE: {"layer": "coordinator", "symbols": {"refresh": "coordinator"}},
        _RENDER_PROBE: {"layer": "render"},
    },
    "line_ceilings": {GRAPHVIEW_FILE: 800, VIEW_FILE: 800},
}


def _composed_case(label, overrides, expected):
    return (label, {**_COMPOSED_SOURCES, INVENTORY: json.dumps(_COMPOSED_INVENTORY),
                    **overrides}, expected)


_SELF_CASES += [
    _composed_case("clean composed tree at the limits", {
        VIEW_FILE: _COMPOSED_SOURCES[VIEW_FILE] + "# pad\n" * 798,
        _PLUGIN_PROBE: "# pad\n" * 3000,
    }, []),
    _composed_case("plugin test over the ceiling", {
        _PLUGIN_PROBE: "# pad\n" * 3001,
    }, [f"{_PLUGIN_PROBE}:3001: R3 3001 physical lines"]),
    _composed_case("coordinator over the ceiling", {
        VIEW_FILE: _COMPOSED_SOURCES[VIEW_FILE] + "# pad\n" * 799,
    }, [f"{VIEW_FILE}:801: R3 801 physical lines exceeds the ceiling 800"]),
    _composed_case("shim over the ceiling", {
        GRAPHVIEW_FILE: _COMPOSED_SOURCES[GRAPHVIEW_FILE] + "# pad\n" * 800,
    }, [f"{GRAPHVIEW_FILE}:801: R3 801 physical lines exceeds the ceiling 800"]),
    _composed_case("renderer imports model writers", {
        _RENDER_PROBE: "import graphview.model as m\n"
        "from UsdNoodles.graphview import selection as s\n"
        "from .editing import Editor as E\n"
        "def helper():\n    from . import view as v\n",
    }, [f"R4 {name} imported by a render-layer module" for name in
        ("model", "selection", "editing", "view")]),
    _composed_case("shim cannot hide an unassigned method", {
        VIEW_FILE: _COMPOSED_SOURCES[VIEW_FILE] + "    def new_method(self): pass\n",
    }, ["R1 new_method: no layer assigned"]),
    _composed_case("methods need an owning layer", {
        f"{PACKAGE_DIR}/compat.py": "def bind_compat(cls):\n    cls.draw = render.draw\n",
        INVENTORY: json.dumps({**_COMPOSED_INVENTORY, "files": {
            **_COMPOSED_INVENTORY["files"],
            f"{PACKAGE_DIR}/compat.py": {"layer": "compat", "symbols": {"draw": "compat"}},
        }}),
    }, ["R1 draw: record its owning layer"]),
    _composed_case("explicit ceilings cannot exceed 800 or be omitted", {
        INVENTORY: json.dumps({**_COMPOSED_INVENTORY, "line_ceilings": {VIEW_FILE: 801}}),
    }, [f"{rel}:0: R3 {INVENTORY} must record an explicit ceiling"
        for rel in (GRAPHVIEW_FILE, VIEW_FILE)]),
]


def _self_test():
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        for label, source_files, expected in _SELF_CASES:
            root = pathlib.Path(tmp) / f"case{label.replace(' ', '_')}"
            for rel, text in source_files.items():
                path = root / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8", newline="\n")
            hits = _check(root, _read_json(root / INVENTORY))
            if len(hits) != len(expected):
                failures.append(
                    f"{label}: got {len(hits)} violation(s) {hits}, expected "
                    f"{len(expected)}"
                )
                continue
            for fragment in expected:
                if not any(fragment in line for line in hits):
                    failures.append(f"{label}: missing {fragment!r} in {hits}")
    if failures:
        for failure in failures:
            print(f"self-test FAIL {failure}")
        return 1
    print(
        f"ok: graphview layer checker self-test passed ({len(_SELF_CASES)} "
        f"cases)"
    )
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Check the GraphView layer inventory against the checkout."
    )
    parser.add_argument(
        "--root",
        default=str(pathlib.Path(__file__).resolve().parent.parent),
        help="checkout root to inspect (default: the repository root)",
    )
    parser.add_argument(
        "--write-inventory",
        action="store_true",
        help='add newly appearing symbols as "unassigned" without touching '
        "existing assignments",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run the rules over synthetic sources and print a verdict",
    )
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    root = pathlib.Path(args.root).resolve()
    inventory_path = root / INVENTORY
    if not inventory_path.is_file():
        print(f"{INVENTORY}:0: the layer inventory is missing")
        return 1
    try:
        inventory = _read_json(inventory_path)
    except (json.JSONDecodeError, ValueError) as error:
        print(f"{INVENTORY}:0: {INVENTORY} is not readable JSON: {error}")
        return 1

    if args.write_inventory:
        added = _write_inventory(root, inventory)
        if added:
            print(
                f"{INVENTORY}: added {len(added)} entry(ies) as unassigned: "
                f"{', '.join(added)}"
            )

    violations = _check(root, inventory)
    if violations:
        for violation in violations:
            print(violation)
        print(f"GraphView layer check failed: {len(violations)} violation(s)")
        return 1
    definitions, total, _ = _graphview_definitions(root) or ([], 0, 0)
    symbols = (inventory.get("files", {}).get(GRAPHVIEW_FILE) or {}).get(
        "symbols"
    ) or {}
    if (inventory.get("files", {}).get(GRAPHVIEW_FILE) or {}).get("layer") == "compat":
        print(f"ok: {GRAPHVIEW_FILE} re-exports {VIEW_FILE}; "
              f"both have explicit ceilings at or below {VIEW_CEILING} lines")
    else:
        print(
            f"ok: {len(symbols)} layer assignments cover the {total} direct "
            f"definitions ({len(definitions)} unique names) of class "
            f"{CLASS_NAME} in {GRAPHVIEW_FILE}"
        )
    print(f"ok: {len(_plugin_files(root))} plugin Python files within the "
          f"{PLUGIN_CEILING}-line ceiling")
    for rel, ceiling in sorted(inventory.get("line_ceilings", {}).items()):
        print(f"ok: {rel}: {_physical_lines(root / rel)} physical lines "
              f"within the {ceiling}-line ceiling")
    return 0


if __name__ == "__main__":
    sys.exit(main())
