#!/usr/bin/env python3
"""Fail when a GraphView definition carries no layer, a non-UI module under
plugin/usdNoodles/graphview/ imports Qt, OpenGL or pxr.Usdviewq, or a module
outgrows the line ceiling recorded with its layer.

usage: tools/check_graphview_layers.py [--root DIR] [--write-inventory] [--self-test]

Reads the layer assignments and ceilings in tools/graphViewLayers.json.
Standard library only: no build tree, no OpenUSD, no Qt. Ceilings ratchet
downward -- lower them in the inventory, never raise them. --write-inventory
marks newly appearing definitions "unassigned" and never rewrites an existing
assignment, so a reviewer still has to choose the layer. --self-test runs the
rules over synthetic sources. Exit 0 when clean, 1 with one line per violation.
"""
import argparse
import ast
import json
import pathlib
import sys
import tempfile

GRAPHVIEW_FILE = "plugin/usdNoodles/graphView.py"
PACKAGE_DIR = "plugin/usdNoodles/graphview"
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
UI_LAYERS = {"interaction", "render"}
MODULE_CEILING = 3000

QT_ROOTS = {"PySide2", "PySide6", "QtCore", "QtGui", "QtWidgets", "QtOpenGL"}
GL_ROOTS = {"OpenGL"}
USDVIEWQ = "Usdviewq"


def _physical_lines(path):
    return len(path.read_text(encoding="utf-8", errors="replace").splitlines())


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
                if isinstance(child, ast.FunctionDef):
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
    if entry is not None and entry.get("class") != CLASS_NAME:
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
        symbols = (entry or {}).get("symbols") or {}
        for name, line in names:
            layer = symbols.get(name)
            if layer is None:
                violations.append(
                    f"{GRAPHVIEW_FILE}:{line}: R1 {name}: no layer assigned "
                    f'in {INVENTORY}; add "{name}": "<layer>" to the '
                    f"{GRAPHVIEW_FILE} symbols map"
                )
            elif layer == "unassigned":
                violations.append(
                    f"{GRAPHVIEW_FILE}:{line}: R1 {name}: layer is still "
                    f"unassigned; pick one of {', '.join(LAYERS)}"
                )
            elif layer not in LAYERS:
                violations.append(
                    f"{GRAPHVIEW_FILE}:{line}: R1 {name}: layer {layer!r} is "
                    f"not in the vocabulary ({', '.join(LAYERS)})"
                )
        for name, layer in symbols.items():
            if name not in dict(names):
                violations.append(
                    f"{GRAPHVIEW_FILE}:{class_line}: R1 {name}: the inventory "
                    f"assigns a layer to a definition class GraphView no "
                    f"longer has; remove the stale entry"
                )

    modules = _package_modules(root)
    for rel in modules:
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
        if layer in UI_LAYERS:
            continue
        try:
            tree = ast.parse(
                path.read_text(encoding="utf-8", errors="replace")
            )
        except SyntaxError as error:
            violations.append(f"{rel}:{error.lineno or 1}: R2 cannot parse {rel}")
            continue
        for node in ast.walk(tree):
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

    for rel in modules:
        count = _physical_lines(root / rel)
        if count > MODULE_CEILING:
            violations.append(
                f"{rel}:{count}: R3 {count} physical lines exceeds the "
                f"{MODULE_CEILING}-line ceiling for modules under "
                f"{PACKAGE_DIR}"
            )

    for rel, ceiling in (inventory.get("line_ceilings") or {}).items():
        path = root / rel
        if not isinstance(ceiling, int) or ceiling <= 0:
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
            count = _physical_lines(path)
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
    for rel in _package_modules(root):
        if rel not in files:
            files[rel] = {"layer": "unassigned"}
            added.append(rel)
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
            f"{PACKAGE_DIR}/model.py": "# pad\n" * (MODULE_CEILING + 1),
        },
        [f"R3 {MODULE_CEILING + 1} physical lines"],
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
    print(
        f"ok: {len(symbols)} layer assignments cover the {total} direct "
        f"definitions ({len(definitions)} unique names) of class "
        f"{CLASS_NAME} in {GRAPHVIEW_FILE}; "
        f"{len(_package_modules(root))} module(s) under {PACKAGE_DIR} within "
        f"the {MODULE_CEILING}-line ceiling"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
