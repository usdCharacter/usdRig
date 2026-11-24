#!/usr/bin/env python3
"""Fail when usdRig links, includes or imports an OpenUSD library it dropped.

usage: tools/check_pxr_libraries.py [--root DIR] [--self-test]

FORBIDDEN names the OpenUSD libraries no usdRig target may depend on. Three
checks, each textual:

1. Links. A CMake file the repository owns names a forbidden library outside
   a comment (a target_link_libraries edge, a component list, a label list).
2. C/C++ includes. A source includes a header under pxr/usd/<library>/ or
   pxr/usdImaging/<library>/.
3. Python imports. A module imports pxr.<Module> or names <Module> in a
   `from pxr import` list.

A prim type or attribute the library's schema defines is data, not a
dependency: read and author it by name through the generic Usd/Sdf API.
--self-test runs the scanner against synthetic sources covering each form.
"""
import argparse
import pathlib
import re
import sys

# OpenUSD library name -> its Python module name.
FORBIDDEN = {
    "usdSkel": "UsdSkel",
}
SCANNED_DIRS = [
    "cmake",
    "libs",
    "tests",
    "tools",
    "python",
    "plugin",
    "examples",
    "bin",
]
EXCLUDED_DIRS = {
    "third_party",
    "thirdparty",
    "plugin/usdNoodles",
    "libs/rigExecBinary/generated",
    # Imported-target description of Houdini's own USD build, not a usdRig
    # target.
    "cmake/houdini",
}
CMAKE_NAMES = {"CMakeLists.txt"}
CMAKE_SUFFIXES = {".cmake"}
CXX_SUFFIXES = {".h", ".hpp", ".cpp", ".cc", ".mm", ".inl"}
PY_SUFFIXES = {".py"}
SELF = "tools/check_pxr_libraries.py"
SELF_TEST = "tests/testRigExecPxrLibraryChecker.py"

_LIB_WORD = re.compile(
    r"(?<![A-Za-z0-9_])(?:usd_)?(" + "|".join(FORBIDDEN) + r")(?![A-Za-z0-9_])")
_INCLUDE = re.compile(
    r'^\s*#\s*include\s*[<"]pxr/(?:usd|usdImaging)/('
    + "|".join(FORBIDDEN) + r')[A-Za-z]*/')
_PY_MODULES = "|".join(FORBIDDEN.values())
_PY_DOTTED = re.compile(
    r"^\s*(?:import|from)\s+pxr\.(" + _PY_MODULES + r")\b")
_PY_FROM = re.compile(r"^\s*from\s+pxr\s+import\s+(.*)$")


def _cmake_code(line):
    """The part of a CMake line before a # comment, outside quoted text."""
    quoted = False
    for index, char in enumerate(line):
        if char == '"':
            quoted = not quoted
        elif char == "#" and not quoted:
            return line[:index]
    return line


def _py_from_names(text, start):
    """The names a `from pxr import` statement brings in, following a
    parenthesized or backslash-continued list onto later lines."""
    lines = text.split("\n")
    match = _PY_FROM.match(lines[start])
    body = match.group(1).split("#", 1)[0]
    index = start
    if "(" in body:
        while ")" not in body and index + 1 < len(lines):
            index += 1
            body += " " + lines[index].split("#", 1)[0]
    else:
        while body.rstrip().endswith("\\") and index + 1 < len(lines):
            index += 1
            body = body.rstrip()[:-1] + " " + lines[index].split("#", 1)[0]
    names = re.split(r"[\s,()\\]+", body)
    return [name for name in names if name and name != "as"]


def _scan_text(rel, text):
    """[(rel, line number, kind, text)] for every forbidden use in one file."""
    path = pathlib.PurePosixPath(rel)
    hits = []
    lines = text.split("\n")
    if path.name in CMAKE_NAMES or path.suffix in CMAKE_SUFFIXES:
        for number, line in enumerate(lines, 1):
            if _LIB_WORD.search(_cmake_code(line)):
                hits.append((rel, number, "link", line.strip()))
    elif path.suffix in CXX_SUFFIXES:
        for number, line in enumerate(lines, 1):
            if _INCLUDE.match(line):
                hits.append((rel, number, "include", line.strip()))
    elif path.suffix in PY_SUFFIXES:
        modules = set(FORBIDDEN.values())
        for number, line in enumerate(lines, 1):
            if _PY_DOTTED.match(line):
                hits.append((rel, number, "import", line.strip()))
            elif _PY_FROM.match(line) and modules & set(
                    _py_from_names(text, number - 1)):
                hits.append((rel, number, "import", line.strip()))
    return hits


def _excluded(rel):
    return any(rel == excluded or rel.startswith(excluded + "/")
               for excluded in EXCLUDED_DIRS)


def _candidates(root):
    yield root / "CMakeLists.txt"
    for top in SCANNED_DIRS:
        base = root / top
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            rel = path.relative_to(root).as_posix()
            if _excluded(rel) or rel in (SELF, SELF_TEST) or not path.is_file():
                continue
            if (path.name in CMAKE_NAMES or path.suffix in CMAKE_SUFFIXES
                    or path.suffix in CXX_SUFFIXES
                    or path.suffix in PY_SUFFIXES):
                yield path


def _scan(root):
    hits = []
    for path in _candidates(root):
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        hits.extend(_scan_text(path.relative_to(root).as_posix(), text))
    return hits


_SELF_CASES = [
    ("CMakeLists.txt", "target_link_libraries(t PUBLIC usd usdSkel)", ["link"]),
    ("CMakeLists.txt", "target_link_libraries(t PUBLIC usd\n    usdSkel\n)",
     ["link"]),
    ("cmake/x.cmake", "set(libs tf usdSkel work)", ["link"]),
    ("CMakeLists.txt", 'set(dll "usd_usdSkel.dll")', ["link"]),
    ("CMakeLists.txt", "# usdSkel is not linked", []),
    ("CMakeLists.txt", "target_link_libraries(t usd) # not usdSkel", []),
    ("CMakeLists.txt", "target_link_libraries(t usdSkelImaging)", []),
    ("CMakeLists.txt", "target_link_libraries(t myusdSkel)", []),
    ("CMakeLists.txt", 'message("# usdSkel")', ["link"]),
    ("a.cpp", '#include "pxr/usd/usdSkel/blendShape.h"', ["include"]),
    ("a.h", "  #  include <pxr/usd/usdSkel/tokens.h>", ["include"]),
    ("a.cpp", '#include "pxr/usdImaging/usdSkelImaging/api.h"', ["include"]),
    ("a.cpp", "// a UsdSkelBlendShape prim, read by name", []),
    ("a.cpp", 'const char *t = "BlendShape";', []),
    ("a.py", "from pxr import Gf, Usd, UsdSkel, Vt", ["import"]),
    ("a.py", "from pxr import (Gf, Usd,\n    UsdSkel)", ["import"]),
    ("a.py", "from pxr import Gf, \\\n    UsdSkel", ["import"]),
    ("a.py", "from pxr import UsdSkel as S", ["import"]),
    ("a.py", "import pxr.UsdSkel", ["import"]),
    ("a.py", "from pxr.UsdSkel import BlendShape", ["import"]),
    ("a.py", "from pxr import Usd, UsdGeom  # UsdSkel not needed", []),
    ("a.py", "from pxr import UsdSkelImaging", []),
    ("a.py", 'stage.DefinePrim(path, "BlendShape")', []),
]


def _self_test():
    failures = []
    for rel, source, expected in _SELF_CASES:
        found = [kind for _, _, kind, _ in _scan_text(rel, source + "\n")]
        if found != expected:
            failures.append(f"{rel} {source!r}: got {found}, expected "
                            f"{expected}")
    if failures:
        for failure in failures:
            print(f"self-test FAIL {failure}")
        return 1
    print(f"ok: pxr library checker self-test passed ({len(_SELF_CASES)} "
          f"cases)")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root",
                        default=pathlib.Path(__file__).resolve().parents[1],
                        type=pathlib.Path, help="repository root")
    parser.add_argument("--self-test", action="store_true",
                        help="check the scanner against synthetic sources")
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
    hits = _scan(args.root.resolve())
    for rel, number, kind, text in hits:
        print(f"{rel}:{number}: {kind}: {text}")
    if hits:
        print(f"\n{len(hits)} use(s) of a dropped OpenUSD library ("
              f"{', '.join(sorted(FORBIDDEN))}). Read and author its prims "
              f"by name through the generic Usd/Sdf API instead.")
        return 1
    print(f"ok: no target links, includes or imports "
          f"{', '.join(sorted(FORBIDDEN))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
