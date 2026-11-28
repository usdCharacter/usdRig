#!/usr/bin/env python3
"""Fail when a target includes headers whose symbols or include path it
does not link, a break that only Linux and macOS show.

usage: tools/check_test_links.py [--root DIR] [--self-test]

cmake/WindowsArchiveLinks.cmake gives Windows targets links the root
CMakeLists.txt does not declare, so a target can build on Windows and fail
on the other platforms. Two checks, each over every executable the root
CMakeLists.txt declares (tests, benches, probes and tools):

1. FlatBuffers include path. A source includes a header that reaches the
   vendored flatbuffers/ headers (directly or through other headers under
   libs/), and the target links no library that carries the
   rigExecFlatBuffers include path publicly.
2. rigExecMathGf symbols. A source includes libs/rigExecMath/<name>.h for a
   <name>.cpp in the rigExecMathGf archive, and the target does not link
   rigExecMathGf. The rigExec shared library exports only the archive
   members it references itself.

The carrier sets are derived from the PUBLIC and INTERFACE link lines: a
library carries a seed when it links the seed, or a carrier, publicly. The
CMake text is read with the parser of tools/select_affected_tests.py, so
the trailing foreach() blocks that add rigExecMathGf and rigExecFlatBuffers
to a list of targets count as links.

A source's own includes are followed, plus ONE level of quoted includes
into tests/*.h helpers; a deeper chain of test helpers is not followed.
Headers under libs/ are followed to any depth for the FlatBuffers check.
--self-test runs the check over synthetic CMake text and sources.
"""
import argparse
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import select_affected_tests as selector  # noqa: E402

FLATBUFFERS = "rigExecFlatBuffers"
MATH_ARCHIVE = "rigExecMathGf"
LEGACY_MATH_ARCHIVE = "rigExecMath"
FLATBUFFERS_INCLUDE_ROOT = "thirdparty/flatbuffers/include"
INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', re.MULTILINE)


def _include_path_seeds(cmake_text):
    """Targets that name the vendored FlatBuffers include directory in a
    PUBLIC or INTERFACE target_include_directories()."""
    text = selector.expand_foreach(selector.strip_comments(cmake_text))
    seeds = {FLATBUFFERS}
    for name, args in selector.iter_commands(text):
        if name != "target_include_directories" or len(args) < 3:
            continue
        public = args[1] in {"PUBLIC", "INTERFACE"}
        if public and any(FLATBUFFERS_INCLUDE_ROOT in arg for arg in args[2:]):
            seeds.add(args[0])
    return seeds


def _carriers(model, seeds):
    """Libraries that carry a seed to a linker: the seeds and every library
    that links a carrier publicly."""
    libraries = model["libraries"]
    carriers = set(seeds)
    changed = True
    while changed:
        changed = False
        for name, info in libraries.items():
            if name not in carriers and carriers & set(
                    selector.public_names(info)):
                carriers.add(name)
                changed = True
    return carriers


def _linked(model, target):
    """Every library a target links, directly or through public links."""
    edges = {name: selector.public_names(info)
             for name, info in model["libraries"].items()}
    return selector.closure(set(model["executables"][target]["links"]), edges)


class _Tree:
    """Includes of the files under a root, read through a callable."""

    def __init__(self, read):
        self._read = read
        self._includes = {}
        self._reaches = {}

    def includes(self, rel):
        if rel not in self._includes:
            text = self._read(rel)
            self._includes[rel] = ([] if text is None else
                                   [(m.group(1), m.group(2))
                                    for m in INCLUDE.finditer(text)])
        return self._includes[rel]

    def resolve(self, name, quote, origin):
        """Repo-relative file an include names, or None."""
        candidates = []
        if quote == '"':
            candidates.append(
                str(pathlib.PurePosixPath(origin).parent / name))
        candidates += ["libs/" + name, "tests/" + name,
                       FLATBUFFERS_INCLUDE_ROOT + "/" + name]
        for rel in candidates:
            if self._read(rel) is not None:
                return rel
        return None

    def reaches_flatbuffers(self, rel, active=()):
        if rel in self._reaches:
            return self._reaches[rel]
        if rel.startswith(FLATBUFFERS_INCLUDE_ROOT + "/"):
            return True
        found = False
        for quote, name in self.includes(rel):
            target = self.resolve(name, quote, rel)
            if target and target not in active and \
                    self.reaches_flatbuffers(target, active + (rel,)):
                found = True
                break
        self._reaches[rel] = found
        return found


def _direct_includes(tree, source):
    """(include text, header file, helper or None) a source includes, with
    one level of quoted includes followed into tests/*.h helpers."""
    found = []
    for quote, name in tree.includes(source):
        rel = tree.resolve(name, quote, source)
        if rel is None:
            continue
        found.append((name, rel, None))
        if rel.startswith("tests/") and rel.endswith(".h"):
            for q2, n2 in tree.includes(rel):
                inner = tree.resolve(n2, q2, rel)
                if inner is not None:
                    found.append((n2, inner, name))
    return found


def check(cmake_text, read):
    """[violation line] for the CMake text and a path -> text-or-None
    reader."""
    model = selector.parse(cmake_text)
    libraries = model["libraries"]
    archive = MATH_ARCHIVE if MATH_ARCHIVE in libraries else \
        LEGACY_MATH_ARCHIVE
    for seed in (FLATBUFFERS, archive):
        if seed not in libraries:
            return [f"CMakeLists.txt: no add_library({seed}); the lint "
                    f"cannot derive its carrier set"]
    flatbuffer_carriers = _carriers(model, _include_path_seeds(cmake_text))
    math_carriers = _carriers(model, {archive})
    math_headers = {}
    for source in libraries[archive]["sources"]:
        path = pathlib.PurePosixPath(source)
        math_headers["libs/rigExecMath/" + path.stem + ".h"] = path.stem
    tree = _Tree(read)
    violations = []
    for target, info in sorted(model["executables"].items()):
        linked = _linked(model, target)
        needs_flatbuffers = {}
        needs_math = {}
        for source in info["sources"]:
            for name, rel, helper in _direct_includes(tree, source):
                via = helper or "-"
                helper_header = rel.startswith("tests/") and rel.endswith(".h")
                if (not linked & flatbuffer_carriers and not helper_header
                        and tree.reaches_flatbuffers(rel)):
                    needs_flatbuffers.setdefault((name, via), None)
                if rel in math_headers and archive not in linked:
                    needs_math.setdefault((name, via), None)
        for (name, via), _ in sorted(needs_flatbuffers.items()):
            where = f" (via {via})" if via != "-" else ""
            violations.append(
                f"{target}: includes {name}{where} but links no library "
                f"carrying the {FLATBUFFERS} include path "
                f"({', '.join(sorted(flatbuffer_carriers))})")
        for (name, via), _ in sorted(needs_math.items()):
            where = f" (via {via})" if via != "-" else ""
            violations.append(
                f"{target}: includes {name}{where} but links no library "
                f"carrying {archive} symbols (link {archive} "
                f"directly; carriers: "
                f"{', '.join(sorted(math_carriers))})")
    return violations


def _disk_reader(root):
    def read(rel):
        path = root / rel
        if not path.is_file():
            return None
        return path.read_text(encoding="utf-8", errors="replace")
    return read


_SYNTH_CMAKE = """
add_library(rigExecFlatBuffers INTERFACE)
add_library(rigExecMathGf STATIC libs/rigExecMath/solvers.cpp)
add_library(rigExecMath STATIC libs/rigExecMath/other.cpp)
add_library(rigExecOpCore SHARED libs/rigExec/internal/a.cpp)
add_library(rigExecBinary STATIC libs/rigExecBinary/format.cpp)
target_link_libraries(rigExecBinary PUBLIC rigExecFlatBuffers rigExecOpCore)
add_library(rigExecEvalCore STATIC libs/rigExecEvalCore/kinds.cpp)
target_include_directories(rigExecEvalCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/flatbuffers/include>)
target_link_libraries(rigExecEvalCore PUBLIC rigExecOpCore)
add_library(rigExec SHARED libs/rigExec/a.cpp)
target_link_libraries(rigExec PRIVATE rigExecMathGf rigExecEvalCore
    PUBLIC rigExecOpCore)
"""
_SYNTH_FILES = {
    "libs/rigExecBinary/format.h": '#include "flatbuffers/flatbuffers.h"\n',
    "thirdparty/flatbuffers/include/flatbuffers/flatbuffers.h": "\n",
    "libs/rigExecEvalCore/step.h": '#include "rigExecBinary/format.h"\n',
    "libs/rigExecEvalCore/leaf.h": "#include <vector>\n",
    "libs/rigExecMath/solvers.h": "\n",
    "libs/rigExecMath/other.h": "\n",
    "tests/helper.h": '#include "rigExecEvalCore/step.h"\n',
    "tests/outer.h": '#include "helper.h"\n',
    "tests/fb.cpp": '#include "rigExecEvalCore/step.h"\n',
    "tests/leaf.cpp": '#include "rigExecEvalCore/leaf.h"\n',
    "tests/viaHelper.cpp": '#include "helper.h"\n',
    "tests/deep.cpp": '#include "outer.h"\n',
    "tests/math.cpp": '#include "rigExecMath/solvers.h"\n',
    "tests/inlineMath.cpp": '#include "rigExecMath/other.h"\n',
}
_FB = "t: includes rigExecEvalCore/step.h"
_NO_FB = " but links no library carrying the rigExecFlatBuffers"
_MATH = ("t: includes rigExecMath/solvers.h but links no library carrying "
         "rigExecMathGf symbols")
_FOREACH = ("foreach(_u t)\n  if(TARGET ${_u})\n"
            "    target_link_libraries(${_u} PRIVATE %s)\n"
            "  endif()\nendforeach()")
_SELF_CASES = [
    ("add_executable(t tests/fb.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)", [_FB + _NO_FB]),
    ("add_executable(t tests/fb.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec rigExecEvalCore)", []),
    ("add_executable(t tests/fb.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec rigExecBinary)", []),
    ("add_executable(t tests/fb.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)\n"
     + _FOREACH % "rigExecFlatBuffers", []),
    ("add_executable(t tests/leaf.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)", []),
    ("add_executable(t tests/viaHelper.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)",
     [_FB + " (via helper.h)" + _NO_FB]),
    ("add_executable(t tests/deep.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)", []),
    ("add_executable(t tests/math.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)", [_MATH]),
    ("add_executable(t tests/math.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec rigExecMathGf)", []),
    ("add_executable(t tests/math.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)\n"
     + _FOREACH % "rigExecMathGf", []),
    ("add_executable(t tests/inlineMath.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)", []),
    ("if (RIGEXEC_WITH_USD)\nadd_executable(t tests/math.cpp)\n"
     "target_link_libraries(t PRIVATE rigExec)\nendif()", [_MATH]),
]


def _self_test():
    failures = []
    for extra, expected in _SELF_CASES:
        found = check(_SYNTH_CMAKE + extra + "\n", _SYNTH_FILES.get)
        if len(found) != len(expected) or not all(
                line.startswith(want)
                for line, want in zip(found, expected)):
            failures.append(f"{extra!r}: got {found}, expected {expected}")
    if failures:
        for failure in failures:
            print(f"self-test FAIL {failure}")
        return 1
    print(f"ok: test link checker self-test passed ({len(_SELF_CASES)} "
          f"cases)")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root",
                        default=pathlib.Path(__file__).resolve().parents[1],
                        type=pathlib.Path, help="repository root")
    parser.add_argument("--self-test", action="store_true",
                        help="check the lint against synthetic sources")
    parser.add_argument("--cmake", type=pathlib.Path,
                        help="CMakeLists.txt to parse instead of the root's")
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
    root = args.root.resolve()
    cmake = args.cmake or root / "CMakeLists.txt"
    violations = check(cmake.read_text(encoding="utf-8"), _disk_reader(root))
    for violation in violations:
        print(violation)
    if violations:
        print(f"\n{len(violations)} violation(s): these targets build on "
              f"Windows only. Link the named library in the root "
              f"CMakeLists.txt (the rigExecMathGf and rigExecFlatBuffers "
              f"foreach() lists at its end).")
        return 1
    print("ok: every target links what its includes need")
    return 0


if __name__ == "__main__":
    sys.exit(main())
