#!/usr/bin/env python3
"""Fail on new uses of rigExec's internal headers from outside rigExec.

usage: tools/check_internal_includes.py [--root DIR] [--write-allowlist]

rigExec's public headers live in libs/rigExec/api/ and everything else in
libs/rigExec/internal/. Two checks:

1. Includes. A file outside libs/rigExec/ may include rigExec/internal/
   only from libs/rigExecBinary/, libs/rigExecBake/ or tests/, or when the
   (file, header) pair is in tools/internal_includes_allowlist.txt. The
   allowlist records today's leaks; it may only shrink.

2. Installed headers. Every installed header must compile from the install
   tree, so the internal headers that installed headers reach must be
   installed too. tools/rigexec_installed_internal.txt lists them and CMake
   installs exactly that list; the check fails when the list and the
   computed closure differ.

--write-allowlist rewrites both files from the current tree; review the
diff like code.
"""
import argparse
import pathlib
import re
import sys

ALLOWLIST = "tools/internal_includes_allowlist.txt"
INSTALLED = "tools/rigexec_installed_internal.txt"
INTERNAL = "rigExec/internal/"
ALLOWED_PREFIXES = ("libs/rigExecBinary/", "libs/rigExecBake/", "tests/")
SOURCE_SUFFIXES = {".h", ".cpp", ".mm", ".cc", ".hpp", ".inl"}
SKIPPED_DIRS = {"build", "thirdparty", ".git"}
# Libraries whose headers install; keep in step with CMakeLists.txt.
INSTALLED_LIBS = (
    "rigExecGraph rigExecEvalCore rigExecScene rigExecMath rigExec "
    "rigExecSchema rigExecStandalone rigExecAuthoring rigExecImaging "
    "rigExecGpu rigExecBinary rigExecBake rigExecRuntime rigExecSampler "
    "rigExecStdTypes rigExecRigTypes rigExecBuiltins"
).split()
NOT_INSTALLED = {"guideBoundsInternal.h", "registryInternal.h",
                 "evaluatorTesting.h"}
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')


def _sources(root):
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root)
        if rel.parts and rel.parts[0] in SKIPPED_DIRS:
            continue
        if path.suffix in SOURCE_SUFFIXES and path.is_file():
            yield rel.as_posix(), path


def _includes(path):
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = INCLUDE.match(line)
        if match:
            yield match.group(1)


def _include_hits(root):
    hits = set()
    for rel, path in _sources(root):
        if rel.startswith("libs/rigExec/") or rel.startswith(ALLOWED_PREFIXES):
            continue
        for header in _includes(path):
            if header.startswith(INTERNAL):
                hits.add((rel, header))
    return hits


def _resolve(root, path, header):
    """Repo-relative path of an include, or None outside the repo libs."""
    for candidate in (root / "libs" / header, path.parent / header):
        if candidate.is_file():
            return candidate.resolve().relative_to(root.resolve()).as_posix()
    return None


def _installed_internal(root):
    stack = []
    for lib in INSTALLED_LIBS:
        base = root / "libs" / lib
        for path in sorted(base.rglob("*.h")):
            rel = path.relative_to(root).as_posix()
            if path.name in NOT_INSTALLED or "/internal/" in rel:
                continue
            stack.append(rel)
    seen = set()
    while stack:
        rel = stack.pop()
        if rel in seen:
            continue
        seen.add(rel)
        for header in _includes(root / rel):
            resolved = _resolve(root, root / rel, header)
            if resolved:
                stack.append(resolved)
    prefix = "libs/rigExec/internal/"
    return sorted(s[len(prefix):] for s in seen if s.startswith(prefix))


def _read_list(path):
    if not path.is_file():
        return []
    return [line.strip() for line in path.read_text().splitlines()
            if line.strip() and not line.startswith("#")]


def _write(root, hits, installed):
    (root / ALLOWLIST).write_text("\n".join(
        ["# Includes of rigExec/internal/ from outside rigExec: file | header.",
         "# Each is a leak to remove; the list may only shrink.",
         "# Regenerate with tools/check_internal_includes.py --write-allowlist.",
         ""] + [f"{f} | {h}" for f, h in sorted(hits)]) + "\n")
    (root / INSTALLED).write_text("\n".join(
        ["# Internal headers that installed headers still include, relative",
         "# to libs/rigExec/internal/. CMake installs exactly these; each is a",
         "# leak through the public headers to remove.",
         "# Regenerate with tools/check_internal_includes.py --write-allowlist.",
         ""] + installed) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=pathlib.Path(__file__).parent.parent,
                        type=pathlib.Path)
    parser.add_argument("--write-allowlist", action="store_true",
                        help="rewrite both lists from the current tree")
    args = parser.parse_args()
    root = args.root
    hits = _include_hits(root)
    installed = _installed_internal(root)
    if args.write_allowlist:
        _write(root, hits, installed)
        print(f"wrote {len(hits)} include(s), {len(installed)} installed header(s)")
        return 0

    allowed = set()
    for line in _read_list(root / ALLOWLIST):
        file, _, header = line.partition(" | ")
        allowed.add((file.strip(), header.strip()))
    failed = False
    for file, header in sorted(hits - allowed):
        print(f"{file}: includes {header}")
        failed = True
    if failed:
        print("\nOnly rigExecBinary, rigExecBake and tests include rigExec "
              "internals. Use a header from rigExec/api/ instead.")

    listed = _read_list(root / INSTALLED)
    if listed != installed:
        for header in sorted(set(installed) - set(listed)):
            print(f"installed headers now reach internal/{header}")
        for header in sorted(set(listed) - set(installed)):
            print(f"internal/{header} is no longer reached; drop it from {INSTALLED}")
        failed = True
    if failed:
        return 1
    stale = len(allowed - hits)
    print(f"ok: {len(hits)} known include(s), {len(installed)} installed "
          f"internal header(s)"
          + (f"; {stale} allowlist entr{'y' if stale == 1 else 'ies'} no longer "
             f"match (run --write-allowlist to prune)" if stale else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
