#!/usr/bin/env python3
"""Fail on new hidden locks in the sources that run on parallel lanes.

usage: tools/check_hidden_locks.py [--root DIR] [--write-allowlist]

Scans the compile, runtime, graph, math, and sampler libraries for the
calls AGENTS.md lists under "Hidden locks": a TfToken built from text, a
function-local static with dynamic initialization, an environment read, and
an explicit lock. Every hit must appear in tools/hidden_locks_allowlist.txt;
a hit that does not fails the check. Entries match by file, kind, and the
line's text, so unrelated edits that move a line do not break the check.

A deliberate hit (a Build-time setting, a documented coordination lock, a
token built once at load) goes into the allowlist. --write-allowlist rewrites
the allowlist from the current tree; review its diff like code.

The scan is textual. It cannot tell an SdfPath from a TfToken receiver, so
path text (GetString, GetText) is not checked here, and it does not see a
lock taken inside a library call. Review added lines against AGENTS.md too.
"""
import argparse
import collections
import pathlib
import re
import sys

SCANNED_DIRS = [
    "libs/rigExec",
    "libs/rigExecScene",
    "libs/rigExecGraph",
    "libs/rigExecMath",
    "libs/rigExecRuntime",
    "libs/rigExecSampler",
]
SUFFIXES = {".cpp", ".h"}
ALLOWLIST = "tools/hidden_locks_allowlist.txt"

_DYNAMIC_STATIC_TYPES = (
    r"TfToken|SdfPath|TfType|VtValue|auto|"
    r"std::(?:string|vector|set|map|multimap|unordered_\w+|array|regex|"
    r"function|shared_ptr|unique_ptr|deque|list|pair|tuple|initializer_list)"
)

CHECKS = [
    ("token-text", re.compile(
        r"\bTfToken\(\s*(?:\"|std::string\b|[^()\n]*\+|"
        r"[^()\n]*\.(?:c_str|data)\(\)|[A-Za-z_]\w*(?:\[[^\]]*\])+\s*[,)]|"
        r"[a-z_]\w*\s*\))")),
    ("token-text", re.compile(r"\bTfToken::Find\(")),
    ("token-text", re.compile(
        r"^\s+(?:const\s+)?TfToken\s+\w+\s*[({]\s*(?:\"|std::string\b|"
        r"[^(){}\n]*\+|[a-z_]\w*(?:\[[^\]]*\])*\s*[)}])")),
    ("local-static", re.compile(
        r"^\s+static\s+(?:thread_local\s+)?(?:const\s+)?"
        r"(?:" + _DYNAMIC_STATIC_TYPES + r")\b[^;(]*?\s[\w\[\]]+\s*(?:=|\{|\()")),
    ("local-static", re.compile(
        r"^\s+static\s+(?:const\s+)?(?:bool|int|double|float|size_t)\s+\w+\s*="
        r"\s*[^;]*\w\(")),
    ("env", re.compile(r"\b(?:std::)?getenv\(|\bTfGetenv\w*\(|\bTfGetEnvSetting\(")),
    ("lock", re.compile(
        r"\bstd::(?:recursive_|shared_|timed_)?mutex\b|\b(?:lock_guard|unique_lock|"
        r"scoped_lock|shared_lock)\b|\bcall_once\b|\bcondition_variable\b")),
]

# A static member function declaration: functions are UpperCamel (optionally
# with a leading underscore) in this code base, variables are not.
_FUNCTION_DECL = re.compile(r"^\s+static\s+[\w:<>,\s\*&]+?\s+_?[A-Z]\w*\s*\(")


def _normalize(line):
    return " ".join(line.strip().split())


def _scan(root):
    hits = []
    for top in SCANNED_DIRS:
        base = root / top
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix not in SUFFIXES or not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            text = path.read_text(encoding="utf-8", errors="replace")
            in_block_comment = False
            for number, line in enumerate(text.splitlines(), 1):
                stripped = line.strip()
                if in_block_comment:
                    if "*/" in stripped:
                        in_block_comment = False
                    continue
                if stripped.startswith("/*") and "*/" not in stripped:
                    in_block_comment = True
                    continue
                if stripped.startswith(("//", "*", "#include")):
                    continue
                code = line.split("//", 1)[0]
                for kind, pattern in CHECKS:
                    if not pattern.search(code):
                        continue
                    if kind == "local-static" and _FUNCTION_DECL.match(code):
                        continue
                    hits.append((rel, kind, _normalize(code), number))
                    break
    return hits


def _read_allowlist(path):
    allowed = collections.Counter()
    if not path.exists():
        return allowed
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        parts = [part.strip() for part in raw.split(" | ", 2)]
        if len(parts) == 3:
            allowed[tuple(parts)] += 1
    return allowed


def _write_allowlist(path, hits):
    lines = [
        "# Deliberate hits of tools/check_hidden_locks.py: file | kind | line.",
        "# Each is a Build/Open-time setting, a documented coordination lock,",
        "# a token or table built once at load, or a cold diagnostic path.",
        "# Regenerate with --write-allowlist and review the diff like code.",
    ]
    previous = None
    for rel, kind, text, _ in sorted(hits, key=lambda h: (h[0], h[3])):
        if rel != previous:
            lines.append("")
            previous = rel
        lines.append(f"{rel} | {kind} | {text}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=pathlib.Path(__file__).resolve().parents[1],
                        type=pathlib.Path, help="repository root")
    parser.add_argument("--write-allowlist", action="store_true",
                        help="rewrite the allowlist from the current tree")
    args = parser.parse_args()
    root = args.root.resolve()
    hits = _scan(root)
    allowlist = root / ALLOWLIST
    if args.write_allowlist:
        _write_allowlist(allowlist, hits)
        print(f"wrote {len(hits)} entries to {ALLOWLIST}")
        return 0
    allowed = _read_allowlist(allowlist)
    remaining = collections.Counter(allowed)
    new = []
    for rel, kind, text, number in hits:
        key = (rel, kind, text)
        if remaining[key] > 0:
            remaining[key] -= 1
        else:
            new.append((rel, number, kind, text))
    for rel, number, kind, text in new:
        print(f"{rel}:{number}: {kind}: {text}")
    stale = sum(remaining.values())
    if new:
        print(f"\n{len(new)} new hidden-lock site(s). Remove them (AGENTS.md, "
              f"'Hidden locks'), or add deliberate ones to {ALLOWLIST}.")
        return 1
    print(f"ok: {len(hits)} known site(s), none new"
          + (f"; {stale} allowlist entr{'y' if stale == 1 else 'ies'} no longer "
             f"match (run --write-allowlist to prune)" if stale else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
