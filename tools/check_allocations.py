#!/usr/bin/env python3
"""Fail on new bare `new` allocation expressions in owned C++ sources.

usage: tools/check_allocations.py [--root DIR] [--write-allowlist] [--self-test]

Scans every C++ file the repository owns for the allocation expressions
AGENTS.md bans under "Allocation and ownership". In C++ the token `new` in
code is always an allocation expression, so every `new` keyword outside a
comment, a string literal or an `operator new` definition is a hit: bare
`new T`, `new T{}`, `new T`, `new T[n]`, an allocation split across lines,
and placement `new (buffer) T` (reported as its own kind so a reviewer sees
the buffer it lands in). `make_unique`/`make_shared` are the replacements.
Macro bodies are scanned: `#define MAKE_WIDGET() new Widget()` is a hit.
Only `#include` is exempt, since `#include <new>` names the allocator header
rather than allocating. A raw string ends at its own delimiter, so a bare `)`
inside `R"tag(...)tag"` does not close it. Blanked regions keep their
newlines, so every hit reports the line it is really on, and a region with no
newline is replaced by a space so `new/**/Widget()` is still a hit.
Every hit must appear in tools/allocations_allowlist.txt; a hit that does not
fails the check. Entries match by file, kind, and the line's text, so
unrelated edits that move a line do not break the check.

A deliberate hit (a passkey factory that reaches a private constructor, a
TfRefPtr adopted once, a documented process-lifetime `.release()` contract)
goes into the allowlist. --write-allowlist rewrites the allowlist from the
current tree; review its diff like code.

The scan is textual: it cannot tell whether a `new` inside a template or a
macro expansion owns anything, so review added lines against AGENTS.md too.
Third-party bindings and generated FlatBuffers headers are not scanned.
--self-test runs the extractor against synthetic sources covering each form.
"""
import argparse
import collections
import pathlib
import re
import sys
import tempfile

SCANNED_DIRS = [
    "libs",
    "tests",
    "tools",
    "python",
    "plugin/rigExecUsdview",
    "plugin/rigExecSchema",
    "examples",
]
EXCLUDED_DIRS = {
    "third_party",
    "thirdparty",
    "plugin/usdNoodles",
    "libs/rigExecBinary/generated",
}
SUFFIXES = {".cpp", ".h", ".hpp", ".cc", ".cxx", ".inl"}
ALLOWLIST = "tools/allocations_allowlist.txt"

_NEW = re.compile(r"\bnew\b")
# `operator new` and `operator new[]` are the allocator itself, not a hit.
_OPERATOR_NEW = re.compile(r"operator\s+new\b")
# An include names the allocator header; every other directive's body can
# still hold an allocation the rule bans.
_INCLUDE = re.compile(r"#\s*include\b")


def _code_of(text):
    """Return the source with comments and string contents blanked out.

    Blanked regions keep their newlines so a hit still reports its real line,
    and a region with no newline becomes one space so the tokens on either
    side stay separate: `new/**/Widget()` is an allocation, not `newWidget()`.
    """
    out = []
    i, n = 0, len(text)

    def skip(end):
        region = text[i:end]
        newlines = region.count("\n")
        out.append("\n" * newlines if newlines else " ")
        return end

    while i < n:
        ch = text[i]
        if ch == "/" and i + 1 < n and text[i + 1] == "/":
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if ch == "/" and i + 1 < n and text[i + 1] == "*":
            j = text.find("*/", i + 2)
            i = skip(n if j < 0 else j + 2)
            continue
        if ch == "R" and i + 1 < n and text[i + 1] == '"':
            # R"delim(...)delim": the closing ) belongs to the delimiter, so
            # a bare ) inside the body does not end the literal.
            opener = text.find("(", i + 2)
            if opener < 0:
                i = skip(n)
                continue
            delimiter = text[i + 2:opener]
            closer = text.find(")" + delimiter + '"', opener + 1)
            i = skip(n if closer < 0 else closer + len(delimiter) + 2)
            continue
        if ch == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            i = skip(min(j + 1, n))
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def _normalize(line):
    return " ".join(line.strip().split())


def _kind_of(code, start):
    """Classify the `new` at `start`; None when it is not an allocation."""
    if _OPERATOR_NEW.search(code[max(0, start - 12) : start + 4]):
        return None
    line = code.splitlines()[code[:start].count("\n")]
    if _INCLUDE.match(line.lstrip()):
        return None
    tail = code[start + 3:]
    if re.match(r"\s*\(", tail):
        return "placement-new"
    return "bare-new"


def _scan_text(rel, text):
    code = _code_of(text)
    hits = []
    for match in _NEW.finditer(code):
        kind = _kind_of(code, match.start())
        if kind is None:
            continue
        number = code[:match.start()].count("\n") + 1
        line = code.splitlines()[number - 1]
        hits.append((rel, kind, _normalize(line), number))
    return hits


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
            if any(rel.startswith(exc) for exc in EXCLUDED_DIRS):
                continue
            hits.extend(_scan_text(rel, path.read_text(
                encoding="utf-8", errors="replace")))
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
        "# Deliberate hits of tools/check_allocations.py: file | kind | line.",
        "# Each is a passkey factory, a TfRefPtr adopted once, a documented",
        "# process-lifetime .release() contract, or a test that deliberately",
        "# keeps an evaluator open. Regenerate with --write-allowlist and",
        "# review the diff like code.",
    ]
    previous = None
    for rel, kind, text, _ in sorted(hits, key=lambda h: (h[0], h[3])):
        if rel != previous:
            lines.append("")
            previous = rel
        lines.append(f"{rel} | {kind} | {text}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


# Each case is (source, expected hits), a hit being (kind, line). An empty
# expectation means the extractor must find nothing: comments, strings,
# `#include <new>` and `operator new` are not allocations, and
# make_unique/make_shared are the replacements AGENTS.md asks for. Line
# numbers are checked too, so a blanked multiline region cannot shift a hit.
_SELF_CASES = [
    ("auto p = new Widget();", [("bare-new", 1)]),
    ("auto p = new Widget;", [("bare-new", 1)]),
    ("auto p = new Widget{};", [("bare-new", 1)]),
    ("auto p = new Widget{1, 2};", [("bare-new", 1)]),
    ("auto p = new Widget[4];", [("bare-new", 1)]),
    ("auto p = new Widget[4]{};", [("bare-new", 1)]),
    ("auto p = new std::vector<int>(n);", [("bare-new", 1)]),
    ("auto p = new rigExec::Widget();", [("bare-new", 1)]),
    ("auto p = new\n    Widget(1, 2);", [("bare-new", 1)]),
    ("auto p = std::unique_ptr<Widget>(\n    new Widget());", [("bare-new", 2)]),
    ("items.push_back(std::make_unique<Widget>());", []),
    ("auto p = std::make_shared<Widget>();", []),
    ("// auto p = new Widget();", []),
    ("/* new Widget[3] */", []),
    ('TF_WARN("old style: new Widget()");', []),
    ("#include <new>", []),
    ("void *operator new(std::size_t n);", []),
    ("void *operator new[](std::size_t n);", []),
    ("new (arena) Widget;", [("placement-new", 1)]),
    ("new (buffer) Widget[2];", [("placement-new", 1)]),
    ("renewWidget();", []),
    ("News news;", []),
    # A raw string ends at )delim", not at the first bare ) in its body.
    ('const char *s = R"tag(text)tag";\nauto p = new Widget();',
     [("bare-new", 2)]),
    ('auto s = R"(new Widget())";', []),
    ('auto s = R"del(new Widget[4])del";\nauto p = new Widget[2];',
     [("bare-new", 2)]),
    ('auto s = R"(\nnew Widget();\n)";\nauto p = new Widget();',
     [("bare-new", 4)]),
    # Macro bodies allocate too; only includes are exempt.
    ("#define MAKE_WIDGET() new Widget()", [("bare-new", 1)]),
    ("#define PLACE(arena) new (arena) Widget()", [("placement-new", 1)]),
    ("#define MAKE() \\\n    new Widget()", [("bare-new", 2)]),
    ("#pragma once\n#define WIDGET() new Widget", [("bare-new", 2)]),
    # Blanked regions must not move the line a hit reports on.
    ("/*\nnew Widget[3];\n*/\nauto p = new Widget();", [("bare-new", 4)]),
    ('"a string with new Widget()\nspanning lines"\nauto p = new Widget();',
     [("bare-new", 3)]),
    # Blanking a comment must keep the space between the tokens it separates.
    ("auto p = new/**/Widget();", [("bare-new", 1)]),
    ("auto p = new /* inline */ Widget[4];", [("bare-new", 1)]),
    ("auto p = /* note\nline */\n    new Widget();", [("bare-new", 3)]),
    ("auto p = new\n/**/Widget[2]();", [("bare-new", 1)]),
    ("renew/**/Widget();", []),
]


def _self_test():
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        for index, (source, expected) in enumerate(_SELF_CASES):
            hits = _scan_text(f"case{index}.cpp", source + "\n")
            found = [(kind, number) for _, kind, _, number in hits]
            if found != expected:
                failures.append(f"{source!r}: got {found}, expected {expected}")
    if failures:
        for failure in failures:
            print(f"self-test FAIL {failure}")
        return 1
    print(f"ok: allocation checker self-test passed ({len(_SELF_CASES)} cases)")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=pathlib.Path(__file__).resolve().parents[1],
                        type=pathlib.Path, help="repository root")
    parser.add_argument("--write-allowlist", action="store_true",
                        help="rewrite the allowlist from the current tree")
    parser.add_argument("--self-test", action="store_true",
                        help="check the extractor against synthetic sources")
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
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
        print(f"\n{len(new)} new bare-allocation site(s). Replace them with "
              f"make_unique/make_shared (AGENTS.md, 'Allocation and "
              f"ownership'), or add deliberate ones to {ALLOWLIST}.")
        return 1
    print(f"ok: {len(hits)} known site(s), none new"
          + (f"; {stale} allowlist entr{'y' if stale == 1 else 'ies'} no longer "
             f"match (run --write-allowlist to prune)" if stale else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
