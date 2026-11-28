#!/usr/bin/env python3
"""Report how far two sets of RigExec golden captures lie apart.

A report, not a gate: it always exits 0 once both sides parse. It reads the
text captures \`rigExecPose --golden-out\` and the exec-row recorder write
(\`rigexec-golden 2\`) and prints, per capture, how many value rows differ and
the largest absolute difference between them, then the largest over all.

Inputs:
  BEFORE, AFTER   two .golden files, or two directories; directories are
                  matched by file name (*.golden, searched recursively).

Rows are matched by visit (leg, ordinal) and by the row's key (every token
up to and including its first quoted one), in order of appearance for a key
that repeats. Within a matched row, hexadecimal tokens of 16 digits are read
as IEEE doubles and of 8 digits as floats, so a value recorded as a double on
one side and as a float on the other still compares numerically; the token
after \`frame\` (point-frame flags), decimal integers and other text compare
exactly; a quoted string value is split at its separators first, so numbers
it carries are measured too. Input digests (\`in=\`) and digest-only lines (\`h\`, \`g\`) cannot be
measured and are counted as changed or unchanged only.

Usage:
  python3 tools/golden_deviation.py BEFORE AFTER [--rows N]

--rows N prints the N rows with the largest deviation per capture.
"""

import argparse
import math
import re
import struct
import sys
from pathlib import Path

TOKEN = re.compile(r'"(?:[^"\\]|\\.)*"|\S+')
HEX16 = re.compile(r"^[0-9a-f]{16}$")
HEX8 = re.compile(r"^[0-9a-f]{8}$")


def number(token):
    if HEX16.match(token):
        return struct.unpack(">d", bytes.fromhex(token))[0]
    if HEX8.match(token):
        return struct.unpack(">f", bytes.fromhex(token))[0]
    return None


def parse(path):
    """Header lines, value rows and digest lines of one capture."""
    header, rows, digests = [], {}, {}
    visit = ("", "")
    counts = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        tokens = TOKEN.findall(line)
        if not tokens:
            continue
        kind = tokens[0]
        if kind == "visit":
            visit = (tokens[1], tokens[2]) if len(tokens) > 2 else (line, "")
            continue
        if kind in ("h", "g"):
            key = (visit, kind, " ".join(tokens[1:-2]))
            digests[key] = " ".join(tokens[-2:])
            continue
        if kind != "v":
            header.append(line)
            continue
        quoted = next(
            (i for i, t in enumerate(tokens) if t.startswith('"')), len(tokens) - 1
        )
        name = " ".join(tokens[1 : quoted + 1])
        occurrence = counts.get((visit, name), 0)
        counts[(visit, name)] = occurrence + 1
        rows[(visit, name, occurrence)] = tokens[quoted + 1 :]
    return header, rows, digests


def flatten(tokens):
    """Value tokens, with quoted text (a string value) split at its own
    separators so the numbers it carries compare too."""
    flat = []
    for token in tokens:
        if token.startswith('"'):
            flat.extend(t for t in re.split(r'[\s;=,"]+', token) if t)
        else:
            flat.append(token)
    return flat


def compare_values(a, b):
    """(largest |difference|, exact tokens equal, digest changed)."""
    a, b = flatten(a), flatten(b)
    worst, exact, digest = 0.0, len(a) == len(b), False
    for i, (x, y) in enumerate(zip(a, b)):
        if x == y:
            continue
        if x.startswith("in=") and y.startswith("in="):
            digest = True
            continue
        flags = i > 0 and a[i - 1] == "frame" and b[i - 1] == "frame"
        u, v = (None, None) if flags else (number(x), number(y))
        if u is None or v is None:
            exact = False
            continue
        if math.isnan(u) and math.isnan(v):
            continue
        d = abs(u - v)
        worst = d if math.isnan(d) or not math.isfinite(d) else max(worst, d)
    return worst, exact, digest


def report(name, before, after, rows_to_show):
    header_a, rows_a, digests_a = parse(before)
    header_b, rows_b, digests_b = parse(after)
    shared = [k for k in rows_a if k in rows_b]
    differing, structural, digest_only, worst = 0, 0, 0, 0.0
    ranked = []
    for key in shared:
        a, b = rows_a[key], rows_b[key]
        if a == b:
            continue
        d, exact, digest = compare_values(a, b)
        if not exact:
            structural += 1
        elif d == 0.0 and digest:
            digest_only += 1
            continue
        differing += 1
        worst = max(worst, d) if not math.isnan(d) else math.nan
        ranked.append((d, key))
    digest_changed = sum(
        1 for k in digests_a if k in digests_b and digests_a[k] != digests_b[k]
    )
    only_a = len(rows_a) - len(shared)
    only_b = len(rows_b) - len(shared)
    print(
        f"{name}: {len(shared)} rows, {differing} differ, max |d| {worst:.3e}"
        + (f", {structural} not numeric" if structural else "")
        + (f", {digest_only} input digest only" if digest_only else "")
        + (f", {only_a} only before" if only_a else "")
        + (f", {only_b} only after" if only_b else "")
        + (f", {digest_changed} digest lines changed" if digest_changed else "")
        + (", header differs" if header_a != header_b else "")
    )
    ranked.sort(key=lambda r: -r[0] if not math.isnan(r[0]) else -math.inf)
    for d, (visit, row, occurrence) in ranked[:rows_to_show]:
        print(f"    {d:.3e}  {visit[0]} {visit[1]}  {row}")
    return worst


def pairs(before, after):
    if before.is_file() and after.is_file():
        return [(after.name, before, after)]
    if not (before.is_dir() and after.is_dir()):
        sys.exit("BEFORE and AFTER must both be files or both directories")
    a = {p.name: p for p in sorted(before.rglob("*.golden"))}
    b = {p.name: p for p in sorted(after.rglob("*.golden"))}
    for name in sorted(set(a) ^ set(b)):
        side = "before" if name in a else "after"
        print(f"{name}: only {side}")
    return [(name, a[name], b[name]) for name in sorted(set(a) & set(b))]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--rows", type=int, default=0)
    args = parser.parse_args()
    overall = 0.0
    for name, before, after in pairs(args.before, args.after):
        overall = max(overall, report(name, before, after, args.rows))
    print(f"overall max |d| {overall:.3e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
