#!/usr/bin/env python3
"""Run the bare-allocation checker over synthetic sources and this checkout.

usage: tests/testRigExecAllocationChecker.py

AGENTS.md ("Allocation and ownership") bans bare `new` expressions in the
owned sources. tools/check_allocations.py is a textual scan, so its extractor
is golden-tested here on synthetic sources covering every banned form (bare,
brace-init, array, split across lines, placement) and every non-hit (comments,
string literals, `#include <new>`, `operator new` definitions, renewWidget,
News), then the repository scan runs so a new hit fails the test rather than
only CI.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_allocations  # noqa: E402  (tools/check_allocations.py)


def main():
    if check_allocations.main(["--root", str(ROOT), "--self-test"]):
        return 1
    return check_allocations.main(["--root", str(ROOT)])


if __name__ == "__main__":
    sys.exit(main())
