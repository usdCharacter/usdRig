#!/usr/bin/env python3
"""Run the dropped-OpenUSD-library checker over synthetic sources and this
checkout.

usage: tests/testRigExecPxrLibraryChecker.py

tools/check_pxr_libraries.py is a textual scan, so its scanner is
golden-tested on synthetic CMake, C++ and Python sources (link lines, label
lists, includes, every import form, and comments and prim-type strings that
are not dependencies), then the repository scan runs so a new use fails the
test rather than only CI.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_pxr_libraries  # noqa: E402  (tools/check_pxr_libraries.py)


def main():
    if check_pxr_libraries.main(["--root", str(ROOT), "--self-test"]):
        return 1
    return check_pxr_libraries.main(["--root", str(ROOT)])


if __name__ == "__main__":
    sys.exit(main())
