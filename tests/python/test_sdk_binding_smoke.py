#!/usr/bin/env python
"""SDK binding smoke: the SDK modules resolve inside the tree under test and
a native numeric binding returns the kernel's value.

Usage: test_sdk_binding_smoke.py <expected module root>

<expected module root> is the directory that owns the resolved ``rigexec``
package and ``_rigexec`` extension -- the build-tree python directory when
ctest invokes it. A module resolved from anywhere else (an installed SDK
shadowing the build tree, or vice versa) fails the test, so an SDK smoke
never reports against the wrong binary.
"""
import os
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import rigexec      # noqa: E402
import _rigexec     # noqa: E402


def _resolved_under(expected_root):
    failures = []
    for name, module in (("rigexec", rigexec), ("_rigexec", _rigexec)):
        path = os.path.realpath(module.__file__)
        if os.path.commonpath([expected_root, path]) != expected_root:
            failures.append(f"{name} resolved outside {expected_root}: {path}")
    return failures


def main():
    if len(sys.argv) != 2:
        print("usage: test_sdk_binding_smoke.py <expected module root>")
        return 2
    expected_root = os.path.realpath(sys.argv[1])

    failures = _resolved_under(expected_root)
    if failures:
        for line in failures:
            print("FAIL:", line)
        return 1

    # rigexec.bone2d_clamp_local_angle is the skeleton2d kernel's angle
    # clamp: value bounded to [rest_local + min_angle, rest_local +
    # max_angle]. The bounds are returned exactly, so equality holds.
    cases = (
        (10.0, -30.0, 45.0, 100.0, 55.0),
        (10.0, -30.0, 45.0, -100.0, -20.0),
        (10.0, -30.0, 45.0, 20.0, 20.0),
    )
    for rest, lo, hi, value, want in cases:
        got = rigexec.bone2d_clamp_local_angle(rest, lo, hi, value)
        if got != want:
            print(f"FAIL: bone2d_clamp_local_angle({rest}, {lo}, {hi}, "
                  f"{value}) -> {got}, want {want}")
            return 1
    print("ok: _rigexec", os.path.realpath(_rigexec.__file__))
    return 0


if __name__ == "__main__":
    sys.exit(main())
