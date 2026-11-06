"""Reject incomplete CTest JUnit evidence before the full-suite job passes."""

import sys
import xml.etree.ElementTree as ET


def verify(path):
    root = ET.parse(path).getroot()
    cases = list(root.iter("testcase"))
    if not cases:
        raise ValueError("No tests executed")
    for case in cases:
        if (any(case.find(tag) is not None for tag in ("skipped", "failure", "error"))
                or case.get("status", "run") in ("notrun", "disabled")):
            raise ValueError(f"Test did not pass: {case.get('name')}")
    for suite in root.iter("testsuite"):
        for attr in ("failures", "errors", "skipped", "disabled"):
            if int(suite.get(attr, "0")):
                raise ValueError(f"Suite reports {attr}")
    print(f"All {len(cases)} tests passed; none skipped")


if __name__ == "__main__":
    verify(sys.argv[1])
