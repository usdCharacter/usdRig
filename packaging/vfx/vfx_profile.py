#!/usr/bin/env python3
"""Read the VFX Reference Platform year profiles in profiles.json.

  outputs <year>            print the year's build pins as key=value lines
                            (append them to $GITHUB_OUTPUT with --github-output)
  pin-build-usd <year> <f>  point OpenUSD's build_usd.py at the year's oneTBB
                            and OpenSubdiv releases (edits <f> in place)
  notes                     print the release-notes compatibility table

Stdlib only; test_vfx_profile.py covers it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

PROFILES = Path(__file__).with_name("profiles.json")

ONETBB_URL = "https://github.com/uxlfoundation/oneTBB/archive/refs/tags/{tag}.zip"
OPENSUBDIV_URL = ("https://github.com/PixarAnimationStudios/OpenSubdiv/"
                  "archive/{tag}.zip")


def load(path: Path = PROFILES) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def year_profile(data: dict, year: str) -> dict:
    year = year.lower().removeprefix("cy").removeprefix("vfx")
    try:
        return data["years"][year]
    except KeyError:
        known = ", ".join(sorted(data["years"]))
        raise SystemExit(f"no VFX platform profile for {year} (have {known})")


def outputs(data: dict, year: str) -> dict[str, str]:
    profile = year_profile(data, year)
    values = dict(profile["build"])
    values["openusd"] = data["openusd"]
    values["year"] = year.lower().removeprefix("cy").removeprefix("vfx")
    digest = hashlib.sha256(json.dumps(
        {"openusd": data["openusd"], "build": profile["build"]},
        sort_keys=True).encode()).hexdigest()
    values["profile_hash"] = digest[:12]
    return values


def pin_build_usd(text: str, onetbb_tag: str, opensubdiv_tag: str) -> str:
    """Return build_usd.py source with the oneTBB and OpenSubdiv URLs pinned.

    Fails loudly when either assignment is missing, so a build_usd.py
    upgrade that renames them cannot silently build the wrong version.
    """
    replacements = (
        (r'^ONETBB_URL = "[^"]*"$',
         f'ONETBB_URL = "{ONETBB_URL.format(tag=onetbb_tag)}"'),
        (r'^OPENSUBDIV_URL = "[^"]*"$',
         f'OPENSUBDIV_URL = "{OPENSUBDIV_URL.format(tag=opensubdiv_tag)}"'),
    )
    for pattern, line in replacements:
        text, count = re.subn(pattern, line, text, flags=re.MULTILINE)
        if count != 1:
            raise SystemExit(
                f"build_usd.py: expected one match for {pattern!r}, "
                f"found {count}")
    return text


def notes(data: dict) -> str:
    years = sorted(data["years"])
    rows = [
        ("Linux", lambda p: f"glibc {p['platform']['glibc']}+ "
                            f"({p['build']['linux_baseline']} or newer), "
                            "x86-64"),
        ("macOS", lambda p: f"{p['platform']['macos_minimum']}+, "
                            "Apple Silicon"),
        ("Windows", lambda p: "x64, " + p["platform"]["msvc"]),
        ("C++", lambda p: f"C++{p['build']['cxx_standard']}"),
        ("Python", lambda p: p["build"]["python"]),
        ("PySide6 / Qt", lambda p: p["build"]["pyside6"]),
        ("NumPy", lambda p: p["build"]["numpy"]),
        ("oneTBB", lambda p: p["build"]["onetbb_tag"].lstrip("v")),
        ("OpenSubdiv", lambda p: p["build"]["opensubdiv_tag"]
         .lstrip("v").replace("_", ".")),
    ]
    head = "| | " + " | ".join(f"CY{y}" for y in years) + " |"
    sep = "|---|" + "---|" * len(years)
    lines = [head, sep]
    for label, cell in rows:
        lines.append(f"| {label} | " + " | ".join(
            cell(data["years"][y]) for y in years) + " |")
    lines.append(f"| OpenUSD | " + " | ".join(
        data["openusd"].lstrip("v") for _ in years) + " |")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profiles", default=str(PROFILES))
    sub = parser.add_subparsers(dest="command", required=True)
    out = sub.add_parser("outputs")
    out.add_argument("year")
    out.add_argument("--github-output", default="")
    pin = sub.add_parser("pin-build-usd")
    pin.add_argument("year")
    pin.add_argument("build_usd")
    sub.add_parser("notes")
    args = parser.parse_args(argv)
    data = load(Path(args.profiles))

    if args.command == "outputs":
        lines = [f"{k}={v}" for k, v in outputs(data, args.year).items()]
        print("\n".join(lines))
        if args.github_output:
            with open(args.github_output, "a", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
    elif args.command == "pin-build-usd":
        build = year_profile(data, args.year)["build"]
        path = Path(args.build_usd)
        path.write_text(pin_build_usd(path.read_text(encoding="utf-8"),
                                      build["onetbb_tag"],
                                      build["opensubdiv_tag"]),
                        encoding="utf-8")
        print(f"pinned {path}: oneTBB {build['onetbb_tag']}, "
              f"OpenSubdiv {build['opensubdiv_tag']}")
    else:
        sys.stdout.write(notes(data))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
