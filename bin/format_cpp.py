#!/usr/bin/env python3
"""Check or format changed first-party C/C++ files with clang-format 18.1.8.

Install: uv tool install clang-format==18.1.8
Check:   python3 bin/format_cpp.py --base origin/main
Fix:     python3 bin/format_cpp.py --base origin/main --fix
Audit:   python3 bin/format_cpp.py --all
Explicit file paths may be passed instead of --base or --all.
"""
import argparse
import difflib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

VERSION = "18.1.8"
ROOT = Path(__file__).resolve().parents[1]
EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".inl"}
EXCLUDED = ("third_party/", "thirdparty/", "plugin/usdNoodles/", "libs/rigExecBinary/generated/")


def git(root, *args):
    return subprocess.check_output(["git", *args], cwd=root).decode("utf-8")


def eligible(name):
    path = Path(name)
    name = path.as_posix()
    return (path.suffix in EXTENSIONS and not name.startswith(EXCLUDED)
            and not any(part.startswith("build") for part in path.parts[:-1]))


def select_files(root, base=None, all_files=False, paths=()):
    if paths:
        names = paths
    elif all_files:
        names = git(root, "ls-files", "-z").split("\0")
    else:
        # The working tree is compared with the branch point, including staged
        # edits. Renames contribute their destination; deletions are omitted.
        ancestor = git(root, "merge-base", "HEAD", base or "origin/main").strip()
        names = git(root, "diff", "--name-only", "-z", "--diff-filter=ACMR", ancestor, "--").split("\0")
        names += git(root, "ls-files", "--others", "--exclude-standard", "-z").split("\0")
    selected = []
    for name in sorted(set(names)):
        if not name:
            continue
        name = Path(name).as_posix()
        if not eligible(name):
            continue
        path = root / name
        if Path(name).is_absolute() or ".." in Path(name).parts or path.is_symlink():
            raise ValueError(f"not a repository source file: {name}")
        if not path.resolve().is_relative_to(root):
            raise ValueError(f"source escapes repository: {name}")
        if path.is_file():
            selected.append(name)
        elif paths and not path.exists():
            raise ValueError(f"file does not exist: {name}")
    return selected


def formatter():
    binary = os.environ.get("CLANG_FORMAT") or shutil.which("clang-format") or shutil.which("clang-format-18")
    if not binary:
        raise ValueError(f"Install clang-format: uv tool install clang-format=={VERSION}")
    version = subprocess.check_output([binary, "--version"], text=True)
    if not re.search(r"\bversion " + re.escape(VERSION) + r"\b", version):
        raise ValueError(f"Expected clang-format {VERSION}, got {version.strip()}")
    return binary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*")
    parser.add_argument("--repo", type=Path, default=ROOT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--base")
    group.add_argument("--all", action="store_true")
    parser.add_argument("--fix", action="store_true")
    args = parser.parse_args(argv)
    if args.paths and (args.base or args.all):
        parser.error("explicit paths cannot be combined with --base or --all")
    root = args.repo.resolve()
    try:
        files = select_files(root, args.base, args.all, args.paths)
        if not files:
            print("clang-format: no changed first-party C/C++ files")
            return 0
        binary = formatter()
        # Use the candidate's policy, never silently fall back to LLVM defaults.
        config = root / ".clang-format"
        if not config.is_file():
            raise ValueError(f"Missing formatting policy: {config}")
        changed = 0
        for name in files:
            path = root / name
            original = path.read_bytes()
            formatted = subprocess.check_output([binary, f"--style=file:{config}",
                f"--assume-filename={path}", "--fallback-style=none"], input=original, cwd=root)
            if formatted == original:
                continue
            changed += 1
            if args.fix:
                path.write_bytes(formatted)
            else:
                sys.stdout.writelines(difflib.unified_diff(
                    original.decode().splitlines(keepends=True), formatted.decode().splitlines(keepends=True),
                    fromfile=name, tofile=name + " (formatted)"))
        print(f"clang-format: {len(files)} file(s) checked; {changed} {'formatted' if args.fix else 'need formatting'}")
        return 0 if args.fix or not changed else 1
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(f"clang-format: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
