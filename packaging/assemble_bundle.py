#!/usr/bin/env python3
"""Assemble a relocatable usdRig download bundle.

Copies a complete OpenUSD install, the RigExec ``cmake --install`` prefix,
and the CPython home the USD build was compiled against into one directory
with the ``usdRigView`` launcher at its head. On Windows it also copies
the MSVC runtime from the CPython home beside the USD and RigExec DLLs.
Nothing in the output refers to the build machine: the launcher derives
every path from its own location at run time (see
packaging/launcher/usdRigView.cpp), and on macOS every Mach-O install
name is rewritten to a bundle-relative ``@loader_path`` name (see
relocate_macos_bundle); assembly fails if any non-system absolute
dependency remains.

Layout (Windows/Linux)::
    <output>/
      usdRigView[.exe]          launcher
      bundle-manifest.json
      README-bundle.md, LICENSE, NOTICE, THIRD_PARTY_NOTICES.md
      app/
        usd/                    OpenUSD install
        rig/                    RigExec install (lib, lib/python, lib/usd)
        python/                 bundled CPython home
        plugins/touchPose, plugins/shapeEditor
        examples/rigexec_flat.usda, examples/ArmShotAnim.usda

On macOS the launcher and payload are wrapped as an app instead:
``usdRigView.app/Contents/MacOS/usdRigView`` with the same ``app`` tree in
``Contents/Resources/app``, so the whole bundle can be code-signed and
notarized as one unit. Manifest, README and legal texts sit beside the
``.app``.

Stdlib only. Run ``--help`` for the inputs; every required input is
validated before anything is copied and all missing pieces are reported
at once. ``packaging/test_assemble_bundle.py`` exercises this script
against synthetic trees.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
import tarfile
import zipfile
from pathlib import Path, PurePosixPath

PLATFORMS = ("windows", "macos", "linux")
IMAGING_NAMES = ("rigExecImaging.dll", "librigExecImaging.dylib",
                 "librigExecImaging.so")
TOOL_NAMES = ("rigExecPose", "rigExecBake", "rigExecReplayInputs")
EXAMPLE_NAMES = ("rigexec_flat.usda", "ArmShotAnim.usda", "ArmRig.usda")
LEGAL_NAMES = ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md")
MSVC_RUNTIME_NAMES = ("msvcp140.dll", "msvcp140_1.dll", "msvcp140_2.dll",
                      "msvcp140_atomic_wait.dll",
                      "msvcp140_codecvt_ids.dll", "vcruntime140.dll",
                      "vcruntime140_1.dll", "concrt140.dll")

_IGNORE_RUNTIME = shutil.ignore_patterns(
    "__pycache__", "*.pyc", "*.pyo", "*.pdb")
_IGNORE_TRIM_DEV = shutil.ignore_patterns(
    "__pycache__", "*.pyc", "*.pyo", "*.pdb", "*.lib", "*.a",
    "include", "Headers", "cmake")


def _ignore(trim_dev: bool):
    return _IGNORE_TRIM_DEV if trim_dev else _IGNORE_RUNTIME


def python_executable(python_home: Path, platform: str) -> Path:
    if platform == "windows":
        return python_home / "python.exe"
    for candidate in (python_home / "bin" / "python3",
                      python_home / "bin" / "python",
                      python_home / "python3"):
        if candidate.is_file():
            return candidate
    return python_home / "bin" / "python3"


def resolve_python_home(python_home: Path, platform: str) -> Path:
    if platform != "macos":
        return python_home
    try:
        real = python_executable(python_home, platform).resolve()
    except OSError:
        return python_home
    parts = real.parts
    if "Python.framework" not in parts:
        return python_home
    framework = parts.index("Python.framework")
    try:
        versions = parts.index("Versions", framework)
    except ValueError:
        return python_home
    if versions + 1 >= len(parts):
        return python_home
    return Path(*parts[:versions + 2])


def validate(args) -> list[str]:
    problems = []
    usd = Path(args.usd_install)
    rig = Path(args.rig_install)
    python_home = Path(args.python_home)
    source = Path(args.source_root)
    if not (usd / "bin" / "usdview").is_file():
        problems.append(f"OpenUSD install has no bin/usdview: {usd}")
    if not any((rig / "lib" / name).is_file() for name in IMAGING_NAMES):
        problems.append(f"RigExec install has no imaging library: {rig}/lib")
    menu = rig / "lib" / "python" / "usdRigViewMenu.py"
    if not menu.is_file():
        problems.append(f"RigExec install is missing {menu}")
    for plug in ("rigExecSchema", "rigExecImaging"):
        info = rig / "lib" / "usd" / plug / "resources" / "plugInfo.json"
        if not info.is_file():
            problems.append(f"RigExec install is missing {info}")
    if not python_executable(python_home, args.platform).is_file():
        problems.append(
            f"Python home has no interpreter for {args.platform}: "
            f"{python_home} (the bundle needs the interpreter the "
            "OpenUSD build was compiled against, e.g. setup-python's "
            "pythonLocation)")
    if not Path(args.launcher).is_file():
        problems.append(f"launcher not found: {args.launcher}")
    for plugin in ("touchPose", "shapeEditor"):
        if not (source / "plugin" / plugin / "plugInfo.json").is_file():
            problems.append(f"source tree is missing plugin/{plugin}")
    if args.rig_build_dir and not Path(args.rig_build_dir).is_dir():
        problems.append(f"rig build dir not found: {args.rig_build_dir}")
    if args.usdnoodles_dir and not Path(args.usdnoodles_dir).is_dir():
        problems.append(f"usdNoodles dir not found: {args.usdnoodles_dir}")
    return problems


def _copy_link(link: Path, dst: Path, src_root: Path,
               trim_dev: bool) -> None:
    """Recreate an internal symlink; materialize one that escapes the tree.

    Framework-internal links (Versions/Current, a framework's binary)
    are relative and belong to the bundle, so they are kept verbatim:
    dereferencing them breaks the structure codesign seals. A link
    pointing outside the copied tree (setup-python's bin/python3 ->
    /Library/Frameworks/...) must become a real copy, or the bundle
    depends on the build machine and patching it hits the system
    interpreter.
    """
    raw = os.readlink(link)
    if os.path.isabs(raw):
        lexical = Path(os.path.normpath(raw))
    else:
        lexical = Path(os.path.normpath(link.parent.resolve() / raw))
    if lexical.is_relative_to(src_root):
        dst.parent.mkdir(parents=True, exist_ok=True)
        if os.path.lexists(dst):
            os.unlink(dst)
        os.symlink(raw, dst)
        return
    real = link.resolve()
    if real.is_dir():
        shutil.copytree(real, dst, dirs_exist_ok=True,
                        ignore=_ignore(trim_dev))
    elif real.is_file():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(real, dst)


def _copy_tree_keeping_internal_links(src: Path, dst: Path,
                                      trim_dev: bool) -> None:
    src_root = src.resolve()
    ignore = _ignore(trim_dev)
    for dirpath, dirnames, filenames in os.walk(src):
        current = Path(dirpath)
        ignored = ignore(dirpath, dirnames + filenames)
        out = dst / current.relative_to(src)
        out.mkdir(parents=True, exist_ok=True)
        for name in list(dirnames):
            entry = current / name
            if not entry.is_symlink():
                continue
            dirnames.remove(name)
            if name not in ignored:
                _copy_link(entry, out / name, src_root, trim_dev)
        dirnames[:] = [name for name in dirnames if name not in ignored]
        for name in filenames:
            if name in ignored:
                continue
            entry = current / name
            if entry.is_symlink():
                _copy_link(entry, out / name, src_root, trim_dev)
            else:
                shutil.copy2(entry, out / name)


def copy_tree(src: Path, dst: Path, trim_dev: bool,
              preserve_symlinks: bool = False) -> None:
    if preserve_symlinks:
        _copy_tree_keeping_internal_links(src, dst, trim_dev)
    else:
        shutil.copytree(src, dst, dirs_exist_ok=True,
                        ignore=_ignore(trim_dev))


def copy_msvc_runtime(app_dir: Path, python_home: Path, usd: Path,
                      rig: Path, runtime_dir: Path | None = None) -> list[str]:
    """Ship the MSVC runtime beside the Windows DLLs that need it.

    Neither the OpenUSD nor the RigExec install carries msvcp140.dll, so
    without this the bundle depends on whatever runtime the host's
    System32 (or a stray PATH entry) provides, which may predate the
    compiler the binaries were built with. The bundled CPython's
    PySide6/shiboken packages carry a redistributable set; fall back to the
    Python home and the install trees.

    With ``runtime_dir`` (the redistributable of the toolset that built the
    binaries) that runtime is used instead, and it also replaces the older
    copies anywhere in the bundled Python (PySide6 and shiboken6 carry
    their own): Windows loads one msvcp140.dll per process, and code from a
    newer toolset can fail to initialise against an older runtime (PySide6
    6.5 ships 14.29, 6.8 ships 14.39; VS 2022 17.10+ code needs 14.40+).
    """
    candidates = (
        *((runtime_dir,) if runtime_dir else ()),
        python_home / "Lib" / "site-packages" / "PySide6",
        python_home / "Lib" / "site-packages" / "shiboken6",
        python_home,
        usd / "bin",
        usd / "lib",
        rig / "lib",
    )
    source = next((d for d in candidates if (d / "msvcp140.dll").is_file()),
                  None)
    if source is None:
        return []
    copied = []
    for lib in (app_dir / "usd" / "lib", app_dir / "rig" / "lib"):
        lib.mkdir(parents=True, exist_ok=True)
        for name in MSVC_RUNTIME_NAMES:
            candidate = source / name
            if candidate.is_file():
                shutil.copy2(candidate, lib / name)
                copied.append(name)
    if runtime_dir and source == runtime_dir:
        for name in MSVC_RUNTIME_NAMES:
            if (source / name).is_file():
                for stale in (app_dir / "python").rglob(name):
                    shutil.copy2(source / name, stale)
    return copied


def merge_lib64(tree: Path) -> list[str]:
    """Move shared libraries from ``tree/lib64`` into ``tree/lib``.

    On RHEL-family hosts CMake's GNUInstallDirs installs into lib64, so
    build_usd.py's oneTBB lands in usd-install/lib64 while OpenUSD itself
    uses lib. The launcher puts only lib on the loader path, and the
    install's absolute RUNPATH to lib64 is gone once the bundle moves.
    """
    lib64 = tree / "lib64"
    if not lib64.is_dir() or lib64.is_symlink():
        return []
    lib = tree / "lib"
    lib.mkdir(parents=True, exist_ok=True)
    moved = []
    for path in sorted(lib64.iterdir()):
        if ".so" not in path.name or path.is_dir():
            continue
        target = lib / path.name
        if os.path.lexists(target):
            continue
        if path.is_symlink() and "/" not in os.readlink(path):
            os.symlink(os.readlink(path), target)
        else:
            shutil.copy2(path, target)
        moved.append(path.name)
    return moved


def link_libpython(app_dir: Path) -> list[str]:
    """Put libpython on the launcher's loader path (Linux).

    python-build-standalone's interpreter links CPython statically, so
    nothing loads libpython3.X.so.1.0 for the extension modules and
    libraries (OpenUSD's, usdRig's) that were linked against it; inside
    the build tree their RUNPATH found it. A relative link in usd/lib
    resolves it once the bundle moves.
    """
    lib = app_dir / "usd" / "lib"
    linked = []
    for path in sorted((app_dir / "python" / "lib").glob("libpython3*.so*")):
        target = lib / path.name
        if path.is_dir() or os.path.lexists(target):
            continue
        lib.mkdir(parents=True, exist_ok=True)
        os.symlink(os.path.relpath(path, lib), target)
        linked.append(path.name)
    return linked


def make_executable(path: Path) -> None:
    if os.name != "nt":
        target = path.resolve() if path.is_symlink() else path
        target.chmod(target.stat().st_mode | 0o755)


_SYSTEM_DEP_PREFIXES = ("/usr/lib/", "/System/")


# usdview does not load these optional Qt SQL drivers.
_OPTIONAL_DEP_BASENAMES = frozenset(
    {"libiodbc.2.dylib", "libpq.5.dylib", "libmimerapi.dylib"})
_MACHO_MAGICS = frozenset({
    b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe",
    b"\xfe\xed\xfa\xcf", b"\xfe\xed\xfa\xce",
    b"\xca\xfe\xba\xbe", b"\xbe\xba\xfe\xca"})


def _is_macho(path: Path) -> bool:
    try:
        with path.open("rb") as handle:
            return handle.read(4) in _MACHO_MAGICS
    except OSError:
        return False


def _otool_names(flag: str, path: Path) -> list[str]:
    proc = subprocess.run(["otool", flag, str(path)],
                          capture_output=True, text=True, check=True)
    names = []
    for line in proc.stdout.splitlines():
        if flag == "-L":
            if not line[:1].isspace():
                continue
            name = line.strip().split(" (")[0]
        else:
            stripped = line.strip()
            if not stripped or stripped.endswith(":") or "(" in stripped:
                continue
            name = stripped
        if name and name not in names:
            names.append(name)
    return names


def _is_system_dep(name: str) -> bool:
    return name.startswith(_SYSTEM_DEP_PREFIXES)


def _is_optional_dep(name: str, source: Path) -> bool:
    return (source.parent.name == "sqldrivers" and
            source.parent.parent.name == "plugins" and
            PurePosixPath(name).name in _OPTIONAL_DEP_BASENAMES)


def _dylib_index(app_dir: Path) -> dict[str, Path]:
    # Prefer libraries over executables and the framework root Python over
    # its nested Python.app host when both use the same basename.
    index: dict[str, Path] = {}
    rank: dict[str, tuple[int, int]] = {}
    for path in sorted(app_dir.rglob("*")):
        if path.is_symlink() or not path.is_file() or not _is_macho(path):
            continue
        key = (0 if path.suffix in (".dylib", ".so") else 1,
               len(path.parts))
        if path.name not in rank or key < rank[path.name]:
            rank[path.name] = key
            index[path.name] = path
    return index


def _resolve_dep(name: str, index: dict[str, Path]) -> Path | None:
    parts = PurePosixPath(name).parts
    if PurePosixPath(name).name == "Python" and "Versions" in parts:
        lib = index.get(
            f"libpython{parts[parts.index('Versions') + 1]}.dylib")
        if lib is not None:
            return lib
    return index.get(PurePosixPath(name).name)


def _loader_path_name(source: Path, target: Path) -> str:
    rel = os.path.relpath(target.parent, source.parent)
    if rel == ".":
        return f"@loader_path/{target.name}"
    return f"@loader_path/{rel}/{target.name}"


def _patch_macho(path: Path, new_id: str | None,
                 changes: list[tuple[str, str]]) -> str | None:
    args = ["install_name_tool"]
    if new_id is not None:
        args += ["-id", new_id]
    for old, new in changes:
        args += ["-change", old, new]
    args.append(str(path))
    mode = path.stat().st_mode
    try:
        if not mode & 0o200:
            path.chmod(mode | 0o200)
        proc = subprocess.run(args, capture_output=True, text=True)
    finally:
        if not mode & 0o200:
            path.chmod(mode)
    if proc.returncode != 0:
        return proc.stderr.strip() or "install_name_tool failed"
    return None


def relocate_macos_bundle(app_dir: Path) -> tuple[list[str], list[str]]:
    problems: list[str] = []
    warnings: list[str] = []
    index = _dylib_index(app_dir)
    for path in sorted(app_dir.rglob("*")):
        if path.is_symlink() or not path.is_file() or not _is_macho(path):
            continue
        rel = path.relative_to(app_dir)
        ids = _otool_names("-D", path)
        deps = _otool_names("-L", path)
        new_id = None
        changes: list[tuple[str, str]] = []
        for ident in ids:
            if not ident.startswith("/") or _is_system_dep(ident):
                continue
            target = _resolve_dep(ident, index)
            if target is not None:
                new_id = f"@rpath/{target.name}"
            elif _is_optional_dep(ident, path):
                warnings.append(f"optional dependency {ident} in {rel}")
            else:
                problems.append(
                    f"non-relocatable install name {ident} in {rel}")
        for dep in deps:
            if dep in ids or not dep.startswith("/") or _is_system_dep(dep):
                continue
            target = _resolve_dep(dep, index)
            if target is not None:
                changes.append((dep, _loader_path_name(path, target)))
            elif _is_optional_dep(dep, path):
                warnings.append(f"optional dependency {dep} in {rel}")
            else:
                problems.append(
                    f"non-relocatable dependency {dep} in {rel}")
        if new_id is not None or changes:
            error = _patch_macho(path, new_id, changes)
            if error:
                problems.append(f"cannot patch {rel}: {error}")
    return problems, warnings


def check_macos_python_stub(app_dir: Path) -> list[str]:
    python_dir = app_dir / "python"
    exe = python_executable(python_dir, "macos")
    try:
        data = exe.read_bytes()
    except OSError:
        return []
    if b"Resources/Python.app/Contents/MacOS/Python" not in data:
        return []
    targets = (python_dir / "Resources" / "Python.app" / "Contents" /
               "MacOS" / "Python",
               python_dir / "lib" / "Resources" / "Python.app" /
               "Contents" / "MacOS" / "Python")
    if any(target.is_file() for target in targets):
        return []
    return [f"bundled Python stub spawns Resources/Python.app, but no "
            f"Python.app is bundled under {python_dir}"]


def find_unrelocated_macos_deps(app_dir: Path) -> list[str]:
    problems = []
    for path in sorted(app_dir.rglob("*")):
        if path.is_symlink() or not path.is_file() or not _is_macho(path):
            continue
        rel = path.relative_to(app_dir)
        for name in _otool_names("-D", path) + _otool_names("-L", path):
            if not name.startswith("/") or _is_system_dep(name):
                continue
            if _is_optional_dep(name, path):
                continue
            problems.append(
                f"non-relocatable dependency {name} in {rel}")
    return problems


def info_plist(version: str, minimum_system: str = "13.0") -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" \
"http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>usdRigView</string>
  <key>CFBundleIdentifier</key><string>org.usdrig.usdRigView</string>
  <key>CFBundleName</key><string>usdRigView</string>
  <key>CFBundleDisplayName</key><string>usdRig Viewer</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>{version}</string>
  <key>CFBundleVersion</key><string>{version}</string>
  <key>LSMinimumSystemVersion</key><string>{minimum_system}</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
"""


def readme_text(version: str, platform: str) -> str:
    if platform == "windows":
        run = ("Double-click `usdRigView.exe`, or from a terminal:\n\n"
               "```bat\nusdRigView.exe path\\to\\stage.usda\n```")
    elif platform == "macos":
        run = ("Double-click `usdRigView.app`, or from a terminal:\n\n"
               "```sh\nopen usdRigView.app --args \"$PWD/stage.usda\"\n```\n\n"
               "An unsigned build is stopped by Gatekeeper on first "
               "launch: right-click the app, choose Open, and confirm. "
               "Signed release builds do not show this.")
    else:
        run = ("From a terminal in this directory:\n\n"
               "```sh\n./usdRigView path/to/stage.usda\n```")
    return f"""# usdRig {version} ({platform})

A self-contained usdview with the RigExec evaluation engine. No separate
OpenUSD, Python, or Qt install is needed, and nothing is installed
system-wide: extract the archive anywhere and run the launcher.

{run}

With no stage argument the viewer opens a blank stage. Every other
argument is passed to usdview unchanged.

## Checking the download

```sh
usdRigView --usdRig-check
```

prints `usdRigView bundle OK` when every bundled component is in place,
and names the missing file otherwise. Keep the launcher beside its `app`
folder (inside the `.app` on macOS); the pair only works together.

## Contents

- `app/usd` - OpenUSD install the viewer runs on
- `app/python` - the CPython those OpenUSD bindings were built for
- `app/rig` - RigExec libraries, schemas, imaging plug-ins, and the
  `rigExecPose` / `rigExecBake` command-line tools under `app/rig/bin`
- `app/plugins` - the TouchPose and Shape Editor usdview plug-ins
- `app/examples` - `rigexec_flat.usda` and `ArmShotAnim.usda` to try

Licences for usdRig and every bundled component are in the legal texts
beside this file and under `app/rig/share/rigExec`.
"""


def assemble(args) -> Path:
    output = Path(args.output).resolve()
    if output.exists() and any(output.iterdir()) and not args.force:
        raise SystemExit(f"output is not empty (pass --force): {output}")
    version = args.version.lstrip("v")
    platform = args.platform

    if platform == "macos":
        app_bundle = output / "usdRigView.app"
        contents = app_bundle / "Contents"
        launcher_dst = contents / "MacOS" / "usdRigView"
        app_dir = contents / "Resources" / "app"
    else:
        suffix = ".exe" if platform == "windows" else ""
        launcher_dst = output / f"usdRigView{suffix}"
        app_dir = output / "app"
    app_dir.mkdir(parents=True, exist_ok=True)
    launcher_dst.parent.mkdir(parents=True, exist_ok=True)

    shutil.copy2(args.launcher, launcher_dst)
    make_executable(launcher_dst)
    if platform == "macos":
        (contents / "Info.plist").write_text(
            info_plist(version, getattr(args, "macos_minimum", "13.0")),
            encoding="utf-8")
        (contents / "PkgInfo").write_text("APPL????", encoding="utf-8")

    keep_links = platform == "macos"
    copy_tree(Path(args.usd_install), app_dir / "usd", args.trim_dev,
              preserve_symlinks=keep_links)
    copy_tree(Path(args.rig_install), app_dir / "rig", args.trim_dev,
              preserve_symlinks=keep_links)
    if platform == "linux":
        for sub in ("usd", "rig"):
            for name in merge_lib64(app_dir / sub):
                print(f"assemble_bundle: {sub}/lib64/{name} -> {sub}/lib")
    python_src = resolve_python_home(Path(args.python_home), platform)
    if python_src != Path(args.python_home):
        print(f"assemble_bundle: note: python home resolves to "
              f"{python_src}; bundling that prefix instead of the "
              f"shim tree {args.python_home}", file=sys.stderr)
    copy_tree(python_src, app_dir / "python", args.trim_dev,
              preserve_symlinks=keep_links)
    make_executable(python_executable(app_dir / "python", platform))
    if platform == "linux":
        for name in link_libpython(app_dir):
            print(f"assemble_bundle: usd/lib/{name} -> python/lib/{name}")
    runtime_dir = getattr(args, "msvc_runtime_dir", "")
    if platform == "windows" and not copy_msvc_runtime(
            app_dir, Path(args.python_home), Path(args.usd_install),
            Path(args.rig_install), Path(runtime_dir) if runtime_dir else None):
        print("assemble_bundle: warning: no MSVC runtime (msvcp140.dll) "
              "found in the Python home or install trees; the bundle "
              "will rely on the host's Visual C++ runtime",
              file=sys.stderr)

    source = Path(args.source_root)
    for plugin in ("touchPose", "shapeEditor"):
        copy_tree(source / "plugin" / plugin,
                  app_dir / "plugins" / plugin, trim_dev=False)

    noodles_dst = app_dir / "rig" / "lib" / "python" / "UsdNoodles"
    if args.usdnoodles_dir and not (noodles_dst / "plugInfo.json").is_file():
        copy_tree(Path(args.usdnoodles_dir), noodles_dst, trim_dev=False)

    if args.rig_build_dir:
        bin_dir = app_dir / "rig" / "bin"
        for tool in TOOL_NAMES:
            for name in (tool, tool + ".exe"):
                candidate = Path(args.rig_build_dir) / name
                if candidate.is_file():
                    bin_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(candidate, bin_dir / name)
                    make_executable(bin_dir / name)

    examples_dir = app_dir / "examples"
    for name in EXAMPLE_NAMES:
        candidate = source / "examples" / name
        if candidate.is_file():
            examples_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(candidate, examples_dir / name)

    if (source / "icons").is_dir():
        copy_tree(source / "icons", app_dir / "icons", trim_dev=False)

    if platform == "macos":
        problems = check_macos_python_stub(app_dir)
        relocated, warnings = relocate_macos_bundle(app_dir)
        problems += relocated
        for warning in warnings:
            print(f"assemble_bundle: warning: {warning}", file=sys.stderr)
        problems += find_unrelocated_macos_deps(app_dir)
        if problems:
            details = "\n".join(f"  {problem}" for problem in problems)
            raise SystemExit(
                "assemble_bundle: bundle is not relocatable; it would "
                "only run on a machine with the build machine's "
                f"libraries installed:\n{details}")

    for name in LEGAL_NAMES:
        candidate = source / name
        if candidate.is_file():
            shutil.copy2(candidate, output / name)
    (output / "README-bundle.md").write_text(
        readme_text(version, platform), encoding="utf-8")

    manifest = {
        "bundle": "usdRig",
        "version": version,
        "platform": args.platform_slug or platform,
        "git_sha": args.git_sha or None,
        "generated_utc": datetime.datetime.now(
            datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "launcher": launcher_dst.name,
        "components": {
            "openusd": "app/usd",
            "python": "app/python",
            "rigexec": "app/rig",
            "plugins": "app/plugins",
        },
    }
    manifest_path = output / "bundle-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n",
                             encoding="utf-8")
    return output


def create_archive(output: Path, archive: Path) -> Path:
    archive = Path(archive).resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)
    if archive.suffix == ".zip":
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zf:
            for path in sorted(output.rglob("*")):
                if path.is_file():
                    zf.write(path, path.relative_to(output.parent))
    elif archive.name.endswith((".tar.gz", ".tgz")):
        with tarfile.open(archive, "w:gz") as tf:
            tf.add(output, arcname=output.name)
    else:
        raise SystemExit(f"unsupported archive type: {archive}")
    return archive


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-install", required=True)
    parser.add_argument("--rig-install", required=True)
    parser.add_argument("--python-home", required=True,
                        help="CPython home to bundle (its bin/python3 or "
                             "python.exe is copied with the tree)")
    parser.add_argument("--launcher", required=True,
                        help="built usdRigView binary for this platform")
    parser.add_argument("--output", required=True)
    parser.add_argument("--version", required=True,
                        help="release version, with or without leading v")
    parser.add_argument("--platform", choices=PLATFORMS,
                        default={"win32": "windows", "darwin": "macos"}.get(
                            sys.platform, "linux"))
    parser.add_argument("--platform-slug", default="",
                        help="manifest/archive label, e.g. windows-x64")
    parser.add_argument("--source-root", default=".",
                        help="usdRig checkout holding plugin/, examples/ "
                             "and the legal texts")
    parser.add_argument("--rig-build-dir", default="",
                        help="build tree to copy rigExecPose/rigExecBake/"
                             "rigExecReplayInputs from")
    parser.add_argument("--usdnoodles-dir", default="",
                        help="staged UsdNoodles package, when the RigExec "
                             "install does not already carry it")
    parser.add_argument("--macos-minimum", default="13.0",
                        help="LSMinimumSystemVersion written to the macOS "
                             "app's Info.plist (the build's deployment "
                             "target)")
    parser.add_argument("--msvc-runtime-dir", default="",
                        help="Windows: the build toolset's CRT redist "
                             "directory (Microsoft.VC143.CRT); preferred "
                             "over the runtime PySide6 ships")
    parser.add_argument("--git-sha", default="")
    parser.add_argument("--archive", default="",
                        help="also write this .zip / .tar.gz of the bundle")
    parser.add_argument("--trim-dev", action="store_true",
                        help="drop headers, import/static libraries and "
                             "CMake package files (smaller, runtime-only)")
    parser.add_argument("--force", action="store_true",
                        help="assemble into a non-empty output directory")
    args = parser.parse_args(argv)

    problems = validate(args)
    if problems:
        for problem in problems:
            print(f"assemble_bundle: {problem}", file=sys.stderr)
        return 2
    output = assemble(args)
    print(f"bundle: {output}")
    if args.archive:
        print(f"archive: {create_archive(output, Path(args.archive))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
