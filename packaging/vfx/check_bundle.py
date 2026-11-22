#!/usr/bin/env python3
"""Check an assembled bundle against a VFX Reference Platform year.

Linux (``--platform linux``): every ELF file in the bundle must resolve each
library it needs either where the relocated bundle's loader looks (the
launcher's library directories, or a RUNPATH inside the bundle) or to a
system library that every
supported distribution provides (the manylinux policy list plus the GL,
X11/xcb, Wayland, font and D-Bus libraries a desktop session supplies).
With ``--collect``, a needed library that resolves on the build machine to a
non-system location (for example a container's /usr/local) is copied into
``app/usd/lib``, which the launcher puts on ``LD_LIBRARY_PATH``. Separately,
no ELF file may require a ``GLIBC_`` or ``GLIBCXX_`` symbol version newer than
the year allows, so the bundle loads on the year's oldest supported OS.

macOS (``--platform macos``): every Mach-O file under ``app/usd`` and
``app/rig`` (the code this repository builds) must declare a minimum macOS
no newer than ``--macos-minimum``.

Unresolved dependencies inside the PySide6, shiboken6 and NumPy wheels are
reported as warnings and never collected: the wheels are manylinux builds
whose optional parts load only when the host has the library.

Stdlib only, plus readelf/ldd (binutils, glibc) on Linux and otool on macOS.
"""

from __future__ import annotations

import argparse
import fnmatch
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

# Libraries a desktop Linux system provides. The manylinux_2_28 policy list,
# plus what a graphical session supplies and Qt's wheels already expect from
# the host.
SYSTEM_LIBS = [
    "ld-linux-x86-64.so.2", "libc.so.6", "libm.so.6", "libdl.so.2",
    "libpthread.so.0", "librt.so.1", "libutil.so.1", "libresolv.so.2",
    "libanl.so.1", "libnsl.so.1", "libcrypt.so.1", "libstdc++.so.6",
    "libgcc_s.so.1", "libz.so.1", "libexpat.so.1",
    "libGL.so.1", "libEGL.so.1", "libGLX.so.0", "libOpenGL.so.0",
    "libGLdispatch.so.0", "libGLESv2.so.2", "libgbm.so.1", "libdrm.so.2",
    "libvulkan.so.1",
    "libX11.so.6", "libX11-xcb.so.1", "libXext.so.6", "libXrender.so.1",
    "libXi.so.6", "libXrandr.so.2", "libXcursor.so.1", "libXinerama.so.1",
    "libXfixes.so.3", "libXdamage.so.1", "libXcomposite.so.1",
    "libXtst.so.6", "libXau.so.6", "libXdmcp.so.6", "libICE.so.6",
    "libSM.so.6", "libxcb*.so*", "libxkbcommon*.so.0", "libxshmfence.so.1",
    "libwayland-*.so.0", "libwayland-egl.so.1", "libdecor-0.so.0",
    "libfontconfig.so.1", "libfreetype.so.6", "libdbus-1.so.3",
    "libglib-2.0.so.0", "libgobject-2.0.so.0", "libgthread-2.0.so.0",
    "libgio-2.0.so.0", "libgmodule-2.0.so.0", "libuuid.so.1",
]

# Pip wheels the bundle ships as-is. They are already built to the manylinux
# policy and load their optional parts (Qt platform themes, multimedia, NFC,
# WebEngine ...) only when the host has the library, so their unresolved
# dependencies are warnings and are never copied in: collecting them would
# drag a desktop's GTK, NSS and PulseAudio stacks into the bundle.
OPTIONAL_DIR_GLOBS = [
    "*/site-packages/PySide6/*", "*/site-packages/shiboken6/*",
    "*/site-packages/numpy/*", "*/site-packages/numpy.libs/*",
    "*/python*/lib-dynload/*",
]

ELF_MAGIC = b"\x7fELF"


def is_system(name: str) -> bool:
    return any(fnmatch.fnmatch(name, pattern) for pattern in SYSTEM_LIBS)


def is_optional(path: Path, root: Path) -> bool:
    rel = "/" + path.relative_to(root).as_posix()
    return any(fnmatch.fnmatch(rel, pattern) for pattern in OPTIONAL_DIR_GLOBS)


def version_tuple(text: str) -> tuple[int, ...]:
    return tuple(int(part) for part in text.split("."))


def elf_files(root: Path):
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            path = Path(dirpath) / name
            if path.is_symlink() or not path.is_file():
                continue
            try:
                with path.open("rb") as f:
                    if f.read(4) == ELF_MAGIC:
                        yield path
            except OSError:
                continue


def readelf(flag: str, path: Path) -> str:
    return subprocess.run(["readelf", flag, "-W", str(path)],
                          capture_output=True, text=True,
                          check=False).stdout


def runpath(path: Path) -> list[Path]:
    """RUNPATH/RPATH entries, with $ORIGIN expanded."""
    found = re.findall(r"\((?:RUNPATH|RPATH)\)\s+Library r(?:un)?path: "
                       r"\[([^\]]*)\]", readelf("-d", path))
    dirs = []
    for entry in ":".join(found).split(":"):
        if entry:
            dirs.append(Path(entry.replace("${ORIGIN}", str(path.parent))
                             .replace("$ORIGIN", str(path.parent))))
    return dirs


def needed(path: Path) -> list[str]:
    return re.findall(r"\(NEEDED\)\s+Shared library: \[([^\]]+)\]",
                      readelf("-d", path))


def required_versions(path: Path) -> set[str]:
    """Symbol versions the file requires from other objects."""
    text = readelf("-V", path)
    section = text.split("Version needs section", 1)
    if len(section) < 2:
        return set()
    return set(re.findall(r"Name: ((?:GLIBC|GLIBCXX|CXXABI)_[0-9.]+)",
                          section[1]))


def newest(versions: set[str], prefix: str) -> str | None:
    found = [v[len(prefix):] for v in versions if v.startswith(prefix)
             and re.fullmatch(r"[0-9.]+", v[len(prefix):])]
    return max(found, key=version_tuple) if found else None


def resolve_system(name: str, search: list[Path]) -> Path | None:
    """Where the build machine's loader would find ``name``."""
    for directory in search:
        candidate = directory / name
        if candidate.exists():
            return candidate
    out = subprocess.run(["ldconfig", "-p"], capture_output=True, text=True,
                         check=False).stdout
    match = re.search(rf"^\s+{re.escape(name)} \([^)]*x86-64[^)]*\) => (\S+)$",
                      out, flags=re.MULTILINE)
    return Path(match.group(1)) if match else None


def check_linux(root: Path, glibc_max: str, glibcxx_max: str,
                collect: bool) -> int:
    app = root / "app"
    collect_dir = app / "usd" / "lib"
    # Where the relocated bundle's loader looks: the launcher's
    # LD_LIBRARY_PATH. A library that only sits elsewhere in the bundle, or
    # is found through an absolute RUNPATH into the build tree, does not
    # count (a statically linked interpreter never loads libpython for the
    # modules that need it).
    loader_dirs = [app / "rig" / "lib", app / "rig" / "bin",
                   app / "usd" / "lib"]
    host_dirs = [Path(p) for p in os.environ.get(
        "LD_LIBRARY_PATH", "").split(":") if p]
    host_dirs += [Path("/usr/local/lib64"), Path("/usr/local/lib")]
    errors: list[str] = []
    warnings: list[str] = []
    collected: list[str] = []

    changed = True
    while changed:  # a collected library may need further libraries
        changed = False
        files = list(elf_files(root))
        for path in files:
            optional = is_optional(path, root)
            search = loader_dirs + [d for d in runpath(path)
                                    if d.resolve().is_relative_to(root)]
            for name in needed(path):
                if "/" in name:  # a path, usually $ORIGIN-relative
                    target = Path(name.replace("$ORIGIN", str(path.parent))
                                  .replace("${ORIGIN}", str(path.parent)))
                    if target.exists():
                        continue
                elif is_system(name) or any((d / name).exists()
                                            for d in search):
                    continue
                source = (None if optional or "/" in name
                          else resolve_system(name, host_dirs))
                if collect and source is not None:
                    collect_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source.resolve(), collect_dir / name)
                    collected.append(f"{name} (from {source})")
                    changed = True
                    continue
                message = (f"{path.relative_to(root)} needs {name}, which is "
                           "neither in the bundle nor a system library")
                if optional:
                    warnings.append(message)
                else:
                    errors.append(message)
        if changed:
            errors.clear()
            warnings.clear()

    for path in elf_files(root):
        versions = required_versions(path)
        for prefix, limit in (("GLIBC_", glibc_max),
                              ("GLIBCXX_", glibcxx_max)):
            top = newest(versions, prefix)
            if top and version_tuple(top) > version_tuple(limit):
                errors.append(f"{path.relative_to(root)} requires "
                              f"{prefix}{top} (limit {prefix}{limit})")

    for line in collected:
        print(f"check_bundle: collected {line}")
    for line in sorted(set(warnings)):
        print(f"check_bundle: warning: {line}")
    for line in sorted(set(errors)):
        print(f"check_bundle: error: {line}", file=sys.stderr)
    print(f"check_bundle: linux: {len(set(errors))} error(s), "
          f"{len(set(warnings))} warning(s), {len(collected)} collected; "
          f"limits GLIBC_{glibc_max}, GLIBCXX_{glibcxx_max}")
    return 1 if errors else 0


def macho_minimum(path: Path) -> str | None:
    out = subprocess.run(["otool", "-l", str(path)], capture_output=True,
                         text=True, check=False).stdout
    match = re.search(r"\bminos ([0-9.]+)", out)
    if match:
        return match.group(1)
    match = re.search(r"LC_VERSION_MIN_MACOSX.*?\n\s+version ([0-9.]+)",
                      out, flags=re.DOTALL)
    return match.group(1) if match else None


def check_macos(root: Path, minimum: str) -> int:
    app = next(root.glob("*.app/Contents/Resources/app"), root / "app")
    errors = []
    checked = 0
    for sub in ("usd", "rig"):
        for dirpath, _, filenames in os.walk(app / sub):
            for name in filenames:
                path = Path(dirpath) / name
                if path.is_symlink():
                    continue
                with path.open("rb") as f:
                    magic = f.read(4)
                if magic not in (b"\xcf\xfa\xed\xfe", b"\xca\xfe\xba\xbe"):
                    continue
                checked += 1
                found = macho_minimum(path)
                if found and version_tuple(found) > version_tuple(minimum):
                    errors.append(f"{path.relative_to(root)}: minimum macOS "
                                  f"{found} (limit {minimum})")
    for line in errors:
        print(f"check_bundle: error: {line}", file=sys.stderr)
    print(f"check_bundle: macos: {checked} Mach-O files, {len(errors)} "
          f"newer than {minimum}")
    return 1 if errors or not checked else 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bundle")
    parser.add_argument("--platform", choices=("linux", "macos"),
                        required=True)
    parser.add_argument("--glibc-max", default="2.28")
    parser.add_argument("--glibcxx-max", default="3.4.25")
    parser.add_argument("--macos-minimum", default="14.0")
    parser.add_argument("--collect", action="store_true",
                        help="copy non-system dependencies into the bundle")
    args = parser.parse_args(argv)
    root = Path(args.bundle).resolve()
    if args.platform == "linux":
        return check_linux(root, args.glibc_max, args.glibcxx_max,
                           args.collect)
    return check_macos(root, args.macos_minimum)


if __name__ == "__main__":
    raise SystemExit(main())
