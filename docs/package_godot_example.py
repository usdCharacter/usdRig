"""Package the sibling Godot tutorial for docs; omit caches and stale binaries."""
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import argparse
import hashlib
import json

from legal_members import (archive_member_name, legal_member_status,
                          legal_members, member_bytes)

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT.parent / "godot_rigExec"
OUT = ROOT / "docs/examples/godot_rolling_ball.zip"
SOURCE = "godot_rigExec"
START_HERE = """Godot rolling-ball tutorial
Windows x86-64 binaries, Godot 4.7+. Open demo/project.godot and press F5.
The prebuilt game does not need USD.

For rebuilds, extract godot_rigExec beside usdRig and usd-install.
Setup exports the presentation (numpy must be installed in the Python with
the USD bindings), then bakes it into demo/rolling_ball.rigexec.
Close the Godot editor before running python demo/setup_rolling.py.
For native builds, first fetch godot-cpp into thirdparty/godot-cpp:
git clone https://github.com/godotengine/godot-cpp.git thirdparty/godot-cpp
git -C thirdparty/godot-cpp checkout 507ed9d840c01a3c5b2a39af8bb4000bfac30bf5
git -C thirdparty/godot-cpp submodule update --init --recursive
python demo/setup_rolling.py --build

No Godot cache, editor state, credentials, or platform binaries other than
the tested Windows x86-64 pair are included. Source is included; other
platforms need their own native builds and exporter toolchain setup.

Licenses: LICENSE, NOTICE and THIRD_PARTY_NOTICES.md at the root of this
archive (the canonical usdRig texts, copied from the source checkout); the
LICENSE-*, NOTICE-usdRig and THIRD_PARTY_NOTICES-usdRig.md copies kept for the
tutorial layout; addons/rigexec/THIRD_PARTY_NOTICES.md and its license texts;
tools/thirdparty/flatbuffers/LICENSE for the vendored FlatBuffers Python
runtime. tools/license_manifest.json records which text covers which member.
"""


def _tree(folder):
    """Every file under a checked-in plugin folder, without Python caches."""
    return [path for path in (PLUGIN / folder).rglob("*")
            if path.is_file() and "__pycache__" not in path.parts
            and path.suffix != ".pyc"]


def _member(relative):
    return archive_member_name(f"{SOURCE}/{relative}")


def main():
    global PLUGIN
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--addon-root", type=Path, default=PLUGIN,
                        help="Godot addon checkout (default: sibling godot_rigExec)")
    PLUGIN = parser.parse_args().addon_root.resolve()
    if not PLUGIN.is_dir():
        raise SystemExit(f"Godot add-on checkout not found: {PLUGIN} "
                         "(build it, or pass --addon-root)")

    files = [PLUGIN / "SConstruct"]
    files += list((PLUGIN / "addons/rigexec").glob("*.gd"))
    files += list((PLUGIN / "addons/rigexec").glob("*.cfg"))
    files += list((PLUGIN / "addons/rigexec").glob("*.gdextension"))
    files += [PLUGIN / "addons/rigexec" / "README.md"]
    files += list((PLUGIN / "addons/rigexec/src").glob("*.cpp"))
    files += list((PLUGIN / "addons/rigexec/src").glob("*.h"))
    # Windows only: bin/ can also hold other platforms' untested libraries.
    files += list((PLUGIN / "addons/rigexec/bin").glob("*.windows.*.dll"))
    files += [PLUGIN / "tools" / name for name in (
        "export_ball_assets.py", "gen_presentation.py", "ball_subdivide.cpp",
        "build_ball_subdivide.bat", "retarget_ball_star_uvs.py")]
    # The exporter's generated presentation builder and the vendored
    # FlatBuffers Python runtime it runs on.
    files += _tree("tools/generated") + _tree("tools/thirdparty")
    files += [PLUGIN / "demo" / name for name in (
        "project.godot", "rolling_ball.tscn", "rolling_ball.gd",
        "rolling_motion.gd", "rolling_game.tscn", "rolling_game.gd",
        "rolling_ball.rigexec", "verify_rolling.gd", "record_rolling.gd",
        "record_ball_material.gd", "setup_rolling.py", "play_rolling.bat",
        "README.md")]
    payload = sorted(set(files))

    legal = legal_members("godot_zip", addon_root=PLUGIN)
    members = {name: path for name, path, _ in legal}
    members.update({_member(path.relative_to(PLUGIN).as_posix()): path for path in payload})
    # The demo folder ships the add-on again so the demo opens standalone;
    # the manifest already names the legal copies it needs.
    for member, source in list(members.items()):
        relative = member[len(SOURCE) + 1:]
        if relative.startswith("addons/") and "/src/" not in relative:
            members.setdefault(_member("demo/" + relative), source)

    missing = [str(source) for source in members.values() if not source.is_file()]
    if missing:
        raise SystemExit("missing: " + ", ".join(missing))

    previous = {}
    if OUT.is_file():
        with ZipFile(OUT) as existing:
            previous = {name: existing.read(name) for name in existing.namelist()}
    status = {name: legal_member_status(previous.get(name), path)
              for name, path, _ in legal}
    for name, state in status.items():
        print(f"{name}: {state}")

    manifest = {"legal_members": [{"member": name,
                                   "source": entry.get("source"),
                                   "spdx": entry.get("spdx"),
                                   "status": status[name]}
                                  for name, _, entry in legal],
                "files": {}}
    with ZipFile(OUT, "w", ZIP_DEFLATED, compresslevel=9) as archive:
        for member, source in sorted(members.items()):
            data = member_bytes(previous.get(member), source)
            archive.writestr(member, data)
            manifest["files"][member[len(SOURCE) + 1:]] = hashlib.sha256(data).hexdigest()
        archive.writestr(f"{SOURCE}/package_manifest.json",
                         json.dumps(manifest, indent=2) + "\n")
        archive.writestr(f"{SOURCE}/START_HERE.txt", START_HERE)
    print(f"Wrote {OUT} ({OUT.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
