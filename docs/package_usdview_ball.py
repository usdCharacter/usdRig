"""Package the editable rolling-ball USD with its texture and every referenced icon.

Run as `python docs/package_usdview_ball.py` from any directory: it rewrites
docs/examples/usdview_rolling_ball.zip in place, taking its legal members from
tools/license_manifest.json.
"""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import hashlib
import json
import re

from legal_members import (legal_member_status, legal_members, member_bytes,
                          missing_sources)

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ARCHIVE = DOCS / "examples/usdview_rolling_ball.zip"
files = {DOCS / "examples/tutorial_rolling_ball.usda",
         DOCS / "examples/tutorial_rolling_ball_free.usda"}
for stage in list(files):
    for relative in re.findall(r"@([^@]+)@", stage.read_text()):
        dependency = (stage.parent / relative).resolve()
        assert dependency.is_relative_to(ROOT) and dependency.is_file(), dependency
        files.add(dependency)

previous = {}
if ARCHIVE.is_file():
    with ZipFile(ARCHIVE) as archive:
        previous = {name: archive.read(name) for name in archive.namelist()}

legal = legal_members("usdview_zip")
missing = missing_sources(legal)
if missing:
    raise SystemExit("missing legal source: " + ", ".join(str(path) for _, path in missing))
status = {name: legal_member_status(previous.get(name), path)
          for name, path, _ in legal}
for name, state in status.items():
    print(f"{name}: {state}")

members = [(name, member_bytes(previous.get(name), path)) for name, path, _ in legal]
payload = []
for path in sorted(files):
    name = path.relative_to(ROOT).as_posix()
    payload.append((name, member_bytes(previous.get(name), path)))

manifest = {
    "legal_members": [{"member": name,
                       "source": entry.get("source"),
                       "spdx": entry.get("spdx"),
                       "status": status[name]}
                      for name, _, entry in legal],
    "files": {name: hashlib.sha256(data).hexdigest() for name, data in payload},
}
readme = """USD rolling ball: orange basketball with black seams

Extract the entire archive. Keep docs/ and icons/ beside one another.
Open docs/examples/tutorial_rolling_ball.usda in a usdRig-enabled usdview.
Scrub 1001-1049 for travel-driven rolling.
Open docs/examples/tutorial_rolling_ball_free.usda for the Godot bake source.

The basketball image texture in docs/examples/textures is shared by both
USD variants through face-varying UVs and a UsdPreviewSurface material.
The stage's rig, controllers and material remain editable.

Legal texts in this archive: {legal}.
The links inside THIRD_PARTY_NOTICES.md use the same source-relative paths as
the repository and the installed usdRig prefix, so they resolve here.
docs/references.md and docs/concepts/riglogic.md are documentation prose, not
legal texts, and ship only in the source checkout.

Live rig evaluation requires the usdRig schema and imaging plugins.
""".format(legal=", ".join(member for member, _, _ in legal))

with ZipFile(ARCHIVE, mode="w", compresslevel=ZIP_DEFLATED) as archive:
    for member, data in members:
        archive.writestr(member, data)
    for member, data in payload:
        archive.writestr(member, data)
    archive.writestr("manifest.json", json.dumps(manifest, indent=2) + "\n")
    archive.writestr("README.txt", readme)

site_docs = DOCS / "site"
site_docs.mkdir(exist_ok=True)
for member, data in payload:
    if not member.startswith("docs/"):
        continue
    target = site_docs / member[len("docs/"):]  # docs/site mirrors docs/
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
(site_docs / "examples").mkdir(exist_ok=True)
(site_docs / "examples" / ARCHIVE.name).write_bytes(ARCHIVE.read_bytes())
print(f"Packed {len(members) + len(payload) + 2} members into {ARCHIVE.name}")
