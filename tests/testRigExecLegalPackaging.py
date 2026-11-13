#!/usr/bin/env python3
"""Stdlib tests for the legal-packaging layer: manifest member names, source
binding, byte preservation and the checker's rules.

Run as `python tests/testRigExecLegalPackaging.py`, or through
`ctest -R testRigExecLegalPackaging`. Nothing here imports OpenUSD or touches
the rig; the layer under test is the license manifest, docs/legal_members.py
and tools/check_licenses.py.
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "docs"))
sys.path.insert(0, str(ROOT / "tools"))

from legal_members import (archive_member_name, legal_member_status,  # noqa: E402
                           legal_members, member_bytes, missing_sources,
                           normalize_line_endings, resolve_legal_source)
import check_licenses  # noqa: E402

MANIFEST = json.loads((ROOT / "tools" / "license_manifest.json").read_text(encoding="utf-8"))
ADDON_NAME = "godot_rigExec"


class MemberNameRules(unittest.TestCase):
    def test_traversal_and_absolute_names_are_refused(self):
        for name in ("../LICENSE", "/LICENSE", "LICENSE\\x", "a/../b", "a//b", "", "..",
                     "C:\\LICENSE", "addons/../LICENSE"):
            with self.assertRaises(ValueError, msg=name):
                archive_member_name(name)

    def test_source_relative_names_are_accepted(self):
        for name in ("LICENSE", "thirdparty/lzma/DOC/lzma-sdk.txt",
                     "godot_rigExec/addons/rigexec/LICENSE",
                     "tools/license_manifest.json"):
            self.assertEqual(archive_member_name(name), name)


class SourceBinding(unittest.TestCase):
    def test_repo_relative_source_resolves_inside_the_checkout(self):
        path = resolve_legal_source("LICENSE", "LICENSE")
        self.assertEqual(path, (ROOT / "LICENSE").resolve())

    def test_addon_source_needs_the_explicit_root(self):
        source = f"../{ADDON_NAME}/LICENSE"
        with self.assertRaises(ValueError):
            resolve_legal_source("godot_rigExec/LICENSE", source)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / ADDON_NAME
            root.mkdir()
            (root / "LICENSE").write_text("MIT", encoding="utf-8")
            self.assertEqual(resolve_legal_source("godot_rigExec/LICENSE", source, root),
                             (root / "LICENSE").resolve())

    def test_addon_source_cannot_leave_the_root_or_name_another_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / ADDON_NAME
            root.mkdir()
            for source in (f"../{ADDON_NAME}/../elsewhere/LICENSE",
                           "../other_checkout/LICENSE",
                           f"../{ADDON_NAME}"):
                with self.assertRaises(ValueError, msg=source):
                    resolve_legal_source("member", source, root)

    def test_prose_and_url_sources_refuse_to_pack_bytes(self):
        for source in ("the riglogic port notes", "https://openusd.org/license", ""):
            with self.assertRaises(ValueError, msg=source):
                resolve_legal_source("member", source)

    def test_missing_sources_are_reported_not_invented(self):
        members = legal_members("usdview_zip")
        self.assertTrue(members)
        self.assertEqual(missing_sources(members), [])
        self.assertEqual(missing_sources([("ghost", ROOT / "no" / "such" / "text", {})]),
                         [("ghost", ROOT / "no" / "such" / "text")])


class BytePreservation(unittest.TestCase):
    def test_line_ending_only_difference_keeps_the_archived_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "LICENSE"
            source.write_bytes(b"MIT\nlicense text\n")
            archived = b"MIT\r\nlicense text\r\n"
            self.assertEqual(member_bytes(archived, source), archived)
            self.assertEqual(member_bytes(None, source), b"MIT\nlicense text\n")
            self.assertEqual(member_bytes(b"MIT\r\nother\r\n", source), b"MIT\nlicense text\n")
            self.assertEqual(legal_member_status(archived, source), "line endings differ")
            self.assertEqual(legal_member_status(source.read_bytes(), source), "current")
            self.assertEqual(legal_member_status(None, source), "new member")
            self.assertEqual(legal_member_status(b"stale\n", source),
                            "refreshed (archived copy was stale)")
        self.assertEqual(normalize_line_endings(b"a\r\nb\rc"), b"a\nb\nc")


class ManifestContract(unittest.TestCase):
    def test_every_member_name_is_safe(self):
        for section, entries in MANIFEST["distribution"].items():
            for entry in entries.get("legal_files", []) + entries.get("legal_members", []):
                name = entry.get("member") or entry.get("destination")
                archive_member_name(name)

    def test_install_map_names_the_canonical_root_texts(self):
        destinations = {entry["destination"]
                        for entry in MANIFEST["distribution"]["cmake_install"]["legal_files"]}
        for canonical in ("share/rigExec/LICENSE", "share/rigExec/NOTICE",
                         "share/rigExec/THIRD_PARTY_NOTICES.md",
                         "share/rigExec/tools/license_manifest.json"):
            self.assertIn(canonical, destinations)

    def test_prose_references_are_never_shipped_as_legal_texts(self):
        paths = [ref["path"] for ref in MANIFEST["prose_references"]]
        self.assertEqual(paths, ["docs/references.md", "docs/concepts/riglogic.md",
                                 "tools/check_licenses.py"])
        for content in MANIFEST["distribution"].values():
            for entry in content.get("legal_members", []):
                self.assertNotIn(entry["member"], paths)
                self.assertFalse(any(entry["member"].endswith(p) for p in paths))
        # The one prose reference that travels is the verifier, and it ships as
        # a tool under share/rigExec/tools, never as a legal text in the legal
        # root that the notice links resolve against.
        for entry in MANIFEST["distribution"]["cmake_install"]["legal_files"]:
            if any(entry["destination"].endswith(p) for p in paths):
                self.assertTrue(entry["destination"].startswith("share/rigExec/tools/"),
                                f"{entry['destination']} ships a prose reference "
                                "as a legal text")

    def test_the_manifest_is_never_hashed_against_itself(self):
        inventories = MANIFEST["inventory_boundary"]["archive_members_inventory"]
        for record in inventories:
            self.assertNotEqual(record["archive"], "tools/license_manifest.json")
        usdview = next(r for r in inventories
                       if r["archive"] == "docs/examples/usdview_rolling_ball.zip")
        # The archive holds the manifest, so a blob of it cannot live inside
        # the manifest; the payload digest is the stable record.
        self.assertNotIn("blob", usdview)
        self.assertIn("payload_digest", usdview)
        self.assertIn("tools/license_manifest.json",
                      [e["member"] for e in MANIFEST["distribution"]["usdview_zip"]["legal_members"]])

    def test_distribution_links_and_sources_resolve(self):
        errors, _ = check_licenses.validate(MANIFEST, ROOT, None, partition=False)
        self.assertEqual(errors, [])

    def test_checker_rejects_a_bad_member_and_a_bad_required_text(self):
        broken = json.loads(json.dumps(MANIFEST))
        broken["distribution"]["usdview_zip"]["legal_members"].append(
            {"member": "../LICENSE", "source": "LICENSE", "status": "current"})
        broken["families"][0]["required_texts"].append({"path": "LICENSE", "prose": "notes"})
        errors = check_licenses.check_path_forms(broken)
        self.assertTrue(any("../LICENSE" in error for error in errors))
        self.assertTrue(any("required text entry" in error for error in errors))

    def test_checker_notices_a_section_that_lost_its_canonical_name(self):
        broken = json.loads(json.dumps(MANIFEST))
        broken["distribution"]["usdview_zip"]["legal_members"] = [
            entry for entry in broken["distribution"]["usdview_zip"]["legal_members"]
            if entry["member"] != "NOTICE"]
        errors = check_licenses.check_canonical_names(broken)
        self.assertTrue(any("canonical root name NOTICE" in error for error in errors))

    def test_install_path_math_applies_rename_and_destination(self):
        cmake = ('install(FILES LICENSE NOTICE DESTINATION "share/rigExec")\n'
                 'install(FILES thirdparty/flatbuffers/LICENSE '
                 'DESTINATION "share/rigExec/licenses" RENAME FlatBuffers-LICENSE.txt)\n')
        paths = check_licenses.install_paths(cmake)
        self.assertEqual(check_licenses.installed_path("LICENSE", *paths["LICENSE"][0]),
                         "share/rigExec/LICENSE")
        source = "thirdparty/flatbuffers/LICENSE"
        self.assertEqual(check_licenses.installed_path(source, *paths[source][0]),
                         "share/rigExec/licenses/FlatBuffers-LICENSE.txt")

    def test_archives_hold_the_mapped_members_and_bytes(self):
        errors, notes = check_licenses.check_archives(MANIFEST, ROOT, None)
        self.assertEqual(errors, [])
        self.assertTrue(notes)  # add-on-bound texts are recorded but not packed

    def test_a_shipped_mirror_of_a_legal_text_is_a_note_not_an_error(self):
        archive = ROOT / "docs/examples/godot_rolling_ball.zip"
        errors, stats, notes = check_licenses.check_archive(MANIFEST, archive, ROOT,
                                                           None)
        self.assertEqual(errors, [])
        with zipfile.ZipFile(archive) as handle:
            members = handle.namelist()
            texts = {name: handle.read(name) for name in members}
        mapped = {entry["member"] for entry in
                  MANIFEST["distribution"]["godot_zip"]["legal_members"]
                  if entry["member"] in members}
        mirrors = [name for name in members
                   if name not in mapped and check_licenses.mirror_of(name, mapped)]
        self.assertTrue(mirrors)
        self.assertEqual(stats["mirrored_legal"], len(mirrors))
        for name in mirrors:
            original = check_licenses.mirror_of(name, mapped)
            self.assertEqual(check_licenses.normalize_line_endings(texts[name]),
                            check_licenses.normalize_line_endings(texts[original]))
            self.assertTrue(any(name in note for note in notes if "mirror" in note))

    def test_a_differing_copy_of_a_legal_text_is_an_error(self):
        with zipfile.ZipFile(ROOT / "docs/examples/godot_rolling_ball.zip") as handle:
            members = handle.namelist()
            texts = {name: handle.read(name) for name in members}
        name = next(n for n in members
                    if n.endswith("THIRD_PARTY_NOTICES.md") and "demo" in n)
        broken = json.loads(json.dumps(MANIFEST))
        broken["distribution"]["godot_zip"]["legal_members"].append(
            {"member": name, "source": "THIRD_PARTY_NOTICES.md", "status": "current"})
        temp = Path(tempfile.mktemp(suffix="demo.zip"))
        with zipfile.ZipFile(temp, "w") as handle:
            handle.writestr(name, texts[name] + b"\nand a line the source lacks\n")
        errors = check_licenses.compare_legal_members(broken, temp.name, [{
            "member": name, "source": "THIRD_PARTY_NOTICES.md", "status": "current"}],
            [name], temp, None, ROOT)
        self.assertTrue(any("differs from its source" in error for error in errors))


class Partition(unittest.TestCase):
    def test_families_partition_the_tree_exactly_once(self):
        files = check_licenses.tracked_files(ROOT)
        self.assertIsNotNone(files, "the test tree must be a git checkout")
        errors, unverifiable = check_licenses.check_partition(MANIFEST, ROOT, files)
        self.assertEqual(errors, [])
        self.assertEqual(unverifiable, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
