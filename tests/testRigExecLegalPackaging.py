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
import re
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "docs"))
sys.path.insert(0, str(ROOT / "tools"))

from legal_members import (archive_member_name, legal_member_status,  # noqa: E402
                           legal_members, member_bytes, missing_sources,
                           normalize_line_endings, resolve_legal_source)
import check_licenses  # noqa: E402

MANIFEST = json.loads((ROOT / "tools" / "license_manifest.json").read_text(encoding="utf-8"))
ADDON_NAME = "godot_rigExec"


def _link(link: Path, target: Path, *, directory: bool = False) -> None:
    """Create a link, or skip the test: an unprivileged Windows runner has no
    symlink privilege, so that host leaves the rule unverified."""
    try:
        link.symlink_to(target, target_is_directory=directory)
    except (OSError, NotImplementedError):
        raise unittest.SkipTest("this host cannot create links")


def _guarded_checkout(base: Path) -> Path:
    """A checkout with a file, a link that leaves it, and a link that stays."""
    (base / "licenses").mkdir(parents=True)
    (base / "licenses" / "LICENSE").write_text("MIT\n", encoding="utf-8")
    elsewhere = base.parent / "elsewhere"
    elsewhere.mkdir()
    (elsewhere / "LICENSE").write_text("not ours\n", encoding="utf-8")
    _link(base / "ESCAPE", elsewhere / "LICENSE")
    _link(base / "out", elsewhere, directory=True)
    _link(base / "licenses" / "INNER", base / "licenses" / "LICENSE")
    return base


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


class Containment(unittest.TestCase):
    """Containment is checked after resolving both the authorized root and the
    joined source, so a link cannot carry a packed file outside the tree while
    a root the platform spells through a link stays authorized."""

    def test_a_link_may_not_carry_a_repository_source_outside_the_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = _guarded_checkout(Path(tmp) / "checkout")
            for source in ("ESCAPE", "out/LICENSE"):
                with self.assertRaises(ValueError, msg=source):
                    resolve_legal_source("member", source, None, base)
            self.assertEqual(resolve_legal_source("member", "licenses/LICENSE", None, base),
                             (base / "licenses" / "LICENSE").resolve())

    def test_a_linked_root_and_an_internal_link_are_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = _guarded_checkout(Path(tmp) / "checkout")
            linked_root = Path(tmp) / "linked-checkout"
            _link(linked_root, base, directory=True)
            for root in (base, linked_root):
                self.assertEqual(resolve_legal_source("member", "licenses/INNER", None, root),
                                 (base / "licenses" / "LICENSE").resolve())
                self.assertEqual(resolve_legal_source("member", "licenses/LICENSE", None, root),
                                 (base / "licenses" / "LICENSE").resolve())
                with self.assertRaises(ValueError, msg=str(root)):
                    resolve_legal_source("member", "out/LICENSE", None, root)

    def test_a_link_may_not_carry_an_addon_source_outside_the_addon_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = _guarded_checkout(Path(tmp) / ADDON_NAME)
            _link(root / "licenses" / "ESCAPED", root.parent / "elsewhere" / "LICENSE")
            for source in (f"../{ADDON_NAME}/ESCAPE", f"../{ADDON_NAME}/out/LICENSE",
                           f"../{ADDON_NAME}/licenses/ESCAPED"):
                with self.assertRaises(ValueError, msg=source):
                    resolve_legal_source("member", source, root)
            self.assertEqual(
                resolve_legal_source("member", f"../{ADDON_NAME}/licenses/INNER", root),
                (root / "licenses" / "LICENSE").resolve())

    def test_a_linked_addon_root_keeps_the_name_it_was_supplied_with(self):
        with tempfile.TemporaryDirectory() as tmp:
            real = Path(tmp) / "actual-addon-checkout"
            real.mkdir()
            (real / "LICENSE").write_text("MIT\n", encoding="utf-8")
            link = Path(tmp) / ADDON_NAME
            _link(link, real, directory=True)
            self.assertEqual(resolve_legal_source("member", f"../{ADDON_NAME}/LICENSE", link),
                             (real / "LICENSE").resolve())


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

    def test_every_notice_link_maps_to_a_shipped_legal_text(self):
        prose = {ref["path"] for ref in MANIFEST["prose_references"]}
        godot = {entry["member"] for entry in
                 MANIFEST["distribution"]["godot_zip"]["legal_members"]}
        install = {entry["destination"] for entry in
                   MANIFEST["distribution"]["cmake_install"]["legal_files"]}
        notice = (ROOT / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
        links = [target.partition("#")[0]
                 for target in re.findall(r"\]\(([^)]+)\)", notice)
                 if not target.startswith(("http://", "https://"))]
        self.assertTrue(links)
        self.assertEqual([target for target in links
                          if target not in prose and f"godot_rigExec/{target}" not in godot],
                         [], "a notice link with no mapped member in the godot archive")
        for target in links:
            if target not in prose:
                self.assertIn(f"share/rigExec/{target}", install, msg=target)

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

    def test_a_corrupted_alias_copy_in_a_shipped_archive_is_an_error(self):
        """A mapped member is compared whatever its status prose records: the
        usdview archive's historical aliases name repository sources."""
        archive = ROOT / "docs/examples/usdview_rolling_ball.zip"
        with zipfile.ZipFile(archive) as handle:
            texts = {name: handle.read(name) for name in handle.namelist()}
        aliases = [entry["member"] for entry
                   in MANIFEST["distribution"]["usdview_zip"]["legal_members"]
                   if "alias" in entry.get("status", "").lower()]
        self.assertTrue(aliases, "the usdview archive records historical aliases")
        for member in aliases:
            with tempfile.TemporaryDirectory() as tmp:
                temp = Path(tmp) / archive.name
                with zipfile.ZipFile(temp, "w") as handle:
                    for name, data in texts.items():
                        handle.writestr(name, b"CORRUPTED LICENSE TEXT\n"
                                        if name == member else data)
                errors, _stats, _notes = check_licenses.check_archive(MANIFEST, temp,
                                                                      ROOT, None)
                self.assertIn(member, " ".join(errors),
                              f"{member} escaped the byte comparison")

    def test_a_recorded_stale_member_is_a_recorded_difference(self):
        """The stale members of a shipped archive are checked to still differ
        from their sources, so the exception stays a record instead of hiding
        drift or turning a refreshed archive into a false error."""
        stale = [entry for entry
                 in MANIFEST["distribution"]["godot_zip"]["legal_members"]
                 if "stale" in entry.get("status", "").lower()]
        self.assertTrue(stale, "the godot archive records stale members")
        with zipfile.ZipFile(ROOT / "docs/examples/godot_rolling_ball.zip") as handle:
            texts = {name: handle.read(name) for name in handle.namelist()}
        for entry in stale:
            source = resolve_legal_source(entry["member"], entry["source"], None, ROOT)
            self.assertTrue(source.is_file(), entry["source"])
            self.assertNotEqual(
                check_licenses.normalize_line_endings(texts[entry["member"]]),
                check_licenses.normalize_line_endings(source.read_bytes()),
                f"{entry['member']} matches its source: refresh the record")

    def test_the_recorded_legal_counts_of_a_shipped_archive_are_enforced(self):
        """Production inventory records use a repository-relative archive path,
        so both recorded legal counts must be checked against that form."""
        for field in ("legal_members", "legal_members_mapped"):
            broken = json.loads(json.dumps(MANIFEST))
            record = next(r for r in
                          broken["inventory_boundary"]["archive_members_inventory"]
                          if r["archive"] == "docs/examples/usdview_rolling_ball.zip")
            self.assertNotEqual(record[field], 999)
            record[field] = 999
            errors, _notes = check_licenses.check_archives(broken, ROOT, None)
            self.assertTrue(any("records 999" in error for error in errors),
                            f"{field} is not checked for a repository-relative record")


class Partition(unittest.TestCase):
    def test_families_partition_the_tree_exactly_once(self):
        files = check_licenses.tracked_files(ROOT)
        self.assertIsNotNone(files, "the test tree must be a git checkout")
        errors, unverifiable = check_licenses.check_partition(MANIFEST, ROOT, files)
        self.assertEqual(errors, [])
        self.assertEqual(unverifiable, [])

    def test_the_project_art_families_record_an_mit_grant(self):
        """The owner answers for the project's art (PR #49, 2026-11-14): the
        asset families are original MIT, and nothing tracked is left without a
        grant. Only the Squarebit Eye shader, whose EULA is not ours to
        relicense, stays outside MIT in the asset trees."""
        files = check_licenses.tracked_files(ROOT)
        self.assertIsNotNone(files, "the test tree must be a git checkout")
        legal_holder = next(family["holder"] for family in MANIFEST["families"]
                            if family["id"] == "original-legal-records")
        for family_id in ("original-icon-atlas", "original-usdview-icons",
                          "original-component-library", "original-example-assets",
                          "original-doc-assets", "original-test-fixtures"):
            record = next((family for family in MANIFEST["families"]
                           if family["id"] == family_id), None)
            self.assertIsNotNone(record, f"{family_id} is missing")
            self.assertGreater(record["matched_files"], 0, f"{family_id} is empty")
            self.assertEqual(record["category"], "original", family_id)
            self.assertEqual(record["spdx"], "MIT", family_id)
            self.assertEqual(record["holder"], legal_holder, family_id)
            self.assertTrue(any("owner statement" in line
                                for line in record["provenance"]),
                            f"{family_id} records no owner answer")
        art_paths = ("icons/", "plugin/rigExecUsdview/icons/", "asset_lib/",
                     "examples/", "docs/examples/", "docs/images/",
                     "docs/gifs/", "docs/assets/", "tests/fixtures/")
        for name in files:
            if not name.startswith(art_paths):
                continue
            family_id = check_licenses.family_of(name, MANIFEST["families"])
            if family_id == "squarebit-eye-shader":
                continue
            record = next(family for family in MANIFEST["families"]
                          if family["id"] == family_id)
            self.assertEqual(record["category"], "original", f"{name} in {family_id}")
            self.assertEqual(record["spdx"], "MIT", f"{name} in {family_id}")
            self.assertEqual(record["holder"], legal_holder, f"{name} in {family_id}")

    def test_no_tracked_file_is_left_without_a_grant(self):
        """The unresolved category survives only as the precedence-6 fallback,
        so a new file that no family claims is a failure, not an asset with no
        grant."""
        self.assertEqual([family["id"] for family in MANIFEST["families"]
                          if family["category"] == "unresolved"],
                         ["fallback-unclassified"])
        self.assertEqual(MANIFEST["totals"]["by_category"]["unresolved"], 0)
        files = check_licenses.tracked_files(ROOT)
        self.assertIsNotNone(files, "the test tree must be a git checkout")
        falling = [name for name in files
                   if check_licenses.family_of(name, MANIFEST["families"])
                   == "fallback-unclassified"]
        self.assertEqual(falling, [])

    def test_a_dependency_staged_in_the_checkout_stays_outside_the_audit(self):
        """CI builds OpenUSD inside the checkout and downloads its build
        archives into the checkout root; neither is part of the audited tree,
        while a file staged with git add is.

        The staged layout is built here rather than read from this checkout, so
        the boundary is proven with dependency output actually present instead
        of depending on how the host happens to have arranged its own tree.
        """
        audited, extras = check_licenses.inventory_parts(
            ["LICENSE", "libs/rigExec/evaluator.cpp", "docs/home.md"],
            ["usd-install.tar", "usd-install/lib/libusd.so",
             "OpenUSD/pxr/base/tf/tf.h", "build/CMakeCache.txt",
             "libs/rigExec/newKernel.h"])
        self.assertEqual(audited, ["LICENSE", "docs/home.md",
                                   "libs/rigExec/evaluator.cpp"])
        self.assertEqual(extras, ["libs/rigExec/newKernel.h", "usd-install.tar"])
        dependency_output = ("usd-install/lib/libusd.so",
                             "OpenUSD/pxr/base/tf/tf.h",
                             "build/CMakeCache.txt")
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "repo"
            for name in ("LICENSE", "docs/home.md", "libs/rigExec/evaluator.cpp"):
                (repo / name).parent.mkdir(parents=True, exist_ok=True)
                (repo / name).write_text("project\n", encoding="utf-8")
            subprocess.run(["git", "init", "-q"], cwd=repo, check=True,
                           capture_output=True)
            # A personal ignore file on the host must not decide which side of
            # the boundary the fixture's dependency output lands on.
            (repo / ".git" / "no-excludes").write_text("", encoding="utf-8")
            subprocess.run(["git", "config", "core.excludesFile", ".git/no-excludes"],
                           cwd=repo, check=True, capture_output=True)
            subprocess.run(["git", "add", "-A"], cwd=repo, check=True,
                           capture_output=True)
            for name in (*dependency_output, "usd-install.tar"):
                (repo / name).parent.mkdir(parents=True, exist_ok=True)
                (repo / name).write_text("staged dependency\n", encoding="utf-8")
            new_header = repo / "libs" / "rigExec" / "newKernel.h"
            new_header.write_text("new\n", encoding="utf-8")
            files, reported = check_licenses.inventory(repo)
            self.assertIsNotNone(files, "the fixture must be a git checkout")
            self.assertEqual(files, ["LICENSE", "docs/home.md",
                                     "libs/rigExec/evaluator.cpp"])
            self.assertEqual(reported, ["libs/rigExec/newKernel.h",
                                        "usd-install.tar"])
            for name in files:
                self.assertFalse(name.startswith(("usd-install", "OpenUSD/",
                                                  "build/")), name)
            # The guarantee the boundary keeps: once a new file is staged it is
            # audited, so it cannot reach a commit without a family.
            subprocess.run(["git", "add", "libs/rigExec/newKernel.h"], cwd=repo,
                           check=True, capture_output=True)
            staged, _ = check_licenses.inventory(repo)
            self.assertIn("libs/rigExec/newKernel.h", staged)


def _legal_root_installs() -> set[tuple[str, str]]:
    """(source, destination) for every CMake install into the legal root."""
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    landed = set()
    for source, targets in check_licenses.install_paths(cmake).items():
        for destination, rename in targets:
            if destination.startswith(check_licenses.LEGAL_PREFIX):
                landed.add((source, check_licenses.installed_path(source, destination,
                                                                  rename)))
    return landed


class InstalledConsumer(unittest.TestCase):
    def test_the_manifest_names_every_install_into_the_legal_root(self):
        recorded = {(entry["source"], entry["destination"])
                    for entry in MANIFEST["distribution"]["cmake_install"]["legal_files"]}
        self.assertEqual(recorded, _legal_root_installs())

    def test_the_installed_checker_runs_without_a_checkout(self):
        """Stage the prefix from CMake's own install calls, then run the
        installed checker from outside the checkout: it needs its helper
        library beside it, not this tree on PYTHONPATH."""
        with tempfile.TemporaryDirectory() as tmp:
            prefix = Path(tmp) / "prefix"
            for source, destination in _legal_root_installs():
                target = prefix / destination
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((ROOT / source).read_bytes())
            checker = prefix / check_licenses.LEGAL_PREFIX / "tools" / "check_licenses.py"
            self.assertTrue(checker.is_file())
            self.assertTrue((checker.parent / "legal_members.py").is_file(),
                            "the checker's helper must ship beside it")
            result = subprocess.run([sys.executable, str(checker), "--install",
                                     str(prefix), "--quiet"], cwd=tmp,
                                    capture_output=True, text=True)
            self.assertNotIn("ModuleNotFoundError", result.stderr)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_a_rename_is_a_destination_not_an_installed_source(self):
        paths = check_licenses.install_paths(
            'install(FILES docs/legal_members.py RENAME legal_members.py\n'
            '        DESTINATION "share/rigExec/tools")\n'
            'install(FILES docs/examples/godot_rolling_ball.zip\n'
            '        DESTINATION "share/rigExec/docs")')
        self.assertEqual(sorted(paths), ["docs/examples/godot_rolling_ball.zip",
                                        "docs/legal_members.py"])
        self.assertEqual(
            check_licenses.installed_path("docs/legal_members.py",
                                         *paths["docs/legal_members.py"][0]),
            "share/rigExec/tools/legal_members.py")


class Composition(unittest.TestCase):
    """The godot packer reads its add-on-bound texts through the add-on root it
    is given, so every bound source must resolve inside that root and a link
    that leaves it must be refused."""

    def test_godot_addon_sources_resolve_in_the_authorized_root(self):
        bound = [entry for entry in MANIFEST["distribution"]["godot_zip"]["legal_members"]
                 if entry["source"].startswith("../")]
        self.assertTrue(bound)
        with tempfile.TemporaryDirectory() as tmp:
            addon = Path(tmp) / ADDON_NAME
            inside = {}
            for entry in bound:
                relative = PurePosixPath(entry["source"].split("/", 2)[2])
                inside[entry["member"]] = relative
                target = addon / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text("MIT\n", encoding="utf-8")
            for entry in bound:
                self.assertEqual(resolve_legal_source(entry["member"], entry["source"], addon),
                                 (addon / inside[entry["member"]]).resolve())
            outside = Path(tmp) / "elsewhere"
            outside.mkdir()
            (outside / "LICENSE.md").write_text("not ours\n", encoding="utf-8")
            _link(addon / "leaky", outside, directory=True)
            with self.assertRaises(ValueError):
                resolve_legal_source("member", f"../{ADDON_NAME}/leaky/LICENSE.md", addon)


class ProductionAudit(unittest.TestCase):
    def test_the_default_audit_catches_a_header_that_contradicts_its_family(self):
        """The checkout audit, not only the synthetic fixtures, must compare a
        recorded SPDX id and holder against the headers in the tree."""
        broken = json.loads(json.dumps(MANIFEST))
        stated = [family for family in broken["families"]
                  if family["id"] == "stated-mit-headers"]
        self.assertTrue(stated and stated[0]["spdx"] == "MIT")
        stated[0]["spdx"] = "GPL-3.0"
        errors, _ = check_licenses.validate(broken, ROOT, None, partition=False)
        self.assertTrue(any("contradicts" in error and "GPL-3.0" in error
                            for error in errors), errors)
        broken = json.loads(json.dumps(MANIFEST))
        for family in broken["families"]:
            if family.get("holder"):
                family["holder"] = "Squiggle Labs"
        errors, _ = check_licenses.validate(broken, ROOT, None, partition=False)
        self.assertTrue(any("holder replacement" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main(verbosity=2)
