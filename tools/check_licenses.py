#!/usr/bin/env python3
"""Verify tools/license_manifest.json against the checkout, the install map
and the archives.

The manifest claims that every legal text exists, that every link in
THIRD_PARTY_NOTICES.md reaches something a consumer can open, that CMake
installs exactly what the install map records, and that each archive holds
the members the inventory names. Paths, install maps and archive members are
checked here; the family partition is re-derived from the recorded globs.
The checker is stdlib-only so it runs without OpenUSD, and it never hashes
the manifest itself: the manifest's own bytes change whenever the inventory
is refreshed, so it is a provenance record, not a licensed text.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tools" / "license_manifest.json"
NOTICE_PATH = ROOT / "THIRD_PARTY_NOTICES.md"

sys.path.insert(0, str(ROOT / "docs"))
from legal_members import (archive_member_name, normalize_line_endings,  # noqa: E402
                           resolve_legal_source)


def load_manifest(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def glob_to_re(glob: str) -> re.Pattern:
    """Compile a manifest glob: '**' spans segments, '*' does not."""
    out = []
    i = 0
    while i < len(glob):
        if glob[i:i + 2] == "**":
            out.append(".*")
            i += 2
        elif glob[i] == "*":
            out.append("[^/]*")
            i += 1
        else:
            out.append(re.escape(glob[i]))
            i += 1
    return re.compile("^" + "".join(out) + "$")


def glob_matches(path: str, glob: str) -> bool:
    return glob_to_re(glob).match(path) is not None


def family_of(path: str, families: list[dict]) -> str | None:
    for family in sorted(families, key=lambda f: f["precedence"]):
        if family["id"] == "fallback-unclassified":
            continue
        if any(glob_matches(path, g) for g in family["globs"]):
            if any(glob_matches(path, c["glob"]) for c in family["carve_outs"]):
                continue
            return family["id"]
    return "fallback-unclassified"


def marker_scan(root: Path, files: list[str], manifest: dict) -> int:
    """Count in-scope plain-text files carrying a license or copyright marker."""
    excluded = manifest["inventory_boundary"]["inventory_scope_excludes"]
    count = 0
    for name in files:
        if any(glob_matches(name, g) for g in excluded):
            continue
        try:
            data = (root / name).read_bytes()
        except OSError:
            continue
        if b"\0" in data:
            continue
        lowered = data.lower()
        if any(word in lowered for word in (b"copyright", b"license", b"spdx")):
            count += 1
    return count


def tracked_files(root: Path) -> list[str] | None:
    """Every file of the result tree: tracked, plus untracked and not ignored."""
    try:
        tracked = subprocess.run(["git", "ls-files"], cwd=root, capture_output=True,
                                 text=True, check=True).stdout.split()
        added = subprocess.run(["git", "ls-files", "--other", "--exclude-standard"],
                               cwd=root, capture_output=True, text=True,
                               check=True).stdout.split()
    except Exception:
        return None
    return sorted(set(tracked) | set(added))


def git_blob_sha1(path: Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def install_paths(cmake_text: str) -> dict[str, list[tuple[str, str | None]]]:
    """source -> [(destination, rename)] for every install(FILES ...) call.

    A text can be installed more than once: the prefix carries the renamed
    alias and the source-relative copy side by side.
    """
    paths: dict[str, list[tuple[str, str | None]]] = {}
    for match in re.finditer(r"install\(FILES(.*?)\)", cmake_text, re.S):
        body = match.group(1)
        dest = re.search(r"DESTINATION\s+\"([^\"]+)\"", body)
        rename = re.search(r"RENAME\s+([^\s)]+)", body)
        if not dest:
            continue
        for token in body.split():
            if token in ("DESTINATION", "RENAME") or token.startswith("\""):
                continue
            paths.setdefault(token, []).append(
                (dest.group(1), rename.group(1) if rename else None))
    return paths


def installed_path(source: str, destination: str, rename: str | None) -> str:
    return f"{destination}/{rename or Path(source).name}"


def distribution_legal_members(manifest: dict) -> list[tuple[str, dict]]:
    """Every mapped legal member, tagged with its distribution section.

    Archives that hold several members per archive are listed under an
    "archives" key instead of an "archive" key.
    """
    members = []
    for section, content in manifest["distribution"].items():
        for entry in content.get("legal_members", []):
            members.append((section, entry))
        for sub in content.get("archives", []):
            for entry in sub.get("legal_members", []):
                members.append((section, entry))
    return members


def check_path_forms(manifest: dict) -> list[str]:
    errors = []
    for section, content in manifest["distribution"].items():
        for entry in content.get("legal_files", []):
            try:
                archive_member_name(entry["destination"])
            except ValueError as exc:
                errors.append(f"{section}: {exc}")
    for section, entry in distribution_legal_members(manifest):
        member = entry.get("member")
        if member is not None:
            try:
                archive_member_name(member)
            except ValueError as exc:
                errors.append(f"{section}: {exc}")
        source = entry.get("source")
        if isinstance(source, str) and "\\" in source:
            errors.append(f"{section}: backslash in source {source!r}")
        if isinstance(source, str) and not source.startswith("../"):
            if any(part in ("", ".", "..") for part in source.split("/")):
                errors.append(f"{section}: source {source!r} is not a clean "
                              "repository-relative path")
    for archive in manifest["inventory_boundary"]["archive_members_inventory"]:
        if archive["archive"] == "tools/license_manifest.json":
            errors.append("the manifest must not be inventoried as a legal text")
    for family in manifest["families"]:
        for text in family.get("required_texts", []):
            kinds = [kind for kind in ("path", "url", "prose") if kind in text]
            if not isinstance(text, dict) or len(kinds) != 1:
                errors.append(f"{family['id']}: required text entry {text!r} "
                             "must be an object with exactly one of path, url or prose")
    return errors


def check_sources_exist(manifest: dict, root: Path, addon_root: Path | None):
    """Return (errors, notes). Add-on-bound sources are unverifiable without a checkout."""
    errors = []
    notes = []
    for family in manifest["families"]:
        for text in family.get("required_texts", []):
            if "path" in text and not (root / text["path"]).is_file():
                errors.append(f"{family['id']}: required text {text['path']} is missing")
    for archive in manifest["inventory_boundary"]["archive_members_inventory"]:
        if not (root / archive["archive"]).is_file():
            errors.append(f"inventoried archive {archive['archive']} is missing")
    for section, entries in manifest["distribution"].items():
        for entry in entries.get("legal_files", []):
            if not (root / entry["source"]).is_file():
                errors.append(f"{section}: install source {entry['source']} is missing")
    for section, entry in distribution_legal_members(manifest):
        try:
            source = resolve_legal_source(entry["member"], entry["source"], addon_root)
        except ValueError as exc:
            if addon_root is None:
                notes.append(f"{section}: {entry['member']} is add-on bound "
                             f"({entry['source']}); no add-on root was supplied")
            else:
                errors.append(f"{section}: {exc}")
            continue
        if not source.is_file():
            errors.append(f"{section}: legal member {entry['member']} has no "
                         f"source text at {source}")
    return errors, notes


def check_canonical_names(manifest: dict) -> list[str]:
    """Root LICENSE, NOTICE and THIRD_PARTY_NOTICES.md must be named by their
    own file name."""
    errors = []
    for section, entries in manifest["distribution"].items():
        members = [entry for entry in entries.get("legal_members", [])
                   if entry.get("member")]
        if not members:
            continue
        names = {entry["member"] for entry in members}
        for canonical in ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"):
            if canonical in names or any(name.endswith("/" + canonical) for name in names):
                continue
            errors.append(f"{section}: no member carries the canonical root "
                          f"name {canonical}")
    return errors


def check_install_map(manifest: dict, root: Path) -> list[str]:
    installed = install_paths((root / "CMakeLists.txt").read_text(encoding="utf-8"))
    errors = []
    for entry in manifest["distribution"]["cmake_install"]["legal_files"]:
        matches = installed.get(entry["source"], [])
        if not matches:
            errors.append(f"cmake_install: {entry['source']} is not installed by any "
                         "install(FILES ...) call in CMakeLists.txt")
            continue
        landed = [installed_path(entry["source"], dest, renamed)
                  for dest, renamed in matches]
        if entry["destination"] not in landed:
            errors.append(f"cmake_install: {entry['source']} lands at {', '.join(landed)}, "
                         f"the manifest records {entry['destination']}")
        if entry.get("note", "").startswith("renamed") and not any(r for _, r in matches):
            errors.append(f"cmake_install: {entry['source']} is recorded as a renamed "
                         "alias but CMake installs it under its own name")
    return errors


def check_links(manifest: dict, root: Path) -> list[str]:
    notice = (root / NOTICE_PATH.name).read_text(encoding="utf-8")
    install_sources = {entry["source"]
                       for entry in manifest["distribution"]["cmake_install"]["legal_files"]}
    archive_members = set()
    for entries in manifest["distribution"].values():
        archive_members.update(entry["member"] for entry in entries.get("legal_members", []))
    prose = {ref["path"] for ref in manifest.get("prose_references", [])}
    errors = []
    for link in re.findall(r"\]\(([^)\s]+)\)", notice):
        if link.startswith(("http://", "https://")):
            continue
        target = link.lstrip("/").split("#")[0]
        if not target:
            continue
        if not (root / target).is_file():
            errors.append(f"THIRD_PARTY_NOTICES.md links {link}, "
                          "which is not in the checkout")
            continue
        if target in prose:
            continue
        reachable = target in install_sources or any(
            member.endswith("/" + target) for member in archive_members)
        if not reachable:
            errors.append(f"THIRD_PARTY_NOTICES.md links {link}, which reaches no member "
                         "of the install prefix or of a shipped archive")
    return errors


def check_archives(manifest: dict, root: Path, addon_root: Path | None):
    """Return (errors, notes) for every inventoried archive."""
    errors = []
    notes = []
    for record in manifest["inventory_boundary"]["archive_members_inventory"]:
        path = root / record["archive"]
        if not path.is_file():
            continue
        if "blob" in record and git_blob_sha1(path) != record["blob"]:
            errors.append(f"{record['archive']}: blob is {git_blob_sha1(path)}, "
                          f"the manifest records {record['blob']}")
        with zipfile.ZipFile(path) as archive:
            members = archive.namelist()
            payload_manifest = (json.loads(archive.read("manifest.json"))
                                if "manifest.json" in members else None)
        if "payload_digest" in record:
            if payload_manifest is None:
                errors.append(f"{record['archive']}: no manifest.json to recompute the "
                             "payload digest from")
            else:
                hashed = payload_manifest.get("files", {})
                digest = hashlib.sha256(
                    "".join(f"{name}\t{sha}\n" for name, sha in sorted(hashed.items()))
                    .encode("utf-8")).hexdigest()
                if len(hashed) != record.get("payload_members"):
                    errors.append(f"{record['archive']}: manifest.json hashes {len(hashed)} "
                                  f"payload members, the inventory records "
                                  f"{record.get('payload_members')}")
                if digest != record["payload_digest"]:
                    errors.append(f"{record['archive']}: payload digest is {digest}, "
                                  f"the inventory records {record['payload_digest']}")
        if record.get("members") != len(members):
            errors.append(f"{record['archive']}: {len(members)} members, "
                          f"the manifest records {record.get('members')}")
        legal_by_archive = {}
        for entries in manifest["distribution"].values():
            if entries.get("archive"):
                legal_by_archive[entries["archive"]] = entries.get("legal_members", [])
            for sub in entries.get("archives", []):
                legal_by_archive[sub["archive"]] = sub.get("legal_members", [])
        entries = legal_by_archive.get(record["archive"])
        if entries is None:
            continue
        present = [entry for entry in entries if entry["member"] in members]
        if record.get("legal_members") != len(present):
            errors.append(f"{record['archive']}: {len(present)} legal members present, "
                          f"the manifest records {record.get('legal_members')}")
        if record.get("legal_members_mapped") != len(entries):
            errors.append(f"{record['archive']}: {len(entries)} members are mapped, the "
                          f"manifest records {record.get('legal_members_mapped')}")
        for entry in entries:
            status = entry.get("status", "")
            if entry["member"] not in members:
                if "absent from the checked-in archive" not in status:
                    errors.append(f"{record['archive']}: legal member {entry['member']} "
                                  "is in the mapping but not in the archive")
                else:
                    notes.append(f"{record['archive']}: {entry['member']} is a mapping "
                                 "recorded on this boundary, not yet packed")
                continue
            try:
                source = resolve_legal_source(entry["member"], entry["source"], addon_root)
            except ValueError:
                continue
            if not source.is_file():
                continue
            archived = archive_bytes(path, entry["member"])
            if "byte-identical" in status or status == "current":
                if normalize_line_endings(archived) != normalize_line_endings(
                        source.read_bytes()):
                    errors.append(f"{record['archive']}: {entry['member']} differs from "
                                  f"its source {source}")
    return errors, notes


def archive_bytes(path: Path, member: str) -> bytes:
    with zipfile.ZipFile(path) as archive:
        return archive.read(member)


def check_partition(manifest: dict, root: Path) -> list[str]:
    files = tracked_files(root)
    if files is None:
        return ["git is unavailable, so the partition counts were not re-derived"]
    assignment = {f: family_of(f, manifest["families"]) for f in files}
    errors = []
    if len(files) != manifest["totals"]["assigned_files"]:
        errors.append(f"totals.assigned_files is {manifest['totals']['assigned_files']}, "
                      f"the tree has {len(files)} tracked files")
    unassigned = [f for f in files if assignment[f] == "fallback-unclassified"]
    if len(unassigned) != manifest["totals"]["unassigned_files"]:
        errors.append(f"totals.unassigned_files is {manifest['totals']['unassigned_files']}, "
                      f"{len(unassigned)} paths match no family")
    for family in manifest["families"]:
        actual = sum(1 for f in files if assignment[f] == family["id"])
        if actual != family["matched_files"]:
            errors.append(f"family {family['id']} matches {actual} files, "
                          f"the manifest records {family['matched_files']}")
    by_category = {}
    for f in files:
        family = next(x for x in manifest["families"] if x["id"] == assignment[f])
        by_category[family["category"]] = by_category.get(family["category"], 0) + 1
    for category, count in manifest["totals"]["by_category"].items():
        if by_category.get(category, 0) != count:
            errors.append(f"totals.by_category[{category}] is {count}, "
                          f"the partition gives {by_category.get(category, 0)}")
    markers = marker_scan(root, files, manifest)
    recorded = manifest["totals"]["files_with_license_or_copyright_markers_in_scope"]
    if markers != recorded:
        errors.append(f"the marker scan finds {markers} in-scope files, the "
                      f"manifest records {recorded}")
    excluded = manifest["inventory_boundary"]["inventory_scope_excludes"]
    scope = [f for f in files if not any(glob_matches(f, g) for g in excluded)]
    if len(scope) != manifest["inventory_boundary"]["tracked_files_in_scope"]:
        errors.append("inventory_boundary.tracked_files_in_scope is "
                      f"{manifest['inventory_boundary']['tracked_files_in_scope']}, "
                      f"the tree has {len(scope)} in-scope files")
    return errors


def validate(manifest: dict, root: Path = ROOT, addon_root: Path | None = None,
             partition: bool = True) -> tuple[list[str], list[str]]:
    errors = check_path_forms(manifest)
    source_errors, source_notes = check_sources_exist(manifest, root, addon_root)
    errors += source_errors
    notes = source_notes
    errors += check_canonical_names(manifest)
    errors += check_install_map(manifest, root)
    errors += check_links(manifest, root)
    archive_errors, notes = check_archives(manifest, root, addon_root)
    errors += archive_errors
    if partition:
        errors += check_partition(manifest, root)
    return errors, notes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default=str(MANIFEST_PATH))
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--addon-root", default=None,
                        help="add-on checkout root, when the packer has one")
    parser.add_argument("--no-partition", action="store_true",
                        help="skip re-deriving the family partition from git")
    args = parser.parse_args(argv)
    manifest = load_manifest(Path(args.manifest))
    errors, notes = validate(manifest, Path(args.root),
                            Path(args.addon_root) if args.addon_root else None,
                            partition=not args.no_partition)
    for note in notes:
        print(f"note: {note}")
    if errors:
        for error in errors:
            print(f"error: {error}")
        return 1
    print("license manifest is consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
