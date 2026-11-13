#!/usr/bin/env python3
"""Verify tools/license_manifest.json against the checkout, an install prefix
and the shipped archives.

The manifest claims that every legal text exists, that every link in
THIRD_PARTY_NOTICES.md reaches something a consumer can open, that CMake
installs exactly what the install map records, that each archive holds the
members the inventory names, and that the family partition is disjoint. The
manifest is the only exception source: nothing here carries its own allowlist.
The checker is stdlib-only so it runs without OpenUSD, and it never hashes the
manifest itself: the manifest's own bytes change whenever the inventory is
refreshed, so it is a provenance record, not a licensed text.

Modes:
  (default)      audit the checkout
  --install P    audit an install prefix (also accepts DESTDIR staging and a
                 relocated legal root)
  --archive A    audit one archive against its mapping
  --addon-root R resolve add-on-bound legal sources against R
  --self-test    run the synthetic fixtures offline, no checkout required
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import PurePosixPath, Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tools" / "license_manifest.json"
NOTICE_PATH = ROOT / "THIRD_PARTY_NOTICES.md"
LEGAL_PREFIX = "share/rigExec"

# legal_members.py travels beside this checker in the install prefix and lives
# in docs/ in the checkout, so an installed consumer needs no checkout.
for _helper_dir in (Path(__file__).resolve().parent, ROOT / "docs"):
    if (_helper_dir / "legal_members.py").is_file():
        sys.path.insert(0, str(_helper_dir))
        break
else:
    raise SystemExit("legal_members.py is missing: install it beside this "
                     "checker or run from a checkout")
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


def matching_families(path: str, families: list[dict]) -> list[dict]:
    """Every family that claims the path, carve-outs applied, by precedence."""
    hits = []
    for family in sorted(families, key=lambda f: f["precedence"]):
        if family["id"] == "fallback-unclassified":
            continue
        if any(glob_matches(path, g) for g in family["globs"]):
            if any(glob_matches(path, c["glob"]) for c in family.get("carve_outs", [])):
                continue
            hits.append(family)
    return hits


def family_of(path: str, families: list[dict]) -> str | None:
    hits = matching_families(path, families)
    return hits[0]["id"] if hits else "fallback-unclassified"


# Binary payloads are never scanned for markers: compressed data can contain
# 'spdx' by accident (see totals.false_positive_note).
_BINARY_MAGIC = (b"\x89PNG", b"\xff\xd8\xff", b"GIF8", b"PK\x03\x04", b"\x1f\x8b",
                 b"%PDF", b"RIFF", b"\x7fELF", b"MZ", b"OggS", b"\x00\x00\x00\x01")
_TEXT_WORDS = (b"copyright", b"license", b"spdx")


def is_plain_text(data: bytes) -> bool:
    if b"\0" in data or data.startswith(_BINARY_MAGIC):
        return False
    try:
        data.decode("utf-8")
    except UnicodeDecodeError:
        return False
    return not any(b < 32 and b not in (9, 10, 13) for b in data[:4096])


def marker_scan(root: Path, files: list[str], manifest: dict) -> int:
    """Count in-scope plain-text files carrying a license or copyright marker."""
    excluded = manifest["inventory_boundary"].get("inventory_scope_excludes", [])
    count = 0
    for name in files:
        if any(glob_matches(name, g) for g in excluded):
            continue
        try:
            data = (root / name).read_bytes()
        except OSError:
            continue
        if not is_plain_text(data):
            continue
        lowered = data.lower()
        if any(word in lowered for word in _TEXT_WORDS):
            count += 1
    return count


def inventory_result_tree(tracked: list[str], untracked: list[str]) -> list[str]:
    """The repository's inventory result tree.

    Tracked files, plus new files in a directory this repository already
    tracks. A whole untracked directory is outside the inventory even when
    nothing ignores it: a dependency a CI job stages or builds under the
    checkout root, or a nested checkout, is not this repository's file set.
    """
    tracked_dirs = {str(PurePosixPath(name).parent) for name in tracked}
    kept = set(tracked)
    kept.update(name for name in untracked
                if str(PurePosixPath(name).parent) in tracked_dirs)
    return sorted(kept)


def inventory(root: Path) -> tuple[list[str] | None, list[str]]:
    """(result tree, untracked paths left outside it), or (None, []) without
    git."""
    try:
        tracked = subprocess.run(["git", "ls-files"], cwd=root, capture_output=True,
                                 text=True, check=True).stdout.split()
        added = subprocess.run(["git", "ls-files", "--other", "--exclude-standard"],
                               cwd=root, capture_output=True, text=True,
                               check=True).stdout.split()
    except Exception:
        return None, []
    files = inventory_result_tree(tracked, added)
    outside = sorted(set(added) - set(files))
    return (files or None), outside


def tracked_files(root: Path) -> list[str] | None:
    """Every file of the repository's inventory result tree."""
    return inventory(root)[0]


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
        tokens = body.split()
        for index, token in enumerate(tokens):
            if token.startswith("\""):
                continue
            # A keyword's own value is not a source: RENAME names the write.
            if index and tokens[index - 1] in ("RENAME", "DESTINATION"):
                continue
            if token in ("DESTINATION", "RENAME"):
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


def archive_member_lists(manifest: dict, root: Path = ROOT) -> dict[str, list[dict]]:
    """archive path -> mapped legal members, keyed by resolved path and name."""
    mapped: dict[str, list[dict]] = {}
    for content in manifest["distribution"].values():
        records = [content] + content.get("archives", [])
        for record in records:
            if record.get("archive"):
                mapped[str((root / record["archive"]).resolve())] = \
                    record.get("legal_members", [])
                mapped[Path(record["archive"]).name] = record.get("legal_members", [])
    return mapped


def check_path_forms(manifest: dict) -> list[str]:
    errors = []
    for section, content in manifest["distribution"].items():
        for entry in content.get("legal_files", []):
            try:
                archive_member_name(entry["destination"])
            except ValueError as exc:
                errors.append(f"{section}: {exc}")
            if not str(entry.get("destination", "")).startswith(LEGAL_PREFIX + "/"):
                errors.append(f"{section}: install destination "
                              f"{entry.get('destination')!r} is outside the legal root "
                              f"{LEGAL_PREFIX}")
        for entry in content.get("legal_members", []):
            _check_member_forms(section, entry, errors)
        for sub in content.get("archives", []):
            for entry in sub.get("legal_members", []):
                _check_member_forms(section, entry, errors)
    for archive in manifest["inventory_boundary"].get(
            "archive_members_inventory", []):
        if archive["archive"] == "tools/license_manifest.json":
            errors.append("the manifest must not be inventoried as a legal text")
    for family in manifest["families"]:
        for text in family.get("required_texts", []):
            if not isinstance(text, dict):
                errors.append(f"{family['id']}: required text entry {text!r} "
                              "must be an object with one of path, url or prose")
                continue
            kinds = [kind for kind in ("path", "url", "prose") if kind in text]
            if len(kinds) != 1:
                errors.append(f"{family['id']}: required text entry {text!r} "
                             "must name exactly one of path, url or prose")
    return errors


def _check_member_forms(section: str, entry: dict, errors: list[str]) -> None:
    member = entry.get("member")
    if member is not None:
        try:
            archive_member_name(member)
        except ValueError as exc:
            errors.append(f"{section}: {exc}")
    source = entry.get("source")
    if isinstance(source, str):
        parts = source.split("/")
        add_on_bound = parts[0] == ".."
        if "\\" in source:
            errors.append(f"{section}: backslash in source {source!r}")
        elif any(part in ("", ".", "..") for part in (parts[1:] if add_on_bound else parts)):
            errors.append(f"{section}: source {source!r} is not a clean path "
                          "inside this checkout or an add-on root")


def check_family_patterns(manifest: dict, files: list[str]) -> list[str]:
    """Every family pattern must be live, every carve-out explicit, and the
    partition disjoint at equal precedence."""
    errors = []
    ids = {family["id"] for family in manifest["families"]}
    for family in manifest["families"]:
        if not family.get("globs"):
            errors.append(f"family {family['id']} has no globs")
            continue
        claimed = {path for path in files
                   if any(glob_matches(path, g) for g in family["globs"])}
        if family["id"] != "fallback-unclassified" and not claimed:
            errors.append(f"family {family['id']} matches no tracked file: "
                         f"{', '.join(family['globs'])}")
        for carve in family.get("carve_outs", []):
            glob = carve.get("glob")
            if not glob:
                errors.append(f"family {family['id']} has a carve-out with no glob")
                continue
            handled = carve.get("handled_by")
            if not handled or handled not in ids:
                errors.append(f"family {family['id']} carve-out {glob!r} names no "
                             "family that handles the path")
            if not any(glob_matches(path, glob) for path in files):
                errors.append(f"family {family['id']} carve-out {glob!r} names a "
                             "path that no longer exists")

    for path in files:
        hits = matching_families(path, manifest["families"])
        same_precedence = [hit for hit in hits
                          if hit["precedence"] == hits[0]["precedence"]] if hits else []
        if len(same_precedence) > 1:
            errors.append(f"{path} is claimed by {len(same_precedence)} families at "
                          f"the same precedence: {', '.join(h['id'] for h in same_precedence)}")
    return errors


def check_family_terms(manifest: dict) -> list[str]:
    """Category rules: unresolved material carries no grant, generated output
    names its generator, derived material names its upstream."""
    errors = []
    categories = manifest.get("categories", {})
    for family in manifest["families"]:
        category = family.get("category")
        if category not in categories:
            errors.append(f"family {family['id']} has unknown category {category!r}")
            continue
        if category == "unresolved" and family.get("spdx"):
            errors.append(f"family {family['id']} is unresolved but records SPDX "
                          f"{family['spdx']}; unresolved material has no grant")
        if category == "generated" and not family.get("generator"):
            errors.append(f"generated family {family['id']} names no generator")
        if category == "derived" and not family.get("provenance"):
            errors.append(f"derived family {family['id']} names no upstream source")
        if category == "derived" and not family.get("required_texts"):
            errors.append(f"derived family {family['id']} records no upstream "
                          "license text")
        generator = family.get("generator")
        if category == "generated" and generator:
            for token in re.findall(r"[\w./-]+\.(?:py|sh|bat|cmake|usda)", generator):
                if not (ROOT / token).is_file():
                    errors.append(f"generated family {family['id']} names generator "
                                  f"{token}, which is not in the checkout")
    return errors


_SPDX_RE = re.compile(r"SPDX-License-Identifier:\s*([A-Za-z0-9][A-Za-z0-9.\-]*)")
_COPYRIGHT_RE = re.compile(r"Copyright(?:\s*\(\s*[cC]\s*\))?\s*(?:\d{4})?\s*(.*)",
                           re.IGNORECASE)
_STOP_TOKENS = {"copyright", "rights", "reserved", "all", "limited", "company",
                "corporation", "authors", "project", "notice", "notices", "license",
                "licensed", "licensing", "terms", "distribution", "portions", "file",
                "this", "that", "with", "shall", "include", "included", "copy"}


def _holder_tokens(text: str) -> set[str]:
    return {word.rstrip(".") for word in re.findall(r"[A-Za-z][A-Za-z.]*", text)
            if len(word) > 3 and word.lower() not in _STOP_TOKENS}


def holders_agree(declared: str, header: str) -> bool:
    declared_low = declared.lower().strip(". ")
    header_low = header.lower().strip(". ")
    if declared_low in header_low or header_low in declared_low:
        return True
    return bool(_holder_tokens(declared) & _holder_tokens(header))


def source_header(path: Path) -> tuple[list[str], list[str]]:
    """SPDX identifiers and copyright holders stated in the file header."""
    try:
        data = path.read_bytes()
    except OSError:
        return [], []
    if not is_plain_text(data):
        return [], []
    lines = data.decode("utf-8", "replace").splitlines()[:24]
    spdx: list[str] = []
    holders: list[str] = []
    for line in lines:
        for identifier in _SPDX_RE.findall(line):
            spdx.append(identifier)
        body = line.strip().lstrip("#/*!<>\"'{}[]=- ").strip()
        if not body.lower().startswith("copyright"):
            continue
        match = _COPYRIGHT_RE.search(body)
        if not match:
            continue
        holder = match.group(1).split("All rights reserved")[0].strip(" .")
        if re.search(r"[A-Za-z]{3}", holder):
            holders.append(holder)
    return spdx, holders


def _first_holder(family: dict, holders: list[str]) -> str:
    return family.get("holder") or holders[0]

def check_source_headers(manifest: dict, root: Path, files: list[str]) -> list[str]:
    """A source header must agree with the family the manifest put it in.

    Missing headers are not violations: only a header that contradicts the
    recorded holder or license is reported, and the checker never guesses a
    provenance it cannot evidence.
    """
    excluded = manifest["inventory_boundary"].get("inventory_scope_excludes", [])
    errors = []
    for family in manifest["families"]:
        for path in files:
            if any(glob_matches(path, g) for g in excluded):
                continue
            if family_of(path, manifest["families"]) != family["id"]:
                continue
            spdx, holders = source_header(root / path)
            declared_spdx = family.get("spdx")
            if declared_spdx:
                for identifier in spdx:
                    if identifier != declared_spdx:
                        errors.append(f"{path}: header SPDX {identifier} contradicts "
                                      f"family {family['id']} ({declared_spdx}); "
                                      "needs an owner decision, not a guess")
            elif spdx and family["category"] == "unresolved":
                errors.append(f"{path}: header SPDX {spdx[0]} sits in unresolved "
                              f"family {family['id']}; provenance must be recorded "
                              "before any grant is claimed")
            if family.get("holder"):
                for holder in holders:
                    if not holders_agree(family["holder"], holder):
                        errors.append(f"{path}: header names holder {holder!r}, "
                                      f"family {family['id']} records "
                                      f"{family['holder']!r}; holder replacement "
                                      "needs an owner decision")
            elif holders and family["category"] == "generated":
                errors.append(f"{path}: generated output names holder {holders[0]!r} "
                             f"while family {family['id']} records no grant from "
                             "its inputs")
            elif holders and family["category"] == "unresolved":
                errors.append(f"{path}: header names holder {holders[0]!r} while "
                             f"family {family['id']} is recorded as unresolved")
    return errors


def check_sources_exist(manifest: dict, root: Path, addon_root: Path | None):
    """Return (errors, notes). Add-on-bound sources are unverifiable here."""
    errors = []
    notes = []
    for family in manifest["families"]:
        for text in family.get("required_texts", []):
            if isinstance(text, dict) and "path" in text:
                if not (root / text["path"]).is_file():
                    errors.append(f"{family['id']}: required text {text['path']} "
                                  "is missing")
    for archive in manifest["inventory_boundary"].get(
            "archive_members_inventory", []):
        if not (root / archive["archive"]).is_file():
            errors.append(f"inventoried archive {archive['archive']} is missing")
    for section, entries in manifest["distribution"].items():
        for entry in entries.get("legal_files", []):
            if not (root / entry["source"]).is_file():
                errors.append(f"{section}: install source {entry['source']} is missing")
    for section, entry in distribution_legal_members(manifest):
        try:
            source = resolve_legal_source(entry["member"], entry["source"], addon_root,
                                         root)
        except ValueError:
            if addon_root is None:
                notes.append(f"{section}: {entry['member']} is add-on bound "
                            f"({entry['source']}); no add-on root was supplied")
            else:
                errors.append(f"{section}: {entry['member']}: {entry['source']} "
                             "is not inside the add-on root")
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
        members = [entry["member"] for entry in entries.get("legal_members", [])
                   if entry.get("member")]
        if not members:
            continue
        names = set(members)
        for canonical in ("LICENSE", "NOTICE", "THIRD_PARTY_NOTICES.md"):
            if canonical in names or any(name.endswith("/" + canonical)
                                         for name in names):
                continue
            errors.append(f"{section}: no member carries the canonical root "
                          f"name {canonical}")
    return errors


def check_install_map(manifest: dict, root: Path) -> list[str]:
    section = manifest["distribution"].get("cmake_install")
    if not section or not section.get("legal_files"):
        return []
    try:
        installed = install_paths((root / "CMakeLists.txt").read_text(encoding="utf-8"))
    except OSError:
        return ["CMakeLists.txt is missing, so the install map cannot be verified"]
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
            errors.append(f"cmake_install: {entry['source']} lands at "
                          f"{', '.join(landed)}, the manifest records "
                          f"{entry['destination']}")
        if entry.get("note", "").startswith("renamed") and not any(r for _, r in matches):
            errors.append(f"cmake_install: {entry['source']} is recorded as a renamed "
                         "alias but CMake installs it under its own name")
    return errors


def notice_links(notice_text: str) -> list[str]:
    return [link for link in re.findall(r"\]\(([^)\s]+)\)", notice_text)
            if not link.startswith(("http://", "https://", "mailto:"))]


def check_links(manifest: dict, root: Path) -> list[str]:
    try:
        notice = (root / NOTICE_PATH.name).read_text(encoding="utf-8")
    except OSError:
        return [f"THIRD_PARTY_NOTICES.md is missing from the checkout"]
    install_sources = {entry["source"]
                       for entry in manifest["distribution"]["cmake_install"]["legal_files"]}
    archive_members = set()
    for entries in manifest["distribution"].values():
        archive_members.update(entry["member"] for entry in entries.get("legal_members", []))
        for sub in entries.get("archives", []):
            archive_members.update(entry["member"] for entry in sub.get("legal_members", []))
    prose = {ref["path"] for ref in manifest.get("prose_references", [])}
    errors = []
    for link in notice_links(notice):
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
            errors.append(f"THIRD_PARTY_NOTICES.md links {link}, which reaches no "
                         "member of the install prefix or of a shipped archive")
    return errors


def compare_legal_members(manifest: dict, archive_name: str, entries: list[dict],
                          members: list[str], path: Path,
                          addon_root: Path | None, root: Path = ROOT) -> list[str]:
    """Stale legal text is an error; a line-ending difference is not.

    The manifest copy inside an archive is a snapshot, not a legal text: the
    manifest's own bytes change whenever the inventory is refreshed, so it is
    never compared byte for byte. It is checked for presence and shape."""
    errors = []
    with zipfile.ZipFile(path) as handle:
        available = set(handle.namelist())
        texts = {name: handle.read(name) for name in available}
    for entry in entries:
        member = entry["member"]
        status = entry.get("status", "")
        if member not in available:
            if "absent from the checked-in archive" in status or entry.get("packed") is False:
                continue
            errors.append(f"{archive_name}: legal member {member} is in the mapping "
                          "but not in the archive")
            continue
        if Path(member).name == MANIFEST_PATH.name:
            try:
                snapshot = json.loads(texts[member].decode("utf-8"))
            except (UnicodeDecodeError, ValueError):
                errors.append(f"{archive_name}: {member} is not readable JSON")
                continue
            if snapshot.get("schema_version") != manifest.get("schema_version"):
                errors.append(f"{archive_name}: the shipped manifest snapshot has "
                             f"schema_version {snapshot.get('schema_version')}, the "
                             f"manifest being checked is version "
                             f"{manifest.get('schema_version')}")
            if len(snapshot.get("families", [])) != len(manifest.get("families", [])):
                errors.append(f"{archive_name}: the shipped manifest snapshot "
                             f"inventories {len(snapshot.get('families', []))} "
                             f"families, the manifest being checked has "
                             f"{len(manifest.get('families', []))}")
            continue
        try:
            source = resolve_legal_source(member, entry["source"], addon_root, root)
        except ValueError as exc:
            if (addon_root is None
                    and str(entry.get("source", "")).strip().startswith("../")):
                # Add-on bound without a root: reported unverifiable elsewhere.
                continue
            errors.append(f"{archive_name}: legal member {member}: {exc}")
            continue
        if not source.is_file():
            # A missing mapped source is reported by check_sources_exist.
            continue
        normalized = normalize_line_endings(texts[member])
        source_normalized = normalize_line_endings(source.read_bytes())
        if "byte-identical" in status or status == "current":
            if normalized != source_normalized:
                errors.append(f"{archive_name}: {member} differs from its source "
                             f"{source.name}")
        elif "stale" in status:
            if normalized == source_normalized:
                errors.append(f"{archive_name}: {member} is recorded stale but now "
                             "matches its source; refresh the mapping")
    return errors


def check_archives(manifest: dict, root: Path, addon_root: Path | None):
    """Return (errors, notes) for every inventoried archive."""
    errors = []
    notes = []
    for record in manifest["inventory_boundary"].get(
            "archive_members_inventory", []):
        path = root / record["archive"]
        if not path.is_file():
            continue
        if "blob" in record and git_blob_sha1(path) != record["blob"]:
            errors.append(f"{record['archive']}: blob is {git_blob_sha1(path)}, "
                          f"the manifest records {record['blob']}")
        with zipfile.ZipFile(path) as handle:
            members = handle.namelist()
            payload_manifest = (json.loads(handle.read("manifest.json"))
                                if "manifest.json" in members else None)
        if "payload_digest" in record:
            if payload_manifest is None:
                errors.append(f"{record['archive']}: no manifest.json to recompute "
                             "the payload digest from")
            else:
                hashed = payload_manifest.get("files", {})
                digest = hashlib.sha256(
                    "".join(f"{name}\t{sha}\n"
                            for name, sha in sorted(hashed.items())).encode("utf-8")
                ).hexdigest()
                if len(hashed) != record.get("payload_members"):
                    errors.append(f"{record['archive']}: manifest.json hashes "
                                  f"{len(hashed)} payload members, the inventory "
                                  f"records {record.get('payload_members')}")
                if digest != record["payload_digest"]:
                    errors.append(f"{record['archive']}: payload digest is {digest}, "
                                  f"the inventory records {record['payload_digest']}")
        if record.get("members") != len(members):
            errors.append(f"{record['archive']}: {len(members)} members, "
                          f"the manifest records {record.get('members')}")
        entries = archive_member_lists(manifest).get(record["archive"])
        archive_errors, stats, archive_notes = check_archive(manifest, path, root,
                                                            addon_root)
        errors += archive_errors
        notes += archive_notes
        if stats.get("mapped_legal") is None or entries is None:
            continue
        if record.get("legal_members_mapped") != stats["mapped_legal"]:
            errors.append(f"{record['archive']}: {stats['mapped_legal']} members are "
                          f"mapped, the manifest records "
                          f"{record.get('legal_members_mapped')}")
        if record.get("legal_members") != stats["legal_present"]:
            errors.append(f"{record['archive']}: {stats['legal_present']} legal "
                          f"members present, the manifest records "
                          f"{record.get('legal_members')}")
    return errors, notes


def check_partition(manifest: dict, root: Path, files: list[str]):
    errors = check_family_patterns(manifest, files)
    assignment = {f: family_of(f, manifest["families"]) for f in files}
    if len(files) != manifest["totals"]["assigned_files"]:
        errors.append(f"totals.assigned_files is "
                      f"{manifest['totals']['assigned_files']}, the tree has "
                      f"{len(files)} tracked files")
    unassigned = [f for f in files if assignment[f] == "fallback-unclassified"]
    if len(unassigned) != manifest["totals"]["unassigned_files"]:
        errors.append(f"totals.unassigned_files is "
                      f"{manifest['totals']['unassigned_files']}, {len(unassigned)} "
                      "paths match no family")
    for family in manifest["families"]:
        actual = sum(1 for f in files if assignment[f] == family["id"])
        if actual != family["matched_files"]:
            errors.append(f"family {family['id']} matches {actual} files, the "
                          f"manifest records {family['matched_files']}")
    by_category: dict[str, int] = {}
    for f in files:
        family = next(x for x in manifest["families"] if x["id"] == assignment[f])
        by_category[family["category"]] = by_category.get(family["category"], 0) + 1
    for category, count in manifest["totals"]["by_category"].items():
        if by_category.get(category, 0) != count:
            errors.append(f"totals.by_category[{category}] is {count}, the partition "
                          f"gives {by_category.get(category, 0)}")
    markers = marker_scan(root, files, manifest)
    recorded = manifest["totals"].get(
        "files_with_license_or_copyright_markers_in_scope")
    if recorded is not None and markers != recorded:
        errors.append(f"the marker scan finds {markers} in-scope files, the manifest "
                      f"records {recorded}")
    excluded = manifest["inventory_boundary"].get("inventory_scope_excludes", [])
    scope = [f for f in files if not any(glob_matches(f, g) for g in excluded)]
    in_scope = manifest["inventory_boundary"].get("tracked_files_in_scope")
    if in_scope is not None and len(scope) != in_scope:
        errors.append(f"inventory_boundary.tracked_files_in_scope is {in_scope}, "
                      f"the tree has {len(scope)} in-scope files")
    return errors, []


def find_legal_root(prefix: Path) -> Path | None:
    """The legal root of an install prefix: share/rigExec, or the relocated or
    DESTDIR-staged equivalent of it."""
    direct = prefix / LEGAL_PREFIX
    if direct.is_dir():
        return direct
    best = None
    for candidate in prefix.rglob("rigExec"):
        if not candidate.is_dir():
            continue
        if (candidate / "LICENSE").is_file() and (candidate / "NOTICE").is_file():
            depth = len(candidate.relative_to(prefix).parts)
            if best is None or depth < len(best.relative_to(prefix).parts):
                best = candidate
    return best


def check_install_prefix(manifest: dict, prefix: Path, root: Path = ROOT):
    """Audit an installed prefix: every mapped legal text is there, current,
    and every link in the shipped notice reaches a member of the prefix."""
    legal_root = find_legal_root(prefix)
    if legal_root is None:
        return ([f"install prefix {prefix} has no legal root ({LEGAL_PREFIX})"],
                [], {})
    errors: list[str] = []
    notes: list[str] = []
    mapped = manifest["distribution"]["cmake_install"]["legal_files"]
    present = 0
    stale = 0
    for entry in mapped:
        destination = entry["destination"]
        relative = destination[len(LEGAL_PREFIX) + 1:]
        landed = legal_root / relative
        if not landed.is_file():
            errors.append(f"{prefix}: {destination} is not in the installed prefix")
            continue
        present += 1
        source = root / entry["source"]
        if not source.is_file():
            notes.append(f"{prefix}: {destination} has no source text in this "
                        "checkout to compare against")
            continue
        if normalize_line_endings(landed.read_bytes()) != normalize_line_endings(
                source.read_bytes()):
            stale += 1
            errors.append(f"{prefix}: installed {destination} is stale against "
                         f"{entry['source']}")
    notice = legal_root / "THIRD_PARTY_NOTICES.md"
    resolved = 0
    if notice.is_file():
        prose = {ref["path"] for ref in manifest.get("prose_references", [])}
        for link in notice_links(notice.read_text(encoding="utf-8")):
            target = link.lstrip("/").split("#")[0]
            if not target:
                continue
            if (legal_root / target).is_file():
                resolved += 1
            elif target in prose:
                notes.append(f"prose reference {target} is source-only, as the "
                            "manifest records")
            else:
                errors.append(f"{prefix}: installed THIRD_PARTY_NOTICES.md links "
                             f"{link}, which is not in the prefix")
    stats = {"legal_root": str(legal_root.relative_to(prefix)),
             "mapped": len(mapped), "present": present, "stale": stale,
             "links_resolved": resolved}
    return errors, notes, stats


def mirror_of(member: str, legal_members: set[str]) -> str | None:
    """The mapped member this one duplicates, if it is a path mirror of it.

    The Godot archive ships the add-on twice so the demo opens standalone;
    the mirrored license text is the same file, so it is covered by the
    mapping of the original copy."""
    parts = member.split("/")
    for index in range(len(parts) - 1):
        candidate = "/".join(parts[:index] + parts[index + 1:])
        if candidate in legal_members:
            return candidate
    return None


def check_archive(manifest: dict, archive: Path, root: Path = ROOT,
                  addon_root: Path | None = None):
    """Audit one archive against the mapping that names it."""
    if not archive.is_file():
        return [f"archive {archive} does not exist"], {}, []
    mapped = archive_member_lists(manifest, root)
    entries = mapped.get(str(archive.resolve())) or mapped.get(archive.name)
    if entries is None:
        for record in manifest["inventory_boundary"].get(
                "archive_members_inventory", []):
            if record["archive"] == str(archive) or record["archive"] == archive.name:
                return ([f"the manifest inventories {archive} but maps no legal "
                         "members for it"], {"members": None}, [])
        return ([f"the manifest maps no legal members for archive {archive}"], {}, [])
    with zipfile.ZipFile(archive) as handle:
        members = handle.namelist()
    errors = compare_legal_members(manifest, archive.name, entries, members, archive,
                                   addon_root, root)
    legal = {entry["member"] for entry in entries if entry["member"] in members}
    notes: list[str] = []
    with zipfile.ZipFile(archive) as handle:
        texts = {name: handle.read(name) for name in members}
    prose = {ref["path"] for ref in manifest.get("prose_references", [])}
    for name in members:
        if name in legal:
            continue
        data = texts[name]
        if not is_plain_text(data) or not any(word in name.lower()
                                             for word in ("license", "copyright",
                                                         "notice", "eula")):
            continue
        original = mirror_of(name, legal)
        if original and normalize_line_endings(texts[original]) == normalize_line_endings(data):
            notes.append(f"{archive.name}: member {name} is a byte-identical mirror "
                         f"of the mapped legal text {original}")
        else:
            errors.append(f"{archive.name}: member {name} carries a license text the "
                         "manifest does not map")
    for notice in sorted(name for name in members
                         if PurePosixPath(name).name == NOTICE_PATH.name):
        # Every copy of the canonical notice is checked against the members of
        # the directory it sits in, which for the Godot archive is nested under
        # godot_rigExec/. The historical aliases are not notices a consumer reads.
        text = texts[notice].decode("utf-8", "replace")
        folder = PurePosixPath(notice).parent
        base = "" if str(folder) == "." else f"{folder}/"
        for link in notice_links(text):
            target = link.lstrip("/").split("#")[0]
            if not target:
                continue
            if base + target in members:
                continue
            if target in prose:
                notes.append(f"{archive.name}: the shipped notice {notice} links "
                             f"{target}, which the manifest records as a "
                             f"{prose_note(manifest, target)}")
            else:
                errors.append(f"{archive.name}: the shipped notice {notice} links "
                              f"{link}, which is not a member of the archive")
    stats = {"members": len(members), "mapped_legal": len(entries),
             "legal_present": len(legal),
             "mirrored_legal": sum(1 for note in notes if "mirror" in note),
             "payload_members": len(members) - len(legal)}
    return errors, stats, notes


def prose_note(manifest: dict, target: str) -> str:
    for ref in manifest.get("prose_references", []):
        if ref["path"] == target:
            return ref.get("kind", "source-only reference")
    return "source-only reference"


def validate(manifest: dict, root: Path = ROOT, addon_root: Path | None = None,
             partition: bool = True) -> tuple[list[str], list[str]]:
    errors = check_path_forms(manifest)
    errors += check_family_terms(manifest)
    source_errors, notes = check_sources_exist(manifest, root, addon_root)
    errors += source_errors
    errors += check_canonical_names(manifest)
    errors += check_install_map(manifest, root)
    errors += check_links(manifest, root)
    archive_errors, archive_notes = check_archives(manifest, root, addon_root)
    errors += archive_errors
    notes += archive_notes
    files = tracked_files(root)
    if files is None:
        notes.append("partition completeness and source-header agreement are "
                     "unverifiable: this tree has no git, so neither the family "
                     "counts nor the headers were re-derived")
    else:
        errors += check_source_headers(manifest, root, files)
        if partition:
            partition_errors, unverifiable = check_partition(manifest, root, files)
            errors += partition_errors
            notes.extend(unverifiable)
    return errors, notes


# ---------------------------------------------------------------- self test --

def _family(family_id: str, category: str, globs: list[str], *,
            holder: str | None = None, spdx: str | None = None,
            generator: str | None = None, provenance: list[str] | None = None,
            required_texts: list[dict] | None = None,
            carve_outs: list[dict] | None = None, precedence: int = 1) -> dict:
    return {"id": family_id, "precedence": precedence, "category": category,
            "globs": globs, "carve_outs": carve_outs or [], "spdx": spdx,
            "holder": holder, "provenance": provenance or [],
            "required_texts": required_texts or [], "generator": generator,
            "matched_files": 0}


def _manifest(families: list[dict], distribution: dict | None = None,
              archives: list[dict] | None = None) -> dict:
    families = list(families)
    if not any(family["id"] == "fallback-unclassified" for family in families):
        families.append(_family("fallback-unclassified", "unresolved", ["**"],
                                precedence=len(families) + 1))
    return {
        "schema_version": 1,
        "categories": {"original": "authored by the project",
                       "derived": "built from a named upstream",
                       "bundled": "third-party material",
                       "generated": "machine output",
                       "unresolved": "no provenance evidence"},
        "families": families,
        "totals": {"assigned_files": 0, "unassigned_files": 0, "by_category": {}},
        "inventory_boundary": {"inventory_scope_excludes": ["thirdparty/**"],
                               "archive_members_inventory": archives or [],
                               "tracked_files_in_scope": 0},
        "distribution": distribution or {},
        "prose_references": [],
    }


def _tree(files: dict[str, bytes | str]) -> Path:
    root = Path(tempfile.mkdtemp(prefix="license-check-"))
    for name, data in files.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data.encode("utf-8") if isinstance(data, str) else data)
    return root


def _linked_tree(files: dict[str, str]) -> Path:
    """The same tree reached through a path the platform rewrites on resolve.

    A symlink where links may be created: macOS already spells a temp tree
    through its /var link, and an unprivileged Windows runner cannot link, so
    there the plain path is used.
    """
    real = _tree(files)
    holder = Path(tempfile.mkdtemp(prefix="license-check-link-"))
    try:
        (holder / "checkout").symlink_to(real, target_is_directory=True)
    except OSError:
        return real
    return holder / "checkout"


def _guarded_tree(tmp: Path) -> Path:
    """A tree whose ESCAPE link leaves it and whose INNER link stays inside it.

    Raises OSError on a host without link privilege, where containment cannot
    be probed.
    """
    root = tmp / "guarded"
    (root / "licenses").mkdir(parents=True)
    (root / "licenses" / "LICENSE").write_text("MIT\n", encoding="utf-8")
    outside = tmp / "elsewhere"
    outside.mkdir()
    (outside / "LICENSE").write_text("not ours\n", encoding="utf-8")
    (root / "ESCAPE").symlink_to(outside / "LICENSE")
    (root / "licenses" / "INNER").symlink_to(root / "licenses" / "LICENSE")
    return root


def _zip(path: Path, members: dict[str, str | bytes]) -> Path:
    with zipfile.ZipFile(path, "w") as handle:
        for name, data in members.items():
            handle.writestr(name, data.encode("utf-8") if isinstance(data, str) else data)
    return path


MIT_TEXT = "Copyright (c) 2026 Nick Burkard\nPermission is hereby granted.\n"
NOTICE_TEXT = "Copyright (c) 2026 Nick Burkard\nOpenUSD is Pixar's.\n"
ORIGINAL = _family("original-code", "original", ["src/**", "LICENSE", "NOTICE"],
                   holder="Nick Burkard", spdx="MIT",
                   required_texts=[{"path": "LICENSE"}, {"path": "NOTICE"}])
ORIGINAL_TREE = {"LICENSE": MIT_TEXT, "NOTICE": NOTICE_TEXT,
                 "src/engine.cpp": "// engine entry point, no header\n"}


def _cases() -> list[tuple[str, bool | None, str]]:
    cases = []

    def add(name, expected, detail):
        cases.append((name, expected, detail))

    # A file with no header is covered by the root LICENSE: original MIT.
    root = _tree(ORIGINAL_TREE)
    manifest = _manifest([ORIGINAL])
    errors = (check_family_terms(manifest)
              + check_sources_exist(manifest, root, None)[0]
              + check_source_headers(manifest, root, list(ORIGINAL_TREE)))
    add("original MIT covered by LICENSE, headerless file accepted",
        not errors, errors or "no errors")

    # A header naming a different holder than the family records: caught.
    root = _tree({"src/engine.cpp": "// Copyright (c) 2019 Squiggle Labs\n"
                                    "// SPDX-License-Identifier: GPL-3.0\n"})
    errors = check_source_headers(_manifest([ORIGINAL]), root, ["src/engine.cpp"])
    add("holder replacement in an original file is caught",
        any("holder" in error for error in errors), errors[0] if errors else "none")

    # Meta/Epic MIT header inside a family that records Epic MIT: accepted.
    ports = _family("riglogic-ports", "derived", ["ports/**"],
                    holder="Epic Games, Inc.", spdx="MIT",
                    provenance=["rigLogic port, upstream MIT"],
                    required_texts=[{"url": "https://www.unrealengine.com/en/terms"}])
    root = _tree({"ports/chain.cpp": "// Copyright (c) 2020 Epic Games, Inc.\n"
                                     "// All rights reserved.\n"
                                     "// SPDX-License-Identifier: MIT\n"})
    errors = (check_family_terms(_manifest([ports]))
              + check_source_headers(_manifest([ports]), root, ["ports/chain.cpp"]))
    add("Meta/Epic MIT header agrees with its family", not errors, errors or "no errors")

    # Derived material with no named upstream: caught.
    errors = check_family_terms(_manifest([_family("vendor", "derived", ["vendor/**"],
                                                  holder="Someone", spdx="Apache-2.0")]))
    add("derived family without an upstream is caught",
        any("no upstream" in error for error in errors), errors[0] if errors else "none")

    # Derived family whose required text is a url: never a missing file.
    family = _family("vendor", "derived", ["vendor/**"], holder="Upstream",
                     spdx="Apache-2.0", provenance=["upstream commit pinned"],
                     required_texts=[{"url": "https://example.com/license"}])
    manifest = _manifest([family])
    errors = check_sources_exist(manifest, _tree({"vendor/core.cpp": "// port\n"}), None)[0]
    add("a url required text is informational, not a file check",
        not errors, errors or "no errors")

    # Derived family whose required text file is absent: caught.
    family = _family("bundled-lib", "bundled", ["thirdparty/lib/**"], holder="Upstream",
                     spdx="Apache-2.0", required_texts=[{"path": "thirdparty/lib/LICENSE"}])
    manifest = _manifest([family])
    manifest["inventory_scope_excludes"] = []
    root = _tree({"thirdparty/lib/README.md": "no license text here\n"})
    errors = check_sources_exist(manifest, root, None)[0]
    add("a required text that does not exist is caught",
        any("required text" in error for error in errors), errors[0] if errors else "none")

    # Generated output with no generator named: caught.
    errors = check_family_terms(_manifest([_family("gen", "generated", ["generated/**"])]))
    add("generated family without a generator is caught",
        any("no generator" in error for error in errors), errors[0] if errors else "none")

    # Generated output whose generator file no longer exists: caught.
    errors = check_family_terms(_manifest([_family("gen", "generated", ["generated/**"],
                                                  generator="bin/gen_gone.sh")]))
    add("a stale generator name is caught",
        any("generator" in error for error in errors), errors[0] if errors else "none")

    # Generated output carrying a holder the inputs never granted: caught.
    family = _family("gen", "generated", ["generated/**"], generator="docs/build_pages.py")
    root = _tree({"generated/node.md": "// Copyright (c) 2026 Squiggle Labs\n"
                                       "// SPDX-License-Identifier: GPL-3.0\n"})
    errors = check_source_headers(_manifest([family]), root, ["generated/node.md"])
    add("generated output contradicting its inputs is caught", bool(errors),
        errors[0] if errors else "none")

    # Unresolved material may not carry a grant.
    errors = check_family_terms(_manifest([_family("icons", "unresolved", ["icons/**"],
                                                  spdx="MIT")]))
    add("unresolved family with an SPDX grant is caught",
        any("no grant" in error for error in errors), errors[0] if errors else "none")

    # Unresolved material whose header names a holder: caught.
    family = _family("icons", "unresolved", ["icons/**"])
    root = _tree({"icons/atlas.svg": "<!-- Copyright (c) 2021 Icon Foundry -->\n"})
    errors = check_source_headers(_manifest([family]), root, ["icons/atlas.svg"])
    add("a header holder inside an unresolved family is caught", bool(errors),
        errors[0] if errors else "none")

    # Binary payloads are never scanned, even when bytes spell 'spdx'.
    family = _family("assets", "unresolved", ["assets/**"])
    manifest = _manifest([family])
    root = _tree({"assets/texture.png": b"\x89PNG\r\n\x1a\n" + b"spdx\x00\xff\xfe",
                  "assets/blob.bin": b"spdx copyright\x00\xff\xfe"})
    count = marker_scan(root, ["assets/texture.png", "assets/blob.bin"], manifest)
    add("binary payloads carrying 'spdx' are not scanned", count == 0,
        f"{count} marker files found")

    # Stale archive legal text: caught. CRLF-only difference: tolerated.
    sources = _tree(ORIGINAL_TREE)
    stale_archive = _zip(Path(tempfile.mktemp(suffix="tutorial.zip")),
                         {"LICENSE": "MIT but edited\n"})
    entries = [{"member": "LICENSE", "source": "LICENSE",
                "status": "byte-identical to source after CRLF normalization"}]
    fixture_manifest = _manifest([ORIGINAL])
    errors = compare_legal_members(fixture_manifest, stale_archive.name, entries,
                                   ["LICENSE"], stale_archive, None, sources)
    add("a stale archive legal member is caught", bool(errors),
        errors[0] if errors else "none")

    crlf_archive = _zip(Path(tempfile.mktemp(suffix="tutorial.zip")),
                        {"LICENSE": MIT_TEXT.replace("\n", "\r\n")})
    errors = compare_legal_members(fixture_manifest, crlf_archive.name, entries,
                                   ["LICENSE"], crlf_archive, None, sources)
    add("a CRLF-only difference is tolerated", not errors, errors or "no errors")

    # Unsafe member or destination forms: caught.
    manifest = _manifest([ORIGINAL], {
        "tutorial": {"archive": "tutorial.zip",
                     "legal_members": [{"member": "../LICENSE", "source": "LICENSE"},
                                       {"member": "licenses\\MIT.txt",
                                        "source": "thirdparty/MIT.txt"}]},
        "cmake_install": {"legal_files": [{"source": "LICENSE",
                                          "destination": "rigExec/../LICENSE"}]}})
    errors = check_path_forms(manifest)
    add("unsafe member and destination forms are caught", len(errors) >= 3,
        "; ".join(errors))

    # Overlapping families at equal precedence: caught.
    errors = check_family_patterns(
        _manifest([_family("orig-a", "original", ["src/**"]),
                   _family("orig-b", "original", ["src/engine.*"])]),
        ["src/engine.cpp"])
    add("overlapping families at equal precedence are caught",
        any("same precedence" in error for error in errors),
        errors[0] if errors else "none")

    # A carve-out whose path no longer exists: caught.
    errors = check_family_patterns(
        _manifest([_family("orig", "original", ["src/**"],
                           carve_outs=[{"glob": "src/gone.cpp", "handled_by": "other"}]),
                   _family("other", "bundled", ["thirdparty/**"])]),
        ["src/engine.cpp"])
    add("a carve-out path that no longer exists is caught",
        any("carve-out" in error for error in errors), errors[0] if errors else "none")

    # A family pattern that matches nothing: caught.
    errors = check_family_patterns(_manifest([_family("dead", "original", ["widgets/**"])]),
                                   ["src/engine.cpp"])
    add("a family pattern that matches nothing is caught",
        any("matches no tracked file" in error for error in errors),
        errors[0] if errors else "none")

    # An install prefix missing a mapped text, or holding a stale one.
    prefix = _tree({"share/rigExec/LICENSE": MIT_TEXT,
                    "share/rigExec/NOTICE": "stale notice\n"})
    manifest = _manifest([ORIGINAL], {"cmake_install": {"legal_files": [
        {"source": "LICENSE", "destination": "share/rigExec/LICENSE"},
        {"source": "NOTICE", "destination": "share/rigExec/NOTICE"},
        {"source": "thirdparty/flatbuffers/LICENSE",
         "destination": "share/rigExec/thirdparty/flatbuffers/LICENSE"}]}})
    errors, _, stats = check_install_prefix(manifest, prefix, sources)
    add("an install prefix missing a mapped text is caught",
        any("not in the installed prefix" in error for error in errors)
        and stats["mapped"] == 3 and stats["present"] == 2,
        f"{stats} / {errors}")
    add("a stale installed text is caught",
        any("stale" in error for error in errors), errors[0] if errors else "none")

    # A DESTDIR-staged or relocated prefix is still recognized.
    staged = _tree({"build/usr/local/share/rigExec/LICENSE": MIT_TEXT,
                    "build/usr/local/share/rigExec/NOTICE": NOTICE_TEXT})
    errors, _, stats = check_install_prefix(
        _manifest([ORIGINAL], {"cmake_install": {"legal_files": [
            {"source": "LICENSE", "destination": "share/rigExec/LICENSE"},
            {"source": "NOTICE", "destination": "share/rigExec/NOTICE"}]}}),
        staged, _tree(ORIGINAL_TREE))
    add("a staged install prefix is audited", not errors and stats["present"] == 2,
        f"{stats} / {errors}")

    # A RENAME names the file the install writes; it is not itself a source.
    renamed = install_paths('install(FILES thirdparty/flatbuffers/LICENSE\n'
                            '    DESTINATION "share/rigExec/licenses"\n'
                            '    RENAME FlatBuffers-LICENSE.txt)\n')
    add("a rename is a destination, not an installed source",
        renamed == {"thirdparty/flatbuffers/LICENSE": [("share/rigExec/licenses",
                                                       "FlatBuffers-LICENSE.txt")]},
        str(renamed))

    # A notice link in the shipped archive that reaches nothing: caught.
    archive_path = Path(tempfile.mktemp(suffix="tutorial.zip"))
    notice = ("see [the licence](LICENSE) and "
              "[upstream](thirdparty/x/LICENSE)\n")
    _zip(archive_path, {"LICENSE": MIT_TEXT, "THIRD_PARTY_NOTICES.md": notice})
    mapped = [{"member": "LICENSE", "source": "LICENSE", "status": "current"},
              {"member": "THIRD_PARTY_NOTICES.md", "source": "THIRD_PARTY_NOTICES.md",
               "status": "current"}]
    manifest = _manifest([ORIGINAL], {"tutorial": {"archive": str(archive_path),
                                                  "legal_members": mapped}})
    notice_sources = _tree(ORIGINAL_TREE | {"THIRD_PARTY_NOTICES.md": notice})
    errors, stats, notes = check_archive(manifest, archive_path, notice_sources)
    add("a broken notice link inside an archive is caught",
        any("not a member of the archive" in error for error in errors)
        and stats["legal_present"] == 2, f"{stats} / {errors}")

    # A mapped legal member that is not in the archive: caught.
    _zip(archive_path, {"LICENSE": MIT_TEXT})
    errors, stats, notes = check_archive(manifest, archive_path, notice_sources)
    add("a mapped legal member missing from the archive is caught",
        any("not in the archive" in error for error in errors), f"{stats} / {errors}")

    # An archive member carrying a license text the manifest never mapped: caught.
    _zip(archive_path, {"LICENSE": MIT_TEXT,
                        "COPYRIGHT.txt": "some upstream notice\n"})
    errors, stats, notes = check_archive(manifest, archive_path, notice_sources)
    add("an unmapped license member in an archive is caught",
        any("does not map" in error for error in errors), f"{stats} / {errors}")

    # A mirrored copy of a mapped text is a note.
    clean_notice = "see [the licence](LICENSE)\n"
    _zip(archive_path, {"LICENSE": MIT_TEXT, "demo/LICENSE": MIT_TEXT,
                        "THIRD_PARTY_NOTICES.md": clean_notice})
    clean_sources = _tree(ORIGINAL_TREE | {"THIRD_PARTY_NOTICES.md": clean_notice})
    errors, stats, notes = check_archive(manifest, archive_path, clean_sources)
    add("a mirrored copy of a mapped text is reported as a note",
        not errors and stats["mirrored_legal"] == 1
        and any("mirror" in note for note in notes), f"{stats} / {errors or notes}")

    # The manifest copy shipped inside an archive is a snapshot: its bytes may
    # differ, its shape may not.
    mapping_manifest = _manifest([ORIGINAL])
    snapshot = dict(mapping_manifest, status="an older inventory baseline")
    archive_copy = _zip(Path(tempfile.mktemp(suffix="bundle.zip")),
                        {"LICENSE": MIT_TEXT,
                         "tools/license_manifest.json": json.dumps(snapshot)})
    mapping_manifest["distribution"]["bundle"] = {
        "archive": str(archive_copy),
        "legal_members": [{"member": "LICENSE", "source": "LICENSE", "status": "current"},
                          {"member": "tools/license_manifest.json",
                           "source": "tools/license_manifest.json",
                           "status": "provenance snapshot, never byte-compared"}]}
    errors, stats, notes = check_archive(mapping_manifest, archive_copy, sources)
    add("a manifest snapshot in an archive is not byte-compared against itself",
        not errors, f"{stats} / {errors}")
    _zip(archive_copy, {"LICENSE": MIT_TEXT,
                        "tools/license_manifest.json": json.dumps({
                            "schema_version": 0, "families": [ORIGINAL]})})
    errors, stats, notes = check_archive(mapping_manifest, archive_copy, sources)
    add("a manifest snapshot with the wrong shape is caught",
        any("schema_version" in error or "families" in error for error in errors),
        f"{stats} / {errors}")

    # Without git, partition completeness is reported unverifiable, not passed.
    notes = validate(_manifest([ORIGINAL]), _tree(ORIGINAL_TREE), partition=True)[1]
    add("without git the partition is reported unverifiable",
        any("unverifiable" in note for note in notes), notes[0] if notes else "none")

    # Add-on bound sources are verified against the add-on tree when a root is
    # supplied, and reported as unverifiable when it is not.
    addon = Path(tempfile.mkdtemp()) / "addon"
    addon.mkdir()
    addon_license = addon / "LICENSE"
    addon_license.write_text(MIT_TEXT, encoding="utf-8")
    _zip(archive_path, {"LICENSE": MIT_TEXT})
    addon_manifest = _manifest([ORIGINAL], {"addon": {
        "archive": str(archive_path),
        "legal_members": [{"member": "LICENSE",
                           "source": "../addon/LICENSE",
                           "status": "byte-identical after CRLF normalization"}]}})
    errors, stats, notes = check_archive(addon_manifest, archive_path, sources, addon)
    add("an add-on bound source is verified against the add-on root",
        not errors, f"{stats} / {errors}")
    addon_license.write_text("MIT but edited\n", encoding="utf-8")
    errors = compare_legal_members(addon_manifest, archive_path.name,
                                   addon_manifest["distribution"]["addon"]["legal_members"],
                                   ["LICENSE"], archive_path, addon, sources)
    add("a stale add-on bound source is caught", bool(errors), errors or "no errors")
    errors, stats, notes = check_archive(addon_manifest, archive_path, sources)
    source_errors, source_notes = check_sources_exist(addon_manifest, sources, None)
    add("without an add-on root the bound source is unverifiable, not a failure",
        not errors and not source_errors
        and any("add-on bound" in note for note in source_notes),
        f"{errors or source_errors} / {source_notes}")

    # The inventory boundary: staged dependency and build output under the
    # checkout root stay out; a new file in a tracked directory stays in.
    boundary = inventory_result_tree(
        ["LICENSE", "src/engine.cpp", "docs/guide.md"],
        ["build/CMakeCache.txt", "build/CMakeFiles/rules.ninja",
         "OpenUSD/pxr/base/tf/tf.h", "OpenUSD/pxr/usd/usd/stage.h",
         "usd-install/lib/libusd.so", "usd-install/include/pxr/base/tf.h",
         "docs/new_guide.md", "scratch.md"])
    add("the inventory keeps project files and refuses staged dependency output",
        boundary == ["LICENSE", "docs/guide.md", "docs/new_guide.md",
                     "scratch.md", "src/engine.cpp"],
        str(boundary))

    # A source tree reached through a path the platform rewrites on resolve,
    # which is how a Windows temp tree can be spelled, must still be compared.
    linked = _linked_tree(ORIGINAL_TREE)
    stale_errors = compare_legal_members(fixture_manifest, stale_archive.name,
                                         entries, ["LICENSE"], stale_archive, None,
                                         linked)
    add("a stale member is caught when the source tree is reached by a link",
        any("differs from its source" in error for error in stale_errors),
        stale_errors or "none")

    # Containment is checked after resolving both the authorized root and the
    # joined source: a link must not carry a mapped source outside the tree.
    try:
        guarded = _guarded_tree(Path(tempfile.mkdtemp(prefix="license-check-guard-")))
    except OSError as exc:
        add("a link may not carry a source out of its authorized root", None,
            f"this host cannot create links: {exc}")
    else:
        guard_archive = _zip(Path(tempfile.mktemp(suffix="guard.zip")),
                             {"LICENSE": "MIT but edited\n"})
        escaping = compare_legal_members(
            fixture_manifest, guard_archive.name,
            [{"member": "LICENSE", "source": "ESCAPE", "status": "current"}],
            ["LICENSE"], guard_archive, None, guarded)
        add("a link may not carry a source out of its authorized root",
            any("escapes this checkout" in error for error in escaping),
            escaping or "none")
        linked_target = compare_legal_members(
            fixture_manifest, guard_archive.name,
            [{"member": "LICENSE", "source": "licenses/INNER", "status": "current"}],
            ["LICENSE"], guard_archive, None, guarded)
        add("a link that stays inside the root is read as its target",
            any("differs from its source" in error for error in linked_target),
            linked_target or "none")

    # A mapped source that cannot be resolved cannot vouch for archived bytes.
    prose_archive = _zip(Path(tempfile.mktemp(suffix="prose.zip")),
                         {"LICENSE": "MIT but edited\n"})
    errors = compare_legal_members(fixture_manifest, prose_archive.name,
                                   [{"member": "LICENSE", "source": "addon copy",
                                     "status": "current"}],
                                   ["LICENSE"], prose_archive, None, sources)
    add("a legal member whose source will not resolve is reported",
        any("LICENSE" in error for error in errors), errors or "none")

    # A notice that travels under an archive root has its links checked there.
    nested_path = Path(tempfile.mktemp(suffix="nested.zip"))
    nested_notice = "see [the licence](LICENSE) and [methods](docs/references.md)\n"
    _zip(nested_path, {"godot_rigExec/THIRD_PARTY_NOTICES.md": nested_notice,
                       "godot_rigExec/LICENSE": MIT_TEXT})
    nested_entries = [{"member": "godot_rigExec/LICENSE", "source": "LICENSE",
                       "status": "current"},
                      {"member": "godot_rigExec/THIRD_PARTY_NOTICES.md",
                       "source": "THIRD_PARTY_NOTICES.md", "status": "current"}]
    nested_manifest = _manifest([ORIGINAL], {"addon": {
        "archive": str(nested_path), "legal_members": nested_entries}})
    nested_manifest["prose_references"] = [{"path": "docs/references.md",
                                            "kind": "source-only prose reference"}]
    nested_sources = _tree(ORIGINAL_TREE | {"THIRD_PARTY_NOTICES.md": nested_notice})
    errors, stats, notes = check_archive(nested_manifest, nested_path, nested_sources)
    add("a nested archive notice resolves its links under its own directory",
        not errors and any("prose" in note for note in notes),
        f"{stats} / {errors or notes}")
    broken_notice = "see [the licence](LICENSE) and [upstream](thirdparty/x/LICENSE)\n"
    broken_sources = _tree(ORIGINAL_TREE | {"THIRD_PARTY_NOTICES.md": broken_notice})
    _zip(nested_path, {"godot_rigExec/THIRD_PARTY_NOTICES.md": broken_notice,
                       "godot_rigExec/LICENSE": MIT_TEXT})
    errors, stats, notes = check_archive(nested_manifest, nested_path, broken_sources)
    add("a broken link inside a nested archive notice is caught",
        any("thirdparty/x/LICENSE" in error for error in errors),
        f"{stats} / {errors or notes}")

    return cases


def run_self_test() -> int:
    cases = _cases()
    failures = [(name, detail) for name, passed, detail in cases if passed is False]
    skipped = [name for name, passed, _ in cases if passed is None]
    for name, passed, detail in cases:
        if passed is None:
            print(f"  skip  {name}\n        {detail}")
            continue
        print(f"  {'ok   ' if passed else 'FAIL'}  {name}")
        if not passed:
            print(f"        got: {detail}")
    ran = len(cases) - len(skipped)
    print(f"self-test: {ran - len(failures)}/{ran} cases behaved "
          "as expected" + (f", {len(skipped)} skipped" if skipped else ""))
    return 1 if failures else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
    parser.add_argument("--install", type=Path, metavar="PREFIX",
                        help="audit an install prefix instead of the checkout")
    parser.add_argument("--archive", type=Path, metavar="ARCHIVE",
                        help="audit one archive against its mapping")
    parser.add_argument("--addon-root", type=Path,
                        help="root of the add-on repository that owns add-on-bound "
                        "legal sources")
    parser.add_argument("--self-test", action="store_true",
                        help="run the synthetic fixtures against temp trees")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()
    manifest = load_manifest(args.manifest)

    if args.install:
        errors, notes, stats = check_install_prefix(manifest, args.install, ROOT)
        if not args.quiet:
            print(f"install prefix {args.install}: legal root {stats.get('legal_root')}, "
                  f"{stats.get('present', 0)}/{stats.get('mapped', 0)} mapped legal "
                  f"files present, {stats.get('stale', 0)} stale, "
                  f"{stats.get('links_resolved', 0)} notice links resolved")
    elif args.archive:
        errors, stats, notes = check_archive(manifest, args.archive, ROOT,
                                            args.addon_root)
        if not args.quiet and stats:
            print(f"archive {args.archive}: {stats['members']} members, "
                  f"{stats['mapped_legal']} legal members mapped, "
                  f"{stats['legal_present']} present, "
                  f"{stats['mirrored_legal']} mirrored copies, "
                  f"{stats['payload_members']} payload members")
    else:
        errors, notes = validate(manifest, ROOT, args.addon_root)
        if not args.quiet:
            files, outside = inventory(ROOT)
            if files is None:
                notes.append("partition completeness is unverifiable: this tree has "
                            "no git, so the family counts were not re-derived")
            else:
                print(f"partition re-derived over {len(files)} files")
                if outside:
                    notes.append(
                        f"{len(outside)} untracked paths under untracked directories "
                        "are outside the inventory (a staged dependency, a build "
                        "tree or a nested checkout in this checkout)")
    if not args.quiet:
        for note in notes:
            print(f"note: {note}")

    if errors:
        for error in errors:
            print(f"error: {error}")
        return 1
    if not args.quiet:
        print("license manifest is consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
