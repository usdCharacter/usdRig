"""Legal archive members for the tutorial packages, read from the manifest.

tools/license_manifest.json names every legal text that ships in the install
prefix and in each tutorial archive, together with the member name the
consumer reads. Both packers take their legal members from that mapping
instead of a list of their own.
"""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tools" / "license_manifest.json"


def normalize_line_endings(data: bytes) -> bytes:
    """Collapse CRLF and CR to LF.

    The archives were built on Windows, so line endings are the only
    difference allowed between a member and the text it came from.
    """
    return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def archive_member_name(name: str) -> str:
    """Validate an archive member name.

    Names come from the manifest, so reject absolute paths, drive prefixes,
    backslashes, empty segments and '.' or '..'.
    """
    if not name or "\\" in name or name.startswith("/") or ":" in name:
        raise ValueError(f"unsafe archive member name {name!r}")
    if any(part in ("", ".", "..") for part in name.split("/")):
        raise ValueError(f"unsafe archive member name {name!r}")
    return name


def resolve_legal_source(member: str, source, addon_root: Path | None = None,
                         root: Path | None = None) -> Path:
    """Resolve one manifest source to a file, or refuse to guess it.

    ``root`` is the checkout the source is read from; it defaults to this
    repository. The license checker passes the tree it is auditing.

    A source is either a path inside this checkout, or a path inside the
    add-on checkout supplied explicitly as addon_root, written as
    '../<add-on-name>/...'. Anything else is prose: the manifest then names
    no file, and no bytes may be invented for it. No segment may be '.' or
    '..', and an add-on source may never leave the root that was passed in.
    The authorized root and the joined source are both resolved before
    containment is checked: a root reached through a link is authorized, and a
    link that leaves that root is refused.
    """
    if not isinstance(source, str) or not source.strip():
        raise ValueError(f"{member}: the manifest names no source file; "
                         f"refusing to invent a legal text")
    source = source.strip()
    if source.startswith(("http://", "https://")):
        raise ValueError(f"{member}: source {source!r} is a URL, not a file to pack")
    if "\\" in source or ":" in source or " " in source:
        raise ValueError(f"{member}: source {source!r} is prose provenance, not a path")
    parts = source.split("/")
    if parts[0] == "..":
        if addon_root is None:
            raise ValueError(f"{member}: source {source!r} needs an explicit add-on root")
        requested = Path(addon_root)
        root = requested.resolve()
        if len(parts) < 2 or parts[1] not in (requested.name, root.name):
            raise ValueError(f"{member}: source {source!r} is not inside the "
                            f"authorized add-on root {root}")
        if len(parts) < 3:
            raise ValueError(f"{member}: source {source!r} names the add-on root, "
                            "not a legal text")
        if any(part in ("", ".", "..") for part in parts[2:]):
            raise ValueError(f"{member}: unsafe add-on source path {source!r}")
        path = root.joinpath(*parts[2:]).resolve()
        if not path.is_relative_to(root):
            raise ValueError(f"{member}: source {source!r} escapes the add-on root {root}")
        return path
    if any(part in ("", ".", "..") for part in parts):
        raise ValueError(f"{member}: unsafe source path {source!r}")
    base = (ROOT if root is None else Path(root)).resolve()
    path = (base / source).resolve()
    if not path.is_relative_to(base):
        raise ValueError(f"{member}: source {source!r} escapes this checkout")
    return path


def legal_members(section: str, manifest: dict | None = None,
                  addon_root: Path | None = None) -> list[tuple[str, Path, dict]]:
    """Return [(member name, file to pack, manifest entry)] for one channel."""
    if manifest is None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    entries = manifest["distribution"][section]["legal_members"]
    return [(archive_member_name(entry.get("member")),
            resolve_legal_source(entry.get("member", ""), entry.get("source"), addon_root),
            entry)
            for entry in entries]


def missing_sources(members: list[tuple[str, Path, dict]]) -> list[tuple[str, Path]]:
    """Legal members whose mapped source text is not on disk."""
    return [(name, path) for name, path, _ in members if not path.is_file()]


def member_bytes(previous: bytes | None, source: Path) -> bytes:
    """Bytes to store for an archive member.

    If the archive already holds this member and its content matches the
    source apart from line endings, keep the archived bytes: a rebuild on a
    different platform must not churn line endings in a file nobody edited.
    A member whose content really changed is written from its source.
    """
    raw = source.read_bytes()
    if previous is not None:
        if normalize_line_endings(previous) == normalize_line_endings(raw):
            return previous
    return raw


def legal_member_status(previous: bytes | None, source: Path) -> str:
    """How the member already in the archive compares with its source text."""
    if previous is None:
        return "new member"
    current = source.read_bytes()
    if previous == current:
        return "current"
    if normalize_line_endings(previous) == normalize_line_endings(current):
        return "line endings differ"
    return "refreshed (archived copy was stale)"
