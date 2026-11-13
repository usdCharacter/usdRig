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


def _is_checkout_path(source: str) -> bool:
    return bool(source) and " " not in source and not source.startswith("/") and ":" not in source


def legal_members(section: str, checkout: Path | None = None,
                  prefix: str = "") -> list[tuple[str, Path, dict]]:
    """Return [(member name, file to pack, manifest entry)] for one channel.

    A source naming a file in this checkout resolves here. A source naming the
    sibling checkout ('../name/...') resolves there. A prose source means the
    member is a copy inside that checkout at the member path: the add-on
    carries the repository LICENSE wherever it names a LICENSE, and its demo
    folder mirrors the add-on folder.
    """
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    entries = manifest["distribution"][section]["legal_members"]
    members = []
    for entry in entries:
        name = archive_member_name(entry["member"])
        source = (entry.get("source") or "").strip()
        if source.startswith("../"):
            if checkout is None:
                raise ValueError(f"{name}: source {source!r} needs the sibling checkout")
            # '..' plus the checkout directory name; the rest is under it.
            path = checkout.joinpath(*Path(source).parts[2:])
        elif _is_checkout_path(source):
            path = ROOT / source
        else:
            if checkout is None:
                raise ValueError(f"{name}: source {source!r} needs the checkout it came from")
            relative = name[len(prefix):] if name.startswith(prefix) else name
            if relative.startswith("demo/"):
                relative = relative[len("demo/"):]
            path = checkout / "LICENSE" if Path(relative).name == "LICENSE" else checkout / relative
        members.append((name, path, entry))
    return members


def member_bytes(previous: bytes | None, source: Path) -> bytes:
    """Bytes to store for an archive member.

    If the archive already holds this member and its content matches the
    source apart from line endings, keep the archived bytes: a rebuild on a
    different platform must not churn line endings in a file nobody edited.
    A member whose content really changed is written from its source.
    """
    raw = source.read_bytes()
    if previous is not None and normalize_line_endings(previous) == normalize_line_endings(raw):
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
