"""MkDocs hooks for the RigExec documentation site (see mkdocs.yml).

The Markdown under docs/ is written to read well on GitHub and is partly
generated (docs/build_pages.py), so the site adapts to it rather than the
other way round:

  * the Node reference navigation is built from docs/operator_notes.py's
    CATEGORIES, so a new operator appears without editing mkdocs.yml;
  * the repository's icons/ folder, which the generated pages reference
    as ../icons/ or ../../icons/, is published under assets/icons/ and
    those links are rewritten to it;
  * links to files outside docs/ (examples, sources, notices), and to
    pages mkdocs.yml leaves out of the site, point at the repository on
    GitHub, since the site does not carry them;
  * docs/home.md is the landing page and the generated catalog,
    docs/index.md, is published at /reference/.
"""
import os
import posixpath
import re
import sys

from mkdocs.structure.files import File

_DOCS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_DOCS)
_ICONS = os.path.join(_REPO, "icons")
_GITHUB = "https://github.com/usdCharacter/usdRig"
_BRANCH = "main"

# A Markdown link or image target: ](target) or ](target "title").
_LINK = re.compile(r"(\]\()([^)\s]+)((?:\s+\"[^\"]*\")?\))")


def on_config(config, **kwargs):
    """Fill the Node reference section from the generator's categories."""
    sys.path.insert(0, _DOCS)
    try:
        from operator_notes import CATEGORIES, OPERATORS
    finally:
        sys.path.remove(_DOCS)
    for entry in config.nav or []:
        if not isinstance(entry, dict) or "Node reference" not in entry:
            continue
        section = entry["Node reference"]
        for category, keys in CATEGORIES:
            pages = []
            for key in keys:
                path = "nodes/%s.md" % key
                if os.path.isfile(os.path.join(_DOCS, path)):
                    title = OPERATORS.get(key, {}).get("title", key)
                    pages.append({title: path})
            if pages:
                section.append({category: pages})
    return config


def on_files(files, config, **kwargs):
    """Publish icons/ and put the landing page and catalog in place."""
    if os.path.isdir(_ICONS):
        for name in sorted(os.listdir(_ICONS)):
            if name.startswith(".") or not os.path.isfile(
                    os.path.join(_ICONS, name)):
                continue
            files.append(File.generated(
                config, posixpath.join("assets", "icons", name),
                abs_src_path=os.path.join(_ICONS, name)))
    for f in files:
        if f.src_uri == "home.md":
            f.dest_uri = "index.html"
            f.url = "./" if config.use_directory_urls else "index.html"
        elif f.src_uri == "index.md":
            if config.use_directory_urls:
                f.dest_uri = "reference/index.html"
                f.url = "reference/"
            else:
                f.dest_uri = "reference.html"
                f.url = "reference.html"
    return files


def _Rewrite(target, page_dir, files=None):
    """The site's target for a link written for the repository."""
    if re.match(r"^[a-z][a-z0-9+.-]*:", target) or target.startswith(
            ("#", "/")):
        return target
    path, _, anchor = target.partition("#")
    if not path:
        return target
    repo_path = posixpath.normpath(posixpath.join("docs", page_dir, path))
    if repo_path.startswith("icons/"):
        rel = posixpath.relpath(
            posixpath.join("assets", "icons", repo_path[len("icons/"):]),
            page_dir or ".")
        return rel + ("#" + anchor if anchor else "")
    if repo_path.startswith("docs/") or repo_path == "docs":
        # A folder inside docs/ with no page of its own is shown on GitHub.
        full = os.path.join(_REPO, *repo_path.split("/"))
        if os.path.isdir(full) and not os.path.isfile(
                os.path.join(full, "index.md")):
            return "%s/tree/%s/%s" % (_GITHUB, _BRANCH, repo_path.rstrip("/"))
        # A page excluded from the site (exclude_docs) is shown on GitHub.
        page = (files.get_file_from_path(repo_path[len("docs/"):])
                if files is not None and repo_path.endswith(".md") else None)
        if (page is not None and getattr(page, "inclusion", None) is not None
                and page.inclusion.is_excluded()):
            url = "%s/blob/%s/%s" % (_GITHUB, _BRANCH, repo_path)
            return url + ("#" + anchor if anchor else "")
        return target
    if repo_path.startswith("../"):
        return target
    full = os.path.join(_REPO, *repo_path.split("/"))
    kind = "tree" if os.path.isdir(full) else "blob"
    url = "%s/%s/%s/%s" % (_GITHUB, kind, _BRANCH, repo_path)
    return url + ("#" + anchor if anchor else "")


def on_page_markdown(markdown, page, config, files, **kwargs):
    page_dir = posixpath.dirname(page.file.src_uri)
    out = []
    in_fence = False
    for line in markdown.split("\n"):
        if line.lstrip().startswith(("```", "~~~")):
            in_fence = not in_fence
        if not in_fence and "](" in line:
            line = _LINK.sub(
                lambda m: m.group(1) + _Rewrite(m.group(2), page_dir, files)
                + m.group(3), line)
        out.append(line)
    return "\n".join(out)
