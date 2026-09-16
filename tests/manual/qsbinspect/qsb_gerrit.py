# Copyright (C) 2026 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

"""Finding the .qsb files a Gerrit change adds or changes.

Gerrit's public REST API is enough, and it is read straight over HTTP: the change
is never fetched into the local repository, so nothing is cherry-picked, applied,
or left behind afterwards. That also means the change need not belong to the
repository qsb_inspect was launched from - or to any repository, since the packs
arrive as bytes and the reader wants nothing else.

No authentication, so this reaches public changes only; a private or WIP one
answers 404 like a change that does not exist. Only the latest patch set is
looked at.

Standard library only, like qsb_git, whose ChangedFile this reuses so that the
change view can show either kind of change without caring which it has.
"""

from __future__ import annotations

import base64
import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field

from qsb_git import ChangedFile

# Qt's, since that is what this is for; QSBINSPECT_GERRIT_URL points it elsewhere.
# Deliberately not derived from the local repository's remote: a change is fetched
# over HTTP and has nothing to do with wherever qsb_inspect happens to be running.
DEFAULT_URL = "https://codereview.qt-project.org"
URL_ENV = "QSBINSPECT_GERRIT_URL"

TIMEOUT = 30

# the change number in a URL Gerrit hands out: ".../c/qt/qtbase/+/738611" and
# ".../+/738611/6" alike, the trailing patch set being ignored
_URL_TAIL = re.compile(r"/(?:\+/)?(\d+)(?:/\d+)?$")


class GerritError(Exception):
    pass


def base_url() -> str:
    return (os.environ.get(URL_ENV) or DEFAULT_URL).rstrip("/")


# -- the REST calls ---------------------------------------------------------

def _get(path: str) -> bytes:
    url = base_url() + path
    req = urllib.request.Request(url, headers={"User-Agent": "qsbinspect"})
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        if e.code == 404:
            raise GerritError(
                f"not found on {base_url()} - no such change, or not one that "
                f"can be read without signing in") from e
        raise GerritError(f"{url}: HTTP {e.code} {e.reason}") from e
    except urllib.error.URLError as e:
        raise GerritError(f"could not reach {base_url()}: {e.reason}") from e
    except OSError as e:  # a timeout arrives here rather than as a URLError
        raise GerritError(f"could not reach {base_url()}: {e}") from e


def _json(path: str):
    raw = _get(path)
    # Gerrit prefixes its JSON with a line of punctuation to spoil cross-site
    # inclusion; it is not part of the document
    if raw.startswith(b")]}'"):
        _, _, raw = raw.partition(b"\n")
    try:
        return json.loads(raw)
    except ValueError as e:
        raise GerritError(f"unexpected reply from {base_url()}: {e}") from e


# -- what the change view needs --------------------------------------------

@dataclass(frozen=True)
class GerritChange:
    """One patch set of one change, and the .qsb files it touches."""

    number: int
    patch_set: int
    revision: str        # the commit this patch set is, used to pin every request
    project: str
    branch: str
    subject: str
    files: list[ChangedFile] = field(default_factory=list)

    @property
    def short(self) -> str:
        return f"{self.number}/{self.patch_set}"

    @property
    def origin(self) -> str:
        """How the pack's provenance reads on the Overview tab."""
        return f"Gerrit {self.number} patch set {self.patch_set}"

    @property
    def title(self) -> str:
        where = " ".join(bit for bit in (self.project, self.branch) if bit)
        return f"{self.short}  {where}  {self.subject}"

    def blob(self, path: str) -> bytes:
        """The file as this patch set leaves it."""
        return file_content(self.number, self.revision, path)


def parse_change_ref(arg: str) -> int | None:
    """The change number an argument names, or None if it names no change.

    A bare number, or one of the URL forms Gerrit hands out - including the old
    fragment style and a scheme-less paste. Any patch set on the end is ignored,
    the latest one always being used.
    """
    s = arg.strip().rstrip("/")
    if s.isdigit():
        return int(s)
    if "://" not in s and "/+/" not in s and "/c/" not in s:
        # not URL-shaped; a branch like "feature/12345" is not a change number
        return None
    sp = urllib.parse.urlsplit(s if "://" in s else "//" + s)
    # the old UI put the change in the fragment: https://host/#/c/738611/2
    m = _URL_TAIL.search((sp.path + sp.fragment).rstrip("/"))
    return int(m.group(1)) if m else None


def changed_qsb_files(number: int, revision: str) -> list[ChangedFile]:
    """The .qsb files a patch set adds or changes.

    Deletions are dropped, as for a local commit, so every row can be opened.
    Gerrit leaves 'status' out entirely for a plain modification, hence the
    default, and reports a rename at its new path with old_path alongside.
    """
    listing = _json(f"/changes/{number}/revisions/{revision}/files/")
    if not isinstance(listing, dict):
        raise GerritError("the file listing was not an object")
    files: list[ChangedFile] = []
    for path in sorted(listing):
        # /COMMIT_MSG and /MERGE_LIST are Gerrit's own pseudo-files
        if path.startswith("/") or not path.lower().endswith(".qsb"):
            continue
        info = listing[path] or {}
        status = info.get("status", "M")
        if status == "D":
            continue
        files.append(ChangedFile(status, path, info.get("old_path", "")))
    return files


def describe_change(number: int) -> GerritChange:
    """Everything the change view needs, for the latest patch set.

    The revision is resolved once and every later request is pinned to it, so a
    patch set uploaded while the UI is open cannot change what is being shown
    half way through - the same reasoning as reading a local commit's own blobs.
    """
    info = _json(f"/changes/{number}?o=CURRENT_REVISION")
    revision = info.get("current_revision")
    if not revision:
        raise GerritError("the reply names no current revision")
    meta = info.get("revisions", {}).get(revision, {})
    return GerritChange(
        number=number,
        patch_set=meta.get("_number", 0),
        revision=revision,
        project=info.get("project", ""),
        branch=info.get("branch", ""),
        subject=info.get("subject", ""),
        files=changed_qsb_files(number, revision),
    )


def file_content(number: int, revision: str, path: str) -> bytes:
    """One file's contents at a revision, which Gerrit sends base64-encoded."""
    fid = urllib.parse.quote(path, safe="")
    raw = _get(f"/changes/{number}/revisions/{revision}/files/{fid}/content")
    try:
        return base64.b64decode(raw)
    except ValueError as e:
        raise GerritError(f"{path}: the reply was not valid base64: {e}") from e
