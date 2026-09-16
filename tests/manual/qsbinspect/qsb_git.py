# Copyright (C) 2026 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

"""Finding the .qsb files a git commit adds or changes.

Nothing here needs the commit to be checked out: the packs are read from the
commit's own blobs, which is the only way to be sure of showing the after-patch
content. The file in the working tree matches only when no later commit touched
it, so it cannot be relied on.

Standard library only; git is invoked as a subprocess.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field


class GitError(Exception):
    pass


def _git(args: list[str], cwd: str, binary: bool = False):
    try:
        r = subprocess.run(["git", *args], cwd=cwd, capture_output=True)
    except OSError as e:
        raise GitError(f"could not run git: {e}") from e
    if r.returncode != 0:
        msg = (r.stderr or b"").decode("utf-8", "replace").strip()
        raise GitError(msg or f"git {' '.join(args)} exited {r.returncode}")
    return r.stdout if binary else r.stdout.decode("utf-8", "replace")


@dataclass(frozen=True)
class ChangedFile:
    """One .qsb file a commit added or changed."""

    status: str            # the raw letter(s) from --name-status: A, M, T, R056, ...
    path: str              # repo-relative; the destination for a rename or copy
    old_path: str = ""     # the source, for a rename or copy

    @property
    def label(self) -> str:
        return {
            "A": "added",
            "M": "modified",
            "T": "type changed",
            "R": "renamed",
            "C": "copied",
        }.get(self.status[:1], self.status)


@dataclass(frozen=True)
class Commit:
    sha: str
    short: str
    subject: str
    repo: str
    files: list[ChangedFile] = field(default_factory=list)

    # short, origin, title and blob() are what the change view uses, so that it
    # can show a local commit or a Gerrit change without knowing which it holds

    @property
    def origin(self) -> str:
        """How the pack's provenance reads on the Overview tab."""
        return f"commit {self.short}"

    @property
    def title(self) -> str:
        return f"{self.short}  {self.subject}"

    def blob(self, path: str) -> bytes:
        """The file as this commit leaves it."""
        return blob_at(self.sha, path, self.repo)


def resolve_revision(rev: str, cwd: str) -> str | None:
    """The full SHA a revision spec names, or None if it names no commit.

    Accepts anything git does - full and short SHAs, branches, tags, HEAD~3 -
    since rev-parse costs the same either way.
    """
    try:
        out = _git(["rev-parse", "--verify", "--quiet", f"{rev}^{{commit}}"], cwd)
    except GitError:
        return None  # not a revision, or cwd is not a repository
    return out.strip() or None


def changed_qsb_files(sha: str, cwd: str) -> list[ChangedFile]:
    """The .qsb files a commit adds or changes.

    Deletions are excluded (--diff-filter=d), so every entry can actually be
    opened. Renames and copies are reported at their destination path and read
    like any other change.

    A merge commit yields nothing: diff-tree reports no diff for one unless asked
    for a specific parent, and treating a merge as touching no packs is
    deliberate.
    """
    # --root so that a repository's very first commit is diffed against the
    # empty tree rather than reporting nothing; it changes nothing for the
    # ordinary case, and merges still report nothing either way
    out = _git(["diff-tree", "--root", "--no-commit-id", "--name-status", "-r",
                "-M", "--diff-filter=d", sha, "--", "*.qsb"], cwd)
    files: list[ChangedFile] = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) < 2:
            continue
        status = parts[0]
        # rename and copy rows carry both the old and the new path
        if status[:1] in ("R", "C") and len(parts) >= 3:
            files.append(ChangedFile(status, parts[2], parts[1]))
        else:
            files.append(ChangedFile(status, parts[1]))
    return files


def describe_commit(rev: str, cwd: str) -> Commit | None:
    """Everything the commit view needs, or None if rev is not a revision."""
    sha = resolve_revision(rev, cwd)
    if sha is None:
        return None
    return Commit(
        sha=sha,
        short=_git(["rev-parse", "--short", sha], cwd).strip(),
        subject=_git(["log", "-1", "--format=%s", sha], cwd).strip(),
        repo=_git(["rev-parse", "--show-toplevel"], cwd).strip(),
        files=changed_qsb_files(sha, cwd),
    )


def blob_at(sha: str, path: str, cwd: str) -> bytes:
    """The contents of a repo-relative path as of a commit."""
    return _git(["show", f"{sha}:{path}"], cwd, binary=True)
