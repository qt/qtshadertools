# Copyright (C) 2026 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

"""Readable renderings of the binary shader payloads, by way of external tools.

Two modes. DISASM disassembles: SPIR-V through spirv-dis, DXBC through
fxc /dumpbin, DXIL through dxc -dumpbin and metallib through metal-objdump - the
same tools qsb itself invokes to produce the bytecode in the first place, only
pointed the other way. CROSS goes further and only for SPIR-V, decompiling the
module back to Vulkan GLSL with spirv-cross.

The metallib spec is the one nobody has been able to try: see the comment on it.

Nothing here is required: when a tool is missing the caller gets a ToolOutput
with 'error' set and can go on showing the hex dump.

Stdlib only, like qsb_reader: this module knows nothing about the UI.
"""

from __future__ import annotations

import functools
import os
import platform
import shutil
import subprocess
import tempfile
from dataclasses import dataclass

from qsb_reader import SOURCES

# Tools are small and the payloads are a few KB, so this only ever trips when
# something has gone wrong - a tool waiting on input it will never get, say.
TIMEOUT = 30

# Where to look beyond PATH, per tool. The Vulkan SDK announces itself with an
# environment variable; the Windows SDK has to be dug out of Program Files.
_VULKAN_SDK = "vulkan-sdk"
_WINDOWS_SDK = "windows-sdk"

# the modes, and what to call them in a heading
DISASM = "disasm"
CROSS = "cross"
MODE_LABELS = {DISASM: "disassembly", CROSS: "Vulkan GLSL"}


@dataclass(frozen=True)
class _Spec:
    """How to render one QShader::Source readable with one external tool."""

    exe: str
    args: tuple[str, ...]    # arguments before the input
    stdin: bool              # feed the payload on stdin rather than in a file
    suffix: str              # temporary file extension, when a file is needed
    lexer: str               # pygments lexer for the output
    search: tuple[str, ...]  # extra places to look for exe
    hint: str                # where to get it, for the not-found message
    # a wrapper that can say where exe lives when it is on no PATH of its own,
    # asked as '<launcher> --find <exe>'. Apple's toolchain works this way.
    launcher: tuple[str, ...] = ()


# spirv-dis and spirv-cross both take the module on stdin when the filename is
# "-", so SPIR-V needs no temporary file either way. --no-color because the output
# goes into a Rich renderable, not a terminal; --no-header because the shader
# details pane already decodes the SPIR-V header.
_SPECS: dict[str, dict[int, _Spec]] = {
    DISASM: {
        0: _Spec(exe="spirv-dis", args=("--no-color", "--no-header"), stdin=True,
                 suffix=".spv", lexer="text", search=(_VULKAN_SDK,),
                 hint="it ships with the Vulkan SDK"),
        3: _Spec(exe="fxc", args=("/nologo", "/dumpbin"), stdin=False,
                 suffix=".dxbc", lexer="text", search=(_WINDOWS_SDK,),
                 hint="it ships with the Windows SDK, and is Windows-only"),
        5: _Spec(exe="dxc", args=("-dumpbin",), stdin=False,
                 # DXIL is LLVM IR, and pygments knows how to colour that
                 suffix=".dxil", lexer="llvm",
                 search=(_VULKAN_SDK, _WINDOWS_SDK),
                 hint="it ships with both the Vulkan SDK and the Windows SDK"),
        # UNTESTED - there was no Mac to hand when this was written. The tool
        # lives inside the active Xcode rather than on the PATH, hence the xcrun
        # launcher, and a .metallib is a container of AIR (LLVM bitcode), so
        # metal-objdump is the tool that knows how to walk it. If the arguments
        # turn out to be wrong the pane shows its stderr verbatim, which is the
        # quickest way to find out; xcrun metal-nm is the other thing to try.
        # 'text' rather than 'llvm' for the lexer until someone has seen the
        # output and can say which it looks like.
        6: _Spec(exe="metal-objdump", args=("--disassemble",), stdin=False,
                 suffix=".metallib", lexer="text", search=(),
                 launcher=("xcrun",),
                 hint="it comes with Xcode, so this only works on a Mac"),
    },
    # SPIR-V only: decompiling DXBC or DXIL is nobody's business here, and the
    # GLSL a pack already carries was cross-compiled for OpenGL rather than
    # Vulkan, so this is not the same text under another name
    CROSS: {
        0: _Spec(exe="spirv-cross", args=("--vulkan-semantics",), stdin=True,
                 suffix=".spv", lexer="glsl", search=(_VULKAN_SDK,),
                 hint="it ships with the Vulkan SDK"),
    },
}


# one tool per mode and source, and no name used twice, so the executable
# identifies the spec on its own
_BY_EXE = {spec.exe: spec for mode in _SPECS.values() for spec in mode.values()}


@dataclass
class ToolOutput:
    """The result of one attempt: either text, or a reason there is none."""

    text: str = ""
    lexer: str = "text"
    tool: str = ""           # the executable actually run, once resolved
    error: str = ""          # empty when the text is good

    @property
    def ok(self) -> bool:
        return not self.error


def can_run(mode: str, source: int) -> bool:
    """Whether this mode has a tool for this QShader::Source at all.

    True regardless of whether the tool is installed - that is only discovered
    by trying, and the resulting message is worth showing.
    """
    return source in _SPECS.get(mode, {})


def tool_name(mode: str, source: int) -> str:
    spec = _SPECS.get(mode, {}).get(source)
    return spec.exe if spec else ""


# -- finding the tools ------------------------------------------------------

def _version_key(name: str) -> tuple[int, ...]:
    """Sort key for a Windows SDK directory name like '10.0.26100.0'."""
    parts = []
    for bit in name.split("."):
        parts.append(int(bit) if bit.isdigit() else 0)
    return tuple(parts)


def _windows_kit_dirs():
    """The Windows SDK bin directories, newest SDK and host architecture first."""
    if platform.machine().upper() in ("ARM64", "AARCH64"):
        arches = ("arm64", "x64", "x86")
    else:
        arches = ("x64", "x86", "arm64")
    roots = [os.environ.get("ProgramFiles(x86)"), os.environ.get("ProgramFiles")]
    for root in [r for r in roots if r]:
        base = os.path.join(root, "Windows Kits", "10", "bin")
        try:
            versions = [d for d in os.listdir(base) if d[:1].isdigit()]
        except OSError:
            continue
        for version in sorted(versions, key=_version_key, reverse=True):
            for arch in arches:
                yield os.path.join(base, version, arch)


def _search_dirs(spec: _Spec):
    for where in spec.search:
        if where == _VULKAN_SDK:
            sdk = os.environ.get("VULKAN_SDK")
            if sdk:
                yield os.path.join(sdk, "Bin")
        elif where == _WINDOWS_SDK:
            yield from _windows_kit_dirs()


def _via_launcher(spec: _Spec) -> str | None:
    """Ask a wrapper where its tool lives, for the ones on no PATH of their own.

    'xcrun --find metal-objdump' prints the absolute path inside whichever Xcode
    is selected, and fails if there is no such tool - which is also how a machine
    with no Xcode at all answers.
    """
    if not spec.launcher:
        return None
    launcher = shutil.which(spec.launcher[0])
    if launcher is None:
        return None
    try:
        proc = _run([launcher, *spec.launcher[1:], "--find", spec.exe], None)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    path = proc.stdout.decode("utf-8", errors="replace").strip()
    return path or None


def env_override(exe: str) -> str:
    """The environment variable that pins this tool to a given path."""
    return "QSBINSPECT_" + exe.upper().replace("-", "_")


@functools.lru_cache(maxsize=None)
def find_tool(exe: str) -> str | None:
    """Locate one of the tools: an override, then PATH, then the known SDKs."""
    pinned = os.environ.get(env_override(exe))
    if pinned:
        # an explicit path is not second-guessed: if it is wrong, running it says so
        return pinned
    found = shutil.which(exe)
    if found:
        return found
    spec = _BY_EXE.get(exe)
    if spec is None:
        return None
    for d in _search_dirs(spec):
        candidate = os.path.join(d, exe + (".exe" if os.name == "nt" else ""))
        if os.path.isfile(candidate):
            return candidate
    return _via_launcher(spec)


# -- running them -----------------------------------------------------------

def _run(argv: list[str], payload: bytes | None) -> subprocess.CompletedProcess:
    # no shell, no window: on Windows a console tool started from a TUI would
    # otherwise flash a window of its own
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    return subprocess.run(argv, input=payload, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=TIMEOUT,
                          creationflags=flags)


def _clean(raw: bytes) -> str:
    return raw.decode("utf-8", errors="replace").replace("\r\n", "\n").strip("\n")


def run(mode: str, source: int, payload: bytes) -> ToolOutput:
    """Render one payload readable, reporting failure rather than raising."""
    spec = _SPECS.get(mode, {}).get(source)
    if spec is None:
        name = SOURCES.get(source, str(source))
        label = MODE_LABELS.get(mode, mode)
        return ToolOutput(error=f"No {label} is wired up for {name} payloads.")
    if not payload:
        return ToolOutput(error="Nothing to work on: the payload is empty.")

    exe = find_tool(spec.exe)
    if exe is None:
        return ToolOutput(error=(
            f"{spec.exe} was not found on PATH, and none of the usual install "
            f"locations has it either - {spec.hint}.\n"
            f"Set {env_override(spec.exe)} to its full path to point this at a "
            f"copy elsewhere."))

    tmp = None
    try:
        if spec.stdin:
            argv = [exe, *spec.args, "-"]
            stdin_data = payload
        else:
            # fxc and dxc have no stdin mode, so the payload goes to a file
            fd, tmp = tempfile.mkstemp(prefix="qsbinspect-", suffix=spec.suffix)
            with os.fdopen(fd, "wb") as fh:
                fh.write(payload)
            argv = [exe, *spec.args, tmp]
            stdin_data = None
        try:
            proc = _run(argv, stdin_data)
        except subprocess.TimeoutExpired:
            return ToolOutput(tool=exe,
                              error=f"{spec.exe} did not finish within {TIMEOUT}s.")
        except OSError as e:
            return ToolOutput(tool=exe, error=f"Could not run {exe}: {e}")
    finally:
        if tmp:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    out, err = _clean(proc.stdout), _clean(proc.stderr)
    if proc.returncode != 0:
        detail = err or out or "no output"
        # Windows hands back the raw DWORD, so spirv-dis's -4 arrives as
        # 4294967292; show what the tool meant to say
        rc = proc.returncode
        if rc > 0x7FFFFFFF:
            rc -= 0x100000000
        return ToolOutput(tool=exe, error=f"{spec.exe} exited with {rc}:\n{detail}")
    # fxc puts warnings on stderr and still succeeds; some builds of these tools
    # write the listing there too, so stderr is the fallback rather than an error
    text = out or err
    if not text:
        return ToolOutput(tool=exe, error=f"{spec.exe} produced no output.")
    return ToolOutput(text=text, lexer=spec.lexer, tool=exe)
