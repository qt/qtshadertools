# Copyright (C) 2021 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

from conans import ConanFile
import re
from pathlib import Path


def _parse_qt_version_by_key(key: str) -> str:
    with open(Path(__file__).parent.resolve() / ".cmake.conf") as f:
        m = re.search(fr'{key} .*"(.*)"', f.read())
    return m.group(1) if m else ""


def _get_qt_minor_version() -> str:
    return ".".join(_parse_qt_version_by_key("QT_REPO_MODULE_VERSION").split(".")[:2])


class QtShadertools(ConanFile):
    name = "qtshadertools"
    license = "GPL-3.0+, Commercial Qt License Agreement"
    author = "The Qt Company <https://www.qt.io/contact-us>"
    url = "https://code.qt.io/cgit/qt/qtshadertools.git"
    description = (
        "The Qt Shader Tools module builds on the SPIR-V Open Source Ecosystem as "
        "described at the Khronos SPIR-V web site"
    )
    topics = "qt", "qt6", "qtshadertools", "SPIRV"
    settings = "os", "compiler", "arch", "build_type"
    # for referencing the version number and prerelease tag and dependencies info
    exports = ".cmake.conf", "dependencies.yaml"
    exports_sources = "*", "!conan*.*"
    python_requires = f"qt-conan-common/{_get_qt_minor_version()}@qt/everywhere"
    python_requires_extend = "qt-conan-common.QtLeafModule"
