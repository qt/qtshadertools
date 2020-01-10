/****************************************************************************
**
** Copyright (C) 2019 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Shader Tools module
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "qspirvshaderremap_p.h"

#include <SPIRV/SPVRemapper.h>

QT_BEGIN_NAMESPACE

void QSpirvShaderRemapper::remapErrorHandler(const std::string &s)
{
    if (!remapErrorMsg.isEmpty())
        remapErrorMsg.append(QLatin1Char('\n'));
    remapErrorMsg.append(QString::fromStdString(s));
}

void QSpirvShaderRemapper::remapLogHandler(const std::string &)
{
}

QByteArray QSpirvShaderRemapper::remap(const QByteArray &ir, QSpirvShader::RemapFlags flags)
{
    if (ir.isEmpty())
        return QByteArray();

    remapErrorMsg.clear();

    spv::spirvbin_t b;
    b.registerErrorHandler(std::bind(&QSpirvShaderRemapper::remapErrorHandler, this, std::placeholders::_1));
    b.registerLogHandler(std::bind(&QSpirvShaderRemapper::remapLogHandler, this, std::placeholders::_1));

    const uint32_t opts = flags.testFlag(QSpirvShader::StripOnly) ? spv::spirvbin_t::STRIP
                                                                  : spv::spirvbin_t::DO_EVERYTHING;

    std::vector<uint32_t> v;
    v.resize(ir.size() / 4);
    memcpy(v.data(), ir.constData(), v.size() * 4);

    b.remap(v, opts);

    if (!remapErrorMsg.isEmpty())
        return QByteArray();

    return QByteArray(reinterpret_cast<const char *>(v.data()), int(v.size()) * 4);
}

QT_END_NAMESPACE
