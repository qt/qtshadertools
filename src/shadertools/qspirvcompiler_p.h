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

#ifndef QSPIRVCOMPILER_P_H
#define QSPIRVCOMPILER_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists for the convenience
// of a number of Qt sources files.  This header file may change from
// version to version without notice, or even be removed.
//
// We mean it.
//

#include <QtShaderTools/private/qtshadertoolsglobal_p.h>
#include <QtGui/private/qshader_p.h>
#include <QtCore/QString>

QT_BEGIN_NAMESPACE

struct QSpirvCompilerPrivate;
class QIODevice;

class Q_SHADERTOOLS_PRIVATE_EXPORT QSpirvCompiler
{
public:
    QSpirvCompiler();
    ~QSpirvCompiler();

    enum Flag {
        RewriteToMakeBatchableForSG = 0x01,
        FullDebugInfo = 0x02
    };
    Q_DECLARE_FLAGS(Flags, Flag)

    void setSourceFileName(const QString &fileName);
    void setSourceFileName(const QString &fileName, QShader::Stage stage);
    void setSourceDevice(QIODevice *device, QShader::Stage stage, const QString &fileName = QString());
    void setSourceString(const QByteArray &sourceString, QShader::Stage stage, const QString &fileName = QString());
    void setFlags(Flags flags);
    void setPreamble(const QByteArray &preamble);
    void setSGBatchingVertexInputLocation(int location);

    QByteArray compileToSpirv();
    QString errorMessage() const;

private:
    Q_DISABLE_COPY(QSpirvCompiler)
    QSpirvCompilerPrivate *d = nullptr;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QSpirvCompiler::Flags)

QT_END_NAMESPACE

#endif
