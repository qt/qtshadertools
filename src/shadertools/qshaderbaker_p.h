// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#ifndef QSHADERBAKER_P_H
#define QSHADERBAKER_P_H

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

QT_BEGIN_NAMESPACE

struct QShaderBakerPrivate;
class QIODevice;

class Q_SHADERTOOLS_PRIVATE_EXPORT QShaderBaker
{
public:
    enum class SpirvOption {
        GenerateFullDebugInfo = 0x01,
        StripDebugAndVarInfo = 0x02
    };
    Q_DECLARE_FLAGS(SpirvOptions, SpirvOption)

    QShaderBaker();
    ~QShaderBaker();

    void setSourceFileName(const QString &fileName);
    void setSourceFileName(const QString &fileName, QShader::Stage stage);

    void setSourceDevice(QIODevice *device, QShader::Stage stage,
                         const QString &fileName = QString());

    void setSourceString(const QByteArray &sourceString, QShader::Stage stage,
                         const QString &fileName = QString());

    typedef QPair<QShader::Source, QShaderVersion> GeneratedShader;
    void setGeneratedShaders(const QList<GeneratedShader> &v);
    void setGeneratedShaderVariants(const QList<QShader::Variant> &v);

    void setPreamble(const QByteArray &preamble);
    void setBatchableVertexShaderExtraInputLocation(int location);
    void setPerTargetCompilation(bool enable);
    void setBreakOnShaderTranslationError(bool enable);
    void setTessellationMode(QShaderDescription::TessellationMode mode);
    void setTessellationOutputVertexCount(int count);

    void setSpirvOptions(SpirvOptions options);

    QShader bake();

    QString errorMessage() const;

private:
    Q_DISABLE_COPY(QShaderBaker)
    QShaderBakerPrivate *d = nullptr;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QShaderBaker::SpirvOptions)

QT_END_NAMESPACE

#endif
