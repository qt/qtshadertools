// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#ifndef QSPIRVSHADER_P_H
#define QSPIRVSHADER_P_H

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

class QIODevice;
struct QSpirvShaderPrivate;

class Q_SHADERTOOLS_PRIVATE_EXPORT QSpirvShader
{
public:
    enum GlslFlag {
        GlslEs = 0x01,
        FixClipSpace = 0x02,
        FragDefaultMediump = 0x04
    };
    Q_DECLARE_FLAGS(GlslFlags, GlslFlag)

    enum RemapFlag {
        StripOnly = 0x01
    };
    Q_DECLARE_FLAGS(RemapFlags, RemapFlag)

    QSpirvShader();
    ~QSpirvShader();

    void setFileName(const QString &fileName);
    void setDevice(QIODevice *device);
    void setSpirvBinary(const QByteArray &spirv);

    QShaderDescription shaderDescription() const;

    QByteArray spirvBinary() const;
    QByteArray remappedSpirvBinary(RemapFlags flags = RemapFlags(), QString *errorMessage = nullptr) const;

    struct SeparateToCombinedImageSamplerMapping {
        QByteArray textureName;
        QByteArray samplerName;
        QByteArray combinedSamplerName;
    };

    QByteArray translateToGLSL(int version = 120,
                               GlslFlags flags = GlslFlags(),
                               QVector<SeparateToCombinedImageSamplerMapping> *separateToCombinedImageSamplerMappings = nullptr) const;
    QByteArray translateToHLSL(int version = 50,
                               QShader::NativeResourceBindingMap *nativeBindings = nullptr) const;
    QByteArray translateToMSL(int version = 12,
                              QShader::NativeResourceBindingMap *nativeBindings = nullptr) const;
    QString translationErrorMessage() const;

private:
    Q_DISABLE_COPY(QSpirvShader)
    QSpirvShaderPrivate *d = nullptr;
};

Q_DECLARE_OPERATORS_FOR_FLAGS(QSpirvShader::GlslFlags)
Q_DECLARE_OPERATORS_FOR_FLAGS(QSpirvShader::RemapFlags)

QT_END_NAMESPACE

#endif
