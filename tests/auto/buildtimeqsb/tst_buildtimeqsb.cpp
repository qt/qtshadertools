/****************************************************************************
**
** Copyright (C) 2021 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the test suite of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL-EXCEPT$
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
** General Public License version 3 as published by the Free Software
** Foundation with exceptions as appearing in the file LICENSE.GPL3-EXCEPT
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtTest/QtTest>
#include <QFile>
#include <QtGui/private/qshader_p.h>

class tst_BuildTimeQsb : public QObject
{
    Q_OBJECT

private slots:
    void defaultAddShaders();
    void customTargets();
    void withDefines();
    void replacements();
};

static QShader getShader(const QString &name)
{
    QFile f(name);
    if (f.open(QIODevice::ReadOnly))
        return QShader::fromSerialized(f.readAll());

    return QShader();
}

static const int defaultShaderCount = 6;
static const QShaderKey defaultShaderKeys[defaultShaderCount] = {
    QShaderKey(QShader::SpirvShader, QShaderVersion(100)),
    QShaderKey(QShader::HlslShader, QShaderVersion(50)),
    QShaderKey(QShader::MslShader, QShaderVersion(12)),
    QShaderKey(QShader::GlslShader, QShaderVersion(100, QShaderVersion::GlslEs)),
    QShaderKey(QShader::GlslShader, QShaderVersion(120)),
    QShaderKey(QShader::GlslShader, QShaderVersion(150)),
};

void tst_BuildTimeQsb::defaultAddShaders()
{
    QShader color_vert = getShader(QLatin1String(":/test/color.vert.qsb"));
    QVERIFY(color_vert.isValid());
    QCOMPARE(color_vert.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(color_vert.availableShaders().contains(defaultShaderKeys[i]));

    QShader color_frag = getShader(QLatin1String(":/test/color.frag.qsb"));
    QVERIFY(color_frag.isValid());
    QCOMPARE(color_frag.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(color_frag.availableShaders().contains(defaultShaderKeys[i]));

    QShader texture_vert = getShader(QLatin1String(":/some/prefix/texture.vert.qsb"));
    QVERIFY(texture_vert.isValid());
    QCOMPARE(texture_vert.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(texture_vert.availableShaders().contains(defaultShaderKeys[i]));

    QShader texture_frag = getShader(QLatin1String(":/some/prefix/texture.frag.qsb"));
    QVERIFY(texture_frag.isValid());
    QCOMPARE(texture_frag.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(texture_frag.availableShaders().contains(defaultShaderKeys[i]));
}

void tst_BuildTimeQsb::customTargets()
{
    QShader color_vert = getShader(QLatin1String(":/test/color_1.vert.qsb"));
    QVERIFY(color_vert.isValid());
    QCOMPARE(color_vert.availableShaders().count(), 3);
    QVERIFY(color_vert.availableShaders().contains(QShaderKey(QShader::SpirvShader, QShaderVersion(100))));
    QVERIFY(color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(300, QShaderVersion::GlslEs))));
    QVERIFY(color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(330))));

    QShader color_frag = getShader(QLatin1String(":/test/color_1.frag.qsb"));
    QVERIFY(color_frag.isValid());
    QCOMPARE(color_frag.availableShaders().count(), 3);
    QVERIFY(color_frag.availableShaders().contains(QShaderKey(QShader::SpirvShader, QShaderVersion(100))));
    QVERIFY(color_frag.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(300, QShaderVersion::GlslEs))));
    QVERIFY(color_frag.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(330))));

    QShader bat_color_vert = getShader(QLatin1String(":/test/color_1b.vert.qsb"));
    QVERIFY(bat_color_vert.isValid());
    QCOMPARE(bat_color_vert.availableShaders().count(), 6);
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::SpirvShader, QShaderVersion(100))));
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(300, QShaderVersion::GlslEs))));
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(330))));
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::SpirvShader, QShaderVersion(100), QShader::BatchableVertexShader)));
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(300, QShaderVersion::GlslEs), QShader::BatchableVertexShader)));
    QVERIFY(bat_color_vert.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(330), QShader::BatchableVertexShader)));

    QShader bat_color_frag = getShader(QLatin1String(":/test/color_1b.frag.qsb"));
    QVERIFY(bat_color_frag.isValid());
    QCOMPARE(bat_color_frag.availableShaders().count(), 3); // batchable applies to vertex shaders only
    QVERIFY(bat_color_frag.availableShaders().contains(QShaderKey(QShader::SpirvShader, QShaderVersion(100))));
    QVERIFY(bat_color_frag.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(300, QShaderVersion::GlslEs))));
    QVERIFY(bat_color_frag.availableShaders().contains(QShaderKey(QShader::GlslShader, QShaderVersion(330))));
}

void tst_BuildTimeQsb::withDefines()
{
    QShader s = getShader(QLatin1String(":/texture_def.frag.qsb"));
    QVERIFY(s.isValid());
    QCOMPARE(s.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(s.availableShaders().contains(defaultShaderKeys[i]));
}

void tst_BuildTimeQsb::replacements()
{
    QShader s = getShader(QLatin1String(":/test/color_repl.vert.qsb"));
    QVERIFY(s.isValid());
    QCOMPARE(s.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(s.availableShaders().contains(defaultShaderKeys[i]));

    QByteArray src = s.shader(QShaderKey(QShader::SpirvShader, QShaderVersion(100))).shader();
    QVERIFY(!src.isEmpty());
    QCOMPARE(src.left(21), QByteArrayLiteral("Not very valid SPIR-V"));

    src = s.shader(QShaderKey(QShader::GlslShader, QShaderVersion(100, QShaderVersion::GlslEs))).shader();
    QVERIFY(!src.isEmpty());
    QCOMPARE(src.left(7), QByteArrayLiteral("Test r1"));

    src = s.shader(QShaderKey(QShader::HlslShader, QShaderVersion(50))).shader();
    QVERIFY(!src.isEmpty());
    QCOMPARE(src.left(7), QByteArrayLiteral("Test r3"));

    s = getShader(QLatin1String(":/test/color_repl.frag.qsb"));
    QVERIFY(s.isValid());
    QCOMPARE(s.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(s.availableShaders().contains(defaultShaderKeys[i]));

    src = s.shader(QShaderKey(QShader::MslShader, QShaderVersion(12))).shader();
    QVERIFY(!src.isEmpty());
    QCOMPARE(src.left(7), QByteArrayLiteral("Test r4"));
}

#include <tst_buildtimeqsb.moc>
QTEST_MAIN(tst_BuildTimeQsb)
