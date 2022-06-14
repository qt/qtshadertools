// Copyright (C) 2021 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR GPL-3.0-only WITH Qt-GPL-exception-1.0

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
    // "shaders"
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

    // "shaders_in_subdir"
    QShader texture_vert = getShader(QLatin1String(":/some/prefix/subdir/texture.vert.qsb"));
    QVERIFY(texture_vert.isValid());
    QCOMPARE(texture_vert.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(texture_vert.availableShaders().contains(defaultShaderKeys[i]));

    QShader texture_frag = getShader(QLatin1String(":/some/prefix/subdir/texture.frag.qsb"));
    QVERIFY(texture_frag.isValid());
    QCOMPARE(texture_frag.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(texture_frag.availableShaders().contains(defaultShaderKeys[i]));

    // "shaders_in_subdir_with_outputs_as_alias"
    QShader alias_texture_vert = getShader(QLatin1String(":/some/prefix/alias_texture.vert.qsb"));
    QVERIFY(alias_texture_vert.isValid());
    QCOMPARE(alias_texture_vert.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(alias_texture_vert.availableShaders().contains(defaultShaderKeys[i]));

    QShader alias_texture_frag = getShader(QLatin1String(":/some/prefix/x/y/z/alias_texture.frag.qsb"));
    QVERIFY(alias_texture_frag.isValid());
    QCOMPARE(alias_texture_frag.availableShaders().count(), defaultShaderCount);
    for (int i = 0; i < defaultShaderCount; ++i)
        QVERIFY(alias_texture_frag.availableShaders().contains(defaultShaderKeys[i]));

    // "shaders_in_subdir_with_base"
    QShader texture2_vert = getShader(QLatin1String(":/base_test/texture2.vert.qsb"));
    QVERIFY(texture2_vert.isValid());
    QCOMPARE(texture2_vert.availableShaders().count(), defaultShaderCount * 2); // batchable
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
    QShader s = getShader(QLatin1String(":/subdir/test/texture_def.frag.qsb"));
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

    s = getShader(QLatin1String(":/test/x/color_repl.frag.qsb"));
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
