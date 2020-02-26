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

#include "qshaderbaker.h"
#include "qspirvcompiler_p.h"
#include "qspirvshader_p.h"
#include <QFileInfo>
#include <QFile>
#include <QDebug>

QT_BEGIN_NAMESPACE

/*!
    \class QShaderBaker
    \inmodule QtShaderTools

    \brief Compiles a GLSL/Vulkan shader into SPIR-V, translates into other
    shading languages, and gathers reflection metadata.

    QShaderBaker takes a graphics (vertex, fragment, etc.) or compute shader,
    and produces multiple - either source or bytecode - variants of it,
    together with reflection information. The results are represented by a
    QShader instance, which also provides simple and fast serialization
    and deserialization.

    \note Applications and libraries are recommended to avoid using this class
    directly. Rather, all Qt users are encouraged to rely on offline
    compilation by invoking the \c qsb command-line tool at build time. This
    tool uses QShaderBaker itself and writes the serialized version of the
    generated QShader into a file. The usage of this class should be
    restricted to cases where run time compilation cannot be avoided, such as
    when working with user-provided shader source strings.

    The input format is always assumed to be Vulkan-flavored GLSL at the
    moment. See the
    \l{https://github.com/KhronosGroup/GLSL/blob/master/extensions/khr/GL_KHR_vulkan_glsl.txt}{GL_KHR_vulkan_glsl
    specification} for an overview, keeping in mind that the Qt Shader Tools
    module is meant to be used in combination with the QRhi classes from Qt
    Rendering Hardware Interface module, and therefore a number of concepts and
    constructs (push constants, storage buffers, subpasses, etc.) are not
    applicable at the moment. Additional options may be introduced in the
    future, for example, by enabling
    \l{https://docs.microsoft.com/en-us/windows/desktop/direct3dhlsl/dx-graphics-hlsl}{HLSL}
    as a source format, once HLSL to SPIR-V compilation is deemed suitable.

    The reflection metadata is retrievable from the resulting QShader by
    calling QShader::description(). This is essential when having to
    discover what set of vertex inputs and shader resources a shader expects,
    and what the layouts of those are, as many modern graphics APIs offer no
    built-in shader reflection capabilities.

    \section2 Typical Workflow

    Let's assume an application has a vertex and fragment shader like the following:

    Vertex shader:
    \snippet color.vert 0

    Fragment shader:
    \snippet color.frag 0

    To get QShader instances that can be passed as-is to a
    QRhiGraphicsPipeline, there are two options: doing the shader pack
    generation off line, or at run time.

    The former involves running the \c qsb tool:

    \badcode
    qsb --glsl "100 es,120" --hlsl 50 --msl 12 color.vert -o color.vert.qsb
    qsb --glsl "100 es,120" --hlsl 50 --msl 12 color.frag -o color.frag.qsb
    \endcode

    The example uses the translation targets as appropriate for QRhi. This
    means GLSL/ES 100, GLSL 120, HLSL Shader Model 5.0, and Metal Shading
    Language 1.2.

    Note how the command line options correspond to what can be specified via
    setGeneratedShaders(). Once the resulting files are available, they can be
    shipped with the application (typically embedded into the executable the
    the Qt Resource System), and can be loaded and passed to
    QShader::fromSerialized() at run time.

    While not shown here, \c qsb can do more: it is also able to invoke \c fxc
    on Windows or the appropriate XCode tools on macOS to compile the generated
    HLSL or Metal shader code into bytecode and include the compiled versions
    in the QShader. After a baked shader pack is written into a file, its
    contents can be examined by running \c{qsb -d} on it. Run \c qsb with
    \c{--help} for more information.

    The alternative approach is to perform the same at run time. This involves
    creating a QShaderBaker instance, calling setSourceFileName(), and then
    setting up the translation targets via setGeneratedShaders():

    \badcode
        baker.setGeneratedShaderVariants({ QShader::StandardShader });
        QVector<QShaderBaker::GeneratedShader> targets;
        targets.append({ QShader::SpirvShader, QShaderVersion(100) });
        targets.append({ QShader::GlslShader, QShaderVersion(100, QShaderVersion::GlslEs) });
        targets.append({ QShader::SpirvShader, QShaderVersion(120) });
        targets.append({ QShader::HlslShader, QShaderVersion(50) });
        targets.append({ QShader::MslShader, QShaderVersion(12) });
        baker.setGeneratedShaders(targets);
        QShader shaders = baker.bake();
        if (!shaders.isValid())
            qWarning() << baker.errorMessage();
    \endcode

    \sa QShader
 */

struct QShaderBakerPrivate
{
    bool readFile(const QString &fn);

    QString sourceFileName;
    QByteArray source;
    QShader::Stage stage;
    QVector<QShaderBaker::GeneratedShader> reqVersions;
    QVector<QShader::Variant> variants;
    QByteArray preamble;
    int batchLoc = 7;
    QSpirvCompiler compiler;
    QString errorMessage;
};

bool QShaderBakerPrivate::readFile(const QString &fn)
{
    QFile f(fn);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning("QShaderBaker: Failed to open %s", qPrintable(fn));
        return false;
    }
    source = f.readAll();
    sourceFileName = fn;
    return true;
}

/*!
    Constructs a new QShaderBaker.
 */
QShaderBaker::QShaderBaker()
    : d(new QShaderBakerPrivate)
{
}

/*!
    Destructor.
 */
QShaderBaker::~QShaderBaker()
{
    delete d;
}

/*!
    Sets the name of the shader source file to \a fileName. This is the file
    that will be read when calling bake(). The shader stage is deduced
    automatically from the file extension. When this is not desired or not
    possible, use the overload with the stage argument instead.

    The supported file extensions are:
    \list
    \li \c{.vert} - vertex shader
    \li \c{.frag} - fragment (pixel) shader
    \li \c{.tesc} - tessellation control (hull) shader
    \li \c{.tese} - tessellation evaluation (domain) shader
    \li \c{.geom} - geometry shader
    \li \c{.comp} - compute shader
    \endlist
 */
void QShaderBaker::setSourceFileName(const QString &fileName)
{
    if (!d->readFile(fileName))
        return;

    const QString suffix = QFileInfo(fileName).suffix();
    if (suffix == QStringLiteral("vert")) {
        d->stage = QShader::VertexStage;
    } else if (suffix == QStringLiteral("frag")) {
        d->stage = QShader::FragmentStage;
    } else if (suffix == QStringLiteral("tesc")) {
        d->stage = QShader::TessellationControlStage;
    } else if (suffix == QStringLiteral("tese")) {
        d->stage = QShader::TessellationEvaluationStage;
    } else if (suffix == QStringLiteral("geom")) {
        d->stage = QShader::GeometryStage;
    } else if (suffix == QStringLiteral("comp")) {
        d->stage = QShader::ComputeStage;
    } else {
        qWarning("QShaderBaker: Unknown shader stage, defaulting to vertex");
        d->stage = QShader::VertexStage;
    }
}

/*!
    Sets the name of the shader source file to \a fileName. This is the file
    that will be read when calling bake(). The shader stage is specified by \a
    stage.
 */
void QShaderBaker::setSourceFileName(const QString &fileName, QShader::Stage stage)
{
    if (d->readFile(fileName))
        d->stage = stage;
}

/*!
    Sets the source \a device. This allows using any QIODevice instead of just
    files. \a stage specifies the shader stage, while the optional \a fileName
    contains a filename that is used in the error messages.
 */
void QShaderBaker::setSourceDevice(QIODevice *device, QShader::Stage stage, const QString &fileName)
{
    setSourceString(device->readAll(), stage, fileName);
}

/*!
    Sets the input shader \a sourceString. \a stage specified the shader stage,
    while the optional \a fileName contains a filename that is used in the
    error messages.
 */
void QShaderBaker::setSourceString(const QByteArray &sourceString, QShader::Stage stage, const QString &fileName)
{
    d->sourceFileName = fileName; // for error messages, include handling, etc.
    d->source = sourceString;
    d->stage = stage;
}

/*!
    \typedef QShaderBaker::GeneratedShader

    Synonym for QPair<QShader::Source, QShaderVersion>.
*/

/*!
    Specifies what kind of shaders to compile or translate to. Nothing is
    generated by default so calling this function before bake() is mandatory

    \note when this function is not called or \a v is empty or contains only invalid
    entries, the resulting QShader will be empty and thus invalid.

    For example, the minimal possible baking target is SPIR-V, without any
    additional translations to other languages. To request this, do:

    \badcode
        baker.setGeneratedShaders({ QShader::SpirvShader, QShaderVersion(100) });
    \endcode
 */
void QShaderBaker::setGeneratedShaders(const QVector<GeneratedShader> &v)
{
    d->reqVersions = v;
}

/*!
    Specifies which shader variants are generated. Each shader version can have
    multiple variants in the resulting QShader.

    In most cases \a v contains a single entry, QShader::StandardShader.

    \note when no variants are set, the resulting QShader will be empty and
    thus invalid.
 */
void QShaderBaker::setGeneratedShaderVariants(const QVector<QShader::Variant> &v)
{
    d->variants = v;
}

/*!
    Specifies a custom \a preamble that is processed before the normal shader
    code.

    This is more than just prepending to the source string: the validity of the
    GLSL version directive, which is required to be placed before everything
    else, is not affected. Line numbers in the reported error messages also
    remain unchanged, ignoring the contents given in the \a preamble.

    One use case for preambles is to transparently insert dynamically generated
    \c{#define} statements.
 */
void QShaderBaker::setPreamble(const QByteArray &preamble)
{
    d->preamble = preamble;
}

/*!
    When generating a QShader::BatchableVertexShader variant, \a location
    specifies the input location for the inserted vertex input. The value is by
    default 7 and needs to be overridden only if the vertex shader already uses
    input location 7.
 */
void QShaderBaker::setBatchableVertexShaderExtraInputLocation(int location)
{
    d->batchLoc = location;
}

/*!
    Runs the compilation and translation process.

    \return a QShader instance. To check if the process was successful,
    call QShader::isValid(). When that indicates \c false, call
    errorMessage() to retrieve the log.

    This is an expensive operation. When calling this from applications, it can
    be advisable to do it on a separate thread.

    \note QShaderBaker instances are reusable: after calling bake(), the same
    instance can be used with different inputs again. However, a QShaderBaker
    instance should only be used on one single thread during its lifetime.
 */
QShader QShaderBaker::bake()
{
    d->errorMessage.clear();

    if (d->source.isEmpty()) {
        d->errorMessage = QLatin1String("QShaderBaker: No source specified");
        return QShader();
    }

    d->compiler.setSourceString(d->source, d->stage, d->sourceFileName);
    d->compiler.setFlags({});
    d->compiler.setPreamble(d->preamble);
    QByteArray spirv = d->compiler.compileToSpirv();
    if (spirv.isEmpty()) {
        d->errorMessage = d->compiler.errorMessage();
        return QShader();
    }

    QByteArray batchableSpirv;
    if (d->stage == QShader::VertexStage && d->variants.contains(QShader::BatchableVertexShader)) {
        d->compiler.setFlags(QSpirvCompiler::RewriteToMakeBatchableForSG);
        d->compiler.setSGBatchingVertexInputLocation(d->batchLoc);
        batchableSpirv = d->compiler.compileToSpirv();
        if (batchableSpirv.isEmpty()) {
            d->errorMessage = d->compiler.errorMessage();
            return QShader();
        }
    }

    QShader bs;
    bs.setStage(d->stage);

    QSpirvShader spirvShader;
    spirvShader.setSpirvBinary(spirv);

    QSpirvShader batchableSpirvShader;
    if (!batchableSpirv.isEmpty()) {
        batchableSpirvShader.setSpirvBinary(batchableSpirv);
        bs.setDescription(batchableSpirvShader.shaderDescription());
    } else {
        bs.setDescription(spirvShader.shaderDescription());
    }

    for (const GeneratedShader &req: d->reqVersions) {
        for (const QShader::Variant &v : d->variants) {
            QByteArray *currentSpirv = &spirv;
            QSpirvShader *currentSpirvShader = &spirvShader;
            if (v == QShader::BatchableVertexShader) {
                if (!batchableSpirv.isEmpty()) {
                    currentSpirv = &batchableSpirv;
                    currentSpirvShader = &batchableSpirvShader;
                } else {
                    continue;
                }
            }
            const QShaderKey key(req.first, req.second, v);
            QShaderCode shader;
            shader.setEntryPoint(QByteArrayLiteral("main"));
            switch (req.first) {
            case QShader::SpirvShader:
                shader.setShader(*currentSpirv);
                break;
            case QShader::GlslShader:
            {
                QSpirvShader::GlslFlags flags;
                if (req.second.flags().testFlag(QShaderVersion::GlslEs))
                    flags |= QSpirvShader::GlslEs;
                shader.setShader(currentSpirvShader->translateToGLSL(req.second.version(), flags));
                if (shader.shader().isEmpty()) {
                    d->errorMessage = currentSpirvShader->translationErrorMessage();
                    return QShader();
                }
            }
                break;
            case QShader::HlslShader:
            {
                QShader::NativeResourceBindingMap nativeBindings;
                shader.setShader(currentSpirvShader->translateToHLSL(req.second.version(), &nativeBindings));
                if (shader.shader().isEmpty()) {
                    d->errorMessage = currentSpirvShader->translationErrorMessage();
                    return QShader();
                }
                bs.setResourceBindingMap(key, nativeBindings);
            }
                break;
            case QShader::MslShader:
            {
                QShader::NativeResourceBindingMap nativeBindings;
                shader.setShader(currentSpirvShader->translateToMSL(req.second.version(), &nativeBindings));
                if (shader.shader().isEmpty()) {
                    d->errorMessage = currentSpirvShader->translationErrorMessage();
                    return QShader();
                }
                shader.setEntryPoint(QByteArrayLiteral("main0"));
                bs.setResourceBindingMap(key, nativeBindings);
            }
                break;
            default:
                Q_UNREACHABLE();
            }
            bs.setShader(key, shader);
        }
    }

    return bs;
}

/*!
    \return the error message from the last bake() run, or an empty string if
    there was no error.

    \note Errors include file read errors, compilation, and translation
    failures. Not requesting any targets or variants does not count as an error
    even though the resulting QShader is invalid.
 */
QString QShaderBaker::errorMessage() const
{
    return d->errorMessage;
}

QT_END_NAMESPACE
