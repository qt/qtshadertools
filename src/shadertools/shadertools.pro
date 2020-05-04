TARGET = QtShaderTools

QT += gui-private

DEFINES += QT_BUILD_SHADERTOOLS_LIB

HEADERS += \
    $$PWD/qtshadertoolsglobal.h \
    $$PWD/qshaderbaker_p.h \
    $$PWD/qspirvshader_p.h \
    $$PWD/qspirvshaderremap_p.h \
    $$PWD/qspirvcompiler_p.h \
    $$PWD/qshaderbatchablerewriter_p.h

SOURCES += \
    $$PWD/qshaderbaker.cpp \
    $$PWD/qspirvshader.cpp \
    $$PWD/qspirvshaderremap.cpp \
    $$PWD/qspirvcompiler.cpp \
    $$PWD/qshaderbatchablerewriter.cpp

INCLUDEPATH += $$PWD/../3rdparty/SPIRV-Cross $$PWD/../3rdparty/glslang

# Exceptions must be enabled since that is the only sane way to get errors reported from SPIRV-Cross.
# They will not propagate outside of this module though so should be safe enough.
CONFIG += exceptions

!exists($$[QT_HOST_DATA]/.qmake.cache) {
    LIBLOC = $$shadowed($$dirname(_QMAKE_CONF_))/lib
} else {
    LIBLOC = $$[QT_HOST_LIBS]
}

STATICLIBS = qtspirv-cross qtglslang-glslang qtglslang-spirv qtglslang-osdependent qtglslang-oglcompiler # qtglslang-hlsl
for(libname, STATICLIBS) {
    staticlib = $$LIBLOC/$${QMAKE_PREFIX_STATICLIB}$$qtLibraryTarget($$libname).$${QMAKE_EXTENSION_STATICLIB}
    LIBS_PRIVATE += $$staticlib
    PRE_TARGETDEPS += $$staticlib
}

include($$PWD/doc/doc.pri)

load(qt_module)
