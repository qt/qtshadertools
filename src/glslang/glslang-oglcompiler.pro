TARGET = qtglslang-oglcompiler

CONFIG += \
    static \
    hide_symbols \
    exceptions_off rtti_off warn_off

load(qt_helper_lib)

include($$PWD/glslang_common.pri)

SOURCES += \
    $$GLSLANG_PATH/OGLCompilersDLL/InitializeDll.cpp
