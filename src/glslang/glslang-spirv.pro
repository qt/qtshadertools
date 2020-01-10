TARGET = qtglslang-spirv

CONFIG += \
    static \
    hide_symbols \
    exceptions_off rtti_off warn_off

load(qt_helper_lib)

include($$PWD/glslang_common.pri)

SOURCES += \
    $$GLSLANG_PATH/SPIRV/GlslangToSpv.cpp \
    $$GLSLANG_PATH/SPIRV/InReadableOrder.cpp \
    $$GLSLANG_PATH/SPIRV/Logger.cpp \
    $$GLSLANG_PATH/SPIRV/SpvBuilder.cpp \
    $$GLSLANG_PATH/SPIRV/SpvPostProcess.cpp \
    $$GLSLANG_PATH/SPIRV/doc.cpp \
    $$GLSLANG_PATH/SPIRV/disassemble.cpp \
    $$GLSLANG_PATH/SPIRV/SPVRemapper.cpp \
    $$GLSLANG_PATH/SPIRV/SPVTools.cpp
