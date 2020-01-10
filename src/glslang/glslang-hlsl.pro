TARGET = qtglslang-hlsl

CONFIG += \
    static \
    hide_symbols \
    exceptions_off rtti_off warn_off

load(qt_helper_lib)

include($$PWD/glslang_common.pri)

SOURCES += \
    $$GLSLANG_PATH/hlsl/hlslAttributes.cpp \
    $$GLSLANG_PATH/hlsl/hlslParseHelper.cpp \
    $$GLSLANG_PATH/hlsl/hlslScanContext.cpp \
    $$GLSLANG_PATH/hlsl/hlslOpMap.cpp \
    $$GLSLANG_PATH/hlsl/hlslTokenStream.cpp \
    $$GLSLANG_PATH/hlsl/hlslGrammar.cpp \
    $$GLSLANG_PATH/hlsl/hlslParseables.cpp
