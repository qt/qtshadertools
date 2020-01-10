TARGET = qtglslang-osdependent

CONFIG += \
    static \
    hide_symbols \
    exceptions_off rtti_off warn_off

load(qt_helper_lib)

include($$PWD/glslang_common.pri)

win32: GLSLANG_OSDEP_PATH=$$GLSLANG_PATH/glslang/OSDependent/Windows
unix: GLSLANG_OSDEP_PATH=$$GLSLANG_PATH/glslang/OSDependent/Unix
linux:!android: LIBS += -lpthread

SOURCES += \
    $$GLSLANG_OSDEP_PATH/ossource.cpp
