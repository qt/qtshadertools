TARGET = qtglslang-glslang

CONFIG += \
    static \
    hide_symbols \
    exceptions_off rtti_off warn_off

load(qt_helper_lib)

include($$PWD/glslang_common.pri)

SOURCES += \
    $$GLSLANG_PATH/glslang/MachineIndependent/glslang_tab.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/attribute.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/Constant.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/iomapper.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/InfoSink.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/Initialize.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/IntermTraverse.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/Intermediate.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/ParseContextBase.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/ParseHelper.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/PoolAlloc.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/RemoveTree.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/Scan.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/ShaderLang.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/SymbolTable.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/Versions.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/intermOut.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/limits.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/linkValidate.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/parseConst.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/pch.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/reflection.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/preprocessor/Pp.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/preprocessor/PpAtom.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/preprocessor/PpContext.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/preprocessor/PpScanner.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/preprocessor/PpTokens.cpp \
    $$GLSLANG_PATH/glslang/MachineIndependent/propagateNoContraction.cpp \
    $$GLSLANG_PATH/glslang/GenericCodeGen/CodeGen.cpp \
    $$GLSLANG_PATH/glslang/GenericCodeGen/Link.cpp
