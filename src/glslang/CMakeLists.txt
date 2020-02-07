# Generated from glslang.pro.

#####################################################################
## glslang_spirv Generic Library:
#####################################################################

qt_add_3rdparty_library(glslang_spirv
    STATIC
    SOURCES
        ../3rdparty/glslang/SPIRV/GlslangToSpv.cpp
        ../3rdparty/glslang/SPIRV/InReadableOrder.cpp
        ../3rdparty/glslang/SPIRV/Logger.cpp
        ../3rdparty/glslang/SPIRV/SPVRemapper.cpp
        ../3rdparty/glslang/SPIRV/SpvBuilder.cpp
        ../3rdparty/glslang/SPIRV/SpvPostProcess.cpp
        ../3rdparty/glslang/SPIRV/SpvTools.cpp
        ../3rdparty/glslang/SPIRV/disassemble.cpp
        ../3rdparty/glslang/SPIRV/doc.cpp
    # PUBLIC_LIBRARIES # special case
        # Remove: core
        # Remove: gui
)

## Scopes:
#####################################################################

qt_extend_target(glslang_spirv CONDITION WIN32
    DEFINES
        GLSLANG_OSINCLUDE_WIN32
)

qt_extend_target(glslang_spirv CONDITION UNIX
    DEFINES
        GLSLANG_OSINCLUDE_UNIX
)
#####################################################################
## glslang_osdependent Generic Library:
#####################################################################

# special case begin
if(WIN32)
    set(GLSLANG_OSDEP_PATH ../3rdparty/glslang/glslang/OSDependent/Windows)
elseif(UNIX)
    set(GLSLANG_OSDEP_PATH ../3rdparty/glslang/glslang/OSDependent/Unix)
endif()
# special case end

qt_add_3rdparty_library(glslang_osdependent
    STATIC
    SOURCES
        ${GLSLANG_OSDEP_PATH}/ossource.cpp # special case
    # PUBLIC_LIBRARIES # special case
        # Remove: core
        # Remove: gui
)

## Scopes:
#####################################################################

qt_extend_target(glslang_osdependent CONDITION WIN32
    DEFINES
        GLSLANG_OSINCLUDE_WIN32
)

#### Keys ignored in scope 7:.:.:glslang-osdependent.pro:WIN32:
# GLSLANG_OSDEP_PATH = "$$GLSLANG_PATH/glslang/OSDependent/Windows"

qt_extend_target(glslang_osdependent CONDITION UNIX
    DEFINES
        GLSLANG_OSINCLUDE_UNIX
)

#### Keys ignored in scope 8:.:.:glslang-osdependent.pro:UNIX:
# GLSLANG_OSDEP_PATH = "$$GLSLANG_PATH/glslang/OSDependent/Unix"

qt_extend_target(glslang_osdependent CONDITION LINUX
    PUBLIC_LIBRARIES
        pthread
)
#####################################################################
## glslang_oglcompiler Generic Library:
#####################################################################

qt_add_3rdparty_library(glslang_oglcompiler
    STATIC
    SOURCES
        ../3rdparty/glslang/OGLCompilersDLL/InitializeDll.cpp
    # PUBLIC_LIBRARIES # special case
        # Remove: core
        # Remove: gui
)

## Scopes:
#####################################################################

qt_extend_target(glslang_oglcompiler CONDITION WIN32
    DEFINES
        GLSLANG_OSINCLUDE_WIN32
)

qt_extend_target(glslang_oglcompiler CONDITION UNIX
    DEFINES
        GLSLANG_OSINCLUDE_UNIX
)
#####################################################################
## glslang_glslang Generic Library:
#####################################################################

qt_add_3rdparty_library(glslang_glslang
    STATIC
    SOURCES
        ../3rdparty/glslang/glslang/GenericCodeGen/CodeGen.cpp
        ../3rdparty/glslang/glslang/GenericCodeGen/Link.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/Constant.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/InfoSink.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/Initialize.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/IntermTraverse.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/Intermediate.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/ParseContextBase.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/ParseHelper.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/PoolAlloc.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/RemoveTree.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/Scan.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/ShaderLang.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/SymbolTable.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/Versions.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/attribute.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/glslang_tab.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/intermOut.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/iomapper.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/limits.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/linkValidate.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/parseConst.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/pch.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/preprocessor/Pp.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/preprocessor/PpAtom.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/preprocessor/PpContext.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/preprocessor/PpScanner.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/preprocessor/PpTokens.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/propagateNoContraction.cpp
        ../3rdparty/glslang/glslang/MachineIndependent/reflection.cpp
    # PUBLIC_LIBRARIES # special case
        # Remove: core
        # Remove: gui
)

## Scopes:
#####################################################################

qt_extend_target(glslang_glslang CONDITION WIN32
    DEFINES
        GLSLANG_OSINCLUDE_WIN32
)

qt_extend_target(glslang_glslang CONDITION UNIX
    DEFINES
        GLSLANG_OSINCLUDE_UNIX
)
