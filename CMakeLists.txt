# Generated from qtshadertools.pro.

cmake_minimum_required(VERSION 3.15.0)

include(.cmake.conf)
project(QtShaderTools
    VERSION "${QT_REPO_MODULE_VERSION}"
    DESCRIPTION "Qt ShaderTools Libraries"
    HOMEPAGE_URL "https://qt.io/"
    LANGUAGES CXX C
)

# special case begin

# Make sure we only use latest private CMake API, aka no compatibility wrappers.
set(QT_NO_INTERNAL_COMPATIBILITY_FUNCTIONS TRUE)

# Make sure we use the fixed BASE argument of qt_add_resource.
set(QT_USE_FIXED_QT_ADD_RESOURCE_BASE TRUE)

find_package(Qt6 ${PROJECT_VERSION} CONFIG REQUIRED COMPONENTS BuildInternals Core)
find_package(Qt6 ${PROJECT_VERSION} QUIET CONFIG OPTIONAL_COMPONENTS Gui)
# special case end

if(INTEGRITY)
add_compile_definitions(__inline=inline)
endif()

if(NOT TARGET Qt::Gui)
    message(NOTICE "Skipping the build as the condition \"TARGET Qt::Gui\" is not met.")
    return()
endif()
if(VXWORKS OR WATCHOS)
    message(NOTICE "Skipping the build as the condition \"NOT VXWORKS AND NOT WATCHOS\" is not met.")
    return()
endif()

# Ensure macros are available when configuring tests in-tree with the Qt build
include(tools/qsb/Qt6ShaderToolsMacros.cmake)

qt_build_repo()
