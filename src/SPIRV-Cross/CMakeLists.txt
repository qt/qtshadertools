# Generated from SPIRV-Cross.pro.

#####################################################################
## spirv_cross Generic Library:
#####################################################################

qt_add_3rdparty_library(spirv_cross
    STATIC
    EXCEPTIONS
    SOURCES
        ../3rdparty/SPIRV-Cross/spirv_cfg.cpp
        ../3rdparty/SPIRV-Cross/spirv_cpp.cpp
        ../3rdparty/SPIRV-Cross/spirv_cross.cpp
        ../3rdparty/SPIRV-Cross/spirv_cross_c.cpp
        ../3rdparty/SPIRV-Cross/spirv_cross_parsed_ir.cpp
        ../3rdparty/SPIRV-Cross/spirv_cross_util.cpp
        ../3rdparty/SPIRV-Cross/spirv_glsl.cpp
        ../3rdparty/SPIRV-Cross/spirv_hlsl.cpp
        ../3rdparty/SPIRV-Cross/spirv_msl.cpp
        ../3rdparty/SPIRV-Cross/spirv_parser.cpp
        ../3rdparty/SPIRV-Cross/spirv_reflect.cpp
    DEFINES
        SPIRV_CROSS_C_API_GLSL=1
        SPIRV_CROSS_C_API_HLSL=1
        SPIRV_CROSS_C_API_MSL=1
)
