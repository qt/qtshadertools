TARGET = qtspirv-cross

# Exceptions must be enabled since that is the only sane way to get errors reported.
# They will not propagate outside of the shadertools module though so should be safe enough.

CONFIG += \
    static \
    hide_symbols \
    warn_off \
    exceptions

load(qt_helper_lib)

SPIRVCROSS_PATH=$$PWD/../3rdparty/SPIRV-Cross

DEFINES += SPIRV_CROSS_C_API_GLSL=1 SPIRV_CROSS_C_API_HLSL=1 SPIRV_CROSS_C_API_MSL=1

SOURCES += \
    $$SPIRVCROSS_PATH/spirv_cfg.cpp \
    $$SPIRVCROSS_PATH/spirv_cpp.cpp \
    $$SPIRVCROSS_PATH/spirv_cross.cpp \
    $$SPIRVCROSS_PATH/spirv_cross_c.cpp \
    $$SPIRVCROSS_PATH/spirv_cross_parsed_ir.cpp \
    $$SPIRVCROSS_PATH/spirv_cross_util.cpp \
    $$SPIRVCROSS_PATH/spirv_glsl.cpp \
    $$SPIRVCROSS_PATH/spirv_hlsl.cpp \
    $$SPIRVCROSS_PATH/spirv_msl.cpp \
    $$SPIRVCROSS_PATH/spirv_parser.cpp \
    $$SPIRVCROSS_PATH/spirv_reflect.cpp
