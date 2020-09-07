# Invokes qsb on each file in FILES. Extensions must be .vert, .frag, or .comp.
# The resulting .qsb files are added as resources under PREFIX.
# target and resourcename are like for qt6_add_resources.
#
# By default generates SPIR-V, GLSL (100 es, 120, 150), HLSL (shader model 5.0) and MSL (Metal 1.2).
# Configuring qsb:
#     Specify GLSL, HLSL, MSL to override the versions to generate.
#         Note: follows qsb and GLSL-style version syntax, e.g. "300 es,330".
#     Specify NOGLSL, NOHLSL, or NOMSL to skip generating a given language.
#         SPIR-V is always generated.
#     Specify PRECOMPILE to trigger invoking native tools where applicable.
#         F.ex. with HLSL enabled it passes -c to qsb which in turn runs fxc to store DXBC instead of HLSL.
#     Specify BATCHABLE to enable generating batchable vertex shader variants.
#         Mandatory for vertex shaders that are used with Qt Quick (2D) in materials or ShaderEffect.
#     Specify PERTARGETCOMPILE to compile to SPIR-V and translate separately per output language version.
#         Slow, but allows ifdefing based on QSHADER_<LANG>[_VERSION] macros.
#     Specify DEFINES with a "name1=value1;name2=value2" type of list to set custom macros for glslang.
#     Specify DEBUGINFO to enable generating full debug info where applicable (e.g. SPIR-V).
#     Specify OPTIMIZED to enable optimizing for performance where applicable.
#         For SPIR-V this involves invoking spirv-opt from SPIRV-Tools / the Vulkan SDK.
#
# NB! Most of this is documented in qtshadertools-build.qdoc. Changes without updating the documentation
# are not allowed.
#
# Example:
# qt6_add_shaders(testapp "testapp_shaders"
#    BATCHABLE
#    PRECOMPILE
#    PREFIX
#        "/shaders"
#    FILES
#        color.vert
#        color.frag
# )
# This leads to :/shaders/color.vert.qsb and :/shaders/color.frag.qsb being available in the application.
#
function(qt6_add_shaders target resourcename)
    cmake_parse_arguments(
        arg
        "BATCHABLE;PRECOMPILE;PERTARGETCOMPILE;NOGLSL;NOHLSL;NOMSL;DEBUGINFO;OPTIMIZED"
        "PREFIX;GLSL;HLSL;MSL"
        "FILES;DEFINES"
        ${ARGN}
    )

    foreach(file IN LISTS arg_FILES)
        set(qsb_result "${CMAKE_CURRENT_BINARY_DIR}/.qsb/${file}.qsb")
        get_filename_component(qsb_result_name "${qsb_result}" NAME)
        get_filename_component(file_absolute ${file} ABSOLUTE)
        set(qsb_args "")

        if (NOT arg_NOGLSL)
            if (arg_GLSL)
                set(glsl_versions "${arg_GLSL}")
            else()
                set(glsl_versions "100es,120,150") # both 'es' and ' es' are accepted by qsb
            endif()
            list(APPEND qsb_args "--glsl")
            list(APPEND qsb_args "${glsl_versions}")
        endif()

        if (NOT arg_NOHLSL)
            if (arg_HLSL)
                set(shader_model_versions "${arg_HLSL}")
            else()
                set(shader_model_versions 50)
            endif()
            list(APPEND qsb_args "--hlsl")
            list(APPEND qsb_args "${shader_model_versions}")
        endif()

        if (NOT arg_NOMSL)
            if (arg_MSL)
                set(metal_lang_versions "${arg_MSL}")
            else()
                set(metal_lang_versions 12)
            endif()
            list(APPEND qsb_args "--msl")
            list(APPEND qsb_args "${metal_lang_versions}")
        endif()

        if (arg_BATCHABLE)
            list(APPEND qsb_args "-b")
        endif()

        if (arg_PRECOMPILE)
            if (WIN32 AND NOT arg_NOHLSL)
                list(APPEND qsb_args "-c")
            endif()
        endif()

        if (arg_PERTARGETCOMPILE)
            list(APPEND qsb_args "-p")
        endif()

        if (arg_DEBUGINFO)
            list(APPEND qsb_args "-g")
        endif()

        if (arg_OPTIMIZED)
            list(APPEND qsb_args "-O")
        endif()

        foreach(qsb_def IN LISTS arg_DEFINES)
            list(APPEND qsb_args "-D")
            list(APPEND qsb_args "${qsb_def}")
        endforeach()

        list(APPEND qsb_args "-o")
        list(APPEND qsb_args "${qsb_result}")
        list(APPEND qsb_args "${file_absolute}")

        add_custom_command(
            OUTPUT
                ${qsb_result}
            COMMAND
                ${QT_CMAKE_EXPORT_NAMESPACE}::qsb ${qsb_args}
            DEPENDS
                "${file_absolute}"
                ${QT_CMAKE_EXPORT_NAMESPACE}::qsb
            VERBATIM
        )

        list(APPEND qsb_files "${qsb_result}")
        set_source_files_properties("${qsb_result}" PROPERTIES QT_RESOURCE_ALIAS "${qsb_result_name}")
    endforeach()

    qt6_add_resources(${target} ${resourcename}
        PREFIX
            "${arg_PREFIX}"
        FILES
            "${qsb_files}"
    )
endfunction()

if(NOT QT_NO_CREATE_VERSIONLESS_FUNCTIONS)
    function(qt_add_shaders)
        qt6_add_shaders(${ARGV})
    endfunction()
endif()
