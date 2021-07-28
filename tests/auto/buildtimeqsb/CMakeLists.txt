qt_internal_add_test(tst_buildtimeqsb
    SOURCES
        tst_buildtimeqsb.cpp
    PUBLIC_LIBRARIES
        Qt::Gui
        Qt::GuiPrivate
        Qt::ShaderToolsPrivate
)

qt6_add_shaders(tst_buildtimeqsb "shaders"
    PREFIX
        "/test"
    FILES
        "color.vert"
        "color.frag"
)

qt6_add_shaders(tst_buildtimeqsb "shaders_in_subdir"
    PREFIX
        "/some/prefix"
    FILES
        "subdir/texture.vert"
        "subdir/texture.frag"
)

qt6_add_shaders(tst_buildtimeqsb "shaders_custom_targets_and_names"
    PREFIX
        "/test"
    GLSL
        "300es,330"
    NOHLSL
    NOMSL
    FILES
        "color.vert"
        "color.frag"
    OUTPUTS
        "color_1.vert.qsb"
        "color_1.frag.qsb"
)

qt6_add_shaders(tst_buildtimeqsb "shaders_custom_targets_and_names_batchable"
    PREFIX
        "/test"
    BATCHABLE
    GLSL
        "300es,330"
    NOHLSL
    NOMSL
    FILES
        "color.vert"
        "color.frag"
    OUTPUTS
        "color_1b.vert.qsb"
        "color_1b.frag.qsb"
)

qt6_add_shaders(tst_buildtimeqsb "shaders_with_defines"
    PREFIX
        "/"
    DEFINES
        "MY_DEFINE"
        "ANOTHER_DEFINE"
    FILES
        "subdir/test/texture_def.frag"
)

qt6_add_shaders(tst_buildtimeqsb "shaders_with_replacements"
    PREFIX
        "/test"
    FILES
        "color.vert@glsl,100es,replacements/r1.vert@spirv,100,replacements/r2.spv@hlsl,50,replacements/r3.vert"
        "color.frag@msl,12,replacements/r4.frag"
    OUTPUTS
        "color_repl.vert.qsb"
        "color_repl.frag.qsb"
)
