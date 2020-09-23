# Generated from qsb.pro.

# special case begin
if(NOT QT_FEATURE_process)
    message(NOTICE "Skipping the build of qsb as the required feature process is not available.")
    return()
endif()
# special case end

#####################################################################
## qsb Tool:
#####################################################################

qt_get_tool_target_name(target_name qsb)
qt_internal_add_tool(${target_name}
    TOOLS_TARGET # special case
        ShaderTools # special case
    SOURCES
        qsb.cpp
    PUBLIC_LIBRARIES
        Qt::Gui
        Qt::GuiPrivate
        Qt::ShaderToolsPrivate
)

#### Keys ignored in scope 1:.:.:qsb.pro:<TRUE>:
# _REQUIREMENTS = "qtConfig(process)"
