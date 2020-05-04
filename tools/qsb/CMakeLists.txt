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

qt_add_tool(qsb
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
