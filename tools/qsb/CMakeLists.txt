# Generated from qsb.pro.

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
        Qt::ShaderTools
)
