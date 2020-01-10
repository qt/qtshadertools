TEMPLATE = subdirs

SUBDIRS += \
    glslang \
    SPIRV-Cross \
    shadertools

shadertools.depends = glslang SPIRV-Cross
