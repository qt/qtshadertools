#version 440

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 color;
layout(location = 0) out vec3 v_color;

// Declare the uniform block differently for every target. Doing this is a bad
// idea in practice: a QShader has a single QShaderDescription, so the
// reflection info can only ever match one of the targets. It must however
// always be the same one, in every run of the same bake.
#if defined(QSHADER_SPIRV)
layout(std140, binding = 0) uniform buf { mat4 mvp; } ubuf;
#elif defined(QSHADER_GLSL)
layout(std140, binding = 0) uniform buf { mat4 mvp; vec4 glslOnly; vec4 glslOnly2; } ubuf;
#elif defined(QSHADER_HLSL)
layout(std140, binding = 0) uniform buf { mat4 mvp; vec4 hlslOnly; } ubuf;
#elif defined(QSHADER_MSL)
layout(std140, binding = 0) uniform buf { mat4 mvp; vec2 mslOnly; } ubuf;
#else
#error No QSHADER_* macro defined
#endif

out gl_PerVertex { vec4 gl_Position; };

void main()
{
    v_color = color;
    gl_Position = ubuf.mvp * position;
}
