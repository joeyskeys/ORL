<html>
    <head>
        <link href="/themes/vulkan/css/theme.min.css" rel="stylesheet">
        <link href="/themes/vulkan/css/theme-blue.min.css" rel="stylesheet">
    </head>
    <body style="background: #343131">
        <script src="/themes/vulkan/js/highlight.pack.js"></script>
        <script>hljs.initHighlightingOnLoad();</script>
        <pre class="glsl" style="margin: 0; padding-left: 10px"><code>#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) out vec3 fragColor;

vec2 positions[3] = vec2[](
    vec2(0.0, -0.5),
    vec2(0.5, 0.5),
    vec2(-0.5, 0.5)
);

vec3 colors[3] = vec3[](
    vec3(1.0, 0.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 0.0, 1.0)
);

void main() {
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    fragColor = colors[gl_VertexIndex];
}
</code></pre>
    </body>
</html>