#version 450

layout(binding = 0) uniform OrthoGridUBO {
    vec4 origin_px_ppu;
    vec4 viewport_cell;
    vec4 line_color;
    vec4 axis_color;
} ubo;

layout(location = 0) out vec4 outColor;

void main() {
    vec2 origin_px = ubo.origin_px_ppu.xy;
    vec2 ppu = ubo.origin_px_ppu.zw;
    float cell = max(ubo.viewport_cell.z, 1.0e-6);
    float line_px = ubo.viewport_cell.w;
    float axis_px = ubo.axis_color.w;

    // View-plane units: +X right, +Y up (Vulkan y grows downward).
    vec2 world = vec2(
        (gl_FragCoord.x - origin_px.x) / max(ppu.x, 1.0e-6),
        (origin_px.y - gl_FragCoord.y) / max(ppu.y, 1.0e-6));

    float dist_axis = min(abs(world.x) * ppu.x, abs(world.y) * ppu.y);
    float dist_grid = min(
        abs(world.x / cell - round(world.x / cell)) * cell * ppu.x,
        abs(world.y / cell - round(world.y / cell)) * cell * ppu.y);

    if (dist_axis <= axis_px * 0.5) {
        outColor = vec4(ubo.axis_color.xyz, 1.0);
        return;
    }
    if (dist_grid <= line_px * 0.5) {
        outColor = ubo.line_color;
        return;
    }
    discard;
}
