#version 450

layout(binding = 0) uniform CameraUBO {
    mat4 view;
    mat4 proj;
} camera;

layout(location = 0) in vec3 fragPos;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) flat in vec4 fragColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 normal = normalize(fragNormal);
    vec3 camera_pos = (inverse(camera.view) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
    vec3 view_dir = normalize(camera_pos - fragPos);
    float ndotv = abs(dot(normal, view_dir));
    float rim = pow(clamp(1.0 - ndotv, 0.0, 1.0), 2.0);
    float facing = mix(0.05, 0.18, ndotv);
    float alpha = clamp(fragColor.a * facing + rim * 0.55, 0.04, 0.8);
    vec3 color = fragColor.rgb * (0.28 + 0.72 * rim);
    outColor = vec4(color, alpha);
}
