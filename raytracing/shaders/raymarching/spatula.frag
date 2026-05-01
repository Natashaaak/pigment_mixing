#version 450 core
out vec4 FragColor;

in vec3 WorldPos;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3 camPos;
uniform bool fullRender;

const float pi = 3.14159265359;

#include "pbr_lighting.glsl"

uniform Material spatulaMat;

void main() {
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);

    vec3 color;
    if (fullRender) {
        color = computePBRLighting(spatulaMat, WorldPos, N, V);
    } else {
        vec3 irradiance = texture(irradianceMap, N).rgb;
        color = spatulaMat.albedo * irradiance;
    }
    
    FragColor = vec4(color, 1.0);
}