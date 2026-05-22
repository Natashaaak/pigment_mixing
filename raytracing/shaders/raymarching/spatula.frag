#version 450 core
out vec4 FragColor;

in vec3 WorldPos;
in vec3 Normal;
flat in int vMaterialID;

uniform vec3 camPos;
uniform bool fullRender;

const float pi = 3.14159265359;

#include "pbr_lighting.glsl"

uniform Material spatulaMaterials[2]; // 0: metal, 1: wood
uniform Material floorMat;

void main() {
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);
    Material mat = spatulaMaterials[vMaterialID];

    vec3 color;
    if (fullRender) {
        // Simple self-shadowing. Since the spatula is always on top,
        // we just need to check if the normal is pointing towards the light.
        float shadows[MAX_LIGHTS];
        for (int i = 0; i < numLights; ++i) {
            shadows[i] = step(0.0, dot(N, lightDirs[i])); // 1.0 if lit, 0.0 if in self-shadow
        }
        vec3 lit_color = computePBRLighting(mat, floorMat, WorldPos, N, V, shadows);

        // Shadows from direct lighting are applied inside computePBRLighting.
        // The problem is that indirect lighting (reflections from the sky) still fully illuminates the back side,
        // which on a glossy material makes it look as if shadows don't exist.
        // For darker and more realistic shadows, we must also darken this indirect (ambient) lighting.
        // The following code is an approximation of ambient occlusion (AO) based on self-shadowing.

        // We calculate a "lighting amount" from direct lights with a soft transition.
        // 1.0 = fully lit, 0.0 = in full shadow.
        float max_light_dot = 0.0;
        for (int i = 0; i < numLights; ++i) {
            max_light_dot = max(max_light_dot, dot(N, lightDirs[i]));
        }
        float light_amount = smoothstep(0.0, 0.15, max_light_dot);

        // Apply darkening to the final color. mix(0.2, 1.0, ...) means that in full shadow, the resulting color will only have 20% of its original brightness.
        color = lit_color * mix(0.2, 1.0, light_amount);
    } else {
        vec3 irradiance = texture(irradianceMap, N).rgb;
        color = mat.albedo * irradiance;
    }
    
    FragColor = vec4(color, 1.0);
}