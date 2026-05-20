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
        // Jednoduchý self-shadowing. Protože je špachtle vždy nahoře,
        // stačí zkontrolovat, zda normála směřuje ke světlu.
        float shadows[MAX_LIGHTS];
        for (int i = 0; i < numLights; ++i) {
            shadows[i] = step(0.0, dot(N, lightDirs[i])); // 1.0 if lit, 0.0 if in self-shadow
        }
        vec3 lit_color = computePBRLighting(mat, floorMat, WorldPos, N, V, shadows);

        // Stíny z přímého osvětlení jsou aplikovány uvnitř computePBRLighting.
        // Problém je, že nepřímé osvětlení (odrazy od oblohy) stále plně osvětluje i odvrácenou stranu,
        // což na lesklém materiálu vypadá, jako by stíny neexistovaly.
        // Pro tmavší a realističtější stíny musíme ztmavit i toto nepřímé (ambientní) osvětlení.
        // Následující kód je aproximace ambientní okluze (AO) založená na self-shadowingu.

        // Vypočítáme "míru osvětlení" z přímých světel s měkkým přechodem.
        // 1.0 = plně osvětleno, 0.0 = v plném stínu.
        float max_light_dot = 0.0;
        for (int i = 0; i < numLights; ++i) {
            max_light_dot = max(max_light_dot, dot(N, lightDirs[i]));
        }
        float light_amount = smoothstep(0.0, 0.15, max_light_dot);

        // Aplikujeme ztmavení na finální barvu. mix(0.2, 1.0, ...) znamená, že v plném stínu bude výsledná barva mít jen 20% původního jasu.
        color = lit_color * mix(0.2, 1.0, light_amount);
    } else {
        vec3 irradiance = texture(irradianceMap, N).rgb;
        color = mat.albedo * irradiance;
    }
    
    FragColor = vec4(color, 1.0);
}