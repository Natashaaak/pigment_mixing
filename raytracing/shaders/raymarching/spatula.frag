#version 450 core
out vec4 FragColor;

in vec3 WorldPos;
in vec3 Normal;
flat in int vMaterialID;

uniform vec3 camPos;
uniform bool fullRender;
uniform vec3 lightDirs[2];
uniform vec3 lightColors[2];

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
        float shadows[2];
        shadows[0] = step(0.0, dot(N, lightDirs[0])); // 1.0 pokud je osvětleno, 0.0 pokud je ve vlastním stínu
        shadows[1] = step(0.0, dot(N, lightDirs[1])); // step(edge, x) vrací 0.0 pokud x < edge, jinak 1.0
        vec3 lit_color = computePBRLighting(mat, floorMat, WorldPos, N, V, lightDirs, lightColors, shadows);

        // Stíny z přímého osvětlení jsou aplikovány uvnitř computePBRLighting.
        // Problém je, že nepřímé osvětlení (odrazy od oblohy) stále plně osvětluje i odvrácenou stranu,
        // což na lesklém materiálu vypadá, jako by stíny neexistovaly.
        // Pro tmavší a realističtější stíny musíme ztmavit i toto nepřímé (ambientní) osvětlení.
        // Následující kód je aproximace ambientní okluze (AO) založená na self-shadowingu.

        // Vypočítáme "míru osvětlení" z přímých světel s měkkým přechodem.
        // 1.0 = plně osvětleno, 0.0 = v plném stínu.
        float light_amount = smoothstep(0.0, 0.15, max(dot(N, lightDirs[0]), dot(N, lightDirs[1])));

        // Aplikujeme ztmavení na finální barvu. mix(0.2, 1.0, ...) znamená, že v plném stínu bude výsledná barva mít jen 20% původního jasu.
        color = lit_color * mix(0.2, 1.0, light_amount);
    } else {
        vec3 irradiance = texture(irradianceMap, N).rgb;
        color = mat.albedo * irradiance;
    }
    
    FragColor = vec4(color, 1.0);
}