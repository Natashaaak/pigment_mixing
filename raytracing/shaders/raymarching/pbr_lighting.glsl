#ifndef PBR_LIGHTING_GLSL
#define PBR_LIGHTING_GLSL

// PBR Material Structure
struct Material {
    vec3 albedo;
    float metallic;
    float roughness;
};

#define MAX_LIGHTS 4
uniform int numLights;
uniform vec3 lightDirs[MAX_LIGHTS];
uniform vec3 lightColors[MAX_LIGHTS];

// Required textures for PBR
layout(binding = 6) uniform samplerCube irradianceMap;
layout(binding = 5) uniform samplerCube hdrMap; // Used for pre-filtered specular reflections
layout(binding = 7) uniform sampler2D brdfLUT;

// --- Cook-Torrance BRDF functions ---

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness*roughness;
    float a2 = a*a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;
    float num = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = pi * denom * denom;
    return num / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness) {
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;
    float num = NdotV;
    float denom = NdotV * (1.0 - k) + k;
    return num / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

vec3 computePBRLighting(Material mat, Material floorMat, vec3 worldPos, vec3 N, vec3 V, float shadows[MAX_LIGHTS]) {
    vec3 F0 = vec3(0.04); 
    F0 = mix(F0, mat.albedo, mat.metallic);
    float NdotV = max(dot(N, V), 0.0);
    vec3 Lo = vec3(0.0);

    for(int i = 0; i < numLights; i++) {
        vec3 L = normalize(lightDirs[i]);
        // Light from below the horizon is blocked by the infinite floor
        if (L.y < 0.0) continue;

        vec3 H = normalize(V + L);
        float NdotL = max(dot(N, L), 0.0);
        
        float NDF = DistributionGGX(N, H, mat.roughness);   
        float G   = GeometrySmith(N, V, L, mat.roughness);      
        vec3 F    = fresnelSchlick(max(dot(H, V), 0.0), F0);       
        
        vec3 numerator    = NDF * G * F;
        float denominator = 4.0 * NdotV * NdotL + 0.0001;
        vec3 specular     = numerator / denominator;
        
        vec3 kS = F;
        vec3 kD = vec3(1.0) - kS;
        kD *= 1.0 - mat.metallic;	  
        Lo += (kD * mat.albedo / pi + specular) * lightColors[i] * NdotL * shadows[i];
    }

    // Ambient IBL
    vec3 F_ibl = fresnelSchlickRoughness(NdotV, F0, mat.roughness);
    vec3 kS_ibl = F_ibl;
    vec3 kD_ibl = (1.0 - kS_ibl) * (1.0 - mat.metallic);
    
    // Diffuse IBL - simulate occlusion from the ground plane
    vec3 irradiance = texture(irradianceMap, N).rgb;
    // Smoothly fade out light from below to prevent hard edges.
    irradiance *= (0.5 + 0.5 * N.y);
    vec3 diffuse_ibl = irradiance * mat.albedo / pi;
    
    // Specular IBL
    const float MAX_REFLECTION_LOD = 4.0;
    vec3 R = reflect(-V, N);
    vec3 prefilteredColor;
    if (R.y < 0.0) {
        // Reflection ray hits the floor.
        // Approximate the light reflected from the floor. We can't do a full recursive bounce, so we'll
        // calculate the lighting on the floor (diffuse only) and use that as the reflected color.
        // This is much better than just using ambient light, as it includes direct light sources.
        vec3 floorIrradiance = texture(irradianceMap, vec3(0.0, 1.0, 0.0)).rgb;
        vec3 floorNormal = vec3(0.0, 1.0, 0.0);

        // Add direct lighting contribution.
        // NOTE: We are intentionally omitting shadows here for performance. A full shadow
        // calculation for every reflection would be too slow.
        vec3 directLighting = vec3(0.0);
        for (int i = 0; i < numLights; i++) {
            vec3 L = normalize(lightDirs[i]);
            if (L.y > 0.0) { // Only consider lights from above the floor
                float NdotL = max(dot(floorNormal, L), 0.0);
                directLighting += lightColors[i] * NdotL;
            }
        }

        // Combine ambient and direct lighting, then multiply by floor albedo (Lambertian diffuse)
        prefilteredColor = floorMat.albedo * (floorIrradiance + directLighting) / pi;
    } else
        prefilteredColor = textureLod(hdrMap, R, mat.roughness * MAX_REFLECTION_LOD).rgb;

    vec2 brdf  = texture(brdfLUT, vec2(NdotV, mat.roughness)).rg;
    vec3 specular_ibl = prefilteredColor * (F0 * brdf.x + brdf.y);
    
    vec3 ambient = (kD_ibl * diffuse_ibl + specular_ibl);
    return ambient + Lo;
}

#endif // PBR_LIGHTING_GLSL