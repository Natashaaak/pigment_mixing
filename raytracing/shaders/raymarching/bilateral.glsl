#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) readonly uniform image2D inTex;
layout(rgba32f, binding = 1) readonly uniform image2D normalDepthTex;
layout(rgba32f, binding = 2) writeonly uniform image2D outTex;

uniform int width;
uniform int height;

void main() {
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    if (pix.x >= width || pix.y >= height) return;

    vec4 centerData = imageLoad(inTex, pix);
    
    // Ošetření pozadí
    if (centerData.a < -1.5) { // Preview pozadí
        imageStore(outTex, pix, vec4(centerData.rgb, 1.0));
        return;
    }
    if (centerData.a < -0.5) { // Transparentní pozadí pro skybox
        imageStore(outTex, pix, vec4(centerData.rgb, 0.0));
        return;
    }
    // Pevné objekty (tekutina a špachtle) - propustí původní barvu bez rozmazání
    if (centerData.a > 1.5) {
        imageStore(outTex, pix, vec4(centerData.rgb, 1.0));
        return;
    }

    // Jsme na podložce (Alpha obsahuje hodnotu stínu mezi [0, 1])
    vec4 centerND = imageLoad(normalDepthTex, pix);
    vec3 centerNormal = centerND.xyz;
    float centerDepth = centerND.w;

    vec3 resultColor = vec3(0.0);
    float sumWeights = 0.0;
    
    // Parametry filtru
    float sigmaSpatial = 12.0; // Zvětšeno pro silnější rozmazání (zkuste hodnoty 10.0 - 15.0)
    float sigmaDepth = 1.0;
    float sigmaNormal = 0.1;
    int radius = 10; // Zvětšení prohledávané oblasti na 21x21 kernel
    
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            ivec2 offset = ivec2(x, y);
            ivec2 samplePix = clamp(pix + offset, ivec2(0), ivec2(width - 1, height - 1));
            vec4 sampleData = imageLoad(inTex, samplePix);
            
            // Nabereme vzorek pouze pokud se jedná o stejný objekt (podložku)
            if (sampleData.a < 0.0 || sampleData.a > 1.5) continue;
            
            vec4 sampleND = imageLoad(normalDepthTex, samplePix);
            
            float distSq = float(x*x + y*y);
            float wSpatial = exp(-distSq / (2.0 * sigmaSpatial * sigmaSpatial));
            
            float depthDiff = centerDepth - sampleND.w;
            float wDepth = exp(-(depthDiff * depthDiff) / (2.0 * sigmaDepth * sigmaDepth));
            
            float normalDiff = 1.0 - max(dot(centerNormal, sampleND.xyz), 0.0);
            float wNormal = exp(-(normalDiff * normalDiff) / (2.0 * sigmaNormal * sigmaNormal));
            
            float w = wSpatial * wDepth * wNormal;
            resultColor += sampleData.rgb * w;
            sumWeights += w;
        }
    }
    
    imageStore(outTex, pix, vec4(sumWeights > 0.0 ? resultColor / sumWeights : centerData.rgb, 1.0));
}