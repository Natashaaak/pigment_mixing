#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) readonly uniform image2D inTex;
layout(rgba32f, binding = 1) readonly uniform image2D normalDepthTex;
layout(rgba32f, binding = 2) writeonly uniform image2D outTex;

uniform int width;
uniform int height;
uniform ivec2 blurDirection;
uniform bool isFinalPass;

void main() {
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    if (pix.x >= width || pix.y >= height) return;

    vec4 centerData = imageLoad(inTex, pix);
    
    // Handle background
    if (centerData.a < -1.5) { // Background preview
        imageStore(outTex, pix, vec4(centerData.rgb, isFinalPass ? 1.0 : centerData.a));
        return;
    }
    if (centerData.a < -0.5) { // Transparent background for skybox
        imageStore(outTex, pix, vec4(centerData.rgb, isFinalPass ? 0.0 : centerData.a));
        return;
    }
    // Solid objects (fluid, spatula) - pass through original color without blur
    if (centerData.a > 1.5) {
        imageStore(outTex, pix, vec4(centerData.rgb, isFinalPass ? 1.0 : centerData.a));
        return;
    }

    // We are on the floor (Alpha contains shadow value between [0, 1])
    vec4 centerND = imageLoad(normalDepthTex, pix);
    vec3 centerNormal = centerND.xyz;
    float centerDepth = centerND.w;

    vec3 resultColor = vec3(0.0);
    float sumWeights = 0.0;
    
    // Filter parameters
    float sigmaSpatial = 12.0; // Increased for a stronger blur
    float sigmaDepth = 1.0;
    float sigmaNormal = 0.1;
    int radius = 10; // Increased search area to a 21x21 kernel
    
    for (int i = -radius; i <= radius; ++i) {
            ivec2 offset = blurDirection * i;
            ivec2 samplePix = clamp(pix + offset, ivec2(0), ivec2(width - 1, height - 1));
            vec4 sampleData = imageLoad(inTex, samplePix);
            
            // Only take a sample if it's the same object (the floor)
            if (sampleData.a < 0.0 || sampleData.a > 1.5) continue;
            
            vec4 sampleND = imageLoad(normalDepthTex, samplePix);
            
            float distSq = float(i * i);
            float wSpatial = exp(-distSq / (2.0 * sigmaSpatial * sigmaSpatial));
            
            float depthDiff = centerDepth - sampleND.w;
            float wDepth = exp(-(depthDiff * depthDiff) / (2.0 * sigmaDepth * sigmaDepth));
            
            float normalDiff = 1.0 - max(dot(centerNormal, sampleND.xyz), 0.0);
            float wNormal = exp(-(normalDiff * normalDiff) / (2.0 * sigmaNormal * sigmaNormal));
            
            float w = wSpatial * wDepth * wNormal;
            resultColor += sampleData.rgb * w;
            sumWeights += w;
    }
    
    imageStore(outTex, pix, vec4(sumWeights > 0.0 ? resultColor / sumWeights : centerData.rgb, isFinalPass ? 1.0 : centerData.a));
}