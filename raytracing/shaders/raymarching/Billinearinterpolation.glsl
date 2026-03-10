#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(rgba32f, binding = 0) uniform image2D outTex;

layout(rgba32f, binding = 1) uniform image2D normalDepthTex;

layout(binding = 2) uniform usampler2D VarianceTex;

layout(binding = 3) uniform sampler2D Dall;

const vec4 floorCol = vec4(0.375f, 0.35f, 0.325f, 1.0f);
const int lcs = 16; //equal to local size
const vec3 lightDirView = normalize(vec3(1.0f, -1.0f, -1.0f));
const vec3 color = vec3(0.557f, 0.645f, 0.969f);
///Image width
uniform int width;
///Image hegiht
uniform int height;
///Current depth variance pass
uniform int stride;
///Sets max depth for pixels to be considered close to the current fragment.
uniform float maxdv;

uint variance;
///Interpolates the normals of the pixels that had no ray casted for them due to ray casting resoluiton.
///Uses weights that are based on the neighboring pixels depths in order to make less blurriness from interpolation
///of pixels that are far away.
void main(){
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    if(pix.x >= width || pix.y >= height){
        return;
    }
    uint locId = gl_LocalInvocationIndex;
    uvec2 localID = gl_LocalInvocationID.xy;
    variance = texelFetch(VarianceTex, ivec2(pix/lcs), 0).r;
    if(variance != stride) return;
    float depth = texelFetch(Dall, pix, 0).r;
    float test = 1.0f;
    if(depth <= 0.0f){ //means this is zero and there is no sphere
        return;
    }
    ivec2 base = ivec2(gl_WorkGroupID.xy) * ivec2(lcs, lcs);
    bool hasColor = all(equal(localID % variance, uvec2(0)));
    if(!hasColor){//have to interpolate
        uvec2 x1y1 = clamp((localID / variance) * variance, uvec2(0), uvec2(width - 1, height - 1));
        bvec2 bounds = lessThan(x1y1 + variance, uvec2(lcs, lcs));
        uvec2 x2y2 = clamp(x1y1+variance, uvec2(0), uvec2(width - 1, height - 1)); //if is out of bounds then making it the same point
        vec4 poxLT, posLB, posRT, posRB, normLT, normLB, normRT, normRB;
        vec2 fac = (vec2(localID) - vec2(x1y1)) / float(variance);
        float wLT = (1.0f - fac.x) * (1.0f - fac.y), wRT = fac.x * (1.0f - fac.y);
        float wLB = (1.0f - fac.x) * fac.y, wRB = fac.x * fac.y;
        float mLT, mRT, mLB, mRB;
        ivec2 pixLT = base + ivec2(x1y1.x, x1y1.y);
        ivec2 pixRT = base + ivec2(x2y2.x, x1y1.y);
        ivec2 pixLB = base + ivec2(x1y1.x, x2y2.y);
        ivec2 pixRB = base + ivec2(x2y2.x, x2y2.y);

        //Loading normals and depths(at a)
        normLT = imageLoad(normalDepthTex, pixLT);
        normRT = imageLoad(normalDepthTex, pixRT);
        normLB = imageLoad(normalDepthTex, pixLB);
        normRB = imageLoad(normalDepthTex, pixRB);

        mLT = normLT == vec4(1000.0f) ? 0.0f : 1.0f;
        mRT = normRT == vec4(1000.0f) ? 0.0f : 1.0f;
        mLB = normLB == vec4(1000.0f) ? 0.0f : 1.0f;
        mRB = normRB == vec4(1000.0f) ? 0.0f : 1.0f;

        wLT *= mLT;
        wRT *= mRT;
        wLB *= mLB;
        wRB *= mRB;

        float wsum = wLT + wRT + wLB + wRB;
        if(wsum <= 0.0f){
            return;
        }
        float minDepth = min(min(normLT.a, normRT.a), min(normLB.a, normRB.a));
        if(minDepth == 1000.0f) {
            imageStore(outTex, pix, vec4(1.0, 0.0, 1.0, 1.0));
            return;
        }
        wLT *= abs(minDepth - normLT.a) > maxdv ? 1.0f / abs(minDepth - normLT.a) : 1.0f;
        wRT *= abs(minDepth - normRT.a) > maxdv ? 1.0f / abs(minDepth - normRT.a) : 1.0f;
        wLB *= abs(minDepth - normLB.a) > maxdv ? 1.0f / abs(minDepth - normLB.a) : 1.0f;
        wRB *= abs(minDepth - normRB.a) > maxdv ? 1.0f / abs(minDepth - normRB.a) : 1.0f;
        wsum = wLT + wRT + wLB + wRB;
        float a = 1.0f / wsum;
        wLT *= a;
        wRT *= a;
        wLB *= a;
        wRB *= a;

        vec4 res = normLT * wLT + normRT * wRT + normLB * wLB + normRB * wRB;
        imageStore(normalDepthTex, pix, res);
        float diff = max(dot(res.xyz, lightDirView), 0.0);
        imageStore(outTex, pix, vec4(color*diff, 1.0f));
    }
}