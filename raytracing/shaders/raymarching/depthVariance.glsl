#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform sampler2D Dall;

layout(r8ui, binding = 1) writeonly uniform uimage2D VarianceIm;

const int lcs = 16; //equal to local size
const int fullSize = lcs*lcs;
///Image width
uniform int width;
///Image height
uniform int height;
///e1 parameter from the paper used for changing from half to quarter res
uniform float e1;
///e2 parameter used for changing from full to half res
uniform float e2;
uniform bool test;
///Stores the amount of functioning pixels in a tile
shared uint amount[fullSize];
///Stores sum of all depths in a tile
shared float sum[fullSize];
///Stores sum of squared depths in a tile
shared float sumSq[fullSize];
//https://developer.download.nvidia.com/assets/cuda/files/reduction.pdf
///Computes depth variance using parallel reduction and stores it in a small texture where every pixel represents tile
///in the final image.
void main(){
    bool exist = true;
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    uint locId = gl_LocalInvocationIndex;
    if (pix.x >= width || pix.y >= height) {
        amount[locId] = 0;
        exist = false;
//        return; No return before barrier.
    }
    ivec2 tileId = ivec2(gl_WorkGroupID.xy);
    float depth = exist ?  texelFetch(Dall, pix, 0).r : -100.0f;
    sum[locId] = depth;
    sumSq[locId] = depth * depth;
    amount[locId] = depth > 0.0f ? 1 : 0;//check if it is floor or not
    barrier();
    for (uint s = (fullSize)/2; s > 0; s>>=1){
        if (locId < s){
            sum[locId] += sum[locId + s];
            sumSq[locId] += sumSq[locId + s];
            amount[locId] += amount[locId + s];
        }
        barrier();
    }
    if (locId == 0){
        uint res = 0; //if this whole block if floor then we do not care about resolution of ray casting, because we skip
        if (amount[0] > 0){
            float mean = sum[0] / float(amount[0]);
            float variance = sumSq[0] / float(amount[0]) - mean * mean;
            res = variance > e2 ? 1 : variance > e1 ? 2 : 4;
            if(amount[0] != fullSize){
                res = 1;
            }
        }
        if(test) res = 1;
        imageStore(VarianceIm, tileId, uvec4(res));
    }
}