#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform sampler2D Di; //Dall in first, DallTMP in second

layout(binding = 1) uniform sampler2D Dall;

layout(r32f, binding = 2) writeonly uniform image2D Do; //DallTMP in fisrt, DallSmooth in second

///Image width
uniform int width;
///Image height
uniform int height;
///has to be h(sph support radius) since we draw Dall with that, or r if want to see spheres instead
uniform float r;
///field of view
uniform float fov;
uniform float ks;
///Sigma r value
uniform float sigma_r;
///R value
uniform float filterR;
///(1 0) for first pass (0 1) for second. X and Y passes of the separable implementation
uniform ivec2 dir;

const float sqrt2 = 1.41421356237;
const float blur = -1/sqrt2;

//https://www.scribd.com/document/89175478/Screen-Space-Fluid-Rendering-for-Games
///Separable implementation of the gauss bilateral filter that smooths the final depth map. Computed in two dispatches
void main(){
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    if(pix.x >= width || pix.y >= height){
        return;
    }
    float depth = texelFetch(Di, pix, 0).r;
    if(depth <= 0.0f){
        imageStore(Do, pix, vec4(0.0f));
        return;
    }
    float dallDepth = texelFetch(Dall, pix, 0).r;
    float rij = (height * (2*r)) / (2*abs(dallDepth)*tan(fov/2));
    float sigma_s = ks * rij;
    float blurR = blur / sigma_r;
    float blurS = blur / sigma_s;
    float sum = 0.0f;
    float wsum = 0.0f;
    for(float n = -filterR; n <= filterR; n+=1.0f){
        ivec2 neigh = ivec2(clamp(pix + ivec2(int(n))*dir, ivec2(0), ivec2(width-1, height-1)));
        float currDepth = texelFetch(Di, neigh, 0).r;
        if(currDepth <= 0.0f){
            continue;
        }
        float e = n * blurS;
        float gs = exp(-e*e);
        float l = (currDepth - depth) * blurR;
        float gr = exp(-l*l);
        sum += currDepth * gs * gr;
        wsum += gs * gr;
    }
    if (wsum > 0.0f){
        sum/=wsum;
    }
    imageStore(Do, pix, vec4(sum));
}