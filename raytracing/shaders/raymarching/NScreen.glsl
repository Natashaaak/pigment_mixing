#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform sampler2D DSmooth;

layout(rgba32f, binding = 1) writeonly uniform image2D Nscreen;

///Image width
uniform int width;
///Image height
uniform int height;
///Inverted projecting matrix
uniform mat4 invProj;
///Returns the point position in view space based on the depth.
vec3 getViewPos(ivec2 pos){
    float depth = texelFetch(DSmooth, pos, 0).r;
    ivec2 res = ivec2(width, height);
    vec2 ndc = 2.0f * ((vec2(pos) + 0.5f) / res) - 1.0f;
    vec4 p_v = invProj * vec4(ndc, -1.0f, 1.0f);
    p_v /= p_v.w;
    vec3 dir_v = normalize(p_v.xyz);
    vec3 start_v = dir_v * depth/(-dir_v.z);
    return start_v;
}
///Checks if the pixel is valid
bool validSample(ivec2 p){
    if(p.x < 0 || p.y < 0 || p.x >= width || p.y >= height) return false;
    return texelFetch(DSmooth, p, 0).r > 0.0f;
}
///Computes screen space normals for the Dall by noncollinear vertices
void main(){
    ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
    if(pix.x >= width || pix.y >= height){
        return;
    }
    float depth = texelFetch(DSmooth, pix, 0).r;
    if(depth <= 0.0f){
        imageStore(Nscreen, pix, vec4(0.0f));
        return;
    }
    ivec2 left = ivec2(pix.x - 1, pix.y);
    ivec2 right = ivec2(pix.x + 1, pix.y);
    ivec2 up = ivec2(pix.x, pix.y + 1);
    ivec2 down = ivec2(pix.x, pix.y - 1);
    ivec2 neighX = validSample(right) ? right : validSample(left) ? left : (pix.x + 1 >= width ? left : right);
    ivec2 neighY = validSample(up) ? up : validSample(down) ? down : (pix.y + 1 >= height ? down : up);

    vec3 currP = getViewPos(pix);
    vec3 Px = getViewPos(neighX);
    vec3 Py = getViewPos(neighY);
    vec3 dx = Px - currP;
    vec3 dy = Py - currP;
    vec3 normal = normalize(cross(dx, dy));
    imageStore(Nscreen, pix, vec4(normal, 1.0f));
}
