#version 450

layout(location = 0) in vec4 sphere;
layout(location = 1) in mat3 aniso;

uniform mat4 view;
uniform mat4 proj;
uniform int wh;
uniform float h;
uniform bool isAni;

out float rad;
out vec3 center;
out mat3 anMat;
//idea and math: https://stackoverflow.com/questions/25780145/gl-pointsize-corresponding-to-world-space-size
///Computes the size of the particle that would be used in rasterisation and fragment shader.
void main(){
    vec4 vc = view * vec4(sphere.xyz, 1.0f);
    center = vc.xyz;
    rad = h;
    anMat = aniso;
    vec4 cl = proj * vc;
    float size = h;
    if(isAni){
        vec3 x = aniso[0];
        vec3 y = aniso[1];
        vec3 z = aniso[2];
        size = 0.9 * max(length(x), max(length(y), length(z)));
    }
    gl_PointSize = (float(wh) * proj[1][1] * size) / abs(cl.w);
    gl_Position = cl;
}