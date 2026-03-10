#version 450

layout(local_size_x = 16, local_size_y = 8, local_size_z = 8) in;

struct Aniso {
    vec4 col1;
    vec4 col2;
    vec4 col3; //using columns because mat3 will be as 3xvec4 and there will be problem with the access.
    vec4 bDet;
    vec4 spherePosRad; //x,y,z,radius
};
///Anisotropy matrices
layout(std430, binding = 0) buffer Spheres {
    Aniso[] an;
};

layout(rgba32f, binding = 0) uniform image3D densityField;
///amount of spheres
uniform int spheresCount;
///start of the grid
uniform vec3 gridStart;
///voxel size
uniform float voxelS;
///amount of cells in each axis
uniform ivec3 cells;
///sph support radius
uniform float h;

const float pi = 3.141592;
const float kern = 315.0/(64.0*pi); //poly6
///Computes density with poly6 kernel for both isotropic and anisotropic kernels, with isotropic mat3(1/h)
float countKerA(vec3 P){
    float density = 0.0;
    for(int i = 0; i < spheresCount; ++i){ //Might be better iterating through neighbors with helping structures
        vec3 C = an[i].spherePosRad.xyz;
        mat3 B = mat3(an[i].col1.xyz, an[i].col2.xyz, an[i].col3.xyz);
        vec3 r = B * (P - C);
        float r2 = dot(r,r);
        if(r2 > 1.0) continue;
        float w = kern * an[i].bDet.x * pow(1.0 - r2, 3.0);
        density += w;
    }
    return density;
}

void main() {
    ivec3 vertex = ivec3(gl_GlobalInvocationID.xyz);
    if (any(greaterThanEqual(vertex, cells))) return;
    vec3 P = gridStart + vertex * voxelS;
    float density;
    density = countKerA(P);

    imageStore(densityField, vertex, vec4(density));
}
