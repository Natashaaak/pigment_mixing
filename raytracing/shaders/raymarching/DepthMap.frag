#version 450

in float rad;
in vec3 center;
in mat3 anMat;

uniform mat4 proj;
uniform bool isAni;
layout(location = 0) out float depth;
//idea: https://stackoverflow.com/questions/61438498/how-can-i-make-gl-points-overlap-to-look-like-spheres
///Restores the 3D position of the sphere or ellipsoid(anisotropic rendering) in order to compute this pixels depth
///from the 2D particle.
void main(){
    vec2 xy = gl_PointCoord * 2.0f - 1.0f;
    float r2 = dot(xy, xy);
    if(r2 >= 1.0f) discard;

    vec2 xy_view = xy*rad;
    float z = isAni ? sqrt(1.0f - dot(xy, xy)) : rad * sqrt(max(1.0f - r2, 0.0f));
    vec3 point = isAni ? center + anMat * vec3(xy, z) : center + vec3(xy_view, z);
    depth = -point.z; // - is needed because we are looking to -z
    vec4 clip = proj * vec4(point, 1.0f);
    float ndc = clip.z / clip.w;
    gl_FragDepth = 0.5f * (gl_DepthRange.diff * ndc + gl_DepthRange.near + gl_DepthRange.far);
}