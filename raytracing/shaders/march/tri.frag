#version 450

in vec3 FragViewPos;

out vec4 fragColor;

const vec3 lightDirView = normalize(vec3(0.0, 0.0, 1.0));
const vec3 color = vec3(0.557, 0.645, 0.969);
///Renders final surface reconstructed by marching cubes with shading by computing normal for each triangle
void main() {
    vec3 T = dFdx(FragViewPos);
    vec3 B = dFdy(FragViewPos);
    vec3 N = normalize(cross(T, B));

    float diff = max(dot(N, lightDirView), 0.0);

    fragColor = vec4(color*diff, 1.0);
}
