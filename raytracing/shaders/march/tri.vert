#version 450

layout(location = 0) in vec3 pos;

uniform mat4 proj;
uniform mat4 model;
uniform mat4 view;

out vec3 FragViewPos;
///First step in rendering reconstructed surface by marching cubes
void main() {
    vec4 viewPos = view * vec4(pos, 1.0);
    FragViewPos = viewPos.xyz;
    gl_Position = proj * viewPos;
}
