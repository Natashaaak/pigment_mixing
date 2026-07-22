#version 450 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in int aMaterialID;

out vec3 WorldPos;
out vec3 Normal;
flat out int vMaterialID;

uniform mat4 invSpatulaTransform;
uniform mat4 view;
uniform mat4 projection;

void main() {
    mat4 model = inverse(invSpatulaTransform);
    vec4 worldPosition = model * vec4(aPos, 1.0);
    WorldPos = worldPosition.xyz;
    Normal = transpose(inverse(mat3(model))) * aNormal;
    vMaterialID = aMaterialID;

    gl_Position = projection * view * worldPosition;
}