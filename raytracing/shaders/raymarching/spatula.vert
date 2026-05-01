#version 450 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoords;

out vec3 WorldPos;
out vec3 Normal;
out vec2 TexCoords;

uniform mat4 invSpatulaTransform;
uniform mat4 view;
uniform mat4 projection;

void main() {
    mat4 model = inverse(invSpatulaTransform);
    vec4 worldPosition = model * vec4(aPos, 1.0);
    WorldPos = worldPosition.xyz;
    Normal = mat3(model) * aNormal; // Zde předpokládáme uniform scale. Pokud není, použij transpose(inverse(mat3(model)))
    TexCoords = aTexCoords;

    gl_Position = projection * view * worldPosition;
}