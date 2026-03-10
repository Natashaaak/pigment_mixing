#version 450 core

in vec2 texCoord;

out vec4 fragColor;

uniform sampler2D renderedImage;
///Renders final texture
void main(){
    fragColor = texture(renderedImage, texCoord);
}