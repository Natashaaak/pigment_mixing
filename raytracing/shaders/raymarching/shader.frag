#version 450 core

in vec2 texCoord;

out vec4 fragColor;

uniform sampler2D renderedImage;
uniform sampler2D normalDepthTex;
uniform mat4 proj;

///Renders final texture
void main(){
    fragColor = texture(renderedImage, texCoord);
    
    vec4 nd = texture(normalDepthTex, texCoord);
    if (nd.w < 999.0) { // Pokud paprsek něco zasáhl (w obsahuje viewZ, pozadí má 1000.0)
        vec4 clipSpace = proj * vec4(0.0, 0.0, nd.w, 1.0);
        gl_FragDepth = (clipSpace.z / clipSpace.w) * 0.5 + 0.5;
    } else {
        gl_FragDepth = 1.0; // Prázdné pozadí padá dozadu
    }
}