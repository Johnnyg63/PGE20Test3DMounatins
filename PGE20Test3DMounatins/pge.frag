#version 330 core
out vec4 pixel;
in vec2 oTex;
in vec4 oCol;
uniform sampler2D sprTex;
void main(){pixel = texture(sprTex, oTex) * oCol;}