#version 330 core
out vec4 pixel;
in vec2 oTex;
in vec4 oCol;
uniform sampler2D sprTex;
uniform int lightmode;


void main()
{

    switch (lightmode) 
    {
        case 1: // directLight
            pixel = texture(sprTex, oTex) * oCol; // direcLight();
            break;
        case 2: // tba
            
            break;
        case 0: // no light
        default:
            pixel = texture(sprTex, oTex) * oCol;
            break;
    }

	
}