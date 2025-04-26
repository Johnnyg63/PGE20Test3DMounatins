#version 330 core
out vec4 pixel;
in vec2 oTex;
in vec4 oCol;

// Imports the normal from the Vertex Shader
in vec3 Normal;
// Imports the current position from the Vertex Shader
in vec3 crntPos;


uniform sampler2D sprTex;
uniform sampler2D sprTex1;
uniform int lightmode;
uniform int enablelight;
uniform vec4 lightcolour;
uniform int enableshadow;


vec4 direcLight()
{
	return texture(sprTex, oTex) * oCol * lightcolour;
}

void main()
{

    if(enablelight!=0)
    {
         switch (lightmode) 
        {
            case 1: // directLight
                pixel = direcLight();
                break;
            case 2: // tba
            
                break;
            case 0: // no light
            default:
                pixel = texture(sprTex, oTex) * oCol;
                break;
        }
    }
    else
    {
         pixel = texture(sprTex, oTex) * oCol;
    }

       


    

	
}