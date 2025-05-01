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
uniform vec3 lightposition;
uniform vec3 cameraposition;
uniform float ambientlight;
uniform float specularlight;

uniform int enableshadow;


vec4 directLight()
{
	// diffuse lighting
	vec3 normal = normalize(Normal);
	vec3 lightDirection = normalize(lightposition - cameraposition);
	float diffuse = max(dot(normal, lightDirection), 0.0f);

    // specular lighting
	vec3 viewDirection = normalize(cameraposition - crntPos);
	vec3 reflectionDirection = reflect(-lightDirection, normal);
	float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16);
	float specular = specAmount * specularlight;


    vec4 lightcolor = lightcolour * (diffuse + ambientlight + specular);
	return texture(sprTex, oTex) * oCol * vec4(lightcolor.x, lightcolor.y, lightcolor.z, 1.0f);
}


vec4 pointLight()
{
    return texture(sprTex, oTex) * oCol;
}

vec4 spotLight()
{
    return texture(sprTex, oTex) * oCol;
}




void main()
{

    if(enablelight!=0)
    {
         switch (lightmode) 
        {
            case 1: 
                pixel = directLight();
                break;
            case 2:
                pixel = pointLight();
                break;
            case 3: 
                pixel = spotLight();
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