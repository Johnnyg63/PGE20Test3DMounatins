#version 330 core

layout(location = 0) in vec4 aPos;	// Position		{ x ,y, z, w}
layout(location = 1) in vec2 aTex;	// TexCoords	{ x, y }
layout(location = 2) in vec4 aCol;	// Colour		{ R, G, B, A}
layout (location = 3) in vec3 aNormal; // Normals (not necessarily normalized)

uniform mat4 mvp;		// Mat View Projection
uniform int is3d;		// Is it a 2D or 3D
uniform vec4 tint;		// Tint colour to be applied	{ R, G, B, A}
out vec2 oTex;			// Output text coords
out vec4 oCol;			// Output Colours

out vec3 crntPos;		// Outputs the current position for the Fragment Shader
out vec3 Normal;		// Outputs the normal for the Fragment Shader


void main()
{
	if(is3d!=0) 
	{
		// calculates current position // used in lighting and shadows
		crntPos = vec3(mvp * vec4(aPos.x, aPos.y, aPos.z, 1.0f));

		gl_Position = mvp * vec4(aPos.x, aPos.y, aPos.z, 1.0); 
		oTex = aTex;
		Normal = aNormal;
	} 
	else 
	{
		float p = 1.0 / aPos.z; 
		gl_Position = p * vec4(aPos.x, aPos.y, 0.0, 1.0); 
		oTex = p * aTex;
	} 

	oCol = aCol * tint;
}
			