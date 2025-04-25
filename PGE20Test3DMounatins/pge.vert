#version 330 core

layout(location = 0) in vec4 aPos;	// Position		{ x ,y, z, w}
layout(location = 1) in vec2 aTex;	// TexCoords	{ x, y }
layout(location = 2) in vec4 aCol;	// Colour		{ R, G, B, A}
uniform mat4 mvp;		// Mat View Projection
uniform int is3d;		// Is it a 2D or 3D
uniform vec4 tint;		// Tint colour to be applied	{ R, G, B, A}
out vec2 oTex;			// Output text coords
out vec4 oCol;			// Outout oCol

void main()
{
	if(is3d!=0) 
	{
		gl_Position = mvp * vec4(aPos.x, aPos.y, aPos.z, 1.0); 
		oTex = aTex;
	} 
	else 
	{
		float p = 1.0 / aPos.z; 
		gl_Position = p * vec4(aPos.x, aPos.y, 0.0, 1.0); 
		oTex = p * aTex;
	} 

	oCol = aCol * tint;
}
			