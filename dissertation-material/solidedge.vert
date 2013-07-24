#version 130

uniform mat4 modelViewProjectionMatrix;
varying vec4 idColour;

void main()
{
	idColour = gl_Color;
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}