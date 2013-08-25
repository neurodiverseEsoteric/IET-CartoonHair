#version 130

//simple pass through shader although the surface colour is passed to the fragment shader as the ID colour
uniform mat4 modelViewProjectionMatrix;
varying vec4 idColour;

void main()
{
	idColour = gl_Color;
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}