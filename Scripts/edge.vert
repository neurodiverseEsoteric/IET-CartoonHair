#version 130

uniform mat4 modelViewMatrix;
uniform mat4 modelViewProjectionMatrix;

void main()
{
    gl_Position = gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}