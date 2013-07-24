#version 130

uniform mat4 modelViewProjectionMatrix;
varying vec4 idColour;
varying vec2 texc;

void main()
{
	//partially derived from http://www.blitzbasic.com/Community/posts.php?topic=85263
	texc = vec2(gl_MultiTexCoord0);
	idColour = gl_Color;
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}