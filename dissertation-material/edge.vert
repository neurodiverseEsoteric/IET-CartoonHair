#version 130

uniform mat4 modelViewProjectionMatrix;
varying vec4 idColour;
varying vec2 texc;

void main()
{
	//we pass the current texture position to the fragment shader so that we can identify the current pixel's position in the id buffer
	//this method was derived from http://www.blog.nathanhaze.com/glsl-edge-detection/
	texc = vec2(gl_MultiTexCoord0);
	idColour = gl_Color;
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}