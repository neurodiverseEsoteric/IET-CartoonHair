//http://www.blog.nathanhaze.com/glsl-edge-detection/
#version 130

//the centre of the texture
varying vec2 texc;

void main()
{	
	texc = vec2(gl_MultiTexCoord0);
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}