#version 130

uniform mat4 modelViewMatrix;
uniform mat4 modelViewProjectionMatrix;

uniform vec4 lightPos;

varying vec3 n;
varying vec3 l;

void main()
{
	vec3 v = vec3(modelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
}