#version 130

uniform mat4 MVP;
uniform mat4 MV;

uniform vec4 lightPos;

varying vec3 n;
varying vec3 l;

void main()
{
	vec3 v = vec3(gl_ModelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}