#version 130

uniform mat4 modelViewMatrix;
uniform mat4 modelViewProjectionMatrix;

uniform vec4 lightPos;
uniform float strokeScale;

varying vec3 proj;
varying vec3 n;
varying vec3 l;
varying vec3 v;
varying float i;

void main()
{
	proj = vec4(modelViewProjectionMatrix*gl_Vertex).xyz;
	proj.s = proj.s*strokeScale;
	proj.t = proj.t*strokeScale;
	
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
	v = vec3(modelViewMatrix*gl_Vertex);
	
	i = max(dot(n,l),0.0);
	
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}