#version 130

uniform mat4 modelViewMatrix;
uniform mat4 modelViewProjectionMatrix;

uniform vec4 lightPos;

varying vec3 n;
varying vec3 l;
varying vec3 proj;

void main()
{
	proj = vec4(gl_TextureMatrix[0]*modelViewProjectionMatrix*gl_Vertex).xyz;
	proj.s = proj.s*5;
	proj.t = proj.t*5;
	
	vec3 v = vec3(modelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
    gl_Position = modelViewProjectionMatrix*gl_Vertex;
	gl_TexCoord[0] = gl_MultiTexCoord0;
}