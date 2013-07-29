#version 130

//hatching code based off of http://www.cmlab.csie.ntu.edu.tw/~daniel/projects/hatching/hatching.htm

uniform mat4 modelViewMatrix;
uniform mat4 modelViewProjectionMatrix;

uniform vec4 cameraPosition;
uniform vec4 lightPos;

uniform float zMax;
uniform float zMin;
uniform float strokeScale;

varying vec3 proj;
varying vec3 n;
varying vec3 l;
varying vec3 v;
varying vec3 viewDirection;
varying float depth;

varying float d;

void main()
{
	v = vec3(modelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
	
	viewDirection = normalize(cameraPosition.xyz-v);
	
	proj = vec4(modelViewProjectionMatrix*gl_Vertex).xyz;
	proj.s = proj.s*strokeScale;
	proj.t = proj.t*strokeScale;
	
	gl_TexCoord[0] = gl_MultiTexCoord0*(1.0f/strokeScale);
	
	vec4 pos = modelViewProjectionMatrix*gl_Vertex;
	
	d = 1.0-log(pos.z/zMin)/log(zMax/zMin);
	
    gl_Position = pos;
}