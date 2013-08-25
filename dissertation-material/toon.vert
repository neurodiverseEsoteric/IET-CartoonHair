#version 130

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
	//calculate the view direction, normal and light direction vectors
	v = vec3(modelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
	
	viewDirection = normalize(cameraPosition.xyz-v);
	
	//determine the projection coordinates for viewport hatching
	proj = vec4(modelViewProjectionMatrix*gl_Vertex).xyz;
	proj.s = proj.s*strokeScale;
	proj.t = proj.t*strokeScale;
	
	//determine the coordinates for surface hatching if enabled
	gl_TexCoord[0] = gl_MultiTexCoord0*strokeScale;
	
	vec4 pos = modelViewProjectionMatrix*gl_Vertex;
	
	//calculate the depth metric for the depth dependent cartoon texture
	d = 1.0-log(pos.z/zMin)/log(zMax/zMin);
	
    gl_Position = pos;
}