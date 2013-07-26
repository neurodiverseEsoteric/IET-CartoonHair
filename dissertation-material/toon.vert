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

varying float weight0;
varying float weight1;
varying float weight2;
varying float weight3;
varying float weight4;
varying float weight5;

varying float i;

varying float d;

void main()
{
	v = vec3(modelViewMatrix*gl_Vertex);
	n = normalize(gl_Normal);
	l = normalize(lightPos.xyz-v);
	
	viewDirection = normalize(v-cameraPosition.xyz);

	weight0 = weight1 = weight2 = weight3 = weight4 = weight5 = 0.0;
	
	i = max(dot(n,l),0.0);
	float index = i*6.0;
	
	if(index > 5.0)
	{
		weight0 = 1.0;
	}
	else if(index > 4.0)
	{
		weight0 = 1.0 - (5.0-index);
		weight1 = 1.0 - weight0;
	}
	else if(index > 3.0)
	{
		weight1 = 1.0 - (4.0-index);
		weight2 = 1.0 - weight1;
	}
	else if(index > 2.0)
	{
		weight2 = 1.0 - (3.0-index);
		weight3 = 1.0 - weight2;
	}
	else if(index > 1.0)
	{
		weight3 = 1.0 - (2.0-index);
		weight4 = 1.0 - weight3;
	}
	else
	{
		weight4 = 1.0 - (1.0-index);
		weight5 = 1.0 - weight4;
	}
	
	proj = vec4(modelViewProjectionMatrix*gl_Vertex).xyz;
	proj.s = proj.s*strokeScale;
	proj.t = proj.t*strokeScale;
	
	gl_TexCoord[0] = gl_MultiTexCoord0*(1.0f/strokeScale);
	
	vec4 pos = modelViewProjectionMatrix*gl_Vertex;
	
	d = 1.0-log(pos.z/zMin)/log(zMax/zMin);
	
    gl_Position = pos;
}