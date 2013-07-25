#version 130
//the hatching code is based off of http://www.cmlab.csie.ntu.edu.tw/~daniel/projects/hatching/hatching.htm

uniform sampler2D cartoonTexture;
uniform sampler2D stroke1;
uniform sampler2D stroke2;
uniform sampler2D stroke3;
uniform sampler2D stroke4;
uniform sampler2D stroke5;
uniform sampler2D stroke6;
uniform sampler2D specularTexture;
uniform sampler2D backlightingTexture;

uniform vec4 hairColour;
uniform float blinnS;
uniform float specularTextureS;
uniform float backlightingS;

uniform int blinnEnabled;
uniform int depthAxisEnabled;
uniform int specularTextureEnabled;
uniform int backlightingEnabled;

varying float d;

varying vec3 n;
varying vec3 l;
varying vec3 v;
varying vec3 viewDirection;

varying float weight0;
varying float weight1;
varying float weight2;
varying float weight3;
varying float weight4;
varying float weight5;

varying float i;

void main()
{ 
	vec4 colour0 = texture2D(stroke1,gl_TexCoord[0].st)*weight0;
	vec4 colour1 = texture2D(stroke2,gl_TexCoord[0].st)*weight1;
	vec4 colour2 = texture2D(stroke3,gl_TexCoord[0].st)*weight2;
	vec4 colour3 = texture2D(stroke4,gl_TexCoord[0].st)*weight3;
	vec4 colour4 = texture2D(stroke5,gl_TexCoord[0].st)*weight4;
	vec4 colour5 = texture2D(stroke6,gl_TexCoord[0].st)*weight5;
	
	vec4 strokeColour = colour0 + colour1 + colour2 + colour3 + colour4 + colour5;
	strokeColour.a = 1;
	
	//Blinn Specular - https://code.google.com/p/glslang-library/source/browse/trunk/trunk/glslang/shaders/material/blinn.frag.glsl?r=7
	vec3 h = normalize(l+v);
	
	vec3 specular = vec3(0,0,0);
	if(blinnEnabled == 1)
	{
		specular = vec3(1,1,1)*pow(max(dot(n,h),0.0),blinnS);
	}
	
	//based on Brennan Rusnell - X-Toon an extended toon shader
	if(specularTextureEnabled == 1)
	{
		vec3 r = reflect(l,n);
		float specIndex = pow(abs(dot(viewDirection,r)),specularTextureS);
		specular = specular + texture2D(specularTexture,vec2(i,specIndex),0.0).xyz;
	}
	
	float y = 1;
	//from x-toon an extended toon shader
	if(depthAxisEnabled == 1)
	{
		y = d;
	}
	
	if(backlightingEnabled == 1)
	{
		float backlightingIndex = pow(abs(dot(n,viewDirection)),backlightingS);
		specular = specular + texture2D(backlightingTexture,vec2(i,backlightingIndex),0.0).xyz;
	}
	
	//cartoon texture
	vec4 cartoonColour = texture2D(cartoonTexture,vec2(i,y),0.0);
	
	
	gl_FragColor = strokeColour*hairColour*cartoonColour + specular;
}