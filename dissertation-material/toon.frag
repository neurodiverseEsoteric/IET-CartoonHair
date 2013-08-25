#version 130

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
uniform int hatchingEnabled;
uniform int simpleHatching;

varying float d;

varying vec3 n;
varying vec3 l;
varying vec3 v;
varying vec3 viewDirection;
varying vec3 proj;

void main()
{ 

	float i = max(dot(n,l),0.0);
	
	//cartoon texture
	float y = 1;
	//from x-toon an extended toon shader
	if(depthAxisEnabled == 1)
	{
		y = d;
	}
	vec4 cartoonColour = texture2D(cartoonTexture,vec2(i,y),0.0);
	
	//lets update i with the cartoon texture so that the hatching matches the tonal value
	//we can use just one channel as the texture is greyscale
	i = cartoonColour.r;
	
	//Blinn Specular - https://code.google.com/p/glslang-library/source/browse/trunk/trunk/glslang/shaders/material/blinn.frag.glsl?r=7
	vec3 h = normalize(l+v);
	
	vec3 specular = vec3(0,0,0);
	if(blinnEnabled == 1)
	{
		specular = vec3(1)*pow(max(dot(n,h),0.0),blinnS);
	}
	
	//based on Brennan Rusnell - X-Toon an extended toon shader
	
	if(specularTextureEnabled == 1)
	{
		vec3 r = reflect(l,n);
		float specIndex = pow(abs(dot(viewDirection,r)),specularTextureS);
		specular = specular + texture2D(specularTexture,vec2(i,specIndex),0.0).xyz;
	}
	
	vec4 backlighting = vec4(0);
	if(backlightingEnabled == 1)
	{
		float backlightingIndex = pow(abs(dot(n,viewDirection)),backlightingS);
		backlighting = vec4(1)-texture2D(backlightingTexture,vec2(i,backlightingIndex),0.0);
	}
	
	vec4 strokeColour = vec4(1);
	
	if(hatchingEnabled == 1)
	{
		if(simpleHatching == 1)
		{
			if(i < 0.16)
			{
				strokeColour = strokeColour*texture2DProj(stroke6,proj);
			}
			else if(i < 0.33)
			{
				strokeColour = strokeColour*texture2DProj(stroke5,proj);
			}
			else if(i < 0.49)
			{
				strokeColour = strokeColour*texture2DProj(stroke4,proj);
			}
			else if(i < 0.65)
			{
				strokeColour = strokeColour*texture2DProj(stroke3,proj);
			}
			else if(i < 0.81)
			{
				strokeColour = strokeColour*texture2DProj(stroke2,proj);
			}
			else
			{
				strokeColour = strokeColour*texture2DProj(stroke1,proj);
			}
		}
		else
		{
			//the hatching code is based off of the single-pass 6-way blending method by http://www.cmlab.csie.ntu.edu.tw/~daniel/projects/hatching/hatching.htm
			float weight0;
			float weight1;
			float weight2;
			float weight3;
			float weight4;
			float weight5;
			weight0 = weight1 = weight2 = weight3 = weight4 = weight5 = 0.0;
	
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
		
			vec4 colour0 = texture2D(stroke1,gl_TexCoord[0].st)*weight0;
			vec4 colour1 = texture2D(stroke2,gl_TexCoord[0].st)*weight1;
			vec4 colour2 = texture2D(stroke3,gl_TexCoord[0].st)*weight2;
			vec4 colour3 = texture2D(stroke4,gl_TexCoord[0].st)*weight3;
			vec4 colour4 = texture2D(stroke5,gl_TexCoord[0].st)*weight4;
			vec4 colour5 = texture2D(stroke6,gl_TexCoord[0].st)*weight5;
			
			strokeColour = colour0 + colour1 + colour2 + colour3 + colour4 + colour5;
			strokeColour.a = 1;
		}
	}
	
	gl_FragColor = strokeColour*hairColour*cartoonColour + backlighting + specular;
}