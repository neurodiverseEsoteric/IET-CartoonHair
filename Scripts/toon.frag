#version 130
//based off of http://www.cmlab.csie.ntu.edu.tw/~daniel/projects/hatching/hatching.htm

uniform sampler1D cartoonTexture;
uniform sampler2D stroke1;
uniform sampler2D stroke2;
uniform sampler2D stroke3;
uniform sampler2D stroke4;
uniform sampler2D stroke5;
uniform sampler2D stroke6;

uniform vec4 hairColour;

varying vec3 n;
varying vec3 l;

varying float weight0;
varying float weight1;
varying float weight2;
varying float weight3;
varying float weight4;
varying float weight5;

varying float i;

void main()
{ 
	//float index = max(dot(n,l),0.0);
	
	vec4 colour0 = texture2D(stroke1,gl_TexCoord[0].st)*weight0;
	vec4 colour1 = texture2D(stroke2,gl_TexCoord[0].st)*weight1;
	vec4 colour2 = texture2D(stroke3,gl_TexCoord[0].st)*weight2;
	vec4 colour3 = texture2D(stroke4,gl_TexCoord[0].st)*weight3;
	vec4 colour4 = texture2D(stroke5,gl_TexCoord[0].st)*weight4;
	vec4 colour5 = texture2D(stroke6,gl_TexCoord[0].st)*weight5;
	
	vec4 strokeColour = colour0 + colour1 + colour2 + colour3 + colour4 + colour5;
	strokeColour.a = 1;
	
	gl_FragColor = strokeColour*hairColour*texture1D(cartoonTexture,i,0.0);
}