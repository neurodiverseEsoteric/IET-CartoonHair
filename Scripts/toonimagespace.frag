#version 130

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
varying vec3 v;
varying vec3 proj;

varying float i;

void main()
{ 
	vec4 strokeColour = vec4(1,1,1,1);
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
	gl_FragColor = strokeColour*hairColour*texture1D(cartoonTexture,i,0.0);
}