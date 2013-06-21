#version 130

uniform sampler1D cartoonTexture;
uniform sampler2D stroke1;
uniform sampler2D stroke2;
uniform sampler2D stroke3;
uniform sampler2D stroke4;

uniform vec4 hairColour;

varying vec3 n;
varying vec3 l;
varying vec3 proj;

void main()
{ 
	float index = max(dot(n,l),0.0);
	vec4 strokeColour;
	
	//if(index < 0.25)
	//{
	//	strokeColour = texture2DProj(stroke4,proj);
	//}
	//else if (index < 0.5)
	//{
	//	strokeColour = texture2DProj(stroke3,proj);
	//}
	//else if( index < 0.75)
	//{
	//	strokeColour = texture2DProj(stroke2,proj);
	//}
	//else
	//{
	//	strokeColour = texture2DProj(stroke1,proj);
	//}
	
	strokeColour = texture2D(stroke1,gl_TexCoord[0].st);
	
	gl_FragColor = strokeColour*hairColour*texture1D(cartoonTexture,index,0.0);
}