#version 130

uniform sampler1D cartoonTexture;
uniform vec4 hairColour;

varying vec3 n;
varying vec3 l;

void main()
{ 
	float index = max(dot(n,l),0.0);
	gl_FragColor = hairColour*texture1D(cartoonTexture,index,0.0);
}