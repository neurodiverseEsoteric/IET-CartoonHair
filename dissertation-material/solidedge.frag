#version 130

uniform sampler2D edgeTexture;
uniform int textureEnabled;
varying vec4 idColour;

void main()
{ 
	vec4 texColour = texture2D(edgeTexture,gl_TexCoord[0].st,0.0);
	texColour = vec4(vec3(1)-texColour.rgb,texColour.a);
	if(textureEnabled == 1)
	{
		gl_FragColor = texColour;
	}
	else
	{
		gl_FragColor = vec4(1);
	}
}