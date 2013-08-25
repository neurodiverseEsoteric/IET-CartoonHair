#version 130

uniform sampler2D edgeTexture;
uniform int textureEnabled;
varying vec4 idColour;

void main()
{ 
	//applies either a saturated version of the silhouette texture or renders it pure white
	vec4 texColour = texture2D(edgeTexture,gl_TexCoord[0].st,0.0);
	//convert black parts of texture to white and retain alpha
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