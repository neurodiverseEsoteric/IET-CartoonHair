#version 130

uniform sampler2D edgeTexture;
varying vec4 idColour;

void main()
{ 
	//vec4 texColour = texture2D(edgeTexture,gl_TexCoord[0].st);
	gl_FragColor = vec4(1);//idColour;
}