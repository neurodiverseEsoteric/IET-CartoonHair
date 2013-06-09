#version 130

uniform sampler2D edgeTexture;

void main()
{ 
	gl_FragColor = texture2D(edgeTexture, gl_TexCoord[0].st);
}