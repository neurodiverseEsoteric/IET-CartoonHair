#version 130

varying vec4 idColour;

void main()
{
	idColour = gl_Color;
	
	//need to flip the silhouette vertically as for some reason the edges are flipped in the rendertarget
	vec4 pos = gl_Vertex;
	pos.y = pos.y*(-1.0);
    gl_Position = pos;
}