#version 130

uniform sampler2D edgeTexture;
uniform sampler2D idTexture;

uniform float width;
uniform float height;
uniform float range;
varying vec4 idColour;
varying vec2 texc;


void main()
{ 
	//determine the size of a pixel in the texture
	float dx = 1.0/width;
	float dy = 1.0/height;
	
	//get the position of the current fragment in the ID buffer
	//texture addressing based on http://www.phasersonkill.com/?p=495
	vec2 texCoord = vec2(gl_FragCoord.x/width,gl_FragCoord.y/height);
	
	//flip y coordinate as gl_FragCoord's origin is the lower left while the texture's is not
	texCoord.y = texCoord.y*(-1.0);
	
	//get the ID colours for the current fragment and the same position in the ID buffer
	vec4 currentId = texture2D(idTexture,texCoord,0.0);
	vec4 texColour = texture2D(edgeTexture,gl_TexCoord[0].st,0.0);
	
	//to overcome the silhouette overlapping at the hair root - we fade the silhouette as we approach the root
	//the alpha values are calculated in the project application and then passed to the shader through the
	//otherwise unused alpha channel
	texColour.a = min(texColour.a,idColour.a);
	float intensity = 1-idColour.a;
	texColour.r = max(texColour.r,intensity);
	texColour.g = max(texColour.g,intensity);
	texColour.b = max(texColour.b,intensity);
	
	//see if id colours match
	if(idColour.r >= currentId.r-range && idColour.r <= currentId.r+range
	&& idColour.g >= currentId.g-range && idColour.g <= currentId.g+range
	&& idColour.b >= currentId.b-range && idColour.b <= currentId.b+range
	)
	{
		gl_FragColor = texColour;
	}
	//see if there is a nearby edge
	else if(currentId.r >= 1-range && currentId.r <= 1+range
	&& currentId.g >= 1-range && currentId.g <= 1+range
	&& currentId.b >= 1-range && currentId.b <= 1+range
	)
	{
		gl_FragColor = texColour;
	}
	else
	{
		gl_FragColor = vec4(1,1,1,0);
	}
		
}