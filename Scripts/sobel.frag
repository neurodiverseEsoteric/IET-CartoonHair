//http://www.blitzbasic.com/Community/posts.php?topic=85263
#version 130

uniform sampler2D frameBuffer;

uniform float width;
uniform float height;
uniform float edgeThreshold;

varying vec2 texc;

void main()
{ 
	float dx = 1.0/width;
	float dy = 1.0/height;
	
	vec3 edgeOutput = vec3(0);
	
	//get the colour value of each pixel around the centre
	for(int y = 0 ; y < 5 ; y++)
	{
		for(int x = 0 ; x < 5 ; x++)
		{
			//if not at the centre
			if(x!=0 && y!=0)
			{
				edgeOutput = edgeOutput+vec3(texture2D(frameBuffer,texc.xy+vec2((x-2)*dx,(y-2)*dy)));
			}
		}
	}
	
	//see how different the surrounding pixels are from the centre
	vec3 centre = vec3(texture2D(frameBuffer,texc.xy));
	centre.r = centre.r*24;
	centre.g = centre.g*24;
	centre.b = centre.b*24;
	edgeOutput = edgeOutput-centre;
	
	edgeOutput.r = abs(edgeOutput.r);
	edgeOutput.g = abs(edgeOutput.g);
	edgeOutput.b = abs(edgeOutput.b);
	
	vec4 edgeColour = vec4(1);
	if(edgeOutput.r > edgeThreshold)
	{
		edgeColour = vec4(0,0,0,1);
	}
	else if(edgeOutput.g > edgeThreshold)
	{
		edgeColour = vec4(0,0,0,1);
	}
	else if(edgeOutput.b > edgeThreshold)
	{
		edgeColour = vec4(0,0,0,1);
	}
	
    gl_FragColor = edgeColour;
}