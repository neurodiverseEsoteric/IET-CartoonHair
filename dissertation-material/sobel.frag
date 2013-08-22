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
	
	//based on http://www.blitzbasic.com/Community/posts.php?topic=85263
	//(-1,1) (0,1) (1,1)
	//(-1,0) (0,0) (1,0)
	//(-1,-1) (0,-1) (1,-1)
	vec4 sample[9];
	
	sample[0] = texture2D(frameBuffer,texc.xy+vec2(-1*dx,1*dy));
	sample[1] = texture2D(frameBuffer,texc.xy+vec2(0,1*dy));
	sample[2] = texture2D(frameBuffer,texc.xy+vec2(1*dx,1*dy));
	
	sample[3] = texture2D(frameBuffer,texc.xy+vec2(-1*dx,0));
	sample[4] = texture2D(frameBuffer,texc.xy+vec2(0,0));
	sample[5] = texture2D(frameBuffer,texc.xy+vec2(1*dx,0));
	
	sample[6] = texture2D(frameBuffer,texc.xy+vec2(-1*dx,-1*dy));
	sample[7] = texture2D(frameBuffer,texc.xy+vec2(0,-1*dy));
	sample[8] = texture2D(frameBuffer,texc.xy+vec2(1,-1*dy));
	
	vec4 h = sample[2] + (2.0*sample[5]) + sample[8] - (sample[0] + (2.0*sample[3]) + sample[6]);
	vec4 v = sample[0] + (2.0*sample[1]) + sample[2] - (sample[6] + (2.0*sample[7]) + sample[8]);
	
	vec4 edgeOutput = sqrt((h*h)+(v*v));
	
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