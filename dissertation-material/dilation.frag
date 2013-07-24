uniform sampler2D frameBuffer;

uniform int dilationX;
uniform int dilationY;

uniform float width;
uniform float height;

varying vec2 texc;

void main()
{ 
	float dx = 1.0/width;
	float dy = 1.0/height;
	
	int startX = -dilationX/2;
	int endX = dilationX/2;
	
	int startY = -dilationY/2;
	int endY = dilationY/2;
	
	vec4 pixelColour = vec4(1);
	
	//get the colour value of each pixel around the centre
	for(int y = startY ; y < endY ; y++)
	{
		for(int x = startX ; x < endX ; x++)
		{
			pixelColour = min(texture2D(frameBuffer,texc.xy+vec2(x*dx,y*dy)),pixelColour);
		}
	}
	
    gl_FragColor = pixelColour;
}