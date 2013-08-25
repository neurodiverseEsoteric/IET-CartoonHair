//the centre of the texture
varying vec2 texc;

void main()
{	
	//we pass the current texture position to the fragment shader so that we can position the dilation kernel there
	//this method was derived from http://www.blog.nathanhaze.com/glsl-edge-detection/
	texc = vec2(gl_MultiTexCoord0);
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}