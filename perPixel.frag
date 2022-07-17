varying vec3 lightDir;
varying vec3 normal;

void main()
{
	float intensity = dot(lightDir,normalize(normal));

	vec4 color = vec4(0.75, 0.75, 0.75, 1) * intensity;
	gl_FragColor = color;
}
