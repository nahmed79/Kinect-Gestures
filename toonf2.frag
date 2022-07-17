uniform sampler2D tex;

//varying vec3 lightDir;
//varying vec3 normal;


void main()
{
	//float intensity = dot(lightDir,normalize(normal));

	//vec4 color = texture2D(tex,gl_TexCoord[0].st) * intensity;
	vec4 color = texture2D(tex,gl_TexCoord[0].st);
	gl_FragColor = color;
	//gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
