//varying vec3 lightDir;
//varying vec3 normal;

uniform float waveTime;
uniform float waveWidth;
uniform float waveHeight;

void main() {

	//lightDir = normalize(vec3(gl_LightSource[0].position));
	//normal = gl_Normal;

	//gl_TexCoord[0] = gl_MultiTexCoord0;
	//gl_Position = ftransform();

	vec4 v = vec4(gl_Vertex);
    v.z = cos(waveWidth * v.x + waveTime) * waveHeight;
 
    gl_Position = gl_ModelViewProjectionMatrix * v;
    gl_TexCoord[0] = gl_MultiTexCoord0;
}
