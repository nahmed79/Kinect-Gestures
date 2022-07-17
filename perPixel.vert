varying vec3 lightDir;
varying vec3 normal;

void main() {

	lightDir = normalize(vec3(gl_LightSource[0].position));
	normal = gl_Normal;

	gl_Position = ftransform();
}
