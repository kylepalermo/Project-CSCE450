#version 120

varying vec3 vPos; // in camera space
varying vec3 vNor; // in camera space
varying vec4 vLightSpacePos;

uniform vec3 lightPos;
uniform vec3 kdFront;
uniform vec3 kdBack;
uniform sampler2D shadowMap;

void main()
{
	
	vec3 n = normalize(vNor);
	vec3 l = normalize(lightPos - vPos);
	vec3 v = -normalize(vPos);
	vec3 h = normalize(l + v);

	vec3 kd = kdFront;
	float ln = dot(l, n);
	if (ln < 0.0) {
		kd = kdBack;
		ln = -ln;
	}

	vec3 ks = vec3(0.3, 0.3, 0.3);
	float s = 40.0;

	vec3 diffuse = kd * ln;
	vec3 specular = ks * pow(max(dot(n, h), 0.0), s);

	vec3 depthProj = vLightSpacePos.xyz / vLightSpacePos.w;
	depthProj = depthProj * 0.5 + 0.5;

	float depthMap = texture2D(shadowMap, depthProj.xy).r;
	
	if (depthProj.z - 0.00001> depthMap) {
		gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
	}
	else {
		gl_FragColor = vec4(diffuse + specular, 1.0);
	}
}
