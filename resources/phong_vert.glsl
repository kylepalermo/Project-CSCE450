#version 120

attribute vec4 aPos;
attribute vec3 aNor;
uniform mat4 P;
uniform mat4 V;
uniform mat4 lightVP;
uniform mat4 M;
varying vec3 vPos;
varying vec3 vLightSpacePos;
varying vec3 vNor;

void main()
{
	vec4 posWorld = M * aPos;
	vec4 posCam = V * posWorld;
	gl_Position = P * posCam;
	vPos = posCam.xyz;
	vNor = normalize(mat3(V * M) * aNor);
	vLightSpacePos = (lightVP * posWorld).xyz;
}
