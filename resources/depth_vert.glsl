#version 120

attribute vec4 aPos;
uniform mat4 lightVP;
uniform mat4 M;

void main() {
	gl_Position = lightVP * (M * aPos);
}
