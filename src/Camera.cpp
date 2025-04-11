#define _USE_MATH_DEFINES
#include <cmath> 
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include "Camera.h"
#include "MatrixStack.h"

Camera::Camera() :
	aspect(1.0f),
	fovy((float)(45.0*M_PI/180.0)),
	znear(0.1f),
	zfar(1000.0f),
	yaw(0.0f),
	pitch(0.0f),
	translations(0.0f, 0.0f, 0.0f),
	mousePrev(0.0f, 0.0f),
	tPrev(0.0f),
	rfactor(0.005f),
	tfactor(1.0f)
{
}

Camera::~Camera()
{
}

void Camera::mouseClicked(float x, float y, bool shift, bool ctrl, bool alt)
{
	mousePrev.x = x;
	mousePrev.y = y;
}

void Camera::mouseMoved(float x, float y)
{
	glm::vec2 mouseCurr(x, y);
	glm::vec2 dv = mouseCurr - mousePrev;
	yaw += rfactor * dv.x;
	pitch += rfactor * dv.y;

	mousePrev = mouseCurr;
}

void Camera::pollKeyPresses(GLFWwindow *window) {
	float t = glfwGetTime();
	float dt = t - tPrev;
	tPrev = t;

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		glm::vec3 frontward(sin(yaw), 0, cos(yaw));
		translations += tfactor * dt * frontward;
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		glm::vec3 leftward(cos(yaw), 0, -sin(yaw));
		translations += tfactor * dt * leftward;
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		glm::vec3 backward(-sin(yaw), 0, -cos(yaw));
		translations += tfactor * dt * backward;
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		glm::vec3 rightward(-cos(yaw), 0, sin(yaw));
		translations += tfactor * dt * rightward;
	}
}

void Camera::applyProjectionMatrix(std::shared_ptr<MatrixStack> P) const
{
	// Modify provided MatrixStack
	P->multMatrix(glm::perspective(fovy, aspect, znear, zfar));
}

void Camera::applyViewMatrix(std::shared_ptr<MatrixStack> MV) const
{
	glm::vec3 forward(sin(yaw), pitch, cos(yaw));

	glm::vec3 eye = translations;
	glm::vec3 target = eye + forward;
	glm::vec3 up(0, 1, 0);

	MV->multMatrix(glm::lookAt(eye, target, up));
}

glm::vec3 Camera::getTranslation() const {
	return translations;
}

glm::vec3 Camera::getForward() const {
	return glm::vec3(sin(yaw), pitch, cos(yaw));
}