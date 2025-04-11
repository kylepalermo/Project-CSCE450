#pragma once
#ifndef Camera_H
#define Camera_H

#include <memory>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

class MatrixStack;

class Camera
{
public:	
	Camera();
	virtual ~Camera();
	void setTranslation(const glm::vec3 &t) { translations = t; };
	void setAspect(float a) { aspect = a; };
	void setFovy(float f) { fovy = f; };
	void setZnear(float z) { znear = z; };
	void setZfar(float z) { zfar = z; };
	void setRotationFactor(float f) { rfactor = f; };
	void setTranslationFactor(float f) { tfactor = f; };
	void mouseClicked(float x, float y, bool shift, bool ctrl, bool alt);
	void mouseMoved(float x, float y);
	void pollKeyPresses(GLFWwindow *window);
	void applyProjectionMatrix(std::shared_ptr<MatrixStack> P) const;
	void applyViewMatrix(std::shared_ptr<MatrixStack> MV) const;
	glm::vec3 getTranslation() const;
	glm::vec3 getForward() const;
	
private:
	float aspect;
	float fovy;
	float znear;
	float zfar;
	float yaw;
	float pitch;
	glm::vec3 translations;
	glm::vec2 mousePrev;
	float tPrev;
	float rfactor;
	float tfactor;
};

#endif
