#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif
#include <thread>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "GLSL.h"
#include "Program.h"
#include "Camera.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Scene.h"

using namespace std;
using namespace Eigen;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

shared_ptr<Camera> camera;
shared_ptr<Program> prog;
shared_ptr<Program> depthProg;
shared_ptr<Scene> scene;

// TODO: dot product with tris and also area
// TODO: make sure shear and nonuniform scale is not used, mvit is not available currently
// TODO: fix colors on static objects
// TODO: work on demo areas

// https://stackoverflow.com/questions/41470942/stop-infinite-loop-in-different-thread
std::atomic<bool> stop_flag;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		stop_flag = true;
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyToggles[key] = !keyToggles[key];
	switch(key) {
		case 'h':
			scene->step(camera);
			break;
		case 'r':
			scene->reset();
			break;
		case '0':
			scene->setHeldObject(NONE, camera);
			break;
		case '1':
			scene->setHeldObject(SPHERE, camera);
			break;
		case '2':
			scene->setHeldObject(TETRAHEDRON, camera);
			break;
	}
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(state == GLFW_PRESS) {
		camera->mouseMoved(float(xmouse), float(ymouse));
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	// Get the current mouse position.
	double xmouse, ymouse;
	glfwGetCursorPos(window, &xmouse, &ymouse);
	// Get current window size.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	if(action == GLFW_PRESS) {
		bool shift = mods & GLFW_MOD_SHIFT;
		bool ctrl  = mods & GLFW_MOD_CONTROL;
		bool alt   = mods & GLFW_MOD_ALT;
		camera->mouseClicked(float(xmouse), float(ymouse), shift, ctrl, alt);
	}
}

static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	// Enable alpha blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	prog = make_shared<Program>();
	prog->setVerbose(true); // Set this to true when debugging.
	prog->setShaderNames(RESOURCE_DIR + "phong_vert.glsl", RESOURCE_DIR + "phong_frag.glsl");
	prog->init();
	prog->addUniform("P");
	prog->addUniform("V");
	prog->addUniform("lightVP");
	prog->addUniform("M");
	prog->addUniform("kdFront");
	prog->addUniform("kdBack");
	prog->addUniform("lightPos");
	prog->addUniform("shadowMap");
	prog->addAttribute("aPos");
	prog->addAttribute("aNor");
	prog->setVerbose(false);

	depthProg = make_shared<Program>();
	depthProg->setShaderNames(RESOURCE_DIR + "depth_vert.glsl", RESOURCE_DIR + "depth_frag.glsl");
	depthProg->init();
	depthProg->addUniform("lightVP");
	depthProg->addUniform("M");
	depthProg->addAttribute("aPos");
	depthProg->setVerbose(false);

	depthProg->initFrameBuffer(8192, 8192, true);
	
	camera = make_shared<Camera>();
	camera->setTranslation(glm::vec3(0.0f, 1.0f, -2.0f));

	scene = make_shared<Scene>();
	scene->load(RESOURCE_DIR);
	scene->tare();
	scene->init();
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Pass 1
	auto P = make_shared<MatrixStack>();
	auto V = make_shared<MatrixStack>();
	auto M = make_shared<MatrixStack>();

	P->pushMatrix();
	P->multMatrix(glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 1000.0f));
	V->pushMatrix();
	glm::vec3 lightTarget(0.0f, 0.0f, 0.0f);
	glm::vec3 lightEye = lightTarget - 10.0f * glm::vec3(0.0f, -1.0f, -1.0f);
	glm::vec3 lightUp(0.0f, 1.0f, 0.0f);
	V->multMatrix(glm::lookAt(lightEye, lightTarget, lightUp));
	M->pushMatrix();

	glm::mat4 lightVP = P->topMatrix() * V->topMatrix();

	depthProg->bindFrameBuffer();
	glViewport(0, 0, 8192, 8192);
	glClear(GL_DEPTH_BUFFER_BIT);

	depthProg->bind();
	glUniformMatrix4fv(depthProg->getUniform("lightVP"), 1, GL_FALSE, glm::value_ptr(lightVP));
	scene->draw(M, depthProg);
	depthProg->unbind();
	depthProg->unbindFrameBuffer();
	P->popMatrix();
	V->popMatrix();
	M->popMatrix();

	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Use the window size for camera.
	glfwGetWindowSize(window, &width, &height);
	camera->setAspect((float)width/(float)height);
	camera->pollKeyPresses(window);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles[(unsigned)'c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles[(unsigned)'z']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	// Apply camera transforms
	P->pushMatrix();
	camera->applyProjectionMatrix(P);
	V->pushMatrix();
	camera->applyViewMatrix(V);
	M->pushMatrix();

	// Draw scene
	prog->bind();
	glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	glUniformMatrix4fv(prog->getUniform("V"), 1, GL_FALSE, glm::value_ptr(V->topMatrix()));
	glUniformMatrix4fv(prog->getUniform("lightVP"), 1, GL_FALSE, glm::value_ptr(lightVP));
	lightEye = glm::vec3(M->topMatrix() * V->topMatrix() * glm::vec4(lightEye, 1.0f));
	glUniform3fv(prog->getUniform("lightPos"), 1, glm::value_ptr(lightEye));
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depthProg->getTextureID());
	scene->draw(M, prog);
	prog->unbind();
	
	//////////////////////////////////////////////////////
	// Cleanup
	//////////////////////////////////////////////////////
	
	// Pop stacks
	P->popMatrix();
	V->popMatrix();
	M->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

void stepperFunc()
{
	auto stepInterval = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
		std::chrono::duration<double>(1.0 / 300.0) // 1/300 second
	);
	auto nextStepTime = std::chrono::high_resolution_clock::now();

	while(!stop_flag) {
		if(keyToggles[(unsigned)' ']) {
			auto now = std::chrono::high_resolution_clock::now();
			if (now >= nextStepTime) {
				scene->step(camera);
				nextStepTime += stepInterval;
			}
			else {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		cout << "Please specify the resource directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
	
	// Set error callback.
	glfwSetErrorCallback(error_callback);
	// Initialize the library.
	if(!glfwInit()) {
		return -1;
	}
	// Create a windowed mode window and its OpenGL context.
	window = glfwCreateWindow(640, 480, "Kyle Palermo", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return -1;
	}
	// Make the window's context current.
	glfwMakeContextCurrent(window);
	// Initialize GLEW.
	glewExperimental = true;
	if(glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return -1;
	}
	glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	// Set vsync.
	glfwSwapInterval(1);
	// Set keyboard callback.
	glfwSetKeyCallback(window, key_callback);
	// Set char callback.
	glfwSetCharCallback(window, char_callback);
	// Set cursor position callback.
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// Set mouse button callback.
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	// Initialize scene.
	init();
	// Start simulation thread.
	stop_flag = false;
	thread stepperThread(stepperFunc);
	// Loop until the user closes the window.
	while(!glfwWindowShouldClose(window)) {
		if(!glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
			// Render scene.
			render();
			// Swap front and back buffers.
			glfwSwapBuffers(window);
		}
		// Poll for and process events.
		glfwPollEvents();
	}
	// Quit program.
	stop_flag = true;
	stepperThread.join();
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
