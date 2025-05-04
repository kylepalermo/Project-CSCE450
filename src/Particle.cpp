#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Particle.h"
#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

using namespace std;

Particle::Particle() :
	r(1.0),
	m(1.0),
	x(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true)
{
	
}

Particle::Particle(const shared_ptr<Shape> s) :
	r(1.0),
	m(1.0),
	x(0.0, 0.0, 0.0),
	v(0.0, 0.0, 0.0),
	fixed(true),
	sphere(s)
{
	
}

Particle::~Particle()
{
}

void Particle::tare()
{
	x0 = x;
	v0 = v;
}

void Particle::reset()
{
	x = x0;
	v = v0;
}

void Particle::draw(shared_ptr<MatrixStack> M, const shared_ptr<Program> prog) const
{
	if(sphere) {
		glUniform3f(prog->getUniform("kdFront"), 0.8f, 0.8f, 0.8f);
		glUniform3f(prog->getUniform("kdBack"), 0.0f, 0.0f, 0.0f);
		M->pushMatrix();
		M->translate(float(x(0)), float(x(1)), float(x(2)));
		M->scale(float(r));
		glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, glm::value_ptr(M->topMatrix()));
		sphere->draw(prog);
		M->popMatrix();
	}
}
