#include <iostream>

#include "GLSL.h"
#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 1e-3;
	
	grav << 0.0, -9.8, 0.0;
	
	int rows = 100;
	int cols = 100;
	double mass = 0.1;
	double alpha = 0.0;
	double damping = 1e-5;
	double pradius = 0.01; // Particle radius, used for collisions
	Vector3d x00(-0.25, 0.5, 0.0);
	Vector3d x01(0.25, 0.5, 0.0);
	Vector3d x10(-0.25, 0.5, -0.5);
	Vector3d x11(0.25, 0.5, -0.5);
	cloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	
	sphereShape = make_shared<Shape>();
	sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");
	
	auto sphere = make_shared<Particle>(sphereShape);
	spheres.push_back(sphere);
	sphere->r = 0.1;
	sphere->x = Vector3d(0.0, 0.2, 0.0);
}

void Scene::init()
{
	sphereShape->init();
	cloth->init();
}

void Scene::tare()
{
	for(auto s : spheres) {
		s->tare();
	}
	cloth->tare();
}

void Scene::reset()
{
	t = 0.0;
	for(auto s : spheres) {
		s->reset();
	}
	cloth->reset();
}

void Scene::step()
{
	t += h;
	
	// Move the sphere
	if(!spheres.empty()) {
		auto s = spheres.front();
		s->x(2) = 0.5 * sin(0.5*t);
	}
	
	// Simulate the cloth
	cloth->step(h, grav, spheres);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
	for(auto s : spheres) {
		s->draw(MV, prog);
	}
	cloth->draw(MV, prog);
}
