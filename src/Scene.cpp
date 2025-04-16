#include <iostream>

#include "GLSL.h"
#include "Scene.h"
#include "Particle.h"
#include "Cloth.h"
#include "Shape.h"
#include "Program.h"

#define _USE_MATH_DEFINES
#include <math.h> 

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0),
	wind(0.0, 0.0, 0.0),
	windMaxMagnitude(0.005),
	windTarget(0.0, 0.0, 0.0),
	prevWindTarget(0.0, 0.0, 0.0),
	windN(1500),
	windI(0)
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
	
	int rows = 15;
	int cols = 15;
	double mass = 0.1;
	double alpha = 0.0;
	double damping = 1e-3;
	double pradius = 0.01; // Particle radius, used for collisions
	Vector3d x00(-0.25, 0.5, 0.0);
	Vector3d x01(0.25, 0.5, 0.0);
	Vector3d x10(-0.25, 0.5, -0.5);
	Vector3d x11(0.25, 0.5, -0.5);
	cloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	
	sphereShape = make_shared<Shape>();
	sphereShape->loadMesh(RESOURCE_DIR + "sphere2.obj");

	planeShape = make_shared<Shape>();
	planeShape->loadMesh(RESOURCE_DIR + "square.obj");

	cylinderShape = make_shared<Shape>();
	cylinderShape->loadMesh(RESOURCE_DIR + "cylinder.obj");
	
	auto sphere = make_shared<Particle>(sphereShape);
	spheres.push_back(sphere);
	sphere->r = 0.1;
	sphere->x = Vector3d(0.0, 0.2, 0.0);

	auto heldSphere = make_shared<Particle>(sphereShape);
	spheres.push_back(heldSphere);
	heldSphere->r = 0.1;
	heldSphere->x = Vector3d(-100.0, -100.0, -100.0); // Somewhere arbitrary

	auto ground = make_shared<Plane>(planeShape);
	planes.push_back(ground);

	auto pole = make_shared<Cylinder>(cylinderShape);
	cylinders.push_back(pole);
	pole->r = 0.1;
	pole->h = 0.2;
}

void Scene::init()
{
	sphereShape->init();
	planeShape->init();
	cylinderShape->init();
	cloth->init();
	srand(time(0));
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

void Scene::step(const std::shared_ptr<Camera> camera)
{
	t += h;
	
	// Move the sphere
	if(spheres.size() >= 2) {
		auto s = spheres.at(0);
		s->x(2) = 0.5 * sin(0.5*t);

		auto heldSphere = spheres.at(1);
		glm::vec3 cameraTranslation = camera->getTranslation() + normalize(camera->getForward());
		heldSphere->x = Eigen::Vector3d(cameraTranslation.x, cameraTranslation.y, cameraTranslation.z);
	}
	
	// Update the wind
	windI++;
	if (windI == windN) {
		prevWindTarget = windTarget;
		double windMagnitude = windMaxMagnitude * rand() / RAND_MAX;
		double windDirection = 2.0 * M_PI * rand() / RAND_MAX;
		windTarget = Vector3d(windMagnitude * cos(windDirection), 0.0, windMagnitude * sin(windDirection));

		windI = 0;
	}
	double alpha = double(windI) / double(windN);
	wind = prevWindTarget * (1.0 - alpha) + windTarget * alpha;

	// Simulate the cloth
	cloth->step(h, grav, wind, spheres, planes, cylinders);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
	for(auto s : spheres) {
		s->draw(MV, prog);
	}
	for (auto p : planes) {
		p->draw(MV, prog);
	}
	for (auto c : cylinders) {
		c->draw(MV, prog);
	}
	cloth->draw(MV, prog);
}
