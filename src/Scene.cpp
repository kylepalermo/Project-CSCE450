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
	windN(3000), // currently 10 seconds
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
	shared_ptr<Cloth> sphereCloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	cloths.push_back(sphereCloth);

	x00 = Vector3d(-1.25, 0.5, 0.0);
	x01 = Vector3d(-0.75, 0.5, 0.0);
	x10 = Vector3d(-1.25, 0.5, -0.5);
	x11 = Vector3d(-0.75, 0.5, -0.5);
	shared_ptr<Cloth> cutCloth = make_shared<Cloth>(rows, cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	cloths.push_back(cutCloth);

	x00 = Vector3d(-2.0, 1.0, 0.0);
	x01 = Vector3d(-2.0, 0.5, 0.0);
	x10 = Vector3d(-3.0, 1.0, 0.0);
	x11 = Vector3d(-3.0, 0.5, 0.0);
	shared_ptr<Cloth> windCloth = make_shared<Cloth>(2 * rows, 2 * cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	cloths.push_back(windCloth);

	
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

	auto flagpole = make_shared<Cylinder>(cylinderShape);
	cylinders.push_back(flagpole);
	flagpole->r = 0.025;
	flagpole->h = 1.1;
	flagpole->x = Vector3d(-1.975, 0.0, 0.0);
}

void Scene::init()
{
	sphereShape->init();
	planeShape->init();
	cylinderShape->init();
	for (shared_ptr<Cloth> cloth : cloths) {
		cloth->init();
	}
	srand(time(0));
}

void Scene::tare()
{
	for(auto s : spheres) {
		s->tare();
	}
	for (shared_ptr<Cloth> cloth : cloths) {
		cloth->tare();
	}
}

void Scene::reset()
{
	t = 0.0;
	for(auto s : spheres) {
		s->reset();
	}
	for (shared_ptr<Cloth> cloth : cloths) {
		cloth->reset();
	}
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
	double alpha = min(1.0, 2.0 * double(windI) / double(windN));
	wind = prevWindTarget * (1.0 - alpha) + windTarget * alpha;

	// Simulate the cloths
	for (shared_ptr<Cloth> cloth : cloths) {
		cloth->step(h, grav, wind, spheres, planes, cylinders);
	}
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
	for (shared_ptr<Cloth> cloth : cloths) {
		cloth->draw(MV, prog);
	}
}
