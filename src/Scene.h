#pragma once
#ifndef Scene_H
#define Scene_H

#include <vector>
#include <memory>
#include <string>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#include "Camera.h"
#include "Plane.h"
#include "Cylinder.h"
#include "Tetrahedron.h"
#include "SoftBody.h"


class Cloth;
class Particle;
class MatrixStack;
class Program;
class Shape;

enum HeldObject {
	NONE,
	SPHERE,
	TETRAHEDRON
};

class Scene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Scene();
	virtual ~Scene();
	
	void load(const std::string &RESOURCE_DIR);
	void init();
	void tare();
	void reset();
	void step(const std::shared_ptr<Camera> camera);
	void setHeldObject(HeldObject heldObject, const std::shared_ptr<Camera> camera);
	void draw(std::shared_ptr<MatrixStack> M, const std::shared_ptr<Program> prog) const;
	
	double getTime() const { return t; }
private:
	double t;
	double h;
	Eigen::Vector3d grav;

	Eigen::Vector3d wind;
	double windMaxMagnitude;
	Eigen::Vector3d windTarget;
	Eigen::Vector3d prevWindTarget;
	int windN;
	int windI;

	HeldObject heldObject; // not the best naming
	
	std::shared_ptr<Shape> sphereShape;
	std::shared_ptr<Shape> planeShape;
	std::shared_ptr<Shape> cylinderShape;
	std::shared_ptr<Shape> tetrahedronShape;

	std::vector< std::shared_ptr<Cloth> > cloths;
	std::vector< std::shared_ptr<SoftBody> > softBodies;

	std::vector< std::shared_ptr<Particle> > spheres;
	std::vector< std::shared_ptr<Plane> > planes;
	std::vector< std::shared_ptr<Cylinder> > cylinders;
	std::vector< std::shared_ptr<Tetrahedron> > tetrahedrons;
};

#endif
