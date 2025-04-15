#ifndef CYLINDER_H
#define CYLINDER_H

#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

class Cylinder {
	const std::shared_ptr<Shape> cylinder;
public:
	Cylinder(const std::shared_ptr<Shape> shape);
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const;

	double r; // radius
	double h; // height
	Eigen::Vector3d x; // position
	Eigen::Vector3d axis; // axis
};

#endif // !CYLINDER_H
