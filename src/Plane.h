#ifndef PLANE_H
#define PLANE_H

#include <vector>
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

class Plane {
	const std::shared_ptr<Shape> plane;
public:
	Plane(const std::shared_ptr<Shape> shape);
	void draw(std::shared_ptr<MatrixStack> M, const std::shared_ptr<Program> prog) const;
	
	Eigen::Vector3d x; // position
	Eigen::Vector3d n; // normal
};

#endif // !PLANE_H
