#ifndef TETRAHEDRON_H
#define TETRAHEDRON_H

#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Shape.h"
#include "Program.h"
#include "MatrixStack.h"

struct Face {
	Eigen::Vector3d x;
	Eigen::Vector3d n;
};


class Tetrahedron {
	const std::shared_ptr<Shape> tetrahedron;
public:
	Tetrahedron(const std::shared_ptr<Shape> shape);
	void draw(std::shared_ptr<MatrixStack> M, const std::shared_ptr<Program> prog) const;

	std::array<Eigen::Vector3d, 4> x;
	std::array<std::array<int, 3>, 4 > faceIndices;
	std::array<int, 4> faceOppositeIndices;

	std::array<Face, 4> getFaces() const;
};

#endif // !TETRAHEDRON_H
