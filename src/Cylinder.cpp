#include "Cylinder.h"

Cylinder::Cylinder(const std::shared_ptr<Shape> shape) :
	r(1.0),
	h(1.0),
	x(0.0, 0.0, 0.0),
	axis(0.0, 1.0, 0.0),
	cylinder(shape)
{}

void Cylinder::draw(std::shared_ptr<MatrixStack> M, const std::shared_ptr<Program> prog) const {
	if (cylinder) {
		M->pushMatrix();
		M->translate(float(x(0)), float(x(1)), float(x(2)));
		
		Eigen::Vector3d up(0.0, 1.0, 0.0);
		Eigen::Vector3d rotationAxis = up.cross(axis);
		double dotProduct = std::max(-1.0, std::min(1.0, up.dot(axis)));
		double rotationAngle = acos(dotProduct);

		if (rotationAxis.squaredNorm() < 1e-12) {
			if (dotProduct < 0.0) {
				M->rotate(M_PI, glm::vec3(1.0f, 0.0f, 0.0f));
			}
		}
		else {
			rotationAxis.normalize();
			M->rotate(rotationAngle, glm::vec3(rotationAxis.x(), rotationAxis.y(), rotationAxis.z()));
		}

		M->scale(r, h, r);
		glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, glm::value_ptr(M->topMatrix()));
		cylinder->draw(prog);
		M->popMatrix();
	}
}