#include "Plane.h"

Plane::Plane(const std::shared_ptr<Shape> shape) :
	x(0.0, 0.0, 0.0),
	n(0.0, 1.0, 0.0),
	plane(shape)
{}

void Plane::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const
{
	if (plane) {
		MV->pushMatrix();
		MV->translate(float(x(0)), float(x(1)), float(x(2)));
		MV->scale(1e5f); // might have to fix if not rendering
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		plane->draw(prog);
		MV->popMatrix();
	}
}
