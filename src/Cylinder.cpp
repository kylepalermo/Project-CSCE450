#include "Cylinder.h"

Cylinder::Cylinder(const std::shared_ptr<Shape> shape) :
	r(1.0),
	h(1.0),
	x(0.0, 0.0, 0.0),
	axis(0.0, 1.0, 0.0),
	cylinder(shape)
{}

void Cylinder::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> prog) const {
	if (cylinder) {
		MV->pushMatrix();
		MV->translate(float(x(0)), float(x(1)), float(x(2)));
		// TODO: make sure rotate works the way it should here
		//MV->rotate()
		MV->scale(r, h, r);
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		cylinder->draw(prog);
		MV->popMatrix();
	}
}