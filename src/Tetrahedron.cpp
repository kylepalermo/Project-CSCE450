#include "Tetrahedron.h"

Tetrahedron::Tetrahedron(const std::shared_ptr<Shape> shape) : 
	tetrahedron(shape)
{
	x = { {
		Eigen::Vector3d(0.0, 0.0, 0.0),
		Eigen::Vector3d(1.0, 0.0, 0.0),
		Eigen::Vector3d(0.0, 1.0, 0.0),
		Eigen::Vector3d(0.0, 0.0, 1.0)
	} };

	faceIndices = { {
		{{0, 1, 2}},
		{{0, 2, 3}},
		{{0, 3, 1}},
		{{1, 3, 2}}
	} };
	faceOppositeIndices = { {
		3, 1, 2, 0
	} };
}

void Tetrahedron::draw(std::shared_ptr<MatrixStack> M, const std::shared_ptr<Program> prog) const {
	if (tetrahedron) {
		int kdFrontID = prog->getUniform("kdFront");
		if (kdFrontID != -1) {
			glUniform3f(kdFrontID, 0.8f, 0.8f, 0.8f);
		}
		int kdBackID = prog->getUniform("kdBack");
		if (kdBackID != -1) {
			glUniform3f(kdBackID, 0.0f, 0.0f, 0.0f);
		}

		M->pushMatrix();
		
		glm::vec3 p0(x[0].x(), x[0].y(), x[0].z());
		glm::vec3 p1(x[1].x(), x[1].y(), x[1].z());
		glm::vec3 p2(x[2].x(), x[2].y(), x[2].z());
		glm::vec3 p3(x[3].x(), x[3].y(), x[3].z());

		glm::mat4 transform = glm::identity<glm::mat4>();
		transform[0] = glm::vec4(p1 - p0, 0.0f);
		transform[1] = glm::vec4(p2 - p0, 0.0f);
		transform[2] = glm::vec4(p3 - p0, 0.0f);
		transform[3] = glm::vec4(p0, 1.0f);
		
		M->multMatrix(transform);
		
		glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, glm::value_ptr(M->topMatrix()));
		tetrahedron->draw(prog);
		M->popMatrix();
	}
}

std::array<Face, 4> Tetrahedron::getFaces() const {
	std::array<Face, 4> faces;
	for (int i = 0; i < faces.size(); i++) {
		Eigen::Vector3d p0 = x[faceIndices[i][0]];
		Eigen::Vector3d p1 = x[faceIndices[i][1]];
		Eigen::Vector3d p2 = x[faceIndices[i][2]];

		Eigen::Vector3d n = (p1 - p0).cross(p2 - p0).normalized();
		
		if (n.dot(x[faceOppositeIndices[i]] - p0) > 0.0) {
			n *= -1.0;
		}
		faces[i] = Face{
			p0,
			n
		};
	}
	return faces;
}