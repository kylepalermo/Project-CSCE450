#include "SoftBody.h"

using namespace std;
using namespace Eigen;

SoftBody::SoftBody(int rows, int cols, int tubes,
	const Vector3d &x000,
	const Vector3d &x111,
	double mass,
	double alpha,
	double damping,
	double pradius) {
	assert(rows > 1);
	assert(cols > 1);
	assert(tubes > 1);
	assert(mass > 0.0);
	assert(alpha >= 0.0);
	assert(damping >= 0.0);
	assert(pradius >= 0.0);

	this->rows = rows;
	this->cols = cols;
	this->tubes = tubes;

	cells.resize(
		rows - 1, 
		vector< vector<Hexa>>(
			cols - 1, 
			vector<Hexa>(
				tubes - 1
			)
		)
	);

	int nVerts = rows * cols * tubes;
	double particleM = mass / nVerts;
	for (int i = 0; i < rows; i++) {
		double posGamma = double(i) / (rows - 1);
		for (int j = 0; j < cols; j++) {
			double posBeta = double(j) / (cols - 1);
			for (int k = 0; k < tubes; k++) {
				double posAlpha = double(k) / (tubes - 1);

				Vector3d pos(
					(1.0 - posGamma) * x000.x() + posGamma * x111.x(),
					(1.0 - posBeta) * x000.y() + posBeta * x111.y(),
					(1.0 - posAlpha) * x000.z() + posAlpha * x111.z()
				);

				auto p = make_shared<Particle>();
				p->x = pos;
				p->v = Vector3d::Zero();
				p->p = pos;
				p->x0 = pos;
				p->v0 = Vector3d::Zero();
				p->m = particleM;
				p->r = pradius;
				p->d = damping;

				// necessary to anchor while friction is not yet implemented
				if (i == 0 && j == 0 && k == 0) {
					p->fixed = true;
				}
				
				particles.push_back(p);
			}
		}
	}

	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			for (int k = 0; k < tubes - 1; k++) {
				Hexa &H = cells[i][j][k];

				int a = i * cols * tubes		 + j * tubes		 + k;
				int b = (i + 1) * cols * tubes	 + j * tubes		 + k;
				int c = (i + 1) * cols * tubes	 + (j + 1) * tubes	 + k;
				int d = i * cols * tubes		 + (j + 1) * tubes	 + k;
				int e = i * cols * tubes		 + j * tubes		 + (k + 1);
				int f = (i + 1) * cols * tubes	 + j * tubes		 + (k + 1);
				int g = (i + 1) * cols * tubes	 + (j + 1) * tubes	 + (k + 1);
				int h = i * cols * tubes		 + (j + 1) * tubes	 + (k + 1);

				// abcd
				// abc
				H.quads[0].tris[0].index0 = a;
				H.quads[0].tris[0].index1 = b;
				H.quads[0].tris[0].index2 = c;
				// cda
				H.quads[0].tris[1].index0 = c;
				H.quads[0].tris[1].index1 = d;
				H.quads[0].tris[1].index2 = a;

				// bfgc
				// bfg
				H.quads[1].tris[0].index0 = b;
				H.quads[1].tris[0].index1 = f;
				H.quads[1].tris[0].index2 = g;
				// gcb
				H.quads[1].tris[1].index0 = g;
				H.quads[1].tris[1].index1 = c;
				H.quads[1].tris[1].index2 = b;

				// fehg
				// feh
				H.quads[2].tris[0].index0 = f;
				H.quads[2].tris[0].index1 = e;
				H.quads[2].tris[0].index2 = h;
				// hgf
				H.quads[2].tris[1].index0 = h;
				H.quads[2].tris[1].index1 = g;
				H.quads[2].tris[1].index2 = f;

				// eadh
				// ead
				H.quads[3].tris[0].index0 = e;
				H.quads[3].tris[0].index1 = a;
				H.quads[3].tris[0].index2 = d;
				// dhe
				H.quads[3].tris[1].index0 = d;
				H.quads[3].tris[1].index1 = h;
				H.quads[3].tris[1].index2 = e;

				// dcgh
				// dcg
				H.quads[4].tris[0].index0 = d;
				H.quads[4].tris[0].index1 = c;
				H.quads[4].tris[0].index2 = g;
				// ghd
				H.quads[4].tris[1].index0 = g;
				H.quads[4].tris[1].index1 = h;
				H.quads[4].tris[1].index2 = d;

				// efba
				// efb
				H.quads[5].tris[0].index0 = e;
				H.quads[5].tris[0].index1 = f;
				H.quads[5].tris[0].index2 = b;
				// bae
				H.quads[5].tris[1].index0 = b;
				H.quads[5].tris[1].index1 = a;
				H.quads[5].tris[1].index2 = e;

				for (int q = 0; q < 6; q++) {
					for (int t = 0; t < 2; t++) {
						Tri &T = H.quads[q].tris[t];
						T.vertexParticles[0] = particles[T.index0];
						T.vertexParticles[1] = particles[T.index1];
						T.vertexParticles[2] = particles[T.index2];
					}
				}
			}
		}
	}

	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts * 3);
	norBuf.resize(nVerts * 3);
	updatePosNor();
	updateEle();

	// Texture coordinates (placeholder, may be implemented later)
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			for (int k = 0; k < tubes; k++) {
				texBuf.push_back(i / (rows - 1.0f));
				texBuf.push_back(j / (cols - 1.0f));
			}
		}
	}

}

SoftBody::~SoftBody() {}

void SoftBody::tare() {
	for (auto p : particles) {
		p->tare();
	}
}

void SoftBody::reset() {
	for (auto p : particles) {
		p->reset();
	}
}

void SoftBody::updatePosNor() {
	// Position
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			for (int k = 0; k < tubes; k++) {
				int index = i * cols * tubes + j * tubes + k;
				Vector3d x = particles[index]->x;
				posBuf[3 * index + 0] = float(x(0));
				posBuf[3 * index + 1] = float(x(1));
				posBuf[3 * index + 2] = float(x(2));
			}
		}
	}

	// Normal
	vector<Vector3d> normalAccumulator(rows * cols * tubes, Vector3d::Zero());
	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			for (int k = 0; k < tubes - 1; k++) {
				Hexa &H = cells[i][j][k];
				for (int q = 0; q < 6; q++) {
					for (int t = 0; t < 2; t++) {
						Tri &T = H.quads[q].tris[t];

						int i0 = T.index0;
						int i1 = T.index1;
						int i2 = T.index2;

						Vector3d &x0 = particles[i0]->x;
						Vector3d &x1 = particles[i1]->x;
						Vector3d &x2 = particles[i2]->x;

						Vector3d triNormal = (x1 - x0).cross(x2 - x0);

						normalAccumulator[i0] += triNormal;
						normalAccumulator[i1] += triNormal;
						normalAccumulator[i2] += triNormal;
					}
				}
			}
		}
	}

	for (int i = 0; i < rows * cols * tubes; i++) {
		Vector3d n = normalAccumulator[i].normalized();

		norBuf[3 * i + 0] = float(n.x());
		norBuf[3 * i + 1] = float(n.y());
		norBuf[3 * i + 2] = float(n.z());
	}
}

void SoftBody::updateEle() {
	eleBuf.clear();

	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			for (int k = 0; k < tubes - 1; k++) {
				for (int quadIndex = 0; quadIndex < 6; quadIndex++) {
					for (int triIndex = 0; triIndex < 2; triIndex++) {
						if (true || !cells[i][j][k].quads[quadIndex].tris[triIndex].getBroken()) {
							eleBuf.push_back(cells[i][j][k].quads[quadIndex].tris[triIndex].index0);
							eleBuf.push_back(cells[i][j][k].quads[quadIndex].tris[triIndex].index1);
							eleBuf.push_back(cells[i][j][k].quads[quadIndex].tris[triIndex].index2);
						}
					}
				}
			}
		}
	}
}

void SoftBody::step(
	double h,
	const Eigen::Vector3d &grav,
	const Eigen::Vector3d &wind,
	const std::vector< std::shared_ptr<Particle> > spheres,
	const std::vector< std::shared_ptr<Plane> > planes,
	const std::vector< std::shared_ptr<Cylinder> > cylinders,
	const std::vector< std::shared_ptr<Tetrahedron> > tetrahedrons
) {
	// TODO
}

void SoftBody::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);

	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size() * sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);

	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size() * sizeof(float), &texBuf[0], GL_STATIC_DRAW);

	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size() * sizeof(unsigned int), &eleBuf[0], GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	assert(glGetError() == GL_NO_ERROR);
}

void SoftBody::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) {
	updatePosNor();
	updateEle();

	// Draw mesh
	glUniform3f(p->getUniform("kdFront"), 0.894f, 0.882f, 0.792f);
	glUniform3f(p->getUniform("kdBack"), 0.776f, 0.843f, 0.835f);
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size() * sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_tex = p->getAttribute("aTex");
	if (h_tex >= 0) {
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size() * sizeof(unsigned int), &eleBuf[0], GL_DYNAMIC_DRAW);
	glDrawElements(GL_TRIANGLES, eleBuf.size(), GL_UNSIGNED_INT, 0);
	if (h_tex >= 0) {
		glDisableVertexAttribArray(h_tex);
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}