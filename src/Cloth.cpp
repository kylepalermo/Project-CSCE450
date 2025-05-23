#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double alpha,
			 double damping,
			 double pradius)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(alpha >= 0.0);
	assert(damping >= 0.0);
	assert(pradius >= 0.0);
	
	this->rows = rows;
	this->cols = cols;

	cells.resize(rows - 1, vector<Quad>(cols - 1));
	
	// Create particles
	int nVerts = rows*cols; // Total number of vertices
	double particleM = mass / nVerts;
	for(int i = 0; i < rows; ++i) {
		double posBeta = double(i) / (rows - 1);
		for(int j = 0; j < cols; ++j) {
			double posAlpha = double(j) / (cols - 1);

			Vector3d topEdgePos = (1.0 - posAlpha) * x00 + posAlpha * x01;
			Vector3d bottomEdgePos = (1.0 - posAlpha) * x10 + posAlpha * x11;
			Vector3d pos = (1.0 - posBeta) * topEdgePos + posBeta * bottomEdgePos;

			auto p = make_shared<Particle>();
			p->x = pos;
			p->v = Vector3d::Zero();
			p->p = pos;
			p->x0 = pos;
			p->v0 = Vector3d::Zero();
			p->m = particleM;
			p->r = pradius;
			p->d = damping;
			if (i == 0 && (j == 0 || j == cols - 1)) {
				p->fixed = true;
			}
			else {
				p->fixed = false;
			}
			particles.push_back(p);
		}
	}

	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			Quad &Q = cells[i][j];

			int a = i * cols		+ j;
			int b = (i + 1) * cols	+ j;
			int c = (i + 1) * cols	+ (j + 1);
			int d = i * cols		+ (j + 1);
			
			//abd
			Q.tris[0].index0 = a;
			Q.tris[0].index1 = b;
			Q.tris[0].index2 = c;
			//cdb
			Q.tris[1].index0 = c;
			Q.tris[1].index1 = d;
			Q.tris[1].index2 = b;
		}
	}

	// Create x springs
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols - 1; j++) {
			auto spring = make_shared<Spring>(
				particles.at(i * cols + j),
				particles.at(i * cols + (j + 1)),
				alpha
			);
			springs.push_back(spring);
			if (i < rows - 1) {
				cells.at(i).at(j).tris[0].edgeSprings[0] = spring;
			}
			if (i > 0) {
				cells.at(i - 1).at(j).tris[1].edgeSprings[0] = spring;
			}
		}
	}
	
	// Create y springs
	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols; j++) {
			auto spring = make_shared<Spring>(
				particles.at(i * cols + j),
				particles.at((i + 1) * cols + j),
				alpha
			);
			springs.push_back(spring);
			if (j < rows - 1) {
				cells.at(i).at(j).tris[0].edgeSprings[1] = spring;
			}
			if (j > 0) {
				cells.at(i).at(j - 1).tris[1].edgeSprings[1] = spring;
			}
		}
	}

	// Create shear springs
	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			auto spring1 = make_shared<Spring>(
				particles.at(i * cols + j),
				particles.at((i + 1) * cols + (j + 1)),
				alpha
			);
			springs.push_back(spring1);
			auto spring2 = make_shared<Spring>(
				particles.at((i + 1) * cols + j),
				particles.at(i * cols + (j + 1)),
				alpha
			);
			springs.push_back(spring2);
			cells.at(i).at(j).tris[0].edgeSprings[2] = spring2;
			cells.at(i).at(j).tris[1].edgeSprings[2] = spring2;
		}
	}

	// Create x bending springs
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols - 2; j++) {
			auto spring = make_shared<Spring>(
				particles.at(i * cols + j),
				particles.at(i * cols + (j + 2)),
				alpha
			);
			springs.push_back(spring);
		}
	}
	
	// Create y bending springs
	for (int i = 0; i < rows - 2; i++) {
		for (int j = 0; j < cols; j++) {
			auto spring = make_shared<Spring>(
				particles.at(i * cols + j),
				particles.at((i + 2) * cols + j),
				alpha
			);
			springs.push_back(spring);
		}
	}
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	updateEle();
	
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0f));
			texBuf.push_back(j/(cols-1.0f));
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(auto p : particles) {
		p->tare();
	}
}

void Cloth::reset()
{
	for(auto p : particles) {
		p->reset();
	}
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x; // updated position
			posBuf[3*k+0] = float(x(0));
			posBuf[3*k+1] = float(x(1));
			posBuf[3*k+2] = float(x(2));
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     / | \
			// u0 /__|__\ u1
			//    \  |  /
			//     \ | /
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = float(nor(0));
			norBuf[3*k+1] = float(nor(1));
			norBuf[3*k+2] = float(nor(2));
		}
	}
}

// TODO: can rewrite this to take better advantage of tri struct. be careful not
// to add to much compute when doing so.
void Cloth::updateEle() {
	eleBuf.clear();
	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			bool brokenEdge = false;
			for (int edge = 0; edge < 3; edge++) {
				if (cells.at(i).at(j).tris[0].edgeSprings[edge]->broken) {
					brokenEdge = true;
					break;
				}
			}
			if (!brokenEdge) {
				eleBuf.push_back(i * cols + j);
				eleBuf.push_back((i + 1) * cols + j);
				eleBuf.push_back(i * cols + (j + 1));
			}

			brokenEdge = false;
			for (int edge = 0; edge < 3; edge++) {
				if (cells.at(i).at(j).tris[1].edgeSprings[edge]->broken) {
					brokenEdge = true;
					break;
				}
			}
			if (!brokenEdge) {
				eleBuf.push_back(i * cols + (j + 1));
				eleBuf.push_back((i + 1) * cols + j);
				eleBuf.push_back((i + 1) * cols + (j + 1));
			}
		}
	}
}

void Cloth::step(
	double h,
	const Eigen::Vector3d &grav,
	const Eigen::Vector3d &wind,
	const std::vector< std::shared_ptr<Particle> > spheres,
	const std::vector< std::shared_ptr<Plane> > planes,
	const std::vector< std::shared_ptr<Cylinder> > cylinders,
	const std::vector< std::shared_ptr<Tetrahedron> > tetrahedrons
) {
	vector<Vector3d> windForces(particles.size(), Vector3d::Zero());
	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			const Quad &Q = cells[i][j];
			for (int t = 0; t < 2; t++) {
				const Tri &T = Q.tris[t];

				Vector3d &x0 = particles[T.index0]->x;
				Vector3d &x1 = particles[T.index1]->x;
				Vector3d &x2 = particles[T.index2]->x;

				Vector3d normal = (x1 - x0).cross(x2 - x0);
				double area = normal.norm();
				normal.normalize();
				double pressure = normal.dot(wind);
				Vector3d triForce = normal * (pressure * area);
				
				windForces[T.index0] += triForce / 3.0;
				windForces[T.index1] += triForce / 3.0;
				windForces[T.index2] += triForce / 3.0;
			}
		}
	}

	for (int i = 0; i < particles.size(); i++) {
		shared_ptr<Particle> particle = particles.at(i);
		if (particle->fixed) {
			particle->v = particle->v0;
			continue;
		}

		Vector3d particleForce = particle->m * grav 
			- particle->d * particle->v 
			+ windForces[i];
		particle->v += (h / particle->m) * particleForce;
		particle->p = particle->x;
		particle->x += h * particle->v;
	}

	for (shared_ptr<Spring> spring : springs) {
		if (spring->broken) {
			continue;
		}

		Vector3d deltax = spring->p1->x - spring->p0->x;
		double l = deltax.norm();
		if (l >= spring->L * 2.5) {
			spring->broken = true;
			continue;
		}

		double C = l - spring->L;
		Vector3d deltaC0 = -deltax / l;
		Vector3d deltaC1 = deltax / l;

		double w0 = 1.0 / spring->p0->m;
		double w1 = 1.0 / spring->p1->m;
		double lambda = -C / (w0 + w1 + spring->alpha / (h * h));

		if (!spring->p0->fixed) {
			spring->p0->x += lambda * w0 * deltaC0;
		}
		if (!spring->p1->fixed) {
			spring->p1->x += lambda * w1 * deltaC1;
		}
	}

	for (shared_ptr<Particle> sphere : spheres) {
		for (shared_ptr<Particle> particle : particles) {
			if (particle->fixed) {
				continue;
			}

			if ((particle->x - sphere->x).norm() < particle->r + sphere->r) {
				particle->x = (sphere->r + particle->r) * (particle->x - sphere->x).normalized() + sphere->x;
			}
		}
	}

	for (shared_ptr<Plane> plane : planes) {
		for (shared_ptr<Particle> particle : particles) {
			if (particle->fixed) {
				continue;
			}

			double distance = (particle->x - plane->x).dot(plane->n);
			if (distance < particle->r) {
				particle->x = particle->x - plane->n * (distance - particle->r);
			}
		}
	}

	for (shared_ptr<Cylinder> cylinder : cylinders) {
		for (shared_ptr<Particle> particle : particles) {
			if (particle->fixed) {
				continue;
			}

			double topDistance = (particle->x - (cylinder->x + cylinder->h * cylinder->axis)).dot(cylinder->axis) - particle->r;
			double bottomDistance = (particle->x - cylinder->x).dot(-cylinder->axis) - particle->r;

			Vector3d d = particle->x - cylinder->x;
			d = d - (d.dot(cylinder->axis)) * cylinder->axis;
			double radialDistance = d.norm() - cylinder->r - particle->r;

			if (topDistance < 0.0 && bottomDistance < 0.0 && radialDistance < 0.0) {
				if (topDistance > bottomDistance && topDistance > radialDistance) {
					// Move above cylinder
					particle->x = particle->x - cylinder->axis * topDistance;
				}
				else if (bottomDistance > radialDistance) {
					// Move below cylinder
					particle->x = particle->x + cylinder->axis * bottomDistance;
				}
				else {
					// Move beside cylinder
					particle->x = particle->x - d.normalized() * radialDistance;
				}
			}
		}
	}

	for (shared_ptr<Tetrahedron> tetrahedron : tetrahedrons) {
		std::array<Face, 4> faces = tetrahedron->getFaces();
		for (shared_ptr<Particle> particle : particles) {
			if (particle->fixed) {
				continue;
			}
			
			double constexpr negInfinity = -std::numeric_limits<double>::infinity();
			double maxDistance = negInfinity;
			int maxDistanceIndex = -1;
			for (int i = 0; i < faces.size(); i++) {
				double distance = (particle->x - faces[i].x).dot(faces[i].n) - particle->r;
				if (distance > maxDistance) {
					maxDistance = distance;
					maxDistanceIndex = i;
				}
			}
			if (maxDistance < 0.0) {
				particle->x = particle->x - maxDistance * faces[maxDistanceIndex].n;
			}
		}
	}

	for (shared_ptr<Particle> particle : particles) {
		particle->v = (1 / h) * (particle->x - particle->p);
	}
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_DYNAMIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> M, const shared_ptr<Program> p) {
	updatePosNor();
	updateEle();

	// Draw mesh
	int kdFrontID = p->getUniform("kdFront");
	if (kdFrontID != -1) {
		glUniform3f(kdFrontID, 0.894f, 0.882f, 0.792f);
	}
	int kdBackID = p->getUniform("kdBack");
	if (kdBackID != -1) {
		glUniform3f(kdBackID, 0.776f, 0.843f, 0.835f);
	}
	M->pushMatrix();
	glUniformMatrix4fv(p->getUniform("M"), 1, GL_FALSE, glm::value_ptr(M->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	if (h_nor >= 0) {
		glEnableVertexAttribArray(h_nor);
		glBindBuffer(GL_ARRAY_BUFFER, norBufID);
		glBufferData(GL_ARRAY_BUFFER, norBuf.size() * sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	int h_tex = p->getAttribute("aTex");
	if(h_tex >= 0) {
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_DYNAMIC_DRAW);
	glDrawElements(GL_TRIANGLES, eleBuf.size(), GL_UNSIGNED_INT, 0);
	if(h_tex >= 0) {
		glDisableVertexAttribArray(h_tex);
	}
	if (h_nor >= 0) {
		glDisableVertexAttribArray(h_nor);
	}
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	M->popMatrix();
}
