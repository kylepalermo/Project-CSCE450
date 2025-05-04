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

				// necessary to anchor at times while friction is not yet implemented
				if (false &&
					(i == 0 || i == rows - 1) && 
					(j == 0 || j == cols - 1) && 
					(k == 0 || k == tubes - 1)
				) {
					p->fixed = true;
				}
				else {
					p->fixed = false;
				}
				
				particles.push_back(p);
			}
		}
	}

	unordered_map<int64_t, shared_ptr<Spring>> springMap;
	auto makeKey = [&](int particleIndex0, int particleIndex1) {
		if (particleIndex0 > particleIndex1) {
			swap(particleIndex0, particleIndex1);
		}
		return (int64_t(particleIndex0) << 32) | int64_t(int32_t(particleIndex1));
	};
	auto addSpring = [&](int particleIndex0, int particleIndex1, double springAlpha) {
		shared_ptr<Spring> spring = make_shared<Spring>(
			particles[particleIndex0],
			particles[particleIndex1],
			springAlpha
		);
		springMap[makeKey(particleIndex0, particleIndex1)] = spring;
		springs.push_back(spring);
	};
	auto getSpring = [&](int particleIndex0, int particleIndex1) {
		return springMap[makeKey(particleIndex0, particleIndex1)];
	};

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
				// abd
				H.quads[0].tris[0].index0 = a;
				H.quads[0].tris[0].index1 = b;
				H.quads[0].tris[0].index2 = d;
				// cdb
				H.quads[0].tris[1].index0 = c;
				H.quads[0].tris[1].index1 = d;
				H.quads[0].tris[1].index2 = b;

				// bfgc
				// cbg
				H.quads[1].tris[0].index0 = c;
				H.quads[1].tris[0].index1 = b;
				H.quads[1].tris[0].index2 = g;
				// fgb
				H.quads[1].tris[1].index0 = f;
				H.quads[1].tris[1].index1 = g;
				H.quads[1].tris[1].index2 = b;

				// ehgf
				// feg
				H.quads[2].tris[0].index0 = f;
				H.quads[2].tris[0].index1 = e;
				H.quads[2].tris[0].index2 = h;
				// hge
				H.quads[2].tris[1].index0 = h;
				H.quads[2].tris[1].index1 = g;
				H.quads[2].tris[1].index2 = f;

				// adhe
				// ade
				H.quads[3].tris[0].index0 = a;
				H.quads[3].tris[0].index1 = d;
				H.quads[3].tris[0].index2 = e;
				// hed
				H.quads[3].tris[1].index0 = h;
				H.quads[3].tris[1].index1 = e;
				H.quads[3].tris[1].index2 = d;

				// cghd
				// cgd
				H.quads[4].tris[0].index0 = c;
				H.quads[4].tris[0].index1 = g;
				H.quads[4].tris[0].index2 = d;
				// hdg
				H.quads[4].tris[1].index0 = h;
				H.quads[4].tris[1].index1 = d;
				H.quads[4].tris[1].index2 = g;

				// aefb
				// aeb
				H.quads[5].tris[0].index0 = a;
				H.quads[5].tris[0].index1 = e;
				H.quads[5].tris[0].index2 = b;
				// fbe
				H.quads[5].tris[1].index0 = f;
				H.quads[5].tris[1].index1 = b;
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

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			for (int k = 0; k < tubes; k++) {
				int index0 = i * cols * tubes + j * tubes + k;

				// Structure
				// x
				if (i < rows - 1) {
					int index1 = (i + 1) * cols * tubes + j * tubes + k;
					addSpring(index0, index1, alpha);
				}
				// y
				if (j < cols - 1) {
					int index1 = i * cols * tubes + (j + 1) * tubes + k;
					addSpring(index0, index1, alpha);
				}
				// z
				if (k < tubes - 1) {
					int index1 = i * cols * tubes + j * tubes + (k + 1);
					addSpring(index0, index1, alpha);
				}
				// Shear
				// xy
				if (i < rows - 1 && j < cols - 1) {
					int index1 = (i + 1) * cols * tubes + (j + 1) * tubes + k;
					addSpring(index0, index1, alpha);
					int index2 = (i + 1) * cols * tubes + j * tubes + k;
					int index3 = i * cols * tubes + (j + 1) * tubes + k;
					addSpring(index2, index3, alpha);
				}
				// yz
				if (j < cols - 1 && k < tubes - 1) {
					int index1 = i * cols * tubes + (j + 1) * tubes + (k + 1);
					addSpring(index0, index1, alpha);
					int index2 = i * cols * tubes + (j + 1) * tubes + k;
					int index3 = i * cols * tubes + j * tubes + (k + 1);
					addSpring(index2, index3, alpha);
				}
				// xz
				if (i < rows - 1 && k < tubes - 1) {
					int index1 = (i + 1) * cols * tubes + j * tubes + (k + 1);
					addSpring(index0, index1, alpha);
					int index2 = (i + 1) * cols * tubes + j * tubes + k;
					int index3 = i * cols * tubes + j * tubes + (k + 1);
					addSpring(index2, index3, alpha);
				}
				// xyz
				/*
				if (i < rows - 1 && j < cols - 1 && k < tubes - 1) {
					int index1 = (i + 1) * cols * tubes + (j + 1) * tubes + (k + 1);
					addSpring(index0, index1, alpha);
					int index2 = (i + 1) * cols * tubes + j * tubes + k;
					int index3 = i * cols * tubes + (j + 1) * tubes + (k + 1);
					addSpring(index2, index3, alpha);
					int index4 = i * cols * tubes + (j + 1) * tubes + k;
					int index5 = (i + 1) * cols * tubes + j * tubes + (k + 1);
					addSpring(index4, index5, alpha);
					int index6 = i * cols * tubes + j * tubes + (k + 1);
					int index7 = (i + 1) * cols * tubes + (j + 1) * tubes + k;
					addSpring(index6, index7, alpha);
				}
				*/
				// Bend
				/*
				// x
				if (i < rows - 2) {
					int index1 = (i + 2) * cols * tubes + j * tubes + k;
					addSpring(index0, index1, alpha);
				}
				// y
				if (j < cols - 2) {
					int index1 = i * cols * tubes + (j + 2) * tubes + k;
					addSpring(index0, index1, alpha);
				}
				// z
				if (k < tubes - 2) {
					int index1 = i * cols * tubes + j * tubes + (k + 2);
					addSpring(index0, index1, alpha);
				}
				*/
			}
		}
	}

	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			for (int k = 0; k < tubes - 1; k++) {
				int a = i * cols * tubes + j * tubes + k;
				int b = (i + 1) * cols * tubes + j * tubes + k;
				int c = (i + 1) * cols * tubes + (j + 1) * tubes + k;
				int d = i * cols * tubes + (j + 1) * tubes + k;
				int e = i * cols * tubes + j * tubes + (k + 1);
				int f = (i + 1) * cols * tubes + j * tubes + (k + 1);
				int g = (i + 1) * cols * tubes + (j + 1) * tubes + (k + 1);
				int h = i * cols * tubes + (j + 1) * tubes + (k + 1);

				// tetra at a
				volumes.push_back(std::make_shared<Volume>(
					particles[a],
					particles[b],
					particles[d],
					particles[e],
					0.0,
					getSpring(a, b),
					getSpring(a, d),
					getSpring(a, e),
					getSpring(b, d),
					getSpring(b, e),
					getSpring(d, e)
				));
				// tetra at c
				volumes.push_back(std::make_shared<Volume>(
					particles[c],
					particles[b],
					particles[g],
					particles[d],
					0.0,
					getSpring(c, b),
					getSpring(c, g),
					getSpring(c, d),
					getSpring(b, g),
					getSpring(b, d),
					getSpring(g, d)
				));
				// tetra at f
				volumes.push_back(std::make_shared<Volume>(
					particles[f],
					particles[b],
					particles[e],
					particles[g],
					0.0,
					getSpring(f, b),
					getSpring(f, e),
					getSpring(f, g),
					getSpring(b, e),
					getSpring(b, g),
					getSpring(e, g)
				));
				// tetra at h
				volumes.push_back(std::make_shared<Volume>(
					particles[h],
					particles[d],
					particles[g],
					particles[e],
					0.0,
					getSpring(h, d),
					getSpring(h, g),
					getSpring(h, e),
					getSpring(d, g),
					getSpring(d, e),
					getSpring(g, e)
				));
				// center tetra
				volumes.push_back(std::make_shared<Volume>(
					particles[b],
					particles[d],
					particles[e],
					particles[g],
					0.0,
					getSpring(b, d),
					getSpring(b, e),
					getSpring(b, g),
					getSpring(d, e),
					getSpring(d, g),
					getSpring(e, g)
				));
			}
		}
	}

	for (int i = 0; i < rows - 1; i++) {
		for (int j = 0; j < cols - 1; j++) {
			for (int k = 0; k < tubes - 1; k++) {
				Hexa &H = cells[i][j][k];

				int a = i * cols * tubes + j * tubes + k;
				int b = (i + 1) * cols * tubes + j * tubes + k;
				int c = (i + 1) * cols * tubes + (j + 1) * tubes + k;
				int d = i * cols * tubes + (j + 1) * tubes + k;
				int e = i * cols * tubes + j * tubes + (k + 1);
				int f = (i + 1) * cols * tubes + j * tubes + (k + 1);
				int g = (i + 1) * cols * tubes + (j + 1) * tubes + (k + 1);
				int h = i * cols * tubes + (j + 1) * tubes + (k + 1);

				// abcd
				// abd
				H.quads[0].tris[0].edgeSprings[0] = getSpring(a, b);
				H.quads[0].tris[0].edgeSprings[1] = getSpring(a, d);
				H.quads[0].tris[0].edgeSprings[2] = getSpring(b, d);
				// cdb
				H.quads[0].tris[1].edgeSprings[0] = getSpring(c, d);
				H.quads[0].tris[1].edgeSprings[1] = getSpring(c, b);
				H.quads[0].tris[1].edgeSprings[2] = getSpring(d, b);

				// bfgc
				// cbg
				H.quads[1].tris[0].edgeSprings[0] = getSpring(c, b);
				H.quads[1].tris[0].edgeSprings[1] = getSpring(c, g);
				H.quads[1].tris[0].edgeSprings[2] = getSpring(b, g);
				// fgb
				H.quads[1].tris[1].edgeSprings[0] = getSpring(f, g);
				H.quads[1].tris[1].edgeSprings[1] = getSpring(f, b);
				H.quads[1].tris[1].edgeSprings[2] = getSpring(g, b);

				// ehgf
				// feg
				H.quads[2].tris[0].edgeSprings[0] = getSpring(f, e);
				H.quads[2].tris[0].edgeSprings[1] = getSpring(f, g);
				H.quads[2].tris[0].edgeSprings[2] = getSpring(e, g);
				// hge
				H.quads[2].tris[1].edgeSprings[0] = getSpring(h, g);
				H.quads[2].tris[1].edgeSprings[1] = getSpring(h, e);
				H.quads[2].tris[1].edgeSprings[2] = getSpring(g, e);

				// adhe
				// ade
				H.quads[3].tris[0].edgeSprings[0] = getSpring(a, d);
				H.quads[3].tris[0].edgeSprings[1] = getSpring(a, e);
				H.quads[3].tris[0].edgeSprings[2] = getSpring(d, e);
				// hed
				H.quads[3].tris[1].edgeSprings[0] = getSpring(h, e);
				H.quads[3].tris[1].edgeSprings[1] = getSpring(h, d);
				H.quads[3].tris[1].edgeSprings[2] = getSpring(e, d);

				// cghd
				// cgd
				H.quads[4].tris[0].edgeSprings[0] = getSpring(c, g);
				H.quads[4].tris[0].edgeSprings[1] = getSpring(c, d);
				H.quads[4].tris[0].edgeSprings[2] = getSpring(g, d);
				// hdg
				H.quads[4].tris[1].edgeSprings[0] = getSpring(h, d);
				H.quads[4].tris[1].edgeSprings[1] = getSpring(h, g);
				H.quads[4].tris[1].edgeSprings[2] = getSpring(d, g);

				// aefb
				// aeb
				H.quads[5].tris[0].edgeSprings[0] = getSpring(a, e);
				H.quads[5].tris[0].edgeSprings[1] = getSpring(a, b);
				H.quads[5].tris[0].edgeSprings[2] = getSpring(e, b);
				// fbe
				H.quads[5].tris[1].edgeSprings[0] = getSpring(f, b);
				H.quads[5].tris[1].edgeSprings[1] = getSpring(f, e);
				H.quads[5].tris[1].edgeSprings[2] = getSpring(b, e);
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
	// Need to work on this later
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
						if (!cells[i][j][k].quads[quadIndex].tris[triIndex].getBroken()) {
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
	for (shared_ptr<Particle> particle : particles) {
		if (particle->fixed) {
			particle->v = particle->v0;
			continue;
		}

		Vector3d particleForce = particle->m * grav - particle->d * particle->v + wind;
		particle->v += (h / particle->m) * particleForce;
		particle->p = particle->x;
		particle->x += h * particle->v;
	}

	for (int i = 0; i < 10; i++) {
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
	}

	for (shared_ptr<Volume> volume : volumes) {
		if (volume->getBroken()) {
			continue;
		}
		shared_ptr<Particle> p0 = volume->p0;
		shared_ptr<Particle> p1 = volume->p1;
		shared_ptr<Particle> p2 = volume->p2;
		shared_ptr<Particle> p3 = volume->p3;

		double volumeCurrent = (1.0 / 6.0) * ((p1->x - p0->x).cross(p2->x - p0->x)).dot(p3->x - p0->x);
		double C = 6.0 * (volumeCurrent - volume->volume0);

		Vector3d deltaC0 = (p3->x - p1->x).cross(p2->x - p1->x);
		Vector3d deltaC1 = (p2->x - p0->x).cross(p3->x - p0->x);
		Vector3d deltaC2 = (p3->x - p0->x).cross(p1->x - p0->x);
		Vector3d deltaC3 = (p1->x - p0->x).cross(p2->x - p0->x);

		double w0 = 1.0 / p0->m;
		double w1 = 1.0 / p1->m;
		double w2 = 1.0 / p2->m;
		double w3 = 1.0 / p3->m;
		
		double lambda = -C / 
			(w0 * deltaC0.squaredNorm() + 
				w1 * deltaC1.squaredNorm() + 
				w2 * deltaC2.squaredNorm() + 
				w3 * deltaC3.squaredNorm() + 
				volume->alpha / (h * h));

		if (!p0->fixed) {
			p0->x += lambda * w0 * deltaC0;
		}
		if (!p1->fixed) {
			p1->x += lambda * w1 * deltaC1;
		}
		if (!p2->fixed) {
			p2->x += lambda * w2 * deltaC2;
		}
		if (!p3->fixed) {
			p3->x += lambda * w3 * deltaC3;
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

void SoftBody::draw(shared_ptr<MatrixStack> M, const shared_ptr<Program> p) {
	updatePosNor();
	updateEle();

	// Draw mesh
	glUniform3f(p->getUniform("kdFront"), 0.894f, 0.882f, 0.792f);
	glUniform3f(p->getUniform("kdBack"), 0.776f, 0.843f, 0.835f);
	M->pushMatrix();
	glUniformMatrix4fv(p->getUniform("M"), 1, GL_FALSE, glm::value_ptr(M->topMatrix()));
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
	M->popMatrix();
}