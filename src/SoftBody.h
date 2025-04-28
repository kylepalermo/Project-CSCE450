#pragma once
#ifndef SOFTBODY_H
#define SOFTBODY_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Particle.h"
#include "Plane.h"
#include "Cylinder.h"
#include "Tetrahedron.h"
#include "Spring.h"
#include "Tri.h"

class SoftBody {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int rows;
	int cols;
	int tubes;

	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > springs;
	std::vector< std::vector< std::vector<Hexa> > > cells;

	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	unsigned eleBufID;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
public:
	// TOOD: make constructor accept all 6 points
	SoftBody(int rows, int cols, int tubes,
		const Eigen::Vector3d &x000,
		const Eigen::Vector3d &x111,
		double mass,
		double alpha,
		double damping,
		double pradius
	);
	virtual ~SoftBody();

	void tare();
	void reset();
	void updatePosNor();
	void updateEle();
	void step(
		double h,
		const Eigen::Vector3d &grav,
		const Eigen::Vector3d &wind,
		const std::vector< std::shared_ptr<Particle> > spheres,
		const std::vector< std::shared_ptr<Plane> > planes,
		const std::vector< std::shared_ptr<Cylinder> > cylinders,
		const std::vector< std::shared_ptr<Tetrahedron> > tetrahedrons
	);

	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p);
};


#endif // !SOFTBODY_H
