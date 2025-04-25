#pragma once
#ifndef Cloth_H
#define Cloth_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "Plane.h"
#include "Cylinder.h"
#include "Tetrahedron.h"
#include "Spring.h"

class Particle;
class MatrixStack;
class Program;

struct Tri {
	int index0, index1, index2;
	std::shared_ptr<Particle> vertexParticles[3];
	std::shared_ptr<Spring> edgeSprings[3];
	bool broken;
	bool getBroken(){
		if (broken) {
			return true;
		}

		if (edgeSprings[0]->broken || edgeSprings[1]->broken || edgeSprings[2]->broken) {
			broken = true;
			return true;
		}

		return false;
	}
};

struct Quad {
	Tri tris[2];
};

class Cloth
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Cloth(int rows, int cols,
		  const Eigen::Vector3d &x00,
		  const Eigen::Vector3d &x01,
		  const Eigen::Vector3d &x10,
		  const Eigen::Vector3d &x11,
		  double mass,
		  double alpha,
		  double damping,
		  double pradius);
	virtual ~Cloth();
	
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
	
private:
	int rows;
	int cols;
	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > springs;
	std::vector< std::vector<Quad> > cells;
	
	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	unsigned eleBufID;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
};

#endif
