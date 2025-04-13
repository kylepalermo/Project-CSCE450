#pragma once
#ifndef Cloth_H
#define Cloth_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Spring;
class MatrixStack;
class Program;

struct TriangleSprings {
	std::shared_ptr<Spring> edgeSprings[3];
};

struct QuadSprings {
	TriangleSprings triangleSprings[2];
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
	void step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres);
	
	void init();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	
private:
	int rows;
	int cols;
	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > springs;
	std::vector< std::vector<QuadSprings> > cellSprings;
	
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
