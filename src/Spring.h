#pragma once
#ifndef Spring_H
#define Spring_H

#include <memory>

class Particle;

class Spring
{
public:
	Spring(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1, double alpha);
	virtual ~Spring();
	
	std::shared_ptr<Particle> p0;
	std::shared_ptr<Particle> p1;
	double L;
	double alpha;
};

#endif
