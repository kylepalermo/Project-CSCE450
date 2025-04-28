#pragma once
#ifndef Spring_H
#define Spring_H

#include <memory>

// TODO: maybe add a separate max stretch distance parameter? Just using L isnt as flexible for different resolutions

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
	bool broken;
};

#endif
