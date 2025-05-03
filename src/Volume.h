#ifndef VOLUME_H
#define VOLUME_H

// Very useful video:
// https://www.youtube.com/watch?v=uCaHXkS2cUg
// After implementing, test using only tetrahedral springs and doing fewer iterations
// big wins on compute are possible if those two are possible

#include <memory>

#include "Particle.h"
#include "Spring.h"

class Volume {
public:
	Volume(
		std::shared_ptr<Particle> p0,
		std::shared_ptr<Particle> p1,
		std::shared_ptr<Particle> p2,
		std::shared_ptr<Particle> p3,
		double alpha,
		std::shared_ptr<Spring> spring0,
		std::shared_ptr<Spring> spring1,
		std::shared_ptr<Spring> spring2,
		std::shared_ptr<Spring> spring3,
		std::shared_ptr<Spring> spring4,
		std::shared_ptr<Spring> spring5
	);
	virtual ~Volume();

	std::shared_ptr<Particle> p0;
	std::shared_ptr<Particle> p1;
	std::shared_ptr<Particle> p2;
	std::shared_ptr<Particle> p3;

	std::array<std::shared_ptr<Spring>, 6> springs;

	double volume0;
	double alpha;

	bool broken;
	bool getBroken();
};

#endif // !VOLUME_H


