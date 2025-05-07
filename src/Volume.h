#ifndef VOLUME_H
#define VOLUME_H

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


