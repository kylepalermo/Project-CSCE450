#include "Volume.h"

Volume::Volume(
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
) :
	p0(p0),
	p1(p1),
	p2(p2),
	p3(p3),
	alpha(alpha),
	broken(false)
{
	volume0 = (1.0 / 6.0) * ((p1->x - p0->x).cross(p2->x - p0->x)).dot(p3->x - p0->x);
	springs[0] = spring0;
	springs[1] = spring1;
	springs[2] = spring2;
	springs[3] = spring3;
	springs[4] = spring4;
	springs[5] = spring5;
}

Volume::~Volume() {}

bool Volume::getBroken() {
	if (broken) {
		return true;
	}

	if (
		springs[0]->broken 
		|| springs[1]->broken 
		|| springs[2]->broken 
		|| springs[3]->broken
		|| springs[4]->broken 
		|| springs[5]->broken
		) {
		broken = true;
		return true;
	}

	return false;
}
