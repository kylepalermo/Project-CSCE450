#ifndef TRI_H
#define TRI_H

struct Tri {
	int index0, index1, index2;
	std::shared_ptr<Particle> vertexParticles[3];
	std::shared_ptr<Spring> edgeSprings[3];
	bool broken;
	bool getBroken() {
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

struct Hexa {
	Quad quads[6];
};

#endif // !TRI_H
