#pragma once
#include "core/rigidbody.hpp"
#include <vector>

class PhysicsWorld {
private: 
	std::vector<Rigidbody> bodies;
	Vec3 gravity;
	int bodies_size=0;

public: 
	PhysicsWorld();
	void addBody(const Rigidbody& body);
	void step(float dt);

	std::vector<Rigidbody>& getBodies();
};