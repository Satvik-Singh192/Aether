#pragma once
#include "core/rigidbody.hpp"
#include <vector>

class PhysicsWorld {
private: 
	std::vector<Rigidbody> bodies;
	Vec3 gravity;
public: 
	PhysicsWorld();
	void addBody(const Rigidbody& body);
	void step(float dt);
	void validate_body(Rigidbody& body);
	std::vector<Rigidbody>& getBodies();
};