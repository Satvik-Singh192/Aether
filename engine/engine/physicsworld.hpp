#pragma once
#include "core/rigidbody.hpp"
#include <vector>

class PhysicsEngine {
private: 
	std::vector<Rigidbody*> bodies;

public: 
	void addBody(Rigidbody* body);
	void update(float dt);
};