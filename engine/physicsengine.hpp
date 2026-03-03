#pragma once
#include "rigidbody.hpp"
#include <vector>

class PhysicsEngine {
private: 
	std::vector<Rigidbody*> bodies;

public: 
	void addBody(Rigidbody* body){
		bodies.push_back(body);
	}
	void update(float dt) {
		for (auto& body : bodies) {
			body->update(dt);
		}
	}
};