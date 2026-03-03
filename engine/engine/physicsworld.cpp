#include "engine/physicsworld.hpp"

void PhysicsEngine::addBody(Rigidbody* body) {
	bodies.push_back(body);
}

void PhysicsEngine::update(float dt) {
	for (auto& body : bodies) {
		body->update(dt);
	}
}
