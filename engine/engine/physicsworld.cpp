#include "engine/physicsworld.hpp"
#include<vector>

PhysicsWorld::PhysicsWorld():gravity(0.0f,-9.8f,0.0f){}

void PhysicsWorld::addBody(const Rigidbody& body) {
	bodies.push_back(body);
}

std::vector<Rigidbody>& PhysicsWorld::getBodies(){
	return bodies;
}

void PhysicsWorld::step(float dt) {
	for (auto& body:bodies) {
		if(body.inverse_mass==0.0f)continue;// skipping immovable objects jaise ground or some terrain

		Vec3 GravityForce=gravity*(1.0f/body.inverse_mass);
		body.applyForce(GravityForce);

		Vec3 acceleration=body.force_accum*body.inverse_mass;
		body.velocity+=acceleration*dt;
		body.position+=body.velocity*dt;

		body.clearForces();
	}
}
