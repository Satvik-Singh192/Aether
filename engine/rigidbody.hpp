#pragma once
#include "vec3.hpp"

class Rigidbody {
public: 
	Vec3 position;
	Vec3 velocity;
	float mass;
	Rigidbody(Vec3 position = Vec3(), Vec3 velocity = Vec3(), float m = 1.0f)
		: position(position), velocity(velocity), mass(m) { }
	void applyForce(const Vec3& force, float dt ){
		Vec3 acceleration = force * (1.0f / mass);
		velocity =  velocity + acceleration * dt;
	}
	void update(float dt){
		position =  position + velocity * dt;
	}
};