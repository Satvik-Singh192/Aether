#pragma once
#include "math/vec3.hpp"

class Rigidbody {
public: 
	Vec3 position;
	Vec3 velocity;
	float mass;
	Rigidbody(Vec3 position = Vec3(), Vec3 velocity = Vec3(), float m = 1.0f);
	void applyForce(const Vec3& force, float dt);
	void update(float dt);
};