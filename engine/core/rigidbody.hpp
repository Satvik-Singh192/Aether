#pragma once
#include "math/vec3.hpp"

class Rigidbody {
public: 
	Vec3 position;
	Vec3 velocity;
	Vec3 force_accum;
	float inverse_mass;

	Rigidbody(const Vec3& position,
		const Vec3& velocity,
		float mass=1.0f);

	void applyForce(const Vec3& force);
	void clearForces();
};