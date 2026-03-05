#pragma once
#include "math/vec3.hpp"
#include "collider.hpp"

class Rigidbody {
public: 
	Vec3 position;
	Vec3 velocity;
	Vec3 force_accum;
	float inverse_mass;

	/*
	- Collider is a pointer cuz if we just write "Collider collider;" then the collider will always be a generic one
	  any custom collider will just get sliced when it assign it directly 
	- one more thing, we can make ghost objects by assigning the nullptr, we cant do that the other way
	*/
	Collider* collider;


	Rigidbody(const Vec3& position,
		const Vec3& velocity,
		Collider* col=nullptr,
		float mass=1.0f);

	void applyForce(const Vec3& force);
	void clearForces();
};