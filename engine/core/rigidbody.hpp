#pragma once
#include "common_header.hpp"
#include "bodyid.hpp"
#include "collider.hpp"
#include <iostream>

class Rigidbody {
public: 
	BodyID id;
	
	Vec3 position;
	Vec3 velocity;
	Vec3 force_accum;
	float inverse_mass;
	float friction;
	float restitution;

	/*
	- Collider is a pointer cuz if we just write "Collider collider;" then the collider will always be a generic one
	  any custom collider will just get sliced when it assign it directly 
	- one more thing, we can make ghost objects by assigning the nullptr, we cant do that the other way
	*/
	Collider* collider;


	Rigidbody(
		BodyID body_id,
		const Vec3& position,
		const Vec3& velocity,
		Collider* col=nullptr,
		float mass=1.0f
		);

	void applyForce(const Vec3& force);
	void clearForces();
};