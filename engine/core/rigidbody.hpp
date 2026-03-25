#pragma once
#include "common_header.hpp"
#include "bodyid.hpp"
#include "collider.hpp"
#include <iostream>
#include "math/quat.hpp"
#include "math/mat3.hpp"
#include<memory>

class Rigidbody {
public: 
	BodyID id;
	
	Vec3 position;
	Vec3 velocity;
	Vec3 force_accum;
	float inverse_mass;
	float friction=PHYSICS_DEFAULT_FRICTION;
	float restitution=PHYSICS_DEFAULT_RESTITUTION;
	Quat orientation;//xi + yj+ zk
	Vec3 angvel;//w
	Vec3 acctork;
	Mat3 inverse_inertia_body;   //for local
 	Mat3 inverse_inertia_world;  //for world

	/*
	- Collider is a pointer cuz if we just write "Collider collider;" then the collider will always be a generic one
	  any custom collider will just get sliced when it assign it directly 
	- one more thing, we can make ghost objects by assigning the nullptr, we cant do that the other way
	*/
	Collider* collider; 



	Rigidbody(
		//BodyID body_id,
		const Vec3& position,
		const Vec3& velocity,
		Collider* col=nullptr,
		float mass=1.0f,
		float fric=PHYSICS_DEFAULT_FRICTION,
    	float res=PHYSICS_DEFAULT_RESTITUTION
		);
	void applyTorque(const Vec3& torque);
	void applyForce(const Vec3& force);
	void clearForces();
	void clearAccum();
	void updateworldinvinertia();
};