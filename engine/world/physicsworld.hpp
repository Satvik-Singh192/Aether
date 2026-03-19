#pragma once
#include "core/rigidbody.hpp"
#include "collision/collision.hpp"
#include <vector>
#include <cstddef>

class PhysicsWorld {
private: 
	std::vector<Rigidbody> bodies;
	std::vector<Contact>contacts;
	Vec3 gravity;
	BodyID next_body_id;
public: 
	PhysicsWorld();
	void addBody(const Rigidbody& body);
	void addBody(Rigidbody&& body);
	void step(float dt);
	void validate_body(Rigidbody& body);
	std::vector<Rigidbody>& getBodies();
	std::size_t getContactCount() const;

	void clear_contacts();
	void generate_contacts();
	void solve_contacts(); //solves for vel
	void solve_position();//tis solves for position overlap
};