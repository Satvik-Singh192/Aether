#pragma once
#include "core/rigidbody.hpp"
#include "collision/collision.hpp"
#include "collision/contactmanifold.hpp"
#include <vector>
#include <cstddef>

class PhysicsWorld {
private: 
	std::vector<Rigidbody> bodies;
	std::vector<Contact>contacts;
	std::vector<Contact>prev_contacts;
	std::vector<ContactManifold> manifolds;
	Vec3 gravity;
	BodyID next_body_id;
public: 
	PhysicsWorld();
	std::uint32_t addBody(const Rigidbody& body);
	std::uint32_t addBody(Rigidbody&& body);
	void addforce(const Vec3& force, std::uint32_t id);
	void step(float dt);
	void validate_body(Rigidbody& body);
	std::vector<Rigidbody>& getBodies();
	std::size_t getContactCount() const;

	void clear_contacts();
	void generate_contacts();
	void match_contacts();
	void solve_contacts(); //solves for vel
	void solve_position();//tis solves for position overlap
	void warm_start_contacts();
};