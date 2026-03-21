#pragma once
#include "core/rigidbody.hpp"
#include "collision/collision.hpp"
#include "collision/contactmanifold.hpp"
#include <vector>
#include <cstddef>

class PhysicsWorld {
private: 
	std::vector<Rigidbody> bodies;
	std::vector<ContactManifold> manifolds;
	std::vector<ContactManifold> prev_manifolds;
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
	const Vec3& getGravity() const;
	void setGravity(const Vec3& new_gravity);

	void clear_manifolds();
	void generate_manifolds();
	void match_manifolds();
	void solve_manifold_velocities();
	void solve_manifold_positions();
	void warm_start_manifolds();
};