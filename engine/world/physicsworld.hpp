#pragma once
#include"common_header.hpp"
#include "core/rigidbody.hpp"
#include "collision/collision.hpp"
#include "collision/contactmanifold.hpp"
#include "constraints/distance_constraints.hpp"
#include <vector>
#include<unordered_map>
#include <cstddef>

class PhysicsWorld {
private: 
	std::vector<std::uint32_t> bodies_to_delete;
	std::vector<Rigidbody> bodies;
	std::unordered_map<uint32_t,Rigidbody*>lookup_map;
	void syncAllPointers();
	void rebuildLookupTable();
	std::vector<ContactManifold> manifolds;
	std::vector<ContactManifold> prev_manifolds;

	std::vector<DistanceConstraint> constraints;
	Vec3 gravity;
	BodyID next_body_id;
public: 
	PhysicsWorld();
	Rigidbody* getBodyByID(uint32_t body_id);
	std::uint32_t addBody(const Rigidbody& body);
	std::uint32_t addBody(Rigidbody&& body);
	PhysicsResult deleteBody(std::uint32_t body_id); // deletes the body and removes all the constraints in which the targeted body iz a mem
	void addforce(const Vec3& force, std::uint32_t id);
	void step(float dt);
	void validate_body(Rigidbody& body);
	std::vector<Rigidbody>& getBodies();
	std::size_t getContactCount() const;
	const Vec3& getGravity() const;
	void setGravity(const Vec3& new_gravity);
	const std::vector<DistanceConstraint>& getDistanceConstraints() const;

	void clear_manifolds();
	void generate_manifolds();
	void match_manifolds();
	void solve_manifolds_vel_iteration(); //it solves for velocity (1 iteration only)
	void solve_manifolds_pos();//it is solves for position overlapping
	void warm_start_manifolds();


	//methods for constraints
	PhysicsResult addDistanceConstraints(
		std::uint32_t a_id,
		std::uint32_t b_id,
		float rest_length,
		DistanceConstraint::TYPE type,
		float stiffness,
		float damping
	);
	PhysicsResult deleteConstraint(uint32_t first_body_id,uint32_t second_body_id); //delete ALL constraints between two given bodies
	PhysicsResult deleteConstraint(uint32_t body_id);
	void solve_constraints_vel(float dt);
	void warm_start_constraints();
};

static void solve_1D_constraint(
	Rigidbody& a,
    Rigidbody& b,
    const Vec3& normal,
    float& accumulated_impulse,
    float bias,
    float min_impulse,
    float max_impulse);