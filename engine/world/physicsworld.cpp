#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

using CollisionKey = std::pair<ShapeType, ShapeType>;
using ContactBuilder = bool(*)(Rigidbody&, Rigidbody&, Contact&);

struct CollisionKeyHash {
	size_t operator()(const CollisionKey& key) const {
		return (static_cast<size_t>(key.first) << 8) ^ static_cast<size_t>(key.second);
	}
};

static bool buildBoxSphereContact(Rigidbody& box_body, Rigidbody& sphere_body, Contact& outContact) {
	if (!buildSphereBoxContact(sphere_body, box_body, outContact)) {
		return false;
	}
	std::swap(outContact.a, outContact.b);
	outContact.normal = outContact.normal * -1.0f;
	return true;
}

const std::unordered_map<CollisionKey, ContactBuilder, CollisionKeyHash> collision_builders = {
	{{ShapeType::Box, ShapeType::Box}, buildBoxBoxContact},
	{{ShapeType::Box, ShapeType::Sphere}, buildBoxSphereContact},
	{{ShapeType::Sphere, ShapeType::Sphere}, buildSphereSphereContact},
};

static Vec3 crossVec(const Vec3& a, const Vec3& b) {
	return Vec3(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

static Vec3 anyPerpendicular(const Vec3& n) {
	if (std::fabs(n.x) < 0.577f) {
		return crossVec(n, Vec3(1.0f, 0.0f, 0.0f)).normalized();
	}
	return crossVec(n, Vec3(0.0f, 1.0f, 0.0f)).normalized();
}

PhysicsWorld::PhysicsWorld():gravity(0.0f,PHYSICS_GRAVITY,0.0f){}

void PhysicsWorld::addBody(const Rigidbody& body) {
	bodies.push_back(body);
}

std::vector<Rigidbody>& PhysicsWorld::getBodies(){
	return bodies;
}

void PhysicsWorld::generateContacts() {
	contacts.clear();

	for (size_t i = 0; i + 1 < bodies.size(); ++i) {
		for (size_t j = i + 1; j < bodies.size(); ++j) {
			Collider* ca = bodies[i].collider;
			Collider* cb = bodies[j].collider;
			if (!ca || !cb) {
				continue;
			}

			Rigidbody* first_body = &bodies[i];
			Rigidbody* second_body = &bodies[j];
			ShapeType first_type = ca->type;
			ShapeType second_type = cb->type;

			if (static_cast<int>(first_type) > static_cast<int>(second_type)) {
				std::swap(first_type, second_type);
				std::swap(first_body, second_body);
			}

			auto builder_it = collision_builders.find({first_type, second_type});
			if (builder_it == collision_builders.end()) {
				continue;
			}

			Contact contact;
			if (builder_it->second(*first_body, *second_body, contact)) {
				contacts.push_back(contact);
			}
		}
	}
}

void PhysicsWorld::preStepContacts(float dt) {
	constexpr float baumgarte = 0.2f;
	constexpr float slop = 0.005f;
	constexpr float bounce_threshold = 0.5f;

	for (auto& c : contacts) {
		float invMassA = c.a->inverse_mass;
		float invMassB = c.b->inverse_mass;
		float invMassSum = invMassA + invMassB;

		if (invMassSum <= PHYSICS_EPSILON) {
			c.normalMass = 0.0f;
			c.tangentMass1 = 0.0f;
			c.tangentMass2 = 0.0f;
			c.bias = 0.0f;
			c.velocitybias = 0.0f;
			continue;
		}

		c.tangent1 = anyPerpendicular(c.normal);
		c.tangent2 = crossVec(c.normal, c.tangent1).normalized();

		c.normalMass = 1.0f / invMassSum;
		c.tangentMass1 = 1.0f / invMassSum;
		c.tangentMass2 = 1.0f / invMassSum;

		float penetrationError = std::max(0.0f, c.penetration - slop);
		c.bias = (dt > PHYSICS_EPSILON) ? (baumgarte / dt) * penetrationError : 0.0f;

		Vec3 rv = c.b->velocity - c.a->velocity;
		float vn = rv.dot(c.normal);
		c.velocitybias = (vn < -bounce_threshold) ? (-c.restitution * vn) : 0.0f;

		c.normalimpulse = 0.0f;
		c.tangent1impluse = 0.0f;
		c.tangent2impulse = 0.0f;
	}
}

void PhysicsWorld::warmStartContacts() {
	for (auto& c : contacts) {
		Vec3 normalImpulse = c.normal * c.normalimpulse;
		Vec3 tangentImpulse = c.tangent1 * c.tangent1impluse + c.tangent2 * c.tangent2impulse;
		Vec3 totalImpulse = normalImpulse + tangentImpulse;

		c.a->velocity -= totalImpulse * c.a->inverse_mass;
		c.b->velocity += totalImpulse * c.b->inverse_mass;
	}
}

void PhysicsWorld::solveNormal(Contact& c) {
	if (c.normalMass <= PHYSICS_EPSILON) {
		return;
	}

	Vec3 rv = c.b->velocity - c.a->velocity;
	float vn = rv.dot(c.normal);

	float lambda = -(vn + c.bias - c.velocitybias) * c.normalMass;

	float oldImpulse = c.normalimpulse;
	c.normalimpulse = std::max(0.0f, oldImpulse + lambda);
	lambda = c.normalimpulse - oldImpulse;

	Vec3 impulse = c.normal * lambda;
	c.a->velocity -= impulse * c.a->inverse_mass;
	c.b->velocity += impulse * c.b->inverse_mass;
}

void PhysicsWorld::solveFriction(Contact& c) {
	if (c.tangentMass1 <= PHYSICS_EPSILON || c.tangentMass2 <= PHYSICS_EPSILON) {
		return;
	}

	float maxFriction = c.friction * c.normalimpulse;

	Vec3 rv1 = c.b->velocity - c.a->velocity;
	float vt1 = rv1.dot(c.tangent1);
	float lambda1 = -vt1 * c.tangentMass1;

	float oldT1 = c.tangent1impluse;
	//c.tangent1impluse = std::clamp(oldT1 + lambda1, -maxFriction, maxFriction);
	c.tangent1impluse=std::max(-maxFriction,oldT1+lambda1);
		c.tangent1impluse=std::min(maxFriction,oldT1+lambda1);

	lambda1 = c.tangent1impluse - oldT1;

	Vec3 impulse1 = c.tangent1 * lambda1;
	c.a->velocity -= impulse1 * c.a->inverse_mass;
	c.b->velocity += impulse1 * c.b->inverse_mass;

	Vec3 rv2 = c.b->velocity - c.a->velocity;
	float vt2 = rv2.dot(c.tangent2);
	float lambda2 = -vt2 * c.tangentMass2;

	float oldT2 = c.tangent2impulse;
	//c.tangent2impulse = std::clamp(oldT2 + lambda2, -maxFriction, maxFriction);
		c.tangent2impulse=std::max(-maxFriction,oldT1+lambda1);
		c.tangent2impulse=std::min(maxFriction,oldT1+lambda1);
	lambda2 = c.tangent2impulse - oldT2;

	Vec3 impulse2 = c.tangent2 * lambda2;
	c.a->velocity -= impulse2 * c.a->inverse_mass;
	c.b->velocity += impulse2 * c.b->inverse_mass;
}

void PhysicsWorld::solveContacts(int iterations) {
	for (int i = 0; i < iterations; ++i) {
		for (auto& c : contacts) {
			solveNormal(c);
			solveFriction(c);
		}
	}
}

void PhysicsWorld::validate_body(Rigidbody& body){
	if(is_corrupt(body.position)){
		std::cout << "body position corrupted: " <<body.position<<'\n';
		body.position=Vec3();
	}
	if(is_corrupt(body.velocity)){
		std::cout << "body velocity corrupted: " <<body.velocity<<'\n';
		body.velocity=Vec3();
	}
	if(is_corrupt(body.force_accum)){
		std::cout << "body force accumulator corrupted: " <<body.force_accum<<'\n';
		body.force_accum=Vec3();
	}
}

void PhysicsWorld::step(float dt) {
	for (auto& body : bodies) {
		if (body.inverse_mass == 0.0f) {
			continue;
		}

		Vec3 gravityForce = gravity * (1.0f / body.inverse_mass);
		body.applyForce(gravityForce);
	}

	for (auto& body : bodies) {
		if (body.inverse_mass == 0.0f) {
			continue;
		}

		Vec3 acceleration = body.force_accum * body.inverse_mass;
		body.velocity += acceleration * dt;
	}

	// generateContacts();
	// preStepContacts(dt);
	// warmStartContacts();
	// solveContacts(8);

	for (auto& body : bodies) {
		if (body.inverse_mass == 0.0f) {
			continue;
		}

		body.position += body.velocity * dt;
		body.clearForces();
	}

	for (auto& body : bodies) {
		validate_body(body);
	}
}
