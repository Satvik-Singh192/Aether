#include "core/rigidbody.hpp"

Rigidbody::Rigidbody(Vec3 position, Vec3 velocity, float m)
	: position(position), velocity(velocity), mass(m) {}

void Rigidbody::applyForce(const Vec3& force, float dt) {
	Vec3 acceleration = force * (1.0f / mass);
	velocity = velocity + acceleration * dt;
}

void Rigidbody::update(float dt) {
	position = position + velocity * dt;
}