#include "collision/collision.hpp"

bool checkCollision(const Rigidbody& a, const Rigidbody& b, float radius) {
	Vec3 dist = a.position - b.position;
	return dist.length() < 2 * radius;
}
