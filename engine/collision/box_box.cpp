#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include <cmath>

void resolveBoxBox(Rigidbody& a, Rigidbody& b) {
    auto* ba = static_cast<BoxCollider*>(a.collider);
    auto* bb = static_cast<BoxCollider*>(b.collider); 

    Vec3 delta = b.position - a.position;

    float overlapX = (ba->halfsize.x + bb->halfsize.x) - std::abs(delta.x);
    float overlapY = (ba->halfsize.y + bb->halfsize.y) - std::abs(delta.y);
    float overlapZ = (ba->halfsize.z + bb->halfsize.z) - std::abs(delta.z);

    // no collision if any axis is separating
    if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) return;

    // sabse kam waali ko pkdo
    Vec3 normal;
    float penetration;
    if (overlapX < overlapY && overlapX < overlapZ) {
        penetration = overlapX;
        normal = Vec3(delta.x < 0 ? -1.0f : 1.0f, 0, 0); 
    } else if (overlapY < overlapZ) {
        penetration = overlapY;
        normal = Vec3(0, delta.y < 0 ? -1.0f : 1.0f, 0);
    } else {
        penetration = overlapZ;
        normal = Vec3(0, 0, delta.z < 0 ? -1.0f : 1.0f);
    }

    float total_invmass = a.inverse_mass + b.inverse_mass;
    if (total_invmass == 0.0f) return;

    Vec3 correction = normal * (penetration / total_invmass);
    a.position -= correction * a.inverse_mass;
    b.position += correction * b.inverse_mass;

    Vec3 relative_velocity = b.velocity - a.velocity;
    float velocity_along_normal = relative_velocity.dot(normal);
    if (velocity_along_normal > 0) return;

    float restitution = 0.5f;
    float impulse_magnitude = -(1 + restitution) * velocity_along_normal / total_invmass;

    Vec3 impulse_force = normal * impulse_magnitude;
    a.velocity -= impulse_force * a.inverse_mass;
    b.velocity += impulse_force * b.inverse_mass;
}
