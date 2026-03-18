#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include <cmath>

bool buildBoxBoxContact(Rigidbody& a, Rigidbody& b, Contact& outContact){
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
    outContact = Contact{};
    outContact.a = &a;
    outContact.b = &b;
    outContact.normal = normal;
    outContact.penetration = penetration;

    outContact.contact_point = (a.position + b.position) * 0.5f;

    outContact.restituion = (a.restituion + b.restituion) * 0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    return true;

    // float total_invmass = a.inverse_mass + b.inverse_mass;
    // if (total_invmass == 0.0f) return;

    // Vec3 correction = normal * (penetration / total_invmass);
    // a.position -= correction * a.inverse_mass;
    // b.position += correction * b.inverse_mass;

    // Vec3 relative_velocity = b.velocity - a.velocity;
    // float velocity_along_normal = relative_velocity.dot(normal);
    // if (velocity_along_normal > 0) return;

    // float restitution = PHYSICS_DEFAULT_RESTITUION;
    // float impulse_magnitude = -(1 + restitution) * velocity_along_normal / total_invmass;

    // Vec3 impulse_force = normal * impulse_magnitude;
    // a.velocity -= impulse_force * a.inverse_mass;
    // b.velocity += impulse_force * b.inverse_mass;

    // Vec3 rv = b.velocity - a.velocity;
    // Vec3 tangent = rv - normal * rv.dot(normal);
    // float tlength = tangent.length();
    // if (tlength > 1e-6f) {
    //     tangent = tangent * (1.0f / tlength);
    //     float fricvel = rv.dot(tangent);
    //     float fricmag = -fricvel / total_invmass;
    //     float mu = std::sqrt(a.friction * b.friction);
    //     float max_friction = mu * impulse_magnitude;
    //     if (fricmag > max_friction) fricmag = max_friction;
    //     if (fricmag < -max_friction) fricmag = -max_friction;

    //     Vec3 friction_impulse = tangent * fricmag;
    //     a.velocity -= friction_impulse * a.inverse_mass;
    //     b.velocity += friction_impulse * b.inverse_mass;
    // }
}
