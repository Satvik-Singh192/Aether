#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include <cmath>

bool buildBoxBoxContact(Rigidbody& a, Rigidbody& b, Contact& outContact){
    auto* ba = static_cast<BoxCollider*>(a.collider);
    auto* bb = static_cast<BoxCollider*>(b.collider); 

    if (!ba || !bb) return false;

    Vec3 delta = b.position - a.position;

    float overlapX = (ba->halfsize.x + bb->halfsize.x) - std::abs(delta.x);
    float overlapY = (ba->halfsize.y + bb->halfsize.y) - std::abs(delta.y);
    float overlapZ = (ba->halfsize.z + bb->halfsize.z) - std::abs(delta.z);

    // no collision if any axis is separating
    if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) return false;

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

    outContact.restitution = (a.restitution+ b.restitution)*0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    return true;

}

