#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include <cmath>

namespace
{
    bool isValidFloat(float v)
    {
        return std::isfinite(v);
    }

    bool isValidVec3(const Vec3 &v)
    {
        return isValidFloat(v.x) && isValidFloat(v.y) && isValidFloat(v.z);
    }
}

bool buildRampSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact c;
    if (!buildRampSphereContact(A, B, c))
    {
        return false;
    }

    if (!isValidVec3(c.normal) || !isValidVec3(c.contact_point) ||
        !isValidFloat(c.penetration) || c.penetration <= 0.0f || c.penetration > 100.0f)
    {
        return false;
    }

    manifold.a = &A;
    manifold.b = &B;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    c.a = manifold.a;
    c.b = manifold.b;
    c.a_id = manifold.a_id;
    c.b_id = manifold.b_id;
    c.normal = manifold.normal;
    manifold.contacts[0] = c;

    return true;
}
