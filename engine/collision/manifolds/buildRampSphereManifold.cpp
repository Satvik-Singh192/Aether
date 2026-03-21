#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"

//A ramp B sphere
bool buildRampSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact c;
    if (!buildRampSphereContact(A, B, c))
    {
        return false;
    }
    c.a_id = c.a->id;
    c.b_id = c.b->id;

    manifold.a = c.a;
    manifold.b = c.b;
    manifold.a_id = manifold.a->id;
    manifold.b_id = manifold.b->id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = c;

    return true;
}
