#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"

bool buildRampBoxManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact c;
    if (!buildRampBoxContact(A, B, c))
    {
        return false;
    }

    manifold.a = c.a;
    manifold.b = c.b;
    manifold.a_id = c.a_id;
    manifold.b_id = c.b_id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = c;

    return true;
}
