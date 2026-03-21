#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"

// A and B are ramps.
bool buildRampRampManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact c;
    if (!buildRampRampContact(A, B, c))
    {
        return false;
    }

    manifold.a = c.a;
    manifold.b = c.b;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = c;

    return true;
}
