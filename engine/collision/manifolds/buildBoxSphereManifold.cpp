#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"

// A is box, B is sphere.
bool buildBoxSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &manifold)
{
    Contact c;
    if (!buildSphereBoxContact(B, A, c))
    {
        return false;
    }

    // buildSphereBoxContact returns A=sphere, B=box; flip to A=box, B=sphere manifold convention.
    std::swap(c.a, c.b);
    std::swap(c.a_id, c.b_id);
    c.normal = c.normal * -1.0f;

    manifold.a = c.a;
    manifold.b = c.b;
    manifold.a_id = A.id;
    manifold.b_id = B.id;
    manifold.normal = c.normal;
    manifold.contact_count = 1;
    manifold.contacts[0] = c;

    return true;
}
