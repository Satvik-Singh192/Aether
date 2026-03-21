#include "contactmanifold.hpp"
#include "collision.hpp"
bool boxboxmanifold (Rigidbody& A,Rigidbody& B, ContactManifold& manifold ) {

    Contact c;
    if(!buildBoxBoxContact(A, B,c)){
        return false;
    }
    manifold.a = c.a;
    manifold.b = c.b;

    manifold.a_id = c.a_id;
    manifold.b_id = c.b_id;

    manifold.normal = c.normal;

    manifold.pointCount = 1;

    c.normal = manifold.normal;

    
    

    manifold.points[0] = c;

    return true; 
}
