#include "contactmanifold.hpp"
#include "collision.hpp"
#include <algorithm> 
//A is box, B is sphere
bool buildSphereSphereManifold(Rigidbody& A,Rigidbody& B,ContactManifold& manifold){
    Contact c;
    if(! buildSphereSphereContact(A, B,c)){
        return false;
    }
    manifold.a=c.a;
    manifold.b=c.b;
    manifold.a_id=c.a_id;
    manifold.b_id=c.b_id;
    manifold.normal=c.normal;
    manifold.pointCount=1;
    manifold.points[0]=c;

    return true;
}


