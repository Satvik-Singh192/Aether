#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include <cmath>
bool buildSphereSphereManifold(Rigidbody &A, Rigidbody &B, ContactManifold &m)
{
   auto* sA= static_cast<SphereCollider*>(A.collider);
   auto* sB= static_cast<SphereCollider*>(B.collider);
   if(!sA||!sB) return false;
   Contact c;
   Vec3 Apos=A.position;
   Vec3 Bpos=B.position;
   Vec3 diff =Bpos-Apos;
   float distsq=diff.dot(diff);
   float sum=sA->radius + sB->radius;
   if(distsq>sum*sum){
    return false;
   }
   float dist=sqrt(distsq);
   Vec3 normal;
   float penetration;
   if(dist>1e-6f){
    normal=diff*(1.0f/dist);
    penetration=sum-dist; // Correct: actual penetration depth

   }
   else {
    normal =Vec3(1,0,0);
    penetration=sum; // Perfect overlap case
   }
   Vec3 contact_point = Apos + normal * (sA->radius - penetration * 0.5f);
    m.a = &A;
    m.b = &B;
    m.a_id = A.id;
    m.b_id = B.id;
    m.normal = normal;

    c.a = &A;
    c.b = &B;
    c.a_id = A.id;
    c.b_id = B.id;
    c.normal = normal;
    c.penetration = penetration;
    c.contact_point = contact_point;
    c.restitution = (A.restitution + B.restitution) * 0.5f;
    c.friction_coeff = sqrt(A.friction * B.friction);

    m.contacts[0] = c;
    m.contact_count = 1;

    return true;
}
