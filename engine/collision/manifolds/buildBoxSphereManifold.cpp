#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include "core/sphere_collider.hpp"
#include "collision/obb.hpp"
#include <algorithm>
#include <glm/common.hpp>

// A is box, B is sphere.
bool buildBoxSphereManifold(Rigidbody &boxbody, Rigidbody &spherebody, ContactManifold &m)
{
    auto *box=static_cast<BoxCollider*>(boxbody.collider);
    auto *sphere=static_cast<SphereCollider*>(spherebody.collider);
    if(!box||!sphere) return false;

    Contact c;
    OBB A=makeOBB(boxbody,box);
    Vec3 sphere_center=spherebody.position;
    Vec3 d=sphere_center-A.center;

    Vec3 local;
    local.x=d.dot(A.axis[0]);
    local.y=d.dot(A.axis[1]);
    local.z=d.dot(A.axis[2]);
    Vec3 closest;
    closest.x=glm::clamp(local.x,-box->halfsize.x, box->halfsize.x);
    closest.y=glm::clamp(local.y,-box->halfsize.y, box->halfsize.y);
    closest.z=glm::clamp(local.z,-box->halfsize.z, box->halfsize.z);
    
    Vec3 cw=A.center+A.axis[0]*closest.x+ A.axis[1]*closest.y+ A.axis[2]*closest.z;
    Vec3 diff = sphere_center-cw;
    float ds = diff.dot(diff);
    if(ds>sphere->radius*sphere->radius) return  false;
    Vec3 normal;
    float penetration;
    if(ds>1e-6f) {
        float dist=sqrt(ds);
        normal=diff*(1.0f/dist);
        penetration=sphere->radius -dist;

    }
    else {
        Vec3 absLocal=Vec3(abs(local.x),abs(local.y), abs(local.z));
        if(absLocal.x > absLocal.y && absLocal.x >absLocal.z)
            normal=A.axis[0]*(local.x >0 ? 1:-1 );
        else if(absLocal.y >absLocal.z)
            normal=A.axis[1]*(local.y>0?1:-1);
        else 
            normal=A.axis[2]*(local.z>0?1:-1);
        penetration=sphere->radius;
    }
    m.a=&boxbody;
    m.b=&spherebody;
    m.normal=normal;
    c.a=&boxbody;
    c.b=&spherebody;
    c.normal=normal;
    c.penetration=penetration;
    c.contact_point=cw;
    c.restitution=(boxbody.restitution+spherebody.restitution) *0.5f;
    c.friction_coeff=sqrt(boxbody.friction*spherebody.friction);
    m.contacts[0]=c;
    m.contact_count=1;
    return true;
}
