#include "collision/contactmanifold.hpp"
#include "collision/collision.hpp"
#include "core/box_collider.hpp"
#include <cmath>
#include "collision/contactmanifold.hpp"
#include <vector>

std:: vector<Vec3> getBoxCorners(const Rigidbody& body, const BoxCollider* box){
    std::vector<Vec3> corners;
    for(int x=-1;x<=1;x+=2)
    for(int y=-1;y<=1;y+=2)
    for(int z=-1;z<=1;z+=2)
    {
        corners.push_back(
            body.position + Vec3(
                x*box->halfsize.x,
                y*box->halfsize.y,
                z*box->halfsize.z
            )
        );
    }

    return corners;
}

bool pointInsideBox(const Vec3& p, const Rigidbody& body, const BoxCollider* box)
{
    Vec3 d = p - body.position;

    return (std::abs(d.x) <= box->halfsize.x &&
            std::abs(d.y) <= box->halfsize.y &&
            std::abs(d.z) <= box->halfsize.z);
}

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

bool buildBoxBoxManifold(Rigidbody& a, Rigidbody& b, ContactManifold& m){

    auto* ba = static_cast<BoxCollider*>(a.collider);
    auto* bb = static_cast<BoxCollider*>(b.collider);
    if (!ba || !bb) return false;
    Vec3 delta = b.position - a.position;
    float overlapX = (ba->halfsize.x + bb->halfsize.x) - std::abs(delta.x);
    float overlapY = (ba->halfsize.y + bb->halfsize.y) - std::abs(delta.y);
    float overlapZ = (ba->halfsize.z + bb->halfsize.z) - std::abs(delta.z);
    if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) return false;
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
    m.a=&a;
    m.b=&b;
    m.a_id=a.id;
    m.b_id=b.id;
    m.normal=normal;
    std::vector<Contact> candd;
    auto cornersA=getBoxCorners(a,ba);
    //points of a inside b
    for(auto& it: cornersA){
        if(pointInsideBox(it, b,bb)){
            Contact c;
            c.a = &a;
            c.b = &b;
            c.normal = normal;
            c.penetration = penetration;
            c.contact_point = it;
            c.restitution = (a.restitution + b.restitution) * 0.5f;
            c.friction_coeff = std::sqrt(a.friction * b.friction);

            candd.push_back(c);
        }
    }
    //pointd of b inside a
    auto cornersB = getBoxCorners(b, bb);
    for (auto& it : cornersB) {
        if (pointInsideBox(it, a, ba))
        {
            Contact c;
            c.a = &a;
            c.b = &b;
            c.normal = normal;
            c.penetration = penetration;
            c.contact_point = it;
            c.restitution = (a.restitution + b.restitution) * 0.5f;
            c.friction_coeff = std::sqrt(a.friction * b.friction);

            candd.push_back(c);
        }
    }
    if(candd.empty()){
        Contact c;
        c.a = &a;
        c.b = &b;
        c.normal = normal;
        c.penetration = penetration;
        c.contact_point = (a.position + b.position) * 0.5f;
        c.restitution = (a.restitution + b.restitution) * 0.5f;
        c.friction_coeff = std::sqrt(a.friction * b.friction);

        m.contacts[0] = c;
        m.contact_count = 1;
        return true;
    }

    m.contacts[0]=candd[0];
    m.contact_count=1;

    float md=0.0f;
    int bidx=-1;
    for(int i=1; i<candd.size();i++){
        float dist = (candd[i].contact_point - candd[0].contact_point).length();

        if (dist > md)
        {
            md = dist;
            bidx = i;
        }
    }
    if(bidx!=-1 && md>0.001f) {
        m.contacts[1]=candd[bidx];
        m.contact_count=2;
    }
    
    return true;

}