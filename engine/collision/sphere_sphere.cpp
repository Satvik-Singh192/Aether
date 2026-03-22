#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include<iostream>

bool buildSphereSphereContact(Rigidbody& a, Rigidbody& b, Contact& outContact){

    /*
    b.collider contains the pointer to its special collider but the type of b.collider is Collider* (general collider)
    so we have to tell the compiler that we know for sure that this is a pointer to SphereCollider otherwise we cant access
    special things like radius which belong to the sphere collider and not the general collider directly

    this is called downcasting //okay padh lia bhai smjh gaya//
    */
    auto* sa=static_cast<SphereCollider*>(a.collider);
    auto* sb=static_cast<SphereCollider*>(b.collider);

    if(!sa||!sb)return false;

    Vec3 diff=b.position-a.position;
    float dist_sq=diff.dot(diff);
    float radius_sum=sa->radius+sb->radius;

    if(dist_sq>=radius_sum*radius_sum)return false;

    float dist = std::sqrt(std::max(0.0f, dist_sq));
    Vec3 normal;
    if (dist <= PHYSICS_EPSILON)
        normal = Vec3(1.0f, 0.0f, 0.0f);
    else
        normal = diff * (1.0f / dist);
     outContact = Contact{};
    outContact.a = &a;
    outContact.b = &b;
    outContact.normal = normal;
    outContact.penetration = radius_sum - dist;

    outContact.contact_point = a.position + normal * (sa->radius - 0.5f * outContact.penetration);

    outContact.restitution = (a.restitution +b.restitution)*0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    return true;
}

