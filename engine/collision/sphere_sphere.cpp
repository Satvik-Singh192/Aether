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

    // material combine
    outContact.restituion = (a.restituion + b.restituion) * 0.5f;
    outContact.friction_coeff = std::sqrt(a.friction * b.friction);

    return true;

    // float penetration_depth=radius_sum-dist;
    // float total_invmass=a.inverse_mass+b.inverse_mass;
    // if(total_invmass==0.0f)return;

    // //teleporting the balls outside of each other so it doesnt register as collision again in next frame cuz it didnt get enought time to seprate
    // Vec3 correction=normal*(penetration_depth/total_invmass);
    // a.position-=correction*a.inverse_mass;
    // b.position+=correction*b.inverse_mass;


    // Vec3 relative_velocity=b.velocity-a.velocity;
    // float velocity_along_normal=relative_velocity.dot(normal);
    // if(velocity_along_normal>0)return;

    // float restitution=PHYSICS_DEFAULT_RESTITUION;
    // float impulse_magnitude=-(1+restitution)*velocity_along_normal/total_invmass;

    // Vec3 impulse_force=normal*impulse_magnitude;
    // a.velocity-=impulse_force*a.inverse_mass;
    // b.velocity+=impulse_force*b.inverse_mass;

    // //for friction guyz
    // Vec3 rv=b.velocity -a.velocity ;// rel vel
    // Vec3 tangent=rv - normal*rv.dot(normal); //jis bhi axes (maybe mixed) aligned ho us jagah se dot prd lo for tangent
    // float tlength=tangent.length();
    // if(tlength > 1e-6f) {
    //     tangent=tangent * (1.0f / tlength);
    //     float fricvel=rv.dot(tangent);
    //     float fricmag=-fricvel/total_invmass;
    //     float mu=std::sqrt(a.friction*b.friction);
    //     float max_friction   = mu * impulse_magnitude;
    //     if (fricmag >  max_friction) fricmag =  max_friction;
    //     if (fricmag < -max_friction) fricmag = -max_friction;

    //     Vec3 friction_impulse = tangent * fricmag;
    //     a.velocity -= friction_impulse * a.inverse_mass;
    //     b.velocity += friction_impulse * b.inverse_mass;


    // }

}