#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include "core/box_collider.hpp"
#include <algorithm>
#include <cmath>

static float clamp(float v, float min, float max) {
    return std::max(min, std::min(v, max));
}

bool buildSphereBoxContact(Rigidbody& sphere_body, Rigidbody& box_body, Contact& outContact) {
    auto* sphere = static_cast<SphereCollider*>(sphere_body.collider);
    auto* box = static_cast<BoxCollider*>(box_body.collider);
    if (!sphere || !box) {
        return false;
    }

    Vec3 boxmin = box_body.position - box->halfsize;
    Vec3 boxmax = box_body.position + box->halfsize;

    Vec3 closest;

    closest.x = clamp(sphere_body.position.x, boxmin.x, boxmax.x);
    closest.y = clamp(sphere_body.position.y, boxmin.y, boxmax.y);
    closest.z = clamp(sphere_body.position.z, boxmin.z, boxmax.z);

    Vec3 delta = sphere_body.position - closest;
    float distSq = delta.dot(delta);
    float radius = sphere->radius;

    if (distSq > radius * radius) {
        return false;
    }

    float dist = std::sqrt(std::max(0.0f, distSq));
    Vec3 normal;

    if (dist <= PHYSICS_EPSILON) {
        normal = Vec3(0.0f, 1.0f, 0.0f);
    } else {
        normal = delta * (1.0f / dist);
    }

    outContact = Contact{};
    outContact.a = &sphere_body;
    outContact.b = &box_body;
    outContact.normal = normal;
    outContact.penetration = radius - dist;
    outContact.contactpoint = closest;

    Vec3 correction=normal*(penetration/total_invmass);
    sphere_body.position+=correction*sphere_body.inverse_mass;
    box_body.position-=correction*box_body.inverse_mass;

    float restituion=(sphere_body.restitution+box_body.restitution)*0.5f;
    if(fabs(relvel_normal)<0.5f){
        restituion=0.0f;
    }
    float impulse_mag=-(1+restituion)*relvel_normal*(1.0/total_invmass);

    Vec3 impulse=normal*impulse_mag;
    sphere_body.velocity+=impulse*sphere_body.inverse_mass;
    box_body.velocity-=impulse*box_body.inverse_mass;
    //for friction guyz
    Vec3 rv=sphere_body.velocity - box_body.velocity;// rel vel
    Vec3 tangent=rv - normal*rv.dot(normal); //jis bhi axes (maybe mixed) aligned ho us jagah se dot prd lo for tangent
    float tlength=tangent.length();
    if(tlength > 1e-6f) {
        tangent=tangent * (1.0f / tlength);
        float fricvel=rv.dot(tangent);
        float fricmag=-fricvel/total_invmass;
        float mu=std::sqrt(sphere_body.friction * box_body.friction);
        float max_friction = mu * impulse_mag;
        if (fricmag >  max_friction) fricmag =  max_friction;
        if (fricmag < -max_friction) fricmag = -max_friction;

        Vec3 friction_impulse = tangent * fricmag;
        sphere_body.velocity += friction_impulse * sphere_body.inverse_mass;
        box_body.velocity   -= friction_impulse * box_body.inverse_mass;
    }
    outContact.friction = std::sqrt(std::max(0.0f, sphere_body.friction) * std::max(0.0f, box_body.friction));
    outContact.restitution = std::max(sphere_body.restitution, box_body.restitution);

     return true;
}