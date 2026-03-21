#include"collision.hpp"
#include"core/sphere_collider.hpp"
#include"core/box_collider.hpp"
#include"core/rigidbody.hpp"
#include<algorithm>

float clamp(float v, float min, float max) {
    return std::max(min, std::min(v, max));
}

bool buildSphereBoxContact(Rigidbody& sphere_body, Rigidbody& box_body, Contact& outContact){
    auto* sphere=static_cast<SphereCollider*>(sphere_body.collider);
    auto* box=static_cast<BoxCollider*>(box_body.collider);

    if (!sphere || !box) return false;

    Vec3 boxmin=box_body.position-box->halfsize; //boootoom left back corner
    Vec3 boxmax=box_body.position+box->halfsize; // topp right front corner

    //we get the range for x,y,z  from just those 2 points above ......now we just need to check if anything is present in this range to be detected as collision

    Vec3 closest;

    closest.x=clamp(sphere_body.position.x, boxmin.x, boxmax.x);
    closest.y=clamp(sphere_body.position.y, boxmin.y, boxmax.y);
    closest.z=clamp(sphere_body.position.z, boxmin.z, boxmax.z);

    Vec3 delta=closest-sphere_body.position;
    float dist=delta.dot(delta);
    float radius=sphere->radius;

    if (dist > radius * radius) return false;
    dist = std::sqrt(std::max(0.0f, dist));
    Vec3 normal;

    if(dist<=PHYSICS_EPSILON){
        normal=Vec3(0.0f,1.0f,0.0f);
    }
    else{
        normal=delta*(1.0/dist);
    }

    outContact=Contact{};
    outContact.a=&sphere_body;
    outContact.b=&box_body;
    outContact.a_id = sphere_body.id;
    outContact.b_id = box_body.id;
    outContact.normal=normal;
    outContact.penetration=radius-dist;
    outContact.contact_point=closest;

    outContact.restitution=(sphere_body.restitution+ box_body.restitution)*0.5f;
    outContact.friction_coeff=std::sqrt(sphere_body.friction*box_body.friction);
    return true;


    // float penetration=radius-dist;
    // float total_invmass=sphere_body.inverse_mass+box_body.inverse_mass;
    // if(total_invmass==0)return;

    // Vec3 correction=normal*(penetration/total_invmass);
    // sphere_body.position+=correction*sphere_body.inverse_mass;
    // box_body.position-=correction*box_body.inverse_mass;

    // Vec3 relative_vel=sphere_body.velocity-box_body.velocity;
    // float relvel_normal=relative_vel.dot(normal);

    // float restitution=PHYSICS_DEFAULT_RESTITUTION;
    // float impulse_mag=-(1+restitution)*relvel_normal*(1.0/total_invmass);

    // Vec3 impulse=normal*impulse_mag;
    // sphere_body.velocity+=impulse*sphere_body.inverse_mass;
    // box_body.velocity-=impulse*box_body.inverse_mass;
    // //for friction guyz
    // Vec3 rv=sphere_body.velocity - box_body.velocity;// rel vel
    // Vec3 tangent=rv - normal*rv.dot(normal); //jis bhi axes (maybe mixed) aligned ho us jagah se dot prd lo for tangent
    // float tlength=tangent.length();
    // if(tlength > 1e-6f) {
    //     tangent=tangent * (1.0f / tlength);
    //     float fricvel=rv.dot(tangent);
    //     float fricmag=-fricvel/total_invmass;
    //     float mu=std::sqrt(sphere_body.friction * box_body.friction);
    //     float max_friction = mu * impulse_mag;
    //     if (fricmag >  max_friction) fricmag =  max_friction;
    //     if (fricmag < -max_friction) fricmag = -max_friction;

    //     Vec3 friction_impulse = tangent * fricmag;
    //     sphere_body.velocity += friction_impulse * sphere_body.inverse_mass;
    //     box_body.velocity   -= friction_impulse * box_body.inverse_mass;
    // }
}