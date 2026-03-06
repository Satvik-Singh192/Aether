#include"collision.hpp"
#include"core/sphere_collider.hpp"
#include"core/box_collider.hpp"
#include<algorithm>

float clamp(float v, float min, float max) {
    return std::max(min, std::min(v, max));
}

void resolveSphereBox(Rigidbody&sphere_body,Rigidbody&box_body){
    auto* sphere=static_cast<SphereCollider*>(sphere_body.collider);
    auto* box=static_cast<BoxCollider*>(box_body.collider);

    Vec3 boxmin=box_body.position-box->halfsize;
    Vec3 boxmax=box_body.position+box->halfsize;

    Vec3 closest;

    closest.x=clamp(sphere_body.position.x, boxmin.x, boxmax.x);
    closest.y=clamp(sphere_body.position.y, boxmin.y, boxmax.y);
    closest.z=clamp(sphere_body.position.z, boxmin.z, boxmax.z);

    Vec3 delta=sphere_body.position-closest;
    float dist=delta.dot(delta);
    float radius=sphere->radius;

    if(dist>radius*radius)return;
    dist=std::sqrt(dist);
    Vec3 normal;

    if(dist==0){
        normal=Vec3(0,1,0);
    }
    else{
        normal=delta*(1.0/dist);
    }

    float penetration=radius-dist;
    float total_invmass=sphere_body.inverse_mass+box_body.inverse_mass;
    if(total_invmass==0)return;

    Vec3 correction=normal*(penetration/total_invmass);
    sphere_body.position+=correction*sphere_body.inverse_mass;
    box_body.position-=correction*box_body.inverse_mass;

    Vec3 relative_vel=sphere_body.velocity-box_body.velocity;
    float relvel_normal=relative_vel.dot(normal);

    float restituion=0.5f;
    float impulse_mag=-(1+restituion)*relvel_normal*(1.0/total_invmass);

    Vec3 impulse=normal*impulse_mag;
    sphere_body.velocity+=impulse*sphere_body.inverse_mass;
    box_body.velocity-=impulse*box_body.inverse_mass;
}