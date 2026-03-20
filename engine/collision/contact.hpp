#pragma once
#include"core/rigidbody.hpp"

struct Contact{
    Rigidbody* a=nullptr;
    Rigidbody*b=nullptr;

    Vec3 normal; // lets take A to B convention rn
    Vec3 contact_point;

    float penetration=0.0f;
    
    float restitution=0.0f;
    float friction_coeff=0.0f;

    float accumulated_normal_impulse=0.0f;
    float accumulated_tangent_impulse=0.0f;

    Vec3 tangent=Vec3();
};