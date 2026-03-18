#pragma once
#include "core/rigidbody.hpp"


struct Contact{

    Rigidbody *a =nullptr;
    Rigidbody *b =nullptr;

    Vec3 normal;
    Vec3 contactpoint;

    float penetration=0.0f;
    float restitution=0.0f;

    float friction=0.0f;

    //friction will happen via this
    Vec3 tangent1;
    Vec3 tangent2;

    //effective mass(for now equiv, but will change when rotation)
    float normalMass = 0.0f;
    float tangentMass1 = 0.0f;
    float tangentMass2 = 0.0f;
    //warm start
    float normalimpulse=0.0f;
    float tangent1impluse=0.0f;
    float tangent2impulse=0.0f;
    //stabalisation
    float bias=0.0f;
    float velocitybias=0.0f;

};