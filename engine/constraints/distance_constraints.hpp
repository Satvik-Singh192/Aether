#pragma once
#include"core/rigidbody.hpp"
#include"math/vec3.hpp"
#include<cstdint>

struct DistanceConstraint{
    std::uint32_t a_id;
    std::uint32_t b_id;
    Rigidbody *a;
    Rigidbody *b;
    float rest_length; // distance bw the bodies

    float accumulated_impulse=0.0f;
    float stiffness=0.0f; // if high sstiffness, acts as rod and if 0 then rope

    float damping=0.0f; //how much energy actually lost (if 0 , spring oscillat forever)

    enum TYPE{
        ROPE,
        ROD,
        SPRING
    } type;
};