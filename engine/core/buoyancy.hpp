#pragma once 
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "core/box_collider.hpp"

#include "math/vec3.hpp"

struct Fluid
{
    float density;    
    float height;       
    float drag_force;
    Vec3 beaker_center;  
    float beaker_half_size;
    
    Fluid(float d = 1000.0f, float h = 2.0f, float drag = 0.3f)
        : density(d), height(h), drag_force(drag), 
          beaker_center(0.0f, 0.0f, 0.0f), beaker_half_size(5.0f) {}
    
    Fluid(float d, float h, float drag, const Vec3& center, float half_size)
        : density(d), height(h), drag_force(drag), 
          beaker_center(center), beaker_half_size(half_size) {}
};

bool IsBodyInBeaker(const Rigidbody& body, const Fluid& fluid);
void ApplyBuoyancyToSphere(Rigidbody& body, const Fluid& fluid, float gravity) ;
void ApplyBuoyancyToBox(Rigidbody& body, const Fluid& fluid, float gravity);
void ApplyBuoyancy(Rigidbody& body, const Fluid& fluid, float gravity);
