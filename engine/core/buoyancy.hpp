#pragma once 
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "core/sphere_collider.hpp"
#include "core/box_collider.hpp"

struct Fluid
{
    float density;    
    float height;       
    float drag_force; 
    
    Fluid(float d = 1000.0f, float h = 2.0f, float drag = 0.3f)
        : density(d), height(h), drag_force(drag) {}
};

void ApplyBuoyancyToSphere(Rigidbody& body, const Fluid& fluid, float gravity) ;
void ApplyBuoyancyToBox(Rigidbody& body, const Fluid& fluid, float gravity);
void ApplyBuoyancy(Rigidbody& body, const Fluid& fluid, float gravity);
