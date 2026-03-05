#pragma once
#include "collider.hpp"

class SphereCollider:public Collider{
public:
    float radius;
    SphereCollider(float r){
        radius=r;
        type=ShapeType::Sphere;
    }
};