#pragma once
#include "math/vec3.hpp"
#include "math/mat3.hpp"
#include "rigidbody.hpp"
#include "box_collider.hpp"
struct OBB{
    Vec3 center;
    Vec3 axis[3];
    Vec3 halfsize;
};
