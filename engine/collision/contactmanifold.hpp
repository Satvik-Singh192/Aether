#pragma once
#include "contact.hpp"
#include <vector>
struct ContactManifold{
    Rigidbody* a =nullptr;
    Rigidbody* b =nullptr;
    BodyID a_id;
    BodyID b_id;
    Vec3 normal;
    int pointCount = 0;
    std::vector<Contact> points;
    ContactManifold() : points(4) {}


};