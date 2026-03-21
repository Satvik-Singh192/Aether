#pragma once
#include "contact.hpp"
#include <array>

struct ContactManifold {
    Rigidbody* a = nullptr;
    Rigidbody* b = nullptr;
    BodyID a_id;
    BodyID b_id;
    Vec3 normal;
    
    int contact_count = 0;
    std::array<Contact, 4> contacts;
    
    ContactManifold() : contact_count(0) {}
};