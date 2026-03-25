#pragma once
#include<string>

enum class PhysicsError{
    None,
    IDNotFound
};

struct PhysicsResult{
    bool success;
    PhysicsError error;
    std::string message;
};