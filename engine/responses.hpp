#pragma once
#include<string>

enum class PhysicsError{
    None,
    IDNotFound,
    InvalidRequest
};

struct PhysicsResult{
    bool success;
    PhysicsError error;
    std::string message;
    std::string debug_message="";
};