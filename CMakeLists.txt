cmake_minimum_required(VERSION 3.20)

project(Aether_Test)

set(CMAKE_CXX_STANDARD 20)

add_executable(Aether_Test
    app/main.cpp
    engine/math/vec3.cpp
    engine/core/rigidbody.cpp
    engine/engine/physicsworld.cpp
    engine/collision/collision.cpp
)

target_include_directories(Aether_Test PRIVATE engine)