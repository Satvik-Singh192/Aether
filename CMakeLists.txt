cmake_minimum_required(VERSION 3.20)

project(Aether_Test)

set(CMAKE_CXX_STANDARD 20)

add_executable(Aether_Test
    app/main.cpp
    engine/vec3.cpp
 "engine/physicsengine.hpp" "engine/collision.hpp")

target_include_directories(Aether_Test PRIVATE engine)