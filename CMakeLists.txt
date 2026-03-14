cmake_minimum_required(VERSION 3.20)

project(Aether_Test)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.3.8
)

FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG v0.1.36
)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 0.9.9.8
)

FetchContent_MakeAvailable(glad)
FetchContent_MakeAvailable(glm)

find_package(OpenGL REQUIRED)

add_executable(Aether_Test
    app/main.cpp
    engine/math/vec3.cpp
    engine/core/rigidbody.cpp
    engine/world/physicsworld.cpp
    engine/collision/sphere_sphere.cpp
    engine/collision/box_box.cpp
    engine/collision/sphere_box.cpp
    renderer/window.cpp 
    renderer/camera.cpp
    renderer/drawbodies.cpp
)

target_include_directories(Aether_Test PRIVATE engine)

target_link_libraries(Aether_Test
    glfw
    glad
    OpenGL::GL
    glm
)