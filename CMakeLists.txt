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
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.8
)

FetchContent_MakeAvailable(imgui)

FetchContent_MakeAvailable(glad)
FetchContent_MakeAvailable(glm)

find_package(OpenGL REQUIRED)

add_executable(Aether_Test
    engine/engine_configs.cpp
    app/main.cpp
    app/test_scenarios.cpp
    engine/math/vec3.cpp
    engine/math/mat.cpp
    engine/math/quat.cpp
    engine/core/rigidbody.cpp
    engine/world/physicsworld.cpp
    engine/collision/sphere_sphere.cpp
    engine/collision/box_box.cpp
    engine/collision/sphere_box.cpp
    engine/collision/box_ramp.cpp
    engine/collision/ramp_sphere.cpp
    engine/collision/ramp_ramp.cpp
    engine/collision/manifolds/buildBoxBoxManifold.cpp
    engine/collision/manifolds/buildBoxSphereManifold.cpp
    engine/collision/manifolds/buildSphereSphereManifold.cpp
    engine/collision/manifolds/buildRampBoxManifold.cpp
    engine/collision/manifolds/buildRampSphereManifold.cpp
    engine/collision/manifolds/buildRampRampManifold.cpp
    renderer/window.cpp 
    renderer/camera.cpp
    renderer/drawbodies.cpp
    renderer/bodymenu.cpp
)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)

target_link_libraries(imgui PRIVATE glfw OpenGL::GL)

target_include_directories(Aether_Test PRIVATE engine)

target_link_libraries(Aether_Test
    glfw
    glad
    OpenGL::GL
    glm
    imgui
)
