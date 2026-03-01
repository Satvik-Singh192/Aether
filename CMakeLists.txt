cmake_minimum_required(VERSION 3.20)

project(AetherPhysics)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Collect source files
file(GLOB_RECURSE ENGINE_SOURCES engine/*.cpp)
file(GLOB_RECURSE APP_SOURCES app/*.cpp)

# Create executable
add_executable(AetherPhysics
    ${ENGINE_SOURCES}
    ${APP_SOURCES}
)

# Include directories
target_include_directories(AetherPhysics
    PRIVATE
        engine
)