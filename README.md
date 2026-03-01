# AetherPhysics

AetherPhysics is a C++ project for building a simple real-time 3D rigid body physics simulation with OpenGL visualization.

This project is being developed for FOSS 2026.  
The goal is to implement core physics concepts from scratch and keep the architecture clean and modular.

## Current Scope

- Rigid body simulation
- Gravity
- Collision detection and response
- Friction
- Spring constraints
- Rope (distance constraints)
- Real-time 3D rendering

## Project Structure
```text
AetherPhysics/
├── engine/ # Physics logic
├── renderer/ # OpenGL rendering
├── app/ # Main entry point
├── CMakeLists.txt
└── README.md
```

- `engine/` contains physics-related code only.
- `renderer/` contains OpenGL and visualization code.
- `app/` connects physics and rendering.

## Build
``` bash
mkdir build
cd build
cmake ..
cmake --build .
```

Requires:
- C++20 compiler
- CMake
- OpenGL-compatible system