# Aether

A real-time 3D physics simulator designed to help high school and secondary school students understand physics concepts through interactive visualization and experimentation.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Features](#features)
- [Installation & Build](#installation--build)
- [Usage](#usage)
- [Test Scenarios](#test-scenarios)
- [Implementation](#implementation)
- [Project Structure](#project-structure)
- [Tech Stack](#tech-stack)
- [License](#license)

## Overview

Aether is an educational physics simulator designed to help high school and secondary school students understand complex physics concepts through interactive visualization. Instead of memorizing equations, students can experiment with real-time simulations of rigid bodies, collisions, constraints, fluids, and heat transfer. The simulator bridges the gap between theory and practice by letting students observe how physics concepts manifest in a dynamic 3D world.

Students can experiment with familiar scenarios, objects falling under gravity, collisions between different shapes, springs bouncing, objects floating in water, and even thermal energy transfer, building intuition about how the physical world works.

**Target Audience**: High school and secondary school students studying physics who want to visualize and understand concepts through hands-on simulation.

**Key Goal**: Implement core physics concepts from scratch with a clean, modular architecture that prioritizes correctness and educational clarity.

## Quick Start

### Windows
1. Download the `release/windows` folder
2. Double-click `Aether_Test.exe`
3. The simulator will launch with default test scenarios

### Linux (Ubuntu & derivatives)
1. Open a terminal in the `release/linux` directory
2. Run:
   ```bash
   chmod +x Aether_Test
   ./Aether_Test
   ```

**No additional setup required for pre-built releases!**

## Features

Aether provides an extensive physics simulation environment focused on helping students understand fundamental physics principles:

**Rigid Body Motion**: Watch objects move and rotate realistically under the influence of gravity and applied forces. The simulator handles 6 degrees of freedom, meaning objects can translate and rotate freely in 3D space.

**Collision Realism**: When objects collide, they respond authentically. The simulator supports spheres, boxes, and ramps, calculating collision points and impulses that determine how objects bounce, slide, or come to rest. Students see Newton's laws in action through every collision.

**Constraints**: Simulate ropes, springs, and rigid connections between objects. Watch how a rope constrains motion differently than a spring, and how springs oscillate when pulled. These constraints are essential for understanding structural mechanics and energy transfer.

**Friction**: Objects don't slide infinitely, friction brings them to rest over time. Experiment with different friction levels to see how surface properties affect motion, understanding the balance between kinetic and static friction.

**Buoyancy & Fluids**: Observe Archimedes' principle firsthand. Objects submerged in fluid experience buoyant forces; see how displacement, density, and weight determine whether something floats, sinks, or hovers.

**Thermal Physics**: Watch heat transfer in real-time. Objects change color from cool to hot as thermal energy distributes between bodies through conduction, radiation, and ambient cooling. Understand energy conservation and heat dissipation visually.

**Interactive Experimentation**: Create any scenario you imagine. Use the interactive menu to add spheres, boxes, and ramps to the world, positioned exactly where you want them. Adjust gravity direction and magnitude, modify friction coefficients, change damping, and tune constraint parameters, all in real-time. Build physical models to test hypotheses, recreate examples from textbooks, or invent entirely new scenarios to explore physics principles.

**Custom Physics Worlds**: Unlike static simulations, Aether lets you be an experimenter. Drop objects from different heights to study projectile motion, stack boxes to explore stability and center of mass, create pendulum systems with springs and ropes, or design complex contraptions to understand how different forces interact. The simulator responds immediately to your changes, letting you see the effects of adjusting parameters.

**25 Pre-Built Scenarios**: Learn from carefully designed scenarios demonstrating kinematics, Newton's laws, collisions, rotation, fluids, thermal physics, and stress tests.

## Installation & Build

### Prerequisites

- **C++20 Compatible Compiler** (MSVC, GCC, or Clang)
- **CMake** 3.20 or higher
- **OpenGL Support** (OpenGL 3.3+)
- **Git** (for CMake FetchContent dependencies)

### Build Steps

```bash
# Clone or navigate to the repository
cd path/to/Aether

# Create build directory
mkdir build
cd build

# Configure and build
cmake ..
cmake --build . --config Release

# Run the executable
./Aether_Test  # On Linux/macOS
# or
Aether_Test.exe  # On Windows
```

### Dependencies (Auto-Downloaded)

The build system automatically downloads and configures:
- **GLFW 3.3.8**: Cross-platform window and input handling
- **GLAD**: OpenGL function loader
- **GLM 0.9.9.8**: OpenGL math library
- **ImGui 1.90.8**: Immediate mode GUI framework

No manual dependency installation needed!

## Usage

### Launching the Application

Once built or using a pre-release binary, the simulator opens with a 3D viewport and interactive menu. Pre-loaded test scenarios demonstrate various physics principles.

### Interactive Controls

**Camera Navigation**:
- Mouse: Rotate camera
- Arrow keys or WASD: Pan camera view

**Menu Interactions**:
- Select test scenarios from the menu to observe specific physics concepts
- Pause/resume the simulation
- Adjust physics parameters in real-time (gravity, friction, damping)
- Create new bodies on-the-fly using the UI
- Add constraints (springs, ropes, rods) during simulation

## Test Scenarios

Aether includes 25 pre-built test scenarios organized by physics concepts:

### Kinematics
- **Projectile Motion**: Observe parabolic trajectory under gravity with different launch angles
- **Relative Velocity**: Understand reference frames by comparing velocities of moving objects
- **Inclined Plane**: Study motion of objects on slopes and the effects of angle on acceleration

### Laws of Motion
- **Newton's Third Law**: Action-reaction force pairs in collisions between objects
- **Momentum Transfer**: Observe conservation of momentum in collisions

### Collision Physics
- **Perfect Elastic Collision**: Kinetic energy is conserved (no energy loss)
- **Perfect Inelastic Collision**: Maximum energy dissipation (objects stick together)
- **Partially Elastic Collision**: Real-world collision behavior with partial energy loss

### Rotational Motion
- **Center of Mass Topple**: Bodies tipping over due to center-of-mass offset
- **Constraint Playground**: Experiment with various constraint configurations
- **Angular Impulse**: Angular momentum transfer through impulses
- **Angular Stack**: Stacking objects to observe rotational effects
- **Corner Collision**: Angular momentum transfer when objects collide at corners
- **Rolling Friction**: Friction-driven rolling behavior on surfaces
- **Box Topple on Ramp**: Boxes tipping on inclined surfaces
- **Sphere Topple on Ramp**: Spheres rolling and stabilizing on ramps
- **Collision Cause Topple**: Objects toppling due to collision impacts
- **Circular Motion with Rope**: Objects constrained in circular paths with tension
- **Circular Motion with Spring**: Objects oscillating in circular paths with springs

### Fluid Dynamics
- **Buoyancy Test**: Observe Archimedes' principle as objects interact with fluid

### Thermal Physics
- **Heat Transfer Demo**: Watch heat transfer between objects and color change based on temperature

### Stress Tests
- **Pyramid Stack**: Stability test with many stacked objects
- **Many Boxes**: Simulate numerous box collisions and interactions
- **Many Spheres**: Performance test with large numbers of spheres
- **Random Scatter**: Complex multi-object interactions and collision cascades

## Implementation

Aether is built from scratch with a focus on educational clarity and correctness. The codebase is organized into three main layers:

**Physics Engine** (`engine/`): The core simulation logic is completely independent of graphics. It handles rigid body dynamics using quaternion-based rotations and semi-implicit Euler integration. Collision detection supports all combinations of spheres, boxes, and ramps using specialized solvers. The constraint solver handles ropes, rods, and springs with a stable iterative approach. Friction follows Coulomb's model, and the buoyancy and thermal systems provide additional realism.

**Rendering Layer** (`renderer/`): Built on OpenGL and GLFW, this layer visualizes the physics simulation. ImGui provides the interactive menu for creating objects and adjusting parameters in real-time. The thermal visualization maps temperature to color so students can observe heat transfer visually.

**Application Logic** (`app/`): Bridges the physics engine and renderer, managing the main loop, test scenarios, and user interactions. The test scenarios are carefully designed to demonstrate specific physics principles.

This clean separation means the physics engine could be used in other projects, and the rendering could be swapped for a different graphics system without affecting the core simulation.

## Project Structure

```
Aether/
├── app/                          # Application entry point
│   ├── main.cpp                 # Main loop and initialization
│   ├── test_scenarios.cpp       # Pre-built educational scenarios
│   └── test_scenarios.hpp
│
├── engine/                       # Physics simulation engine
│   ├── engine_configs.cpp/hpp   # Configuration and constants
│   ├── responses.hpp            # Physics response enums
│   ├── common_header.hpp        # Shared definitions
│   │
│   ├── core/                    # Core physics components
│   │   ├── bodyid.hpp           # Body identification system
│   │   ├── rigidbody.cpp/hpp    # RigidBody class (forces, torques, integration)
│   │   ├── collider.hpp         # Collider interface
│   │   ├── sphere_collider.hpp  # Sphere primitive
│   │   ├── box_collider.hpp     # Box primitive
│   │   ├── ramp_collider.hpp    # Ramp primitive
│   │   └── buoyancy.cpp/hpp     # Buoyancy system
│   │
│   ├── math/                    # Custom math library
│   │   ├── vec3.cpp/hpp         # 3D vector operations
│   │   ├── mat3.hpp             # 3x3 matrix operations
│   │   └── quat.cpp/hpp         # Quaternion operations
│   │
│   ├── collision/               # Collision detection & response
│   │   ├── collision.hpp        # Collision system interface
│   │   ├── contact.hpp          # Contact point definition
│   │   ├── contactmanifold.hpp  # Multiple contact points
│   │   ├── obb.hpp              # Oriented bounding box
│   │   ├── sphere_sphere.cpp    # Sphere-sphere collisions
│   │   ├── box_box.cpp          # Box-box collisions (SAT)
│   │   ├── sphere_box.cpp       # Sphere-box collisions
│   │   ├── box_ramp.cpp         # Box-ramp collisions
│   │   ├── ramp_sphere.cpp      # Ramp-sphere collisions
│   │   ├── ramp_ramp.cpp        # Ramp-ramp collisions
│   │   └── manifolds/           # Contact manifold builders
│   │       ├── buildBoxBoxManifold.cpp
│   │       ├── buildBoxSphereManifold.cpp
│   │       ├── buildSphereSphereManifold.cpp
│   │       ├── buildRampBoxManifold.cpp
│   │       ├── buildRampSphereManifold.cpp
│   │       ├── buildRampRampManifold.cpp
│   │       └── manifold_builders.hpp
│   │
│   ├── constraints/             # Physics constraints
│   │   └── distance_constraints.hpp  # Rope, rod, spring constraints
│   │
│   └── world/                   # Physics world simulator
│       └── physicsworld.cpp/hpp # Main simulation loop & body management
│
├── renderer/                     # Visualization & graphics
│   ├── window.cpp/hpp           # Window management & event handling
│   ├── camera.cpp/hpp           # Camera transform & controls
│   ├── bodyshaders.cpp/hpp      # GLSL shader management
│   ├── drawbodies.cpp/hpp       # Body rendering
│   ├── drawconstraints.cpp/hpp  # Constraint visualization
│   ├── bodymenu.cpp/hpp         # ImGui menu interface
│   ├── aether_theme.cpp/hpp     # ImGui theme customization
│   └── thermal_palette.hpp      # Temperature color mapping
│
├── release/                      # Pre-built binaries
│   ├── windows/
│   │   └── Aether_Test.exe
│   └── linux/
│       └── Aether_Test
│
├── CMakeLists.txt               # Build configuration
├── CMakeSettings.json           # IDE-specific settings
├── vcpkg.json                   # Package manager config
├── README.md                    # This file
└── LICENSE                      # MIT License
```

**Architecture Overview**:
- **engine/**: Self-contained physics simulation with no graphics dependencies
- **renderer/**: OpenGL visualization using the physics engine output
- **app/**: Bridges physics engine and renderer, manages scenarios and main loop

## Tech Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Language** | C++20 | Modern, efficient systems programming |
| **Build System** | CMake 3.20+ | Cross-platform compilation |
| **Graphics API** | OpenGL 3.3+ | Hardware-accelerated 3D rendering |
| **Window Framework** | GLFW 3.3.8 | Cross-platform windowing & input |
| **Math Library** | GLM 0.9.9.8 | OpenGL-compatible mathematics |
| **GUI Framework** | ImGui 1.90.8 | Real-time interactive menu system |
| **Graphics Loader** | GLAD | OpenGL function management |
| **Platforms** | Windows, Linux, macOS | Cross-platform support |

## License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for complete details.

## Contributors

- [@Satvik-Singh192](https://github.com/Satvik-Singh192)
- [@urastogi2048](https://github.com/urastogi2048)
- [@PresenceOP-Coder](https://github.com/PresenceOP-Coder)
- [@Just-Here-TO-Code](https://github.com/Just-Here-TO-Code)