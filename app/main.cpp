#include "world/physicsworld.hpp"
#include "../renderer/window.hpp"
#include "test_scenarios.hpp"

int main()
{
    PhysicsWorld world;

    const TestCase active_case = TestCase::BoxSphereRamp;
    LoadSingleTestScenario(world, active_case);

    CreateWindow(world);
    return 0;
}