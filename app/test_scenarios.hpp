#pragma once

class PhysicsWorld;

enum class TestCase
{
    SphereSphere,
    BoxBox,
    SphereBox,
    BoxRamp,
    SphereRamp,
    BoxSphereRamp
};

void LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
