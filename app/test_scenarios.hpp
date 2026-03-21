#pragma once

class PhysicsWorld;

enum class TestCase
{
    SphereSphere,
    BoxBox,
    SphereBox,
    BoxRamp,
    SphereRamp,
    BoxSphereRamp,
    BoxStack,
    PyramidStack,
    ManySpheres,
    ManyBoxes,
    MixedPile,
    BouncyBalls,
    SlidingRampRow,
    ChainCollide,
    RandomScatter,
    StressTestLarge
};

void LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
