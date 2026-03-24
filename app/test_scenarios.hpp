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
    StressTestLarge,
    RopeBasic,
    RodBasic,
    SpringBasic,
    RopeChain,
    RodChain,
    SoftBody,
    RopeCollision,
    AngularTorque,
    AngularImpact,
    AngularStack,
    OffCenterHit,
    BoxCornerCollision,
    StackTipping,
    SphereRolling,
    RampDropOnSphere,
    RampRampStress,
};

void LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
