#pragma once
#include "../renderer/camera.hpp"
class PhysicsWorld;

enum class TestCase
{
    ProjectMotion,
    PerfectElasticCollision,
    PerfectInelasticCollision,
    Collision,
    InclinedPlane,
    MomentumTransfer,
    CenterOfMassTopple,
    ConstraintPlayground,
    AngularImpulse,
    AngularStack,
    CornerCollision,
    RollingFriction,
    BuoyancyTest
};

Camera LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
