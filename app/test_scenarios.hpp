#pragma once
#include "../renderer/camera.hpp"
#include <vector>
#include <string>
#include <unordered_map>
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
    BuoyancyTest,
    HeatTransferDemo,
    RelativeVelocity,
    NewtonThirdLaw,
    BoxToppleOnRamp,
    SphereToppleOnRamp,
    CollisionCauseTopple
};

extern std::vector<std::string> chapters;
extern std::unordered_map<std::string, std::vector<TestCase>> testmap;

void InitializeTestMap();
Camera LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
