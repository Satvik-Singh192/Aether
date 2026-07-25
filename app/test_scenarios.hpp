#pragma once
#include "../renderer/camera.hpp"
#include <vector>
#include <string>
#include <unordered_map>
class AetherAPI;

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
    CollisionCauseTopple,
    CircularMotionRope,
    CircularMotionSpring,
    PyramidStack,
    ManyBoxes,
    ManySpheres,
    RandomScatter
};

extern std::vector<std::string> chapters;
extern std::unordered_map<std::string, std::vector<TestCase>> testmap;

void InitializeTestMap();
Camera LoadSingleTestScenario(AetherAPI &api, TestCase test_case);
