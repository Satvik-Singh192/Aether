#pragma once
#include "../renderer/camera.hpp"
class PhysicsWorld;

enum class TestCase
{
    ProjectMotion,
    PerfectElasticCollision,
    PerfectInelasticCollision,
    Collision,

};

Camera LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case);
