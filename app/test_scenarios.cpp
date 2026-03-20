#include "test_scenarios.hpp"

#include "core/box_collider.hpp"
#include "core/rigidbody.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include "math/vec3.hpp"
#include "world/physicsworld.hpp"

namespace
{
   
    BoxCollider g_floor(Vec3(100.0f, 0.1f, 100.0f));
    SphereCollider g_small_sphere(0.5f);
    SphereCollider g_big_sphere(0.8f);
    BoxCollider g_small_box(Vec3(0.5f, 0.5f, 0.5f));
    BoxCollider g_wide_box(Vec3(1.0f, 0.5f, 0.8f));
    RampCollider g_gentle_ramp(0.35f, 8.0f, 1.5f);
    RampCollider g_steep_ramp(0.70f, 6.0f, 1.2f);

    void add_floor(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(), Vec3(), &g_floor, 0.0f));
    }

    void spawn_sphere_sphere_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-8.0f, 3.0f, 0.0f), Vec3(7.5f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
        world.addBody(Rigidbody(Vec3(-2.0f, 3.0f, 0.0f), Vec3(-4.0f, 0.0f, 0.0f), &g_big_sphere, 2.0f));
    }

    void spawn_box_box_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-8.5f, 1.0f, 0.0f), Vec3(9.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(-3.5f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_wide_box, 2.0f));
    }

    void spawn_sphere_box_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(0.0f, 8.5f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
        world.addBody(Rigidbody(Vec3(0.0f, 0.75f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_wide_box, 0.0f));
    }

    void spawn_box_ramp_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_gentle_ramp, 0.0f));
        world.addBody(Rigidbody(Vec3(0.5f, 4.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
    }

    void spawn_sphere_ramp_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_steep_ramp, 0.0f));
        world.addBody(Rigidbody(Vec3(-0.5f, 4.2f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_big_sphere, 1.2f));
    }

    void spawn_box_sphere_ramp_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_gentle_ramp, 0.0f));
        world.addBody(Rigidbody(Vec3(0.2f, 4.2f, -0.6f), Vec3(0.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(1.0f, 4.6f, 0.7f), Vec3(0.0f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
    }
}

void LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case)
{
    add_floor(world);

    switch (test_case)
    {
    case TestCase::SphereSphere:
        spawn_sphere_sphere_case(world);
        break;
    case TestCase::BoxBox:
        spawn_box_box_case(world);
        break;
    case TestCase::SphereBox:
        spawn_sphere_box_case(world);
        break;
    case TestCase::BoxRamp:
        spawn_box_ramp_case(world);
        break;
    case TestCase::SphereRamp:
        spawn_sphere_ramp_case(world);
        break;
    case TestCase::BoxSphereRamp:
    default:
        spawn_box_sphere_ramp_case(world);
        break;
    }
}
