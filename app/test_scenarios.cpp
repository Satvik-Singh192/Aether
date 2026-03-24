#include "test_scenarios.hpp"

#include "core/box_collider.hpp"
#include "core/rigidbody.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include "math/vec3.hpp"
#include "world/physicsworld.hpp"
#include <vector>

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
        // Position Vec3() defaults to (0,0,0)
        world.addBody(Rigidbody(Vec3(), Vec3(), &g_floor, 0.0f, PHYSICS_DEFAULT_FRICTION, 0.0f));
    }

    void spawn_box_stack(PhysicsWorld &world)
    {
        // stack of boxes: spawn each box higher so they fall into place one-by-one
        const int count = 8;
        const float base_spawn_y = 2.0f;   // start a little above the ground so base layer falls first
        const float layer_spacing = 2.05f; // spacing between spawned boxes so they fall sequentially

        for (int i = 0; i < count; ++i)
        {
            float y = base_spawn_y + i * layer_spacing;
            world.addBody(Rigidbody(Vec3(0.0f, y, 0.0f), Vec3(), &g_small_box, 1.0f));
        }
    }

    void spawn_pyramid_stack(PhysicsWorld &world)
    {
        // pyramid of boxes: spawn layers at increasing heights so each layer falls onto the previous one
        const int base = 10;
        const float base_spawn_y = 2.0f;  // bottom layer spawn height (above ground)
        const float layer_spacing = 2.2f; // vertical spacing between layers to allow visible falling
        const float horizontal_spacing = 1.05f;

        for (int y = 0; y < base; ++y)
        {
            for (int x = 0; x < base - y; ++x)
            {
                float px = (x - (base - y - 1) * 0.5f) * horizontal_spacing;
                float py = base_spawn_y + y * layer_spacing;
                world.addBody(Rigidbody(Vec3(px, py, 0.0f), Vec3(), &g_small_box, 1.0f));
            }
        }
    }

    void spawn_many_spheres(PhysicsWorld &world)
    {
        for (int i = 0; i < 60; ++i)
        {
            float x = (i % 10) - 4.5f;
            float y = 3.0f + (i / 10) * 0.9f;
            float z = ((i / 5) % 2) * 0.6f;
            world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_sphere, 0.5f));
        }
    }

    void spawn_many_boxes(PhysicsWorld &world)
    {
        for (int i = 0; i < 80; ++i)
        {
            float x = (i % 8) - 3.5f;
            float y = 0.6f + (i / 8) * 0.95f;
            float z = ((i / 4) % 2) * 0.6f;
            world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_box, 1.0f));
        }
    }

    void spawn_mixed_pile(PhysicsWorld &world)
    {
        for (int i = 0; i < 40; ++i)
        {
            float x = (rand() % 200 - 100) * 0.05f;
            float y = 2.0f + (rand() % 50) * 0.06f;
            float z = (rand() % 200 - 100) * 0.03f;
            if (i % 2 == 0)
                world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_sphere, 0.6f));
            else
                world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_box, 1.0f));
        }
    }

    void spawn_bouncy_balls(PhysicsWorld &world)
    {
        for (int i = 0; i < 24; ++i)
        {
            float x = (i % 6) - 2.5f;
            float y = 4.0f + (i / 6) * 0.8f;
            world.addBody(Rigidbody(Vec3(x, y, 0.0f), Vec3(), &g_small_sphere, 0.5f, 0.2f, 0.9f));
        }
    }

    void spawn_sliding_ramp_row(PhysicsWorld &world)
    {
        for (int i = 0; i < 6; ++i)
        {
            float x = -6.0f + i * 2.5f;
            world.addBody(Rigidbody(Vec3(x, 0.0f, 0.0f), Vec3(), &g_gentle_ramp, 0.0f));
            world.addBody(Rigidbody(Vec3(x + 0.6f, 4.0f, 0.0f), Vec3(), &g_small_sphere, 0.6f));
        }
    }

    void spawn_chain_collide(PhysicsWorld &world)
    {
        // a row of spheres that knock into a second row
        for (int i = 0; i < 10; ++i)
        {
            world.addBody(Rigidbody(Vec3(-10.0f + i * 1.2f, 3.0f, 0.0f), Vec3(6.0f, 0.0f, 0.0f), &g_small_sphere, 0.5f));
        }
        for (int i = 0; i < 10; ++i)
        {
            world.addBody(Rigidbody(Vec3(2.0f + i * 1.2f, 3.0f, 0.0f), Vec3(), &g_small_sphere, 0.5f));
        }
    }

    void spawn_random_scatter(PhysicsWorld &world)
    {
        for (int i = 0; i < 100; ++i)
        {
            float x = (rand() % 400 - 200) * 0.05f;
            float y = 2.0f + (rand() % 200) * 0.02f;
            float z = (rand() % 400 - 200) * 0.02f;
            if (rand() % 2)
                world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_sphere, 0.4f));
            else
                world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_box, 0.9f));
        }
    }

    void spawn_stress_test_large(PhysicsWorld &world)
    {
        // large number of mixed objects to stress performance & solver
        for (int i = 0; i < 300; ++i)
        {
            float x = (i % 20) - 9.5f;
            float y = 1.0f + (i / 20) * 0.6f;
            float z = ((i / 10) % 3) - 1.0f;
            if (i % 3 == 0)
                world.addBody(Rigidbody(Vec3(x, y, z), Vec3(), &g_small_sphere, 0.5f));
            else if (i % 3 == 1)
                world.addBody(Rigidbody(Vec3(x + 0.3f, y, z), Vec3(), &g_small_box, 1.0f));
            else
                world.addBody(Rigidbody(Vec3(x - 0.3f, y, z), Vec3(), &g_wide_box, 1.5f));
        }
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
        world.addBody(Rigidbody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_gentle_ramp, 2.0f));
        world.addBody(Rigidbody(Vec3(0.5f, 4.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_small_box, 2.0f));
    }
 
    void spawn_sphere_ramp_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-2.0f, 2.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_steep_ramp, 1.0f));
        world.addBody(Rigidbody(Vec3(-2.0f, 5.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_steep_ramp, 1.2f));

        world.addBody(Rigidbody(Vec3(-0.2f, 7.4f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_big_sphere, 1.2f));
    }

    void spawn_box_sphere_ramp_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_gentle_ramp, 0.0f));
        // Changed Z from -0.6f to 0.0f
        world.addBody(Rigidbody(Vec3(0.2f, 4.2f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
        // Changed Z from 0.7f to 0.0f
        world.addBody(Rigidbody(Vec3(1.0f, 4.6f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
    }
    void spawn_box_stack_case(PhysicsWorld &world, int num_boxes = 8)
    {
        // Spawn a vertical stack so each box falls into place sequentially
        const float base_spawn_y = 2.0f;
        const float box_spacing = 2.05f;

        for (int i = 0; i < num_boxes; ++i)
        {
            Vec3 position(0.0f, base_spawn_y + i * box_spacing, 0.0f);
            world.addBody(Rigidbody(position, Vec3(0.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
        }
    }
    void spawn_rope_basic(PhysicsWorld &world)
    {
        auto id1 = world.addBody(Rigidbody(Vec3(0, 5, 0), Vec3(), &g_small_sphere, 1.0f));
        auto id2 = world.addBody(Rigidbody(Vec3(0, 8, 0), Vec3(), &g_small_sphere, 1.0f));

        world.addDistanceConstraints(id1, id2, 2.0f, DistanceConstraint::ROPE, 0.0f, 0.0f);
    }
    void spawn_rod_basic(PhysicsWorld &world)
    {
        auto id1 = world.addBody(Rigidbody(Vec3(-2, 5, 0), Vec3(), &g_small_box, 1.0f));
        auto id2 = world.addBody(Rigidbody(Vec3(2, 5, 0), Vec3(), &g_small_box, 1.0f));

        world.addDistanceConstraints(id1, id2, 4.0f, DistanceConstraint::ROD, 0.5f, 0.5f);
    }
    void spawn_spring_basic(PhysicsWorld &world)
    {
        auto id1 = world.addBody(Rigidbody(Vec3(0, 5, 0), Vec3(), &g_small_sphere, 1.0f));
        auto id2 = world.addBody(Rigidbody(Vec3(0, 10, 0), Vec3(), &g_small_sphere, 1.0f));

        world.addDistanceConstraints(id1, id2, 3.0f, DistanceConstraint::SPRING, 1.0f, 1.0f);
    }
    void spawn_rope_chain(PhysicsWorld &world)
    {
        const int N = 10;
        std::vector<std::uint32_t> ids;

        for (int i = 0; i < N; i++)
        {
            ids.push_back(
                world.addBody(Rigidbody(Vec3(0, 8 - i * 0.8f, 0), Vec3(), &g_small_sphere, 1.0f)));
        }

        for (int i = 0; i < N - 1; i++)
        {
            world.addDistanceConstraints(ids[i], ids[i + 1], 0.8f, DistanceConstraint::ROPE, 0.5f, 0.5f);
        }
    }
    void spawn_rod_chain(PhysicsWorld &world)
    {
        const int N = 8;
        std::vector<std::uint32_t> ids;

        for (int i = 0; i < N; i++)
        {
            ids.push_back(
                world.addBody(Rigidbody(Vec3(-5 + i * 1.2f, 6, 0), Vec3(), &g_small_box, 1.0f)));
        }

        for (int i = 0; i < N - 1; i++)
        {
            world.addDistanceConstraints(ids[i], ids[i + 1], 1.2f, DistanceConstraint::ROD, 0.5f, 0.5f);
        }
    }
    void spawn_soft_body_grid(PhysicsWorld &world)
    {
        const int W = 5;
        const int H = 5;

        std::uint32_t ids[W][H];

        for (int i = 0; i < W; i++)
        {
            for (int j = 0; j < H; j++)
            {
                ids[i][j] = world.addBody(
                    Rigidbody(Vec3(i * 1.0f, 8 + j * 1.0f, 0), Vec3(), &g_small_sphere, 0.8f));
            }
        }

        for (int i = 0; i < W; i++)
        {
            for (int j = 0; j < H; j++)
            {
                if (i + 1 < W)
                    world.addDistanceConstraints(ids[i][j], ids[i + 1][j], 1.0f, DistanceConstraint::SPRING, 50.0f, 6.0f);

                if (j + 1 < H)
                    world.addDistanceConstraints(ids[i][j], ids[i][j + 1], 1.0f, DistanceConstraint::SPRING, 50.0f, 6.0f);
            }
        }
    }
    void spawn_rope_with_collision(PhysicsWorld &world)
    {
        add_floor(world);

        auto id1 = world.addBody(Rigidbody(Vec3(0, 8, 0), Vec3(), &g_small_sphere, 1.0f));
        auto id2 = world.addBody(Rigidbody(Vec3(0, 6, 0), Vec3(), &g_small_sphere, 1.0f));

        world.addDistanceConstraints(id1, id2, 2.0f, DistanceConstraint::ROPE, 0.0f, 0.0f);

        // drop box onto rope
        world.addBody(Rigidbody(Vec3(0, 12, 0), Vec3(), &g_small_box, 2.0f));
    }

    
    void spawn_angular_torque(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(0.0f, 2.0f, 0.0f), Vec3(), &g_small_box, 2.0f));
     
        world.addBody(Rigidbody(Vec3(3.0f, 2.0f, 0.0f), Vec3(), &g_small_box, 1.0f));
    }

    void spawn_angular_impact(PhysicsWorld &world)
    {
        
        world.addBody(Rigidbody(Vec3(0.0f, 1.0f, 0.0f), Vec3(), &g_small_box, 2.0f));
       
        world.addBody(Rigidbody(Vec3(-3.0f, 1.7f, 0.0f), Vec3(6.0f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
       
        world.addBody(Rigidbody(Vec3(0.0f, 4.0f, 0.0f), Vec3(), &g_small_box, 2.0f));
        world.addBody(Rigidbody(Vec3(-3.0f, 4.0f, 0.8f), Vec3(5.0f, 0.0f, 0.0f), &g_small_sphere, 1.0f));
    }

    void spawn_angular_stack(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(0.0f, 0.5f, 0.0f), Vec3(), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(0.0f, 1.55f, 0.0f), Vec3(), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(0.0f, 2.6f, 0.0f), Vec3(), &g_small_box, 1.0f));
        
        world.addBody(Rigidbody(Vec3(-2.5f, 1.55f, 0.0f), Vec3(4.0f, 0.0f, 0.0f), &g_small_sphere, 0.5f));
    }

    void spawn_off_center_hit(PhysicsWorld &world)
    {
        // Test: Sphere hitting box corner point
        // Verify: Box spins from off-center impact
        // Key: Contact point far from box center = high torque
        
        // Target box (should spin markedly)
        world.addBody(Rigidbody(Vec3(0.0f, 2.0f, 0.0f), Vec3(), &g_small_box, 2.0f));
        
        // Sphere aimed at TOP-RIGHT corner (offset in X and Y)
        // Box half-extent is (0.5, 0.5, 0.5), so corner is at (0.5, 0.5, 0)
        // Contact point approximately: (0.5, 2.5, 0)
        // This is (0.5, 0.5, 0) from center = maximum lever arm
        world.addBody(Rigidbody(Vec3(-4.0f, 2.5f, 0.0f), Vec3(8.0f, 0.0f, 0.0f), &g_big_sphere, 1.0f));
        
        // Second test: different corner
        world.addBody(Rigidbody(Vec3(0.0f, 5.0f, 0.0f), Vec3(), &g_small_box, 1.5f));
        world.addBody(Rigidbody(Vec3(4.0f, 5.5f, 0.0f), Vec3(-7.0f, 0.0f, 0.0f), &g_big_sphere, 0.8f));
    }

    void spawn_box_corner_collision(PhysicsWorld &world)
    {
        // Test: Box-to-box collision at corners
        // Verify: Both boxes rotate from corner-to-corner impact
        // Key: Collision at edges = both bodies get rotational energy
        
        // Box A - moving
        world.addBody(Rigidbody(Vec3(-5.0f, 2.0f, 0.0f), Vec3(6.0f, 0.0f, 0.0f), &g_small_box, 1.0f));
        
        // Box B - stationary, offset so collision is corner-to-corner
        // A's right corner will hit B's left corner
        world.addBody(Rigidbody(Vec3(3.0f, 2.0f, 0.0f), Vec3(), &g_small_box, 1.0f));
        
        // Third test: perpendicular approach
        world.addBody(Rigidbody(Vec3(0.0f, 5.0f, 0.0f), Vec3(0.0f, -5.0f, 0.0f), &g_wide_box, 1.5f));
        world.addBody(Rigidbody(Vec3(0.0f, 8.0f, 0.0f), Vec3(), &g_small_box, 1.0f));
    }

    void spawn_stack_tipping(PhysicsWorld &world)
    {
        // Test: Stack toppling from side impact
        // Verify: Stack doesn't just slide - it TIPS/ROTATES
        // Key: Impact at height + side hit = rotation torque
        
        // Build tall stack (3 boxes)
        world.addBody(Rigidbody(Vec3(0.0f, 0.5f, 0.0f), Vec3(), &g_small_box, 1.0f));  // base
        world.addBody(Rigidbody(Vec3(0.0f, 1.55f, 0.0f), Vec3(), &g_small_box, 1.0f)); // middle
        world.addBody(Rigidbody(Vec3(0.0f, 2.6f, 0.0f), Vec3(), &g_small_box, 1.0f));  // top
        
        // Side impact at MIDDLE box height (not center-mass)
        // This creates lever arm: impact point height difference from COM
        // Impact at y=1.55, if COM of stack is at y~1.2, lever arm is ~0.35
        world.addBody(Rigidbody(Vec3(-4.0f, 1.55f, 0.0f), Vec3(7.0f, 0.0f, 0.0f), &g_big_sphere, 1.2f));
    }

    void spawn_sphere_rolling(PhysicsWorld &world)
    {
        // Test: Sphere rolling with friction torque
        // Verify: Sphere rotates due to friction at contact point
        // Key: Sliding sphere → friction impulse creates torque → rolling motion
        
        // Create gentle ramp or flat surface with high friction
        // Spawn sphere with sliding velocity (not rolling)
        world.addBody(Rigidbody(Vec3(-8.0f, 3.0f, 0.0f), Vec3(8.0f, 0.0f, 0.0f), &g_big_sphere, 1.2f, 0.8f, 0.3f));
        
        // Reference: non-sliding sphere for comparison
        world.addBody(Rigidbody(Vec3(-8.0f, 5.0f, 0.0f), Vec3(6.0f, 0.0f, 0.0f), &g_small_sphere, 0.8f, 0.1f, 0.2f));
        
        // Test on ramp: if it exists, rolling down will show rotation
        world.addBody(Rigidbody(Vec3(2.0f, 0.0f, 0.0f), Vec3(), &g_gentle_ramp, 0.0f));
        world.addBody(Rigidbody(Vec3(4.0f, 3.5f, 0.0f), Vec3(), &g_small_sphere, 1.0f, 0.8f, 0.1f));
    }

    void spawn_ramp_drop_on_sphere_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-1.0f, 0.5f, 0.0f), Vec3(), &g_big_sphere, 1.0f));
        world.addBody(Rigidbody(Vec3(-0.8f, 5.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_steep_ramp, 2.0f));
        world.addBody(Rigidbody(Vec3(0.6f, 4.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_gentle_ramp, 1.5f));
    }

    void spawn_ramp_ramp_stress_case(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(-3.0f, 3.0f, 0.0f), Vec3(2.0f, 0.0f, 0.0f), &g_steep_ramp, 1.5f));
        world.addBody(Rigidbody(Vec3(3.0f, 3.0f, 0.0f), Vec3(-2.0f, 0.0f, 0.0f), &g_steep_ramp, 1.5f));
        world.addBody(Rigidbody(Vec3(-1.5f, 6.0f, 0.0f), Vec3(0.5f, -0.5f, 0.0f), &g_gentle_ramp, 1.0f));
        world.addBody(Rigidbody(Vec3(1.5f, 6.0f, 0.0f), Vec3(-0.5f, -0.5f, 0.0f), &g_steep_ramp, 1.0f));
    }
    void spawn_box_45_topple(PhysicsWorld &world)
{
    
    add_floor(world);

   
    Rigidbody box(
        Vec3(0.0f, 3.0f, 0.0f),   // start above ground
        Vec3(0.0f, 0.0f, 0.0f),   // no initial velocity
        &g_small_box,
        2.0f                      // mass
    );

   
    float angle = 45.0f * (3.1415926f / 180.0f);

    box.orientation = Quat::fromAxisAngle(Vec3(0,0,1), angle);

    world.addBody(box);
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
    case TestCase::BoxStack:
        spawn_box_stack(world);
        break;
    case TestCase::PyramidStack:
        spawn_pyramid_stack(world);
        break;
    case TestCase::ManySpheres:
        spawn_many_spheres(world);
        break;
    case TestCase::ManyBoxes:
        spawn_many_boxes(world);
        break;
    case TestCase::MixedPile:
        spawn_mixed_pile(world);
        break;
    case TestCase::BouncyBalls:
        spawn_bouncy_balls(world);
        break;
    case TestCase::SlidingRampRow:
        spawn_sliding_ramp_row(world);
        break;
    case TestCase::ChainCollide:
        spawn_chain_collide(world);
        break;
    case TestCase::RandomScatter:
        spawn_random_scatter(world);
        break;
    case TestCase::StressTestLarge:
        spawn_stress_test_large(world);
        break;
    case TestCase::RopeBasic:
        spawn_rope_basic(world);
        break;
    case TestCase::RodBasic:
        spawn_rod_basic(world);
        break;
    case TestCase::SpringBasic:
        spawn_spring_basic(world);
        break;
    case TestCase::RopeChain:
        spawn_rope_chain(world);
        break;
    case TestCase::RodChain:
        spawn_rod_chain(world);
        break;
    case TestCase::SoftBody:
        spawn_soft_body_grid(world);
        break;
    case TestCase::RopeCollision:
        spawn_rope_with_collision(world);
        break;

    case TestCase::RampDropOnSphere:
        spawn_ramp_drop_on_sphere_case(world);
        break;

    case TestCase::RampRampStress:
        spawn_ramp_ramp_stress_case(world);
        break;

    // ===== ANGULAR PHYSICS TESTS =====
    case TestCase::AngularTorque:
        spawn_angular_torque(world);
        break;

    case TestCase::AngularImpact:
        spawn_angular_impact(world);
        break;

    case TestCase::AngularStack:
        spawn_angular_stack(world);
        break;

    case TestCase::OffCenterHit:
        spawn_off_center_hit(world);
        break;

    case TestCase::BoxCornerCollision:
        spawn_box_corner_collision(world);
        break;

    case TestCase::StackTipping:
        spawn_stack_tipping(world);
        break;

    case TestCase::SphereRolling:
        spawn_sphere_rolling(world);
        break;
    case TestCase::Boxtopple:
   
        spawn_box_45_topple(world);
        break;
        

    // ===== Default =====
    case TestCase::BoxSphereRamp:
    default:
        spawn_box_sphere_ramp_case(world);
        break;
    
    }
}