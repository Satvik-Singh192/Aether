#include "test_scenarios.hpp"
#include "core/box_collider.hpp"
#include "core/rigidbody.hpp"
#include "core/ramp_collider.hpp"
#include "core/sphere_collider.hpp"
#include "math/vec3.hpp"
#include "world/physicsworld.hpp"
#include <vector>
#include<cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::vector<std::string> chapters = {"Kinematics", "Laws of Motion", "Collision", "Rotation", "Fluids", "Thermal Properties", "Fun Tests"};
std::unordered_map<std::string, std::vector<TestCase>> testmap;


void InitializeTestMap()
{
    testmap["Kinematics"] = {TestCase::ProjectMotion, TestCase::RelativeVelocity, TestCase::InclinedPlane};
    testmap["Laws of Motion"] = {TestCase::NewtonThirdLaw, TestCase::MomentumTransfer};
    testmap["Collision"] = {TestCase::PerfectElasticCollision, TestCase::PerfectInelasticCollision, TestCase::Collision};
    testmap["Rotation"] = {TestCase::CenterOfMassTopple, TestCase::ConstraintPlayground, TestCase::AngularImpulse, TestCase::AngularStack, TestCase::CornerCollision, TestCase::RollingFriction, TestCase::BoxToppleOnRamp, TestCase::SphereToppleOnRamp, TestCase::CollisionCauseTopple, TestCase::CircularMotionRope, TestCase::CircularMotionSpring};
    testmap["Fluids"] = {TestCase::BuoyancyTest};
    testmap["Thermal Properties"] = {TestCase::HeatTransferDemo};
    testmap["Fun Tests"] = {TestCase::PyramidStack, TestCase::ManyBoxes, TestCase::ManySpheres, TestCase::RandomScatter};
}


namespace
{
    BoxCollider g_floor(Vec3(100.0f, 0.1f, 100.0f));
    SphereCollider g_small_sphere(0.5f);
    SphereCollider g_big_sphere(0.8f);
    BoxCollider g_small_box(Vec3(0.5f, 0.5f, 0.5f));
    BoxCollider g_wide_box(Vec3(1.0f, 0.5f, 0.8f));
    RampCollider g_gentle_ramp(0.35f, 8.0f, 1.5f);
    RampCollider g_steep_ramp(0.70f, 6.0f, 1.2f);

    Camera spawn_projectile_demo(PhysicsWorld &world)
{
    const float speed = 20.0f;
    const float angle1 = 30.0f * M_PI / 180.0f;
    const float angle2 = 60.0f * M_PI / 180.0f;

    Vec3 start_pos(-10.0f, 0.5f, 0.0f);
    Vec3 vel1(
        speed * cos(angle1),
        speed * sin(angle1),
        0.0f
    );
    Vec3 vel2(
        speed * cos(angle2),
        speed * sin(angle2),
        0.0f
    );

    world.addBody(Rigidbody(start_pos, vel1, &g_small_sphere, 1.0f));
    world.addBody(Rigidbody(start_pos, vel2, &g_small_sphere, 1.0f));

    return Camera().setPosition(glm::vec3(3.0,5.0,25.0));
}

    Camera spawn_perfect_elastic_collision(PhysicsWorld &world){
        const float y = 0.5f;
        Vec3 pos1(-8.0f, y, 0.0f);
        Vec3 pos2(0.0f, y, 0.0f);
        Vec3 vel1(10.0f, 0.0f, 0.0f);
        Vec3 vel2(0.0f, 0.0f, 0.0f);
        Rigidbody b1(pos1, vel1, &g_small_sphere, 1.0f);
        Rigidbody b2(pos2, vel2, &g_small_sphere, 1.0f);
        b1.restitution = 1.0f;
        b2.restitution = 1.0f;
        b1.friction = 0.0f;
        b2.friction = 0.0f;

        world.addBody(b1);
        world.addBody(b2);
        return Camera();
    }
    Camera spawn_perfect_inelastic_collision(PhysicsWorld &world)
{
    const float y = 0.5f;

    Vec3 pos1(-8.0f, y, 0.0f);
    Vec3 pos2(0.0f, y, 0.0f);

    Vec3 vel1(10.0f, 0.0f, 0.0f);
    Vec3 vel2(0.0f, 0.0f, 0.0f);

    Rigidbody b1(pos1, vel1, &g_small_sphere, 1.0f);
    Rigidbody b2(pos2, vel2, &g_small_sphere, 1.0f);

    b1.restitution = 0.0f;
    b2.restitution = 0.0f;

    b1.friction = 0.0f;
    b2.friction = 0.0f;

    world.addBody(b1);
    world.addBody(b2);
    return Camera();
}
    Camera spawn_collision_partial(PhysicsWorld &world)
{
    const float y = 0.5f;

    Vec3 pos1(-8.0f, y, 0.0f);
    Vec3 pos2(0.0f, y, 0.0f);

    Vec3 vel1(10.0f, 0.0f, 0.0f);
    Vec3 vel2(0.0f, 0.0f, 0.0f);

    Rigidbody b1(pos1, vel1, &g_small_sphere, 1.0f);
    Rigidbody b2(pos2, vel2, &g_small_sphere, 1.0f);

    b1.restitution = 0.5f;
    b2.restitution = 0.5f;

    b1.friction = 0.1f;
    b2.friction = 0.1f;

    world.addBody(b1);
    world.addBody(b2);
    return Camera();
}

    Camera spawn_incline_demo(PhysicsWorld& world){
        world.addBody(Rigidbody(Vec3(-2.0f, 0.3f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), &g_steep_ramp, 0.0f));
        world.addBody(Rigidbody(Vec3(3.4f,6.0f,0.0f),Vec3(0.0f,0.0f,0.0f),&g_small_sphere,2.0f));
        return Camera();
    }

    Camera spawn_relative_velocity(PhysicsWorld &world)
{

    
    const float y = 0.5f;
    
    Vec3 pos1(-8.0f, y, 0.0f);
    Vec3 pos2(8.0f, y, 0.0f);
    
    Vec3 vel1(15.0f, 0.0f, 0.0f);  
    Vec3 vel2(5.0f, 0.0f, 0.0f);  
    
    Rigidbody b1(pos1, vel1, &g_small_sphere, 1.0f);
    Rigidbody b2(pos2, vel2, &g_small_sphere, 1.0f);
    
    b1.restitution = 0.0f;
    b2.restitution = 0.0f;
    b1.friction = 0.0f;
    b2.friction = 0.0f;
    
    world.addBody(b1);
    world.addBody(b2);
    
    return Camera().setPosition(glm::vec3(0.0f, 3.0f, 20.0f));
}

    Camera spawn_newton_third_law(PhysicsWorld &world)
{
    
    
    const float y = 0.5f;
    Vec3 pos1(-10.0f, y, 0.0f);
    Vec3 vel1(12.0f, 0.0f, 0.0f);
    Rigidbody light_body(pos1, vel1, &g_small_sphere, 0.5f);
    
    Vec3 pos2(10.0f, y, 0.0f);
    Vec3 vel2(-6.0f, 0.0f, 0.0f);
    Rigidbody heavy_body(pos2, vel2, &g_big_sphere, 2.0f);
    light_body.restitution = 1.0f;
    heavy_body.restitution = 1.0f;
    light_body.friction = 0.0f;
    heavy_body.friction = 0.0f;
    
    world.addBody(light_body);
    world.addBody(heavy_body);
    return Camera().setPosition(glm::vec3(0.0f, 3.0f, 25.0f));
}

    Camera spawn_box_topple_on_ramp(PhysicsWorld &world)
{
    
    world.addBody(Rigidbody(Vec3(0.0f, 0.0f, 0.0f), Vec3(), &g_steep_ramp, 0.0f));
    Rigidbody toppling_box(Vec3(4.5f, 8.0f, 0.0f), Vec3(), &g_small_box, 1.5f);
    toppling_box.friction = 0.3f;
    toppling_box.restitution = 0.4f;
    world.addBody(toppling_box);
    
    return Camera().setPosition(glm::vec3(8.0f, 6.0f, 20.0f));
}

    Camera spawn_sphere_topple_on_ramp(PhysicsWorld &world)
{
    world.addBody(Rigidbody(Vec3(0.0f, 0.0f, 0.0f), Vec3(), &g_steep_ramp, 0.0f));
    Rigidbody rolling_sphere(Vec3(4.5f, 8.0f, 0.0f), Vec3(), &g_big_sphere, 1.2f);
    rolling_sphere.friction = 0.2f;
    rolling_sphere.restitution = 0.6f;
    world.addBody(rolling_sphere);
    
    return Camera().setPosition(glm::vec3(8.0f, 6.0f, 20.0f));
}

    Camera spawn_collision_cause_topple(PhysicsWorld &world)
{ world.addBody(Rigidbody(Vec3(0.0f, 0.0f, 0.0f), Vec3(), &g_gentle_ramp, 0.0f));
 
    Rigidbody moving_box(Vec3(-8.0f, 6.0f, 0.0f), Vec3(8.0f, 0.0f, 0.0f), &g_small_box, 2.0f);
    moving_box.friction = 0.1f;
    moving_box.restitution = 0.5f;
    world.addBody(moving_box);
    Rigidbody target_sphere(Vec3(4.0f, 6.0f, 0.0f), Vec3(), &g_small_sphere, 1.0f);
    target_sphere.friction = 0.2f;
    target_sphere.restitution = 0.7f;
    world.addBody(target_sphere);
    
    return Camera().setPosition(glm::vec3(0.0f, 4.0f, 20.0f));
}

    void add_floor(PhysicsWorld &world)
    {
        // Keep floor top at y=0 so scenario bodies spawn above, not inside.
        world.addBody(Rigidbody(Vec3(0.0f, -0.1f, 0.0f), Vec3(), &g_floor, 0.0f, PHYSICS_DEFAULT_FRICTION, 0.0f));
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
            if(i>=50)
            world.addBody(Rigidbody(Vec3(x, y, z-0.8f), Vec3(), &g_small_sphere, 0.5f));
            else  {
                 world.addBody(Rigidbody(Vec3(x, y, z ), Vec3(), &g_small_sphere, 0.5f));
            }
        }
     //   world.addBody(Rigidbody(Vec3(0, , z), Vec3(), &g_small_sphere, 0.5f));
        
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

    Camera spawn_constraint_playground(PhysicsWorld &world)
    {
        // Rope pair (two dynamic bodies linked together)
        auto rope_top = Rigidbody(Vec3(-9.0f, 7.0f, 0.0f), Vec3(0.4f, -0.2f, 0.0f), &g_small_sphere, 1.0f);
        rope_top.friction = 0.2f;
        auto rope_bottom = Rigidbody(Vec3(-9.2f, 3.2f, 0.0f), Vec3(-0.2f, 0.0f, 0.0f), &g_small_sphere, 1.1f);
        rope_bottom.friction = 0.2f;
        auto rope_top_id = world.addBody(rope_top);
        auto rope_bottom_id = world.addBody(rope_bottom);
        world.addDistanceConstraints(rope_top_id, rope_bottom_id, 4.0f, DistanceConstraint::ROPE, 0.0f, 0.0f);

        // Rod trio (short chain, no static anchor)
        std::uint32_t rod_ids[3];
        for (int i = 0; i < 3; ++i)
        {
            float x = -1.5f + i * 2.0f;
            rod_ids[i] = world.addBody(Rigidbody(Vec3(x, 5.0f, 0.0f), Vec3(0.0f, i == 2 ? -0.6f : 0.0f, 0.0f), &g_small_box, 1.0f));
        }
        world.addDistanceConstraints(rod_ids[0], rod_ids[1], 2.0f, DistanceConstraint::ROD, 0.6f, 0.6f);
        world.addDistanceConstraints(rod_ids[1], rod_ids[2], 2.0f, DistanceConstraint::ROD, 0.6f, 0.6f);

        // Spring pair (two moving spheres)
        auto spring_a = world.addBody(Rigidbody(Vec3(4.8f, 6.5f, 0.0f), Vec3(-0.3f, 0.4f, 0.0f), &g_small_sphere, 0.9f));
        auto spring_b = world.addBody(Rigidbody(Vec3(7.0f, 3.5f, 0.0f), Vec3(0.5f, -0.3f, 0.0f), &g_small_sphere, 0.9f));
        world.addDistanceConstraints(spring_a, spring_b, 3.5f, DistanceConstraint::SPRING, 2.2f, 0.7f);

        // Tiny rope chain (3 bodies) to demonstrate sequential constraints without overload
        std::vector<std::uint32_t> chain;
        chain.reserve(3);
        for (int i = 0; i < 3; ++i)
        {
            float y = 7.0f - i * 1.0f;
            chain.push_back(world.addBody(Rigidbody(Vec3(9.0f + i * 0.2f, y, 0.0f), Vec3(0.2f * i, 0.0f, 0.0f), &g_small_sphere, 0.8f + 0.1f * i)));
        }
        world.addDistanceConstraints(chain[0], chain[1], 1.0f, DistanceConstraint::ROPE, 0.0f, 0.0f);
        world.addDistanceConstraints(chain[1], chain[2], 1.0f, DistanceConstraint::ROPE, 0.0f, 0.0f);

        return Camera().setPosition(glm::vec3(0.0f, 7.0f, 28.0f));
    }

    
    void spawn_angular_stack(PhysicsWorld &world)
    {
        world.addBody(Rigidbody(Vec3(0.0f, 0.5f, 0.0f), Vec3(), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(0.0f, 1.55f, 0.0f), Vec3(), &g_small_box, 1.0f));
        world.addBody(Rigidbody(Vec3(0.0f, 2.6f, 0.0f), Vec3(), &g_small_box, 1.0f));
        
        world.addBody(Rigidbody(Vec3(-2.5f, 1.55f, 0.0f), Vec3(9.0f, 0.0f, 0.0f), &g_small_sphere, 0.5f));
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

    Camera spawn_heat_transfer_demo(PhysicsWorld &world)
    {
        world.thermal_settings.enabled = true;
        world.thermal_settings.conduction_rate = 15.0f;
        world.thermal_settings.radiation_rate = 0.02f;
        world.thermal_settings.ambient_temperature = 295.0f;
        world.thermal_settings.ambient_coupling = 0.03f;
        world.thermal_settings.radiation_distance = 4.0f;
        world.thermal_settings.min_visual_temperature = 240.0f;
        world.thermal_settings.max_visual_temperature = 660.0f;
        world.thermal_spawn_controls.enabled = true;
        world.thermal_spawn_controls.lock_to_basic_shapes = true;
        world.thermal_spawn_controls.spawn_temperature = 295.0f;
        world.thermal_spawn_controls.spawn_heat_capacity = 930.0f;
        world.thermal_spawn_controls.spawn_conductivity = 0.7f;
        world.thermal_spawn_controls.spawn_emissivity = 0.9f;

        const int boxCount = 9;
        const float spacing = 1.0f;
        const float startX = -0.5f * spacing * (boxCount - 1);
        const float coldTemp = 255.0f;
        const float hotTemp = 650.0f;

        for (int i = 0; i < boxCount; ++i)
        {
            float lerp = (boxCount == 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(boxCount - 1);
            float temp = coldTemp + lerp * (hotTemp - coldTemp);
            Vec3 pos(startX + i * spacing, 0.55f, 0.0f);
            Rigidbody body(pos, Vec3(), &g_small_box, 1.8f);
            body.thermal_enabled = true;
            body.temperature = temp;
            body.heat_capacity = 930.0f;
            body.thermal_conductivity = 0.75f;
            body.thermal_emissivity = 0.88f;
            world.addBody(body);
        }
        Rigidbody striker(Vec3(0.0f, 2.0f, 0.0f), Vec3(), &g_small_sphere, 1.0f);
        striker.thermal_enabled = true;
        striker.temperature = 3000.0f;
        striker.heat_capacity = 930.0f;
        striker.thermal_conductivity = 0.75f;
        striker.thermal_emissivity = 0.88f;
        world.addBody(striker);

        return Camera().setPosition(glm::vec3(0.0f, 4.8f, 22.0f));
    }

    Camera spawn_circular_motion_rope(PhysicsWorld &world)
    {
        Rigidbody fixed_box(Vec3(0.0f, 0.0f, 0.0f), Vec3(), &g_small_box, 0.0f);
        fixed_box.friction = 0.5f;
        auto box_id = world.addBody(fixed_box);
        const float rope_length = 3.0f;
        const float orbital_speed = 50.0f;
        Rigidbody orbiting_sphere(
            Vec3(rope_length, 2.0f, 0.0f), 
            Vec3(0.0f, 0.0f, orbital_speed),
            &g_small_sphere,
            1.0f
        );
        orbiting_sphere.friction = 0.1f;
        orbiting_sphere.restitution = 0.3f;
        auto sphere_id = world.addBody(orbiting_sphere);
        world.addDistanceConstraints(box_id, sphere_id, rope_length, DistanceConstraint::ROPE, 0.0f, 0.0f);
        return Camera().setPosition(glm::vec3(0.0f, 15.0f, 0.0f))
                       .setYaw(0.0f)
                       .setPitch(-90.0f);
    }

    Camera spawn_circular_motion_spring(PhysicsWorld &world)
    {
        Rigidbody fixed_box(Vec3(0.0f, 0.0f, 0.0f), Vec3(), &g_small_box, 0.0f);
        fixed_box.friction = 0.5f;
        auto box_id = world.addBody(fixed_box);
        const float spring_length = 3.0f;
        const float spring_constant = 2.0f;
        const float damping = 0.5f;
        const float orbital_speed = 50.0f;
        Rigidbody orbiting_sphere(
            Vec3(spring_length, 2.0f, 0.0f), 
            Vec3(0.0f, 0.0f, orbital_speed),
            &g_small_sphere,
            1.0f
        );
        orbiting_sphere.friction = 0.1f;
        orbiting_sphere.restitution = 0.3f;
        auto sphere_id = world.addBody(orbiting_sphere);
        world.addDistanceConstraints(box_id, sphere_id, spring_length, DistanceConstraint::SPRING, spring_constant, damping);
        return Camera().setPosition(glm::vec3(0.0f, 15.0f, 0.0f))
                       .setYaw(0.0f)
                       .setPitch(-90.0f);
    }

    Camera spawn_pyramid_stack_scenario(PhysicsWorld &world)
    {
        spawn_pyramid_stack(world);
        return Camera().setPosition(glm::vec3(0.0f, 5.0f, 25.0f));
    }

    Camera spawn_many_boxes_scenario(PhysicsWorld &world)
    {
        spawn_many_boxes(world);
        return Camera().setPosition(glm::vec3(0.0f, 5.0f, 25.0f));
    }

    Camera spawn_many_spheres_scenario(PhysicsWorld &world)
    {
        spawn_many_spheres(world);
        return Camera().setPosition(glm::vec3(0.0f, 5.0f, 25.0f));
    }

    Camera spawn_random_scatter_scenario(PhysicsWorld &world)
    {
        spawn_random_scatter(world);
        return Camera().setPosition(glm::vec3(0.0f, 5.0f, 30.0f));
    }

}

Camera LoadSingleTestScenario(PhysicsWorld &world, TestCase test_case)
{
    world.enable_buoyancy = false; // only when boyancy testcase
    world.thermal_settings = PhysicsWorld::ThermalSettings();
    world.thermal_spawn_controls = PhysicsWorld::ThermalSpawnControls();

    add_floor(world);

    switch (test_case)
    {
    case TestCase::ProjectMotion:
        return spawn_projectile_demo(world);
        break;
    case TestCase::PerfectElasticCollision:
        return spawn_perfect_elastic_collision(world);
    case TestCase::PerfectInelasticCollision:
        return spawn_perfect_inelastic_collision(world);
    case TestCase::Collision:
        return spawn_collision_partial(world);
    case TestCase::InclinedPlane:
        return spawn_incline_demo(world);
    case TestCase::MomentumTransfer:
        // Conservation-of-momentum chain (Newton's cradle style)
        spawn_chain_collide(world);
        return Camera().setPosition(glm::vec3(-1.0f, 6.0f, 24.0f));
    case TestCase::CenterOfMassTopple:
        // Demonstrates torque-induced tipping of a stacked tower
        spawn_stack_tipping(world);
        return Camera().setPosition(glm::vec3(0.0f, 5.0f, 20.0f));
    case TestCase::ConstraintPlayground:
        return spawn_constraint_playground(world);
    case TestCase::AngularImpulse:
        // Off-center collisions that inject angular momentum
        spawn_off_center_hit(world);
        return Camera().setPosition(glm::vec3(0.0f, 6.0f, 22.0f));
    case TestCase::AngularStack:
        spawn_angular_stack(world);
        return Camera().setPosition(glm::vec3(0.0f, 4.0f, 22.0f));
    case TestCase::CornerCollision:
        spawn_box_corner_collision(world);
        return Camera().setPosition(glm::vec3(-1.0f, 6.5f, 24.0f));
    case TestCase::RollingFriction:
        spawn_sphere_rolling(world);
        return Camera().setPosition(glm::vec3(-1.0f, 5.0f, 26.0f));
    case TestCase::RelativeVelocity:
        return spawn_relative_velocity(world);
    case TestCase::NewtonThirdLaw:
        return spawn_newton_third_law(world);
    case TestCase::BoxToppleOnRamp:
        return spawn_box_topple_on_ramp(world);
    case TestCase::SphereToppleOnRamp:
        return spawn_sphere_topple_on_ramp(world);
    case TestCase::CollisionCauseTopple:
        return spawn_collision_cause_topple(world);
    case TestCase::BuoyancyTest:
        world.enable_buoyancy = true;
        world.water_fluid = Fluid(2.0f, 2.0f, 0.3f);
        world.addBody(Rigidbody(Vec3(0.0f, 5.0f, 0.0f), Vec3(), &g_small_sphere, 0.5f));
        world.addBody(Rigidbody(Vec3(3.0f, 5.0f, 0.0f), Vec3(), &g_small_box, 0.6f));
        return Camera();
    case TestCase::HeatTransferDemo:
        return spawn_heat_transfer_demo(world);
    case TestCase::CircularMotionRope:
        return spawn_circular_motion_rope(world);
    case TestCase::CircularMotionSpring:
        return spawn_circular_motion_spring(world);
    case TestCase::PyramidStack:
        return spawn_pyramid_stack_scenario(world);
    case TestCase::ManyBoxes:
        return spawn_many_boxes_scenario(world);
    case TestCase::ManySpheres:
        return spawn_many_spheres_scenario(world);
    case TestCase::RandomScatter:
        return spawn_random_scatter_scenario(world);
    default:
        return spawn_projectile_demo(world);
        break;
    }
}