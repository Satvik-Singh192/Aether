#include <iostream>
#include <chrono>
#include <thread>
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include "../renderer/window.hpp"
#include "core/box_collider.hpp"
#include "core/ramp_collider.hpp"
using namespace std;
int main()
{
    PhysicsWorld world;
    BoxCollider box1(Vec3(0.5f, 0.5f, 1.0f));
    BoxCollider box2(Vec3(0.5f, 0.5f, 1.0f));
    Vec3 temp(0.0f, 1.0f, 2.0f);
    BoxCollider box(Vec3(0.5f, 0.5f, 0.5f));

    Vec3 ap = Vec3(5, 5, 0);
    Vec3 av = Vec3(-7.5, 0, 0);
    Rigidbody a(ap, av, &box1, 10.0f);
    // Rigidbody b(Vec3(-5,5,0),Vec3(4.5,0,0),&box2,10.0f);
    Rigidbody tm(temp, temp, &box, 2.0f);
    // world.addBody(a);
    // world.addBody(b);

    // Rigidbody c(Vec3(2, 15, 0), Vec3(0, 0, 0), &box1, 1.0f);
    // Rigidbody d(Vec3(2, 25, -20), Vec3(0, 0, 0), &box2, 1.0f);

    // world.addBody(a);
    // world.addBody(b); world.addBody(c);
    //  world.addBody(d);
    RampCollider ramp(0.5f, 8.0f); 
    Rigidbody rampBody(
        Vec3(0.0f, 0.0f, 0.0f), 
        Vec3(0.0f, 0.0f, 0.0f),
        &ramp,
        0.0f 
    );
    world.addBody(rampBody);

    for (int i = 0; i < 5; i++)
    {
       
        Vec3 spawn_pos = Vec3(5.0f, 1.0f + i * 5.0f, 0.0f);
        Vec3 spawn_vel = Vec3(0.0f, 0.0f, 0.0f);
        Rigidbody body(spawn_pos, spawn_vel, &box, 1.0f);
         
        std::uint32_t body_id = world.addBody(body);
        if(i==4){
            world.addforce(Vec3(100.0f, 100.0f, 0.0f),body_id);
        }
    }
     world.addforce(Vec3(-100.0f, 0.0f, 0.0f),4);
    BoxCollider floorsize(Vec3(100.0f, 0.1f, 100.0f));
    Vec3 floorpos = Vec3();
    Rigidbody floor(
        floorpos,
        floorpos,
        &floorsize,
        0.0f);
    world.addBody(floor);
    CreateWindow(world);
    return 0;
}