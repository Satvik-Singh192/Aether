#include <iostream>
#include<chrono>
#include<thread>
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include "../renderer/window.hpp"
#include "core/box_collider.hpp" 
using namespace std;
int main(){
    PhysicsWorld world;
    BoxCollider box1(Vec3(0.5f, 0.5f, 1.0f));
    BoxCollider box2(Vec3(0.5f, 0.5f, 1.0f));

    Rigidbody a(Vec3(5,5,0),Vec3(-7.5,0,0),&box1,10.0f);
    Rigidbody b(Vec3(-5,5,0),Vec3(4.5,0,0),&box2,10.0f);
    // world.addBody(a);
    // world.addBody(b);
   
    Rigidbody c(Vec3(2, 15, 0), Vec3(0, 0, 0), &box1, 1.0f);
    Rigidbody d(Vec3(2, 25, -20), Vec3(0, 0, 0), &box2, 1.0f);


    //world.addBody(a);
    //world.addBody(b); world.addBody(c);
    // world.addBody(d);
    BoxCollider box(Vec3(0.5f, 0.5f, 0.5f));

    for(int i = 0; i < 5; i++)
    {
        world.addBody(Rigidbody(Vec3(5, 1.0f + i*5.0f, 0), Vec3(0,0,0), &box, 1.0f));
    }
    BoxCollider floorsize(Vec3(100.0f, 0.1f,100.0f));
    Rigidbody floor(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 0.0f, 0.0f),
        &floorsize,
        0.0f
    );
    world.addBody(floor);
    CreateWindow(world);
    return 0;
}