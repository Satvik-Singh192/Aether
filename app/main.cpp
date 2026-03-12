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
    cout << "Hello Worlds\n";
    cout << "Testing\n";

    PhysicsWorld world;
    SphereCollider sphere1(0.5);
    SphereCollider sphere2(0.5);

    Rigidbody a(Vec3(-2,10,0),Vec3(3,0,0),&sphere1,1.0f);
    Rigidbody b(Vec3(2,10,0),Vec3(-3,0,0),&sphere2,1.0f);
    world.addBody(a);
    world.addBody(b);
    BoxCollider box1(Vec3(0.5f, 0.5f, 0.5f));
    BoxCollider box2(Vec3(0.5f, 0.5f, 0.5f));
    Rigidbody c(Vec3(-2, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody d(Vec3(2, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f);
    world.addBody(c);
    world.addBody(d);
    BoxCollider floorsize(Vec3(100.0f, 0.1f,100.0f));
    Rigidbody floor(
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(0.0f, 0.0f, 0.0f),
        &floorsize,
        0.0f
    );
    world.addBody(floor);

    CreateWindow(world);
    

    // PhysicsEngine engine;
    // Rigidbody body1(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), 1.0f);
	// Rigidbody body2(Vec3(10.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), 1.0f);
    // engine.addBody(&body1);
	// engine.addBody(&body2);

    // const float dt = 1.0f / 60.0f;
    // for(int i=0;i<300; i++){
    //     engine.update(dt);
    //     if(checkCollision(body1,body2, 1.0f)){
    //         std::cout << "Collision detected at time " <<  i << " ms\n";
    //         break;
    //     }
    // }
    return 0;
}