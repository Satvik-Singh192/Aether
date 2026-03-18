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
    SphereCollider sphere1(0.5);
    SphereCollider sphere2(0.5);
    SphereCollider sphere3(0.5);
    SphereCollider sphere4(0.5);
    SphereCollider sphere5(0.5);
    SphereCollider sphere6(0.5);
    SphereCollider sphere7(0.5);
    SphereCollider sphere8(0.5);
    SphereCollider sphere9(0.5);

    Rigidbody a(Vec3(-2,10,0),Vec3(3,0,0),&sphere1,1.0f);
    Rigidbody b(Vec3(2,10,0),Vec3(-3,0,0),&sphere2,1.0f);
    world.addBody(a);
    world.addBody(b);
    BoxCollider box1(Vec3(0.5f, 0.5f, 1.0f));
    BoxCollider box2(Vec3(0.5f, 0.5f, 1.0f));
    Rigidbody c(Vec3(-2, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody d(Vec3(2, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f);

    Rigidbody e(Vec3(-3, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody f(Vec3(-4, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody g(Vec3(3, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody p(Vec3(-5, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody h(Vec3(4, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody q(Vec3(-6, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody i(Vec3(5, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody r(Vec3(-7, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody j(Vec3(6, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody s(Vec3(-8, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody k(Vec3(7, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody t(Vec3(-9, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody l(Vec3(8, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody u(Vec3(-10, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody m(Vec3(9, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f); Rigidbody v(Vec3(-11, 5, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody n(Vec3(10, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f);
    Rigidbody o(Vec3(11, 5, 0), Vec3(-3, 0, 0), &box2, 1.0f);

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
    // Spawn many additional bodies for stress testing
    for (int x = -20; x <= 20; x += 2) {
        int z = 0;
            float height = 15.0f + float((x + 20) % 7);
            Vec3 pos((float)x, height, (float)z);
            Vec3 vel((x % 2 == 0) ? 3.0f : -3.0f, 0.0f, 0.0f);
            Rigidbody stressBody(pos, vel, &sphere1, 1.0f);
            world.addBody(stressBody);
    }
    for (int x = -20; x <= 20; x += 2) {
        int z = 0;
            float height = 30.0f + float((x + 20) % 7);
            Vec3 pos((float)x, height, (float)z);
            Vec3 vel((x % 2 == 0) ? 3.0f : -3.0f, 0.0f, 0.0f);
            Rigidbody stressBody(pos, vel, &sphere1, 1.0f);
            world.addBody(stressBody);
    }

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