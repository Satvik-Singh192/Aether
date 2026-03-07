#include <iostream>
#include<chrono>
#include<thread>
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "world/physicsworld.hpp"
#include "collision/collision.hpp"
#include "core/sphere_collider.hpp"
#include "../renderer/opengl_test.hpp"
#include "core/box_collider.hpp" 
using namespace std;
int main(){
    cout << "Hello Worlds\n";
    cout << "Testing\n";

    PhysicsWorld world;
    SphereCollider sphere1(0.5);
    SphereCollider sphere2(0.5);

    Rigidbody a(Vec3(-2,5,0),Vec3(3,0,0),&sphere1,1.0f);
    Rigidbody b(Vec3(2,5,0),Vec3(-3,0,0),&sphere2,1.0f);
    world.addBody(a);
    world.addBody(b);
    BoxCollider box1(Vec3(0.5f, 0.5f, 0.5f));
    BoxCollider box2(Vec3(0.5f, 0.5f, 0.5f));
    Rigidbody c(Vec3(-2, 0, 0), Vec3(3, 0, 0), &sphere1, 1.0f);
    Rigidbody d(Vec3(2, 0, 0), Vec3(-3, 0, 0), &box2, 1.0f);
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
    const float dt=1.0f/60.0f;

    float simulation_time=0.0f;
    float total_runtime=3.0f; // we will run the simulation for 3 seconds in the startung testing phase

    auto last_time=std::chrono::high_resolution_clock::now();
    float accumulator=0.0f;
    int frame=0;
    while(simulation_time<total_runtime){
       auto current_time=std::chrono::high_resolution_clock::now();
       float frametime=std::chrono::duration<float>(current_time-last_time).count();
       last_time=current_time;

       accumulator+=frametime;
       while(accumulator>=dt){
            world.step(dt);
            frame++;

            simulation_time+=dt;
            accumulator-=dt;
            std::cout<<"Time: "<<simulation_time<<"  Frame: "<<frame<<'\n';
            std::cout<<"First sphere: \n Position: ";
            std::cout<<world.getBodies()[2].position.x<<", "<<world.getBodies()[2].position.y<<", "<<world.getBodies()[2].position.z<<"\n Velocity: ";
            std::cout<<world.getBodies()[2].velocity.x<<", "<<world.getBodies()[2].velocity.y<<", "<<world.getBodies()[2].velocity.z<<"\n \n";
            
            std::cout<<"Second Box: \n Position: ";
            std::cout<<world.getBodies()[3].position.x<<", "<<world.getBodies()[3].position.y<<", "<<world.getBodies()[3].position.z<<"\n Velocity: ";
            std::cout<<world.getBodies()[3].velocity.x<<", "<<world.getBodies()[3].velocity.y<<", "<<world.getBodies()[3].velocity.z<<"\n \n";

       }
       
       std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Checking opengl
    RunOpenGLTest();

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