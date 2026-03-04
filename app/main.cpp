#include <iostream>
#include<chrono>
#include<thread>
#include "math/vec3.hpp"
#include "core/rigidbody.hpp"
#include "engine/physicsworld.hpp"
#include "collision/collision.hpp"
#include "../renderer/opengl_test.hpp"
using namespace std;
int main(){
    cout << "Hello Worlds\n";
    cout << "Testing\n";

    PhysicsWorld world;
    Rigidbody test_body(Vec3(0.0f,16.0f,0.0f),Vec3(2.0f,0.0f,0.0f),4.0f);
    world.addBody(test_body);
    const float dt=1.0f/60.0f;

    float simulation_time=0.0f;
    float total_runtime=3.0f; // we will run the simulation for 3 seconds in the startung testing phase

    auto last_time=std::chrono::high_resolution_clock::now();
    float accumulator=0.0f;

    while(simulation_time<total_runtime){
       auto current_time=std::chrono::high_resolution_clock::now();
       float frametime=std::chrono::duration<float>(current_time-last_time).count();
       last_time=current_time;

       accumulator+=frametime;
       while(accumulator>=dt){
           world.step(dt);

           simulation_time+=dt;
           accumulator-=dt;
       }
       std::cout<<"Time: "<<simulation_time<<"  Y Position: "<<world.getBodies()[0].position.y<<std::endl;
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