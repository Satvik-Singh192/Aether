#include <iostream>
#include<chrono>
#include<thread>
#include "vec3.hpp"

using namespace std;
int main(){
    cout << "Hello Worlds\n";
    cout << "Testing";

    const float dt=1.0f/60.0f;
    const Vec3 gravity(0.0f,-9.8f,0.0f);

    Vec3 position(0.0f,10.0f,0.0f);
    Vec3 velocity;

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
            velocity=velocity+(gravity*dt);
            position=position+(velocity*dt);

            simulation_time+=dt;
            accumulator-=dt;
        }
        std::cout<<"Time: "<<simulation_time<<"  Y Position: "<< position.y<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    return 0;
}