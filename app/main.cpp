#include "main.h"
#include "test_scenarios.hpp"
#include "world/physicsworld.hpp"
#include "../renderer/window.hpp"

#include <slint_timer.h>
#include <slint.h>
#include <chrono>
#include <iostream>

int main()
{
    auto ui = AppWindow::create();
    PhysicsWorld world;
    bool start_project_requested = false;

    ui->on_start_simulation([&]
                            {
        const int scenario_id = ui->get_selected_scenario();
        const float gravity_y = ui->get_gravity_y();

        world = PhysicsWorld();
        world.setGravity(Vec3(0.0f, gravity_y, 0.0f));

        TestCase selected_case = TestCase::BoxRamp;
        if (scenario_id == 0) {
            selected_case = TestCase::SphereSphere;
        } else if (scenario_id == 31) {
            selected_case = TestCase::RampRampStress;
        }

        LoadSingleTestScenario(world, selected_case);
        start_project_requested = true;

        std::cout << "[Launcher] Starting main project window"
                  << " scenario=" << scenario_id
                  << " gravityY=" << gravity_y << std::endl;

        slint::quit_event_loop(); });

    ui->run();

    if (start_project_requested)
    {
        CreateWindow(world);
    }

    return 0;
}
