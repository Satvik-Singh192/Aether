#include "main.h"
#include "test_scenarios.hpp"
#include "world/physicsworld.hpp"
#include "../renderer/window.hpp"

#include <slint.h>
#include <iostream>

int main()
{
    auto ui = AppWindow::create();
    PhysicsWorld world;
    bool start_project_requested = false;

    auto load_selected_case = [&]()
    {
        world = PhysicsWorld();
        world.setGravity(Vec3(0.0f, ui->get_gravity_y(), 0.0f));

        const int scenario_id = ui->get_selected_scenario();
        TestCase active_case = TestCase::BoxRamp;
        if (scenario_id == 0)
        {
            active_case = TestCase::SphereSphere;
        }
        else if (scenario_id == 31)
        {
            active_case = TestCase::RampRampStress;
        }

        LoadSingleTestScenario(world, active_case);
    };

    ui->on_start_simulation([&]
                            {
        load_selected_case();
        start_project_requested = true;

        std::cout << "[Launcher] Starting project window" << std::endl;

        slint::quit_event_loop(); });

    ui->run();

    if (start_project_requested)
    {
        CreateWindow(world);
    }

    return 0;
}
