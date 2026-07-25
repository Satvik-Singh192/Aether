#include <emscripten.h>
#include <iostream>

#include "api/AetherAPI.hpp"

static AetherAPI engine;
static bool initialized = false;

extern "C" {

EMSCRIPTEN_KEEPALIVE
void Init()
{
    if (initialized) return;

    initialized = true;

    BoxSpawnInfo box;
    box.position = Vec3(0, 5, 0);
    box.mass = 1.0f;
    box.velocity = Vec3(0, 5, 0);
    engine.createBox(box);
}

EMSCRIPTEN_KEEPALIVE
void Step()
{
    engine.step(1.0f / 60.0f);

    auto bodies = engine.getRenderBodies();

    if (!bodies.empty())
    {
        std::cout
            << "Body position: "
            << bodies[0].position.x << " "
            << bodies[0].position.y << " "
            << bodies[0].position.z
            << std::endl;
    }
}

}