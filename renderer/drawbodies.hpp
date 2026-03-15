#pragma once
#include "../engine/world/physicsworld.hpp"
#include "camera.hpp"

void initDrawBodies();
void RenderBodies(PhysicsWorld &world, const Camera &camera, float aspectRatio);