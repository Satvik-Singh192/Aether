#pragma once
#include "../engine/world/physicsworld.hpp"
#include "camera.hpp"

void initDrawBodies();
void RenderBodies(PhysicsWorld &world, const Camera &camera, float aspectRatio);
void SetBodyDrawWireframeMode(bool wireframe);
bool GetBodyDrawWireframeMode();
void SetBodyTint(float r, float g, float b);
void GetBodyTint(float &r, float &g, float &b);