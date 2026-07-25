#pragma once
#include "api/AetherAPI.hpp"
#include "camera.hpp"

void initDrawBodies();
void RenderBodies(AetherAPI &api, const Camera &camera, float aspectRatio);
void SetBodyDrawWireframeMode(bool wireframe);
bool GetBodyDrawWireframeMode();
void SetBodyTint(float r, float g, float b);
void GetBodyTint(float &r, float &g, float &b);
void SetBodyVelocityArrowVisible(bool enabled);
bool GetBodyVelocityArrowVisible();