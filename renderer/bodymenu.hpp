#pragma once

#include "../engine/world/physicsworld.hpp"

void RenderBodyMenu(PhysicsWorld &world);
void RenderAddBodyMenuContent(PhysicsWorld &world);
void RenderConstraintMenuContent(PhysicsWorld &world);
void RenderWorldMenuContent(PhysicsWorld &world);
void RenderBodyInspectorContent(PhysicsWorld &world, bool showCloseButton = false);
void RenderEnginePopups();
