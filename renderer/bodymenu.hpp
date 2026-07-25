#pragma once

#include "api/AetherAPI.hpp"

void RenderBodyMenu(AetherAPI &api);
void RenderAddBodyMenuContent(AetherAPI &api);
void RenderConstraintMenuContent(AetherAPI &api);
void RenderWorldMenuContent(AetherAPI &api);
void RenderBodyInspectorContent(AetherAPI &api, bool showCloseButton = false);
void RenderEnginePopups();
