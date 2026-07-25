#pragma once

#include "api/AetherAPI.hpp"
#include <glad/glad.h>
#include <glm/glm.hpp>

void RenderDistanceConstraintsSolid(
    AetherAPI &api,
    const glm::mat4 &view,
    const glm::mat4 &projection,
    GLuint program,
    GLuint vao,
    GLuint vbo,
    const glm::vec3 &lightDir,
    const glm::vec3 &camPos,
    float tintR,
    float tintG,
    float tintB);

void RenderDistanceConstraintsWire(
    AetherAPI &api,
    const glm::mat4 &model,
    const glm::mat4 &view,
    const glm::mat4 &projection,
    GLuint program,
    GLuint vao,
    GLuint vbo,
    float tintR,
    float tintG,
    float tintB);
