#pragma once

#include "../engine/world/physicsworld.hpp"
#include <glad/glad.h>
#include <glm/glm.hpp>

void RenderDistanceConstraintsSolid(
    PhysicsWorld &world,
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
    PhysicsWorld &world,
    const glm::mat4 &model,
    const glm::mat4 &view,
    const glm::mat4 &projection,
    GLuint program,
    GLuint vao,
    GLuint vbo,
    float tintR,
    float tintG,
    float tintB);
