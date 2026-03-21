#include "drawbodies.hpp"
#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../engine/core/box_collider.hpp"
#include "../engine/core/sphere_collider.hpp"
#include "../engine/core/ramp_collider.hpp"
#include "bodyselection.hpp"

static GLuint shaderProgram;
static GLuint VAO, VBO;

static const char *vertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

void main()
{
    gl_Position = uProjection*uView*uModel*vec4(aPos, 1.0);
}
)";

static const char *fragmentShaderSrc = R"(
#version 330 core
out vec4 FragColor;

uniform vec4 uColor;

void main()
{
    FragColor = uColor;
}
)";

void initDrawBodies()
{
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertexShaderSrc, nullptr);
    glCompileShader(vs);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragmentShaderSrc, nullptr);
    glCompileShader(fs);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vs);
    glAttachShader(shaderProgram, fs);
    glLinkProgram(shaderProgram);

    glDeleteShader(vs);
    glDeleteShader(fs);

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

static void pushLine(std::vector<float> &v, const glm::vec3 &a, const glm::vec3 &b)
{
    v.push_back(a.x);
    v.push_back(a.y);
    v.push_back(a.z);
    v.push_back(b.x);
    v.push_back(b.y);
    v.push_back(b.z);
}

static void pushCircleLines(std::vector<float> &v, const glm::vec3 &center, float radius, int segments, int planeAxis0, int planeAxis1)
{
    auto point = [&](float t) -> glm::vec3 {
        glm::vec3 p = center;
        p[planeAxis0] += radius * std::cos(t);
        p[planeAxis1] += radius * std::sin(t);
        return p;
    };

    const float twoPi = 6.28318530718f;
    for (int i = 0; i < segments; ++i)
    {
        float t0 = (twoPi * i) / segments;
        float t1 = (twoPi * (i + 1)) / segments;
        pushLine(v, point(t0), point(t1));
    }
}

void RenderBodies(PhysicsWorld &world, const Camera &camera, float aspectRatio)
{
    // Separate vertex lists so we can color axes and body outlines differently
    std::vector<float> axisVertices;
    axisVertices.reserve(18); // 3 axes * 2 endpoints * 3 components

    // Define world axes (X, Y, Z)
    {
        const float axisLen = 100.0f;
        // X axis
        pushLine(axisVertices,
                 glm::vec3(-axisLen, 0.0f, 0.0f),
                 glm::vec3(axisLen, 0.0f, 0.0f));
        // Y axis
        pushLine(axisVertices,
                 glm::vec3(0.0f, -axisLen, 0.0f),
                 glm::vec3(0.0f, axisLen, 0.0f));
        // Z axis
        pushLine(axisVertices,
                 glm::vec3(0.0f, 0.0f, -axisLen),
                 glm::vec3(0.0f, 0.0f, axisLen));
    }

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = camera.getViewMatrix();

    glm::mat4 projection = glm::perspective(
        glm::radians(45.0f),
        aspectRatio,
        0.1f,
        100.0f);

    glUseProgram(shaderProgram);

    GLint modelLoc = glGetUniformLocation(shaderProgram, "uModel");
    GLint viewLoc = glGetUniformLocation(shaderProgram, "uView");
    GLint projLoc = glGetUniformLocation(shaderProgram, "uProjection");
    GLint colorLoc = glGetUniformLocation(shaderProgram, "uColor");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // Draw axes
    if (!axisVertices.empty())
    {
        glBufferData(GL_ARRAY_BUFFER, axisVertices.size() * sizeof(float), axisVertices.data(), GL_DYNAMIC_DRAW);
        if (colorLoc >= 0)
        {
            glUniform4f(colorLoc, 1.0f, 1.0f, 1.0f, 1.0f);
        }
        glLineWidth(3.0f);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(axisVertices.size() / 3));
    }

    // Time to hit on them bodies
    for (auto &body : world.getBodies())
    {
        if (!body.collider)
            continue;

        const glm::vec3 c(body.position.x, body.position.y, body.position.z);

        std::vector<float> bodyVertices;
        bodyVertices.reserve(72);
        
        if (body.collider->type == ShapeType::Box)
        {
            const auto *box = static_cast<const BoxCollider *>(body.collider);
            const glm::vec3 h(box->halfsize.x, box->halfsize.y, box->halfsize.z);

            const glm::vec3 p000 = c + glm::vec3(-h.x, -h.y, -h.z);
            const glm::vec3 p001 = c + glm::vec3(-h.x, -h.y, +h.z);
            const glm::vec3 p010 = c + glm::vec3(-h.x, +h.y, -h.z);
            const glm::vec3 p011 = c + glm::vec3(-h.x, +h.y, +h.z);
            const glm::vec3 p100 = c + glm::vec3(+h.x, -h.y, -h.z);
            const glm::vec3 p101 = c + glm::vec3(+h.x, -h.y, +h.z);
            const glm::vec3 p110 = c + glm::vec3(+h.x, +h.y, -h.z);
            const glm::vec3 p111 = c + glm::vec3(+h.x, +h.y, +h.z);

            // bottom
            pushLine(bodyVertices, p000, p100);
            pushLine(bodyVertices, p100, p101);
            pushLine(bodyVertices, p101, p001);
            pushLine(bodyVertices, p001, p000);
            // top
            pushLine(bodyVertices, p010, p110);
            pushLine(bodyVertices, p110, p111);
            pushLine(bodyVertices, p111, p011);
            pushLine(bodyVertices, p011, p010);
            // sides
            pushLine(bodyVertices, p000, p010);
            pushLine(bodyVertices, p100, p110);
            pushLine(bodyVertices, p101, p111);
            pushLine(bodyVertices, p001, p011);
        }
        else if (body.collider->type == ShapeType::Sphere)
        {
            const auto *sphere = static_cast<const SphereCollider *>(body.collider);
            const float r = sphere->radius;
            const int segments = 24;

            // 3 circles for a simple wire-sphere
            pushCircleLines(bodyVertices, c, r, segments, 0, 1); // XY
            pushCircleLines(bodyVertices, c, r, segments, 0, 2); // XZ
            pushCircleLines(bodyVertices, c, r, segments, 1, 2); // YZ
        }
        else if (body.collider->type == ShapeType::Ramp)
        {
            const auto *ramp = static_cast<const RampCollider *>(body.collider);
            const float L = ramp->length;
            const float H = ramp->getHeight();
            const float w = ramp->half_width_z;
            
            const float x0 = c.x;
            const float y0 = c.y;
            const float x1 = c.x + L;
            const float y1 = c.y + H;

            const float z0 = c.z - w;
            const float z1 = c.z + w;

            // Simple triangular prism, right triangle in X-Y, extruded along Z
            const glm::vec3 p0z0(x0, y0, z0);
            const glm::vec3 p1z0(x1, y0, z0);
            const glm::vec3 p2z0(x1, y1, z0);

            const glm::vec3 p0z1(x0, y0, z1);
            const glm::vec3 p1z1(x1, y0, z1);
            const glm::vec3 p2z1(x1, y1, z1);

            // bottom face edges
            pushLine(bodyVertices, p0z0, p1z0);
            pushLine(bodyVertices, p0z1, p1z1);

            // side face edges
            pushLine(bodyVertices, p1z0, p2z0);
            pushLine(bodyVertices, p1z1, p2z1);

            // hypotenuse edges
            pushLine(bodyVertices, p0z0, p2z0);
            pushLine(bodyVertices, p0z1, p2z1);

            // connect slices along Z
            pushLine(bodyVertices, p0z0, p0z1);
            pushLine(bodyVertices, p1z0, p1z1);
            pushLine(bodyVertices, p2z0, p2z1);
        }

        if (!bodyVertices.empty())
        {
            glBufferData(GL_ARRAY_BUFFER, bodyVertices.size() * sizeof(float), bodyVertices.data(), GL_DYNAMIC_DRAW);

            // Adding colour to wireframe based on stable body id
            BodyID key = body.id;
            float r = ((key * 73u) % 100) / 100.0f;
            float g = ((key * 37u) % 100) / 100.0f;
            float b = ((key * 19u) % 100) / 100.0f;

            // Add brightness
            r = 0.5f + 0.5f * r;
            g = 0.5f + 0.5f * g;
            b = 0.5f + 0.5f * b;

            if (colorLoc >= 0)
            {
                bool isSelected = (body.id == GetSelectedBodyId());
                if (isSelected) // different colour for the body selected in body menu
                {
                    glUniform4f(colorLoc, 1.0f, 1.0f, 0.2f, 1.0f);
                }
                else
                {
                    glUniform4f(colorLoc, r, g, b, 1.0f);
                }
            }
            if (body.id == GetSelectedBodyId())
                glLineWidth(4.0f); // thicker line for the selected body
            else
                glLineWidth(2.0f);
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(bodyVertices.size() / 3));
        }
    }
}
