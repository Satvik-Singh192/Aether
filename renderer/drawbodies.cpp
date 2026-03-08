#include "drawBodies.hpp"
#include <glad/glad.h>
#include <vector>

static GLuint shaderProgram;
static GLuint VAO, VBO;

static const char* vertexShaderSrc = R"(
#version 330 core
layout (location = 0) in vec2 aPos;
void main()
{
    gl_Position = vec4(aPos, 0.0, 1.0);
    gl_PointSize = 10.0;
}
)";

static const char* fragmentShaderSrc = R"(
#version 330 core
out vec4 FragColor;
void main()
{
    FragColor = vec4(1.0,1.0,1.0,1.0);
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
    glBufferData(GL_ARRAY_BUFFER, 1000 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void RenderBodies(PhysicsWorld& world)
{
    std::vector<float> vertices;
        for (auto& body : world.getBodies())
        {
            float scale = 0.1f;
            vertices.push_back(body.position.x * scale);
            vertices.push_back(body.position.y * scale);
        }

    glUseProgram(shaderProgram);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());

    glDrawArrays(GL_POINTS, 0, vertices.size() / 2);
}
