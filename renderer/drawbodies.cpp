#include "drawbodies.hpp"
#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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
    gl_PointSize = 10.0;
}
)";

static const char *fragmentShaderSrc = R"(
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

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void RenderBodies(PhysicsWorld &world, const Camera &camera, float aspectRatio)
{
    std::vector<float> vertices;
    vertices.reserve(world.getBodies().size() * 3);
    for (auto &body : world.getBodies())
    {
        float scale = 0.1f;
        vertices.push_back(body.position.x * scale);
        vertices.push_back(body.position.y * scale);
        vertices.push_back(body.position.z * scale);
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

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(vertices.size() / 3));
}
