#pragma once

#include <glm/glm.hpp>

class Camera
{
public:
    Camera();

    void moveForward(float deltaTime);
    void moveBackward(float deltaTime);
    void moveLeft(float deltaTime);
    void moveRight(float deltaTime);

    glm::mat4 getViewMatrix() const;
    const glm::vec3 &getPosition() const;

private:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    float speed;
};
