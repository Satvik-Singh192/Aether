#include "camera.hpp"

#include <glm/gtc/matrix_transform.hpp>

Camera::Camera()
    : position(0.0f, 5.0f, 15.0f),
      front(0.0f, 0.0f, -1.0f),
      up(0.0f, 1.0f, 0.0f),
      speed(6.0f) {}

void Camera::moveForward(float deltaTime)
{
    position += front * (speed * deltaTime);
}

void Camera::moveBackward(float deltaTime)
{
    position -= front * (speed * deltaTime);
}

void Camera::moveLeft(float deltaTime)
{
    glm::vec3 right = glm::normalize(glm::cross(front, up));
    position -= right * (speed * deltaTime);
}

void Camera::moveRight(float deltaTime)
{
    glm::vec3 right = glm::normalize(glm::cross(front, up));
    position += right * (speed * deltaTime);
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::lookAt(position, position + front, up);
}

const glm::vec3 &Camera::getPosition() const
{
    return position;
}
