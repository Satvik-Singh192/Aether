#include "camera.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

Camera::Camera()
    : position(0.0f, 5.0f, 15.0f),
      front(0.0f, 0.0f, -1.0f),
      up(0.0f, 1.0f, 0.0f),
      worldUp(0.0f, 1.0f, 0.0f),
      yaw(-90.0f),
      pitch(0.0f),
      mouseSensitivity(0.12f),
      speed(6.0f)
{
    updateVectors();
}

void Camera::rotate(float yawOffsetDegrees, float pitchOffsetDegrees)
{
    yaw += yawOffsetDegrees * mouseSensitivity;
    pitch += pitchOffsetDegrees * mouseSensitivity;
    if (pitch > 89.0f)
    {
        pitch = 89.0f;
    }
    else if (pitch < -89.0f)
    {
        pitch = -89.0f;
    }
    updateVectors();
}

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
    glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
    position -= right * (speed * deltaTime);
}

void Camera::moveRight(float deltaTime)
{
    glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
    position += right * (speed * deltaTime);
}

void Camera::updateVectors()
{
    const float yawRadians = glm::radians(yaw);
    const float pitchRadians = glm::radians(pitch);

    glm::vec3 updatedFront;
    updatedFront.x = std::cos(yawRadians) * std::cos(pitchRadians);
    updatedFront.y = std::sin(pitchRadians);
    updatedFront.z = std::sin(yawRadians) * std::cos(pitchRadians);
    front = glm::normalize(updatedFront);

    glm::vec3 right = glm::normalize(glm::cross(front, worldUp));
    up = glm::normalize(glm::cross(right, front));
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::lookAt(position, position + front, up);
}

const glm::vec3 &Camera::getPosition() const
{
    return position;
}
