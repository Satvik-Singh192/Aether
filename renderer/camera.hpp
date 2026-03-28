#pragma once

#include <glm/glm.hpp>

class Camera
{
public:
    Camera();
    Camera& setPosition(glm::vec3 pos) { position = pos; updateVectors(); return *this; }
    Camera& setYaw(float y)            { yaw = y; updateVectors(); return *this; }
    Camera& setPitch(float p)          { pitch = p; updateVectors(); return *this; }
    Camera& setSpeed(float s)          { speed = s; return *this; }
    Camera& setSensitivity(float sens) { mouseSensitivity = sens; return *this; }

    void moveForward(float deltaTime);
    void moveBackward(float deltaTime);
    void moveLeft(float deltaTime);
    void moveRight(float deltaTime);
    void rotate(float yawOffsetDegrees, float pitchOffsetDegrees);

    glm::mat4 getViewMatrix() const;
    const glm::vec3 &getPosition() const;

private:
    void updateVectors();

    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 worldUp;
    float yaw;
    float pitch;
    float mouseSensitivity;
    float speed;
};
