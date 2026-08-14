#include "camera.h"
#include <glm/trigonometric.hpp>

void Camera::rotateYaw(float delta) {
  yaw += delta;
  updateFront();
}

void Camera::rotatePitch(float delta) {
  pitch += delta;
  // Clamp pitch to prevent flipping
  if (pitch > 85.0f)
    pitch = 85.0f;
  if (pitch < -85.0f)
    pitch = -85.0f;
  updateFront();
}

void Camera::moveForward(float distance) { position += front * distance; }

void Camera::moveRight(float distance) {
  glm::vec3 right = glm::normalize(glm::cross(front, up));
  position += right * distance;
}

void Camera::moveUp(float distance) { position += up * distance; }

void Camera::updateFront() {
  glm::vec3 newFront;
  newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
  newFront.y = sin(glm::radians(pitch));
  newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
  front = glm::normalize(newFront);
}
