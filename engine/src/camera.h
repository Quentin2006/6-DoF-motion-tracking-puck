#pragma once

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/trigonometric.hpp>

class Camera {
public:
  Camera(float FOV_, int width_, int height_, float zNear_ = 0.1f,
         float zFar_ = 1000.0f) {
    FOV = FOV_;
    width = width_;
    height = height_;
    zNear = zNear_;
    zFar = zFar_;

    position = glm::vec3(0.0f, 0.0f, 3.0f);
    up = glm::vec3(0.0f, 1.0f, 0.0f);
    yaw = -90.0f;
    pitch = 0.0f;
    updateFront();
    projection = glm::perspective(glm::radians(FOV),
                                  (float)width / (float)height, zNear, zFar);
  }

  void setPos(const glm::vec3 &pos) { position = pos; };

  void move(const glm::vec3 &deltaPos) { position += deltaPos; }
  void moveForward(float distance);
  void moveRight(float distance);
  void moveUp(float distance);
  void rotateYaw(float delta);
  void rotatePitch(float delta);

  void updateAspect(int width_, int height_) {
    width = width_;
    height = height_;

    projection = glm::perspective(glm::radians(FOV),
                                  (float)width / (float)height, zNear, zFar);
  }

  void updateFOV(float FOV_) {
    FOV = FOV_;
    projection = glm::perspective(glm::radians(FOV),
                                  (float)width / (float)height, zNear, zFar);
  }

  void updateZNearZFar(float zNear_, float zFar_) {
    zNear = zNear_;
    zFar = zFar_;

    projection = glm::perspective(glm::radians(FOV),
                                  (float)width / (float)height, zNear, zFar);
  }

  const glm::mat4 &getViewMatrix() {
    viewMatrix = glm::lookAt(position, position + front, up);
    return viewMatrix;
  };

  const glm::mat4 &getProjectionMatrix() { return projection; }
  const glm::vec3 &getPosition() const { return position; }

private:
  float FOV, zNear, zFar;
  int width, height;
  glm::vec3 position;
  glm::vec3 front;
  glm::vec3 up;
  glm::mat4 projection;
  glm::mat4 viewMatrix;
  float yaw;
  float pitch;

  void updateFront();
};
