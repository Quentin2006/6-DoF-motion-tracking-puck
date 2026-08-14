#include "callbacks.h"
#include "app.h"
#include "controls.h"
#include <glm/gtc/type_ptr.hpp>

void key_callback(GLFWwindow *window, int key, int /*scancode*/, int action,
                  int /*mods*/) {
  App *app = static_cast<App *>(glfwGetWindowUserPointer(window));
  if (!app)
    return;

  bool pressed = (action == GLFW_PRESS);
  bool released = (action == GLFW_RELEASE);

  if (!pressed && !released)
    return;

  InputState *input = app->getInputState();
  switch (key) {
  case Controls::MOVE_FORWARD:
    input->w = pressed;
    return;
  case Controls::MOVE_BACKWARD:
    input->s = pressed;
    return;
  case Controls::MOVE_LEFT:
    input->a = pressed;
    return;
  case Controls::MOVE_RIGHT:
    input->d = pressed;
    return;
  case Controls::MOVE_DOWN:
    input->q = pressed;
    return;
  case Controls::MOVE_UP:
    input->e = pressed;
    return;
  case Controls::ROTATE_PITCH_UP:
    input->up = pressed;
    return;
  case Controls::ROTATE_PITCH_DOWN:
    input->down = pressed;
    return;
  case Controls::ROTATE_YAW_LEFT:
    input->left = pressed;
    return;
  case Controls::ROTATE_YAW_RIGHT:
    input->right = pressed;
    return;
  case Controls::NEXT_CAMERA:
    input->c = pressed;
    return;
  case Controls::SUBDIV_UP:
    input->subdivUp = pressed;
    return;
  case Controls::SUBDIV_DOWN:
    input->subdivDown = pressed;
    return;
  default:
    return;
  }
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  App *app = static_cast<App *>(glfwGetWindowUserPointer(window));
  if (!app)
    return;

  app->getWindow()->setWidth(width);
  app->getWindow()->setHeight(height);

  for (auto &camera : *app->getCameras()) {
    camera->updateAspect(width, height);
  }

  auto &cams = *app->getCameras();
  auto *shader = app->getShader();
  glUniformMatrix4fv(
      shader->getUniformLocation("projection"), 1, GL_FALSE,
      glm::value_ptr(cams[app->getCameraIndex()]->getProjectionMatrix()));

  glViewport(0, 0, width, height);
}
