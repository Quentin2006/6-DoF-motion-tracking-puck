#pragma once

#include <GLFW/glfw3.h>

struct Controls {
  static const int MOVE_FORWARD = GLFW_KEY_W;
  static const int MOVE_BACKWARD = GLFW_KEY_S;
  static const int MOVE_LEFT = GLFW_KEY_A;
  static const int MOVE_RIGHT = GLFW_KEY_D;
  static const int MOVE_UP = GLFW_KEY_E;
  static const int MOVE_DOWN = GLFW_KEY_Q;

  static const int ROTATE_PITCH_UP = GLFW_KEY_UP;
  static const int ROTATE_PITCH_DOWN = GLFW_KEY_DOWN;
  static const int ROTATE_YAW_LEFT = GLFW_KEY_LEFT;
  static const int ROTATE_YAW_RIGHT = GLFW_KEY_RIGHT;

  static const int NEXT_CAMERA = GLFW_KEY_C;

  static const int SUBDIV_UP = GLFW_KEY_EQUAL;   // + key
  static const int SUBDIV_DOWN = GLFW_KEY_MINUS;  // - key
};
