#pragma once

#include <GLFW/glfw3.h>
#include <string>
class Window {
public:
  Window(int width, int height, const std::string& title);

  int getWidth() { return width; }
  int getHeight() { return height; }

  void setWidth(int width_) { width = width_; }
  void setHeight(int height_) { height = height_; }

  bool shouldClose() { return glfwWindowShouldClose(window); }

  void swapBuffers() {
    glfwPollEvents();
    glfwSwapBuffers(window);
  }

  void setWindowCallback(GLFWkeyfun callback) {
    glfwSetKeyCallback(window, callback);
  }

  GLFWwindow *getGLFWwindow() { return window; }

private:
  GLFWwindow *window;
  int width, height;
};
