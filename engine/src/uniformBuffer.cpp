#include "uniformBuffer.h"
#include "../include/glad/glad.h"

UniformBuffer::UniformBuffer() : UBO(0) {
  // Generate OpenGL buffer handle
  glGenBuffers(1, &UBO);
}

UniformBuffer::~UniformBuffer() {
  // Only delete if we still own the resource
  if (UBO)
    glDeleteBuffers(1, &UBO);
}

void UniformBuffer::uploadData(const void *data, size_t size) {
  // Upload data to GPU buffer using GL_DYNAMIC_DRAW since lights may change
  // Note: Buffer must already be bound to GL_UNIFORM_BUFFER target via
  // bindToPoint()
  glBufferData(GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW);
}

void UniformBuffer::bindToPoint(unsigned int bindingPoint) {
  // Bind this buffer to the specified uniform binding point
  // This makes the buffer data available to shaders at binding = bindingPoint
  glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, UBO);
}
