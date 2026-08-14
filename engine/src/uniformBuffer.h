#pragma once

#include <cstddef>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>

// NOTE: any object that stays static per frame throw in UBO
constexpr int MAX_LIGHTS = 128;

struct alignas(32) LightData {
  glm::vec3 position;
  float _padding1;
  glm::vec3 color;
  float intensity;
};

struct LightBlock {
  LightData lights[MAX_LIGHTS];
  int count;
  int _padding[3]; // Pad to vec4 alignment
};

struct CameraBlock {
  glm::mat4 view;
  glm::mat4 projection;
};

class UniformBuffer {
public:
  UniformBuffer();
  ~UniformBuffer();

  // Prevent copying/moving (OpenGL resources must stay in one place)
  UniformBuffer(const UniformBuffer &) = delete;
  UniformBuffer &operator=(const UniformBuffer &) = delete;
  UniformBuffer(UniformBuffer &&) = delete;
  UniformBuffer &operator=(UniformBuffer &&) = delete;

  // Upload data to GPU buffer. Can be called multiple times to update data.
  void uploadData(const void *data, size_t size);

  // Bind buffer to a specific binding point for shaders to access
  void bindToPoint(unsigned int bindingPoint);

  // Get the OpenGL buffer handle
  unsigned int getBuffer() { return UBO; }

private:
  unsigned int UBO; // Uniform Buffer Object handle
};
