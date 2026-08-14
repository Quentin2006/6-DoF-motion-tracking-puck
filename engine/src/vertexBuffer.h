#pragma once

#include <cstddef>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <vector>

struct Vertex {
  glm::vec3 position = glm::vec3(0.0f);
  glm::vec2 texCoord = glm::vec2(0.0f);
  glm::vec3 normal = glm::vec3(0.0f);
  unsigned int materialId;
};

class vertexBuffer {
public:
  vertexBuffer();
  ~vertexBuffer();

  // Prevent copying/moving (OpenGL resources must stay in one place)
  vertexBuffer(const vertexBuffer &) = delete;
  vertexBuffer &operator=(const vertexBuffer &) = delete;
  vertexBuffer(vertexBuffer &&) = delete;
  vertexBuffer &operator=(vertexBuffer &&) = delete;

  // Upload vertex data to GPU and configure vertex attributes.
  // Must be called before rendering. Can be called multiple times to update
  // data.
  void uploadVertices(const std::vector<Vertex> &vertices);

  // Returns the OpenGL VAO handle for binding before drawing
  unsigned int getVAO() { return VAO; }

private:
  unsigned int
      VAO; // Vertex Array Object - stores vertex attribute configuration
  unsigned int VBO; // Vertex Buffer Object - stores actual vertex data on GPU
};
