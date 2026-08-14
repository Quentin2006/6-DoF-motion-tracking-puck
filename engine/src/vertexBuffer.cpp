#include "vertexBuffer.h"
#include "../include/glad/glad.h"

vertexBuffer::vertexBuffer() : VAO(0), VBO(0) {
  // Generate OpenGL handles for VAO and VBO
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
}

vertexBuffer::~vertexBuffer() {
  // Only delete if we still own the resources (check for 0)
  if (VBO)
    glDeleteBuffers(1, &VBO);
  if (VAO)
    glDeleteVertexArrays(1, &VAO);
}

void vertexBuffer::uploadVertices(const std::vector<Vertex> &vertices) {
  // Skip if no data provided
  if (vertices.empty())
    return;

  // Bind VAO to store all subsequent configuration
  glBindVertexArray(VAO);
  // Bind VBO for data upload and attribute configuration
  glBindBuffer(GL_ARRAY_BUFFER, VBO);

  // Upload interleaved vertex data to GPU buffer
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
               vertices.data(), GL_STATIC_DRAW);

  // Configure attribute 0: position (3 floats)
  // Offset = 0, Stride = sizeof(Vertex)
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
  glEnableVertexAttribArray(0);

  // Configure attribute 1: texture coordinates (2 floats)
  // Offset = sizeof(glm::vec3) to skip position, Stride = sizeof(Vertex)
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void *)offsetof(Vertex, texCoord));
  glEnableVertexAttribArray(1);

  // Configure attribute 2: normals (3 floats)
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        (void *)offsetof(Vertex, normal));
  glEnableVertexAttribArray(2);

  // Note: We don't unbind VBO here because VAO needs to remember
  // which VBO is bound for this attribute configuration
}
