#pragma once

#include "vertexBuffer.h"
#include <glm/detail/qualifier.hpp>
#include <glm/ext/vector_float3.hpp>
#include <string>
#include <vector>

#pragma once
#include "vertexBuffer.h"
#include <glm/glm.hpp>
#include <string>
#include <vector>

enum TextureType { Diffuse, Specular, Image };

class Mesh {
public:
  Mesh(glm::vec3 color = glm::vec3(1.0f));
  void draw();
  int loadObj(const std::string &filePath, const std::string &objFileName,
              const std::string &texturePath = "");
  int loadVertices(const std::vector<Vertex> &v) {
    vertices.clear();
    vertices = v;
    vertexCount = vertices.size();
    buffer.uploadVertices(vertices);
    return vertices.size();
  }
  bool setTexture(const std::string &path, TextureType type);
  int loadSweep(const std::vector<glm::vec3> &points, int pathSegments,
                int circleSegments, float radius);

  float getShininess() const { return shininess; }

private:
  vertexBuffer buffer;
  int vertexCount;
  unsigned int imageTextureId;
  unsigned int specularTextureId;
  unsigned int diffuseTextureId;
  float shininess;
  glm::vec3 color;
  std::vector<Vertex> vertices;

  // other helper methods (generateCircle, loadSweep, etc.)
  const std::vector<glm::vec3> generateCircle(int res, float radius);
};
