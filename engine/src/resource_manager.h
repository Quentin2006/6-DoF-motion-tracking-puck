#pragma once

#include <glm/ext/vector_float3.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Mesh;
class Shader;

class ResourceManager {
public:
  ResourceManager() = default;

  // Load mesh from file, returns cached copy if already loaded
  std::shared_ptr<Mesh> loadMesh(const std::string &path,
                                 const std::string &filename,
                                 const std::string &texturePath = "");

  // Loads mesh from vector of vec3s, radius and res
  std::shared_ptr<Mesh> loadMesh(const std::vector<glm::vec3> &verts,
                                 int pathSegments, int circleSegments,
                                 float radius,
                                 glm::vec3 color = {1.f, 1.f, 1.f});

private:
  std::unordered_map<std::string, std::weak_ptr<Mesh>> meshCache;
};
