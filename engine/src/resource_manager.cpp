#include "resource_manager.h"
#include "mesh.h"
#include <glm/ext/vector_float3.hpp>
#include <iostream>

std::shared_ptr<Mesh>
ResourceManager::loadMesh(const std::string &path, const std::string &filename,
                          const std::string &texturePath) {
  std::string key = path + filename;

  // Check if already cached
  auto it = meshCache.find(key);
  if (it != meshCache.end()) {
    if (auto shared = it->second.lock()) {
      return shared;
    }
    // Expired, remove from cache
    meshCache.erase(it);
  }

  // Load new mesh
  auto mesh = std::make_shared<Mesh>();
  int verts = mesh->loadObj(path, filename, texturePath);
  if (verts == 0) {
    std::cerr << "Failed to load mesh: " << key << std::endl;
    return nullptr;
  }

  meshCache[key] = mesh;
  return mesh;
}

std::shared_ptr<Mesh>
ResourceManager::loadMesh(const std::vector<glm::vec3> &verts, int pathSegments,
                          int circleSegments, float radius, glm::vec3 color) {
  auto mesh = std::make_shared<Mesh>(color);
  int size = mesh->loadSweep(verts, pathSegments, circleSegments, radius);
  if (size == 0) {
    std::cerr << "Cannot load mesh with no verts" << std::endl;
    return nullptr;
  }
  return mesh;
}
