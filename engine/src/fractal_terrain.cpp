#include "fractal_terrain.h"
#include "vertexBuffer.h"
#include <cmath>
#include <glm/common.hpp>
#include <glm/geometric.hpp>

// Deterministic noise based on position
// will always compute the same displacement for the shared midpoint.
static float positionalNoise(const glm::vec3 &pos, float scale) {
  float h = glm::dot(pos, glm::vec3(127.1f, 311.7f, 74.7f));
  h = std::sin(h) * 43758.5453f;
  h = h - std::floor(h);            // fract
  return (h - 0.5f) * 2.0f * scale; // range [-scale, +scale]
}

std::vector<Vertex> FractalTerrain::generateTerrain(int subDivCount,
                                                    const Vertex &v1,
                                                    const Vertex &v2,
                                                    const Vertex &v3) {
  // We want to linearaly interpolate on each point to get a new point halfway
  // repeate subDivCount times

  // init points
  faces.clear();
  faces.push_back({v1, v2, v3});

  generateSubDivEdges(subDivCount);

  // Recompute per-face normals from actual geometry.
  // After noise displacement the interpolated normals are stale (all still
  // point straight up). Compute the true face normal via cross product.
  for (auto &[A, B, C] : faces) {
    glm::vec3 edge1 = B.position - A.position;
    glm::vec3 edge2 = C.position - A.position;
    glm::vec3 faceNormal = glm::normalize(glm::cross(edge1, edge2));
    A.normal = faceNormal;
    B.normal = faceNormal;
    C.normal = faceNormal;
  }

  std::vector<Vertex> verts;

  // flatten faces into vertex list
  for (const auto &[A, B, C] : faces) {
    verts.push_back(A);
    verts.push_back(B);
    verts.push_back(C);
  }

  return verts;
}

Vertex midVertex(Vertex &v1, Vertex &v2) {
  glm::vec3 pos = glm::mix(v1.position, v2.position, 0.5f);
  glm::vec2 texCoord = glm::mix(v1.texCoord, v2.texCoord, 0.5f);

  // Normal is left as default (0,0,0) -- recomputed per-face after subdivision
  return {pos, texCoord};
}

void FractalTerrain::generateSubDivEdges(int subDivCount) {
  if (subDivCount == 0) {
    return;
  }

  // for all faces
  int faceCount = faces.size();
  for (int i = 0; i < faceCount; i++) {
    auto A = std::get<0>(faces[i]);
    auto B = std::get<1>(faces[i]);
    auto C = std::get<2>(faces[i]);

    // Compute midpoints for THIS face's 3 edges
    Vertex midAB = midVertex(A, B);
    Vertex midBC = midVertex(B, C);
    Vertex midCA = midVertex(C, A);

    // Apply deterministic noise to midpoints AFTER computing them.
    // Because the noise is a pure function of position, two faces
    // sharing an edge will produce the exact same displaced midpoint.
    float noiseScale = 0.05f;
    midAB.position.y += positionalNoise(midAB.position, noiseScale);
    midBC.position.y += positionalNoise(midBC.position, noiseScale);
    midCA.position.y += positionalNoise(midCA.position, noiseScale);

    // 4 new faces
    faces.push_back({A, midAB, midCA});
    faces.push_back({B, midBC, midAB});
    faces.push_back({C, midCA, midBC});
    faces.push_back({midAB, midBC, midCA});
  }

  // remove stale faces
  faces.erase(faces.begin(), faces.begin() + faceCount);

  return FractalTerrain::generateSubDivEdges(subDivCount - 1);
}
