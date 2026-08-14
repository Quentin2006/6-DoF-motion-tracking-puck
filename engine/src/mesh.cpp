#include "mesh.h"
#include "../include/glad/glad.h"
#include "math/spline.h"
#include "vertexBuffer.h"

#include <cmath>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_transform.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/fwd.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/trigonometric.hpp>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "../include/stb/stb_image.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "../include/tol/tiny_obj_loader.h"

#include <iostream>

Mesh::Mesh(glm::vec3 color)
    : buffer(), vertexCount(0), imageTextureId(0), specularTextureId(1),
      diffuseTextureId(2), shininess(32.0f), color(color) {
  // Create default 1x1 texture with the given color (rgba)
  unsigned char pixel[] = {
      static_cast<unsigned char>(glm::clamp(color.r, 0.0f, 1.0f) * 255.0f),
      static_cast<unsigned char>(glm::clamp(color.g, 0.0f, 1.0f) * 255.0f),
      static_cast<unsigned char>(glm::clamp(color.b, 0.0f, 1.0f) * 255.0f),
      255};

  // NOTE: needed to manualy init texter with all params
  auto initTex = [&](GLuint &id) {
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 pixel);
  };

  initTex(imageTextureId);
  initTex(specularTextureId);
  initTex(diffuseTextureId);

  // Set reasonable default params for the placeholder texture
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // Upload single-pixel RGBA
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               pixel);
}

void Mesh::draw() {
  glBindVertexArray(buffer.getVAO());

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, imageTextureId);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, specularTextureId);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, diffuseTextureId);

  glDrawArrays(GL_TRIANGLES, 0, vertexCount);
}

bool Mesh::setTexture(const std::string &path, TextureType type) {
  // Delete old texture if exists
  if (type == TextureType::Diffuse && diffuseTextureId != 0) {
    glDeleteTextures(1, &diffuseTextureId);
    diffuseTextureId = 0;
  } else if (type == TextureType::Specular && specularTextureId != 0) {
    glDeleteTextures(1, &specularTextureId);
    specularTextureId = 0;
  } else if (type == TextureType::Image && imageTextureId != 0) {
    glDeleteTextures(1, &imageTextureId);
    imageTextureId = 0;
  }

  if (type == TextureType::Diffuse) {
    glGenTextures(1, &diffuseTextureId);
    glBindTexture(GL_TEXTURE_2D, diffuseTextureId);
  } else if (type == TextureType::Specular) {
    glGenTextures(1, &specularTextureId);
    glBindTexture(GL_TEXTURE_2D, specularTextureId);
  } else if (type == TextureType::Image) {
    glGenTextures(1, &imageTextureId);
    glBindTexture(GL_TEXTURE_2D, imageTextureId);
  }

  // Texture parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Pixel alignment for arbitrary widths
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  int width = 0, height = 0, nrChannels = 0;
  unsigned char *data =
      stbi_load(path.c_str(), &width, &height, &nrChannels, 0);
  if (!data) {
    std::cerr << "Failed to load texture: " << path << std::endl;
    // fallback: single white pixel
    unsigned char white[] = {255, 255, 255, 255};
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 white);
    return false;
  }

  // Check max texture size
  GLint maxSize = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxSize);
  if (width > maxSize || height > maxSize) {
    std::cerr << "Texture " << path << " is " << width << "x" << height
              << " which exceeds GL_MAX_TEXTURE_SIZE (" << maxSize
              << "). Falling back to white texture.\n";
    stbi_image_free(data);
    unsigned char white[] = {255, 255, 255, 255};
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 white);
    return false;
  }

  GLenum format = GL_RGB;
  GLenum internalFormat = GL_RGB8;
  if (nrChannels == 1) {
    format = GL_RED;
    internalFormat = GL_R8;
  } else if (nrChannels == 3) {
    format = GL_RGB;
    internalFormat = GL_RGB8;
  } else if (nrChannels == 4) {
    format = GL_RGBA;
    internalFormat = GL_RGBA8;
  } else {
    // Unexpected channel count — treat as RGBA
    format = GL_RGBA;
    internalFormat = GL_RGBA8;
  }

  // Upload and generate mipmaps
  glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format,
               GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);

  stbi_image_free(data);
  std::cerr << "Loaded texture: " << path << " (" << width << "x" << height
            << ", " << nrChannels << "chan)\n";
  return true;
}

int Mesh::loadObj(const std::string &filePath, const std::string &objFileName,
                  const std::string &texturePath) {
  tinyobj::ObjReader reader;
  tinyobj::ObjReaderConfig readerConfig;

  if (!reader.ParseFromFile(filePath + objFileName, readerConfig)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error() << std::endl;
    }
    return 0;
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto &attrib = reader.GetAttrib();
  auto &shapes = reader.GetShapes();
  auto &materials = reader.GetMaterials();

  if (!materials.empty()) {
    const auto &mat = materials[0];
    // set diffuse texture if present
    if (!mat.diffuse_texname.empty()) {
      setTexture(filePath + mat.diffuse_texname, TextureType::Diffuse);
    }
    // set specular texture if present
    if (!mat.specular_texname.empty()) {
      setTexture(filePath + mat.specular_texname, TextureType::Specular);
    }
    if (mat.shininess > 1.0f) {
      shininess = mat.shininess;
    }
  }

  if (!texturePath.empty()) {
    setTexture(texturePath, TextureType::Image);
  }

  vertices.clear();
  vertices.reserve(attrib.vertices.size() / 3);

  for (size_t s = 0; s < shapes.size(); s++) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      glm::vec3 faceNormal(0, 0, 1);
      if (fv >= 3 &&
          shapes[s].mesh.indices[index_offset].normal_index < 0) { // compute
        tinyobj::index_t idx0 = shapes[s].mesh.indices[index_offset + 0];
        tinyobj::index_t idx1 = shapes[s].mesh.indices[index_offset + 1];
        tinyobj::index_t idx2 = shapes[s].mesh.indices[index_offset + 2];

        glm::vec3 v0(attrib.vertices[3 * idx0.vertex_index + 0],
                     attrib.vertices[3 * idx0.vertex_index + 1],
                     attrib.vertices[3 * idx0.vertex_index + 2]);
        glm::vec3 v1(attrib.vertices[3 * idx1.vertex_index + 0],
                     attrib.vertices[3 * idx1.vertex_index + 1],
                     attrib.vertices[3 * idx1.vertex_index + 2]);
        glm::vec3 v2(attrib.vertices[3 * idx2.vertex_index + 0],
                     attrib.vertices[3 * idx2.vertex_index + 1],
                     attrib.vertices[3 * idx2.vertex_index + 2]);

        faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
      }

      for (size_t v = 0; v < fv; v++) {
        Vertex vertex;
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
        tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
        tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
        vertex.position = glm::vec3(vx, vy, vz);

        if (idx.normal_index >= 0) {
          tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
          tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
          tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
          vertex.normal = glm::vec3(nx, ny, nz);
        } else {
          vertex.normal = faceNormal;
        }

        if (idx.texcoord_index >= 0) {
          tinyobj::real_t tx =
              attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
          tinyobj::real_t ty =
              1 - attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
          vertex.texCoord = glm::vec2(tx, ty);
        } else {
          vertex.texCoord = glm::vec2(0.0f);
        }

        vertices.push_back(vertex);
      }
      index_offset += fv;
    }
  }

  buffer.uploadVertices(vertices);
  vertexCount = static_cast<int>(vertices.size());
  return vertexCount;
}

// Sweep and generateCircle implementations remain the same as your original
// file (omitted here for brevity — copy-paste your existing implementations).
int Mesh::loadSweep(const std::vector<glm::vec3> &points, int pathSegments,
                    int circleSegments, float radius) {
  if (points.size() < 2)
    return 0;

  // Detect cyclic path: first and last points match
  bool cyclic = glm::length(points.front() - points.back()) < 0.001f;

  // Subdivide control points into a smooth spline path
  std::vector<glm::vec3> smoothPath =
      spline::subdivide(points, pathSegments, cyclic);

  std::vector<glm::vec3> circle_points = generateCircle(circleSegments, radius);

  // circles[i] corresponds to the circle centered at smoothPath[i]
  std::vector<std::vector<glm::vec3>> circles(smoothPath.size());

  glm::vec3 prevTangent(0, 0, 1);

  for (int i = 0; i < (int)smoothPath.size(); ++i) {
    // Compute tangent once per path point (independent of circle vertex j)
    glm::vec3 tangentRaw;
    if (i == 0) {
      tangentRaw = smoothPath[i + 1] - smoothPath[i];
    } else if (i == (int)smoothPath.size() - 1) {
      tangentRaw = smoothPath[i] - smoothPath[i - 1];
    } else {
      tangentRaw = smoothPath[i + 1] - smoothPath[i - 1];
    }

    // Guard against zero-length tangent (duplicate spline samples) to avoid
    // NaN
    glm::vec3 tangent;
    float tangentLen = glm::length(tangentRaw);
    if (tangentLen < 0.0001f) {
      tangent = prevTangent;
    } else {
      tangent = tangentRaw / tangentLen;
    }
    prevTangent = tangent;

    // Avoid degenerate case when tangent is parallel to up vector
    glm::vec3 up(0, 1, 0);
    if (std::abs(glm::dot(tangent, up)) > 0.99f) {
      up = glm::vec3(1, 0, 0);
    }

    glm::vec3 span = glm::normalize(cross(tangent, up));
    glm::vec3 normal = glm::normalize(cross(span, tangent));
    glm::mat3 basis(normal, tangent, span);

    std::vector<glm::vec3> translated_circle(circleSegments);
    for (int j = 0; j < (int)circle_points.size(); ++j) {
      translated_circle[j] = basis * circle_points[j] + smoothPath[i];
    }
    circles[i] = std::move(translated_circle);
  }

  // Connect circles into triangle mesh
  for (int i = 1; i < (int)circles.size(); ++i) {
    for (int j = 0; j < circleSegments; ++j) {
      int next_j = (j + 1) % circleSegments;

      glm::vec3 p0 = circles[i - 1][j];
      glm::vec3 p1 = circles[i][j];
      glm::vec3 p2 = circles[i][next_j];
      glm::vec3 p3 = circles[i - 1][next_j];

      glm::vec3 edge1 = p1 - p0;
      glm::vec3 edge2 = p2 - p0;
      glm::vec3 normal1 = glm::normalize(glm::cross(edge1, edge2));

      vertices.push_back({p1, glm::vec2(0.0f), normal1});
      vertices.push_back({p0, glm::vec2(0.0f), normal1});
      vertices.push_back({p2, glm::vec2(0.0f), normal1});

      glm::vec3 edge3 = p2 - p0;
      glm::vec3 edge4 = p3 - p0;
      glm::vec3 normal2 = glm::normalize(glm::cross(edge3, edge4));

      vertices.push_back({p2, glm::vec2(0.0f), normal2});
      vertices.push_back({p0, glm::vec2(0.0f), normal2});
      vertices.push_back({p3, glm::vec2(0.0f), normal2});
    }
  }

  int last = smoothPath.size() - 1;
  glm::vec3 tangentAtStart = glm::normalize(smoothPath[1] - smoothPath[0]);
  glm::vec3 tangentAtEnd =
      glm::normalize(smoothPath[last] - smoothPath[last - 1]);
  if (!cyclic) {
    // START CAP - at circles[0]
    // Center is smoothPath[0]
    // Normal points opposite to sweep direction (along -tangent)
    // Reverse winding order so normals face outward
    for (int j = 0; j < circleSegments; ++j) {
      int next_j = (j + 1) % circleSegments;

      // Build triangles: center → next_j → j (reversed winding)
      vertices.push_back(
          {circles[0][next_j], glm::vec2(0.0f), -tangentAtStart});
      vertices.push_back({circles[0][j], glm::vec2(0.0f), -tangentAtStart});
      vertices.push_back({smoothPath[0], glm::vec2(0.0f), -tangentAtStart});
    }

    // END CAP - at circles[last]
    // Center is smoothPath[last]
    // Normal points along sweep direction
    for (int j = 0; j < circleSegments; ++j) {
      int next_j = (j + 1) % circleSegments;

      // Build triangles: center → j → next_j (normal winding)
      vertices.push_back({smoothPath[last], glm::vec2(0.0f), tangentAtEnd});
      vertices.push_back({circles[last][j], glm::vec2(0.0f), tangentAtEnd});
      vertices.push_back(
          {circles[last][next_j], glm::vec2(0.0f), tangentAtEnd});
    }
  }

  buffer.uploadVertices(vertices);
  vertexCount = vertices.size();
  return vertexCount;
}

const std::vector<glm::vec3> Mesh::generateCircle(int res, float radius) {
  std::vector<glm::vec3> circle_points(res);

  // We want to go 360/res points from the circle untill we hit 360
  float cur_degree = 0;
  for (auto &point : circle_points) {
    point = glm::vec3(cos(glm::radians(cur_degree)) * radius, 0,
                      sin(glm::radians(cur_degree)) * radius);

    cur_degree += 360.f / res;
  }

  return circle_points;
}
