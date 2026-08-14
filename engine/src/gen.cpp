#define GLM_ENABLE_EXPERIMENTAL

#include "gen.h"
#include "math/spline.h"

#include <cmath>
#include <glm/common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

std::vector<ObjectConfig>
genLightsForCoaster(const std::vector<glm::vec3> &coasterPoints, int count,
                    glm::vec3 color, float phase) {
  std::vector<ObjectConfig> coasterLights = {};

  for (int i = 0; i < count; i++) {
    float phase = (float)i / count + phase;
    coasterLights.push_back(
        {.light = {color, 1}, .parAnim = {coasterPoints, .5f, phase}});
  }

  return coasterLights;
}

std::vector<ObjectConfig> genRailsForCoaster(const std::vector<glm::vec3> &pts,
                                             int count) {
  std::vector<ObjectConfig> coasterRails = {};

  if (pts.size() < 2) {
    return {};
  }

  for (int i = 0; i < count; i++) {
    float phase = (float)i / count;
    int numSegments = (int)pts.size() - 1;
    float t = std::fmod(numSegments * phase, numSegments);

    if (t < 0.0f)
      t += (float)numSegments;

    auto [pos, _] = spline::calculatePosOnSpline(true, numSegments, pts, t);

    const glm::vec3 floorPos = {pos.x / 2, 0.f, pos.z / 2};
    const glm::vec3 topPos = {pos.x / 2, pos.y, pos.z / 2};

    coasterRails.push_back({
        .transform = {floorPos, {0, 0, 0}, {1, 1, 1}, -1},
        .sweep = {{floorPos, topPos}, 0.25, 10, 20, {0.7f, 0.7f, 0.7f}},
    });
  }

  return coasterRails;
}

std::vector<ObjectConfig> genTree(const glm::vec3 &position, float height,
                                  float baseWidth, int numLevels,
                                  int numPerLevel) {

  std::vector<ObjectConfig> resTree;
  glm::vec3 startPos = position;
  glm::vec3 endPos = position + glm::vec3{0, height, 0};

  resTree.clear();
  genTreeBranch(startPos, endPos, baseWidth, numLevels, numPerLevel, resTree);
  return resTree;
}

void genTreeBranch(const glm::vec3 &startPos, const glm::vec3 &endPos,
                   float width, int numLevels, int numPerLevel,
                   std::vector<ObjectConfig> &resTree) {

  if (numLevels <= 0) {
    return;
  }

  glm::vec3 lightBrown = {0.76, 0.60, 0.42};
  glm::vec3 darkBrown = {0.36, 0.22, 0.12};
  glm::vec3 color =
      glm::mix(lightBrown, darkBrown, float(rand() % 50 + 50) / 100.f);

  int circleSegments = std::max(3, 3 + numLevels * 3);

  if (resTree.size() == 0) {
    resTree.push_back({
        .sweep = {{startPos, endPos}, width, 10, circleSegments, color},
    });
    genTreeBranch(startPos, endPos, width * 0.5f, numLevels - 1, numPerLevel,
                  resTree);
    return;
  }

  for (int i = 0; i < numPerLevel; i++) {

    // leaf nodes
    if (numLevels == 1) {
      glm::vec3 lightGreen = {0.12, 0.42, 0.16};
      glm::vec3 darkGreen = {0.55, 0.80, 0.35};

      color = glm::mix(lightGreen, darkGreen, float(rand() % 100) / 100.f);

      glm::vec3 leafCenter = endPos;
      glm::vec3 leafTip = leafCenter + glm::normalize(startPos - endPos) * .01f;
      float leafRadius = width * 70.0f;

      resTree.push_back({
          .sweep =
              {{leafCenter, leafTip}, leafRadius, 10, circleSegments, color},
      });
      continue;
    }

    glm::vec3 branchStart = endPos;

    float branchLen = glm::distance(startPos, endPos) * 0.5f;

    glm::vec3 parentDir = glm::normalize(endPos - startPos);

    glm::vec3 randomUp1 = normalize(glm::vec3{
        float(rand() % 100), float(rand() % 100), float(rand() % 100)});
    glm::vec3 axis1 = glm::normalize(glm::cross(parentDir, randomUp1));

    if (glm::length(axis1) < 0.001f) {
      axis1 = glm::vec3{1, 0, 0};
    }

    float angle1 = glm::radians<float>((rand() % 180) - 90);
    glm::vec3 rotatedDir = glm::normalize(
        glm::rotate(glm::mat4{1.f}, angle1, axis1) * glm::vec4{parentDir, 0.f});

    glm::vec3 randomUp2 = normalize(glm::vec3{
        float(rand() % 100), float(rand() % 100), float(rand() % 100)});
    glm::vec3 axis2 = glm::normalize(glm::cross(rotatedDir, randomUp2));

    if (glm::length(axis2) < 0.001f) {
      axis2 = glm::vec3{0, 1, 0};
    }

    float angle2 = glm::radians<float>((rand() % 180) - 90);
    glm::vec3 direction =
        glm::normalize(glm::rotate(glm::mat4{1.f}, angle2, axis2) *
                       glm::vec4{rotatedDir, 0.f});

    if (direction.y < 0) {
      direction = -direction;
    }

    glm::vec3 branchEnd = branchStart + direction * branchLen;

    resTree.push_back({
        .sweep = {{branchStart, branchEnd}, width, 10, circleSegments, color},
    });

    genTreeBranch(branchStart, branchEnd, width * .3, numLevels - 1,
                  numPerLevel, resTree);
  }
}

void fps(float deltaTime) {
  static float fpsTimer = 0;
  static float totalTime = 0;
  static int totalFrames = 0;
  fpsTimer += deltaTime;
  totalTime += deltaTime;
  ++totalFrames;

  if (fpsTimer >= 1) {
    std::cerr << "\rfps: " << 1 / deltaTime
              << " avg fps: " << totalFrames / totalTime << std::flush;
    fpsTimer = 0;
  }
}
