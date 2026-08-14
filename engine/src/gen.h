#pragma once

#include "app.h"
#include <vector>

std::vector<ObjectConfig>
genLightsForCoaster(const std::vector<glm::vec3> &coasterPoints, int count,
                    glm::vec3 color, float phase = 0.f);

std::vector<ObjectConfig> genRailsForCoaster(const std::vector<glm::vec3> &pts,
                                             int count);

std::vector<ObjectConfig> genTree(const glm::vec3 &position, float height,
                                  float baseWidth, int numLevels,
                                  int numPerLevel);

void genTreeBranch(const glm::vec3 &startPos, const glm::vec3 &endPos,
                   float width, int numLevels, int numPerLevel,
                   std::vector<ObjectConfig> &resTree);

void fps(float deltaTime);
