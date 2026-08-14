#pragma once
#include "../../include/glad/glad.h"
#include "../math/spline.h"
#include "../mesh.h"
#include "registry.h"
#include <cmath>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

inline void updateTransforms(Registry &reg) {
  static bool init = false;

  for (auto &t : reg.getTransforms()) {
    t.matrix = glm::mat4(1.0f);

    if (!init) {
      t.position = t.offset;
    }

    t.matrix = glm::translate(t.matrix, t.position);
    t.matrix =
        glm::rotate(t.matrix, glm::radians(t.rotation.x), glm::vec3(1, 0, 0));
    t.matrix =
        glm::rotate(t.matrix, glm::radians(t.rotation.y), glm::vec3(0, 1, 0));
    t.matrix =
        glm::rotate(t.matrix, glm::radians(t.rotation.z), glm::vec3(0, 0, 1));
    t.matrix = glm::scale(t.matrix, t.scale);

    // NOTE: parrents must ALWAYS come before children when loading
    if (t.parentId >= 0) {
      auto &parentTransform = reg.getTransform(t.parentId);
      t.matrix = parentTransform.matrix * t.matrix;
    }
  }
  init = true;
}

inline void updateCamera(Registry &reg) {
  for (const auto &id : reg.getCameraEntityIds()) {
    // get the cams transform
    auto &cam = reg.getCamera(id);
    auto &t = reg.getTransform(id);

    cam->setPos(glm::vec3(t.matrix[3]) + t.offset);
  }
}

inline void updateAnimations(Registry &reg, float deltaTime) {
  static float totalTime = 0;
  totalTime += deltaTime;
  // UPDATE SINE ANIMATORS
  for (size_t id : reg.getSineAnimatorEntityIds()) {
    auto &sineOpt = reg.getSineAnimator(id);
    auto &anim = sineOpt.value();
    auto &t = reg.getTransform(id);
    float offset =
        sin(totalTime * anim.frequency + anim.phase) * anim.amplitude;
    t.position = anim.axis * offset + t.offset;
  }

  // UPDATE ROTATION ANIMATORS
  for (size_t id : reg.getRotationAnimatorEntityIds()) {
    auto &rotOpt = reg.getRotationAnimator(id);
    auto &anim = rotOpt.value();
    auto &t = reg.getTransform(id);
    t.rotation += anim.axis * anim.rpm * 6.0f * deltaTime;
  }

  // UPDATE PARAMETRIC ANIMATORS
  for (size_t id : reg.getParametricAnimatorEntityIds()) {
    auto &parametricOpt = reg.getParametricAnimator(id);
    auto &anim = parametricOpt.value();
    const auto &pts = anim.points;

    // Need at least 2 points to define a path
    if (pts.size() >= 2) {
      // Check if path is closed (first and last points are nearly identical)
      bool cyclic = glm::length(pts.front() - pts.back()) < 0.001f;

      // Number of segments is points minus one
      int numSegments = (int)pts.size() - 1;

      // Calculate normalized position along the entire path
      // totalTime * speed gives us how far along the path we are
      // phase offsets the starting position on the path
      float globalT =
          std::fmod(totalTime * anim.speed + numSegments * parametricOpt->phase,
                    numSegments);
      // Handle negative time by wrapping to positive range
      if (globalT < 0.0f)
        globalT += (float)numSegments;

      auto [pos, tangent] =
          spline::calculatePosOnSpline(cyclic, numSegments, pts, globalT);

      // Interpolate position along the spline
      auto &t = reg.getTransform(id);

      t.position = pos + t.offset;

      // Convert tangent vector to yaw/pitch rotation angles
      // yaw = horizontal direction, pitch = vertical angle
      float yaw = glm::degrees(std::atan2(-tangent.z, tangent.x));
      float pitch = glm::degrees(std::asin(glm::clamp(tangent.y, -1.0f, 1.0f)));

      // Apply rotation so entity faces along the path (add to initial rotation)
      t.rotation = anim.initialRotation + glm::vec3{pitch, yaw, 0};
    }
  }
}

inline void renderAll(Registry &reg, GLuint modelUniform, GLuint imageTexUnit,
                      GLint diffuseTexUnit, GLint specularTexUnit,
                      GLuint shininessLoc) {

  for (size_t id : reg.getMeshEntityIds()) {
    auto &meshComp = reg.getMesh(id);
    glUniformMatrix4fv(modelUniform, 1, GL_FALSE,
                       glm::value_ptr(reg.getTransform(id).matrix));

    // Set texture units for specular lighting
    glUniform1i(imageTexUnit, 0);
    glUniform1i(diffuseTexUnit, 2);
    glUniform1i(specularTexUnit, 1);
    glUniform1f(shininessLoc, meshComp->mesh->getShininess());

    meshComp->mesh->draw();
  }
}
