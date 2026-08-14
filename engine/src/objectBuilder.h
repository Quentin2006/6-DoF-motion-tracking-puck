#pragma once

#include "camera.h"
#include "ecs/components.h"
#include <glm/vec3.hpp>
#include <optional>
#include <string>
#include <vector>

struct ObjectConfig {
  MeshComp meshObj{nullptr};
  MeshPath mesh{"", "", ""};
  Transform transform{
      {0, 0, 0},
      {0, 0, 0},
      {1, 1, 1},
      -1,
  };
  Light light{{0, 0, 0}, 0.f};
  SineAnimator sineAnim{{0, 0, 0}, 0.f, 0.f, 0.f};
  RotationAnimator rotationAnim{{0, 0, 0}, 0.f};
  Sweep sweep{{}, 0, 0, 0, {1, 1, 1}};
  ParametricAnimator parAnim{{}, 0.f, 0.f};
  bool isCam{false};
};

class ObjectBuilder {
public:
  // will try at auto load textures, however if its unable to locate the
  // textrueres, you can manualy set the search path
  ObjectBuilder &withMesh(const std::string &path, const std::string &name,
                          const std::string &texturePath = "");
  ObjectBuilder &withTransform(const glm::vec3 &offset, const glm::vec3 &rot,
                               const glm::vec3 &scale, int parentId = -1);
  ObjectBuilder &withSineAnimator(const glm::vec3 &axis, float amp, float freq,
                                  float phase);
  ObjectBuilder &withRotationAnimator(const glm::vec3 &axis, float rpm);
  ObjectBuilder &withParametricAnimator(const std::vector<glm::vec3> &points,
                                        float speed, float phase);
  ObjectBuilder &withLight(const glm::vec3 &color, float intensity);
  ObjectBuilder &withSweep(const Sweep &sweep);
  ObjectBuilder &withCamera(std::vector<std::shared_ptr<Camera>> &cameras,
                            float fov, int width, int height);

  ObjectConfig build();

private:
  ObjectConfig config;
};

ObjectBuilder createObject();
