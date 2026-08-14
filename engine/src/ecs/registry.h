#pragma once
#include "../camera.h"
#include "components.h"
#include <algorithm>
#include <complex>
#include <memory>
#include <unordered_set>
#include <vector>

class Registry {
public:
  int createEntity() {
    int id = transforms.size();
    transforms.push_back({});
    meshes.push_back({});
    lights.push_back({});
    sineAnimators.push_back({});
    rotationAnimators.push_back({});
    parametricAnimators.push_back({});
    cameras.push_back({});
    return id;
  }

  // FIXME: change this so it actualy frees the memeory
  void destroyEntity(int entity) {
    transforms[entity] = {};
    meshes[entity].reset();
    lights[entity].reset();
    sineAnimators[entity].reset();
    rotationAnimators[entity].reset();
    parametricAnimators[entity].reset();
    cameras[entity].reset();
  }

  void setTransform(int entity, const Transform &t) { transforms[entity] = t; }

  void setMesh(int entity, const std::optional<MeshComp> &m) {
    meshes[entity] = m;
    updateEntitySet(entity, meshEntityIds, m.has_value());
  }

  void setSineAnimator(int entity, const std::optional<SineAnimator> &a) {
    sineAnimators[entity] = a;
    updateEntitySet(entity, sineAnimatorEntityIds, a.has_value());
  }
  void setRotationAnimator(int entity,
                           const std::optional<RotationAnimator> &a) {
    rotationAnimators[entity] = a;
    updateEntitySet(entity, rotationAnimatorEntityIds, a.has_value());
  }
  void setParametricAnimator(int entity,
                             const std::optional<ParametricAnimator> &a) {
    parametricAnimators[entity] = a;
    updateEntitySet(entity, parametricAnimatorEntityIds, a.has_value());
  }
  void setLight(int entity, const std::optional<Light> &l) {
    lights[entity] = l;
    updateEntitySet(entity, lightEntityIds, l.has_value());
  }
  void setCamera(int entity, const std::shared_ptr<Camera> &c) {
    cameras[entity] = c;

    updateEntitySet(entity, cameraEntityIds, c != nullptr);
  }

  Transform &getTransform(int entity) { return transforms[entity]; }

  const std::optional<MeshComp> &getMesh(int entity) const {
    return meshes[entity];
  }

  const std::optional<SineAnimator> &getSineAnimator(int entity) const {
    return sineAnimators[entity];
  }
  const std::optional<RotationAnimator> &getRotationAnimator(int entity) const {
    return rotationAnimators[entity];
  }
  const std::optional<ParametricAnimator> &
  getParametricAnimator(int entity) const {
    return parametricAnimators[entity];
  }
  const std::optional<Light> &getLight(int entity) const {
    return lights[entity];
  }
  const std::shared_ptr<Camera> &getCamera(int entity) const {
    return cameras[entity];
  }

  std::vector<Transform> &getTransforms() { return transforms; }

  const std::unordered_set<size_t> &getMeshEntityIds() const {
    return meshEntityIds;
  }
  const std::unordered_set<size_t> &getLightEntityIds() const {
    return lightEntityIds;
  }
  const std::unordered_set<size_t> &getSineAnimatorEntityIds() const {
    return sineAnimatorEntityIds;
  }
  const std::unordered_set<size_t> &getRotationAnimatorEntityIds() const {
    return rotationAnimatorEntityIds;
  }
  const std::unordered_set<size_t> &getParametricAnimatorEntityIds() const {
    return parametricAnimatorEntityIds;
  }
  const std::unordered_set<size_t> &getCameraEntityIds() const {
    return cameraEntityIds;
  }

  size_t entityCount() const { return transforms.size(); }

private:
  std::vector<Transform> transforms;

  std::vector<std::optional<MeshComp>> meshes;
  std::unordered_set<size_t> meshEntityIds;

  std::vector<std::optional<Light>> lights;
  std::unordered_set<size_t> lightEntityIds;

  std::vector<std::optional<SineAnimator>> sineAnimators;
  std::unordered_set<size_t> sineAnimatorEntityIds;

  std::vector<std::optional<RotationAnimator>> rotationAnimators;
  std::unordered_set<size_t> rotationAnimatorEntityIds;

  std::vector<std::optional<ParametricAnimator>> parametricAnimators;
  std::unordered_set<size_t> parametricAnimatorEntityIds;

  std::vector<std::shared_ptr<Camera>> cameras;
  std::unordered_set<size_t> cameraEntityIds;

  /**
   * @brief will update the set if the entity is in the set and the component
   * is present, or will remove the entity from the set if the component is
   * absent
   *
   * @param entity the entity id
   * @param s The set to update
   * @param hasValue true if the component is present, false otherwise
   */
  void updateEntitySet(int entity, std::unordered_set<size_t> &s,
                       bool hasValue) {

    bool isInSet = s.find(entity) != s.end();

    if (!isInSet && hasValue) {
      s.insert(entity);
    } else if (isInSet && !hasValue) {
      s.erase(entity);
    }
  }
};
