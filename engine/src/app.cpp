#include "app.h"
#include "callbacks.h"
#include "camera.h"
#include "ecs/components.h"
#include "ecs/registry.h"
#include "ecs/systems.h"
#include "gen.h"
#include "objectBuilder.h"
#include "resource_manager.h"
#include "uniformBuffer.h"
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <glm/common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

void fps(float deltaTime);

App::App(int width, int height, const std::string &title)
    : window(width, height, title), shader(), frameCounter(0) {

  if (!gladLoadGLLoader((void *(*)(const char *))glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    exit(-1);
  }
  ShaderResult shaderLoadResult = shader.loadShaders();
  if (shaderLoadResult != ShaderResult::Success) {
    std::cerr << "Failed to load shaders: "
              << static_cast<int>(shaderLoadResult) << std::endl;
    exit(1);
  }

  cameras.push_back(
      std::shared_ptr<Camera>(new Camera(45.f, width, height, 0.1f, 10000.f)));
  cameraIndex = 0;

  glViewport(0, 0, window.getWidth(), window.getHeight());
  glClearColor(0.2f, 0.2f, 0.5f, 1.f);
  glEnable(GL_DEPTH_TEST);

  // glEnable(GL_CULL_FACE);
  // glCullFace(GL_BACK);
  // glFrontFace(GL_CCW);

  glfwSetWindowUserPointer(window.getGLFWwindow(), this);
  glfwSetKeyCallback(window.getGLFWwindow(), key_callback);
  glfwSetFramebufferSizeCallback(window.getGLFWwindow(),
                                 framebuffer_size_callback);
}

void App::loadObjectFromConfig(const ObjectConfig &cfg) {
  int obj = registry.createEntity();

  if (!cfg.mesh.path.empty()) {
    auto mesh = resourceManager.loadMesh(cfg.mesh.path, cfg.mesh.name,
                                         cfg.mesh.texturePath);
    registry.setMesh(obj, std::optional<MeshComp>({mesh}));

  } else if (!cfg.sweep.points.empty()) {
    auto mesh = resourceManager.loadMesh(
        cfg.sweep.points, cfg.sweep.pathSegments, cfg.sweep.circleSegments,
        cfg.sweep.radius, cfg.sweep.color);
    registry.setMesh(obj, std::optional<MeshComp>({mesh}));
  }

  registry.setTransform(obj, cfg.transform);

  if (cfg.light.intensity != 0.f) {
    registry.setLight(obj, std::optional<Light>(cfg.light));
  }
  if (cfg.sineAnim.amplitude != 0.f) {
    registry.setSineAnimator(obj, std::optional<SineAnimator>(cfg.sineAnim));
  }
  if (cfg.rotationAnim.rpm != 0.f) {
    registry.setRotationAnimator(
        obj, std::optional<RotationAnimator>(cfg.rotationAnim));
  }
  if (!cfg.parAnim.points.empty()) {
    ParametricAnimator pa = cfg.parAnim;
    pa.initialRotation = cfg.transform.rotation;
    registry.setParametricAnimator(obj, std::optional<ParametricAnimator>(pa));
  }
  if (cfg.isCam) {
    ++cameraIndex;
    registry.setCamera(obj, std::shared_ptr<Camera>(cameras[cameraIndex]));
  }
}

struct DeviceData {
  glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};  // wxyz from DMP
  glm::vec3 accel{0.0f};                            // m/s², world frame
  float temperature = 0;                            // °C
};

void getDeviceData(DeviceData &out) {
  static int fd = -1;
  if (fd < 0) {
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      std::cerr << "getDeviceData: failed to open /dev/ttyACM0: "
                << strerror(errno) << std::endl;
      return;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      return;
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    // cfmakeraw(&tty);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      return;
    }
    tcflush(fd, TCIFLUSH);
  }

  static std::string buffer;
  static DeviceData last{};

  char chunk[128];
  ssize_t n = read(fd, chunk, sizeof(chunk));
  if (n > 0)
    buffer.append(chunk, static_cast<size_t>(n));

  size_t pos;
  while ((pos = buffer.find('\n')) != std::string::npos) {
    std::istringstream line{buffer.substr(0, pos)};
    buffer.erase(0, pos + 1);

    glm::quat quat;
    glm::vec3 accel;
    float temp;
    if (line >> quat.w >> quat.x >> quat.y >> quat.z >> accel.x >>
        accel.y >> accel.z >> temp) {
      last.orientation = glm::normalize(quat);
      last.accel = accel;
      last.temperature = temp;
    }
  }

  if (buffer.size() > 4096)
    buffer.clear();

  out = last;
}

void App::run() {
  shader.addUniform("model");
  shader.addUniform("diffuseTexture");
  shader.addUniform("specularTexture");
  shader.addUniform("imageTexture");
  shader.addUniform("shininess");
  shader.addUniform("cameraPos");

  shader.bindUniformBlock("LightBlock", 0);
  shader.bindUniformBlock("CameraBlock", 1);

  lightUniformBuffer.bindToPoint(0);
  cameraUniformBuffer.bindToPoint(1);

  std::vector<ObjectConfig> objectConfigs = {
      // Cube orbiting the coaster cart
      createObject()
          .withMesh("/home/qscheetz/Documents/6-DoF-motion-tracking-puck/"
                    "engine/assets/",
                    "cube.obj")
          .build(),
  };
  std::cerr << "Loading meshes: " << objectConfigs.size() << std::endl;
  for (const auto &cfg : objectConfigs) {
    loadObjectFromConfig(cfg);
  }

  loadObjectFromConfig(createObject()
                           .withLight({1, 0, 0}, .5f)
                           .withTransform({0, 50, 0}, {0, 0, 0}, {1, 1, 1})
                           .build());

  auto prevTime = std::chrono::steady_clock::now();
  float totalTime = 0;

  cubeSpawn = registry.getTransform(0).position;

  while (!window.shouldClose()) {
    auto currentTime = std::chrono::steady_clock::now();
    auto deltaTime =
        std::chrono::duration<float>(currentTime - prevTime).count();
    prevTime = currentTime;
    totalTime += deltaTime;
    fps(deltaTime);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    moveCamera(deltaTime);

    // update the cube
    DeviceData deviceData;
    getDeviceData(deviceData);

    auto &cube = registry.getTransform(0);
    cube.rotation = glm::degrees(glm::eulerAngles(deviceData.orientation));

    // velocity integration: position accumulates, bounds clamp (no damping)
    glm::vec3 accel = deviceData.accel;
    if (glm::length(accel) < ACCEL_DEADBAND)
      accel = glm::vec3(0.0f);  // deadband: kill rest jitter crawl
    cubeVelocity += accel * ACCEL_VELOCITY_GAIN * deltaTime;
    if (glm::length(cubeVelocity) > MAX_CUBE_SPEED)
      cubeVelocity = glm::normalize(cubeVelocity) * MAX_CUBE_SPEED;
    cube.position += cubeVelocity * deltaTime;
    if (glm::length(cube.position) > CUBE_BOUND) {
      cube.position = glm::normalize(cube.position) * CUBE_BOUND;
      cubeVelocity = glm::vec3(0.0f);
    }
    if (input.r && !input.r_pressed) {
      input.r_pressed = true;
      cube.position = cubeSpawn;
      cubeVelocity = glm::vec3(0.0f);
    } else if (!input.r) {
      input.r_pressed = false;
    }

    updateTransforms(registry);
    updateCamera(registry);

    CameraBlock cameraBlock{
        cameras[cameraIndex]->getViewMatrix(),
        cameras[cameraIndex]->getProjectionMatrix(),
    };

    cameraUniformBuffer.bindToPoint(1);
    cameraUniformBuffer.uploadData(&cameraBlock, sizeof(CameraBlock));

    glUniform3fv(shader.getUniformLocation("cameraPos"), 1,
                 glm::value_ptr(cameras[cameraIndex]->getPosition()));

    shader.use();
    renderAll(registry, shader.getUniformLocation("model"),
              shader.getUniformLocation("imageTexture"),
              shader.getUniformLocation("diffuseTexture"),
              shader.getUniformLocation("specularTexture"),
              shader.getUniformLocation("shininess"));
    window.swapBuffers();
  }

  glfwTerminate();
}

void App::moveCamera(float deltaTime) {
  float moveAmount = MOVEMENT_SPEED * deltaTime;
  float rotAmount = ROTATION_SPEED * deltaTime;

  if (input.w)
    cameras[cameraIndex]->moveForward(moveAmount);
  if (input.s)
    cameras[cameraIndex]->moveForward(-moveAmount);
  if (input.a)
    cameras[cameraIndex]->moveRight(-moveAmount);
  if (input.d)
    cameras[cameraIndex]->moveRight(moveAmount);
  if (input.q)
    cameras[cameraIndex]->moveUp(moveAmount);
  if (input.e)
    cameras[cameraIndex]->moveUp(-moveAmount);
  if (input.up)
    cameras[cameraIndex]->rotatePitch(rotAmount);
  if (input.down)
    cameras[cameraIndex]->rotatePitch(-rotAmount);
  if (input.left)
    cameras[cameraIndex]->rotateYaw(-rotAmount);
  if (input.right)
    cameras[cameraIndex]->rotateYaw(rotAmount);
  if (input.c && !input.c_pressed) {
    cameraIndex = (cameraIndex + 1) % cameras.size();
    input.c_pressed = true;
  } else if (!input.c) {
    input.c_pressed = false;
  }

  //
  if (input.subdivUp && !input.subdivUp_pressed) {
    if (subdivLevel < 11) {
      subdivLevel++;
      regenerateTerrain();
    }
    input.subdivUp_pressed = true;
  } else if (!input.subdivUp) {
    input.subdivUp_pressed = false;
  }

  if (input.subdivDown && !input.subdivDown_pressed) {
    if (subdivLevel > 0) {
      subdivLevel--;
      regenerateTerrain();
    }
    input.subdivDown_pressed = true;
  } else if (!input.subdivDown) {
    input.subdivDown_pressed = false;
  }
}

void App::regenerateTerrain() {
  FractalTerrain fractalTerrain;
  auto verts = fractalTerrain.generateTerrain(subdivLevel, terrainV1, terrainV2,
                                              terrainV3);
  int count = terrainMesh->loadVertices(verts);
  std::cerr << "Subdivision level: " << subdivLevel << " (" << count
            << " vertices)" << std::endl;
}
