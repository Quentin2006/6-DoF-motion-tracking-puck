#pragma once

#include "../include/glad/glad.h"
#include <map>
#include <string>

enum class ShaderResult {
  Success,
  FileNotFound,
  CompilationFailed,
  LinkingFailed
};

class Shader {
public:
  Shader();
  ~Shader();
  ShaderResult loadShaders();
  void use() { glUseProgram(currentShaderProgram); }
  unsigned int getShaderProgram() { return currentShaderProgram; }
  int addUniform(const std::string &name);
  unsigned int getUniformLocation(const std::string &name);
  void bindUniformBlock(const std::string &name, unsigned int bindingPoint);

private:
  std::string readFile(const char *path);
  unsigned int createShaderProgram(const char *vertexSource,
                                   const char *fragmentSource);
  unsigned int compileShader(unsigned int type, const char *source);

  std::string vertexSource;
  std::string fragmentSource;
  unsigned int currentShaderProgram;
  std::map<std::string, unsigned int> uniformLocations;
};
