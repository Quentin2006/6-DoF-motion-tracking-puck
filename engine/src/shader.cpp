#include "shader.h"
#include "../include/glad/glad.h"
#include <GL/glext.h>
#include <fstream>
#include <iostream>
#include <sstream>

Shader::Shader() {}
Shader::~Shader() { glDeleteProgram(currentShaderProgram); }

ShaderResult Shader::loadShaders() {
  vertexSource = readFile("src/shaders/shader.vert");
  if (vertexSource.empty()) {
    std::cerr << "Failed to read vertex shader file: src/shaders/shader.vert" << std::endl;
    return ShaderResult::FileNotFound;
  }
  
  fragmentSource = readFile("src/shaders/shader.frag");
  if (fragmentSource.empty()) {
    std::cerr << "Failed to read fragment shader file: src/shaders/shader.frag" << std::endl;
    return ShaderResult::FileNotFound;
  }

  glDeleteProgram(currentShaderProgram);

  currentShaderProgram =
      createShaderProgram(vertexSource.c_str(), fragmentSource.c_str());

  if (currentShaderProgram == 0) {
    return ShaderResult::LinkingFailed;
  }

  glUseProgram(currentShaderProgram);
  return ShaderResult::Success;
}

std::string Shader::readFile(const char *path) {
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

unsigned int Shader::createShaderProgram(const char *vertexSource,
                                         const char *fragmentSource) {
  unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
  if (vertexShader == 0) {
    return 0;
  }
  
  unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
  if (fragmentShader == 0) {
    glDeleteShader(vertexShader);
    return 0;
  }

  unsigned int shaderProgram = glCreateProgram();
  glAttachShader(shaderProgram, vertexShader);
  glAttachShader(shaderProgram, fragmentShader);
  glLinkProgram(shaderProgram);

  int success;
  char infoLog[512];
  glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
    std::cout << "Shader program linking failed: " << infoLog << std::endl;
    return 0;
  }

  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  return shaderProgram;
}

unsigned int Shader::compileShader(unsigned int type, const char *source) {
  unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, NULL);
  glCompileShader(shader);

  int success;
  char infoLog[512];
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shader, 512, NULL, infoLog);
    std::cout << "Shader compilation failed: " << infoLog << std::endl;
    return 0;
  }
  return shader;
}

int Shader::addUniform(const std::string &name) {
  uniformLocations[name] =
      glGetUniformLocation(currentShaderProgram, name.c_str());
  return uniformLocations[name];
}

unsigned int Shader::getUniformLocation(const std::string &name) {
  return uniformLocations[name];
}

void Shader::bindUniformBlock(const std::string &name,
                              unsigned int bindingPoint) {
  unsigned int blockIndex =
      glGetUniformBlockIndex(currentShaderProgram, name.c_str());
  if (blockIndex != GL_INVALID_INDEX) {
    glUniformBlockBinding(currentShaderProgram, blockIndex, bindingPoint);
  } else {
    std::cerr << "Warning: Uniform block '" << name << "' not found in shader"
              << std::endl;
  }
}
