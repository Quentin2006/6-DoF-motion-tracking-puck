# OpenGL Template

OpenGL 3.3 Core template using C++, GLFW, and GLAD.

## Prerequisites

Install GLFW

## Building

```bash
make
```

## Running

```bash
make run
```

## Cleaning

```bash
make clean
```

## Project Structure

- `src/main.cpp` - Application entry point
- `src/app.cpp` / `src/app.h` - Main application orchestrator
  - Owns: Window, Shader, Camera, Buffer, InputState
  - Manages render loop: input → update uniforms → draw → swap
- `src/window.cpp` / `src/window.h` - GLFW window abstraction
  - Window creation, context management, event callbacks
- `src/shader.cpp` / `src/shader.h` - GLSL shader management
  - Compiles/loads shader programs from source files
  - Provides uniform location queries
- `src/buffer.cpp` / `src/buffer.h` - OpenGL buffer utilities
  - VAO/VBO creation and vertex data loading
- `src/camera.cpp` / `src/camera.h` - 3D camera system
  - Position, rotation (yaw/pitch), view/projection matrices
- `src/shaders/` - GLSL shader files
  - `shader.vert` - Vertex shader
  - `shader.frag` - Fragment shader
- `src/glad.c` - GLAD loader (OpenGL 3.3 Core)
- `include/glad/glad.h` - GLAD header
- `include/KHR/khrplatform.h` - KHR platform header
- `Makefile` - Build configuration
- `bin/` - Compiled executable output
- `obj/` - Object files (build artifacts)
