#include "app.h"
#include "ecs/registry.h"

Registry g_registry;

const unsigned int WIDTH = 1000;
const unsigned int HEIGHT = 1000;

int main() {
  App app(WIDTH, HEIGHT, "OpenGL Template");

  app.run();

  return 0;
}
