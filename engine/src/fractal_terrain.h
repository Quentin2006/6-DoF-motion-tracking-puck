#include "vertexBuffer.h"
#include <vector>

class FractalTerrain {
public:
  // NOTE: a,b,c must have ccw winding
  std::vector<Vertex> generateTerrain(int subDivCount, const Vertex &v1,
                                      const Vertex &v2, const Vertex &v3);

private:
  void generateSubDivEdges(int subDivCount);
  std::vector<std::tuple<Vertex, Vertex, Vertex>> faces;
};
